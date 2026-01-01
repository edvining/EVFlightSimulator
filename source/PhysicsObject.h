#pragma once
#include "triple.h"
#include "GravitySimulator.h"
#include <cmath>

class PhysicsObject
{
public:
	const double c = 299792458.0;
	triple p, v, a;
	triple p1, p2, p3, p4, p5, p6;
	triple v1, v2, v3, v4;
	triple a1, a2, a3, a4, a5, a6;
	triple ExternalForces;
	std::vector<triple> pastPositions;
	std::vector<triple> pastPositionstemp;
	PhysicsObject* referenceObject = nullptr;
	const std::string name;
	double GPE;
	double m;
	double mu;
	std::mutex storingMutex;
	color colour = { 1, 1, 1 };
	float outputPosition[3];
	float radius;
	float swartzchildRadius;
	int index;
	int referenceObjectIndex = 0;
	bool firstIter = true;
	bool contributesToGravity = true;
	bool request1xTimeWarp = false;
	bool requestedAlready = false;
	bool resumeTimeWarp = false;
	/// <summary>
	/// Mass, radius, position, velocity
	/// </summary>
	/// <param name="m"></param>
	/// <param name="radius"></param>
	/// <param name="p"></param>
	/// <param name="v"></param>
	/// <param name="a"></param>
	PhysicsObject(const char* name, float m, float radius, triple p, triple v, bool contributesToGravSim = true, PhysicsObject* refObj = nullptr) : name(name), p(p), v(v), a(triple::zero()), m(m), mu(m * 6.67e-11), radius(radius), GPE(0), outputPosition{ (float)p.x, (float)p.y, (float)p.z }, contributesToGravity(contributesToGravSim), referenceObject(refObj) {}
	PhysicsObject() : name("Empty"), p(triple::zero()), v(triple::zero()), a(triple::zero()), m(10), mu(m * 6.67e-11), radius(1), GPE(0), outputPosition{ (float)p.x, (float)p.y, (float)p.z } {}
	triple GetPosition()
	{
		return p;
	}
	float* GetPositionF3()
	{
		float vectorF3[3] = { (float)p.x / 1000, (float)p.y / 1000, (float)p.z / 1000 };
		return vectorF3;
	}
	triple GetVelocity()
	{
		return v;
	}
	triple GetAcceleration()
	{
		return a;
	}
	void EulerStep(double dt)
	{
		this->p = this->p + this->v * dt;
		this->outputPosition[0] = (float)this->p.x;
		this->outputPosition[1] = (float)this->p.y;
		this->outputPosition[2] = (float)this->p.z;
		this->v = this->v + this->a * dt;
		if (this->v.magnitude() > c) {
			this->v = this->v.normalized() * c;
		}
		ClearForce();
	}
	void VerletStep(double dt)
	{
		this->p += this->v * dt + this->a * (dt * dt * 0.5);
		this->outputPosition[0] = (float)this->p.x;
		this->outputPosition[1] = (float)this->p.y;
		this->outputPosition[2] = (float)this->p.z;
		this->v += this->a * dt;
		if (this->v.magnitude() > c) {
			this->v = this->v.normalized() * c;
		}
		ClearForce();
	}
	void EulerStep2(double dt)
	{
		this->v = this->v + this->a * dt;
		this->p = this->p + this->v * dt;
		this->outputPosition[0] = (float)this->p.x;
		this->outputPosition[1] = (float)this->p.y;
		this->outputPosition[2] = (float)this->p.z;
		if (this->v.magnitude() > c) {
			this->v = this->v.normalized() * c;
		}
		ClearForce();
	}
	void RK4Step1(double dt)
	{
		this->p1 = this->p;
		this->p2 = this->p1 + this->v * dt + 0.5 * this->a1 * dt * dt;
	}

	void RK4Step2(double dt)
	{
		double dtOver2 = dt * 0.5;
		this->p3 = this->p1 + this->v * dtOver2 + 0.5 * this->a2 * dtOver2 * dtOver2;
	}

	void RK4Step3(double dt)
	{
		double dtOver2 = dt * 0.5;
		this->p4 = this->p1 + this->v * dtOver2 + 0.5 * this->a3 * dtOver2 * dtOver2;
	}

	void RK4Step4(double dt)
	{
		triple a1 = this->a1;
		triple a2 = this->a2;
		triple a3 = this->a3;
		triple a4 = this->a4;
		this->a = (a1 + (2 * a4) + (2 * a3) + a2) / 6;
		this->p = this->p + this->v * dt + 0.5 * this->a * dt * dt;
		this->v = this->v + this->a * dt;
		if (this->v.magnitude() > c) {
			this->v = this->v.normalized() * c;
		}
	}

	void AddForce(triple F)
	{
		this->ExternalForces += F;
	}

	void ClearForce()
	{
		this->a = triple(0, 0, 0);
		this->a1 = triple(0, 0, 0);
		this->a2 = triple(0, 0, 0);
		this->a3 = triple(0, 0, 0);
		this->a4 = triple(0, 0, 0);
	}

	void ClearExternalForce()
	{
		this->ExternalForces = triple(0, 0, 0);
	}

	void StoreCurrentPosition(int numberOfStoredPositions)
	{
		pastPositions.push_back(this->p);
		while (pastPositions.size() > numberOfStoredPositions) {
			pastPositions.erase(pastPositions.begin());
		}
	}

	void SetReferenceObject(std::vector<PhysicsObject*>& allobjects) {
		referenceObjectIndex = find(allobjects.begin(), allobjects.end(), referenceObject) - allobjects.begin();
		if (referenceObject == nullptr) {
			referenceObjectIndex = 0;
		}
		referenceObject = allobjects[referenceObjectIndex];
	}

	void SetOrbitAround(PhysicsObject* refObj, double SMA, double ECC, double AOP, double LAN, double INC, double MA) {
		double vt = 2 * atan2(sqrt(1 + ECC) * sin((MA) * 0.5f), sqrt(1 - ECC) * cos(MA * 0.5f));
		double rc = SMA * (1 - ECC * cos(MA));
		triple ot{ rc * cos(vt), rc * sin(vt), 0 };
		double _const = sqrt(refObj->mu * SMA) / rc;
		triple odot{ _const * (-sin(MA)), _const * (sqrt(1 - ECC * ECC) * cos(MA)), 0 };
		triple rt{
			ot.x * (cos(AOP) * cos(LAN) - sin(AOP) * cos(INC) * sin(LAN)) - ot.y * (sin(AOP) * cos(LAN) + cos(AOP) * cos(INC) * sin(LAN)),
			ot.x * (cos(AOP) * sin(LAN) + sin(AOP) * cos(INC) * cos(LAN)) + ot.y * (cos(AOP) * cos(INC) * cos(LAN) - sin(AOP) * sin(LAN)),
			ot.x * (sin(AOP) * sin(INC)) - ot.y * (cos(AOP) * sin(INC))
		};
		triple rdot{
			odot.x * (cos(AOP) * cos(LAN) - sin(AOP) * cos(INC) * sin(LAN)) - odot.y * (sin(AOP) * cos(LAN) + cos(AOP) * cos(INC) * sin(LAN)),
			odot.x * (cos(AOP) * sin(LAN) + sin(AOP) * cos(INC) * cos(LAN)) + odot.y * (cos(AOP) * cos(INC) * cos(LAN) - sin(AOP) * sin(LAN)),
			odot.x * (sin(AOP) * sin(INC)) - odot.y * (cos(AOP) * sin(INC))
		};
		this->p = refObj->p + rt;
		this->v = refObj->v + rdot;
	}

	virtual triple GetExternalForces() const {
		return ExternalForces;
	}

	virtual void PreForceUpdate(double simTime, double dt, int RKStep) {}

};

struct Burn {
	triple direction;
	double thrust, startTime, durationInSeconds;
	Burn(triple i_direction, double i_thrust, double i_startTime, double i_durationInSeconds) : direction(i_direction), thrust(i_thrust), startTime(i_startTime), durationInSeconds(i_durationInSeconds) {};
};

enum AutopilotMode {
	IDLE,
	AUTO_ORBIT,
	TRANSIT
};

class Spaceship : public PhysicsObject {
public:
	PhysicsObject* targetObject = nullptr;
	AutopilotMode autopilot = AutopilotMode::IDLE;
	double propellantAmount = 0.0;
	triple currentThrustVector = triple::zero();
	double currentThrustAmount = 0.0;
	std::vector<Burn> listOfBurns;
	Spaceship(const char* name, float m, float radius, triple p, triple v, bool contributesToGravSim = true, PhysicsObject* refObj = nullptr) : PhysicsObject(name, m, radius, p, v, contributesToGravSim, refObj) {}
	Spaceship() : PhysicsObject("Empty", 10, 1, triple::zero(), triple::zero(), false, nullptr){}

	void PreForceUpdate(double simTime, double dt, int RKStep) override
	{
		currentThrustAmount = 0.0;
		currentThrustVector = triple::zero();

		if (autopilot == AutopilotMode::AUTO_ORBIT && targetObject) {
			UpdateAutoOrbit(simTime, dt, RKStep);
		}

		for (const Burn& burn : listOfBurns)
		{
			if (simTime >= burn.startTime &&
				simTime < burn.startTime + burn.durationInSeconds)
			{
				currentThrustAmount += burn.thrust;
				currentThrustVector += burn.direction;
			}
		}

		if (currentThrustAmount > 0.0)
		{
			currentThrustVector = currentThrustVector.normalized();

			double thrust = std::min(currentThrustAmount, maxThrustAvailable);
			AddForce(currentThrustVector * thrust);
		}
	}

	void AddBurn(triple direction, double thrust, double startTime, double durationInSeconds)
	{
		listOfBurns.emplace_back(direction.normalized(), thrust, startTime, durationInSeconds);
	}

	void AutoOrbit(PhysicsObject* CelestialBody) 
	{
		targetObject = CelestialBody;
		autopilot = AUTO_ORBIT;
	}

	void UpdateAutoOrbit(double simTime, double dt, int RKStep) 
	{
		if (!targetObject) return;

		triple r = p - targetObject->p;
		triple vrel = v - targetObject->v;

		/*switch (RKStep) {
		case 0: { r = p - targetObject->p;
			 vrel = v - targetObject->v; } break;
		case 1: { r = p1 - targetObject->p1;
			 vrel = v1 - targetObject->v1; } break;
		case 2: { r = p2 - targetObject->p2;
			 vrel = v2 - targetObject->v2; } break;
		case 3: { r = p3 - targetObject->p3;
			 vrel = v3 - targetObject->v3; } break;
		case 4: { r = p4 - targetObject->p4;
			 vrel = v4 - targetObject->v4; } break;
		}*/

		triple rhat = r.normalized();
		triple v_radial_vec = vrel.onto(rhat);

		double v_radial = v_radial_vec.magnitude();
		triple v_tangent_vec = vrel - v_radial_vec;
		double v_tangent = v_tangent_vec.magnitude();

		double rmag = r.magnitude();
		double v_circ = std::sqrt(targetObject->mu / rmag);
		triple v_circ_vec = v_tangent_vec.normalized() * v_circ;

		constexpr double RADIAL_EPS = 100.0;
		constexpr double ERROR_EPS = 1.0;

		if (std::abs(v_radial) < 150 && !request1xTimeWarp && !requestedAlready)
		{
			request1xTimeWarp = true;
			requestedAlready = true;
		}

		if (std::abs(v_radial) > RADIAL_EPS)
			return; // wait for apoapsis/periapsis
		triple error = vrel - v_circ_vec;
		if (error.magnitude() < ERROR_EPS)
		{
			autopilot = AutopilotMode::IDLE;
			resumeTimeWarp = true;
			requestedAlready = false;
			std::cout << "Manoeuvre Complete" << std::endl;
			return;
		}

		triple retrograde = -1*error.normalized();

		double Kp = 0.5;

		currentThrustVector += retrograde;
		currentThrustAmount += std::clamp(
			Kp * error.magnitude(),
			0.0,
			maxThrustAvailable
		);
		double desiredAcc = currentThrustAmount / m;

		// Compute safe dt
		double t_safe = std::min(dt, error.magnitude() / desiredAcc);

		// Apply scaled thrust for this timestep
		currentThrustAmount = currentThrustAmount * (t_safe / dt);
		currentThrustVector = retrograde;
	}

private:
	double maxThrustAvailable = 100.0;
};
