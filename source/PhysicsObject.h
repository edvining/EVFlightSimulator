#pragma once
#include "triple.h"
#include "GravitySimulator.h"

class PhysicsObject
{
public:
	const double c = 299792458.0;
	triple p, v, a;
	triple p1, p2, p3, p4;
	triple a1, a2, a3, a4;
	triple ExternalForces;
	std::vector<triple> pastPositions;
	std::vector<triple> pastPositionstemp;
	PhysicsObject* referenceObject = nullptr;
	const std::string name;
	double GPE;
	double m;
	float radius;
	float outputPosition[3];
	bool firstIter = true;
	bool contributesToGravity = true;
	int index;
	int referenceObjectIndex = 0;
	std::mutex storingMutex;
	/// <summary>
	/// Mass, radius, position, velocity
	/// </summary>
	/// <param name="m"></param>
	/// <param name="radius"></param>
	/// <param name="p"></param>
	/// <param name="v"></param>
	/// <param name="a"></param>
	PhysicsObject(const char* name, float m, float radius, triple p, triple v, bool contributesToGravSim = true, PhysicsObject* refObj = nullptr) : name(name), p(p), v(v), a(triple::zero()), m(m), radius(radius), GPE(0), outputPosition{ (float)p.x, (float)p.y, (float)p.z }, contributesToGravity(contributesToGravSim), referenceObject(refObj) { pastPositions.push_back(p); }
	PhysicsObject() : name("Empty"), p(triple::zero()), v(triple::zero()), a(triple::zero()), m(10), radius(1), GPE(0), outputPosition{ (float)p.x, (float)p.y, (float)p.z } { pastPositions.push_back(p); }
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
		this->p += this->v * dt + this->a * (dt * dt * 0.5f);
		this->outputPosition[0] = (float)this->p.x;
		this->outputPosition[1] = (float)this->p.y;
		this->outputPosition[2] = (float)this->p.z;
		this->v += this->a * dt;
		if (this->v.magnitude() > c) {
			this->v = this->v.normalized() * c;
		}
		ClearForce();
	}

	void RK4Step1(double dt)
	{
		this->p1 = this->p;
		double dtOver2 = dt * 0.5f;
		this->p2 = this->p1 + this->v * dtOver2 + 0.5f * this->a1 * dtOver2 * dtOver2;
	}

	void RK4Step2(double dt)
	{
		double dtOver2 = dt * 0.5f;
		this->p3 = this->p1 + this->v * dtOver2 + 0.5f * this->a2 * dtOver2 * dtOver2;
	}

	void RK4Step3(double dt)
	{
		this->p4 = this->p1 + this->v * dt + 0.5f * this->a3 * dt * dt;
	}

	void RK4Step4(double dt)
	{
		triple a1 = this->a1;
		triple a2 = this->a2;
		triple a3 = this->a3;
		triple a4 = this->a4;
		this->a = (a1 + (2 * a2) + (2 * a3) + a4) / 6;
		this->p = this->p + this->v * dt + 0.5f * this->a * dt * dt;
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

	void SetReferenceObject(std::vector<PhysicsObject*> &allobjects) {
		referenceObjectIndex = find(allobjects.begin(), allobjects.end(), referenceObject) - allobjects.begin(); 
		if (referenceObject == nullptr) {
			referenceObjectIndex = 0;
		}
		referenceObject = allobjects[referenceObjectIndex];
	}
};