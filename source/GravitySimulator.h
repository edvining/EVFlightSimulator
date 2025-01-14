#pragma once
#include <thread>
#include <future>
#include <iostream>
#include <vector>
#include <mutex>
#include <queue>
#include <functional>
#include <condition_variable>
#include "PhysicsObject.h"

using namespace std::chrono_literals;

struct SimType {
public:
    enum RunMode { SingleThreaded, MultiThreaded, WorkerThreads, Modified };
};

class GravitySimulator
{
private:
    std::queue<std::function<void()>> workQueue;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool stop = false;
    std::vector<std::thread> threads;
public:
    bool finished = false;
    static constexpr double G = 6.67e-11;
    static constexpr double LIGHTSPEED = 3e+8;
    std::vector<PhysicsObject*> allObjects;
    std::vector<PhysicsObject*> gravitationalObjects;
    std::vector<PhysicsObject*> physicsObjects;
    PhysicsObject* selectedObject;
    PhysicsObject* referenceObject;
    int selectedObjectIndex;
    double timeElapsed = 0;
    int years = 0, days = 0, hours = 0, minutes = 0;
    double seconds = 0.0;
    float positionStoreDelay = 1000;
    double nextStorageTime = 0;
    SimType::RunMode type = SimType::Modified;
    int numThreads = (int)std::thread::hardware_concurrency();
    std::mutex accelMutex;
    std::mutex storingPositionsMutex;
    bool useRK = true;
    bool showTraces = true;
    int RKStep = 1;
    int numberOfCalculationsPerThread = 0;
    float zoomLevel = 1;
    int substeps = 1;
    double lastMouseX = 0, lastMouseY = 0;
    double currentMouseX = 0, currentMouseY = 0;
    double viewPosX = 0, viewPosY = 0;
    double deltaX = 0, deltaY = 0;
    int width = 0, height = 0;
    double cameraRotationX = 0.0f;
    double cameraRotationY = 0.0f;
    double timeWarp = 1;
    double myDt = 0;
    bool paused = false;
    bool storingPositions = false;
    int numberOfStoredPositions = 1000;
    int currentObjectIndex = 0;

    void RKSimStep(double dt)
    {
        switch (RKStep) {
        case 1: for (PhysicsObject* object : allObjects) {
            object->RK4Step1(dt);
        }break;
        case 2: for (PhysicsObject* object : allObjects) {
            object->RK4Step2(dt);
        }break;
        case 3: for (PhysicsObject* object : allObjects) {
            object->RK4Step3(dt);
        }break;
        case 4: for (PhysicsObject* object : allObjects) {
            object->RK4Step4(dt);
        }break;
        }
    }

    static double GetLightSpeed() {
        return LIGHTSPEED;
    }

    void RunSimulation(double inputdt, int substeps)
    {
        if(!paused)
        {
            SetReferenceObjects();
            /*if(selectedObject == nullptr){}
            else { ResetUniverseOrigin(selectedObject); }*/
            double dt = timeWarp * inputdt;
            myDt = inputdt;
            for (int i = 0; i < substeps; i++)
            {
                if (!useRK)
                {
                    switch (type) {
                    case 0:   CalculateForces();      break;
                    case 1:   CalculateForcesMT();    break;
                    case 2:   CalculateForcesWorker();    break;
                    case 3:   CalculateForcesModified();  break;
                    }

                    UpdateObjects((dt) / substeps, type);
                    SolveDistanceConstraints();
                    timeElapsed += dt / substeps;
                    seconds += dt / substeps;

                    if (seconds >= 60.0) {
                        minutes += static_cast<int>(timeElapsed) / 60;
                        seconds = fmod(seconds, 60.0); // Remainder after dividing by 60

                        if (minutes >= 60) {
                            hours += minutes / 60;
                            minutes %= 60; // Remainder after dividing by 60
                        }

                        if (hours >= 24) {
                            days += hours / 24;
                            hours %= 24; // Remainder after dividing by 24
                        }

                        if (days >= 365) {
                            years += days / 365;
                            days %= 365; // Remainder after dividing by 365
                        }
                    }
                    if (timeElapsed > nextStorageTime && storingPositions) {
                        for (PhysicsObject* object : allObjects)
                        {
                            object->StoreCurrentPosition(numberOfStoredPositions);
                        }
                        if (positionStoreDelay < dt / substeps) {
                            nextStorageTime += dt / substeps;
                        }
                        else { nextStorageTime += positionStoreDelay; }
                    }
                }
                else
                {
                    for (RKStep = 1; RKStep < 5; RKStep++)
                    {
                        switch (type) {
                        case 0:   CalculateForces();      break;
                        case 1:   CalculateForcesMT();    break;
                        case 2:   CalculateForcesWorker();    break;
                        case 3:   CalculateForcesModified();  break;
                        }
                        RKSimStep(dt / substeps);
                    }
                    SolveDistanceConstraints();
                    timeElapsed += dt / substeps;
                    seconds += dt / substeps;
                    if (seconds >= 60.0) {
                        minutes += static_cast<int>(seconds) / 60;
                        seconds = fmod(seconds, 60.0); // Remainder after dividing by 60

                        if (minutes >= 60) {
                            hours += minutes / 60;
                            minutes %= 60; // Remainder after dividing by 60
                        }

                        if (hours >= 24) {
                            days += hours / 24;
                            hours %= 24; // Remainder after dividing by 24
                        }

                        if (days >= 365) {
                            years += days / 365;
                            days %= 365; // Remainder after dividing by 365
                        }
                    }
                    if (timeElapsed > nextStorageTime && storingPositions) {
                        for (PhysicsObject* object : allObjects)
                        {
                            object->StoreCurrentPosition(numberOfStoredPositions);
                            object->ClearForce();
                        }
                        if (positionStoreDelay < dt / substeps) {
                            nextStorageTime += dt / substeps;
                        }
                        else { nextStorageTime += positionStoreDelay; }
                    }
                    else {
                        for (PhysicsObject* object : allObjects)
                        {
                            object->ClearForce();
                        }
                    }

                }
            }
            for (PhysicsObject* object : allObjects)
            {
                object->ClearExternalForce();
            }
            
        }
    }

    void ResetUniverseOrigin(PhysicsObject* selectedObject) {
        for (PhysicsObject* object : allObjects)
        {
            if (object == selectedObject) {
                continue;
            }
            else {
                object->p -= selectedObject->p;
                /*for (auto& position : object->pastPositions) {
                    position -= selectedObject->p;
                }*/
            }
        }
        selectedObject->p = vec3(0, 0, 0);
    }

    void SolveDistanceConstraints() {
        size_t k = allObjects.size();
        for (int i = 0; i < k; i++) {
            for (int j = i + 1; j < k; j++) {
                triple displacement = (allObjects[j]->p - allObjects[i]->p);
                double distance = displacement.magnitude();
                if (distance == 0) {
                    allObjects[i]->p -= triple(0, 1, 0);
                    allObjects[j]->p += triple(0, 1, 0);
                    triple displacement = (allObjects[j]->p - allObjects[i]->p);
                    double distance = displacement.magnitude();
                }
                double combinedRadii = allObjects[i]->radius + allObjects[j]->radius;

                // Check if the objects are intersecting
                if (distance < combinedRadii) {
                    // Normalize the displacement vector to get the collision normal
                    triple normal = displacement.normalized();

                    // Calculate the overlap (negative means penetration)
                    double overlap = distance - combinedRadii;

                    // Adjust positions to resolve the overlap
                    double totalMass = allObjects[i]->m + allObjects[j]->m;
                    allObjects[j]->p -= normal * (overlap * (allObjects[i]->m / totalMass));
                    allObjects[i]->p += normal * (overlap * (allObjects[j]->m / totalMass));
                    double e = 0.5;
                    triple* v1 = &allObjects[i]->v;
                    triple* v2 = &allObjects[j]->v;
                    double m1 = allObjects[i]->m;
                    double m2 = allObjects[j]->m;
                    triple relativeVelocity = *v2 - *v1;
                    double velocityAlongNormal = relativeVelocity.Dot(normal);
                    if (velocityAlongNormal > 0) continue; // Skip if moving apart

                    double restitution = 0.5; // Coefficient of restitution
                    double impulseMagnitude = -(1 + restitution) * velocityAlongNormal / (1 / m1 + 1 / m2);
                    triple impulse = normal * impulseMagnitude;

                    *v1 -= impulse / m1;
                    *v2 += impulse / m2;

                    /**v1 =    (e * *v1 * m2 - e * *v2 * m2 + m1 * *v1 + m2 * *v2) / (m1 + m2);
                    *v2 = -1*(e * *v1 * m1 - e * *v2 * m1 - m1 * *v1 - m2 * *v2) / (m1 + m2);*/
                    if (v1->magnitude() > LIGHTSPEED) {
                        *v1 = v1->normalized() * LIGHTSPEED;
                    }
                    if (v2->magnitude() > LIGHTSPEED) {
                        *v2 = v2->normalized() * LIGHTSPEED;
                    }
                }
            }
        }
    }

    void PurgeObjects()
    {
        allObjects.clear();
    }

    void CalculateForces()
    {
        size_t k = allObjects.size();
        for (int i = 0; i < k; i++)
        {
            allObjects[i]->GPE = 0;
            for (int j = i; j < k; j++)
            {
                if (i == j)
                    continue;
                if (allObjects[i]->contributesToGravity || allObjects[j]->contributesToGravity)
                {
                    CalculateForce(i, j);
                }
            }
        }
    }

    void CalculateForcesMT()
    {
        std::vector<std::thread> threads;
        size_t k = allObjects.size();
        for (int i = 0; i < k; i++)
        {
            threads.push_back(std::thread([this, i, k]() {
                allObjects[i]->GPE = 0;
                for (int j = i; j < k; j++)
                {
                    if (i == j)
                        continue;
                    if (allObjects[i]->contributesToGravity || allObjects[j]->contributesToGravity)
                    {
                        CalculateForce(i, j);

                    }
                }
                }));
        }
        JoinThreads(threads);
        threads.clear();
    }

    void CalculateForcesWorker() {
        int k = (int)allObjects.size();
        for (int i = 0; i < k; i++)
        {
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                workQueue.push([this, i, k] { CalculateThisObjectsForces(i, k); });
            }
            condition.notify_one();
        }
    }

    void CalculateForcesModified() {
        size_t k = gravitationalObjects.size();
        for (int i = 0; i < k; i++)
        {
            for (int j = i; j < k; j++)
            {
                if (i == j)
                    continue;
                CalculateForceGrav(i, j);
            }
        }
        size_t l = physicsObjects.size();
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < k; j++) {
                CalculateForcePhys(i, j);
            }
        }
    }

    void CalculateForcesModifiedMT() {
        std::vector<std::thread> threads;
        size_t k = gravitationalObjects.size();
        for (int i = 0; i < k; i++)
        {
            threads.push_back(std::thread([this, i, k]()
                {
                    for (int j = i; j < k; j++)
                    {
                        if (i == j)
                            continue;
                        CalculateForceGrav(i, j);
                    }
                }));
        }
        JoinThreads(threads);
        threads.clear();
        size_t l = physicsObjects.size();
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < k; j++) {
                CalculateForcePhys(i, j);
            }
        }
    }

    void CalculateThisObjectsForces(int i, int k) {
        for (int j = i; j < k; j++)
        {
            if (i == j)
                continue;
            if (allObjects[i]->contributesToGravity || allObjects[j]->contributesToGravity)
            {
                CalculateForce(i, j);
            }
        }
    }

    void CalculateForcesForObject(int currentObj, int totalNumber)
    {

        for (int j = currentObj; j < totalNumber; j++)
        {
            if (currentObj == j)
                continue;

            triple displacement = allObjects[j]->p - allObjects[currentObj]->p;
            triple force = (G * displacement.normalized()) / displacement.sqrMagnitude();
            triple a1 = force * allObjects[j]->m;
            triple a2 = force * allObjects[currentObj]->m;

            accelMutex.lock();
            allObjects[currentObj]->a += a1;
            allObjects[j]->a -= a2;
            accelMutex.unlock();
        }
    }

    void CalculateForcesForMT(int lowerObj, int upperObj, int totalObjs)
    {
        for (int i = lowerObj; i < upperObj; i++)
        {
            allObjects[i]->GPE = 0;
            for (int j = i; j < totalObjs; j++)
            {
                if (i == j)
                    continue;

                if (allObjects[i]->contributesToGravity || allObjects[j]->contributesToGravity) CalculateForce(i, j);
            }
        }
    }

    void CalculateForceBetween(int i, int j)
    {
        triple displacement = allObjects[j]->p - allObjects[i]->p;
        triple force = (G * displacement.normalized()) / displacement.sqrMagnitude();
        triple a1 = force * allObjects[j]->m;
        triple a2 = force * allObjects[i]->m;

        allObjects[i]->a += a1;
        allObjects[j]->a -= a2;
    }

    void DoNothing()
    {

    }

    void JoinThreads(std::vector<std::thread>& threads) {
        for (auto& thread : threads) { thread.join(); }
    }

    void CalculateForcesMTOld2()
    {
        size_t k = allObjects.size();
        int numberOfLoops = ((int)k / numThreads) + 1;
        std::vector<std::thread> threads;
        int currentThread = 0;
        for (int i = 0; i < numberOfLoops; i++)
        {
            threads.push_back(std::thread([this, i, k]() {CalculateForcesForObject(i, (int)k); }));
            currentThread++;
            if (currentThread > numThreads - 1) {
                currentThread = 0;
                for (auto& thread : threads) { thread.join(); }
                threads.clear();
            }
        }
        for (auto& thread : threads) { thread.join(); }
        threads.clear();
    }

    void CalculateForcesMTOld()
    {
        int k = (int)allObjects.size();
        int numberOfBatches = (int)std::floorf((k / (float)numThreads)) + 1;

        for (int batch = 0; batch < numberOfBatches; batch++)
        {
            int a = batch * numThreads;
            int numThreadsToCreate = k - a;
            if (numThreadsToCreate > numThreads) numThreadsToCreate = numThreads;
            {
                std::vector<std::thread> threads(numThreadsToCreate);
                for (int i = a; i < (batch + 1) * numThreads; i++)
                {
                    if (i < k)
                    {
                        int currentThread = i - (a);
                        threads[currentThread] = std::thread([this, i, k]()
                            {
                                CalculateForcesForObject(i, k);
                            });
                    }
                }

                /*for (int i = 0; i < k; i++)
                {
                    if (i < k)
                    {
                        threads[i] = std::thread([this, i, k]()
                            {
                                CalculateForcesForObject(i, k);
                            });
                    }
                }*/
                for (auto& thread : threads) {
                    thread.join();
                }
            }
        }
    }

    void CalculateForcesAsync()
    {
        size_t k = allObjects.size();
        const size_t numThreads = std::thread::hardware_concurrency();
        std::vector<std::future<void>> futures;

        auto calculateForObject = [&](size_t index) {
            allObjects[index]->GPE = 0;
            for (size_t j = 0; j < k; ++j) {
                if (index != j) {
                    CalculateForce((int)index, (int)j);
                }
            }
            };

        for (size_t i = 0; i < numThreads; ++i) {
            futures.emplace_back(std::async(std::launch::async, [=]() {
                for (size_t j = i; j < k; j += numThreads) {
                    calculateForObject(j);
                }
                }));
        }

        for (auto& future : futures) {
            future.get();
        }
    }

    void CalculateForce(int i, int j)
    {
        PhysicsObject* object1 = allObjects[i];
        PhysicsObject* object2 = allObjects[j];
        if (!useRK)
        {
            triple displacement = object2->p - object1->p;
            double magnitude = displacement.magnitude();
            triple force = (G * displacement) / (magnitude * magnitude * magnitude);
            object1->a += force * object2->m;
            object2->a -= force * object1->m;
            object1->a += object1->ExternalForces / object1->m;
            object2->a += object2->ExternalForces / object2->m;
        }
        else
        {
            switch (RKStep)
            {
            case 1:
            {
                triple displacement = object2->p - object1->p;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a1 += force * object2->m;
                object2->a1 -= force * object1->m;
                object1->a1 += object1->ExternalForces / object1->m;
                object2->a1 += object2->ExternalForces / object2->m;

                
            }break;
            case 2:
            {
                triple displacement = object2->p2 - object1->p2;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a2 += force * object2->m;
                object2->a2 -= force * object1->m;
                object1->a2 += object1->ExternalForces / object1->m;
                object2->a2 += object2->ExternalForces / object2->m;
                
            }break;
            case 3:
            {
                triple displacement = object2->p3 - object1->p3;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a3 += force * object2->m;
                object2->a3 -= force * object1->m;
                object1->a3 += object1->ExternalForces / object1->m;
                object2->a3 += object2->ExternalForces / object2->m;
                
            }break;
            case 4:
            {
                triple displacement = object2->p4 - object1->p4;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a4 += force * object2->m;
                object2->a4 -= force * object1->m;
                object1->a4 += object1->ExternalForces / object1->m;
                object2->a4 += object2->ExternalForces / object2->m;
                
            }break;
            default:
            {
                triple displacement = object2->p - object1->p;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a += force * object2->m;
                object2->a -= force * object1->m;
                object1->a += object1->ExternalForces / object1->m;
                object2->a += object2->ExternalForces / object2->m;
                
            }break;
            }
        }
    }

    void CalculateFriction(int i, int j) {
        PhysicsObject* object1 = allObjects[i];
        PhysicsObject* object2 = allObjects[j];
        triple displacement = object2->p - object1->p;
        double magnitude = displacement.magnitude();
        triple force = (G * displacement) / (magnitude * magnitude * magnitude);
        // Friction Model
        if (magnitude < object1->radius + object2->radius + 0.01f)
        {
            std::clog << "Friction applied no rk" << std::endl;
            triple relativeV = object2->v - object1->v;

            // Coefficient of friction (kinetic friction)
            float mu = 1000.0f; // Example value, you can adjust this

            // Calculate the normal force (magnitude of the contact force)
            triple normalForce = (force * (object2->m / (object1->m + object2->m))); // Approximation of normal contact force

            // Calculate the frictional force direction (opposite to the relative velocity)
            triple frictionForce = -mu * normalForce.magnitude() * relativeV.normalized();

            // Apply the friction forces
            object1->a += frictionForce / object1->m;
            object2->a -= frictionForce / object2->m;
        }
    }

    void CalculateForceGrav(int i, int j) {
        PhysicsObject* object1 = gravitationalObjects[i];
        PhysicsObject* object2 = gravitationalObjects[j];
        if (!useRK)
        {
            triple displacement = object2->p - object1->p;
            double magnitude = displacement.magnitude();
            triple force = (G * displacement) / (magnitude * magnitude * magnitude);
            object1->a += force * object2->m;
            object2->a -= force * object1->m;
            object1->a += object1->ExternalForces / object1->m;
            object2->a += object2->ExternalForces / object2->m;
        }
        else
        {
            switch (RKStep)
            {
            case 1:
            {
                triple displacement = object2->p - object1->p;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a1 += force * object2->m;
                object2->a1 -= force * object1->m;
                object1->a1 += object1->ExternalForces / object1->m;
                object2->a1 += object2->ExternalForces / object2->m;
            }break;
            case 2:
            {
                triple displacement = object2->p2 - object1->p2;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a2 += force * object2->m;
                object2->a2 -= force * object1->m;
                object1->a2 += object1->ExternalForces / object1->m;
                object2->a2 += object2->ExternalForces / object2->m;
            }break;
            case 3:
            {
                triple displacement = object2->p3 - object1->p3;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a3 += force * object2->m;
                object2->a3 -= force * object1->m;
                object1->a3 += object1->ExternalForces / object1->m;
                object2->a3 += object2->ExternalForces / object2->m;
            }break;
            case 4:
            {
                triple displacement = object2->p4 - object1->p4;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a4 += force * object2->m;
                object2->a4 -= force * object1->m;
                object1->a4 += object1->ExternalForces / object1->m;
                object2->a4 += object2->ExternalForces / object2->m;
            }break;
            default:
            {
                triple displacement = object2->p - object1->p;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a += force * object2->m;
                object2->a -= force * object1->m;
                object1->a += object1->ExternalForces / object1->m;
                object2->a += object2->ExternalForces / object2->m;
            }break;
            }
        }
    }

    void CalculateForcePhys(int i, int j) {
        PhysicsObject* object1 = physicsObjects[i];
        PhysicsObject* object2 = gravitationalObjects[j];
        if (!useRK)
        {
            triple displacement = object2->p - object1->p;
            double magnitude = displacement.magnitude();
            triple force = (G * displacement) / (magnitude * magnitude * magnitude);
            object1->a += force * object2->m;
            object1->a += object1->ExternalForces / object1->m;
        }
        else
        {
            switch (RKStep)
            {
            case 1:
            {
                triple displacement = object2->p - object1->p;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a1 += force * object2->m;
                object1->a1 += object1->ExternalForces / object1->m;
            }break;
            case 2:
            {
                triple displacement = object2->p2 - object1->p2;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a2 += force * object2->m;
                object1->a2 += object1->ExternalForces / object1->m;
            }break;
            case 3:
            {
                triple displacement = object2->p3 - object1->p3;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a3 += force * object2->m;
                object1->a3 += object1->ExternalForces / object1->m;
            }break;
            case 4:
            {
                triple displacement = object2->p4 - object1->p4;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a4 += force * object2->m;
                object1->a4 += object1->ExternalForces / object1->m;
            }break;
            default:
            {
                triple displacement = object2->p - object1->p;
                double magnitude = displacement.magnitude();
                triple force = (G * displacement) / (magnitude * magnitude * magnitude);
                object1->a += force * object2->m;
                object1->a += object1->ExternalForces / object1->m;
            }break;
            }
        }
    }

    void UpdateObjects(double dt, int type)
    {
        for (PhysicsObject* object : allObjects)
        {
            switch (type)
            {
            case 0:  object->VerletStep(dt); break;
            case 1:  object->EulerStep(dt); break;
            case 2: { object->RK4Step4(dt); } break;
            default: object->EulerStep(dt); break;
            }
            object->ClearForce();
        }
    }

    triple CalculateAcceleration(PhysicsObject* object1, triple location)
    {
        triple accelerationHere(0, 0, 0);

        for (PhysicsObject* object2 : allObjects)
        {
            if (object1 == object2)
                continue;

            triple displacement = object2->p - location;
            accelerationHere += (G * object2->m * displacement.normalized()) / displacement.sqrMagnitude();
        }

        return accelerationHere;
    }

    void AddObject(PhysicsObject* object)
    {
        if (object->contributesToGravity) {
            gravitationalObjects.push_back(object);
        }
        else {
            physicsObjects.push_back(object);
        }
        object->index = currentObjectIndex++;
        allObjects.push_back(object);
    }

    int GetNumberOfObjects()
    {
        return (int)allObjects.size();
    }

    float GetEnergy()
    {
        double energy = 0;
        for (unsigned int i = 0; i < allObjects.size(); i++)
        {
            energy += allObjects[i]->GPE + (0.5 * allObjects[i]->m * allObjects[i]->v.magnitude() * allObjects[i]->v.magnitude());
        }
        return (float)(energy / 1000000);
    }

    float GetMomentum()
    {
        float momentum;
        triple momentumVec;
        for (unsigned int i = 0; i < allObjects.size(); i++)
        {
            momentumVec += allObjects[i]->m * allObjects[i]->v;
        }
        momentum = momentumVec.magnitudef();
        return (float)(momentum);

    }

    // Worker thread function
    void workerThread() {
        while (true) {
            std::function<void()> task;

            // Lock the queue and wait for work
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                condition.wait(lock, [this] { return !workQueue.empty() || stop; });

                if (stop && workQueue.empty())
                    return;

                task = workQueue.front();
                workQueue.pop();
            }

            // Execute the task
            task();
        }
    }

    // Function to make the thread do work
    static void work(int id) {
        std::this_thread::sleep_for(1ms);
        int result = 0;
        for (int i = 0; i < id + 1; i++) {
            result += i;
        }
        std::cout << result << " ";
    }

    // Function to add work to the queue
    void addWork(int id) {
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            workQueue.push([id] { work(id); });
        }
        condition.notify_one();
    }

    void startThreads(int numberOfWorkerThreads) {
        stop = false;
        // Start worker threads
        for (int i = 0; i < numberOfWorkerThreads; ++i) {
            threads.push_back(std::thread(&GravitySimulator::workerThread, this));
        }
    }

    void stopThreads() {
        // Stop the workers
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();

        // Join all threads
        for (std::thread& t : threads) {
            if (t.joinable()) {
                t.join();
            }
        }

        // Clear the thread vector to prevent reuse issues
        threads.clear();
    }

    void SetReferenceObjects()
    {
        selectedObjectIndex = find(allObjects.begin(), allObjects.end(), selectedObject) - allObjects.begin();
        for (PhysicsObject* object : allObjects)
        {
            object->SetReferenceObject(allObjects);
            if (object->referenceObject == nullptr) {
                object->referenceObject = allObjects[0];
            }
        }
        finished = true;
    }
};
