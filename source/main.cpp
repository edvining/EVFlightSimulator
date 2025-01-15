
#include <GL/glew.h>
#define GLEW_STATIC
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <chrono>
#include "Renderer.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "Shader.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "GravitySimulator.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include <thread>
#include <atomic>
#include <future>

using application = renderer;
float minimumdt = 1.0f/50000000000000000.0f;

bool paused = false;

void RunSim(GravitySimulator* sim, application* app) {
	timepoint start = clock1::now();
	while (app->running) {
		while (!paused && app->running) {
			double dt = (clock1::now() - start).count() / 1000000000.0;
			start = clock1::now();
			if (dt < minimumdt) {
				sim->RunSimulation(minimumdt, sim->substeps);
				do {
				} while ((clock1::now() - start).count() / 1000000000.0 < minimumdt);
			}
			else {
				sim->RunSimulation(dt, sim->substeps);
			}
		}
	}
}



void LotsOfBalls() {
    // Initialise application
    application app1("OpenGL", 4, 6);
    app1.renderingMethod = RenderingMethod::SingleThreading;
    // Initialise simulator and objects
    GravitySimulator simulator;
    simulator.zoomLevel = /*0.01f;*/ 63781.37f; // metres / pixel7
    simulator.timeWarp = 1;
    simulator.substeps = 1;
    simulator.type = SimType::SingleThreaded;
    simulator.useRK = true;
    simulator.showTraces = true;
    simulator.storingPositions = true;
    /*simulator.startThreads(16);*/
    simulator.cameraRotationX = 0.0f;
    PhysicsObject sun("Sun", 1988500e24f, 695700000.0f, { -1.009146052453886E+09, -6.342248515004860E+08, 2.918025134412420E+07 },
        { 1.102867590529470E+01, -8.970075624225537, -1.590822813779761E-01 });
    simulator.AddObject(&sun);
    PhysicsObject earth("Earth", 5.97219e24f, 6378137, { 1.001221085597017E+11, -1.138184590187444E+11, 3.459909583488852E+07 },
        /*{ 0, 0, 0 }*/{ 2.174985495402457E+04, 1.973326349215320E+04, -9.113292098188452E-01 });
    simulator.AddObject(&earth);

    /*PhysicsObject ball("Ball", 10, 0.5, earth.p + vec3{ 0, 0, earth.radius + 1 }, earth.v + vec3{ 0, 0, 0 }, false);
    simulator.AddObject(&ball);
    PhysicsObject ball2("Ball2", 10, 0.5, earth.p + vec3{ 2, 0, earth.radius + 10 }, earth.v + vec3{ 0, 0, 1000 }, false);
    simulator.AddObject(&ball2);*/
    PhysicsObject moon("Moon", 7.349e22f, 1737530.0f, { 9.988829984389585E+10, -1.135011116884250E+11, 6.536801629186422E+07 },
        { 2.092295035930196E+04, 1.917371149264998E+04, -4.031214427303542E+01 });
    simulator.AddObject(&moon);
    PhysicsObject mars("Mars", 6.4171e23f, 3389920.0f, { 1.838132282343054E+11, 1.077250455786663E+11, -2.233343150142968E+09 },
        { -1.132073342589737E+04, 2.296074025888803E+04, 7.591572611068891E+02 });
    simulator.AddObject(&mars);
    PhysicsObject spaceship("Spaceship", 20.0f, 10000.0f, earth.p + triple{ 100000 + 6378137, 0, 0 },
        /*{ 0, 0, 0 }*/earth.v + triple{ 0, 11005, 0 });
    simulator.AddObject(&spaceship);
    spaceship.referenceObject = &earth;
    simulator.numberOfStoredPositions = 10000;
    PhysicsObject mercury("Mercury", 3.302E+23f, 2439400.0f, 1000 * triple{ 3.252515818176519E+07, -5.550392669785608E+07, -7.567397717898630E+06 },
        1000 * triple{ 3.182356791384326E+01,  2.782212905746022E+01, -6.436334037578586E-01 });
    simulator.AddObject(&mercury);
    PhysicsObject venus("Venus", 48.685E+23f, 6051840.0f, 1000 * triple{ -5.681719175482940E+07,  9.149556918242723E+07,  4.501626945937518E+06 },
        1000 * triple{ -3.007311742715811E+01, -1.833648995571871E+01,  1.483984945419926E+00 });
    simulator.AddObject(&venus);
    PhysicsObject jupiter("Jupiter", 189818.722E+22f, 69911000.0f, 1000 * triple{ 5.530900972296501E+08,  4.966321324505337E+08, -1.443456502238074E+07 },
        1000 * triple{ -8.872966908142011E+00,  1.033880191187417E+01,  1.556173515335626E-01 });
    simulator.AddObject(&jupiter);
    PhysicsObject saturn("Saturn", 5.6834E+26f, 58232000.0f, 1000 * triple{ 1.333357825865451E+09, -5.870000938448141E+08, -4.288097567254049E+07 },
        1000 * triple{ 3.351857209931528E+00,  8.822830821269344E+00, -2.872934017773172E-01 });
    simulator.AddObject(&saturn);
    std::vector<PhysicsObject*> generatedObjs;
   /* for (int i = 0; i < 400; i++) {
        double calculatedV = sqrt((GravitySimulator::G * earth.m) / ((double)(1000000.0 * i) + earth.radius+400000.0));
        std::cout << "Hi there, generating: Empty " << i << "Naming it: " << std::format("Empty {}", i).c_str() << std::endl;
        const char* string = std::format("Empty {}", i).c_str();
        std::cout << "String: " << string << std::endl;
        generatedObjs.push_back(new PhysicsObject(string, 5, 40000.0, earth.p + triple((1000000.0 * i) + earth.radius + 400000.0, 0, 0), earth.v + triple(0, calculatedV + (0.001 * i), 0), false, &earth));
    }
    for (int i = 0; i < generatedObjs.size(); i++) {
        simulator.AddObject(generatedObjs[i]);
    }*/
    std::string string = std::format("Empty {}", 284);
    double calculatedV = sqrt((GravitySimulator::G * earth.m) / ((double)(1000000.0 * 284) + earth.radius + 400000.0));
    generatedObjs.push_back(new PhysicsObject(string.c_str(), 5, 40000.0, earth.p + triple((1000000.0 * 284) + earth.radius + 400000.0, 0, 0), earth.v + triple(0, calculatedV + (0.001 * 284), 0), false, &earth));
    simulator.AddObject(generatedObjs[0]);
    simulator.selectedObject = &earth;
    simulator.referenceObject = &sun;
    moon.referenceObject = &earth;
    earth.referenceObject = &sun;
    sun.referenceObject = &sun;
    simulator.positionStoreDelay = 1000;
    simulator.useRK = true;
    simulator.numberOfStoredPositions = 1000;
    simulator.SetReferenceObjects();
    // Link the simulator to the visualiser app
    app1.linkSimulator(&simulator);
    if(app1.renderingMethod == RenderingMethod::MultiThreading)
    {
        // Start simulator thread
        std::thread simulatorThread(RunSim, &simulator, &app1);

        // Run the application (rendering)
        app1.run();

        // Wait for the simulator thread to finish
        simulatorThread.join();
    }
    else {
        app1.run();

    }

}

void GravitationalLensing() {
    // Initialise application
    application app1("OpenGL", 4, 6);

    // Initialise simulator and objects
    GravitySimulator simulator;
    
    PhysicsObject sun("Sun", 1988500e29f, 3000, { -1.009146052453886E+09, -6.342248515004860E+08, 2.918025134412420E+07 },
        { 1.102867590529470E+01, -8.970075624225537, -1.590822813779761E-01 });
    simulator.AddObject(&sun);
    std::vector<PhysicsObject*> generatedObjs;
    for (int i = 0; i < 30; i++) {
        double calculatedV = sqrt((GravitySimulator::G * sun.m) / ((double)(1000000.0f * i) + sun.radius));
        generatedObjs.push_back(new PhysicsObject("Empty", 10, 100000, sun.p + triple((double)(10000000.0f * i)+sun.radius, 0, 0.0), sun.v + triple(0, 0, 3e8), false, &sun));
    }
    for (int i = 0; i < generatedObjs.size(); i++) {
        simulator.AddObject(generatedObjs[i]);
    }
    simulator.zoomLevel = 69570000.0f;
    simulator.selectedObject = &sun;
    simulator.referenceObject = &sun;
    simulator.showTraces = true;
    simulator.storingPositions = true;
    simulator.useRK = true;
    simulator.paused = false;
    sun.referenceObject = nullptr;
    simulator.positionStoreDelay = 1;
    simulator.numberOfStoredPositions = 1000;
    // Link the simulator to the visualiser app
    app1.linkSimulator(&simulator);

    // Start simulator thread
    std::thread simulatorThread(RunSim, &simulator, &app1);

    // Run the application (rendering)
    app1.run();

    // Wait for the simulator thread to finish
    simulatorThread.join();
}

void LotsOfGravObjs() {
    // Initialise application
    application app1("OpenGL", 4, 6);

    // Initialise simulator and objects
    GravitySimulator simulator;
    std::vector<PhysicsObject*> generatedObjs;
    PhysicsObject earth("Earth", 5.97219e24f, 6378137, { 1.001221085597017E+11, -1.138184590187444E+11, 3.459909583488852E+07 },
        /*{ 0, 0, 0 }*/{ 2.174985495402457E+04, 1.973326349215320E+04, -9.113292098188452E-01 });
    simulator.AddObject(&earth);
    PhysicsObject moon("Moon", 7.349e22f, 1737530.0f, { 9.988829984389585E+10, -1.135011116884250E+11, 6.536801629186422E+07 },
        { 2.092295035930196E+04, 1.917371149264998E+04, -4.031214427303542E+01 });
    simulator.AddObject(&moon);
    PhysicsObject sun("Sun", 1988500e24f, 695700000.0f, { -1.009146052453886E+09, -6.342248515004860E+08, 2.918025134412420E+07 },
        { 1.102867590529470E+01, -8.970075624225537, -1.590822813779761E-01 });
    simulator.AddObject(&sun);
    PhysicsObject mars("Mars", 6.4171e23f, 3389920.0f, { 1.838132282343054E+11, 1.077250455786663E+11, -2.233343150142968E+09 },
        { -1.132073342589737E+04, 2.296074025888803E+04, 7.591572611068891E+02 });
    simulator.AddObject(&mars);
    PhysicsObject spaceship("Spaceship", 20.0f, 10000.0f, earth.p + triple{ 100000 + 6378137, 0, 0 },
        /*{ 0, 0, 0 }*/earth.v + triple{ 0, 11005, 0 });
    simulator.AddObject(&spaceship);
    spaceship.referenceObject = &earth;
    PhysicsObject mercury("Mercury", 3.302E+23f, 2439400.0f, 1000 * triple{ 3.252515818176519E+07, -5.550392669785608E+07, -7.567397717898630E+06 },
        1000 * triple{ 3.182356791384326E+01,  2.782212905746022E+01, -6.436334037578586E-01 });
    simulator.AddObject(&mercury);
    PhysicsObject venus("Venus", 48.685E+23f, 6051840.0f, 1000 * triple{ -5.681719175482940E+07,  9.149556918242723E+07,  4.501626945937518E+06 },
        1000 * triple{ -3.007311742715811E+01, -1.833648995571871E+01,  1.483984945419926E+00 });
    simulator.AddObject(&venus);
    PhysicsObject jupiter("Jupiter", 189818.722E+22f, 69911000.0f, 1000 * triple{ 5.530900972296501E+08,  4.966321324505337E+08, -1.443456502238074E+07 },
        1000 * triple{ -8.872966908142011E+00,  1.033880191187417E+01,  1.556173515335626E-01 });
    simulator.AddObject(&jupiter);
    PhysicsObject saturn("Saturn", 5.6834E+26f, 58232000.0f, 1000 * triple{ 1.333357825865451E+09, -5.870000938448141E+08, -4.288097567254049E+07 },
        1000 * triple{ 3.351857209931528E+00,  8.822830821269344E+00, -2.872934017773172E-01 });
    simulator.AddObject(&saturn);
    simulator.AddObject(new PhysicsObject("Empty", 1e28f, 100000.0f, triple(0, 6957000000, 0), triple(0, -3.0e8, 0), true, &earth));
    simulator.zoomLevel = 40000.0f;
    simulator.showTraces = true;
    simulator.storingPositions = true;
    simulator.selectedObject = simulator.allObjects.back();
    simulator.useRK = true;
    simulator.positionStoreDelay = 1;
    simulator.numberOfStoredPositions = 100;
    simulator.SetReferenceObjects();
    // Link the simulator to the visualiser app
    app1.linkSimulator(&simulator);

    // Start simulator thread
    std::thread simulatorThread(RunSim, &simulator, &app1);

    // Run the application (rendering)
    app1.run();

    // Wait for the simulator thread to finish
    simulatorThread.join();
}

void GridOfParticles() {
    // Initialise application
    application app1("OpenGL", 4, 6);

    // Initialise simulator and objects
    GravitySimulator simulator;
    int size = 5;
    std::vector<PhysicsObject*> generatedObjs;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            for (int k = 0; k < size; k++)
            {
                generatedObjs.push_back(new PhysicsObject("Empty", 10e10f, 1, triple((double)(1.0f * i - 0.5f*size), (double)(1.0f * j - 0.5f *size), (double)(1.0f * k - 0.5f *size)), triple(0, 0, 0), true));
            }
        }
    }
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            for (int k = 0; k < size; k++)
            {
                generatedObjs.push_back(new PhysicsObject("Empty", 10e10f, 1, triple((double)(1.0f * i - 0.5f *size), (double)(1.0f * j - 0.5f *size)+ 50* 0.5f * size, (double)(1.0f * k - 0.5f *size)), triple(2.5, 0, 0), true));
            }
        }
    }
    for (int i = 0; i < generatedObjs.size(); i++) {
        simulator.AddObject(generatedObjs[i]);
    }
    simulator.zoomLevel = 1.0f;
    simulator.showTraces = true;
    simulator.storingPositions = true;
    simulator.useRK = true;
    simulator.paused = false;
    simulator.positionStoreDelay = 1;
    simulator.numberOfStoredPositions = 0;
    simulator.SetReferenceObjects();
    // Link the simulator to the visualiser app
    app1.linkSimulator(&simulator);

    // Start simulator thread
    std::thread simulatorThread(RunSim, &simulator, &app1);

    // Run the application (rendering)
    app1.run();

    // Wait for the simulator thread to finish
    simulatorThread.join();
}

void TestThreeBody() {
    // Initialise application
    application app1("OpenGL", 4, 6);

    // Initialise simulator and objects
    GravitySimulator simulator;
    int size = 5;
    std::vector<PhysicsObject*> generatedObjs;
    generatedObjs.push_back(new PhysicsObject("Empty", 10e10f, 1, triple(10, 0, 0), triple(0, 1, 0), true));
    generatedObjs.push_back(new PhysicsObject("Empty", 10e10f, 1, triple(0, 10, 0), triple(-1, 0, 0), true));
    generatedObjs.push_back(new PhysicsObject("Empty", 10e10f, 1, triple(10, 10, 0), triple(0, 0, 1), true));
    for (int i = 0; i < generatedObjs.size(); i++) {
        simulator.AddObject(generatedObjs[i]);
    }
    simulator.zoomLevel = 0.1f;
    simulator.showTraces = true;
    simulator.storingPositions = true;
    simulator.useRK = true;
    simulator.paused = false;
    simulator.positionStoreDelay = 1;
    simulator.numberOfStoredPositions = 0;
    simulator.SetReferenceObjects();
    // Link the simulator to the visualiser app
    app1.linkSimulator(&simulator);

    // Start simulator thread
    std::thread simulatorThread(RunSim, &simulator, &app1);

    // Run the application (rendering)
    app1.run();

    // Wait for the simulator thread to finish
    simulatorThread.join();
}

int main() {
    LotsOfBalls();
	return 0;
}