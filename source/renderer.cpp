#include "Renderer.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <algorithm>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "GravitySimulator.h"
#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "VertexBuffer.h"

void GLClearError() {
    while (glGetError() != GL_NO_ERROR);
}
bool GLLogCall(const char* function, const char* file, int line)
{
    while (GLenum error = glGetError())
    {
        std::cout << "[OpenGL ERROR] - (" << error << "):" << function << " in file: " << file << "(Line " << line << ")" << std::endl;
        return false;
    }
    return true;
}

renderer::renderer() : renderer(4, 3, 800, 600, "This is a default window") {}
renderer::renderer(const char* title) : renderer(3, 3, 800, 600, title) {}
renderer::renderer(const char* title, int openGLversionMajor, int openGLversionMinor) : renderer(openGLversionMajor, openGLversionMinor, 800, 600, title) {}
renderer::renderer(int openGLversionMajor, int openGLversionMinor, int width, int height, const char* title)
    : scrWidth(width), scrHeight(height), title(title) {
    // Initialize GLFW
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // Set OpenGL version hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, openGLversionMajor);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, openGLversionMinor);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GL_TRUE);
    // Create the window
    window = glfwCreateWindow(scrWidth, scrHeight, title, NULL, NULL);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    // Set instance pointer for callbacks
    glfwSetWindowUserPointer(window, this);

    // Set callback for framebuffer resizing
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Set callback for key events
    glfwSetKeyCallback(window, key_callback);
    // Initialize OpenGL context
    glfwMakeContextCurrent(window);
    glfwSwapInterval(VSYNC); // Enable V-Sync
    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        throw std::runtime_error("GLEW initialization error!");
    }

    std::cout << "renderer initialized with OpenGL " << openGLversionMajor << "." << openGLversionMinor << std::endl;
    std::cout << "Graphics Version: " << glGetString(GL_VERSION) << std::endl;

    // Create ImGui context
    ImGui::CreateContext();
    // Initialize ImGui for GLFW and OpenGL3
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui::StyleColorsDark();

    // After creating the context, retrieve IO pointer
    io = &ImGui::GetIO(); // Correct initialization after context creation
    io->ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange; // Optional: hide cursor in ImGui

    // Initialize OpenGL3 for ImGui
    ImGui_ImplOpenGL3_Init(glsl_version);
}

renderer::~renderer() {
    stopRendering();
    if (renderThread.joinable()) renderThread.join();
    if (window) glfwDestroyWindow(window);
    glfwTerminate();
}

void renderer::startRendering() {
    running = true;
    renderThread = std::thread(&renderer::render, this);
}

void renderer::stopRendering() {
    running = false;
    if (renderThread.joinable()) renderThread.join();
}

std::vector<double> frameTimes;
std::vector<double> frameTimes2;

void renderer::render() {
    {
        // Make OpenGL context current in this thread
        glfwMakeContextCurrent(window);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_CONSTANT_COLOR);
        timepoint start = clock1::now();
        timepoint timeSinceStart = clock1::now();
        Shader shader("res/shaders/shader.vert", "res/shaders/shader.frag");
        Shader shader2("res/shaders/shaderFilled.vert", "res/shaders/shaderFilled.frag");
        Shader shader3("res/shaders/shaderFilled.vert", "res/shaders/shaderFilled.frag");
        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        shader.Bind();
        shader.SetUniform4f("u_Colour", r, g, b, 1.0f);
        shader.Unbind();
        shader2.Bind();
        shader2.SetUniform4f("u_Colour", 1.0f, 0.0f, 0.0f, 1.0f);
        shader2.Unbind();
        shader3.Bind();
        float test = 0.0f;
        test = linkedSim->allObjects[4]->ExternalForces.magnitude() / 100.0f;
        shader3.SetUniform4f("u_Colour", test, 1.0f, 0.0f, 1.0f);
        shader3.Unbind();

        glfwSetScrollCallback(window, scroll_callback);
        glfwSetKeyCallback(window, key_callback);

        while (running && !glfwWindowShouldClose(window)) {
            glViewport(0, 0, scrWidth, scrHeight);
            ImGui::GetIO().DisplaySize = ImVec2((float)scrWidth, (float)scrHeight);
            double renderdt = (clock1::now() - start).count() / 1000000000.0;
            start = clock1::now();

            // Save render and sim times to vectors
            frameTimes.push_back((float)renderdt);
            while (frameTimes.size() > 400) {
                frameTimes.erase(frameTimes.begin());
            }
            frameTimes2.push_back(linkedSim->myDt);
            while (frameTimes2.size() > 400) {
                frameTimes2.erase(frameTimes2.begin());
            }
            
            // Clear the screen
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            setMVPMatrix(shader);
            setMVPMatrix(shader2);
            setMVPMatrix(shader3);

            {
                std::lock_guard<std::mutex> guard(linkedSim->storingPositionsMutex);
                if (linkedSim->showTraces) {
                    renderTrailsLines(linkedSim, shader2);
                }
                renderExternalForces(linkedSim, shader3);
                renderSimulatorObjects(linkedSim, shader);
            }

            renderImGui(linkedSim);
            // Handle mouse input (this part is unconventional to place here, usually in the render loop)
            if (!ImGui::GetIO().WantCaptureMouse) {
                int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
                if (state == GLFW_PRESS)
                {
                    if (mouseBTN1Held) {
                        glfwGetCursorPos(window, &linkedSim->currentMouseX, &linkedSim->currentMouseY);
                        linkedSim->deltaX = (linkedSim->currentMouseX - linkedSim->lastMouseX) * 2.0f;
                        linkedSim->deltaY = (linkedSim->currentMouseY - linkedSim->lastMouseY) * 2.0f;
                    }
                    else {
                        mouseBTN1Held = true;
                        glfwGetCursorPos(window, &linkedSim->lastMouseX, &linkedSim->lastMouseY);
                    }
                }
                if (state == GLFW_RELEASE) {
                    if (mouseBTN1Held) {
                        linkedSim->viewPosX += linkedSim->deltaX * linkedSim->zoomLevel / scrHeight;
                        linkedSim->viewPosY += -linkedSim->deltaY * linkedSim->zoomLevel / scrHeight;
                        linkedSim->deltaX = 0;
                        linkedSim->deltaY = 0;
                    }
                    mouseBTN1Held = false;
                }

                state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
                if (state == GLFW_PRESS)
                {
                    linkedSim->viewPosX = 0;
                    linkedSim->viewPosY = 0;
                }
                state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
                if (state == GLFW_PRESS) {
                    if (mouseBTN2Held) {
                        double currentMouseX, currentMouseY;
                        glfwGetCursorPos(window, &currentMouseX, &currentMouseY);

                        // Calculate the difference in mouse position
                        double deltaX = (currentMouseX - linkedSim->lastMouseX);
                        double deltaY = (currentMouseY - linkedSim->lastMouseY);

                        // Adjust the camera rotation based on the mouse movement
                        linkedSim->cameraRotationX += deltaY * 0.001f;  // Rotation around X-axis
                        linkedSim->cameraRotationY += deltaX * 0.001f;  // Rotation around Y-axis
                        if (linkedSim->cameraRotationX > 1.4f)
                            linkedSim->cameraRotationX = 1.4f;
                        if (linkedSim->cameraRotationX < -1.4f)
                            linkedSim->cameraRotationX = -1.4f;
                        // Store the current mouse position for the next frame
                        linkedSim->lastMouseX = currentMouseX;
                        linkedSim->lastMouseY = currentMouseY;
                    }
                    else {
                        mouseBTN2Held = true;
                        glfwGetCursorPos(window, &linkedSim->lastMouseX, &linkedSim->lastMouseY);
                    }
                }
                else {
                    mouseBTN2Held = false;
                }
            }
            // Swap buffers
            glfwSwapBuffers(window);
        }

    }
    // Cleanup ImGui and OpenGL
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);  // Proper cleanup of the GLFW window
}

void renderer::rendernonmt() {
    {
        // Make OpenGL context current in this thread
        glfwMakeContextCurrent(window);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_CONSTANT_COLOR);
        timepoint start = clock1::now();
        timepoint timeSinceStart = clock1::now();
        Shader shader("res/shaders/shader.vert", "res/shaders/shader.frag");
        Shader shader2("res/shaders/shaderFilled.vert", "res/shaders/shaderFilled.frag");
        Shader shader3("res/shaders/shaderFilled.vert", "res/shaders/shaderFilled.frag");
        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        shader.Bind();
        shader.SetUniform4f("u_Colour", r, g, b, 0.3f);
        shader.Unbind();
        shader2.Bind();
        shader2.SetUniform4f("u_Colour", 1.0f, 0.0f, 0.0f, 0.3f);
        shader2.Unbind();
        shader3.Bind();
        shader3.SetUniform4f("u_Colour", 0.0f, 1.0f, 0.0f, 0.3f);
        shader3.Unbind();

        glfwSetScrollCallback(window, scroll_callback);
        glfwSetKeyCallback(window, key_callback);

        while (!glfwWindowShouldClose(window)) {
            glViewport(0, 0, scrWidth, scrHeight);
            double renderdt = (clock1::now() - start).count() / 1000000000.0;
            start = clock1::now();

            // Save render and sim times to vectors
            frameTimes.push_back(renderdt);
            while (frameTimes.size() > 400) {
                frameTimes.erase(frameTimes.begin());
            }
            frameTimes2.push_back(linkedSim->myDt);
            while (frameTimes2.size() > 400) {
                frameTimes2.erase(frameTimes2.begin());
            }

            // Clear the screen
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            setMVPMatrix(shader);
            setMVPMatrix(shader2);
            setMVPMatrix(shader3);

            if (linkedSim->showTraces) {
                renderTrailsLines(linkedSim, shader2);
            }
			renderExternalForces(linkedSim, shader3);
            renderSimulatorObjects(linkedSim, shader);

            renderImGui(linkedSim);
            if (!ImGui::GetIO().WantCaptureMouse) {
                int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
                if (state == GLFW_PRESS)
                {
                    if (mouseBTN1Held) {
                        glfwGetCursorPos(window, &linkedSim->currentMouseX, &linkedSim->currentMouseY);
                        linkedSim->deltaX = (linkedSim->currentMouseX - linkedSim->lastMouseX) * 2.0f;
                        linkedSim->deltaY = (linkedSim->currentMouseY - linkedSim->lastMouseY) * 2.0f;
                    }
                    else {
                        mouseBTN1Held = true;
                        glfwGetCursorPos(window, &linkedSim->lastMouseX, &linkedSim->lastMouseY);
                    }
                }
                if (state == GLFW_RELEASE) {
                    if (mouseBTN1Held) {
                        linkedSim->viewPosX += linkedSim->deltaX * linkedSim->zoomLevel / scrHeight;
                        linkedSim->viewPosY += -linkedSim->deltaY * linkedSim->zoomLevel / scrHeight;
                        linkedSim->deltaX = 0;
                        linkedSim->deltaY = 0;
                    }
                    mouseBTN1Held = false;
                }

                state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
                if (state == GLFW_PRESS)
                {
                    linkedSim->viewPosX = 0;
                    linkedSim->viewPosY = 0;
                }
                state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
                if (state == GLFW_PRESS) {
                    if (mouseBTN2Held) {
                        double currentMouseX, currentMouseY;
                        glfwGetCursorPos(window, &currentMouseX, &currentMouseY);

                        // Calculate the difference in mouse position
                        double deltaX = (currentMouseX - linkedSim->lastMouseX);
                        double deltaY = (currentMouseY - linkedSim->lastMouseY);

                        // Adjust the camera rotation based on the mouse movement
                        linkedSim->cameraRotationX += deltaY * 0.001f;  // Rotation around X-axis
                        linkedSim->cameraRotationY += deltaX * 0.001f;  // Rotation around Y-axis
                        if (linkedSim->cameraRotationX > 1.4f)
                            linkedSim->cameraRotationX = 1.4f;
                        if (linkedSim->cameraRotationX < -1.4f)
                            linkedSim->cameraRotationX = -1.4f;
                        // Store the current mouse position for the next frame
                        linkedSim->lastMouseX = currentMouseX;
                        linkedSim->lastMouseY = currentMouseY;
                    }
                    else {
                        mouseBTN2Held = true;
                        glfwGetCursorPos(window, &linkedSim->lastMouseX, &linkedSim->lastMouseY);
                    }
                }
                else {
                    mouseBTN2Held = false;
                }
            }
            // Swap buffers
            glfwSwapBuffers(window);
            linkedSim->RunSimulation(1.0f/ ImGui::GetIO().Framerate, linkedSim->substeps);
        }

    }
    // Cleanup ImGui and OpenGL
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);  // Proper cleanup of the GLFW window
}

void renderer::setMVPMatrix(Shader& shader) {
    shader.Bind();
    float aspectRatio = static_cast<float>(scrWidth) / static_cast<float>(scrHeight);
    /* Projection Matrix */
    glm::mat4 proj = glm::ortho(-aspectRatio* linkedSim->zoomLevel, +aspectRatio* linkedSim->zoomLevel, -linkedSim->zoomLevel, +linkedSim->zoomLevel, -1.0f, +1.0f);
    /* View Matrix - Move the camera around */
    glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
    /* Model Matrix - Move the model around */
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
    /* Combine into an MVP */
    glm::mat4 mvp = proj * view * model;
    shader.SetUniformMat4f("u_MVP", mvp);
}

void renderer::setMVPMatrixNoZoom(Shader& shader) {
    shader.Bind();
    float aspectRatio = static_cast<float>(scrWidth) / static_cast<float>(scrHeight);
    /* Projection Matrix */
    glm::mat4 proj = glm::ortho(-aspectRatio, +aspectRatio, -1.0f, +1.0f, -1.0f, +1.0f);
    /* View Matrix - Move the camera around */
    glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
    /* Model Matrix - Move the model around */
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
    /* Combine into an MVP */
    glm::mat4 mvp = proj * view * model;
    shader.SetUniformMat4f("u_MVP", mvp);
}

void renderer::renderImGui(GravitySimulator* linkedSim) {
    //// Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    if(missionData)
    {
        // Render ImGui debug window
        ImGui::Begin("Mission Data");
        ImGui::Checkbox("Show Controls List:", &showControls);
        ImGui::Text("Frametime %.10fms (%.1fFPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);  // Access io correctly
        if (linkedSim != nullptr) {
            ImGui::Text("Linked Simulator dt: %.10fms (%.1fHz)", linkedSim->myDt * 1000.0, 1.0 / linkedSim->myDt);
            /*float position[3] = { linkedSim->allObjects[0]->p.x, linkedSim->allObjects[0]->p.y, linkedSim->allObjects[0]->p.z };
            ImGui::SliderFloat3("Earth Location: ", position, 0, 10000);*/
            int years = linkedSim->years;
            int days = linkedSim->days;
            int hrs = linkedSim->hours;
            int mins = linkedSim->minutes;
            double secs = linkedSim->seconds;
            if (years < 1) {
                ImGui::Text("Elapsed Time: %i days %02i:%02i:%06.3f (Timewarp %.0fx)", days, hrs, mins, secs, linkedSim->timeWarp);
            }
            else if (years == 1) {
                ImGui::Text("Elapsed Time: %i year %i days %02i:%02i:%06.3f (Timewarp %.0fx)", years, days, hrs, mins, secs, linkedSim->timeWarp);
            }
            else {
                ImGui::Text("Elapsed Time: %i years %i days %02i:%02i:%06.3f (Timewarp %.0fx)", years, days, hrs, mins, secs, linkedSim->timeWarp);
            }
            ImGui::Text("Simulator Delta T: %.5fs", (linkedSim->timeWarp * linkedSim->myDt) / linkedSim->substeps);
            ImGui::Text("Substeps: %i", linkedSim->substeps);
            ImGui::DragFloat("Position store Delay: ", &linkedSim->positionStoreDelay, 0.01f, 100.0f);
            ImGui::DragInt("Number of stored positions: ", &linkedSim->numberOfStoredPositions, 1, 1000);
            ImGui::Checkbox("Use Runge-Kutta 4th order method: ", &linkedSim->useRK);
            // Dropdown menu to select an object
            std::vector<PhysicsObject*> vec = linkedSim->allObjects;
            std::vector<std::string> objectNames;
            for (size_t i = 0; i < linkedSim->allObjects.size(); ++i) {
                objectNames.push_back(linkedSim->allObjects[i]->name);
            }

            // Convert names to a char* array (required by ImGui::Combo)
            std::vector<const char*> objectNamesCStr;
            for (const auto& name : objectNames) {
                objectNamesCStr.push_back(name.c_str());
            }
            selectedObjectIndex = (int)std::distance(vec.begin(), std::find(vec.begin(), vec.end(), linkedSim->selectedObject)); // Index of the selected object
            if (selectedObjectIndex >= vec.size()) {
                selectedObjectIndex = 0;
                linkedSim->selectedObject = linkedSim->allObjects[0];
            }
            selectedObjectIndex2 = (int)std::distance(vec.begin(), std::find(vec.begin(), vec.end(), linkedSim->selectedObject->referenceObject));
            if (selectedObjectIndex2 >= vec.size()) {
                selectedObjectIndex2 = 0;
            }
            // Render the dropdown
            if (ImGui::Combo("Select Object", &selectedObjectIndex, objectNamesCStr.data(), (int)objectNamesCStr.size())) {
                // Optional: Handle object selection changes
            }

            // Display the selected object's distance
            double distance = linkedSim->selectedObject->p.magnitude();
            ImGui::Text("Current Distance From Centre: %.5f m (%.5f ly)", (linkedSim->selectedObject->p).magnitude(), (linkedSim->selectedObject->p).magnitude() / 9.461e15);
            triple currentV = (linkedSim->selectedObject->v - linkedSim->selectedObject->referenceObject->v);
            ImGui::Text("Current Speed: %.5f m/s (%.5fc)", currentV.magnitude(), currentV.magnitude() / 299792458.0);
            triple radialV = (linkedSim->selectedObject->v - linkedSim->selectedObject->referenceObject->v).onto((linkedSim->selectedObject->p - linkedSim->selectedObject->referenceObject->p).normalized());
            ImGui::Text("Radial Speed: %.5f m/s (%.5fc)", radialV.magnitude(), radialV.magnitude() / 299792458.0);
            triple tangenV = currentV - radialV;
            ImGui::Text("Tangential Speed: %.5f m/s (%.5fc)", tangenV.magnitude(), tangenV.magnitude() / 299792458.0);

            // Render the dropdown
            if (ImGui::Combo("Select Reference Object", &selectedObjectIndex2, objectNamesCStr.data(), (int)objectNamesCStr.size())) {
                // Optional: Handle object selection changes
            }
            if (linkedSim->selectedObject != linkedSim->allObjects[selectedObjectIndex])
            {
                linkedSim->selectedObject = linkedSim->allObjects[selectedObjectIndex];
                selectedObjectIndex2 = (int)std::distance(vec.begin(), std::find(vec.begin(), vec.end(), linkedSim->selectedObject->referenceObject));
            }
            linkedSim->selectedObject->referenceObject = linkedSim->allObjects[selectedObjectIndex2];

            // Adding energy and momentum readouts

            /*float totalEnergy = 0;
            triple totalEnergyVector;
            for (int i = 0; i < linkedSim->allObjects.size(); i++) {
                totalEnergyVector += linkedSim->allObjects[i]->m * linkedSim->allObjects[i]->v;
            }
            totalEnergy = totalEnergyVector.magnitudef();
            ImGui::Text("Total Momentum: %.5f J", totalEnergy);
            totalEnergy = 0;
            for (int i = 0; i < linkedSim->allObjects.size(); i++) {
                totalEnergy += 0.5 * linkedSim->allObjects[i]->m * linkedSim->allObjects[i]->v * linkedSim->allObjects[i]->v;
                for (int j = 0; j < linkedSim->allObjects.size(); j++) {
                    if (i == j) {
                        continue;
                    }
                    float distance1 = (linkedSim->allObjects[i]->p - linkedSim->allObjects[j]->p).magnitudef();
                    totalEnergy -= (linkedSim->G * linkedSim->allObjects[i]->m * linkedSim->allObjects[j]->m) / distance1;
                }
            }
            ImGui::Text("Total Kinetic Energy: %.5f J", totalEnergy);*/
        }
        //ImGui::PlotLines("Frame Time", frameTimes.data(), frameTimes.size(), 0, nullptr, 0.0f, 0.01f, ImVec2(0, 100));
        /*ImGui::Text("Window Size: %dx%d", scrWidth, scrHeight);
        ImGui::Text("Angle: %.2f° %.2f°", linkedSim->cameraRotationX, linkedSim->cameraRotationY);
        ImGui::Text("Position: %.2f° %.2f°", linkedSim->viewPosX, linkedSim->viewPosY);*/
        ImGui::End();
    }
    if (showControls) {
        ImGui::Begin("Controls List");
        ImGui::Text("F      - Fullscreen");
        ImGui::Text("C      - Show Controls");
        ImGui::Text("M      - Show Mission Data");
        ImGui::Text("P      - Pause Simulation");
        ImGui::Text("TAB    - Next object (SHIFT + TAB for previous)");
        ImGui::Text(".>     - Timewarp x2");
        ImGui::Text(",<     - Timewarp /2");
        ImGui::Text("/      - Timewarp 1x");
        ImGui::Text("LMB    - Slew View");
        ImGui::Text("RMB    - Reset View");
        ImGui::Text("MMB    - Rotate View");
        ImGui::Text("SCROLL - Zoom View");
        ImGui::End();
    }

    // Render ImGui frame
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void renderer::run() {
    running = true;
    if (renderingMethod == RenderingMethod::SingleThreading) {
        // Start rendering in a separate thread
        renderThread = std::thread(&renderer::rendernonmt, this);
        // Poll events on the main thread
        pollEvents();

        // Wait for render thread to finish
        if (renderThread.joinable()) {
            renderThread.join();
        }
    }
    else {
        // Start rendering in a separate thread
        renderThread = std::thread(&renderer::render, this);
        // Poll events on the main thread
        pollEvents();

        // Wait for render thread to finish
        if (renderThread.joinable()) {
            renderThread.join();
        }
    }
    glfwTerminate();  // Terminate GLFW only after render thread finishes
}

void renderer::pollEvents() {
    // Ensure this is called on the main thread
    glfwMakeContextCurrent(nullptr);
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
    }
    running = false;  // Stop the rendering loop after closing the window
}

void renderer::linkSimulator(GravitySimulator* simulator) {
    linkedSim = simulator;
    std::cout << "Linked Simulator!" << std::endl;
}

void renderer::renderSimulatorObjects(GravitySimulator* simulator, Shader& shader) {
    float screenHeightInv = 1.0f / scrHeight;
    indexBuffer.clear();
    positions3.clear();

    centre[0] = simulator->viewPosX + ((simulator->deltaX) * simulator->zoomLevel * screenHeightInv);
    centre[1] = simulator->viewPosY + ((-simulator->deltaY) * simulator->zoomLevel * screenHeightInv);
    
    int selectedObjIndex;
    for (unsigned int i = 0; i < simulator->allObjects.size(); i++)
    {
        if (simulator->allObjects[i] == simulator->selectedObject) {
            selectedObjIndex = i;
        }
        float _va1l = simulator->allObjects[i]->radius;
        if (_va1l / simulator->zoomLevel < 2.0f)
        {
            _va1l = simulator->zoomLevel * 2.0f;
        }

        // Calculate the object's position relative to the selected object
        double objX, objY, objZ;
		
        if (simulator->selectedObject == 0) {
            triple lastPos = simulator->allObjects[i]->p;
            objX = (lastPos.x);
            objY = (lastPos.y);
            objZ = (lastPos.z);
        }
        else {
            triple lastPos = simulator->allObjects[i]->p;
			triple selectedPos = simulator->selectedObject->p;
            objX = (lastPos.x - selectedPos.x);
            objY = (lastPos.y - selectedPos.y);
            objZ = (lastPos.z - selectedPos.z);
        }

        // Apply camera rotation with Z as the up-down axis
        // Yaw (Z-axis rotation)
        double cosZ = (float)cos(simulator->cameraRotationY);
        double sinZ = (float)sin(simulator->cameraRotationY);
        double tempX = objX * cosZ - objY * sinZ;
        double tempY = objX * sinZ + objY * cosZ;

        // Pitch (X-axis rotation)
        float cosX = (float)cos(simulator->cameraRotationX);
        float sinX = (float)sin(simulator->cameraRotationX);
        float finalY = (float)(tempY * cosX - objZ * sinX);
        float finalZ = (float)(tempY * sinX + objZ * cosX + centre[1] * scrHeight);
        float finalX = (float)(tempX + centre[0] * scrHeight);

        // Use the final X and Y positions for rendering
        positions3.insert(positions3.end(), {
            (-_va1l + finalX) * screenHeightInv, (-_va1l + finalZ) * screenHeightInv, 0.0f, 0.0f,
            (_va1l + finalX) * screenHeightInv, (-_va1l + finalZ) * screenHeightInv, 1.0f, 0.0f,
            (_va1l + finalX) * screenHeightInv, (_va1l + finalZ) * screenHeightInv, 1.0f, 1.0f,
            (-_va1l + finalX) * screenHeightInv, (_va1l + finalZ) * screenHeightInv, 0.0f, 1.0f });
        /*positions3.insert(positions3.end(), {
            -0.5f, -0.5f, 0.0f, 0.0f,
             0.5f, -0.5f, 1.0f, 0.0f,
             0.5f,  0.5f, 1.0f, 1.0f,
            -0.5f,  0.5f, 0.0f, 1.0f }); */

        indexBuffer.insert(indexBuffer.end(),
            { (0 + 4 * i), (1 + 4 * i), (2 + 4 * i),
            (2 + 4 * i), (3 + 4 * i), (0 + 4 * i) });
        //shader.Bind();
        ////std::clog << "I am drawing a circle at " << finalX / scrWidth << ", " << finalZ / scrHeight << " with radius " << _va1l / scrHeight << std::endl;
        //shader.SetUniform2f("u_Position", finalX / scrWidth, finalZ / scrHeight);
        //shader.SetUniform1f("u_Radius", _va1l / scrHeight);

    }

    VertexArray va1;
    VertexBuffer vb(positions3.data(), static_cast<int>(positions3.size() * sizeof(float)));
    VertexBufferLayout layout;
    layout.Push<float>(2);
    layout.Push<float>(2);
    va1.AddBuffer(vb, layout);
    IndexBuffer ib1(indexBuffer.data(), static_cast<int>(indexBuffer.size()));
    
    Draw(va1, ib1, shader, window);
}

void renderer::renderTrails(GravitySimulator* simulator, Shader& shader) {
    float screenHeightInv = 1.0f / scrHeight;
    indexBuffer.clear();
    positions3.clear();

    centre[0] = simulator->viewPosX + ((simulator->deltaX) * simulator->zoomLevel * screenHeightInv);
    centre[1] = simulator->viewPosY + ((-simulator->deltaY) * simulator->zoomLevel * screenHeightInv);
    unsigned int baseIndex = 0;
    for (unsigned int i = 0; i < simulator->allObjects.size(); i++)
    {
        if (!simulator->allObjects[i]->pastPositions.empty()) {
            for (unsigned int j = 0; j < simulator->allObjects[i]->pastPositions.size(); j++)
            {
                float _va1l = simulator->allObjects[i]->radius * 0.5f;
                if (_va1l / simulator->zoomLevel < 1.5f)
                {
                    _va1l = simulator->zoomLevel * 1.5f;
                }
                triple pastPosition, referencePosition, referenceCurrentPosition;
                pastPosition = simulator->allObjects[i]->pastPositions[j];
                if (simulator->referenceObject != nullptr)
                {
                    referencePosition = simulator->referenceObject->pastPositions[j];
                    referenceCurrentPosition = simulator->referenceObject->p;
                }
                if (simulator->allObjects[i]->referenceObject != nullptr)
                {
                    referencePosition = simulator->allObjects[i]->referenceObject->pastPositions[j];
                    referenceCurrentPosition = simulator->allObjects[i]->referenceObject->p;
                }
                // Calculate the object's position relative to the selected object
                double objX, objY, objZ;
                if (simulator->selectedObject == nullptr) {
                        objX = (pastPosition.x) - (referencePosition.x) + referenceCurrentPosition.x;
                        objY = (pastPosition.y) - (referencePosition.y) + referenceCurrentPosition.y;
                        objZ = (pastPosition.z) - (referencePosition.z) + referenceCurrentPosition.z;
                }
                else 
                {
                        objX = (pastPosition.x) - (referencePosition.x) + referenceCurrentPosition.x - (simulator->selectedObject->p.x);
                        objY = (pastPosition.y) - (referencePosition.y) + referenceCurrentPosition.y - (simulator->selectedObject->p.y);
                        objZ = (pastPosition.z) - (referencePosition.z) + referenceCurrentPosition.z - (simulator->selectedObject->p.z);
                }

                // Apply camera rotation with Z as the up-down axis
                // Yaw (Z-axis rotation)
                double cosZ = (float)cos(simulator->cameraRotationY);
                double sinZ = (float)sin(simulator->cameraRotationY);
                double tempX = objX * cosZ - objY * sinZ;
                double tempY = objX * sinZ + objY * cosZ;

                // Pitch (X-axis rotation)
                float cosX = (float)cos(simulator->cameraRotationX);
                float sinX = (float)sin(simulator->cameraRotationX);
                float finalY = (float)(tempY * cosX - objZ * sinX);
                float finalZ = (float)(tempY * sinX + objZ * cosX + centre[1] * scrHeight);
                float finalX = (float)(tempX + centre[0] * scrHeight);

                // Use the final X and Y positions for rendering
                positions3.insert(positions3.end(), {
                    (-_va1l + finalX) * screenHeightInv, (-_va1l + finalZ) * screenHeightInv, 0.0f, 0.0f,
                    (_va1l + finalX) * screenHeightInv, (-_va1l + finalZ) * screenHeightInv, 1.0f, 0.0f,
                    (_va1l + finalX) * screenHeightInv, (_va1l + finalZ) * screenHeightInv, 1.0f, 1.0f,
                    (-_va1l + finalX) * screenHeightInv, (_va1l + finalZ) * screenHeightInv, 0.0f, 1.0f });
                baseIndex += 4;

                // Add indices for the current quad
                indexBuffer.insert(indexBuffer.end(),
                    { baseIndex, baseIndex + 1, baseIndex + 2,
                      baseIndex + 2, baseIndex + 3, baseIndex });
            }
        }
    }

    VertexArray va1;
    VertexBuffer vb(positions3.data(), static_cast<int>(positions3.size() * sizeof(float)));
    VertexBufferLayout layout;
    layout.Push<float>(2);
    layout.Push<float>(2);
    va1.AddBuffer(vb, layout);
    IndexBuffer ib1(indexBuffer.data(), static_cast<int>(indexBuffer.size()));

    Draw(va1, ib1, shader, window);
}

void renderer::renderTrailsLines(GravitySimulator* simulator, Shader& shader)
{
    float screenHeightInv = 1.0f / scrHeight;
    indexBuffer.clear();
    positions3.clear();

    centre[0] = simulator->viewPosX + ((simulator->deltaX) * simulator->zoomLevel * screenHeightInv);
    centre[1] = simulator->viewPosY + ((-simulator->deltaY) * simulator->zoomLevel * screenHeightInv);

    /* ============================
       FIX 1: freeze live positions
       ============================ */
    std::vector<triple> frozenPositions(simulator->allObjects.size());
    for (size_t k = 0; k < simulator->allObjects.size(); ++k)
        frozenPositions[k] = simulator->allObjects[k]->p;

    triple frozenSelectedPos;
    if (simulator->selectedObject)
        frozenSelectedPos = simulator->selectedObject->p;

    for (unsigned int i = 0; i < simulator->allObjects.size(); i++)
    {
        if (!simulator->allObjects[i]->pastPositions.empty())
        {
            unsigned int baseIndex = positions3.size() / 4;
            unsigned int lastIndex = simulator->allObjects[i]->pastPositions.size();

            /* FIX 2: use frozen current position */
            triple currentPosition = frozenPositions[i];

            for (unsigned int j = 0; j < lastIndex; j++)
            {
                float _va1l = simulator->allObjects[i]->radius * 0.5f;
                if (_va1l / simulator->zoomLevel < 1)
                    _va1l = simulator->zoomLevel;

                triple pastPosition1 = simulator->allObjects[i]->pastPositions[j];
                triple pastPosition2 =
                    (j == lastIndex - 1)
                    ? currentPosition
                    : simulator->allObjects[i]->pastPositions[j + 1];

                triple referencePosition1{}, referencePosition2{}, referenceCurrentPosition{};

                if (simulator->allObjects[i]->referenceObject)
                {
                    auto* ref = simulator->allObjects[i]->referenceObject;
                    size_t refIdx =
                        std::distance(simulator->allObjects.begin(),
                            std::find(simulator->allObjects.begin(),
                                simulator->allObjects.end(), ref));

                    referencePosition1 = ref->pastPositions[j];
                    referencePosition2 =
                        (j == lastIndex - 1)
                        ? frozenPositions[refIdx]
                        : ref->pastPositions[j + 1];

                    /* FIX 3: frozen reference current */
                    referenceCurrentPosition = frozenPositions[refIdx];
                }

                double objX1, objY1, objZ1;
                double objX2, objY2, objZ2;

                if (simulator->selectedObject == nullptr)
                {
                    objX1 = pastPosition1.x - referencePosition1.x + referenceCurrentPosition.x;
                    objY1 = pastPosition1.y - referencePosition1.y + referenceCurrentPosition.y;
                    objZ1 = pastPosition1.z - referencePosition1.z + referenceCurrentPosition.z;

                    objX2 = pastPosition2.x - referencePosition2.x + referenceCurrentPosition.x;
                    objY2 = pastPosition2.y - referencePosition2.y + referenceCurrentPosition.y;
                    objZ2 = pastPosition2.z - referencePosition2.z + referenceCurrentPosition.z;
                }
                else
                {
                    objX1 = pastPosition1.x - referencePosition1.x + referenceCurrentPosition.x - frozenSelectedPos.x;
                    objY1 = pastPosition1.y - referencePosition1.y + referenceCurrentPosition.y - frozenSelectedPos.y;
                    objZ1 = pastPosition1.z - referencePosition1.z + referenceCurrentPosition.z - frozenSelectedPos.z;

                    objX2 = pastPosition2.x - referencePosition2.x + referenceCurrentPosition.x - frozenSelectedPos.x;
                    objY2 = pastPosition2.y - referencePosition2.y + referenceCurrentPosition.y - frozenSelectedPos.y;
                    objZ2 = pastPosition2.z - referencePosition2.z + referenceCurrentPosition.z - frozenSelectedPos.z;
                }

                double cosZ = cos(simulator->cameraRotationY);
                double sinZ = sin(simulator->cameraRotationY);

                double tempX1 = objX1 * cosZ - objY1 * sinZ;
                double tempY1 = objX1 * sinZ + objY1 * cosZ;
                double tempX2 = objX2 * cosZ - objY2 * sinZ;
                double tempY2 = objX2 * sinZ + objY2 * cosZ;

                float cosX = cos(simulator->cameraRotationX);
                float sinX = sin(simulator->cameraRotationX);

                float finalZ1 = tempY1 * sinX + objZ1 * cosX + centre[1] * scrHeight;
                float finalX1 = tempX1 + centre[0] * scrHeight;
                float finalZ2 = tempY2 * sinX + objZ2 * cosX + centre[1] * scrHeight;
                float finalX2 = tempX2 + centre[0] * scrHeight;

                float dx = finalX2 - finalX1;
                float dz = finalZ2 - finalZ1;
                float len = std::sqrt(dx * dx + dz * dz);
                if (len > 0.0001f) { dx /= len; dz /= len; }

                float px = -dz * _va1l;
                float pz = dx * _va1l;

                positions3.insert(positions3.end(), {
                    (finalX1 + px) * screenHeightInv, (finalZ1 + pz) * screenHeightInv, 0, 0,
                    (finalX1 - px) * screenHeightInv, (finalZ1 - pz) * screenHeightInv, 1, 0,
                    (finalX2 - px) * screenHeightInv, (finalZ2 - pz) * screenHeightInv, 1, 1,
                    (finalX2 + px) * screenHeightInv, (finalZ2 + pz) * screenHeightInv, 0, 1,
                    });

                indexBuffer.insert(indexBuffer.end(), {
                    baseIndex + j * 4, baseIndex + j * 4 + 1, baseIndex + j * 4 + 2,
                    baseIndex + j * 4 + 2, baseIndex + j * 4 + 3, baseIndex + j * 4
                    });
            }
        }
    }

    VertexArray va;
    VertexBuffer vb(positions3.data(), positions3.size() * sizeof(float));
    VertexBufferLayout layout;
    layout.Push<float>(2);
    layout.Push<float>(2);
    va.AddBuffer(vb, layout);
    IndexBuffer ib(indexBuffer.data(), indexBuffer.size());

    Draw(va, ib, shader, window);
}


void renderer::renderExternalForces(GravitySimulator* simulator, Shader& shader)
{
    float screenHeightInv = 1.0f / scrHeight;
    indexBuffer.clear();
    positions3.clear();

    centre[0] = simulator->viewPosX + (simulator->deltaX * simulator->zoomLevel * screenHeightInv);
    centre[1] = simulator->viewPosY + (-simulator->deltaY * simulator->zoomLevel * screenHeightInv);

    const float maxVisibleForce = 100.0f; // Force that fills full visible length
    const float lineLength = 100.0f;        // Maximum length on screen for max force (pixels)
    const float pixelThickness = 1.0f;     // desired line thickness in pixels

    // Convert pixels -> world units so length/thickness stay constant across zooms.
    // Orthographic projection used: total world height = 2 * zoomLevel maps to scrHeight pixels
    const float worldUnitsPerPixel = (simulator->zoomLevel);


    for (unsigned int i = 0; i < simulator->allObjects.size(); i++)
    {
        triple pos = simulator->allObjects[i]->p;
        triple force = simulator->allObjects[i]->ExternalForces; // total external force

        // Optional: subtract reference object position (keep original behaviour)
        if (simulator->selectedObject != nullptr && simulator->selectedObject->referenceObject != nullptr)
        {
            pos.x -= simulator->selectedObject->referenceObject->p.x;
            pos.y -= simulator->selectedObject->referenceObject->p.y;
            pos.z -= simulator->selectedObject->referenceObject->p.z;
            pos.x -= simulator->selectedObject->p.x - simulator->selectedObject->referenceObject->p.x;
            pos.y -= simulator->selectedObject->p.y - simulator->selectedObject->referenceObject->p.y;
            pos.z -= simulator->selectedObject->p.z - simulator->selectedObject->referenceObject->p.z;
        }

        // Map force magnitude -> desired pixel length (clamped)
        float forceMag = std::sqrt(force.x * force.x + force.y * force.y + force.z * force.z);
        if (forceMag <= 1e-9f) {
            continue; // nothing to draw
        }
        float desiredPixels = (forceMag / maxVisibleForce) * lineLength;
        if (desiredPixels > lineLength) desiredPixels = lineLength;
        if (desiredPixels < 1.0f) desiredPixels = 1.0f;

        // Convert desired length in pixels to world units so it's stable under zoom
        float worldLineLen = 2.0f * desiredPixels * worldUnitsPerPixel;

        // Build a world-space force direction scaled to worldLineLen
        float fx = (force.x / forceMag) * worldLineLen;
        float fy = (force.y / forceMag) * worldLineLen;
        float fz = (force.z / forceMag) * worldLineLen;

        // Camera yaw (Z) rotation
        double cosZ = cos(simulator->cameraRotationY);
        double sinZ = sin(simulator->cameraRotationY);

        // Rotate BOTH the object position and the force vector by the yaw so they live in same camera-space
        double posX_rot = pos.x * cosZ - pos.y * sinZ;
        double posY_rot = pos.x * sinZ + pos.y * cosZ;

        double fx_rot = fx * cosZ - fy * sinZ;
        double fy_rot = fx * sinZ + fy * cosZ;
        double fz_rot = fz;

        // Camera pitch (X)
        float cosX = cos(simulator->cameraRotationX);
        float sinX = sin(simulator->cameraRotationX);

        // Apply pitch rotation to the force's Y/Z components (this was missing)
        float forceY_afterPitch = static_cast<float>(fy_rot * cosX - fz_rot * sinX);
        float forceZ_afterPitch = static_cast<float>(fy_rot * sinX + fz_rot * cosX);

        // Compute final positions in same space used elsewhere in renderer (world-like coordinates)
        float finalX1 = static_cast<float>(posX_rot + centre[0] * scrHeight);
        float finalY1 = static_cast<float>(posY_rot * cosX - pos.z * sinX);
        float finalZ1 = static_cast<float>(posY_rot * sinX + pos.z * cosX + centre[1] * scrHeight);

        // Add the pitched force contribution to the endpoints
        float finalX2 = static_cast<float>(posX_rot + fx_rot + centre[0] * scrHeight);
        float finalY2 = static_cast<float>(forceY_afterPitch + posY_rot * cosX - pos.z * sinX);
        float finalZ2 = static_cast<float>(forceZ_afterPitch + posY_rot * sinX + pos.z * cosX + centre[1] * scrHeight);

        // Calculate direction in renderer space and normalize (using X,Z as in other render code)
        float dx = finalX2 - finalX1;
        float dz = finalZ2 - finalZ1;
        float length = std::sqrt(dx * dx + dz * dz);
        if (length > 0.0001f)
        {
            dx /= length;
            dz /= length;
        }

        // Convert pixel thickness to renderer world units (same space as finalX/finalZ)
        float thicknessWorld = pixelThickness * worldUnitsPerPixel;

        // Perpendicular offset in renderer space (world units)
        float px = -dz * thicknessWorld;
        float pz = dx * thicknessWorld;

        unsigned int baseIndex = positions3.size() / 4;
        positions3.insert(positions3.end(), {
            (finalX1 + px) * screenHeightInv, (finalZ1 + pz) * screenHeightInv, 0.0f, 0.0f,
            (finalX1 - px) * screenHeightInv, (finalZ1 - pz) * screenHeightInv, 1.0f, 0.0f,
            (finalX2 - px) * screenHeightInv, (finalZ2 - pz) * screenHeightInv, 1.0f, 1.0f,
            (finalX2 + px) * screenHeightInv, (finalZ2 + pz) * screenHeightInv, 0.0f, 1.0f,
            });

        indexBuffer.insert(indexBuffer.end(), {
            baseIndex, baseIndex + 1, baseIndex + 2,
            baseIndex + 2, baseIndex + 3, baseIndex
            });
    }

    VertexArray va1;
    VertexBuffer vb(positions3.data(), static_cast<int>(positions3.size() * sizeof(float)));
    VertexBufferLayout layout;
    layout.Push<float>(2);
    layout.Push<float>(2);
    va1.AddBuffer(vb, layout);
    IndexBuffer ib1(indexBuffer.data(), static_cast<int>(indexBuffer.size()));

    Draw(va1, ib1, shader, window);
}

void renderer::renderCircle(const Shader& shader) {
    // Define a quad that covers the area where the circle will be rendered
    positions3.clear();
    indexBuffer.clear();

    // Full-screen quad (or any desired bounds for the circle)
    positions3.insert(positions3.end(), {
        -0.3f, -0.3f, 0.0f, 0.0f, // Bottom-left
         0.3f, -0.3f, 1.0f, 0.0f, // Bottom-right
         0.3f,  0.3f, 1.0f, 1.0f, // Top-right
        -0.3f,  0.3f, 0.0f, 1.0f  // Top-left
        });

    // Define the indices for the quad (two triangles)
    indexBuffer.insert(indexBuffer.end(), {
        0, 1, 2, // First triangle
        2, 3, 0  // Second triangle
        });

    // Set up VAO, VBO, and IBO
    VertexArray va1;
    VertexBuffer vb(positions3.data(), static_cast<int>(positions3.size() * sizeof(float)));
    VertexBufferLayout layout;
    layout.Push<float>(2); // Position
    layout.Push<float>(2); // Texture coordinates
    va1.AddBuffer(vb, layout);
    IndexBuffer ib1(indexBuffer.data(), static_cast<int>(indexBuffer.size()));

    // Draw the quad
    Draw(va1, ib1, shader, window);
}

void renderer::Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader, GLFWwindow* window)
{
    shader.Bind();
    va.Bind();
    ib.Bind();
    GLCall(glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, nullptr));
}

void renderer::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glfwMakeContextCurrent(nullptr);
    renderer* instance = static_cast<renderer*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->scrWidth = width;
        instance->scrHeight = height;
        glViewport(0, 0, width, height);
        ImGui::GetIO().DisplaySize = ImVec2((float)width, (float)height);
    }
}
void renderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    renderer* instance = static_cast<renderer*>(glfwGetWindowUserPointer(window));
    if (key == GLFW_KEY_C && action == GLFW_PRESS)
    {
        instance->showControls = !instance->showControls;
    }
    if (key == GLFW_KEY_M && action == GLFW_PRESS)
    {
        instance->missionData = !instance->missionData;
    }
    if (key == GLFW_KEY_TAB && action == GLFW_PRESS)
    {
        if (mods & GLFW_MOD_SHIFT) { // Check if SHIFT is pressed
            // Decrease the index and wrap around
            instance->linkedSim->selectedObjectIndex -= 1;
            if (instance->linkedSim->selectedObjectIndex < 0) {
                instance->linkedSim->selectedObjectIndex = instance->linkedSim->allObjects.size() - 1;
            }
            instance->selectedObjectIndex2 = (int)std::distance(instance->linkedSim->allObjects.begin(), std::find(instance->linkedSim->allObjects.begin(), instance->linkedSim->allObjects.end(), instance->linkedSim->selectedObject->referenceObject));
            if (instance->selectedObjectIndex2 >= instance->linkedSim->allObjects.size()) {
                instance->selectedObjectIndex2 = 0;
            }
        }
        else {
            // Increase the index and wrap around
            instance->linkedSim->selectedObjectIndex += 1;
            if (instance->linkedSim->selectedObjectIndex >= instance->linkedSim->allObjects.size()) {
                instance->linkedSim->selectedObjectIndex = 0;
            }
            instance->selectedObjectIndex2 = (int)std::distance(instance->linkedSim->allObjects.begin(), std::find(instance->linkedSim->allObjects.begin(), instance->linkedSim->allObjects.end(), instance->linkedSim->selectedObject->referenceObject));
            if (instance->selectedObjectIndex2 >= instance->linkedSim->allObjects.size()) {
                instance->selectedObjectIndex2 = 0;
            }
        }
        instance->linkedSim->selectedObject = instance->linkedSim->allObjects[instance->linkedSim->selectedObjectIndex];
    }
    if (key == GLFW_KEY_PERIOD && action == GLFW_PRESS)
    {
        instance->linkedSim->timeWarp = instance->linkedSim->timeWarp * 2.0f;
    }
    if (key == GLFW_KEY_COMMA && action == GLFW_PRESS)
    {
        instance->linkedSim->timeWarp = instance->linkedSim->timeWarp / 2.0f;
        if (instance->linkedSim->timeWarp < 0.01f) instance->linkedSim->timeWarp = 0.01f;
    }
    if (key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_PRESS)
    {
        instance->linkedSim->substeps = instance->linkedSim->substeps * 2;
    }
    if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS)
    {
        instance->linkedSim->substeps = instance->linkedSim->substeps / 2;
        if (instance->linkedSim->substeps < 1) instance->linkedSim->substeps = 1;
    }
    if (key == GLFW_KEY_SLASH && action == GLFW_PRESS)
    {
        instance->linkedSim->timeWarp = 1;
    }
    if (key == GLFW_KEY_T && action == GLFW_PRESS)
    {
        if (instance->linkedSim->showTraces) instance->linkedSim->showTraces = false;
        else instance->linkedSim->showTraces = true;
    }
    if (key == GLFW_KEY_F && action == GLFW_PRESS)
    {
        if (instance->fullscreen) {
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();
            glfwSetWindowMonitor(window, NULL, instance->xpos, instance->ypos, instance->linkedSim->width, instance->linkedSim->height, 0);
            instance->fullscreen = false;
        }
        else
        {
            glfwGetWindowPos(window, &instance->xpos, &instance->ypos);
            glfwGetWindowSize(window, &instance->linkedSim->width, &instance->linkedSim->height);
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();

            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            instance->fullscreen = true;
        }
    }
    if (key == GLFW_KEY_P && action == GLFW_PRESS)
    {
        instance->linkedSim->paused = !instance->linkedSim->paused;
    }

    
}
void renderer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    glfwMakeContextCurrent(nullptr);
    renderer* instance = static_cast<renderer*>(glfwGetWindowUserPointer(window));
    float zoomScale = instance->linkedSim->zoomLevel / 10.0f;
    instance->linkedSim->zoomLevel = instance->linkedSim->zoomLevel + (float)(yoffset * zoomScale);
}