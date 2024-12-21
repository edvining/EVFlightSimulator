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
    window = glfwCreateWindow(scrWidth, scrHeight, title, nullptr, nullptr);
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
    if (VSYNC) glfwSwapInterval(1);
    else glfwSwapInterval(0); // Enable V-Sync
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

std::vector<float> frameTimes;
std::vector<float> frameTimes2;

void renderer::render() {
    {
        // Make OpenGL context current in this thread
        glfwMakeContextCurrent(window);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_CONSTANT_COLOR);
        timepoint start = clock1::now();
        timepoint timeSinceStart = clock1::now();
        Shader shader("res/shaders/shader.vert", "res/shaders/shader.frag");
        Shader shader2("res/shaders/shader.vert", "res/shaders/shader.frag");
        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        shader.Bind();
        shader.SetUniform4f("u_Colour", r, g, b, 1.0f);
        shader.Unbind();
        shader2.Bind();
        shader2.SetUniform4f("u_Colour", 1.0f, 0.0f, 0.0f, 1.0f);
        shader2.Unbind();

        glfwSetScrollCallback(window, scroll_callback);
        glfwSetKeyCallback(window, key_callback);

        while (running && !glfwWindowShouldClose(window)) {
            glViewport(0, 0, scrWidth, scrHeight);
            ImGui::GetIO().DisplaySize = ImVec2((float)scrWidth, (float)scrHeight);
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
            
            if (linkedSim->showTraces) {
                renderTrails(linkedSim, shader2);
            }
            renderSimulatorObjects(linkedSim, shader);

            renderImGui(linkedSim);
            // Handle mouse input (this part is unconventional to place here, usually in the render loop)
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
        Shader shader2("res/shaders/shader.vert", "res/shaders/shader.frag");
        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        shader.Bind();
        shader.SetUniform4f("u_Colour", r, g, b, 1.0f);
        shader.Unbind();
        shader2.Bind();
        shader2.SetUniform4f("u_Colour", 1.0f, 0.0f, 0.0f, 1.0f);
        shader2.Unbind();

        glfwSetScrollCallback(window, scroll_callback);
        glfwSetKeyCallback(window, key_callback);

        while (!glfwWindowShouldClose(window)) {
            glViewport(0, 0, scrWidth, scrHeight);
            ImGui::GetIO().DisplaySize = ImVec2((float)scrWidth, (float)scrHeight);
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

            if (linkedSim->showTraces) {
                renderTrails(linkedSim, shader2);
            }
            renderSimulatorObjects(linkedSim, shader);

            renderImGui(linkedSim);
            // Handle mouse input (this part is unconventional to place here, usually in the render loop)
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
            // Swap buffers
            glfwSwapBuffers(window);
            glfwPollEvents();
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

void renderer::renderImGui(GravitySimulator* linkedSim) {
    //// Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    // Render ImGui debug window
    ImGui::Begin("ImGui Debugger");
    ImGui::Text("Frametime %.10fms (%.1fFPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);  // Access io correctly
    if (linkedSim != nullptr) {
        ImGui::Text("Linked Simulator dt: %.10fms (%.1fHz)", linkedSim->myDt * 1000.0, 1.0 / linkedSim->myDt);
        /*float position[3] = { linkedSim->allObjects[0]->p.x, linkedSim->allObjects[0]->p.y, linkedSim->allObjects[0]->p.z };
        ImGui::SliderFloat3("Earth Location: ", position, 0, 10000);*/
        int years = linkedSim->years;
        int days = linkedSim->days;
        int hrs = linkedSim->hours;
        int mins = linkedSim->minutes;
        float secs = linkedSim->seconds;
        if (years < 1) {
            ImGui::Text("Elapsed Time: %i days %02i:%02i:%06.3f", days, hrs, mins, secs);
        }
        else if (years == 1) {
            ImGui::Text("Elapsed Time: %i year %i days %02i:%02i:%06.3f", years, days, hrs, mins, secs);
        }
        else {
            ImGui::Text("Elapsed Time: %i years %i days %02i:%02i:%06.3f", years, days, hrs, mins, secs);
        }
        ImGui::Text("Delta T: %.5fs", (linkedSim->timeWarp * linkedSim->myDt) / linkedSim->substeps);
        ImGui::Text("Substeps: %i", linkedSim->substeps);
        ImGui::DragFloat("Position store Delay: ", &linkedSim->positionStoreDelay, 0.01f, 100.0f);
        ImGui::DragInt("Number of stored positions: ", &linkedSim->numberOfStoredPositions, 1, 1000);
        // Dropdown menu to select an object
        std::vector<PhysicsObject*> vec = linkedSim->allObjects;
        static int selectedObjectIndex = std::distance(vec.begin(), std::find(vec.begin(), vec.end(), linkedSim->selectedObject)); // Index of the selected object
        static int selectedObjectIndex2 = std::distance(vec.begin(), std::find(vec.begin(), vec.end(), linkedSim->selectedObject->referenceObject));;
        std::vector<std::string> objectNames; // Placeholder for object names
        for (size_t i = 0; i < linkedSim->allObjects.size(); ++i) {
            objectNames.push_back(linkedSim->allObjects[i]->name); // Generate names like "Object 1", "Object 2", etc.
        }

        // Convert names to a char* array (required by ImGui::Combo)
        std::vector<const char*> objectNamesCStr;
        for (const auto& name : objectNames) {
            objectNamesCStr.push_back(name.c_str());
        }

        // Render the dropdown
        if (ImGui::Combo("Select Object", &selectedObjectIndex, objectNamesCStr.data(), objectNamesCStr.size())) {
            // Optional: Handle object selection changes
        }

        // Display the selected object's distance
        if (linkedSim->allObjects.size() > selectedObjectIndex) {
            linkedSim->selectedObject = linkedSim->allObjects[selectedObjectIndex];
            linkedSim->selectedObject->referenceObject = linkedSim->allObjects[selectedObjectIndex2];
            double distance = linkedSim->selectedObject->p.magnitude();
            ImGui::Text("Current Distance From Centre: %.5f m (%.5f ly)", (linkedSim->allObjects[selectedObjectIndex]->p).magnitude(), (linkedSim->allObjects[selectedObjectIndex]->p).magnitude() / 9.461e15);
            ImGui::Text("Current Speed: %.5f m/s (%.5fc)", (linkedSim->allObjects[selectedObjectIndex]->v).magnitude(), (linkedSim->allObjects[selectedObjectIndex]->v).magnitude() / 299792458.0);
        }
        // Render the dropdown
        if (ImGui::Combo("Select Reference Object", &selectedObjectIndex2, objectNamesCStr.data(), objectNamesCStr.size())) {
            // Optional: Handle object selection changes
        }
    }
    /*ImGui::PlotLines("Frame Time", frameTimes.data(), frameTimes.size(), 0, nullptr, 0.0f, 0.01f, ImVec2(0, 100));
    ImGui::Text("Window Size: %dx%d", scrWidth, scrHeight);
    ImGui::Text("Angle: %.2f° %.2f°", linkedSim->cameraRotationX, linkedSim->cameraRotationY);
    ImGui::Text("Position: %.2f° %.2f°", linkedSim->viewPosX, linkedSim->viewPosY);*/
    ImGui::End();

    // Render ImGui frame
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void renderer::run() {
    running = true;
    // Start rendering in a separate thread
    renderThread = std::thread(&renderer::render, this); 
    // Poll events on the main thread
    pollEvents();

    // Wait for render thread to finish
    if (renderThread.joinable()) {
        renderThread.join();
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
    /*for (unsigned int i = 0; i < simulator->allObjects.size(); i++)
    {
        float _va1l = simulator->allObjects[i]->radius;
        if (_va1l * 2 / zoomLevel < 1.5f)
        {
            _va1l = zoomLevel * 2.0f;
        }
        float objX, objY, objZ;
        if (simulator->selectedObject == 0) {
            objX = (float)(simulator->allObjects[i]->p.x + centre[0] * screen_height);
            objY = (float)(simulator->allObjects[i]->p.y + centre[1] * screen_height);
            objZ = (float)(simulator->allObjects[i]->p.z + centre[1] * screen_height);
        }
        else {
            objX = (float)(simulator->allObjects[i]->p.x - simulator->selectedObject->p.x + centre[0] * screen_height);
            objY = (float)(simulator->allObjects[i]->p.y - simulator->selectedObject->p.y + centre[1] * screen_height);
            objZ = (float)(simulator->allObjects[i]->p.z - simulator->selectedObject->p.z + centre[1] * screen_height);
        }
        positions3.insert(positions3.end(), {
            (-_va1l + objX) * screenHeightInv, (-_va1l + objY) * screenHeightInv, 0.0f, 0.0f,
            (_va1l + objX) * screenHeightInv, (-_va1l + objY) * screenHeightInv, 1.0f, 0.0f,
            (_va1l + objX) * screenHeightInv, (_va1l + objY) * screenHeightInv, 1.0f, 1.0f,
            (-_va1l + objX) * screenHeightInv, (_va1l + objY) * screenHeightInv, 0.0f, 1.0f });
        indexBuffer.insert(indexBuffer.end(),
            { (0 + 4 * i), (1 + 4 * i), (2 + 4 * i),
            (2 + 4 * i), (3 + 4 * i), (0 + 4 * i) });
    }*/
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
            objX = (simulator->allObjects[i]->p.x);
            objY = (simulator->allObjects[i]->p.y);
            objZ = (simulator->allObjects[i]->p.z);
        }
        else {
            objX = (simulator->allObjects[i]->p.x - simulator->selectedObject->p.x);
            objY = (simulator->allObjects[i]->p.y - simulator->selectedObject->p.y);
            objZ = (simulator->allObjects[i]->p.z - simulator->selectedObject->p.z);
        }

        // Apply camera rotation with Z as the up-down axis
        // Yaw (Z-axis rotation)
        double cosZ = cos(simulator->cameraRotationY);
        double sinZ = sin(simulator->cameraRotationY);
        double tempX = objX * cosZ - objY * sinZ;
        double tempY = objX * sinZ + objY * cosZ;

        // Pitch (X-axis rotation)
        float cosX = cos(simulator->cameraRotationX);
        float sinX = sin(simulator->cameraRotationX);
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
    int selectedObjIndex;
    unsigned int baseIndex = 0;
    for (unsigned int i = 0; i < simulator->allObjects.size(); i++)
    {
        simulator->allObjects[i]->storingMutex.lock();
        if (!simulator->allObjects[i]->pastPositions.empty()) {
            for (unsigned int j = 0; j < simulator->allObjects[i]->pastPositions.size(); j++)
            {
                float _va1l = simulator->allObjects[i]->radius * 0.5f;
                if (_va1l / simulator->zoomLevel < 1.5f)
                {
                    _va1l = simulator->zoomLevel * 1.5f;
                }
                triple pastPosition = simulator->allObjects[i]->pastPositions[j];
                triple referencePosition, referenceCurrentPosition;
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
                double cosZ = cos(simulator->cameraRotationY);
                double sinZ = sin(simulator->cameraRotationY);
                double tempX = objX * cosZ - objY * sinZ;
                double tempY = objX * sinZ + objY * cosZ;

                // Pitch (X-axis rotation)
                float cosX = cos(simulator->cameraRotationX);
                float sinX = sin(simulator->cameraRotationX);
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
        simulator->allObjects[i]->storingMutex.unlock();
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
        instance->linkedSim->substeps = instance->linkedSim->substeps * 2.0f;
    }
    if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS)
    {
        instance->linkedSim->substeps = instance->linkedSim->substeps / 2.0f;
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
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            if (instance->VSYNC)glfwSetWindowMonitor(window, NULL, instance->xpos, instance->ypos, instance->linkedSim->width, instance->linkedSim->height, mode->refreshRate);
            else glfwSetWindowMonitor(window, NULL, instance->xpos, instance->ypos, instance->linkedSim->width, instance->linkedSim->height, 0);
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