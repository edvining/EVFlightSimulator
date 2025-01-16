#pragma once
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "VertexArray.h"
#include "IndexBuffer.h"
#include "Shader.h"
#include "GravitySimulator.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) GLClearError();x;ASSERT(GLLogCall(#x, __FILE__, __LINE__));

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);

using timepoint = std::chrono::high_resolution_clock::time_point;
using clock1 = std::chrono::high_resolution_clock;
using duration = std::chrono::high_resolution_clock::duration;
enum RenderingMethod { SingleThreading, MultiThreading };
class renderer {
public:
    GLFWwindow* window = nullptr;
    const char* glsl_version = "#version 130";
    int scrWidth = 800, scrHeight = 600;
    int xpos = 0, ypos = 0;
    int origWidth = 800; // Stores original window width before fullscreen
    int origHeight = 600; // Stores original window height before fullscreen
    bool fullscreen = false;
    const char* title = "OpenGL Window";
    ImGuiIO* io;
    std::atomic<bool> running{ false };
    std::atomic<bool> windowResized{ false };
    std::thread renderThread;
    GravitySimulator* linkedSim = nullptr;
    std::vector<float> positions3;
    std::vector<unsigned int> indexBuffer;
    double lastMouseX = 0, lastMouseY = 0;
    double currentMouseX = 0, currentMouseY = 0;
    double viewPosX = 0, viewPosY = 0;
    double deltaX = 0, deltaY = 0;
    int width = 800, height = 600;
    bool paused = true;
    bool mouseBTN1Held = false;
    float cameraRotationX = 1.4f;
    float cameraRotationY = 1.4f;
    bool mouseBTN2Held = false;
    double centre[2] = { 0.0, 0.0 };
    bool VSYNC = false;
    bool showControls = true;
    bool missionData = false;
    int selectedObjectIndex = 0;
    int selectedObjectIndex2 = 0;
    RenderingMethod renderingMethod = RenderingMethod::MultiThreading;

    renderer();
    renderer(const char* title);
    renderer(const char* title, int openGLversionMajor, int openGLversionMinor);
    renderer(int openGLversionMajor, int openGLversionMinor, int width, int height, const char* title);

    ~renderer();

    void startRendering();

    void stopRendering();

    void render();

    void rendernonmt();

    void run();

    void pollEvents();

    void linkSimulator(GravitySimulator* simulator);

    void setMVPMatrix(Shader& shader);

    void renderImGui(GravitySimulator* linkedSim);

    void renderSimulatorObjects(GravitySimulator* simulator, Shader& shader);

    void renderTrails(GravitySimulator* simulator, Shader& shader);

    void renderTrailsLines(GravitySimulator* simulator, Shader& shader);

    void renderCircle(const Shader& shader);

    void Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader, GLFWwindow* window);

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
};
