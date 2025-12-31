cmake_minimum_required(VERSION 3.15)
project(EVFlightSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(USE_SYSTEM_GLEW "Try find_package(GLEW) and use system GLEW if available" ON)
option(USE_SYSTEM_GLFW "Try find_package(glfw3) and use system GLFW if available" ON)

# Collect project sources
file(GLOB_RECURSE EVFS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/source/*.cpp"
    "${CMAKE_SOURCE_DIR}/source/*.cxx"
    "${CMAKE_SOURCE_DIR}/source/*.cc"
)

# ---- Subprojects / bundled libs ----
add_subdirectory(resource/glm)

# ImGui (build from bundled sources)
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/resource/imgui")
file(GLOB IMGUI_SOURCES
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_demo.cpp"
    "${IMGUI_DIR}/imgui_impl_glfw.cpp"
    "${IMGUI_DIR}/imgui_impl_opengl3.cpp"
)
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${IMGUI_DIR})
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)

# ---- Find system libraries (OpenGL, GLEW, GLFW) ----
find_package(OpenGL REQUIRED)

# --- GLEW: prefer find_package, then try to find prebuilt libs under resource, otherwise fail with instructions
if(USE_SYSTEM_GLEW)
    find_package(GLEW QUIET)
endif()

set(GLEW_TARGET "")
if(TARGET GLEW::GLEW)
    set(GLEW_TARGET GLEW::GLEW)
elseif(GLEW_FOUND)
    # older FindGLEW variant exposed variables; create imported wrapper
    add_library(glew_imported UNKNOWN IMPORTED)
    set_target_properties(glew_imported PROPERTIES
        IMPORTED_LOCATION "${GLEW_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GLEW_INCLUDE_DIRS}"
    )
    set(GLEW_TARGET glew_imported)
else()
    # try to find prebuilt libs in repo under resource/GLEW/lib or lib64
    find_library(GLEW_LIB NAMES glew32s glew32
        PATHS
            "${CMAKE_SOURCE_DIR}/resource/GLEW/lib"
            "${CMAKE_SOURCE_DIR}/resource/GLEW/lib64"
            "${CMAKE_SOURCE_DIR}/resource/GLEW/Lib"
            "${CMAKE_SOURCE_DIR}/resource/GLEW/Lib64"
        NO_DEFAULT_PATH
    )
    if(GLEW_LIB)
        add_library(glew_static STATIC IMPORTED)
        set_target_properties(glew_static PROPERTIES
            IMPORTED_LOCATION "${GLEW_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/resource/GLEW/include"
            INTERFACE_COMPILE_DEFINITIONS "GLEW_STATIC"
        )
        set(GLEW_TARGET glew_static)
        message(STATUS "Using bundled GLEW library: ${GLEW_LIB}")
    else()
        message(FATAL_ERROR
"GLEW not found. Options:
 - Install via vcpkg and configure CMake with -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg.cmake (recommended)
   e.g. vcpkg install glew:x64-windows
 - Or provide a built GLEW library in resource/GLEW/lib (glew32s.lib or glew32.lib)
 - Or set variables -DGLEW_LIB=<path> and -DGLEW_INCLUDE=<path> and update CMakeLists accordingly.
See README or run: git clone https://github.com/microsoft/vcpkg.git && .\\vcpkg\\bootstrap-vcpkg.bat && .\\vcpkg\\vcpkg.exe install glew:x64-windows")
    endif()
endif()

# --- GLFW: prefer find_package, then try to find prebuilt libs under resource, otherwise fail with instructions
if(USE_SYSTEM_GLFW)
    find_package(glfw3 QUIET)
endif()

set(GLFW_TARGET "")
if(TARGET glfw)
    set(GLFW_TARGET glfw)
elseif(TARGET glfw3)
    set(GLFW_TARGET glfw3)
elseif(glfw3_FOUND)
    # some Find modules only set glfw3_FOUND and variables; create an imported wrapper if needed
    add_library(glfw3_imported UNKNOWN IMPORTED)
    # If find_package provided variables (GLFW_LIBRARIES/GLFW_INCLUDE_DIRS), you could set properties here.
    set(GLFW_TARGET glfw3_imported)
else()
    # try to find prebuilt glfw lib files in repo under resource/GLFW/lib*
    find_library(GLFW_LIB NAMES glfw3 glfw
        PATHS
            "${CMAKE_SOURCE_DIR}/resource/GLFW/lib"
            "${CMAKE_SOURCE_DIR}/resource/GLFW/lib64"
            "${CMAKE_SOURCE_DIR}/resource/GLFW/Lib"
            "${CMAKE_SOURCE_DIR}/resource/GLFW/Lib64"
        NO_DEFAULT_PATH
    )
    if(GLFW_LIB)
        add_library(glfw3_static STATIC IMPORTED)
        set_target_properties(glfw3_static PROPERTIES
            IMPORTED_LOCATION "${GLFW_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/resource/GLFW/include"
        )
        set(GLFW_TARGET glfw3_static)
        message(STATUS "Using bundled GLFW library: ${GLFW_LIB}")
    else()
        message(FATAL_ERROR
"GLFW not found. Options:
 - Install via vcpkg and configure CMake with -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg.cmake (recommended)
   e.g. vcpkg install glfw3:x64-windows
 - Or provide a built GLFW library in resource/GLFW/lib (glfw3.lib)
 - Or set variables -DGLFW_LIB=<path> and -DGLFW_INCLUDE=<path> and update CMakeLists accordingly.")
    endif()
endif()

# Link imgui to GLFW/GLEW so backend sources can find their headers when building
if(GLEW_TARGET)
    target_link_libraries(imgui PRIVATE ${GLEW_TARGET})
endif()
if(GLFW_TARGET)
    target_link_libraries(imgui PRIVATE ${GLFW_TARGET})
endif()

# ---- Target: EVFlightSimulator ----
add_executable(EVFlightSimulator ${EVFS_SOURCES})

target_include_directories(EVFlightSimulator PRIVATE
    "${CMAKE_SOURCE_DIR}/resource"
    "${CMAKE_SOURCE_DIR}/source"
    "${CMAKE_SOURCE_DIR}/resource/stb_image"
)

# Ensure GLEW static define is visible to compile units (matches your code)
target_compile_definitions(EVFlightSimulator PRIVATE GLEW_STATIC)

# Link third-party targets (glm produces target 'glm' in resource/glm/CMakeLists.txt)
target_link_libraries(EVFlightSimulator PRIVATE
    imgui
    glm
    ${GLEW_TARGET}
    ${GLFW_TARGET}
    OpenGL::GL
)

# Convenience
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/source")

message(STATUS "Configured EVFlightSimulator")
message(STATUS "  - source files: ${EVFS_SOURCES}")
message(STATUS "  - imgui: ${IMGUI_DIR}")
message(STATUS "  - glm: resource/glm (added subdirectory)")