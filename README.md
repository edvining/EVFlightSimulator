# Echo Victor Flight Simulator

**Echo Victor Flight Simulator** is a high-performance spaceflight and gravitational simulation project developed in **C++**, with a focus on physical accuracy, numerical stability, and efficient use of modern hardware.

The project aims to deliver a realistic simulation environment while maintaining strong performance characteristics through careful architectural and threading decisions.

## Project Goals

- High-fidelity physics simulation suitable for large scale spaceflight 
- Deterministic, numerically stable integration over large spatial and temporal scales  
- Efficient multi-threaded architecture separating physics and rendering workloads  
- Modern graphics pipeline using OpenGL, with future extensibility toward Vulkan

## Installation

- Clone Repository
- Use Visual Studio with C++ CMake for Windows to Open Folder
- Allow CMake to compile the project
- Click Run

## Technology Stack

- **Language:** C++  
- **Graphics API:** OpenGL (Khronos)  
- **Math Precision:** Double-precision floating point  
- **Concurrency:** Multi-threaded (physics and rendering separation)

## Graphics Backend

### OpenGL

The current implementation uses **OpenGL** to enable rapid development and iteration while retaining strong performance characteristics. OpenGL provides direct GPU access with significantly lower implementation complexity than Vulkan.

### Vulkan (Planned)

Vulkan support is being considered for future versions once the simulation core is sufficiently mature. Vulkan's explicit API design offers greater control and performance potential at the cost of substantially increased complexity.

## System Architecture

The simulator is structured around two primary subsystems:

| Subsystem | Description |
|---------|------------|
| **Gravity Simulator** | Handles gravity calculations, collision detection, and physics integration |
| **Renderer** | Responsible for visual output, camera control, and GPU interaction |

## Multithreading Model

The ```GravitySimulator``` runs on a dedicated physics thread and operates continuously at maximum speed, independent of the rendering frame rate.

The ```Renderer``` runs on the main thread and synchronises with the physics system by locking shared position mutexes. This allows the renderer to sample a consistent snapshot of the simulation state at a fixed timestep.

This design prevents visual artefacts such as jitter when simulating high-velocity objects in close proximity.

## Physics System

### PhysicsObject

`PhysicsObject` is the foundational data structure representing any physical entity in the simulation.

#### Core Properties

| Property | Description | Type |
|--------|------------|------|
| Mass | Object mass | `double` |
| Radius | Collision radius | `double` |
| Position | World-space position | `triple` |
| Velocity | Linear velocity | `triple` |

**Current Limitation:**  
- All physics objects are presently modelled as spheres. Support for mesh-based geometry and more advanced collision models is planned.

## Mathematical Types

### `triple`

`triple` is a custom three-dimensional vector type implemented using **double-precision floating point** values.

It is used throughout the simulation for:
- Position and displacement vectors  
- Velocity and acceleration  
- Force and momentum calculations  

Double precision is required to maintain accuracy across large spatial scales and long-duration simulations, particularly for orbital mechanics.

Future plans include adding multiple local inertial reference frames such as star systems so that enormous distances can be calculated for.

## Design Principles

- **Performance First:** Avoid unnecessary abstraction in performance-critical systems  
- **Determinism:** Stable, reproducible physics behaviour  
- **Scalability:** Designed to support increasing object counts and simulation complexity  
- **Extensibility:** Architecture structured to accommodate future rendering and physics upgrades  

## Roadmap

- Mesh-based collision and object representation  
- Atmospheric lift and drag calculations
- Optional Vulkan rendering backend  
- Data-oriented refactoring for large-scale simulations

## Author

**Edward Vining**  
Flight simulation enthusiast and C++ developer  

## License

No license information as of yet.
