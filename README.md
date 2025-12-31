# Echo Victor Flight Simulator

This is the start of the OpenGL Spaceflight / Flight Simulator made by me, Edward Vining (aka Echo Victor).

I am aiming to make this a highly realistic, very performant, OpenGL / Vulkan application, written in C++. 

This allows me to take full advantage of the computer hardware, as C++ is as close to directly coding the hardware itself.

Khronos OpenGL (Open Graphics Library) is an Open-Source graphics library, enabling direct rendering to the screen, and whilst not as verbose as Vulkan, still enables very good levels of performance.

Khronos Vulkan is also an Open-Source graphics library, with almost everything exposed to the user, making it incredibly verbose to write, but enabling better performance capabilities.

I have decided to make this project using OpenGL as it is much easier to write and create a working prototype faster.

## Structure

This simulator has two main objects, the ```GravitySimulator``` object and ```Renderer``` object.

The ```GravitySimulator``` object handles all the gravity, collision, and physics calculations, while the ```Renderer``` object handles all of the (you guessed it), Rendering calculations.

By design, the ```GravitySimulator``` runs on a completely separate thread from the main rendering thread, and works at maximum speed all the time, no matter what the rest of the threads are doing. The renderer thread can lock some of the mutexes in the Gravity Simulator, so that it can render the objects as they were at one timeframe, rather than the objects moving during frame creation. This stops objects "jittering" when at high velocity in close proximity with other objects.

```PhysicsObject``` is the base object used by the gravity simulator, and describes parameters of each object in the scene.
Basic Parameters:
```m``` - Mass
```r``` - Radius
```p``` - Position
```v``` - Velocity

For this initial implementation, all objects are spheres, but this will change to allow meshes to be imported as objects shapes.

Positions and Velocities are Double Precision Floating Point vectors, implemented as ```Vector3D```
