# A simple 2d Verlet integration based physics simulator

`physics.h` is the physics simulator, implemented as a single header file library. 
It has to be compiled with `-lm` under gcc.
<!--[Explanation of the code on GitHub pages](http://10maurycy10.github.io/tutorials/a_super_simple_physics_engine/)-->

This repository also includes a few simulations with a minimal UI and renderer.

- `stress-test.c` fills a box with objects to test optimizations of the engine.

- `rope.c` Simulates rope made out of discrete objects attached together using constraints. Click to add an object.

- `bowl.c` Simulates a bunch of circles bounded inside of a radius around the origin. Click to add an object.

If using gcc, compile with `gcc [FILE] -lm -lSDL2` and run `a.out`.

To use this in your own code, just copy over `physics.h` (`physcis_optimized.h` if you want the optimized solver) and include it in your program.
See the comments in the header files for information on usage.

## Verlet integration

Verlet integration is based on a simple principles: Instead of tracking position and velocity, track position and the position at the last timestep.
The biggest advantage of this is it makes applying constraints and handling collisions super easy.
For basic inelastic collisions between objects of the same mass, the algorithm can be summed up as: If the objects are intersecting, move them away from each other until they are not.
This is what is implemented in the `physics.h`'s `world_collide` and `physics_optimized.h`'s `world_optimized_collide` functions.

You can also apply other constraints, like limiting objects distances from each other, allowing rope or cloth simulations.
These constraints can also be made less stiff (unimplemented) to make soft bodies more soft.

