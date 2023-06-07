# A simple Verlet integration based physics simulator

`physics.h` is the physics simulator, implemented as a single header file library. 
It has to be compiled with `-lm` under gcc.
<!--[Explanation of the code on GitHub pages](http://10maurycy10.github.io/tutorials/a_super_simple_physics_engine/)-->

This repository also includes a few simulations with a basic UI and renderer.

- `rope.c` Simulates rope made out of descrete particles attached together using constrants.

- `bowl.c` Simulates a bunch of circles bounded inside of a radius around the origin.

If using gcc, compile with `gcc [FILE] -lm -lSDL2` and run `a.out`.
