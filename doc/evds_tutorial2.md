Tutorial #2: Custom RK2 Propagator
================================================================================

General
--------------------------------------------------------------------------------
This tutorial shows how to create a new propagator for the EVDS simulator. It will
use the simulator from the previous tutorial as a simple reference simulator.

An EVDS propagator is what implements numerical integration algorithm to calculate
new state of vehicle in space. In this case, a numerical integration method called
*2nd order Runge-Kutta* will be implemented.

Only the basic C API will be used in this tutorial.



Basic math
--------------------------------------------------------------------------------
\f{eqnarray*}{
  f1 &=& h f(t_n,y_n) \\
  f2 &=& h f(t_n + \frac{1}{2} h,y_n + \frac{1}{2} k1) \\
  y_{n+1} &=& y_n + f2
\f}



Setting Up
--------------------------------------------------------------------------------
Refer to tutorial #1 for more information on setting up the project. This tutorial
adds an additional C ("rk2.c") and an additional header ("rk2.h") files.

The new files initially contain a little bit of code, which registers the new
propagator as a new object type within EVDS. The new propagator is implemented
as an object type - the structure used for this in EVDS is called a "solver".

These are the contents of "main.c" (see previous tutorial):
\include evds_tutorial1.c

Initial contents of "rk2.h":
~~~~~~~~~~~~~~~{.c}
TEST
~~~~~~~~~~~~~~~

Initial contents of "rk2.c":
~~~~~~~~~~~~~~~{.c}
TEST
~~~~~~~~~~~~~~~



Registering New Solver
--------------------------------------------------------------------------------
To register new solver (which implements the RK2 propagator), simply call its
register function (already defined in the header file):
~~~~~~~~~~~~~~~{.c}

~~~~~~~~~~~~~~~

For every object type, each solver will be presented with the object. Object
will be handled by the solver that has claimed ownership of it. In this tutorial,
the RK2 propagator solver must claim objects with _propagator_rk2_ defined
as its type.

Initialization callback is called for every solver, for every object. This is how
it will be implemented in this tutorial:
~~~~~~~~~~~~~~~{.c}

~~~~~~~~~~~~~~~

This callback simply checks if object is fitting, and rejects it if it's not the
expected propagator object.



Writing Code
--------------------------------------------------------------------------------
Now the RK2 method must be implemented itself.
