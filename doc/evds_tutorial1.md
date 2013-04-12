Tutorial #1: 6-DOF Earth Satellite Model
================================================================================

General
--------------------------------------------------------------------------------
This tutorial shows how to create a simple 6-DOF simulation of a satellites
motion in Earth low orbit, and how to extend this simulation to include Moon.

No additional effects will be accounted for (no drag, no solar drag, no
additional gravity sources beyond Moon).

Only the basic C API will be used in this tutorial.




Setting Up
--------------------------------------------------------------------------------
@todo More information on how to create project and include EVDS libraries

These are the initial contents of "main.c". It simply initializes EVDS system,
and then deinitializes it. For clarity, all error checking has been omitted.

~~~~~~~~~~~~~~~{.c}
#include <stdio.h>
#include <math.h>
#include "evds.h"

void main() {
    EVDS_SYSTEM* system;

    printf("Tutorial 1: 6-DOF Earth Satellite Model\n");
    EVDS_System_Create(&system);
    EVDS_Common_Register(system);

    // ... do work ...

    EVDS_System_Destroy(system);
}
~~~~~~~~~~~~~~~

The EVDS_Common_Register() macro will initialize all object types from common
solvers. This will be required for actually doing useful things with EVDS.




Creating the Planet
--------------------------------------------------------------------------------
First, a root object must be created - an inertial coordinate system in which all
motion will be computed. Two objects will be declared:
~~~~~~~~~~~~~~~{.c}
EVDS_OBJECT* inertial_system;
EVDS_OBJECT* earth;
~~~~~~~~~~~~~~~

Creating inertial system is fairly straightforward. The root inertial coordinate
system will be propagating state of the simulated satellite, so its type
must be a propagator. Fourth order Runge-Kutta propagator will provide satisfying
precision:
~~~~~~~~~~~~~~~{.c}
EVDS_Object_Create(system,0,&inertial_system);
EVDS_Object_SetType(inertial_system,"propagator_rk4");
EVDS_Object_Initialize(inertial_system,1);
~~~~~~~~~~~~~~~

Inertial system will be initialized right away (blocking initialization - the
application will wait until object has finished initializing before continuing).
_propagator_rk4_ is the object type corresponding to Runge-Kutta 4th order numerical
integration. Root coordinate system does not require a parent object.

Creating planet Earth is very similar, but some additional properties will be
specified:
Parameter  | Value
-----------|-------------------------
\f$\mu\f$  | \f$ 3.9860044 \cdot 10^{14} \ m^3 sec^{-2}\f$
Radius     | \f$ 6378.145 \ km\f$

Code to do this:
~~~~~~~~~~~~~~~{.c}
EVDS_Object_Create(system,inertial_system,&earth);
EVDS_Object_SetType(earth,"planet");
EVDS_Object_SetName(earth,"Earth");
EVDS_Object_AddFloatVariable(earth,"mu",3.9860044e14,0);    //m3 sec-2
EVDS_Object_AddFloatVariable(earth,"radius",6378.145e3,0);  //m
EVDS_Object_SetPosition(earth,inertial_system,0,0,0);
EVDS_Object_Initialize(earth,1);
~~~~~~~~~~~~~~~

Earth center will be located in the center of inertial coordinate system. The
simulator will additionally create coordinate systems inside planet Earth, but
for this tutorial those can be ignored.

In EVDS a _planet_ type object represents the planet and its _location_ in space
in general. Planet object automatically sets rotation rate of the planets local
non-inertial coordinate system (tied to planets surface, not used in this tutorial).

The simulation source code so far:
~~~~~~~~~~~~~~~{.c}
#include <stdio.h>
#include <math.h>
#include "evds.h"

void main() {
    EVDS_SYSTEM* system;
    EVDS_OBJECT* inertial_system;
    EVDS_OBJECT* earth;

    printf("Tutorial 1: 6-DOF Earth Satellite Model\n");
    EVDS_System_Create(&system);
    EVDS_Common_Register(system);

    //Create inertial sysetm
    EVDS_Object_Create(system,0,&inertial_system);
    EVDS_Object_SetType(inertial_system,"propagator_rk4");
    EVDS_Object_Initialize(inertial_system,1);

    //Create planet Earth
    EVDS_Object_Create(system,inertial_system,&earth);
    EVDS_Object_SetType(earth,"planet");
    EVDS_Object_SetName(earth,"Earth");
    EVDS_Object_AddFloatVariable(earth,"mu",3.9860044e14,0);    //m3 sec-2
    EVDS_Object_AddFloatVariable(earth,"radius",6378.145e3,0);  //m
    EVDS_Object_SetPosition(earth,inertial_system,0,0,0);
    EVDS_Object_Initialize(earth,1);

    EVDS_System_Destroy(system);
}
~~~~~~~~~~~~~~~




Loading Satellite Data
--------------------------------------------------------------------------------
Just as before, define an object for the satellite:
~~~~~~~~~~~~~~~{.c}
EVDS_OBJECT* satellite;
~~~~~~~~~~~~~~~

The satellite will use a very simple XML configuration. These are the physics
parameters of the satellite (mass, moments of inertia, center of mass), and
initial conditions of the simulation (in inertial coordinates):
Parameter  | Value
-----------|-------------------------
Mass       | \f$ 1000 \ kg\f$
Ixx        | \f$ 100  \ N \cdot m\f$
Iyy        | \f$ 1000 \ N \cdot m\f$
Izz        | \f$ 500  \ N \cdot m\f$
Mx         | \f$ +1.0 \ m \f$
My         | \f$ +0.0 \ m \f$
Mz         | \f$ -0.5 \ m \f$
X          | \f$ 6728.0 \ km \f$ (350 km above mean Earth radius)
Y          | \f$ 0.0 \ km \f$
Z          | \f$ 0.0 \ km \f$
VX         | \f$ 0.0 \ m/s \f$
VY         | \f$ 7700.0 \ m/s \f$ (circular orbit)
VZ         | \f$ 0.0 \ m/s \f$

All specified coordinates are in local coordinates of the satellite, around the
reference point (arbitrary point, whose coordinates are zero). EVDS uses a
right handed coordinate system, so in local coordinates:
 - X axis: positive aft, negative forward (\f$+1.0 \ m\f$ is located 1 meter _behind_
   the reference point)
 - Y axis: positive right, negative left (looking from aft to forward)
 - Z axis: positive up, negative down
 
This is the final look of XML configuration file:
~~~~~~~~~~~~~~~{.c}
<EVDS>
  <object type="vessel" name="Satellite">
    <parameter name="mass">1000</parameter>
    <parameter name="ixx">100</parameter>
    <parameter name="iyy">1000</parameter>
    <parameter name="izz">500</parameter>
    <parameter name="cm" vtype="position">1.0 0.0 -0.5</parameter>
  </object>
</EVDS>
~~~~~~~~~~~~~~~

It can be loaded using the EVDS_Object_LoadFromString() call (which accepts
any XML configuration in UTF-8 encoding). The object must be additionally
initialized, just like the objects before. EVDS_Object_SetPosition() and
EVDS_Object_SetVelocity() API calls can be used to specify initial conditions.

The full code now looks like this:
~~~~~~~~~~~~~~~{.c}
#include <stdio.h>
#include <math.h>
#include "evds.h"

void main() {
    EVDS_SYSTEM* system;
    EVDS_OBJECT* inertial_system;
    EVDS_OBJECT* earth;

    printf("Tutorial 1: 6-DOF Earth Satellite Model\n");
    EVDS_System_Create(&system);
    EVDS_Common_Register(system);

    //Create inertial sysetm
    EVDS_Object_Create(system,0,&inertial_system);
    EVDS_Object_SetType(inertial_system,"propagator_rk4");
    EVDS_Object_Initialize(inertial_system,1);

    //Create planet Earth
    EVDS_Object_Create(system,inertial_system,&earth);
    EVDS_Object_SetType(earth,"planet");
    EVDS_Object_SetName(earth,"Earth");
    EVDS_Object_AddFloatVariable(earth,"mu",3.9860044e14,0);    //m3 sec-2
    EVDS_Object_AddFloatVariable(earth,"radius",6378.145e3,0);  //m
    EVDS_Object_SetPosition(earth,inertial_system,0,0,0);
    EVDS_Object_Initialize(earth,1);

    //Load satellite
    EVDS_Object_LoadFromString(inertial_system,
"<EVDS>"
"  <object type=\"vessel\" name=\"Satellite\">"
"    <parameter name=\"mass\">1000</parameter>"
"    <parameter name=\"ixx\">100</parameter>"
"    <parameter name=\"iyy\">1000</parameter>"
"    <parameter name=\"izz\">500</parameter>"
"    <parameter name=\"cm\">1.0 0.0 -0.5</parameter>"
"  </object>"
"</EVDS>",&satellite);
    EVDS_Object_SetPosition(satellite,inertial_system,6728e3,0,0);
    EVDS_Object_SetVelocity(satellite,inertial_system,0,7700,0);
    EVDS_Object_Initialize(satellite,1);

    EVDS_System_Destroy(system);
}
~~~~~~~~~~~~~~~




Adding Simulation Loop, Printing Results
--------------------------------------------------------------------------------
The simulator will simply run an infinite loop, modelling the flight with a timestep
of 1.0 seconds, 50 steps per one real second.
Basic timing can be provided with EVDS_Thread_Sleep() function, which takes number
of seconds to wait as a parameter.

To propagate the trajectory, propagation object must be solver (in case of this
tutorial - _inertial_system_ object):
~~~~~~~~~~~~~~~{.c}
EVDS_Object_Solve(inertial_system,1.0); //One second time step
~~~~~~~~~~~~~~~

Printing object state requires a copy of a state vector, which can be used in any
way later. Altitude above surface and velocity are computed from state vector:
~~~~~~~~~~~~~~~{.c}
EVDS_STATE_VECTOR state;
EVDS_REAL radius2,velocity2;
EVDS_Object_GetStateVector(satellite,&state);
EVDS_Vector_Dot(&radius2,&state.position,&state.position);
EVDS_Vector_Dot(&velocity2,&state.velocity,&state.velocity);
printf("MJD: %.5f  [%.0f %.0f] H = %.3f km  [%.0f %.0f] V = %.3f m/s]\r",
    state.time,
    state.position.x*1e-3,state.position.y*1e-3,sqrt(radius2)*1e-3-6378.0,
    state.velocity.x,state.velocity.y,sqrt(velocity2));
~~~~~~~~~~~~~~~
State vector is always specified and returned in the coordinates of parent object -
in this case that's the propagator (inertial system).

This is the final look of this simple simulator:
\include evds_tutorial1.c




Exercise: Output Simulation Results to File
--------------------------------------------------------------------------------
Output the satellites trajectory to file. This is the expected plot of satellites
altitude above mean Earth surface (\f$6378.0 \ km\f$) over time:
\image html tutorial1_radius1.png





Exercise: Adding Moon
--------------------------------------------------------------------------------
Add moon as a second "planet", with the following parameters:
Parameter  | Value
-----------|-------------------------
\f$\mu\f$  | \f$ 0.0490277 \cdot 10^{14} \ m^3 sec^{-2}\f$
Radius     | \f$ 1737.0 \ km\f$
X          | \f$ 0.0 \ km\f$
Y          | \f$ 362\thinspace 570 \ km\f$
Z          | \f$ 0.0 \ km\f$

Plot of satellites altitude above mean Earth surface, with Moon gravity influence:
\image html tutorial1_radius2.png




Exercise: Running Physics Simulation in the Second Thread
--------------------------------------------------------------------------------
EVDS provides basic threading support through EVDS threading library. Here's an
example that creates an extra thread to print current simulation state:
~~~~~~~~~~~~~~~{.c}
void print_status_thread(EVDS_OBJECT* object) {
    while (1) {
        EVDS_STATE_VECTOR state;
        EVDS_Object_GetStateVector(object,&state);
        printf("x %f y %f  vx %f vy %f\r",
            state.position.x,state.position.y,
            state.velocity.x,state.velocity.y);
        EVDS_Thread_Sleep(0.1);
    }
}
~~~~~~~~~~~~~~~

To create a thread, a function pointer and an arbitrary user variable (of a
pointer type) must be passed into EVDS_Thread_Create():
~~~~~~~~~~~~~~~{.c}
EVDS_Thread_Create(print_status_thread,satellite);
~~~~~~~~~~~~~~~

Create a thread to run the main simulation loop (with status being
printed in the main thread). The simulation will run in parallel, any state reads
will be completed correctly through the thread-safe synchronization of
EVDS_Object_GetStateVector().
