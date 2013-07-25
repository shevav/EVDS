notitle {#mainpage}
================================================================================

EVDS is an open-source aerospace physics simulation library (flight dynamics library, FDM)
written in C. It's designed for realtime and non-realtime, multithreaded simulation
of vessel dynamics. In the context of this library, "vessel" means any object to be
simulated, for example:
 - Autonomous spacecraft (satellites)
 - Manned spacecraft
 - Aircraft
 - Trains, vehicles
 
Conceptually, EVDS allows user to construct a set of equations for the entire
system from small basic equations (e.g. equations for numerical integration,
equations for aerodynamic forces). The equations are constructed from hierarchy
of objects, as defined by user.

Every object in EVDS has a coordinate system attached, and any vectors may be
specified in the objects coordinate system. Runtime conversion is performed
between vector coordinate systems.

A procedural approach to defining models is used: "it looks how it behaves".
The vessels are constructed from small pieces, each of which provides a contribution
to the total forces and torques or vessel behavior.

Every object contains a set of variables (floating-point, string, vector, quaternion
variables, or custom data with a pointer to arbitrary data structure). Custom
data variables may also contain nested attributes and variables (used for
storing complex structures in data files). A set of modifiers may be specified,
which create patterns/copies of objects according to given rules.

Vessel configuration can be defined in runtime by the program, or loaded from
an XML file.

EVDS does not contain a rendering engine, but provides facilities to generate
3D meshes and model data for rendering. The physics engine is designed for high-perfomance
multi-threaded environment with support for asynchronous rendering in mind.

EVDS contains a powerful vector math library which provides consistency checks
across different coordinate frames and allows correct transformation of vectors
between non-inertial coordinate frames.

Target Use
--------------------------------------------------------------------------------
EVDS was designed to be used in a realtime aerospace flight simulator, but
it can also be used for:
 - Quick first-order vessel parameter & motion estimation
 - Flight simulation
 - Physics calculations in embedded systems
 - Non-realtime simulation
 - Tessellation of 3D vessel models
 
The main version of EVDS is 64-bit, but it is also available in a 32-bit version.
EVDS supports the following platforms at the moment:
 - Windows (32-bit and 64-bit)
 - Linux (32-bit and 64-bit)
