Features
================================================================================

At the moment this page lists both existing features and *features under development*.

### Objects
 - Procedural approach to defining vessel models
 - Each vessel can be a hierarchic tree of objects of any size
 - Each object provides additional behavior to the vessel
 - All objects can have mass defined and take part in mass-related calculations
 - Moments of inertia and mechanical properties calculated from object geometry
 - Some objects have predefined parametric geometry

### Coordinate Frames
 - Support for cartesian coordinate frames in every object
 - Automatic transformation between non-inertial and inertial coordinate frames
 - The following coordinate frames are supported:
    * Any body-relative coordinates
    * LVLH (local vertical, local horizontal)
    * Geodetic (any celestial body or vessel)
 - Additionally the following special coordinate frames can be added:
    * *Barycenter-relative coordinates*
    
### Simulation
 - 6-DOF (degrees of freedom) simulation
 - Realtime and non-realtime
 - Many different propagators can be used at once
 - Numerical propagators available:
    * Heuns predictor-corrector method
    * Runge-Kutta 4th order
    * Forward Euler integration (for debugging purposes)
    * *Bullet physics engine propagator* (local coordinate frame propagator for simulating collisions)
 - Analytical propagators available:
    * *Work in progress*
 - Automatic transition between different coordinate systems for best numerical precision
 - Forces and torques generated from vessel objects and other bodies
 - Support for approximate collision detection via Bullet physics propagator
 
### Models
 - *Custom aerodynamic drag model based on body shape*
 - Newton gravity model with J2 coefficient correction
 - Atmospheric models:
    * Exponential atmosphere
    * NRLMSISE-00
    
### Additional Features
 - Can be configured for less degrees of freedom
 - Modifiers to automatically generate arrays of similar objects
 - Vessels/objects can be referenced across files (imported from external file)
 - Built-in tesselator to generate meshes for procedural models (for rendering,
   finite element models, etc)
 - @ref EVDS_Object_Types "List of common objects available in EVDS".
 - @ref EVDS_Addon_List "List of optional official addons for EVDS".
