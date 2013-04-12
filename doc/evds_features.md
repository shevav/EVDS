Features
================================================================================

Simulation features:
 - 6-DOF (degrees of freedom) simulation
 - Realtime and non-realtime
 - Support for different propagators (numerical integrators) integrating state at once
 - Procedural approach to defining models
 - Calculates vessel trajectories from a set of forces and a set of torques
 - Automatic transition between different coordinate systems for best numerical precision
 - Automatic runtime conversion between coordinate systems (vector component values
   are always specified in some coordinate system)

 
Additional features:
 - Can be configured for less degrees of freedom
 - Support for approximate collision detection via Bullet physics propagator
 - Modifiers to automatically generate arrays of similar objects
 - Vessels/objects can be referenced across files (imported from external file)
 - Built-in tesselator to generate meshes for procedural models (for rendering,
   finite element models, etc)
 - @ref EVDS_Objects "List of common objects available in EVDS".
 - @ref EVDS_Addons "List of optional official addons for EVDS".
