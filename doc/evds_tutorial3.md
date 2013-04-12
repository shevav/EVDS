Tutorial #3: Rendering Gyroscope with Precession
================================================================================

General
--------------------------------------------------------------------------------
This tutorial will show how to create basic OpenGL rendering for a spinning gyroscope
in weightless conditions.

EVDS vessel simulation implements full rigid body physics support for combined
bodies. In this tutorial torque-free precession of an unballanced gyroscope
will be demonstrated.

Only the basic C API will be used in this tutorial. Setting up OpenGL window and
drawing the object meshes will not be covered by this tutorial. GLFW library
is be used for setting up the window. GLU functions are used for setting up the
view.




Setting Up
--------------------------------------------------------------------------------
Working with OpenGL is not the focus of this tutorial, so these helper functions
will be used, but not explained (GLFW is used for setting up the window):
Signature                                                    | Description
-------------------------------------------------------------|------------------
void glfw_init()                                             | Initialize GLFW window
void glfw_setup_3d()                                         | Setup 3D rendering
void glfw_draw_cube(float width, float length, float height) | Draw a cube with given size around current origin

...





Drawing Objects
--------------------------------------------------------------------------------
...

This is the final source code of this tutorial (OpenGL code included):
\include evds_tutorial3.c

The program outputs image similar to this:
\image html tutorial3_demo.png




Exercise: Determine Precession Rate
--------------------------------------------------------------------------------
