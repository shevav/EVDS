Coordinate Systems
================================================================================

Every object in EVDS has a coordinate system attached to it. All the vectors
are stored in cartesian coordinate system of a specific object, but the can also
be converted to/from geodetic coordinates.


### Cartesian Coordinates

The cartesian coordinate systems used by EVDS are right-handed. The following
coordinate system is suggested for vessels:
\image html cartesian_coordinates.png

Therefore for local coordinates this means that axes point in following directions:
 - `X+` points aft (towards rear)
 - `X-` points forward (towards front)
 - `Y+` points right (towards starboard)
 - `Y-` points left (towards port)
 - `Z+` points up
 - `Z-` points down
 
For celestial body coordinates, the axes point through following geodetic coordinates:
 - `X+` through latitude 0 degrees, longitude 0 degrees
 - `Y+` through latitude 0 degrees, longitude 90 degrees
 - `Z+` through latitude 90
 

### LVLH Coordinates

EVDS LVLH coordinates are derived from cartesian coordinates by rotating the coordinate
system so the `Z+` axis remains normal to planetary body mean spheric surface.
