////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2013, Black Phoenix
///
/// This program is free software; you can redistribute it and/or modify it under
/// the terms of the GNU Lesser General Public License as published by the Free Software
/// Foundation; either version 2 of the License, or (at your option) any later
/// version.
///
/// This program is distributed in the hope that it will be useful, but WITHOUT
/// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
/// FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
/// details.
///
/// You should have received a copy of the GNU Lesser General Public License along with
/// this program; if not, write to the Free Software Foundation, Inc., 59 Temple
/// Place - Suite 330, Boston, MA  02111-1307, USA.
///
/// Further information about the GNU Lesser General Public License can also be found on
/// the world wide web at http://www.gnu.org.
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Convert position vector into geographic coordinates of the object.
///
/// This function converts a position vector to geographic coordinates relative to the object.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_ToGeographicCoordinates(EVDS_OBJECT* object, EVDS_VECTOR* v,
										 EVDS_GEODETIC_COORDIANTE* coordinate) {
	EVDS_REAL x,y,z,r;
	if (!object) return;

	//Get vector in target coordinates
	EVDS_Vector_Get(v,&x,&y,&z,object);
	r = sqrt(x*x+y*y+z*z)+EVDS_EPS;

	//Write back geographic coordinates
	if (EVDS_Object_CheckType(object,"planet") == EVDS_OK) {
		EVDS_VARIABLE* radius_var = 0;
		EVDS_REAL radius = 0;

		//Read planet radius
		if (EVDS_Object_GetVariable(object,"radius",&radius_var) == EVDS_OK) {
			EVDS_Variable_GetReal(radius_var,&radius);
		}
		//Read planet flattening to output coordinates over ellipsoid
		//FIXME

		//Output coordinates
		coordinate->longitude		= EVDS_DEG(atan2(y,x));
		coordinate->latitude		= EVDS_DEG(asin(z/r));
		coordinate->mean_elevation  = r - radius;
	} else {
		coordinate->longitude		= EVDS_DEG(atan2(y,x));
		coordinate->latitude		= EVDS_DEG(asin(z/r));
		coordinate->mean_elevation	= r;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert geographic coordinates of the object to a position vector.
///
/// This function converts geographic coordinates relative to the object to a position vector.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_FromGeographicCoordinates(EVDS_OBJECT* object, EVDS_VECTOR* target,
										   EVDS_GEODETIC_COORDIANTE* coordinate) {
	EVDS_REAL x,y,z,r;
	if (!object) return;

	//Convert geographic coordinates to x,y,z
	if (EVDS_Object_CheckType(object,"planet") == EVDS_OK) {
		EVDS_VARIABLE* radius_var = 0;
		EVDS_REAL radius = 0;

		//Read planet radius
		if (EVDS_Object_GetVariable(object,"radius",&radius_var) == EVDS_OK) {
			EVDS_Variable_GetReal(radius_var,&radius);
		}
		//Read planet flattening to output coordinates over ellipsoid
		//FIXME

		//Output coordinates
		r = coordinate->mean_elevation + radius;
		x = r*cos(EVDS_RAD(coordinate->longitude))*cos(EVDS_RAD(coordinate->latitude));
		y = r*sin(EVDS_RAD(coordinate->longitude))*cos(EVDS_RAD(coordinate->latitude));
		z = r*sin(EVDS_RAD(coordinate->latitude));
	} else {
		r = coordinate->mean_elevation;
		x = r*cos(EVDS_RAD(coordinate->longitude))*cos(EVDS_RAD(coordinate->latitude));
		y = r*sin(EVDS_RAD(coordinate->longitude))*cos(EVDS_RAD(coordinate->latitude));
		z = r*sin(EVDS_RAD(coordinate->latitude));
	}
	
	//Set vector in target coordinates
	EVDS_Vector_Set(target,EVDS_VECTOR_POSITION,object,x,y,z);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion to objects LVLH frame.
///
/// Returns quaternion in objects coordinate frame (although the quaternion represents
/// attitude in LVLH coordinate system).
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_ToLVLHCoordinates(EVDS_OBJECT* object, EVDS_QUATERNION* q_lvlh, EVDS_QUATERNION* q,
									   EVDS_GEODETIC_COORDIANTE* coordinate) {
	EVDS_QUATERNION q_lon,q_lat; //Rotations to go from North pole to given latitude/longitude
	EVDS_Quaternion_FromEuler(&q_lon,object,0,0,EVDS_RAD(coordinate->longitude));
	EVDS_Quaternion_FromEuler(&q_lat,object,0,EVDS_RAD(90-coordinate->latitude),0);

	//Rotate quaternion Q from inertial to LVLH coordinates
	EVDS_Quaternion_Convert(q_lvlh,q,object);
	EVDS_Quaternion_MultiplyConjugatedQ(q_lvlh,&q_lon,q_lvlh);
	EVDS_Quaternion_MultiplyConjugatedQ(q_lvlh,&q_lat,q_lvlh);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion from objects LVLH frame.
///
/// Returns quaternion in objects coordinate frame.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_FromLVLHCoordinates(EVDS_OBJECT* object, EVDS_QUATERNION* q_lvlh, EVDS_QUATERNION* q,
										 EVDS_GEODETIC_COORDIANTE* coordinate) {
	EVDS_QUATERNION q_lon,q_lat; //Rotations to go from North pole to given latitude/longitude
	EVDS_Quaternion_FromEuler(&q_lon,object,0,0,EVDS_RAD(coordinate->longitude));
	EVDS_Quaternion_FromEuler(&q_lat,object,0,EVDS_RAD(90-coordinate->latitude),0);

	//Rotate quaternion Q from LVLH to inertial coordinates
	EVDS_Quaternion_Convert(q_lvlh,q,object);
	EVDS_Quaternion_Multiply(q_lvlh,&q_lat,q_lvlh);
	EVDS_Quaternion_Multiply(q_lvlh,&q_lon,q_lvlh);
}
