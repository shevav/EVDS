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
/// @brief Create datum based on the object
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_DatumFromObject(EVDS_GEODETIC_DATUM* datum, EVDS_OBJECT* object) {
	//Default: datum corresponding to bearing/elevation around vessels reference point
	datum->semimajor_axis = 0;
	datum->semiminor_axis = 0;
	datum->object = object;

	//If no object passed, don't try to guess anything
	if (!object) return;

	//Determine datum based on celestial body parameters
	if (EVDS_Object_CheckType(object,"planet") == EVDS_OK) {
		EVDS_VARIABLE* variable;

		//Planet is an ellipsoid
		if (EVDS_Object_GetVariable(object,"geometry.semimajor_axis",&variable) == EVDS_OK) {
			//Presence of semi-major axis defines an ellipsoid
			EVDS_Variable_GetReal(variable,&datum->semimajor_axis);

			if (EVDS_Object_GetVariable(object,"geometry.semiminor_axis",&variable) == EVDS_OK) {
				//Semi-minor axis defined
				EVDS_Variable_GetReal(variable,&datum->semiminor_axis);
			} else if (EVDS_Object_GetVariable(object,"geometry.flattening",&variable) == EVDS_OK) {
				//Flattening is defined
				EVDS_REAL f;
				EVDS_Variable_GetReal(variable,&f);

				datum->semiminor_axis = datum->semimajor_axis * (1 - f);
			} else if (EVDS_Object_GetVariable(object,"geometry.inverse_flattening",&variable) == EVDS_OK) {
				//Inverse of flattening is defined
				EVDS_REAL inv_f;
				EVDS_Variable_GetReal(variable,&inv_f);

				if (inv_f != 0.0) {
					datum->semiminor_axis = datum->semimajor_axis * (1 - 1/inv_f);
				} else {
					datum->semiminor_axis = 0.0;
				}
			} else {
				//No information about ellipsoid shape
				datum->semiminor_axis = datum->semimajor_axis;
			}
		} else if (EVDS_Object_GetVariable(object,"geometry.radius",&variable) == EVDS_OK) {
			//Only radius is defined, use spheric datum
			EVDS_Variable_GetReal(variable,&datum->semimajor_axis);
			datum->semiminor_axis = datum->semimajor_axis;
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set geodetic coordinate around a given body.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_Set(EVDS_GEODETIC_COORDIANTE* coordinate, EVDS_OBJECT* object,
					   EVDS_REAL latitude, EVDS_REAL longitude, EVDS_REAL elevation) {
	coordinate->latitude = latitude;
	coordinate->longitude = longitude;
	coordinate->elevation = elevation;
	EVDS_Geodetic_DatumFromObject(&coordinate->datum,object);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Converts geodetic coordinates around object to a position vector.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_ToVector(EVDS_VECTOR* target, EVDS_GEODETIC_COORDIANTE* source) {
	EVDS_REAL x,y,z; //Components of the result
	EVDS_REAL eccentricity_squared,normal;
	EVDS_REAL sin_lat,cos_lat,sin_lon,cos_lon;
	if (!target) return;
	if (!source) return;

	//Perform some self-checks
	EVDS_ASSERT(source->datum.object);

	//Calculate sines and cosines
	sin_lat = sin(EVDS_RAD(source->latitude));
	cos_lat = cos(EVDS_RAD(source->latitude));
	sin_lon = sin(EVDS_RAD(source->longitude));
	cos_lon = cos(EVDS_RAD(source->longitude));
	
	//Calculate eccentricity of ellipsoid
	// The 'if' excludes invalid inputs and optimizes calculation for spheres
	if (source->datum.semiminor_axis < source->datum.semimajor_axis) {
		eccentricity_squared = 1 - 
			(source->datum.semiminor_axis*source->datum.semiminor_axis)/
			(source->datum.semimajor_axis*source->datum.semimajor_axis);
	} else {
		eccentricity_squared = 0.0;
	}

	//Calculate normal
	if (eccentricity_squared > 0.0) {
		normal = source->datum.semimajor_axis / sqrt(1 - eccentricity_squared * sin_lat * sin_lat);
	} else {
		normal = source->datum.semimajor_axis;
	}

	//Convert geographic coordinates to X,Y,Z
	x = (normal + source->elevation)*cos_lon*cos_lat;
	y = (normal + source->elevation)*sin_lon*cos_lat;
	z = (normal*(1 - eccentricity_squared) + source->elevation)*sin_lat;

	//Set vector in target coordinates
	EVDS_Vector_Set(target,EVDS_VECTOR_POSITION,source->datum.object,x,y,z);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Converts position vector into geodetic coordinate.
///
/// @note The geodetic coordinate will be calculated around the body, in which
///       the source vector is specified. EVDS_Vector_Convert() call is required
///       to get geodetic coordinates around a specific body, or alternatively
///       the target datum may be specified around a specific object.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_FromVector(EVDS_GEODETIC_COORDIANTE* target, EVDS_VECTOR* source, EVDS_GEODETIC_DATUM* target_datum) {
	EVDS_REAL x,y,z; //Components of the result
	if (!target) return;
	if (!source) return;

	//Determine datum based on vector coordinate system object if needed
	if (!target_datum) {
		EVDS_Geodetic_DatumFromObject(&target->datum,source->coordinate_system);
	} else {
		memcpy(&target->datum,target_datum,sizeof(EVDS_GEODETIC_DATUM));

	}

	//Get vector components relative to datum body
	EVDS_Vector_Get(source,&x,&y,&z,target->datum.object);

	//Check if datum is spheric
	if (target->datum.semimajor_axis == target->datum.semiminor_axis) {
		EVDS_REAL r = sqrt(x*x+y*y+z*z)+EVDS_EPS;

		target->longitude	= EVDS_DEG(atan2(y,x));
		target->latitude	= EVDS_DEG(asin(z/r));
		target->elevation	= r - target->datum.semimajor_axis;
	} else { //Non-spheric datum
		EVDS_REAL eccentricity_squared,normal;
		EVDS_REAL p,q,k,k_prev,k0,ci;
		EVDS_REAL lat,tan_lat,sin_lat;
		int iterations;

		//Calculate some information about ellipsoid
		eccentricity_squared = 1 - 
			(target->datum.semiminor_axis*target->datum.semiminor_axis)/
			(target->datum.semimajor_axis*target->datum.semimajor_axis);
		k0 = 1/(1 - eccentricity_squared);
		p = sqrt(x*x+y*y);

		//Calculate longitude
		target->longitude = EVDS_DEG(atan2(y,x));
		if (target->longitude == 180.0) target->longitude = -180.0;

		//Use newtons method to calculate latitude
		k = k0;
		k_prev = 1/EVDS_EPS;
		iterations = 0;

		while ((fabs(k-k_prev) > EVDS_EPS) && (iterations < 8)) {
			q = p*p + (1 - eccentricity_squared)*z*z*k*k*k;
			ci = pow(q,3.0/2.0)/(target->datum.semimajor_axis*eccentricity_squared);

			k_prev = k;
			k = 1 + q / (ci - p*p);
			iterations++;
		}

		if (p != 0.0) {
			tan_lat = k*z/p;
		} else {
			if (k*z > 0) {
				tan_lat = 1e100;
			} else {
				tan_lat = -1e100;
			}
		}
		lat = atan(tan_lat);
		target->latitude = EVDS_DEG(lat);

		//Calculate elevation around ellipsoid
		sin_lat = sin(lat);
		normal = target->datum.semimajor_axis / sqrt(1 - eccentricity_squared * sin_lat * sin_lat);
		if (sin_lat != 0.0) {
			target->elevation = z/sin_lat - normal*(1 - eccentricity_squared);
		} else {
			target->elevation = p - normal;
		}
	}
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion to objects LVLH frame.
///
/// Returns quaternion in objects coordinate frame (although the quaternion represents
/// attitude in LVLH coordinate system).
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_LVLH_QuaternionToLVLH(EVDS_OBJECT* object, EVDS_QUATERNION* target_lvlh, EVDS_QUATERNION* source,
								EVDS_GEODETIC_COORDIANTE* coordinate) {
	EVDS_QUATERNION q_lon,q_lat; //Rotations to go from North pole to given latitude/longitude
	EVDS_Quaternion_FromEuler(&q_lon,object,0,0,EVDS_RAD(coordinate->longitude));
	EVDS_Quaternion_FromEuler(&q_lat,object,0,EVDS_RAD(90-coordinate->latitude),0);

	//Rotate quaternion Q from inertial to LVLH coordinates
	EVDS_Quaternion_Convert(target_lvlh,source,object);
	EVDS_Quaternion_MultiplyConjugatedQ(target_lvlh,&q_lon,target_lvlh);
	EVDS_Quaternion_MultiplyConjugatedQ(target_lvlh,&q_lat,target_lvlh);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion from objects LVLH frame.
///
/// Returns quaternion in objects coordinate frame.
///
/// @todo Add documentation
////////////////////////////////////////////////////////////////////////////////
void EVDS_LVLH_QuaternionFromLVLH(EVDS_OBJECT* object, EVDS_QUATERNION* target, EVDS_QUATERNION* source_lvlh,
								  EVDS_GEODETIC_COORDIANTE* coordinate) {
	EVDS_QUATERNION q_lon,q_lat; //Rotations to go from North pole to given latitude/longitude
	EVDS_Quaternion_FromEuler(&q_lon,object,0,0,EVDS_RAD(coordinate->longitude));
	EVDS_Quaternion_FromEuler(&q_lat,object,0,EVDS_RAD(90-coordinate->latitude),0);

	//Rotate quaternion Q from LVLH to inertial coordinates
	EVDS_Quaternion_Convert(target,source_lvlh,object);
	EVDS_Quaternion_Multiply(target,&q_lat,target);
	EVDS_Quaternion_Multiply(target,&q_lon,target);
}
