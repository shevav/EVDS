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
/// @page EVDS_Solver_Modifier Modifier
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize modifier and create children copies
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalModifier_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	/*EVDS_SOLVER_GIMBAL_USERDATA* userdata;
	EVDS_STATE_VECTOR vector;
	EVDS_VARIABLE* variable;
	EVDS_OBJECT* parent;
	EVDS_OBJECT* platform;
	if (EVDS_Object_CheckType(object,"gimbal") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Create child object (static body that will collect forces from underlying bodies)
	EVDS_Object_GetParent(object,&parent);
	if (EVDS_Object_CreateBy(object,"Modifier platform",parent,&platform) == EVDS_ERROR_NOT_FOUND) {

		//Create new gimbal platform as a static body
		EVDS_Object_SetType(platform,"static_body");
		if (EVDS_Object_GetVariable(object,"mass",&variable) == EVDS_OK) {
			EVDS_REAL mass;
			EVDS_Variable_GetReal(variable,&mass);
			EVDS_Object_AddRealVariable(platform,"mass",mass,0);
		}

		//Move the platform into position of gimbal
		EVDS_Object_GetStateVector(object,&vector);
		EVDS_Object_SetStateVector(platform,&vector);

		//Move children to that platform
		EVDS_Object_MoveChildren(object,platform);
	}

	//Remove mass from gimbal object. It will not take part in mass calculations, but the platform will
	EVDS_Object_AddRealVariable(object,"mass",0,&variable);
	EVDS_Variable_SetReal(variable,0);

	//Create userdata
	userdata = (EVDS_SOLVER_GIMBAL_USERDATA*)malloc(sizeof(EVDS_SOLVER_GIMBAL_USERDATA));
	memset(userdata,0,sizeof(EVDS_SOLVER_GIMBAL_USERDATA));
	userdata->platform = platform;
	EVDS_ERRCHECK(EVDS_Object_SetSolverdata(object,userdata));

	//Add variables to this object
	EVDS_Object_AddRealVariable(object,"pitch.min",0,&userdata->pitch_min);
	EVDS_Object_AddRealVariable(object,"pitch.max",0,&userdata->pitch_max);
	EVDS_Object_AddRealVariable(object,"pitch.rate",0,&userdata->pitch_rate);
	EVDS_Object_AddRealVariable(object,"pitch.bits",0,&userdata->pitch_bits);
	EVDS_Object_AddRealVariable(object,"pitch.command",0,&userdata->pitch_command);
	EVDS_Object_AddRealVariable(object,"pitch.current",0,&userdata->pitch_current);

	EVDS_Object_AddRealVariable(object,"yaw.min",0,&userdata->yaw_min);
	EVDS_Object_AddRealVariable(object,"yaw.max",0,&userdata->yaw_max);
	EVDS_Object_AddRealVariable(object,"yaw.rate",0,&userdata->yaw_rate);
	EVDS_Object_AddRealVariable(object,"yaw.bits",0,&userdata->yaw_bits);
	EVDS_Object_AddRealVariable(object,"yaw.command",0,&userdata->yaw_command);
	EVDS_Object_AddRealVariable(object,"yaw.current",0,&userdata->yaw_current);

	EVDS_Object_AddVariable(object,"zero_quaternion",EVDS_VARIABLE_TYPE_QUATERNION,&userdata->zero_quaternion);
	EVDS_Object_GetStateVector(object,&vector);
	EVDS_Variable_SetQuaternion(userdata->zero_quaternion,&vector.orientation);*/
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Modifier = {
	EVDS_InternalModifier_Initialize, //OnInitialize
	0, //OnDeinitialize
	0, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register gimballing platform
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Modifier_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Modifier);
}
