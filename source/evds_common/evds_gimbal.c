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
/// @page EVDS_Solver_Gimbal Gimballing Platform
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Update planet position and state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	EVDS_OBJECT* platform;
	EVDS_Object_GetSolverdata(object,(void*)&platform);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Output accelerations for the planet
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
								  EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	EVDS_OBJECT* platform;
	EVDS_Object_GetSolverdata(object,(void*)&platform);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_OBJECT* parent;
	EVDS_OBJECT* platform;
	if (EVDS_Object_CheckType(object,"gimbal") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Create child object (static body that will collect forces from underlying bodies)
	EVDS_Object_GetParent(object,&parent);
	if (EVDS_Object_CreateBy(object,"Gimbal platform",parent,&platform) == EVDS_ERROR_NOT_FOUND) {
		//FIXME: copy physics properties of gimbal too
		EVDS_Object_SetType(platform,"static_body");
		EVDS_Object_CopyChildren(object,platform);
	}

	//Remove mass from gimbal object. It will not take part in mass calculations, but the platform will
	EVDS_Object_AddFloatVariable(object,"mass",0,0);

	//Store platform as the solverdata
	EVDS_Object_SetSolverdata(object,platform);
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Gimbal = {
	EVDS_InternalGimbal_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalGimbal_Solve, //OnSolve
	EVDS_InternalGimbal_Integrate, //OnIntegrate
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
int EVDS_Gimbal_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Gimbal);
}
