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
	//Modifier container which will hold the children copies
	EVDS_OBJECT* container;
	EVDS_OBJECT* parent;

	SIMC_LIST* list;
	SIMC_LIST_ENTRY* entry;
	EVDS_STATE_VECTOR vector;

	//Modifier variables
	int i,j,k;
	EVDS_REAL vector1_count = 0;
	EVDS_REAL vector2_count = 0;
	EVDS_REAL vector3_count = 0;
	EVDS_REAL vector1[3] = { 0 };
	EVDS_REAL vector2[3] = { 0 };
	EVDS_REAL vector3[3] = { 0 };

	//Check type
	if (EVDS_Object_CheckType(object,"modifier") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Read variables
	EVDS_Object_GetRealVariable(object,"vector1.count",&vector1_count,0);
	EVDS_Object_GetRealVariable(object,"vector2.count",&vector2_count,0);
	EVDS_Object_GetRealVariable(object,"vector3.count",&vector3_count,0);

	EVDS_Object_GetRealVariable(object,"vector1.x",&vector1[0],0);
	EVDS_Object_GetRealVariable(object,"vector1.y",&vector1[1],0);
	EVDS_Object_GetRealVariable(object,"vector1.z",&vector1[2],0);
	EVDS_Object_GetRealVariable(object,"vector2.x",&vector2[0],0);
	EVDS_Object_GetRealVariable(object,"vector2.y",&vector2[1],0);
	EVDS_Object_GetRealVariable(object,"vector2.z",&vector2[2],0);
	EVDS_Object_GetRealVariable(object,"vector3.x",&vector3[0],0);
	EVDS_Object_GetRealVariable(object,"vector3.y",&vector3[1],0);
	EVDS_Object_GetRealVariable(object,"vector3.z",&vector3[2],0);

	//Create new container as a mass-less static body
	EVDS_Object_GetParent(object,&parent);
	if (EVDS_Object_CreateBy(object,"Modifier children",parent,&container) != EVDS_ERROR_NOT_FOUND) {
		return EVDS_CLAIM_OBJECT; //Early return (modifier already created)
	}
	EVDS_Object_SetType(container,"static_body");

	//Move the platform into position of modifier
	EVDS_Object_GetStateVector(object,&vector);
	EVDS_Object_SetStateVector(container,&vector);

	//For every child, create instances
	/*EVDS_Object_GetAllChildren(object,&list);
	entry = SIMC_List_GetFirst(list);
	while (entry) {
		for (int i = 0; i < (int)vector1_count; i++) {
			for (int j = 0; j < (int)vector2_count; j++) {
				for (int k = 0; k < (int)vector3_count; k++) {
					EVDS_VECTOR offset;
					EVDS_Object_GetStateVector(object,&vector);

					EVDS_Vector_Set(&offset,EVDS_VECTOR_POSITION,container,
						i*vector1[0] + j*vector2[0] + k*vector2[0],
						i*vector1[1] + j*vector2[1] + k*vector2[1],
						i*vector1[2] + j*vector2[2] + k*vector2[2]);

											
				}
			}
		}
		entry = SIMC_List_GetNext(list,entry);
	}*/
	//EVDS_Object_GetStateVector(object,&vector);
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
