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
#include <stdio.h>
#include <math.h>
#include "evds.h"


#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_MODIFIER_VARIABLES_TAG {
	int i,j,k;

	EVDS_REAL vector1_count;
	EVDS_REAL vector2_count;
	EVDS_REAL vector3_count;
	EVDS_REAL vector1[3];
	EVDS_REAL vector2[3];
	EVDS_REAL vector3[3];
} EVDS_MODIFIER_VARIABLES;
#endif


int EVDS_InternalModifier_Copy(EVDS_MODIFIER_VARIABLES* vars, EVDS_OBJECT* container, EVDS_OBJECT* object) {
	char name[257] = { 0 }; //Null-terminate the name
	char modifier_suffix[257] = { 0 };
	EVDS_OBJECT* new_object;
	EVDS_STATE_VECTOR vector;
	EVDS_VECTOR offset;

	//Create a new copy of the object
	EVDS_Object_Copy(object,container,&new_object);
	EVDS_Object_GetStateVector(new_object,&vector);

	//Add suffix and rename
	EVDS_Object_GetName(new_object,name,256);
	snprintf(modifier_suffix,256," (%dx%dx%d)",vars->i+1,vars->j+1,vars->k+1);
	strncat(name,modifier_suffix,256);
	EVDS_Object_SetName(new_object,name);

	//Calculate child copies offset
	EVDS_Vector_Set(&offset,EVDS_VECTOR_POSITION,container,
		vars->i*vars->vector1[0] + vars->j*vars->vector2[0] + vars->k*vars->vector3[0],
		vars->i*vars->vector1[1] + vars->j*vars->vector2[1] + vars->k*vars->vector3[1],
		vars->i*vars->vector1[2] + vars->j*vars->vector2[2] + vars->k*vars->vector3[2]);

	//Apply transformation
	EVDS_Vector_Add(&vector.position,&vector.position,&offset);
	EVDS_Object_SetStateVector(new_object,&vector);
	return EVDS_OK;
}



////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize modifier and create children copies
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalModifier_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	SIMC_LIST* list;
	SIMC_LIST_ENTRY* entry;
	EVDS_STATE_VECTOR vector;

	//Modifier container which will hold the children copies
	EVDS_OBJECT* container;
	EVDS_OBJECT* parent;

	//Modifier variables
	EVDS_MODIFIER_VARIABLES vars = { 0 };

	//Check type
	if (EVDS_Object_CheckType(object,"modifier") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Read variables
	EVDS_Object_GetRealVariable(object,"vector1.count",&vars.vector1_count,0);
	EVDS_Object_GetRealVariable(object,"vector2.count",&vars.vector2_count,0);
	EVDS_Object_GetRealVariable(object,"vector3.count",&vars.vector3_count,0);
	
	if (vars.vector1_count < 1.0) vars.vector1_count = 1.0;
	if (vars.vector2_count < 1.0) vars.vector2_count = 1.0;
	if (vars.vector3_count < 1.0) vars.vector3_count = 1.0;

	EVDS_Object_GetRealVariable(object,"vector1.x",&vars.vector1[0],0);
	EVDS_Object_GetRealVariable(object,"vector1.y",&vars.vector1[1],0);
	EVDS_Object_GetRealVariable(object,"vector1.z",&vars.vector1[2],0);
	EVDS_Object_GetRealVariable(object,"vector2.x",&vars.vector2[0],0);
	EVDS_Object_GetRealVariable(object,"vector2.y",&vars.vector2[1],0);
	EVDS_Object_GetRealVariable(object,"vector2.z",&vars.vector2[2],0);
	EVDS_Object_GetRealVariable(object,"vector3.x",&vars.vector3[0],0);
	EVDS_Object_GetRealVariable(object,"vector3.y",&vars.vector3[1],0);
	EVDS_Object_GetRealVariable(object,"vector3.z",&vars.vector3[2],0);

	//Create new container as a mass-less static body
	EVDS_Object_GetParent(object,&parent);
	if (EVDS_Object_CreateBy(object,"Children",parent,&container) != EVDS_ERROR_NOT_FOUND) {
		return EVDS_CLAIM_OBJECT; //Early return (modifier already created)
	}
	EVDS_Object_SetType(container,"static_body");

	//Move the platform into position of modifier
	EVDS_Object_GetStateVector(object,&vector);
	EVDS_Object_SetStateVector(container,&vector);

	//For every child, create instances
	EVDS_Object_GetAllChildren(object,&list);
	entry = SIMC_List_GetFirst(list);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
		for (vars.i = 0; vars.i < (int)vars.vector1_count; vars.i++) {
			for (vars.j = 0; vars.j < (int)vars.vector2_count; vars.j++) {
				for (vars.k = 0; vars.k < (int)vars.vector3_count; vars.k++) {
					if ((vars.i != 0) || (vars.j != 0) || (vars.k != 0)) {
						EVDS_InternalModifier_Copy(&vars,container,child);
					}
				}
			}
		}

		//Move the child into container
		SIMC_List_Stop(list,entry);
		EVDS_Object_SetParent(child,container);

		//Start again from scratch (list is 1 child less full)
		entry = SIMC_List_GetFirst(list);
	}

	//Initialize container
	EVDS_Object_Initialize(container,1);
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
