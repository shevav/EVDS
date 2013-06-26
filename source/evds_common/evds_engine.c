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
/// @page EVDS_Solver_RocketEngine Rocket Engine
///
/// The rocket engine simulation has the following features:
///	 - Parametric rocket engine geometry.
///	 - Rocket engine perfomance is calculated from variables defined (geometric variables
///    or explicit perfomance data).
///  - Engine perfomance variation due to external pressure is computed.
///	 - Thrust throttle limit can be defined.
///	 - Finite throttle up/throttle down time is simulated, finite startup/shutdown times.
///	 - Engine thrust variation is integrated from several points over a single time step
///    (if a solver like RK4 is used).
///
/// The following parameters define the rocket engine geometry:
///  - 
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"


#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_SOLVER_ENGINE_USERDATA_TAG {
	EVDS_VARIABLE *key;
	EVDS_VARIABLE *force;
} EVDS_SOLVER_ENGINE_USERDATA;
#endif




////////////////////////////////////////////////////////////////////////////////
/// @brief Generate geometry for the rocket engine
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_GenerateGeometry(EVDS_OBJECT* object) {
	EVDS_VARIABLE* geometry;

	//Default values
	EVDS_REAL exit_radius = 0;
	EVDS_REAL chamber_radius = 0;
	EVDS_REAL chamber_length = 0;
	EVDS_REAL area_ratio = 0;
	EVDS_REAL nozzle_length = 0;
	EVDS_REAL divergence_angle = 0;

	//Reset the cross-sections for the engine
	if (EVDS_Object_GetVariable(object,"geometry.cross_sections",&geometry) == EVDS_OK) {
		EVDS_Variable_Destroy(geometry);
	}
	EVDS_Object_AddVariable(object,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&geometry);

	//Get engine parameters (FIXME: use userdata)
	EVDS_Object_GetRealVariable(object,"exit_radius",&exit_radius,0);
	EVDS_Object_GetRealVariable(object,"chamber_radius",&chamber_radius,0);
	EVDS_Object_GetRealVariable(object,"chamber_length",&chamber_length,0);
	EVDS_Object_GetRealVariable(object,"area_ratio",&area_ratio,0);
	EVDS_Object_GetRealVariable(object,"nozzle_length",&nozzle_length,0);
	EVDS_Object_GetRealVariable(object,"divergence_angle",&divergence_angle,0);

	//Sanity checks and limits for inputs
	if (divergence_angle > 0.0) {
		if (divergence_angle < 1.0) divergence_angle = 1.0;
		if (divergence_angle > 80.0) divergence_angle = 80.0;
		nozzle_length = exit_radius / tan(EVDS_RAD(divergence_angle));
	}
	if (area_ratio < 0.1) area_ratio = 0.1;

	//Generate correct geometry
	if (1) {
		EVDS_VARIABLE* chamber_tip;
		EVDS_VARIABLE* chamber_rim;
		EVDS_VARIABLE* nozzle_start;
		EVDS_VARIABLE* nozzle_throat;
		EVDS_VARIABLE* nozzle_end;

		//Add cross-sections
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&chamber_tip);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&chamber_rim);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&nozzle_start);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&nozzle_throat);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&nozzle_end);

		//Radius
		EVDS_Variable_AddFloatAttribute(chamber_tip,	"r",0.00,0);
		EVDS_Variable_AddFloatAttribute(chamber_rim,	"r",chamber_radius,0);
		EVDS_Variable_AddFloatAttribute(nozzle_start,	"r",chamber_radius,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"r",exit_radius*sqrt(1/area_ratio),0);
		EVDS_Variable_AddFloatAttribute(nozzle_end,		"r",exit_radius,0);

		//Tangents
		//EVDS_Variable_AddFloatAttribute(chamber_tip,	"tangent.radial.pos",chamber_radius,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"tangent.radial.neg",0.0,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"tangent.offset.neg",0.0,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"tangent.radial.pos",exit_radius*0.5,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"tangent.offset.pos",exit_radius*0.5,0);

		//Offsets
		EVDS_Variable_AddFloatAttribute(chamber_tip,	"offset",0,0);
		EVDS_Variable_AddFloatAttribute(chamber_rim,	"offset",0,0);
		EVDS_Variable_AddFloatAttribute(nozzle_start,	"offset",chamber_length,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"offset",nozzle_length*0.1,0);
		EVDS_Variable_AddFloatAttribute(nozzle_end,		"offset",nozzle_length*0.9,0);

		//Thickness
		EVDS_Variable_AddFloatAttribute(chamber_tip,	"thickness", 0.02,0);
		EVDS_Variable_AddFloatAttribute(chamber_rim,	"thickness", 0.02,0);
		EVDS_Variable_AddFloatAttribute(nozzle_start,	"thickness", 0.02,0);
		EVDS_Variable_AddFloatAttribute(nozzle_throat,	"thickness", 0.02,0);
		EVDS_Variable_AddFloatAttribute(nozzle_end,		"thickness", 0.02,0);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Try to determine one or more rocket engine variables.
///
/// Returns EVDS_OK if one or more variables could be determined - the rocket engine initializer
/// stops working when all possible variables have been determined
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_DetermineMore(EVDS_SYSTEM* system, EVDS_OBJECT* object) {
	EVDS_VARIABLE* variable;

	//Try to determine oxidizer:fuel ratio from fuel tanks
	if (EVDS_Object_GetVariable(object,"combustion.of_ratio",&variable) != EVDS_OK) {
		EVDS_OBJECT* vessel;
		if (EVDS_Object_GetParentObjectByType(object,"vessel",&vessel) == EVDS_OK) {
			//Total mass of fuel and oxidizer
			EVDS_REAL total_fuel = 0.0;
			EVDS_REAL total_oxidizer = 0.0;

			//Method 1: find all fuel/oxidizer tanks in vessel, divide total oxy by total mass
			SIMC_LIST* list;
			SIMC_LIST_ENTRY* entry;
			EVDS_Object_GetChildren(vessel,&list);

			entry = SIMC_List_GetFirst(list);
			while (entry) {
				char fuel_type_str[256] = { 0 };
				EVDS_REAL fuel_mass_value;
				EVDS_VARIABLE* fuel_type;
				EVDS_VARIABLE* fuel_mass;
				EVDS_OBJECT* fuel_tank = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);

				//Check if object is really a fuel tank
				if (EVDS_Object_CheckType(fuel_tank,"fuel_tank") != EVDS_OK) {
					entry = SIMC_List_GetNext(list,entry);
					continue;
				}

				//Check if fuel type is explicitly defined
				if (EVDS_Object_GetVariable(fuel_tank,"fuel_type",&fuel_type) != EVDS_OK) {
					entry = SIMC_List_GetNext(list,entry);
					continue;
				}
				EVDS_Variable_GetString(fuel_type,fuel_type_str,255,0);

				//Check if fuel mass information is present
				if (EVDS_Object_GetVariable(fuel_tank,"fuel_mass",&fuel_mass) != EVDS_OK) {
					entry = SIMC_List_GetNext(list,entry);
					continue;
				}
				EVDS_Variable_GetReal(fuel_mass,&fuel_mass_value);

				//Add up
				if (EVDS_Material_IsOxidizer(system,fuel_type_str) == EVDS_OK) {
					total_oxidizer += fuel_mass_value;
				}
				if (EVDS_Material_IsFuel(system,fuel_type_str) == EVDS_OK) {
					total_fuel += fuel_mass_value;
				}
				entry = SIMC_List_GetNext(list,entry);
			}

			//Add O/F ratio if possible
			if ((total_oxidizer > 0.0) && (total_fuel > 0.0)) {
				EVDS_Object_AddFloatVariable(object,"combustion.of_ratio",total_oxidizer/total_fuel,0);
				return EVDS_OK;
			}
		}		
	}
	return EVDS_ERROR_INTERNAL;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_REAL mass = 1.0;
	EVDS_SOLVER_ENGINE_USERDATA* userdata;
	if (EVDS_Object_CheckType(object,"rocket_engine") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Create userdata
	userdata = (EVDS_SOLVER_ENGINE_USERDATA*)malloc(sizeof(EVDS_SOLVER_ENGINE_USERDATA));
	memset(userdata,0,sizeof(EVDS_SOLVER_ENGINE_USERDATA));
	EVDS_ERRCHECK(EVDS_Object_SetSolverdata(object,userdata));

	//Add utility variables
	EVDS_Object_AddVariable(object,"key",EVDS_VARIABLE_TYPE_STRING,&userdata->key);
	EVDS_Object_AddVariable(object,"force",EVDS_VARIABLE_TYPE_FLOAT,&userdata->force);

	//Add all possible rocket engine variables
	while (EVDS_InternalRocketEngine_DetermineMore(system,object) == EVDS_OK) ;

	//Generate geometry for the rocket engine
	EVDS_InternalRocketEngine_GenerateGeometry(object);
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize engine solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_SOLVER_ENGINE_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,(void**)&userdata));
	free(userdata);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update engine internal state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Output engine forces
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
								  EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	char key = 0;
	EVDS_REAL force;
	EVDS_SOLVER_ENGINE_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,(void**)&userdata));

	//Get variables
	EVDS_Variable_GetReal(userdata->force,&force);
	EVDS_Variable_GetString(userdata->key,&key,1,0);

	//Calculate force
	//if (key) {
		/*extern int glfwGetKey( int key );

		if (glfwGetKey(key)) {
			EVDS_Vector_Set(&derivative->force,EVDS_VECTOR_FORCE,object,force,0.0,0.0);
		} else {
			EVDS_Vector_Set(&derivative->force,EVDS_VECTOR_FORCE,object,0.0,0.0,0.0);
		}*/
	//} else {
		//EVDS_Vector_Set(&derivative->force,EVDS_VECTOR_FORCE,object,0.0,0.0,0.0);
	//}
	EVDS_Vector_Set(&derivative->force,EVDS_VECTOR_FORCE,object,-100.0,0.0,0.0);
	EVDS_Vector_SetPosition(&derivative->force,object,0.0,0.0,0.0);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_RocketEngine = {
	EVDS_InternalRocketEngine_Initialize, //OnInitialize
	EVDS_InternalRocketEngine_Deinitialize, //OnDeinitialize
	EVDS_InternalRocketEngine_Solve, //OnSolve
	EVDS_InternalRocketEngine_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register engine solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_RocketEngine_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_RocketEngine);
}
