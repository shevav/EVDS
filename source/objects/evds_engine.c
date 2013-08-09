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
	SIMC_LIST* fuel_tanks;
	SIMC_LIST* oxidizer_tanks;

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
	EVDS_Object_GetRealVariable(object,"nozzle.exit_radius",&exit_radius,0);
	EVDS_Object_GetRealVariable(object,"nozzle.area_ratio",&area_ratio,0);
	EVDS_Object_GetRealVariable(object,"nozzle.length",&nozzle_length,0);
	EVDS_Object_GetRealVariable(object,"nozzle.divergence_angle",&divergence_angle,0);
	EVDS_Object_GetRealVariable(object,"combustion.chamber_radius",&chamber_radius,0);
	EVDS_Object_GetRealVariable(object,"combustion.chamber_length",&chamber_length,0);

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
/// @brief Try to determine fuel tanks which feed this engine.
///
/// The fuel tanks will be marked as stored until next engine solver run detects
/// either of the tanks data was deleted.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_DetermineFuelTanks(EVDS_SOLVER_ENGINE_USERDATA* userdata, 
												 EVDS_SYSTEM* system, EVDS_OBJECT* object) {
	EVDS_OBJECT* vessel;
	EVDS_VARIABLE* variable;
	SIMC_LIST* list;
	SIMC_LIST_ENTRY* entry;
	char fuel_type[257] = { 0 };
	char oxidizer_type[257] = { 0 };

	//No tanks are used by default
	userdata->fuel_tanks = 0;
	userdata->oxidizer_tanks = 0;

	//Find parent vessel
	if (EVDS_Object_GetParentObjectByType(object,"vessel",&vessel) != EVDS_OK) {
		//No parent vessel: no fuel tanks are used
		return EVDS_OK;
	}

	//Try to determine rocket oxidizer/fuel type
	if (EVDS_Object_GetVariable(object,"combustion.fuel",&variable) == EVDS_OK) {
		EVDS_Variable_GetString(variable,fuel_type,256,0);
	}
	if (EVDS_Object_GetVariable(object,"combustion.oxidizer",&variable) == EVDS_OK) {
		EVDS_Variable_GetString(variable,oxidizer_type,256,0);
	}

	//Making use of fuel tanks
	SIMC_List_Create(&userdata->fuel_tanks,0);
	SIMC_List_Create(&userdata->oxidizer_tanks,0);

	//Find fuel tanks that are direct children of the vessel
	EVDS_Object_GetChildren(vessel,&list);
	entry = SIMC_List_GetFirst(list);
	while (entry) {
		int is_oxidizer = 0;
		char tank_type[256] = { 0 };
		EVDS_OBJECT* tank = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);

		//Check if object is really a fuel tank
		if (EVDS_Object_CheckType(tank,"fuel_tank") != EVDS_OK) {
			entry = SIMC_List_GetNext(list,entry);
			continue;
		}

		//Check if fuel type is explicitly defined
		if (EVDS_Object_GetVariable(tank,"fuel.type",&variable) != EVDS_OK) {
			entry = SIMC_List_GetNext(list,entry);
			continue;
		}
		EVDS_Variable_GetString(variable,tank_type,255,0);

		//Define fuel used by rocket engine (if not already defined)
		if (EVDS_Material_IsOxidizer(system,tank_type) == EVDS_OK) is_oxidizer = 1;
		if ((!fuel_type[0]) && (!is_oxidizer)) {
			strncpy(fuel_type,tank_type,256);
			EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"combustion.fuel",EVDS_VARIABLE_TYPE_STRING,&variable));
			EVDS_Variable_SetString(variable,fuel_type,strlen(fuel_type));
		}
		if ((!oxidizer_type[0]) && (is_oxidizer)) {
			strncpy(oxidizer_type,tank_type,256);
			EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"combustion.oxidizer",EVDS_VARIABLE_TYPE_STRING,&variable));
			EVDS_Variable_SetString(variable,oxidizer_type,strlen(oxidizer_type));
		}

		//If this fuel tank matches required types, add it to lists
		if (strncmp(fuel_type,tank_type,256) == 0) { //Fuel
			EVDS_Object_Store(tank);
			SIMC_List_Append(userdata->fuel_tanks,tank);
		}
		if (strncmp(oxidizer_type,tank_type,256) == 0) { //Oxidizer
			EVDS_Object_Store(tank);
			SIMC_List_Append(userdata->oxidizer_tanks,tank);
		}		

		//Check next object
		entry = SIMC_List_GetNext(list,entry);
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


	////////////////////////////////////////////////////////////////////////////////
	// Nozzle parameters
	////////////////////////////////////////////////////////////////////////////////
	// Determine nozzle exit area from exit radius
	if (EVDS_Object_GetVariable(object,"nozzle.exit_area",&variable) != EVDS_OK) {
		EVDS_REAL exit_radius;
		if ((EVDS_Object_GetRealVariable(object,"nozzle.exit_radius",&exit_radius,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"nozzle.exit_area",EVDS_PI*(exit_radius*exit_radius),0);
			return EVDS_OK;
		}
	}
	// Determine nozzle exit radius from exit area
	if (EVDS_Object_GetVariable(object,"nozzle.exit_radius",&variable) != EVDS_OK) {
		EVDS_REAL exit_area;
		if ((EVDS_Object_GetRealVariable(object,"nozzle.exit_area",&exit_area,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"nozzle.exit_radius",sqrt(exit_area/EVDS_PI),0);
			return EVDS_OK;
		}
	}


	////////////////////////////////////////////////////////////////////////////////
	// Mass flow
	////////////////////////////////////////////////////////////////////////////////
	// Determine mass flow from from Isp
	if (EVDS_Object_GetVariable(object,"vacuum.mass_flow",&variable) != EVDS_OK) {
		EVDS_REAL isp,thrust;
		if ((EVDS_Object_GetRealVariable(object,"vacuum.isp",&isp,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"vacuum.thrust",&thrust,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"vacuum.mass_flow",thrust/(EVDS_G0*isp),0);
			return EVDS_OK;
		}
	}
	// Determine fuel flow from mass flow
	if (EVDS_Object_GetVariable(object,"vacuum.fuel_flow",&variable) != EVDS_OK) {
		EVDS_REAL of_ratio,mass_flow;
		if ((EVDS_Object_GetRealVariable(object,"combustion.of_ratio",&of_ratio,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"vacuum.mass_flow",&mass_flow,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"vacuum.fuel_flow",mass_flow/(of_ratio+1.0),0);
			return EVDS_OK;
		}
	}
	// Determine oxidizer flow from mass flow
	if (EVDS_Object_GetVariable(object,"vacuum.oxidizer_flow",&variable) != EVDS_OK) {
		EVDS_REAL of_ratio,mass_flow;
		if ((EVDS_Object_GetRealVariable(object,"combustion.of_ratio",&of_ratio,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"vacuum.mass_flow",&mass_flow,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"vacuum.oxidizer_flow",of_ratio*mass_flow/(of_ratio+1.0),0);
			return EVDS_OK;
		}
	}
	// Determine mass flow from from Isp [atmospheric]
	if (EVDS_Object_GetVariable(object,"atmospheric.mass_flow",&variable) != EVDS_OK) {
		EVDS_REAL isp,thrust;
		if ((EVDS_Object_GetRealVariable(object,"atmospheric.isp",&isp,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"atmospheric.thrust",&thrust,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"atmospheric.mass_flow",thrust/(EVDS_G0*isp),0);
			return EVDS_OK;
		}
	}
	// Determine fuel flow from mass flow [atmospheric]
	if (EVDS_Object_GetVariable(object,"atmospheric.fuel_flow",&variable) != EVDS_OK) {
		EVDS_REAL of_ratio,mass_flow;
		if ((EVDS_Object_GetRealVariable(object,"combustion.of_ratio",&of_ratio,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"atmospheric.mass_flow",&mass_flow,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"atmospheric.fuel_flow",mass_flow/(of_ratio+1.0),0);
			return EVDS_OK;
		}
	}
	// Determine oxidizer flow from mass flow [atmospheric]
	if (EVDS_Object_GetVariable(object,"atmospheric.oxidizer_flow",&variable) != EVDS_OK) {
		EVDS_REAL of_ratio,mass_flow;
		if ((EVDS_Object_GetRealVariable(object,"combustion.of_ratio",&of_ratio,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"atmospheric.mass_flow",&mass_flow,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"atmospheric.oxidizer_flow",of_ratio*mass_flow/(of_ratio+1.0),0);
			return EVDS_OK;
		}
	}


	////////////////////////////////////////////////////////////////////////////////
	// Isp and exhaust velocity
	////////////////////////////////////////////////////////////////////////////////
	//Determine Ve from Isp
	if (EVDS_Object_GetVariable(object,"vacuum.exhaust_velocity",&variable) != EVDS_OK) {
		EVDS_REAL isp;
		if (EVDS_Object_GetRealVariable(object,"vacuum.isp",&isp,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"vacuum.exhaust_velocity",EVDS_G0*isp,0);
			return EVDS_OK;
		}
	}
	//Determine Isp from Ve
	if (EVDS_Object_GetVariable(object,"vacuum.isp",&variable) != EVDS_OK) {
		EVDS_REAL exhaust_velocity;
		if (EVDS_Object_GetRealVariable(object,"vacuum.exhaust_velocity",&exhaust_velocity,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"vacuum.isp",exhaust_velocity/EVDS_G0,0);
			return EVDS_OK;
		}
	}
	//Determine Ve from Isp [atmospheric]
	if (EVDS_Object_GetVariable(object,"atmospheric.exhaust_velocity",&variable) != EVDS_OK) {
		EVDS_REAL isp;
		if (EVDS_Object_GetRealVariable(object,"atmospheric.isp",&isp,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"atmospheric.exhaust_velocity",EVDS_G0*isp,0);
			return EVDS_OK;
		}
	}
	//Determine Isp from Ve [atmospheric]
	if (EVDS_Object_GetVariable(object,"atmospheric.isp",&variable) != EVDS_OK) {
		EVDS_REAL exhaust_velocity;
		if (EVDS_Object_GetRealVariable(object,"atmospheric.exhaust_velocity",&exhaust_velocity,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"atmospheric.isp",exhaust_velocity/EVDS_G0,0);
			return EVDS_OK;
		}
	}


	////////////////////////////////////////////////////////////////////////////////
	// Thrust and Ve
	////////////////////////////////////////////////////////////////////////////////
	//Determine atmospheric F from vacuum F, assuming 1 bar atmospheric
	/*if (EVDS_Object_GetVariable(object,"atmospheric.thrust",&variable) != EVDS_OK) {
		EVDS_REAL thrust,exit_area;
		if ((EVDS_Object_GetRealVariable(object,"vacuum.thrust",&thrust,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"nozzle.exit_area",&exit_area,0) == EVDS_OK)) {
			//Fatm = F-(Pe - P0)*Ae
			EVDS_Object_AddRealVariable(object,"atmospheric.thrust",thrust - 1e5*exit_area,0);
			return EVDS_OK;
		}
	}
	//Determine vacuum F from atmospheric F, assuming 1 bar atmospheric
	if (EVDS_Object_GetVariable(object,"vacuum.thrust",&variable) != EVDS_OK) {
		EVDS_REAL thrust,exit_area;
		if ((EVDS_Object_GetRealVariable(object,"atmospheric.thrust",&thrust,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"nozzle.exit_area",&exit_area,0) == EVDS_OK)) {
			//F = Fatm+(Pe - P0)*Ae
			EVDS_Object_AddRealVariable(object,"vacuum.thrust",thrust + 1e5*exit_area,0);
			return EVDS_OK;
		}
	}*/
	//Determine atmospheric Ve (effective)
	if (EVDS_Object_GetVariable(object,"atmospheric.exhaust_velocity",&variable) != EVDS_OK) {
		EVDS_REAL mass_flow,thrust;
		if ((EVDS_Object_GetRealVariable(object,"atmospheric.mass_flow",&mass_flow,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"atmospheric.thrust",&thrust,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"atmospheric.exhaust_velocity",thrust/mass_flow,0); //Ve = F/mdot
			return EVDS_OK;
		}
	}
	//Determine Ve
	if (EVDS_Object_GetVariable(object,"vacuum.exhaust_velocity",&variable) != EVDS_OK) {
		EVDS_REAL mass_flow,thrust;
		if ((EVDS_Object_GetRealVariable(object,"vacuum.mass_flow",&mass_flow,0) == EVDS_OK) &&
			(EVDS_Object_GetRealVariable(object,"vacuum.thrust",&thrust,0) == EVDS_OK)) {
			EVDS_Object_AddRealVariable(object,"vacuum.exhaust_velocity",thrust/mass_flow,0);
			return EVDS_OK;
		}
	}


	////////////////////////////////////////////////////////////////////////////////
	// Fall-back hacks (must be last in list)
	////////////////////////////////////////////////////////////////////////////////
	//Determine atmospheric F from vacuum F, in a hacky way
	if (EVDS_Object_GetVariable(object,"atmospheric.thrust",&variable) != EVDS_OK) {
		EVDS_REAL thrust;
		if (EVDS_Object_GetRealVariable(object,"vacuum.thrust",&thrust,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"atmospheric.thrust",thrust*0.90,0);
			return EVDS_OK;
		}
	}
	//Determine vacuum F from atmospheric F, in a hacky way
	if (EVDS_Object_GetVariable(object,"vacuum.thrust",&variable) != EVDS_OK) {
		EVDS_REAL thrust;
		if (EVDS_Object_GetRealVariable(object,"atmospheric.thrust",&thrust,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"vacuum.thrust",thrust/0.90,0);
			return EVDS_OK;
		}
	}
	//Determine atmospheric Isp from vacuum Isp, in a hacky way
	if (EVDS_Object_GetVariable(object,"atmospheric.isp",&variable) != EVDS_OK) {
		EVDS_REAL isp;
		if (EVDS_Object_GetRealVariable(object,"vacuum.isp",&isp,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"atmospheric.isp",isp*0.90,0);
			return EVDS_OK;
		}
	}
	//Determine vacuum Isp from atmospheric Isp, in a hacky way
	if (EVDS_Object_GetVariable(object,"vacuum.isp",&variable) != EVDS_OK) {
		EVDS_REAL isp;
		if (EVDS_Object_GetRealVariable(object,"atmospheric.isp",&isp,0) == EVDS_OK) {
			EVDS_Object_AddRealVariable(object,"vacuum.isp",isp/0.90,0);
			return EVDS_OK;
		}
	}


	////////////////////////////////////////////////////////////////////////////////
	// ...
	////////////////////////////////////////////////////////////////////////////////
	//Try to determine oxidizer:fuel ratio from fuel tanks
	/*if (EVDS_Object_GetVariable(object,"combustion.of_ratio",&variable) != EVDS_OK) {
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
				EVDS_Object_AddRealVariable(object,"combustion.of_ratio",total_oxidizer/total_fuel,0);
				return EVDS_OK;
			}
		}		
	}*/
	return EVDS_ERROR_INTERNAL;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRocketEngine_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_SOLVER_ENGINE_USERDATA* userdata;
	if (EVDS_Object_CheckType(object,"rocket_engine") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Create userdata
	userdata = (EVDS_SOLVER_ENGINE_USERDATA*)malloc(sizeof(EVDS_SOLVER_ENGINE_USERDATA));
	memset(userdata,0,sizeof(EVDS_SOLVER_ENGINE_USERDATA));
	EVDS_ERRCHECK(EVDS_Object_SetSolverdata(object,userdata));

	//Determine fuel tanks
	EVDS_InternalRocketEngine_DetermineFuelTanks(userdata,system,object);
	//Add all possible rocket engine variables
	while (EVDS_InternalRocketEngine_DetermineMore(system,object) == EVDS_OK) ;

	//Remember some of the variables for userdata
	EVDS_Object_AddVariable(object,"key",EVDS_VARIABLE_TYPE_STRING,&userdata->key);
	EVDS_Object_AddVariable(object,"vacuum.thrust",EVDS_VARIABLE_TYPE_FLOAT,&userdata->force);

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

	if (userdata->fuel_tanks) SIMC_List_Destroy(userdata->fuel_tanks);
	if (userdata->oxidizer_tanks) SIMC_List_Destroy(userdata->oxidizer_tanks);

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
	{
		/*static shut = 0;
		EVDS_OBJECT* inertial;
		EVDS_STATE_VECTOR vector;
		EVDS_VECTOR vel;
		EVDS_REAL mag2;
		EVDS_System_GetRootInertialSpace(system,&inertial);
		EVDS_Object_GetStateVector(object,&vector);
		EVDS_Vector_Convert(&vel,&vector.velocity,inertial);
		EVDS_Vector_Dot(&mag2,&vel,&vel);
		mag2 = sqrt(mag2);
		if (mag2 > 3500.0) shut = 1;
		if (shut) return EVDS_OK;*/
		return EVDS_OK;
	}


	//EVDS_Variable_GetReal(userdata->force,&force);
	//EVDS_Variable_GetString(userdata->key,&key,1,0);
	//EVDS_Vector_Set(&derivative->force,EVDS_VECTOR_FORCE,object,-force,0.0,0.0);
	//EVDS_Vector_SetPosition(&derivative->force,object,0.0,0.0,0.0);

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
	//EVDS_Vector_Set(&derivative->force,EVDS_VECTOR_FORCE,object,-100.0,0.0,0.0);
	//EVDS_Vector_SetPosition(&derivative->force,object,0.0,0.0,0.0);
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
