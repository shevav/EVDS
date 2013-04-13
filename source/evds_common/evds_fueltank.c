////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2013, Black Phoenix
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///   - Redistributions of source code must retain the above copyright
///     notice, this list of conditions and the following disclaimer.
///   - Redistributions in binary form must reproduce the above copyright
///     notice, this list of conditions and the following disclaimer in the
///     documentation and/or other materials provided with the distribution.
///   - Neither the name of the author nor the names of the contributors may
///     be used to endorse or promote products derived from this software without
///     specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_Solver_FuelTank Fuel tank
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Generate geometry for the fuel tank
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_GenerateGeometry(EVDS_OBJECT* object) {
	EVDS_VARIABLE* geometry;
	EVDS_REAL upper_radius = 0.0;
	EVDS_REAL lower_radius = 0.0;
	EVDS_REAL outer_radius = 0.0;
	EVDS_REAL inner_radius = 0.0;
	EVDS_REAL middle_length = 0.0;
	
	EVDS_Object_GetRealVariable(object,"upper_radius",&upper_radius,0);
	EVDS_Object_GetRealVariable(object,"lower_radius",&lower_radius,0);
	EVDS_Object_GetRealVariable(object,"outer_radius",&outer_radius,0);
	EVDS_Object_GetRealVariable(object,"inner_radius",&inner_radius,0);
	EVDS_Object_GetRealVariable(object,"middle_length",&middle_length,0);

	if ((upper_radius == 0.0) &&
		(lower_radius == 0.0) &&
		(outer_radius == 0.0) &&
		(inner_radius == 0.0) &&
		(middle_length == 0.0)) {
		return EVDS_OK;
	}

	//Reset the cross-sections for the engine
	if (EVDS_Object_GetVariable(object,"csection_geometry",&geometry) == EVDS_OK) {
		EVDS_Variable_Destroy(geometry);
	}
	EVDS_Object_AddVariable(object,"csection_geometry",EVDS_VARIABLE_TYPE_NESTED,&geometry);

	//Generate correct geometry
	if (1) {
		EVDS_VARIABLE* upper_tip;
		EVDS_VARIABLE* upper_rim;
		EVDS_VARIABLE* lower_rim;
		EVDS_VARIABLE* lower_tip;

		//Add cross-sections
		EVDS_Variable_AddNested(geometry,"csection_geometry",EVDS_VARIABLE_TYPE_NESTED,&upper_tip);
		EVDS_Variable_AddNested(geometry,"csection_geometry",EVDS_VARIABLE_TYPE_NESTED,&upper_rim);
		EVDS_Variable_AddNested(geometry,"csection_geometry",EVDS_VARIABLE_TYPE_NESTED,&lower_rim);
		EVDS_Variable_AddNested(geometry,"csection_geometry",EVDS_VARIABLE_TYPE_NESTED,&lower_tip);

		//Radius
		EVDS_Variable_AddFloatAttribute(upper_tip,"rx",inner_radius,0);
		EVDS_Variable_AddFloatAttribute(upper_rim,"rx",outer_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"rx",outer_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_tip,"rx",inner_radius,0);

		//Tangents
		//EVDS_Variable_AddFloatAttribute(upper_tip,"tangent_p_radial",0.0,0); //outer_radius
		EVDS_Variable_AddFloatAttribute(upper_rim,"tangent_m_offset",upper_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"tangent_p_offset",lower_radius,0);
		//EVDS_Variable_AddFloatAttribute(lower_tip,"tangent_p_offset",0.0,0);

		//Offsets
		EVDS_Variable_AddFloatAttribute(upper_tip,"add_offset",0,0);
		EVDS_Variable_AddFloatAttribute(upper_rim,"add_offset",1,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"add_offset",1,0);
		EVDS_Variable_AddFloatAttribute(lower_tip,"add_offset",1,0);

		EVDS_Variable_AddFloatAttribute(upper_tip,"offset",0.0,0);
		EVDS_Variable_AddFloatAttribute(upper_rim,"offset",upper_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"offset",middle_length,0);
		EVDS_Variable_AddFloatAttribute(lower_tip,"offset",lower_radius,0);

	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update engine internal state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	EVDS_VARIABLE* v_mass;
	EVDS_VARIABLE* v_fuel_mass;
	EVDS_VARIABLE* v_total_mass;
	EVDS_REAL mass;
	EVDS_REAL fuel_mass;

	//Calculate total mass of the tank
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"mass",&v_mass));
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"fuel_mass",&v_fuel_mass));
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"total_mass",&v_total_mass));

	//Set total mass
	EVDS_Variable_GetReal(v_mass,&mass);
	EVDS_Variable_GetReal(v_fuel_mass,&fuel_mass);
	EVDS_Variable_SetReal(v_total_mass,mass+fuel_mass);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_VARIABLE* variable;
	EVDS_REAL fuel_mass = 0.0;
	EVDS_REAL fuel_volume = 0.0;
	EVDS_REAL is_cryogenic = 0.0;
	if (EVDS_Object_CheckType(object,"fuel_tank") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Generate geometry for the rocket engine
	EVDS_InternalFuelTank_GenerateGeometry(object);

	//Is fuel cryogenic
	if (EVDS_Object_GetVariable(object,"is_cryogenic",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&is_cryogenic);
	} else {
		is_cryogenic = 0;
		EVDS_Object_AddFloatVariable(object,"is_cryogenic",0,0);
	}
	
	//Calculate total volume
	if (EVDS_Object_GetVariable(object,"fuel_volume",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&fuel_volume);
	} 
	if (fuel_volume < EVDS_EPS) {
		EVDS_MESH* mesh;
		EVDS_Mesh_Generate(object,&mesh,50.0f,EVDS_MESH_USE_DIVISIONS);
		fuel_volume = mesh->total_volume;
		EVDS_Object_AddFloatVariable(object,"fuel_volume",fuel_volume,0);
		EVDS_Mesh_Destroy(mesh);
	}

	//Calculate total mass
	if (EVDS_Object_GetVariable(object,"fuel_mass",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&fuel_mass);
	} 
	if (fuel_mass < EVDS_EPS) {
		EVDS_VARIABLE* material_database;
		EVDS_VARIABLE* material;
		EVDS_REAL fuel_density = 1000.0;

		//Check if material is defined
		if ((EVDS_Object_GetVariable(object,"fuel_type",&variable) == EVDS_OK) &&
			(EVDS_System_GetDatabaseByName(system,"material",&material_database) == EVDS_OK)) {
			char material_name[1024] = { 0 };
			EVDS_Variable_GetString(variable,material_name,1023,0);

			//Get material parameters
			if (EVDS_Variable_GetNested(material_database,material_name,&material) == EVDS_OK) {
				EVDS_REAL boiling_point = 0.0;
				EVDS_REAL fuel_temperature = 293.15; //20 C

				//Try liquid-phase fuel
				if (EVDS_Variable_GetNested(material,"boiling_point",&variable) == EVDS_OK) {
					EVDS_Variable_GetReal(variable,&boiling_point);
				}

				//Check if fuel is cryogenic or otherwise cooled down
				if (boiling_point < 273) {
					is_cryogenic = 1; //Force cryogenic fuel
					fuel_temperature = boiling_point - 0.1;

					EVDS_Object_GetVariable(object,"is_cryogenic",&variable);
					EVDS_Variable_SetReal(variable,1);
				}

				//Get density
				if (EVDS_Variable_GetNested(material,"density",&variable) == EVDS_OK) {
					EVDS_Variable_GetInterpolated1D(variable,fuel_temperature,&fuel_density);
				}
			}

			//Calculate fuel mass
			fuel_mass = fuel_volume * fuel_density;
		}
		EVDS_Object_AddFloatVariable(object,"fuel_mass",fuel_mass,0);
	}

	//Remember the tank capacity
	EVDS_Object_AddFloatVariable(object,"fuel_capacity",fuel_mass,0);

	//Fuel tanks have mass
	if (EVDS_Object_GetVariable(object,"mass",&variable) != EVDS_OK) {
		EVDS_Object_AddFloatVariable(object,"mass",0,0);
	}
	EVDS_Object_AddFloatVariable(object,"total_mass",0,0);
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize engine solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_FuelTank = {
	EVDS_InternalFuelTank_Initialize, //OnInitialize
	EVDS_InternalFuelTank_Deinitialize, //OnDeinitialize
	EVDS_InternalFuelTank_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register fuel tank solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_FuelTank_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_FuelTank);
}
