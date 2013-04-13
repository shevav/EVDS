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
/// @page EVDS_Solver_Planet Planetary Body
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize engine solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_Planet_GetNearest(EVDS_OBJECT* object, EVDS_OBJECT** p_planet) {
	EVDS_SYSTEM* system;
	SIMC_LIST_ENTRY* entry;
	SIMC_LIST* planets;

	//State vectors
	EVDS_REAL  nearest_distance = 1.0/EVDS_EPS;
	EVDS_OBJECT* nearest_planet = 0;
	EVDS_STATE_VECTOR state;
	EVDS_Object_GetStateVector(object,&state);

	//Get list of planets
	EVDS_Object_GetSystem(object,&system);
	EVDS_System_GetObjectsByType(system,"planet",&planets);
	entry = SIMC_List_GetFirst(planets);
	while (entry) {
		EVDS_REAL distance;
		EVDS_OBJECT* planet = (EVDS_OBJECT*)SIMC_List_GetData(planets,entry);

		//Get state vector
		EVDS_VECTOR direction;
		EVDS_STATE_VECTOR planet_state;
		EVDS_Object_GetStateVector(planet,&planet_state);

		//Calculate distance
		EVDS_Vector_Subtract(&direction,&planet_state.position,&state.position);
		EVDS_Vector_Dot(&distance,&direction,&direction);
		distance = sqrt(distance);

		//Check if smallest
		if (distance < nearest_distance) {
			nearest_distance = distance;
			nearest_planet = planet;
		}

		//Check next planet
		entry = SIMC_List_GetNext(planets,entry);
	}	

	//Write back planet
	if (nearest_planet) {
		*p_planet = nearest_planet;
		return EVDS_OK;
	} else {
		return EVDS_ERROR_NOT_FOUND;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update planet position and state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPlanet_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	//FIXME: Manual orbital calculations
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Output accelerations for the planet
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPlanet_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
								  EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	EVDS_REAL is_static;
	EVDS_Object_GetRealVariable(object,"is_static",&is_static,0);

	//Apply physics if planet is not static or updated via ephemeris
	if (is_static < 0.5) {
		//Update planets velocity
		EVDS_Vector_Copy(&derivative->velocity,&state->velocity);
		EVDS_Vector_Copy(&derivative->angular_velocity,&state->angular_velocity);

		//Calculate acceleration due to gravity
		EVDS_Environment_GetGravitationalField(system,&state->position,0,&derivative->acceleration);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPlanet_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"planet") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Add non-optional variables
	EVDS_Object_AddVariable(object,"is_static",EVDS_VARIABLE_TYPE_FLOAT,0);
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Planet = {
	EVDS_InternalPlanet_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPlanet_Solve, //OnSolve
	EVDS_InternalPlanet_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register planetary body
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Planet_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Planet);
}
