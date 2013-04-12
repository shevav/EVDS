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
/// @page EVDS_Propagator_Heun Heun Iterative Propagator
////////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Heun propagator-corrector solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_Heun_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL h) {
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;

	//Process all children
	EVDS_Object_GetChildren(coordinate_system,&children);
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		EVDS_REAL error,mag2;
		EVDS_VECTOR temporary;
		EVDS_STATE_VECTOR state_0; //t = 0
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_0;
		EVDS_STATE_VECTOR state_1; //t = h
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_1;
		EVDS_STATE_VECTOR state_1n; //(new state) t = h
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_0n; //(new derivative) t = 0
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		// Solve everything inside the child
		if (EVDS_Object_Solve(object,h) != EVDS_OK) {
			// In case there is an error (most likely object is not yet initialized)
			//  move to the next object in list.
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}
		EVDS_Vector_Initialize(temporary);

		// Get initial state vector
		EVDS_Object_GetStateVector(object,&state_0);

		// Calculate derivative at starting point (forward integration)
		EVDS_Object_Integrate(object,0.0,&state_0,&state_derivative_0);

		// Make a forward-integration estimate of final state (predictor)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_1,&state_0,&state_derivative_0,h);

		// Iterative integration
		error = 1e9;
		while (error > 1e-5) {
			// Calculate derivative in the final state
			EVDS_Object_Integrate(object,h,&state_1,&state_derivative_1);

			// Calculate new derivative at the starting point as average between two derivatives (corrector)
			//d0' = (d0 + d1) / 2
			EVDS_StateVector_Derivative_Initialize(&state_derivative_0n,coordinate_system);
			EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative_0n,&state_derivative_0n,&state_derivative_0,0.5);
			EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative_0n,&state_derivative_0n,&state_derivative_1,0.5);

			// Calculate new final state
			//s1' = s0 + d0' * dt
			EVDS_StateVector_MultiplyByTimeAndAdd(&state_1n,&state_0,&state_derivative_0n,h);

			// Estimate error (FIXME: better criteria)
			error = 0;
			EVDS_Vector_Subtract(&temporary,&state_1.position,&state_1n.position);
			EVDS_Vector_Dot(&mag2,&temporary,&temporary); error += mag2;
			EVDS_Vector_Subtract(&temporary,&state_1.velocity,&state_1n.velocity);
			EVDS_Vector_Dot(&mag2,&temporary,&temporary); error += mag2;
			error = sqrt(error);

			// Set new final state
			EVDS_StateVector_Copy(&state_1,&state_1n);
		}
	
		// Update object state vector
		EVDS_Object_SetStateVector(object,&state_1);

		//Move to next object in list
		entry = SIMC_List_GetNext(children,entry);
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize propagator
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_Heun_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"propagator_heun") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Propagator_Heun = {
	EVDS_InternalPropagator_Heun_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPropagator_Heun_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register Heun propagator solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Propagator_Heun_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Propagator_Heun);
}
