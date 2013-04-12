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
/// @page EVDS_Propagator_RK4 Runge-Kutta 4th order Propagator
////////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief RK4 integration method
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_RK4_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL h) {
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;

	//Process all children
	EVDS_Object_GetChildren(coordinate_system,&children);
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		//Initial state, final derivative:
		EVDS_STATE_VECTOR state;							//Initial state (t = 0)
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative;		//Derivative at initial state
		//Variables for RK4:
		EVDS_STATE_VECTOR state_temporary;					//Used in calculations
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_1;	//Four derivatives for RK4
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_2;
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_3;
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_4;
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		// Solve everything inside the child
		if (EVDS_Object_Solve(object,h) != EVDS_OK) {
			// In case there is an error move to the next object in list.
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}

		// Get initial state vector
		EVDS_Object_GetStateVector(object,&state);
		
		// f1 = f(0,y)
		EVDS_Object_Integrate(object,0.0,&state,&state_derivative_1);

		// f2 = f(t+0.5*h,y+0.5*h*f1)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_1,0.5*h);
		EVDS_Object_Integrate(object,0.5*h,&state_temporary,&state_derivative_2);

		// f3 = f(t+0.5h,y+0.5*h*f2)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_2,0.5*h);
		EVDS_Object_Integrate(object,0.5*h,&state_temporary,&state_derivative_3);

		// f4 = f(t+h,y+h*f3)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_3,h);
		EVDS_Object_Integrate(object,h,&state_temporary,&state_derivative_4);

		// state = state + h*(1/6 f1 + 1/3 f2 + 1/3 f3 + 1/6 f4)
		EVDS_StateVector_Derivative_Initialize(&state_derivative,coordinate_system);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_1,1.0/6.0);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_2,1.0/3.0);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_3,1.0/3.0);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_4,1.0/6.0);

		// Calculate new final state
		EVDS_StateVector_MultiplyByTimeAndAdd(&state,&state,&state_derivative,h);

		// Update object state vector
		EVDS_Object_SetStateVector(object,&state);

		//Move to next object in list
		entry = SIMC_List_GetNext(children,entry);
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize propagator
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_RK4_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"propagator_rk4") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Propagator_RK4 = {
	EVDS_InternalPropagator_RK4_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPropagator_RK4_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register RK4 propagator solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Propagator_RK4_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Propagator_RK4);
}
