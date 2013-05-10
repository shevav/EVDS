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
/// @page EVDS_Solver_Antenna Radio Antenna
///
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <evds.h>
#include <rdrs.h>
#include "evds_antenna.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Generate geometry for the antenna
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalAntenna_GenerateGeometry(EVDS_OBJECT* object) {
	EVDS_VARIABLE* geometry;
	EVDS_VARIABLE* variable;

	//Default values
	EVDS_REAL size = 0;
	char antenna_type[257] = { 0 };

	//Reset the cross-sections
	if (EVDS_Object_GetVariable(object,"geometry.cross_sections",&geometry) == EVDS_OK) {
		EVDS_Variable_Destroy(geometry);
	}
	EVDS_Object_AddVariable(object,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&geometry);

	//Get antenna parameters
	if (EVDS_Object_GetVariable(object,"antenna_type",&variable) == EVDS_OK)		EVDS_Variable_GetString(variable,antenna_type,256,0);
	if (EVDS_Object_GetVariable(object,"size",&variable) == EVDS_OK)				EVDS_Variable_GetReal(variable,&size);

	//Generate correct geometry
	if (1) {
		EVDS_VARIABLE* dipole_start_tip;
		EVDS_VARIABLE* dipole_start;
		EVDS_VARIABLE* dipole_end;
		EVDS_VARIABLE* dipole_end_tip;

		//Add cross-sections
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&dipole_start_tip);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&dipole_start);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&dipole_end);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&dipole_end_tip);

		//Radius
		EVDS_Variable_AddFloatAttribute(dipole_start_tip,	"rx",0.00,0);
		EVDS_Variable_AddFloatAttribute(dipole_start,		"rx",0.01,0);
		EVDS_Variable_AddFloatAttribute(dipole_end,			"rx",0.01,0);
		EVDS_Variable_AddFloatAttribute(dipole_end_tip,		"rx",0.00,0);

		//Offsets
		EVDS_Variable_AddFloatAttribute(dipole_start_tip,	"add_offset",0.0,0);
		EVDS_Variable_AddFloatAttribute(dipole_start,		"add_offset",0.0,0);
		EVDS_Variable_AddFloatAttribute(dipole_end,			"add_offset",0.0,0);
		EVDS_Variable_AddFloatAttribute(dipole_end_tip,		"add_offset",0.0,0);

		EVDS_Variable_AddFloatAttribute(dipole_start_tip,	"offset",-size/2,0);
		EVDS_Variable_AddFloatAttribute(dipole_start,		"offset",-size/2,0);
		EVDS_Variable_AddFloatAttribute(dipole_end,			"offset", size/2,0);
		EVDS_Variable_AddFloatAttribute(dipole_end_tip,		"offset", size/2,0);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update engine internal state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalAntenna_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalAntenna_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_REAL size = 0;
	EVDS_REAL design_frequency = 0;
	EVDS_REAL frequency = 0;
	EVDS_REAL tx = 0;
	EVDS_REAL rx = 0;
	EVDS_REAL efficiency = 0;
	EVDS_REAL bandwidth = 0;
	EVDS_REAL data_rate = 0;
	EVDS_VARIABLE* variable;
	RDRS_ANTENNA* antenna;
	RDRS_CHANNEL channel;
	int flags = 0;

	//Check if the correct object
	if (EVDS_Object_CheckType(object,"antenna") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Make sure variables exist
	EVDS_Object_AddVariable(object,"size",EVDS_VARIABLE_TYPE_FLOAT,&variable);
	EVDS_Object_AddVariable(object,"frequency",EVDS_VARIABLE_TYPE_FLOAT,&variable);
	EVDS_Object_AddVariable(object,"tx",EVDS_VARIABLE_TYPE_FLOAT,&variable);
	EVDS_Object_AddVariable(object,"rx",EVDS_VARIABLE_TYPE_FLOAT,&variable);
	EVDS_Object_AddVariable(object,"efficiency",EVDS_VARIABLE_TYPE_FLOAT,&variable);
	EVDS_Object_AddVariable(object,"bandwidth",EVDS_VARIABLE_TYPE_FLOAT,&variable);
	EVDS_Object_AddVariable(object,"data_rate",EVDS_VARIABLE_TYPE_FLOAT,&variable);

	//Read antenna parameters
	if (EVDS_Object_GetVariable(object,"size",&variable) == EVDS_OK)				EVDS_Variable_GetReal(variable,&size);
	if (EVDS_Object_GetVariable(object,"design_frequency",&variable) == EVDS_OK)	EVDS_Variable_GetReal(variable,&design_frequency);
	if (EVDS_Object_GetVariable(object,"frequency",&variable) == EVDS_OK)			EVDS_Variable_GetReal(variable,&frequency);
	if (EVDS_Object_GetVariable(object,"tx",&variable) == EVDS_OK)					EVDS_Variable_GetReal(variable,&tx);
	if (EVDS_Object_GetVariable(object,"rx",&variable) == EVDS_OK)					EVDS_Variable_GetReal(variable,&rx);
	if (EVDS_Object_GetVariable(object,"efficiency",&variable) == EVDS_OK)			EVDS_Variable_GetReal(variable,&efficiency);
	if (EVDS_Object_GetVariable(object,"bandwidth",&variable) == EVDS_OK)			EVDS_Variable_GetReal(variable,&bandwidth);
	if (EVDS_Object_GetVariable(object,"data_rate",&variable) == EVDS_OK)			EVDS_Variable_GetReal(variable,&data_rate);

	//Calculate antenna size from design frequency
	if (design_frequency > 0.0) {
		size = 299792458.0/(1e6*design_frequency);
	}
	if (size < 0.0) size = 0.0;
	EVDS_Object_AddFloatVariable(object,"size",size,0);

	//Sanity check for frequency
	if (frequency <= 0.0) frequency = 299792458.0/(1e6*size);

	//Create anntena object
	channel.frequency = frequency;
	channel.bandwidth = bandwidth;
	channel.data_rate = data_rate;
	channel.modulation = RDRS_CHANNEL_BPSK;

	if (tx < 0.5) flags |= RDRS_ANTENNA_NO_SEND;
	if (rx < 0.5) flags |= RDRS_ANTENNA_NO_RECEIVE;
	RDRS_Antenna_Create((RDRS_SYSTEM*)solver->userdata,&antenna,channel,flags);
	RDRS_Antenna_SetSize(antenna,size);
	EVDS_Object_SetSolverdata(object,(void*)antenna);

	//Generate geometry for the antenna
	EVDS_InternalAntenna_GenerateGeometry(object);
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize engine solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalAntenna_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	RDRS_ANTENNA* antenna;
	EVDS_Object_GetSolverdata(object,(void**)&antenna);
	RDRS_Antenna_Destroy(antenna);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize RDRS instance
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalAntenna_Startup(EVDS_SYSTEM* system, EVDS_SOLVER* solver) {
	RDRS_SYSTEM* rdrs_system;

	RDRS_System_Create(&rdrs_system);
	RDRS_System_SetRealTime(rdrs_system,1);
	RDRS_System_Start(rdrs_system);
	solver->userdata = rdrs_system;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Denitialize RDRS instance
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalAntenna_Shutdown(EVDS_SYSTEM* system, EVDS_SOLVER* solver) {
	RDRS_SYSTEM* rdrs_system = solver->userdata;
	RDRS_System_Destroy(rdrs_system);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Antenna = {
	EVDS_InternalAntenna_Initialize, //OnInitialize
	EVDS_InternalAntenna_Deinitialize, //OnDeinitialize
	EVDS_InternalAntenna_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	EVDS_InternalAntenna_Startup, //OnStartup
	EVDS_InternalAntenna_Shutdown, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register radio antenna solver.
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Antenna_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Antenna);
}
