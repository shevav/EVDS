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
/// @page EVDS_Solver_Train_WheelsGeometry Train wheels (geometry only)
///
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <evds.h>
#include "evds_train_wheels.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalTrain_WheelsGeometry_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_VARIABLE* geometry;
	EVDS_VARIABLE* variable;

	//Variable values
	EVDS_REAL gauge = 0.0;
	EVDS_REAL outer_diameter = 0.0;
	EVDS_REAL inner_diameter = 0.0;
	EVDS_REAL axle_diameter = 0.0;
	EVDS_REAL hub_diameter = 0.0;
	EVDS_REAL hub_height = 0.0;
	EVDS_REAL rim_height = 0.0;
	EVDS_REAL disk_thickness = 0.0;
	EVDS_REAL flange_thickness = 0.0;
	EVDS_REAL flange_height = 0.0;

	//Check if the correct object
	if (EVDS_Object_CheckType(object,"train_wheels") != EVDS_OK) return EVDS_IGNORE_OBJECT;

	//Reset the cross-sections
	if (EVDS_Object_GetVariable(object,"geometry.cross_sections",&geometry) == EVDS_OK) {
		EVDS_Variable_Destroy(geometry);
	}
	EVDS_Object_AddVariable(object,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&geometry);

	//Get parameters
	if (EVDS_Object_GetVariable(object,"gauge",&variable) == EVDS_OK)				EVDS_Variable_GetReal(variable,&gauge);
	if (EVDS_Object_GetVariable(object,"outer_diameter",&variable) == EVDS_OK)		EVDS_Variable_GetReal(variable,&outer_diameter);
	if (EVDS_Object_GetVariable(object,"inner_diameter",&variable) == EVDS_OK)		EVDS_Variable_GetReal(variable,&inner_diameter);
	if (EVDS_Object_GetVariable(object,"axle_diameter",&variable) == EVDS_OK)		EVDS_Variable_GetReal(variable,&axle_diameter);
	if (EVDS_Object_GetVariable(object,"hub_diameter",&variable) == EVDS_OK)		EVDS_Variable_GetReal(variable,&hub_diameter);
	if (EVDS_Object_GetVariable(object,"hub_height",&variable) == EVDS_OK)			EVDS_Variable_GetReal(variable,&hub_height);
	if (EVDS_Object_GetVariable(object,"rim_height",&variable) == EVDS_OK)			EVDS_Variable_GetReal(variable,&rim_height);
	if (EVDS_Object_GetVariable(object,"disk_thickness",&variable) == EVDS_OK)		EVDS_Variable_GetReal(variable,&disk_thickness);
	if (EVDS_Object_GetVariable(object,"flange_thickness",&variable) == EVDS_OK)	EVDS_Variable_GetReal(variable,&flange_thickness);
	if (EVDS_Object_GetVariable(object,"flange_height",&variable) == EVDS_OK)		EVDS_Variable_GetReal(variable,&flange_height);

	//Default values
	if (flange_thickness <= 0.0)	flange_thickness = 0.020;
	if (flange_height <= 0.0)		flange_height = 0.050;
	if (rim_height <= 0.0)			rim_height = 0.100;
	if (disk_thickness <= 0.0)		disk_thickness = 0.050;
	
	//Generate correct geometry
	{
		EVDS_VARIABLE* section;
		EVDS_REAL flange_taper = 0.02; //m

		//Wheel hub left
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2-hub_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		0,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2-hub_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		hub_diameter/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2-disk_thickness,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		hub_diameter/2,0);
		EVDS_Variable_AddFloatAttribute(section,"tangent.radial.pos",fabs(hub_diameter-inner_diameter)/2,0);

		//Wheel core left
		//EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		//EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		//EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2-disk_thickness,0);
		//EVDS_Variable_AddFloatAttribute(section,"r",		inner_diameter/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2-rim_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		inner_diameter/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2-rim_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2,0);

		//Wheel flange left
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Flange base
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2+flange_taper,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Flange outer rim
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2+flange_height,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Flange inner rim
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2+flange_thickness,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2+flange_height,0);


		//Axle
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Axle left
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	-gauge/2+flange_thickness,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		axle_diameter/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Axle right
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2-flange_thickness,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		axle_diameter/2,0);


		//Wheel flange right
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Flange inner rim
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2-flange_thickness,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2+flange_height,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Flange outer rim
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2+flange_height,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section); //Flange base
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2+flange_taper,0);

		//Wheel core right
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2+rim_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		outer_diameter/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2+rim_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		inner_diameter/2,0);
		//EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		//EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		//EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2+disk_thickness,0);
		//EVDS_Variable_AddFloatAttribute(section,"r",		inner_diameter/2,0);

		//Wheel hub right
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2+disk_thickness,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		hub_diameter/2,0);
		EVDS_Variable_AddFloatAttribute(section,"tangent.radial.neg",fabs(hub_diameter-inner_diameter)/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2+hub_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		hub_diameter/2,0);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&section);
		EVDS_Variable_AddFloatAttribute(section,"absolute",	1.0,0);
		EVDS_Variable_AddFloatAttribute(section,"offset",	gauge/2+hub_height,0);
		EVDS_Variable_AddFloatAttribute(section,"r",		0,0);
		
	}
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Train_WheelsGeometry = {
	EVDS_InternalTrain_WheelsGeometry_Initialize, //OnInitialize
	0, //OnDeinitialize
	0, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register train wheels geometry solver.
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Train_WheelsGeometry_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Train_WheelsGeometry);
}
