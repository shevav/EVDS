#include "framework.h"

void Test_EVDS_MODIFIER() {
	START_TEST("Linear modifier test") {
		/// This test verifies that linear modifier creates children copies,
		/// and that these copies are named correctly and present at their correct spots.
		EVDS_OBJECT* container;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Modifier\" type=\"modifier\">"
"        <parameter name=\"pattern\">linear</parameter>"
"        <parameter name=\"vector1.count\">5</parameter>"
"        <parameter name=\"vector2.count\">5</parameter>"
"        <parameter name=\"vector3.count\">5</parameter>"
"        <parameter name=\"vector1.x\">1</parameter>"
"        <parameter name=\"vector2.y\">2</parameter>"
"        <parameter name=\"vector3.z\">3</parameter>"
"        <object name=\"Object\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Check if original object becomes part of the modifiers children container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Modifier (Children)",0,&container), EVDS_OK);
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,container);
		EQUAL_TO(container->initialized,1);

		/// Check some key objects that must be part of the container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*0, 2*0, 3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x1x1)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*4, 2*0, 3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x5x1)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*0, 2*4, 3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x5x1)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*4, 2*4, 3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x1x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*0, 2*0, 3*4);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x1x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*4, 2*0, 3*4);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x5x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*0, 2*4, 3*4);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x5x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 1*4, 2*4, 3*4);
	} END_TEST

	START_TEST("Circular modifier test") {
		/// This test does same as for linear modifier, but now using circular modifier.
	} END_TEST

	START_TEST("Pattern modifier test") {
		/// This test verifies that pattern modifier creates copies of children according to
		/// predefined set of points.
	} END_TEST
}




void Test_EVDS_GIMBAL() {
	START_TEST("Parameterless gimbal platform") {
		/// Test gimbal platform with default parameters. Such platform will instantly
		/// respond to its commands.
		EVDS_REAL x,y,z;
		EVDS_OBJECT* platform;
		EVDS_VARIABLE* pitch_command;
		EVDS_VARIABLE* yaw_command;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Gimbal\" type=\"gimbal\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <object name=\"Rocket nozzle\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Check that platform exists, and mass of the platform equals to mass specified for gimbal object
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Gimbal (Platform)",0,&platform), EVDS_OK);
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Rocket nozzle",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,platform);
		EQUAL_TO(platform->initialized,1);
		ERROR_CHECK(EVDS_Object_GetVariable(platform,"mass",&variable));
		ERROR_CHECK(EVDS_Variable_GetReal(variable,&real));
		REAL_EQUAL_TO(real,1000.0);

		/// Get variables corresponding to platform commands
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Gimbal",0,&object));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"pitch.command",&pitch_command));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"yaw.command",&yaw_command));

		/// Get nozzle and make sure its initialized
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Rocket nozzle",platform,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);

		/// Check response to commands. Reset gimbal to stationary position
		ERROR_CHECK(EVDS_Object_Solve(root,0.0)); //Initialize state
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		EQUAL_TO(x,EVDS_RAD(0.0));
		EQUAL_TO(y,EVDS_RAD(0.0));
		EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check command response on pitch (must be instant)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));

		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,-90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(-90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check command response on yaw (must be instant)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(90.0));

		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,-90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(-90.0));

		/// Check command response on mixed channels (must be instant)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(90.0));

		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,-90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,-90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(-90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(-90.0));
	} END_TEST


	START_TEST("Gimbal platform behavior") {
		/// Test gimbal platform behavior. This test verifies that finite discrete states
		/// for platform deflection, min & max limits, platform movement rates all work.
		EVDS_REAL t,x,y,z;
		EVDS_OBJECT* platform;
		EVDS_VARIABLE* pitch_command;
		EVDS_VARIABLE* yaw_command;
		EVDS_VARIABLE* pitch_current;
		EVDS_VARIABLE* yaw_current;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Gimbal\" type=\"gimbal\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"pitch.min\">-10</parameter>"
"        <parameter name=\"pitch.max\">10</parameter>"
"        <parameter name=\"pitch.rate\">10</parameter>"
"        <parameter name=\"pitch.bits\">8</parameter>"
"        <parameter name=\"yaw.min\">-10</parameter>"
"        <parameter name=\"yaw.max\">10</parameter>"
"        <parameter name=\"yaw.rate\">20</parameter>"
"        <parameter name=\"yaw.bits\">16</parameter>"
"        <object name=\"Rocket nozzle\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Setup all for testing
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Gimbal (Platform)",0,&platform));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Gimbal",0,&object));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"pitch.command",&pitch_command));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"yaw.command",&yaw_command));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"pitch.current",&pitch_current));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"yaw.current",&yaw_current));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Rocket nozzle",0,&object));

		/// Gimbal stationary (default command)
		ERROR_CHECK(EVDS_Object_Solve(root,0.0)); //Initialize state
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		EQUAL_TO(x,EVDS_RAD(0.0));
		EQUAL_TO(y,EVDS_RAD(0.0));
		EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check response to commands on pitch
		/// This test verifies that gimbal moves with predefined rate on pitch channel
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.5 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(5.0)); //Gimbal must be half-way down
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.6
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //1.0 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(10.0)); //Gimbal must be fully down
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));


		//Reset command (also test very long delta-times)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,100.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));


		/// Check response to commands on yaw
		/// This test verifies that gimbal moves with predefined rate on yaw channel
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,90.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.5 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(10.0)); //Gimbal must be fully right


		/// Reset command
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,100.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));


		/// Check precision in bits
		/// Out of 8 bits of precision only 255 different values are used.
		/// This allows both max and min values, as well as midpoint value to be properly represented
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,60*20.0/255.0 + 0.4*20.0/255.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //Must take no longer than 0.5 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(60*20.0/255.0)); //Actual angle snaps because of precision limits
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check both rounding up and down behavior (corresponds to internal transformation
		/// of raw commanded value into deflection angle assuming finite bits precision).
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,60*20.0/255.0 - 0.4*20.0/255.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(60*20.0/255.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));



		/// Reset command
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,100.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));



		/// Run same checks with yaw (16 bits precision)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,10000*20.0/65535.0 + 0.4*20.0/65535.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.05)); //Must take no longer than 0.25 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(10000*20.0/65535.0));

		/// Check both rounding up and down behavior (corresponds to internal transformation
		/// of raw commanded value into deflection angle assuming finite bits precision).
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,10000*20.0/65535.0 - 0.4*20.0/65535.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(10000*20.0/65535.0));
	} END_TEST
}