#include <stdio.h>
#include <math.h>
#include "evds.h"

/*void query_objects(EVDS_SYSTEM* system) {
	SIMC_LIST* list;
	SIMC_LIST_ENTRY* entry;
	
	EVDS_System_GetObjectsByType(system,"vessel",&list);
	entry = SIMC_List_GetFirst(list);
	while (entry) {
		unsigned int uid;
		EVDS_STATE_VECTOR state;
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
		EVDS_Object_GetUID(object,&uid);
		EVDS_Object_GetStateVector(object,&state);

		printf("\t[%5d]: pos = (%.2f; %.2f; %.2f)\n",
			uid,
			state.position.x,state.position.y,state.position.z);


		entry = SIMC_List_GetNext(list,entry);
	}
}

void main() {
	EVDS_SYSTEM* system;
	EVDS_OBJECT* root;
	EVDS_OBJECT* satellite;
	EVDS_MODIFIER* modifier;
	EVDS_VECTOR axis1,axis2;

	printf("Tutorial 5: Object Modifiers\n");
	EVDS_System_Create(&system);
	EVDS_Common_Register(system);

	//Create inertial sysetm
	EVDS_Object_Create(system,0,&root);
	EVDS_Object_Initialize(root,1);

	//Load satellite
	EVDS_Object_LoadFromString(root,
"<EVDS>"
"  <object type=\"vessel\" name=\"Satellite\">"
"    <parameter name=\"mass\">1000</parameter>"
"    <parameter name=\"ixx\">100</parameter>"
"    <parameter name=\"iyy\">1000</parameter>"
"    <parameter name=\"izz\">500</parameter>"
"    <parameter name=\"cm\">1.0 0.0 -0.5</parameter>"
"  </object>"
"</EVDS>",&satellite);
	EVDS_Object_Initialize(satellite,1);

	//Create a modifier
	EVDS_Modifier_Create(system,&modifier);
	EVDS_Vector_Set(&axis1,EVDS_VECTOR_POSITION,root,1.0,0.0,0.0);
	EVDS_Vector_Set(&axis2,EVDS_VECTOR_POSITION,root,0.0,0.5,0.0);
	EVDS_Modifier_SetVector(modifier,EVDS_MODIFIER_VECTOR_AXIS1,&axis1);
	EVDS_Modifier_SetVector(modifier,EVDS_MODIFIER_VECTOR_AXIS2,&axis2);
	EVDS_Modifier_SetParameter(modifier,EVDS_MODIFIER_PAREMETER_I,5);
	EVDS_Modifier_SetParameter(modifier,EVDS_MODIFIER_PAREMETER_J,3);
	EVDS_Modifier_AddObject(modifier,satellite);

	//Show how list of objects changes
	printf("Objects before modifier:\n");
	query_objects(system);

	EVDS_Modifier_Execute(modifier);
	printf("Objects after modifier:\n");
	query_objects(system);

	EVDS_Modifier_Cleanup(modifier);
	printf("Objects after cleanup:\n");
	query_objects(system);

	//Release memory (only required under multithreading mode)
	EVDS_System_CleanupObjects(system);
	
	EVDS_System_Destroy(system);
}*/
void main() { }
