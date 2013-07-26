#include "framework.h"

void Test_EVDS_MODIFIER() {
	START_TEST("Linear modifier test") {
		EVDS_OBJECT* root;
		EVDS_OBJECT* container;
		ERROR_CHECK(EVDS_System_GetRootInertialSpace(system,&root));
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Modifier\" type=\"modifier\">"
"        <parameter name=\"pattern\">linear</parameter>"
"        <parameter name=\"vector1.count\">5</parameter>"
"        <parameter name=\"vector2.count\">5</parameter>"
"        <parameter name=\"vector3.count\">5</parameter>"
"        <parameter name=\"vector1.x\">1</parameter>"
"        <parameter name=\"vector2.y\">1</parameter>"
"        <parameter name=\"vector3.z\">1</parameter>"
"        <object name=\"Object\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Modifier (Children)",0,&container), EVDS_OK);
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,container);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 0.0, 0.0, 0.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x1x1)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 4.0, 0.0, 0.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x5x1)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 0.0, 4.0, 0.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x5x1)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 4.0, 4.0, 0.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x1x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 0.0, 0.0, 4.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x1x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 4.0, 0.0, 4.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x5x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 0.0, 4.0, 4.0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x5x5)",container,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 4.0, 4.0, 4.0);
	} END_TEST

	START_TEST("Circular modifier test") {
	} END_TEST

	START_TEST("Pattern modifier test") {
	} END_TEST
}