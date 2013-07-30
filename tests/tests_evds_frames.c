#include "framework.h"

void Test_EVDS_FRAMES() {
	START_TEST("Geodetic coordinates (spheric planet)") {
		EVDS_GEODETIC_COORDIANTE geocoord = { 0 };
		EVDS_OBJECT* root;
		EVDS_OBJECT* earth;
		ERROR_CHECK(EVDS_System_GetRootInertialSpace(system,&root));
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.radius\">6378145.0</parameter>" //Spherical planet
"		<object name=\"Object\" type=\"static_body\" />"
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		geocoord.elevation = 1000.0;

		//Check different coordinates
		geocoord.latitude = 90.0; //North pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,6378145.0+1000.0,1e-7);
		EQUAL_TO(vector.coordinate_system,earth);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-6378145.0-1000.0,1e-7);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,6378145.0+1000.0,0,0,1e-7);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,6378145.0+1000.0,0,1e-7);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = 180.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-6378145.0-1000.0,0,0,1e-7);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-6378145.0-1000.0,0,1e-7);
	} END_TEST


	START_TEST("Geodetic coordinates (oblate planet)") {
		EVDS_GEODETIC_COORDIANTE geocoord = { 0 };
		EVDS_OBJECT* root;
		EVDS_OBJECT* earth;
		ERROR_CHECK(EVDS_System_GetRootInertialSpace(system,&root));
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.semimajor_axis\">6378137.0</parameter>"
"		<parameter name=\"geometry.semiminor_axis\">6356752.0</parameter>"
"		<object name=\"Object\" type=\"static_body\" />"
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		geocoord.elevation = 1000.0;

		//Check different coordinates
		geocoord.latitude = 90.0; //North pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,6356752.0+1000.0,1e-7);
		EQUAL_TO(vector.coordinate_system,earth);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-6356752.0-1000.0,1e-7);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,6378137.0+1000.0,0,0,1e-7);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,6378137.0+1000.0,0,1e-7);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = 180.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-6378137.0-1000.0,0,0,1e-7);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(earth,&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-6378137.0-1000.0,0,1e-7);
	} END_TEST
}