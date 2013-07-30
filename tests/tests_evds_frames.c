#include "framework.h"

void Test_EVDS_FRAMES() {
	START_TEST("Geodetic coordinates (around vessel)") {
		EVDS_GEODETIC_COORDIANTE geocoord = { 0 };
		EVDS_GEODETIC_COORDIANTE target;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Vessel\" type=\"vessel\" />"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		//Distance from vessel of 1 km
		EVDS_Geodetic_Set(&geocoord,object,0,0,1000);

		//Check different coordinates
		geocoord.latitude = 90.0; //Elevation
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		EQUAL_TO(vector.coordinate_system,object);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
	} END_TEST


	START_TEST("Geodetic coordinates (spheric planet)") {
		EVDS_GEODETIC_COORDIANTE geocoord = { 0 };
		EVDS_GEODETIC_COORDIANTE target;
		EVDS_OBJECT* earth;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.radius\">6378145.0</parameter>" //Spherical planet
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);

		//Check different coordinates
		geocoord.latitude = 90.0; //North pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,6378145.0+1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		EQUAL_TO(vector.coordinate_system,earth);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-6378145.0-1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,6378145.0+1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,6378145.0+1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-6378145.0-1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-6378145.0-1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
	} END_TEST


	START_TEST("Geodetic coordinates (oblate planet)") {
		EVDS_GEODETIC_COORDIANTE geocoord = { 0 };
		EVDS_GEODETIC_COORDIANTE target;
		EVDS_OBJECT* earth;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.semimajor_axis\">6378137.0</parameter>"
"		<parameter name=\"geometry.semiminor_axis\">6356752.0</parameter>"
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);

		//Check different coordinates
		geocoord.latitude = 90.0; //North pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,6356752.0+1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
		EQUAL_TO(vector.coordinate_system,earth);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-6356752.0-1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,6378137.0+1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,6378137.0+1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-6378137.0-1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-6378137.0-1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
	} END_TEST



	START_TEST("Geodetic coordinates (oblate planet, numerical stability)") {
		EVDS_GEODETIC_COORDIANTE geocoord = { 0 };
		EVDS_GEODETIC_COORDIANTE target;
		EVDS_OBJECT* earth;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.semimajor_axis\">6378137.0</parameter>"
"		<parameter name=\"geometry.semiminor_axis\">6356752.0</parameter>"
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);

		//Check numerical stability
		geocoord.latitude = EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = EVDS_EPS;
		geocoord.longitude = EVDS_EPS;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 0.0;
		geocoord.longitude = EVDS_EPS;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		geocoord.latitude = 90.0+100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,90.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,-180.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0-100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,-90.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,-180.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);
	} END_TEST
}