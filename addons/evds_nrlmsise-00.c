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
/// @page EVDS_Callback_NRLMSISE00 NRLMSISE-00 Earth atmospheric model
///
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdio.h>
#include <evds.h>
#include <nrlmsise-00.h>
#include "evds_nrlmsise-00.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Callback that returns atmospheric data according to NRLMSISE-00
////////////////////////////////////////////////////////////////////////////////
int EVDS_NRLMSISE_00_GetAtmosphericData(EVDS_OBJECT* earth, EVDS_VECTOR* r, EVDS_ENVIRONMENT_ATMOSPHERE* atmosphere) {
	int i;
	EVDS_REAL value;
	EVDS_VARIABLE* variable;

	struct nrlmsise_output output;
	struct nrlmsise_input input;
	struct nrlmsise_flags flags;
	struct ap_array aph;
	EVDS_REAL latitude,longitude,altitude;

	//Read position
	EVDS_Vector_ToGeographicCoordinates(earth,r,&latitude,&longitude,&altitude);

	//Setup input for the model
	//input.year = 1900+cur_time->tm_year;
	//input.doy = cur_time->tm_yday;
	//input.sec = cur_time->tm_sec+cur_time->tm_min*60+cur_time->tm_hour*3600;
	input.alt = altitude*1e-3;
	input.g_lat = latitude;
	input.g_long = longitude;
	//input.lst = input.sec/3600.0 + input.g_long/15.0;

	//Read AP indexes
	for (i = 0; i < 7; i++) {
		char variable_name[256];
		sprintf(variable_name,"nrlmsise-00_ap%d",i);

		aph.a[i] = 4.0;
		EVDS_Object_GetRealVariable(earth,variable_name,&value,&variable);
		if (variable) aph.a[i] = value;
	}
	input.ap = aph.a[0];
	input.ap_a = &aph;

	//Read f107/f107a
	input.f107 = 150.0;
	input.f107A = 150.0;
	EVDS_Object_GetRealVariable(earth,"nrlmsise-00_f107",&value,&variable);
	if (variable) input.f107 = value;
	EVDS_Object_GetRealVariable(earth,"nrlmsise-00_f107a",&value,&variable);
	if (variable) input.f107A = value;

	//Setup switches
	for (i = 0; i < 24; i++) flags.switches[i] = 1;
	flags.switches[0] = 0; //Output data in meters
	flags.switches[9] = -1; //Use ap_a array

	//Execute correct model
	if (altitude < 200000) { //Mass density
		gtd7(&input, &flags, &output);
	} else { //Effective density
		gtd7d(&input, &flags, &output);
	}

	//Read back
	atmosphere->density = output.d[5]*1e3;
	atmosphere->pressure = 287*output.t[1]*output.d[5]*1e3;
	atmosphere->temperature = output.t[1];
	/*v->air.density_He	= 1e-3*output.d[0]*4.0/6.022e23;
	v->air.density_O	= 1e-3*output.d[1]*16.0/6.022e23;
	v->air.density_N2	= 1e-3*output.d[2]*28.0/6.022e23;
	v->air.density_O2	= 1e-3*output.d[3]*32.0/6.022e23;
	v->air.density_Ar	= 1e-3*output.d[4]*40.0/6.022e23;
	v->air.density_H	= 1e-3*output.d[6]*1.0/6.022e23;
	v->air.density_N	= 1e-3*output.d[7]*14.0/6.022e23;
	v->air.density		= output.d[5]*1e3;
	v->air.concentration_He	= output.d[0];
	v->air.concentration_O	= output.d[1];
	v->air.concentration_N2	= output.d[2];
	v->air.concentration_O2	= output.d[3];
	v->air.concentration_Ar	= output.d[4];
	v->air.concentration_H	= output.d[6];
	v->air.concentration_N	= output.d[7];
	v->air.concentration = output.d[0]+output.d[1]+output.d[2]+
	                       output.d[3]+output.d[4]+output.d[6]+
	                       output.d[7];

	v->air.temperature = output.t[1];
	v->air.exospheric_temperature = output.t[0];
	v->air.pressure = 287*output.t[1]*output.d[5]*1e3; //P = (R[air]T)/d

	vmag = sqrt(v->sim.vx*v->sim.vx+v->sim.vy*v->sim.vy+v->sim.vz*v->sim.vz);
	v->air.Q = (0.5)*vmag*vmag*v->air.density;*/
	return EVDS_OK;
}