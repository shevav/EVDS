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
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "evds.h"

// List of supported cross-section types
#define CSECTION_ELLIPSE	0
#define CSECTION_RECTANGLE	1
#define CSECTION_NGON		2

// Structure that stores cross-section variables (for all types), just for the quick lookup
#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_INTERNALMESH_ATTRIBUTES_TAG {
	//Shared between all
	int type;
	EVDS_REAL offset;
	EVDS_REAL offset_x;
	EVDS_REAL offset_y;
	EVDS_REAL absolute;
	EVDS_REAL thickness;
	EVDS_REAL continuous;

	EVDS_REAL tangent_offset_pos;
	EVDS_REAL tangent_offset_neg;
	EVDS_REAL tangent_radial_pos;
	EVDS_REAL tangent_radial_neg;

	//ELLIPSE, RECTANGLE
	EVDS_REAL rx;
	EVDS_REAL ry;

	//NGON
	EVDS_REAL n,phi;
} EVDS_INTERNALMESH_ATTRIBUTES;
#endif


////////////////////////////////////////////////////////////////////////////////
/// @brief Get attributes from the cross-section (and its parent properties too)
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalMesh_GetAttributes(EVDS_VARIABLE* cross_section, EVDS_INTERNALMESH_ATTRIBUTES* attributes) {
	char type[256] = { 0 };
	EVDS_VARIABLE* v;
	memset(attributes,0,sizeof(EVDS_INTERNALMESH_ATTRIBUTES));

	//Get type
	strcpy(type,""); attributes->type = CSECTION_ELLIPSE;
	if (EVDS_Variable_GetAttribute(cross_section,"type",&v) == EVDS_OK) EVDS_Variable_GetString(v,type,256,0);
	if (strncmp(type,"rectangle",256) == 0) attributes->type = CSECTION_RECTANGLE;
	if (strncmp(type,"ngon",256) == 0) attributes->type = CSECTION_NGON;

	//Get variables
	if (EVDS_Variable_GetAttribute(cross_section,"offset",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->offset);
	if (EVDS_Variable_GetAttribute(cross_section,"offset.x",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->offset_x);
	if (EVDS_Variable_GetAttribute(cross_section,"offset.y",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->offset_y);
	if (EVDS_Variable_GetAttribute(cross_section,"absolute",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->absolute);
	if (EVDS_Variable_GetAttribute(cross_section,"thickness",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->thickness);
	if (EVDS_Variable_GetAttribute(cross_section,"continuous",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->continuous);

	if (EVDS_Variable_GetAttribute(cross_section,"tangent.offset.pos",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->tangent_offset_pos);
	if (EVDS_Variable_GetAttribute(cross_section,"tangent.offset.neg",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->tangent_offset_neg);
	if (EVDS_Variable_GetAttribute(cross_section,"tangent.radial.pos",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->tangent_radial_pos);
	if (EVDS_Variable_GetAttribute(cross_section,"tangent.radial.neg",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->tangent_radial_neg);

	if (EVDS_Variable_GetAttribute(cross_section,"r",&v) == EVDS_OK) { //Special variable for easier manual input
		EVDS_VARIABLE* a;
		EVDS_REAL value;
		EVDS_Variable_GetReal(v,&value);

		if (EVDS_Variable_AddAttribute(cross_section,"rx",EVDS_VARIABLE_TYPE_FLOAT,&a) == EVDS_OK) EVDS_Variable_SetReal(a,value);
		if (EVDS_Variable_AddAttribute(cross_section,"ry",EVDS_VARIABLE_TYPE_FLOAT,&a) == EVDS_OK) EVDS_Variable_SetReal(a,value);
	}
	if (EVDS_Variable_GetAttribute(cross_section,"rx",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->rx);
	if (EVDS_Variable_GetAttribute(cross_section,"ry",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->ry);
	else { //Fix for old saves which were missing "ry" variable
		EVDS_VARIABLE* a;
		if (EVDS_Variable_AddAttribute(cross_section,"ry",EVDS_VARIABLE_TYPE_FLOAT,&a) == EVDS_OK) EVDS_Variable_SetReal(a,attributes->rx);
	}

	if (EVDS_Variable_GetAttribute(cross_section,"n",&v) == EVDS_OK)   EVDS_Variable_GetReal(v,&attributes->n);
	if (EVDS_Variable_GetAttribute(cross_section,"phi",&v) == EVDS_OK) EVDS_Variable_GetReal(v,&attributes->phi);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Generate point on the cross-sections plane.
///
/// Returns X and Y offsets relative to cross-section guiding line, local smoothing
/// group number.
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalMesh_GetPoint(EVDS_INTERNALMESH_ATTRIBUTES* attributes, float time, float* x, float* y, int* group) {
	switch (attributes->type) {
		case CSECTION_ELLIPSE: {
			float phi = time*2.0f*EVDS_PIf;
			*x = (float)attributes->rx*cosf(phi);
			*y = (float)attributes->ry*sinf(phi);
			*group = 0;
		} break;
		case CSECTION_RECTANGLE: {
			float phi = time*2.0f*EVDS_PIf;
			float n = 4.0f;
			float r = 1.0f/cosf(fmodf(0.25f*EVDS_PIf+phi,2.0f*EVDS_PIf/n) - EVDS_PIf/n); //cosf(EVDS_PIf/n)
			*x = (float)attributes->rx*0.5f*r*cosf(phi);
			*y = (float)attributes->ry*0.5f*r*sinf(phi);
			*group = (int)((time+0.125f-EVDS_EPSf)*4.0f);
			if (*group >= 4) *group = 0;
		} break;
		case CSECTION_NGON: {
			float phi = time*2.0f*EVDS_PIf;
			float n = (float)((int)attributes->n);
			float dphi = (float)(EVDS_RAD(attributes->phi));
			float r;
			if (n < 3.0) n = 3.0;
			if (dphi < 0) dphi = 0;
			r = cosf(EVDS_PIf/n)/cosf(fmodf(dphi+phi,2.0f*EVDS_PIf/n) - EVDS_PIf/n);

			*x = (float)attributes->rx*r*cosf(phi);
			*y = (float)attributes->rx*r*sinf(phi);
			//*group = (int)(time*n);
			*group = (int)(((phi + dphi) / (2.0*EVDS_PIf))*n - EVDS_EPSf);
			if (*group >= n) *group = 0;
		} break;
		default: {
			*x = 0;
			*y = 0;
		} break;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Add a new vertex
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalMesh_AddVertex(EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info, EVDS_MESH_VECTOR* v, 
								int cross_section, int smoothing_group) {
	int index; 

	//Check if storage must be grown
	if (mesh->num_vertices == mesh->internal->num_vertices_allocated) {
		if (mesh->internal->num_vertices_allocated < 16384) {
			mesh->internal->num_vertices_allocated *= 2;
		} else {
			mesh->internal->num_vertices_allocated += 16384;
		}
		mesh->vertices = (EVDS_MESH_VECTOR*)realloc(mesh->vertices,sizeof(EVDS_MESH_VECTOR)*mesh->internal->num_vertices_allocated);
		if (!(info->flags & EVDS_MESH_SKIP_VERTEX_INFO)) {
			mesh->vertex_info = (EVDS_MESH_VERTEX_INFO*)realloc(mesh->vertex_info,sizeof(EVDS_MESH_VERTEX_INFO)*mesh->internal->num_vertices_allocated);
		}
	}

	//Update smoothing groups counter
	if (smoothing_group > mesh->internal->max_smoothing_group) mesh->internal->max_smoothing_group = smoothing_group;

	//Update bounding box
	if (v->x < mesh->bbox_min.x) mesh->bbox_min.x = v->x;
	if (v->x > mesh->bbox_max.x) mesh->bbox_max.x = v->x;
	if (v->y < mesh->bbox_min.y) mesh->bbox_min.y = v->y;
	if (v->y > mesh->bbox_max.y) mesh->bbox_max.y = v->y;
	if (v->z < mesh->bbox_min.z) mesh->bbox_min.z = v->z;
	if (v->z > mesh->bbox_max.z) mesh->bbox_max.z = v->z;

	//Store vertex and return new index
	index = mesh->num_vertices;
	memcpy(&mesh->vertices[index],v,sizeof(EVDS_MESH_VECTOR));
	if (!(info->flags & EVDS_MESH_SKIP_VERTEX_INFO)) {
		mesh->vertex_info[index].num_allocated = 16;
		mesh->vertex_info[index].num_triangles = 0;
		mesh->vertex_info[index].triangles = (EVDS_MESH_INDEX*)malloc(sizeof(EVDS_MESH_INDEX)*mesh->vertex_info[index].num_allocated);
		mesh->vertex_info[index].tri_index = (EVDS_MESH_INDEX*)malloc(sizeof(EVDS_MESH_INDEX)*mesh->vertex_info[index].num_allocated);
		mesh->vertex_info[index].cross_section = cross_section;
		mesh->vertex_info[index].smoothing_group = smoothing_group;
	}
	mesh->num_vertices++;
	return index;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Add a new index
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalMesh_AddIndex(EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info, 
							   EVDS_MESH_INDEX tri, EVDS_MESH_INDEX v_id, EVDS_MESH_INDEX v) {
	int index,v_index; 

	//Check if storage must be grown
	if (mesh->num_indices == mesh->internal->num_indices_allocated) {
		if (mesh->internal->num_indices_allocated < 16384) {
			mesh->internal->num_indices_allocated *= 2;
		} else {
			mesh->internal->num_indices_allocated += 16384;
		}
		mesh->indices = (EVDS_MESH_INDEX*)realloc(mesh->indices,sizeof(EVDS_MESH_INDEX)*mesh->internal->num_indices_allocated);
	}

	//Store vertex and return new index
	index = mesh->num_indices;
	mesh->indices[index] = v;	
	mesh->num_indices++;

	//Add this triangle to vertex
	if (!(info->flags & EVDS_MESH_SKIP_VERTEX_INFO)) {
		if (mesh->vertex_info[v].num_triangles == mesh->vertex_info[v].num_allocated) {
			if (mesh->vertex_info[v].num_allocated < 64) {
				mesh->vertex_info[v].num_allocated *= 2;
			} else {
				mesh->vertex_info[v].num_allocated += 64;
			}

			mesh->vertex_info[v].triangles = (EVDS_MESH_INDEX*)realloc(mesh->vertex_info[v].triangles,
				sizeof(EVDS_MESH_INDEX)*mesh->vertex_info[v].num_allocated);
			mesh->vertex_info[v].tri_index = (EVDS_MESH_INDEX*)realloc(mesh->vertex_info[v].tri_index,
				sizeof(EVDS_MESH_INDEX)*mesh->vertex_info[v].num_allocated);
		}
		v_index = mesh->vertex_info[v].num_triangles;
		mesh->vertex_info[v].triangles[v_index] = tri;
		mesh->vertex_info[v].tri_index[v_index] = v_id;
		mesh->vertex_info[v].num_triangles++;
	}
	return index;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Add a new triangle
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalMesh_AddTriangle(EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info, 
								  EVDS_MESH_INDEX* pv1, EVDS_MESH_INDEX* pv2, EVDS_MESH_INDEX* pv3,
								  int cross_section, int smoothing_group) {
	int index;
	int v1 = *pv1;
	int v2 = *pv2;
	int v3 = *pv3;

	//Check if storage must be grown
	if (mesh->num_triangles == mesh->internal->num_triangles_allocated) {
		if (mesh->internal->num_triangles_allocated < 16384) {
			mesh->internal->num_triangles_allocated *= 2;
		} else {
			mesh->internal->num_triangles_allocated += 16384;
		}
		mesh->triangles = (EVDS_MESH_TRIANGLE*)realloc(mesh->triangles,sizeof(EVDS_MESH_TRIANGLE)*mesh->internal->num_triangles_allocated);
	}

	//Update smoothing groups counter
	if (smoothing_group > mesh->internal->max_smoothing_group) mesh->internal->max_smoothing_group = smoothing_group;

	//Store vertex and return new index
	index = mesh->num_triangles;
	memcpy(&mesh->triangles[index].vertex[0],&mesh->vertices[v1],sizeof(EVDS_MESH_VECTOR));
	memcpy(&mesh->triangles[index].vertex[1],&mesh->vertices[v2],sizeof(EVDS_MESH_VECTOR));
	memcpy(&mesh->triangles[index].vertex[2],&mesh->vertices[v3],sizeof(EVDS_MESH_VECTOR));

	//Make sure all vertices are of same smoothing group, otherwise create additional ones
	if (mesh->vertex_info[v1].smoothing_group != smoothing_group) {
		v1 = EVDS_InternalMesh_AddVertex(mesh,info,&mesh->triangles[index].vertex[0],cross_section,smoothing_group);
	}
	if (mesh->vertex_info[v2].smoothing_group != smoothing_group) {
		v2 = EVDS_InternalMesh_AddVertex(mesh,info,&mesh->triangles[index].vertex[1],cross_section,smoothing_group);
	}
	if (mesh->vertex_info[v3].smoothing_group != smoothing_group) {
		v3 = EVDS_InternalMesh_AddVertex(mesh,info,&mesh->triangles[index].vertex[2],cross_section,smoothing_group);
	}

	//Store new (possibly modified) indices
	mesh->triangles[index].indices[0] = v1;
	mesh->triangles[index].indices[1] = v2;
	mesh->triangles[index].indices[2] = v3;
	mesh->triangles[index].cross_section = cross_section;
	mesh->triangles[index].smoothing_group = smoothing_group;
	mesh->num_triangles++;

	//Add to array of indices
	EVDS_InternalMesh_AddIndex(mesh,info,index,0,v1);
	EVDS_InternalMesh_AddIndex(mesh,info,index,1,v2);
	EVDS_InternalMesh_AddIndex(mesh,info,index,2,v3);
	*pv1 = v1;
	*pv2 = v2;
	*pv3 = v3;
	return index;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get optimal segment count
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalMesh_CrossSections_NumSegments(SIMC_LIST* cross_sections_list, float resolution, int* num_segments) {
	SIMC_LIST_ENTRY* entry;
	EVDS_INTERNALMESH_ATTRIBUTES attributes;
	float average_radius; //Average radius of the body
	int average_radius_ctr; //Counter for average radius

	//Reset averages
	average_radius = 0.0;
	average_radius_ctr = 0;

	//Count average cross-section radius
	entry = SIMC_List_GetFirst(cross_sections_list);
	while (entry) {
		int group;
		float x,y,r1,r2,r3,r4;
		EVDS_VARIABLE* cross_section = (EVDS_VARIABLE*)SIMC_List_GetData(cross_sections_list,entry);
		EVDS_InternalMesh_GetAttributes(cross_section,&attributes);

		//Calculate four points on edges
		EVDS_InternalMesh_GetPoint(&attributes,0.00,&x,&y,&group); //Right
		r1 = sqrtf(x*x+y*y);
		EVDS_InternalMesh_GetPoint(&attributes,0.25,&x,&y,&group); //Top
		r2 = sqrtf(x*x+y*y);
		EVDS_InternalMesh_GetPoint(&attributes,0.50,&x,&y,&group); //Left
		r3 = sqrtf(x*x+y*y);
		EVDS_InternalMesh_GetPoint(&attributes,0.75,&x,&y,&group); //Bottom
		r4 = sqrtf(x*x+y*y);

		//Accumulate
		average_radius *= average_radius_ctr;
		average_radius += r1+r2+r3+r4;
		average_radius_ctr += 4;
		average_radius /= average_radius_ctr;

		entry = SIMC_List_GetNext(cross_sections_list,entry);
	}

	//Determine number of segments
	*num_segments = (int)((float)(EVDS_PI)*average_radius/resolution + 0.5f);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Generate mesh that is built from cross-section information in the object
///
/// For every pair of cross-sections, generate enough triangles to interpolate between
/// these sections with given resolution.
///
/// The resolution is computed as following:
///  - For sectors (number of vertexes in a circle): resolution of average-radius circle
///  - For segments (number of divisions by length): new segment according to average of
///    curve length over four edgepoints (left, right, top, bottom).
///
/// FIXME: the cross-sections must be calculated as 3D objects rather than 2D, around
/// a single guiding line.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalMesh_CrossSections(EVDS_OBJECT* object, EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info) {
	SIMC_LIST_ENTRY* entry;
	SIMC_LIST* cross_sections_list;
	EVDS_VARIABLE* geometry;
	EVDS_VARIABLE* cross_section;

	int index = 0; //Index of cross-section
	float offset = 0.0; //Current offset through cross-sections (along the main axis)	
	int num_segments; //Number of segments (points on cross-section)
	EVDS_VARIABLE* previous_cross_section = 0;
	EVDS_INTERNALMESH_ATTRIBUTES attributes;
	EVDS_INTERNALMESH_ATTRIBUTES previous_attributes;

	EVDS_MESH_INDEX* previous_indices = 0; //Indices of previous cross-section
	int v1,v2,v3; //Vertices in a triangle strip (outer shell)

	//Prepare to accumulate total volume
	mesh->total_volume = 0.0f;

	//Get list of cross-sections
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"geometry.cross_sections",&geometry));
	EVDS_ERRCHECK(EVDS_Variable_GetList(geometry,&cross_sections_list));

	//Find average body radius
	if (!(info->flags & EVDS_MESH_FORCE_NUMSEGMENTS)) {
		EVDS_InternalMesh_CrossSections_NumSegments(cross_sections_list,info->resolution,&num_segments);
	} else {
		num_segments = info->num_segments;
	}

	//Make sure invalid mesh is not generated
	if (num_segments < 4) num_segments = 4;

	//Previous indices are the ones that tessellator must tie to when there is no rough change of normal
	// between cross-sections
	previous_indices = (EVDS_MESH_INDEX*)malloc(sizeof(EVDS_MESH_INDEX)*num_segments);
	for (v1 = 0; v1 < num_segments; v1++) previous_indices[v1] = 0xFFFFFFFF; //Index not present
	v1 = -1; v2 = -1; v3 = -1;

	//Iterate through all cross-sections
	entry = SIMC_List_GetFirst(cross_sections_list);
	while (entry) {
		int group;
		float x1,y1,x2,y2,length,new_offset;
		float correct_resolution;
		int num_sections,i,j;
		int max_local_smoothing_group = 0; //How many smoothing groups do segments add

		//Get cross-section
		cross_section = (EVDS_VARIABLE*)SIMC_List_GetData(cross_sections_list,entry);
		if (!previous_cross_section) { //Start from second cross-section
			EVDS_InternalMesh_GetAttributes(cross_section,&previous_attributes);
			previous_cross_section = cross_section;
			offset = (float)previous_attributes.offset; //Reset offset
			entry = SIMC_List_GetNext(cross_sections_list,entry);
			index++;
			continue;
		}
		//Get attributes
		EVDS_InternalMesh_GetAttributes(cross_section,&attributes);
		
		//Determine length of curve and propagate offset
		if (attributes.absolute >= 0.5) { //Offset of cross-section absolute relative to origin
			length = fabsf((float)attributes.offset-offset);
			new_offset = (float)attributes.offset;
		} else { //Offset relative to previous cross-section
			length = (float)attributes.offset;
			new_offset = offset+(float)attributes.offset;
		}
		EVDS_InternalMesh_GetPoint(&previous_attributes,0.0,&x1,&y1,&group);
		EVDS_InternalMesh_GetPoint(&attributes,0.0,&x2,&y2,&group);
		length = sqrtf(length*length + (x2-x1)*(x2-x1)); //FIXME: must use length of bezier curve

		//Find correct resolution for a more or less uniform grid
		num_sections = (int)(0.5*(length/info->resolution + 0.5));
		if (num_sections > 16) num_sections = 16;
		if (num_sections < 1) num_sections = 1;
		correct_resolution = length / ((float)num_sections);

		//Generate all sections for two cross-sections
		for (i = 0; i < num_sections; i++) {
			float px1,py1,px2,py2; //Previous x1,y1,x2,y2 for determining total area/volume
			float section_area1 = 0.0; //Previous sections area
			float section_area2 = 0.0; //New sections area
			int i0; //First of new indices in a sector
			int i1; //First of the previous indices in a sector
			float t1,t2; //Section interpolation times
			float z1,z2,tz1,tz2,z_s,z_e; //Coordinates for bezier curves
			t1 = ((float)i) / ((float)num_sections); //Get interpolation times
			t2 = ((float)i+1) / ((float)num_sections);

			//Compute start and end of section (offset is part of the bezier curve)
			z1 = offset;
			z2 = new_offset;
			tz1 = z1 + (float)previous_attributes.tangent_offset_pos; //start tangent offset
			tz2 = z2 - (float)attributes.tangent_offset_neg; //end tangent offset
			z_s = powf(1-t1,3)*z1+3*t1*powf(1-t1,2)*tz1+3*(1-t1)*powf(t1,2)*tz2+powf(t1,3)*z2; //Start of section
			z_e = powf(1-t2,3)*z1+3*t2*powf(1-t2,2)*tz1+3*(1-t2)*powf(t2,2)*tz2+powf(t2,3)*z2; //End of section

			//Generate all segments for this section
			for (j = 0; j <= num_segments; j++) {
				EVDS_MESH_VECTOR v;
				int vA,vB;
				int group;
				float t; //Segment interpolation time
				float trx1,try1,trx2,try2; //Radial tangents
				float theta1,theta2; //Radial vector in polar coordinates
				float x_s,y_s,x_e,y_e; //Final positions
				t = ((float)j) / ((float)num_segments); //Get interpolation time

				//Small hack for interpolation time: make sure rectangle vertices are generated
				if (fabs(t - 0.125f) < 0.5f/((float)num_segments)) t = 0.125f;
				if (fabs(t - 0.375f) < 0.5f/((float)num_segments)) t = 0.375f;
				if (fabs(t - 0.625f) < 0.5f/((float)num_segments)) t = 0.625f;
				if (fabs(t - 0.875f) < 0.5f/((float)num_segments)) t = 0.875f;

				//Get points
				EVDS_InternalMesh_GetPoint(&previous_attributes,t,&x1,&y1,&group);
				EVDS_InternalMesh_GetPoint(&attributes,t,&x2,&y2,&group);

				//Count maximum smoothing group number and transform it from local to global
				if (group > max_local_smoothing_group) max_local_smoothing_group = group;
				//if (group2 > max_local_smoothing_group) max_local_smoothing_group = group2;
				group += mesh->num_smoothing_groups;
				//group2 += mesh->num_smoothing_groups;

				//Compute radial tangent
				theta1 = atan2f(y1,x1);
				theta2 = atan2f(y2,x2);
				if ((x1 == 0.0) && (y1 == 0.0)) theta1 = theta2;
				if ((x2 == 0.0) && (y2 == 0.0)) theta2 = theta1;
				if ((x1 == 0.0) && (y1 == 0.0) &&
					(x2 == 0.0) && (y2 == 0.0)) {
					theta1 = t*2.0f*EVDS_PIf;
					theta2 = theta1;
				}

				//Add cross-section offset
				x1 += (float)previous_attributes.offset_x;
				y1 += (float)previous_attributes.offset_y;
				x2 += (float)attributes.offset_x;
				y2 += (float)attributes.offset_y;

				//Compute radial tangent vectors
				trx1 = x1 + (float)previous_attributes.tangent_radial_pos*cosf(theta1);
				try1 = y1 + (float)previous_attributes.tangent_radial_pos*sinf(theta1);
				trx2 = x2 - (float)attributes.tangent_radial_neg*cosf(theta2);
				try2 = y2 - (float)attributes.tangent_radial_neg*sinf(theta2);

				//Bezier curve interpolation
				x_s = powf(1-t1,3)*x1+3*t1*powf(1-t1,2)*trx1+3*(1-t1)*powf(t1,2)*trx2+powf(t1,3)*x2;
				y_s = powf(1-t1,3)*y1+3*t1*powf(1-t1,2)*try1+3*(1-t1)*powf(t1,2)*try2+powf(t1,3)*y2;
				x_e = powf(1-t2,3)*x1+3*t2*powf(1-t2,2)*trx1+3*(1-t2)*powf(t2,2)*trx2+powf(t2,3)*x2;
				y_e = powf(1-t2,3)*y1+3*t2*powf(1-t2,2)*try1+3*(1-t2)*powf(t2,2)*try2+powf(t2,3)*y2;

				//Calculate area
				if (j > 0) {
					section_area1 += 0.5f*(px1*y_s - py1*x_s);
					section_area2 += 0.5f*(px2*y_e - py2*x_e);
				}
				px1 = x_s;
				py1 = y_s;
				px2 = x_e;
				py2 = y_e;

				//Add previous set of vertices (if not already added)
				if ((j != num_segments) && (previous_indices[j] == 0xFFFFFFFF)) {
					v.x = z_s;
					v.y = x_s;
					v.z = y_s;
					previous_indices[j] = EVDS_InternalMesh_AddVertex(mesh,info,&v,index,group);
				}

				//Add end vertex and define new vertices for the next triangle
				if (j == num_segments) {
					vB = i0;
					vA = i1;
				} else {
					v.x = z_e;
					v.y = x_e;
					v.z = y_e;
					vB = EVDS_InternalMesh_AddVertex(mesh,info,&v,index,group);

					//Store this index for next sector
					vA = previous_indices[j];
					previous_indices[j] = vB;

					//Remember first two indices to generate continuous mesh
					if (j == 0) {
						i0 = vB;
						i1 = vA;
					}
				}

				//Add two triangles to a triangle strip
				v3 = vB;
				if ((v1 >= 0) && (v2 >= 0) && (v3 >= 0)) EVDS_InternalMesh_AddTriangle(mesh,info,&v1,&v2,&v3,index,group);
				v1 = v2;
				v2 = v3;

				v3 = vA;
				if ((v1 >= 0) && (v2 >= 0) && (v3 >= 0)) EVDS_InternalMesh_AddTriangle(mesh,info,&v2,&v1,&v3,index,group);
				v1 = v2;
				v2 = v3;
			}

			//Reset triangle strip before going to next section. This avoids a degenerate triangle
			// which otherwise appears between last edge from previous section and the first edge
			// of the next section.
			v1 = -1; v2 = -1; v3 = -1;

			//Add to total volume
			mesh->total_volume += 0.5f*(section_area1+section_area2)*(z_e - z_s);
		}

		//Next cross-section may have a different or same smoothing group set
		if (attributes.continuous < 0.5) {
			mesh->num_smoothing_groups += 1 + max_local_smoothing_group;
		}

		//Move to next section
		offset = new_offset;
		previous_cross_section = cross_section;
		memcpy(&previous_attributes,&attributes,sizeof(EVDS_INTERNALMESH_ATTRIBUTES));
		index++;
		entry = SIMC_List_GetNext(cross_sections_list,entry);
	}

	//Return
	free(previous_indices);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Generate triangle information for a single triangle (area, normals, neighbours)
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalMesh_FinishTriangle(EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info, EVDS_MESH_TRIANGLE* tri) {
	float ax,ay,az;
	float bx,by,bz;
	int i;

	//Calculate normals for the triangle (not normalized yet)
	ax = tri->vertex[0].x - tri->vertex[1].x; //v0-v1
	ay = tri->vertex[0].y - tri->vertex[1].y;
	az = tri->vertex[0].z - tri->vertex[1].z;
	bx = tri->vertex[1].x - tri->vertex[2].x; //v1-v2
	by = tri->vertex[1].y - tri->vertex[2].y;
	bz = tri->vertex[1].z - tri->vertex[2].z;

	tri->triangle_normal.x = ay*bz-az*by; //(v0-v1) x (v1-v2)
	tri->triangle_normal.y = az*bx-ax*bz;
	tri->triangle_normal.z = ax*by-ay*bx;

	//Triangle center
	tri->center.x = (1.0f/3.0f)*(tri->vertex[0].x+tri->vertex[1].x+tri->vertex[2].x);
	tri->center.y = (1.0f/3.0f)*(tri->vertex[0].y+tri->vertex[1].y+tri->vertex[2].y);
	tri->center.z = (1.0f/3.0f)*(tri->vertex[0].z+tri->vertex[1].z+tri->vertex[2].z);

	//Calculate triangle area and normalize normal vector
	tri->area = 0.5f*sqrtf(tri->triangle_normal.x*tri->triangle_normal.x+
						   tri->triangle_normal.y*tri->triangle_normal.y+
						   tri->triangle_normal.z*tri->triangle_normal.z)+EVDS_EPSf;
	tri->triangle_normal.x /= 2.0f*tri->area;
	tri->triangle_normal.y /= 2.0f*tri->area;
	tri->triangle_normal.z /= 2.0f*tri->area;

	//Compute per-vertex normals as triangle normals (will be updated later)
	for (i = 0; i < 3; i++) {
		tri->normal[i].x = tri->triangle_normal.x;
		tri->normal[i].y = tri->triangle_normal.y;
		tri->normal[i].z = tri->triangle_normal.z;
	}		
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Generate triangle information (area, normals, neighbours)
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalMesh_FinishTriangles(EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info) {
	int i;
	mesh->total_area = 0.0;
	for (i = 0; i < mesh->num_triangles; i++) {
		EVDS_MESH_TRIANGLE* tri = &mesh->triangles[i];
		EVDS_InternalMesh_FinishTriangle(mesh,info,tri);
		mesh->total_area += tri->area;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Finish computing information for vertices
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalMesh_FinishVertices(EVDS_MESH* mesh, EVDS_MESH_GENERATEEX* info) {
	int i,j;
	if (info->flags & EVDS_MESH_SKIP_VERTICES) return;
	if (info->flags & EVDS_MESH_SKIP_VERTEX_NORMALS) return;
	if (info->flags & EVDS_MESH_SKIP_VERTEX_INFO) return;

	//Create every normal by averaging out normals of neighbouring triangles
	mesh->normals = (EVDS_MESH_VECTOR*)malloc(sizeof(EVDS_MESH_VECTOR)*mesh->num_vertices);
	for (i = 0; i < mesh->num_vertices; i++) {
		float mag;

		//Find normal
		mesh->normals[i].x = 0.0;
		mesh->normals[i].y = 0.0;
		mesh->normals[i].z = 0.0;
		for (j = 0; j < mesh->vertex_info[i].num_triangles; j++) {
			EVDS_MESH_TRIANGLE* tri = &mesh->triangles[mesh->vertex_info[i].triangles[j]];
			mesh->normals[i].x += tri->triangle_normal.x;
			mesh->normals[i].y += tri->triangle_normal.y;
			mesh->normals[i].z += tri->triangle_normal.z;
		}
		mesh->normals[i].x /= (float)mesh->vertex_info[i].num_triangles + EVDS_EPSf;
		mesh->normals[i].y /= (float)mesh->vertex_info[i].num_triangles + EVDS_EPSf;
		mesh->normals[i].z /= (float)mesh->vertex_info[i].num_triangles + EVDS_EPSf;

		//Normalize
		mag = sqrtf(mesh->normals[i].x*mesh->normals[i].x+
					mesh->normals[i].y*mesh->normals[i].y+
					mesh->normals[i].z*mesh->normals[i].z) + EVDS_EPSf;
		mesh->normals[i].x /= mag;
		mesh->normals[i].y /= mag;
		mesh->normals[i].z /= mag;
	}

	//Write normals into triangles
	for (i = 0; i < mesh->num_triangles; i++) {
		EVDS_MESH_TRIANGLE* tri = &mesh->triangles[i];
		for (j = 0; j < 3; j++) {
			memcpy(&tri->normal[j],&mesh->normals[tri->indices[j]],sizeof(EVDS_MESH_VECTOR));
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Generate full or partial 3D mesh for the given object.
///
/// This operation returns the procedural mesh that defines this object. If object is
/// defined by cross-sections, its mesh is returned directly.
/// If object is built from CSG (constructive solid geometry), the mesh returned is the
/// combined mesh for all children.
///
/// There is no inherent support for animation, but for animating objects mesh for every
/// object may be generated, and animated using the object state.
///
/// The following flags can be used:
/// Name								| Description
/// ------------------------------------|---------------------------
/// EVDS_MESH_SKIP_TRIANGLES			| Do not generate triangles
/// EVDS_MESH_SKIP_VERTICES				| Do not generate per-vertex information (normals, vertex info, list of vertices)
/// EVDS_MESH_SKIP_VERTEX_NORMALS		| Do not generate per-vertex normals
/// EVDS_MESH_SKIP_VERTEX_INFO			| Do not generate per-vertex information
/// EVDS_MESH_SKIP_INDICES				| Do not generate indices
/// EVDS_MESH_SKIP_EDGES				| Do not generate edge data
/// EVDS_MESH_NO_THICKNESS				| Do not generate additional triangles for thickness
/// EVDS_MESH_FORCE_NUMSEGMENTS			| Force number of segments when generating mesh by cross-sections
/// EVDS_MESH_USE_DIVISIONS				| Use number of divisions instead of resolution as a quality parameter
///
/// Some flags require other flags to be present, for example inner hull geometry will only
/// be generated when no other skip flags are set.
///
/// If @c EVDS_MESH_FORCE_NUMSEGMENTS flag is used, "info.num_segments" field must be set to a valid integer
/// value.
///
/// If @c EVDS_MESH_USE_DIVISIONS flag is used, the resolution will be interpreted as an estimate for number of
/// sections/segments (to return a nearly constant number of vertices for any size of input model). The real resolution
/// is determined from mesh bounding box and can be clamped using EVDS_MESH_GENERATEEX::min_resolution field.
///
/// @note The actual resolution with which mesh was generated will be written to EVDS_MESH_GENERATEEX::resolution field
///   if @c EVDS_MESH_USE_DIVISIONS is used.
///
/// @param[in] object Object, from which mesh must be generated
/// @param[out] p_mesh Pointer to EVDS_MESH will be written here
/// @param[in] info Additional information used for generating the mesh
///
/// @returns Error code, 3D object mesh
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for mesh
////////////////////////////////////////////////////////////////////////////////
int EVDS_Mesh_GenerateEx(EVDS_OBJECT* object, EVDS_MESH** p_mesh, EVDS_MESH_GENERATEEX* info) {
	EVDS_MESH* mesh;

	//Allocate mesh
	mesh = (EVDS_MESH*)malloc(sizeof(EVDS_MESH));
	if (!mesh) return EVDS_ERROR_MEMORY;
	*p_mesh = mesh;
	memset(mesh,0,sizeof(EVDS_MESH));
	mesh->object = object;

	//Calculate correct resolution
	if (info->flags & EVDS_MESH_USE_DIVISIONS) {
		float dx,dy,dz;
		float est = 0.0; //Estimated object size
		EVDS_MESH* bounding_mesh;

		//Find bounding box and largest dimension (FIXME: proper call to EVDS_Mesh_GenerateEx)
		EVDS_ERRCHECK(EVDS_Mesh_Generate(object,&bounding_mesh,EVDS_MESH_LOWEST_RESOLUTION,
			info->flags & (!EVDS_MESH_USE_DIVISIONS)));
		dx = bounding_mesh->bbox_max.x - bounding_mesh->bbox_min.x;
		dy = bounding_mesh->bbox_max.y - bounding_mesh->bbox_min.y;
		dz = bounding_mesh->bbox_max.z - bounding_mesh->bbox_min.z;
		est = sqrt(dx*dx+dy*dy+dz*dz);

		//Calculate resolution from number of divisions
		info->resolution = est / info->resolution;

		//Clamp resolution
		if (info->resolution < info->min_resolution) {
			info->resolution = info->min_resolution;
		}

		//Clean up bounding box mesh
		EVDS_Mesh_Destroy(bounding_mesh);
	}

	//Set invalid bounding box
	mesh->bbox_min.x =  1.0f/EVDS_EPSf;
	mesh->bbox_min.y =  1.0f/EVDS_EPSf;
	mesh->bbox_min.z =  1.0f/EVDS_EPSf;
	mesh->bbox_max.x = -1.0f/EVDS_EPSf;
	mesh->bbox_max.y = -1.0f/EVDS_EPSf;
	mesh->bbox_max.z = -1.0f/EVDS_EPSf;

	//Create internal mesh information
	mesh->internal = (EVDS_MESH_INTERNAL*)malloc(sizeof(EVDS_MESH_INTERNAL));

	//Allocate lists (FIXME: EVDS_ERROR_MEMORY checks)
	mesh->internal->num_triangles_allocated = 128;//65536; //128;
	mesh->triangles = (EVDS_MESH_TRIANGLE*)malloc(sizeof(EVDS_MESH_TRIANGLE)*mesh->internal->num_triangles_allocated);
	mesh->internal->num_indices_allocated = mesh->internal->num_triangles_allocated*3;
	mesh->indices = (EVDS_MESH_INDEX*)malloc(sizeof(EVDS_MESH_INDEX)*mesh->internal->num_indices_allocated);
	mesh->internal->num_vertices_allocated = mesh->internal->num_triangles_allocated*3;
	mesh->vertices = (EVDS_MESH_VECTOR*)malloc(sizeof(EVDS_MESH_VECTOR)*mesh->internal->num_vertices_allocated);
	mesh->vertex_info = (EVDS_MESH_VERTEX_INFO*)malloc(sizeof(EVDS_MESH_VERTEX_INFO)*mesh->internal->num_vertices_allocated);

	//Start counting smoothing groups
	mesh->internal->max_smoothing_group = 0;
	mesh->num_smoothing_groups = 0;

	//Generate mesh for this object
	EVDS_InternalMesh_CrossSections(object,mesh,info);

	//Finalize data
	EVDS_InternalMesh_FinishTriangles(mesh,info);
	EVDS_InternalMesh_FinishVertices(mesh,info);

	//Make sure all smoothing groups are accounted for
	mesh->num_smoothing_groups = mesh->internal->max_smoothing_group + 1;

	//Make sure bounding box is correct
	if (mesh->bbox_min.x > mesh->bbox_max.x) {
		mesh->bbox_min.x = 0.0f;
		mesh->bbox_max.x = 0.0f;
	}
	if (mesh->bbox_min.y > mesh->bbox_max.y) {
		mesh->bbox_min.y = 0.0f;
		mesh->bbox_max.y = 0.0f;
	}
	if (mesh->bbox_min.z > mesh->bbox_max.z) {
		mesh->bbox_min.z = 0.0f;
		mesh->bbox_max.z = 0.0f;
	}
	
	//Optimize data
	mesh->triangles = (EVDS_MESH_TRIANGLE*)realloc(mesh->triangles,sizeof(EVDS_MESH_TRIANGLE)*mesh->num_triangles);
	mesh->indices = (EVDS_MESH_INDEX*)realloc(mesh->indices,sizeof(EVDS_MESH_INDEX)*mesh->num_indices);
	mesh->vertices = (EVDS_MESH_VECTOR*)realloc(mesh->vertices,sizeof(EVDS_MESH_VECTOR)*mesh->num_vertices);
	mesh->vertex_info = (EVDS_MESH_VERTEX_INFO*)realloc(mesh->vertex_info,sizeof(EVDS_MESH_VERTEX_INFO)*mesh->num_vertices);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Generate a full 3D mesh for the given object.
///
/// See EVDS_Mesh_GenerateEx() for more information on generating meshes and use
/// of mesh generation flags.
///
/// @param[in] object Object, from which mesh must be generated
/// @param[out] p_mesh Pointer to EVDS_MESH will be written here
/// @param[in] resolution Mesh resolution (distance in meters between vertices)
/// @param[in] flags Mesh generation flags
///
/// @returns Error code, 3D object mesh
/// @retval EVDS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Mesh_Generate(EVDS_OBJECT* object, EVDS_MESH** p_mesh, float resolution, int flags) {
	EVDS_MESH_GENERATEEX info = { 0 };
	info.resolution = resolution;
	info.flags = flags;
	return EVDS_Mesh_GenerateEx(object,p_mesh,&info);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroys all resources related to this mesh.
///
/// @param[in] mesh Pointer to mesh that must be destroyed
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
////////////////////////////////////////////////////////////////////////////////
EVDS_API int EVDS_Mesh_Destroy(EVDS_MESH* mesh) {
	if (mesh->vertices) free(mesh->vertices);
	if (mesh->indices) free(mesh->indices);
	if (mesh->normals) free(mesh->normals);
	if (mesh->triangles) free(mesh->triangles);
	if (mesh->vertex_info) {
		int i;
		for (i = 0; i < mesh->num_vertices; i++) {
			free(mesh->vertex_info[i].tri_index);
			free(mesh->vertex_info[i].triangles);
		}
		free(mesh->vertex_info);
	}
	if (mesh->internal) free(mesh->internal);
	free(mesh);
	return EVDS_OK;
}