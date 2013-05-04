////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief External Vessel Dynamics Simulator - OpenGL Renderer
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
#ifndef EVDS_OPENGL_H
#define EVDS_OPENGL_H
#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_OpenGL OpenGL Rendering
///
/// (text)
////////////////////////////////////////////////////////////////////////////////

int EVDS_OpenGL_Initialize(EVDS_OPENGL_STATE** p_state);
int EVDS_OpenGL_Destroy(EVDS_OPENGL_STATE* state);

int EVDS_OpenGL_SetupViewport(EVDS_OPENGL_STATE* state);
int EVDS_OpenGL_RenderObject(EVDS_OPENGL_STATE* state, EVDS_OBJECT* object);

int EVDS_OpenGL_PushTransformation(EVDS_OPENGL_STATE* state, EVDS_OBJECT* object);
int EVDS_OpenGL_PopTransformation(EVDS_OPENGL_STATE* state);


////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_OPENGL OpenGL Rendering API
/// @brief API for EVDS OpenGL rendering
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
#endif
