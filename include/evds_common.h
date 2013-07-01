////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief External Vessel Dynamics Simulator - Common Objects
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
#ifndef EVDS_COMMON_H
#define EVDS_COMMON_H
#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_Objects Common Objects
///
/// The following object types are available in EVDS by default:
/// - @subpage EVDS_Solver_RigidBody "'vessel'", "'rigid_body'", "'static_body'":
///		Default rigid body object, accumulates forces and torques from children
///		objects, affected by gravity and other effects (drag, reentry heating, etc).
/// - @subpage EVDS_Solver_RocketEngine "'rocket_engine'":
///		Default rocket engine object, supports a variety of rocket engine
///		parameters. Calculates physics from shape or shape from physics parameters.
/// - @subpage EVDS_Solver_Gimbal "'gimbal'"
///		Gimballing platform that allows controlled movement/rotation of children
///		objects.
/// - @subpage EVDS_Solver_FuelTank "'fuel_tank'":
///		Default fuel tank object, provides basic model of a fuel tank with
///		a variable center of mass and sloshing support.
/// - @subpage EVDS_Solver_Planet "'planet'"
///		Planet or moon with built-in coordinate systems. Planetary position can be
///		updated from an ephemeris, orbital information, or be physically simulated.
/// - @subpage EVDS_Solver_Wiring "'wire'", "'wire.node'", "'wire.connector'"
///		Wiring or piping connecting nodes and connectors.
///
///
/// The following propagators are available:
/// - @subpage EVDS_Propagator_RK4 "'propagator_rk4'":
///		Runge-Kutta 4th order numerical integration
/// - @subpage EVDS_Propagator_Heun "'propagator_heun'":
///		Heun's predictor-corrector numerical integration method
////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_Addons Addons
///
/// The following official addons are available for EVDS:
/// - @subpage EVDS_Solver_Antenna "'antenna'"
///		Radio antenna with various shapes and geometric parameters available. If
///		Realtime Digital Radio Simulator support is enabled, it can be used for
///		simulating the digital radio link within the current EVDS_SYSTEM.
/// - @subpage EVDS_Solver_Train_WheelsGeometry "'train_wheels'"
///		Temporary placeholder that only represents geometry of train wheels.
/// - @subpage EVDS_Callback_NRLMSISE_00
///		NRLMSISE-00 Earth atmospheric model callback for the EVDS_ENVIRONMENT API.
/// - @subpage EVDS_Callback_WMM
///		World Magnetic Model callbacks for magnetic model of Earth.
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_ADDONS Addons API
/// @brief API for optional EVDS addons.
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_COMMON Common Objects API
/// @brief API for common objects (vessels, propagators, etc)
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Rigid body (moving object with mass), vessel (same as rigid body), vessel part (static object)
EVDS_API int EVDS_RigidBody_Register(EVDS_SYSTEM* system);
// Get center of mass of a rigid body
EVDS_API int EVDS_RigidBody_GetCenterOfMass(EVDS_OBJECT* object, EVDS_VECTOR* cm);
// Update all vessels and detach them if required. Must be called by user to support "detach" variable for vessels.
EVDS_API int EVDS_RigidBody_UpdateDetaching(EVDS_SYSTEM* system);

// Rocket engine (creates force along it's forward axis)
EVDS_API int EVDS_RocketEngine_Register(EVDS_SYSTEM* system);

// Fuel tank (stores fuel)
EVDS_API int EVDS_FuelTank_Register(EVDS_SYSTEM* system);

// Gimbal platform (allows turning position of set of objects in space)
EVDS_API int EVDS_Gimbal_Register(EVDS_SYSTEM* system);


// Planet (represents a planetary body or a star)
EVDS_API int EVDS_Planet_Register(EVDS_SYSTEM* system);
// Find the nearest planetary body
EVDS_API int EVDS_Planet_GetNearest(EVDS_OBJECT* object, EVDS_OBJECT** p_planet);


// Wiring, piping, connectors, etc
EVDS_API int EVDS_Wiring_Register(EVDS_SYSTEM* system);


// Forward euler propagator
EVDS_API int EVDS_Propagator_ForwardEuler_Register(EVDS_SYSTEM* system);
// Heun propagator-corrector solver
EVDS_API int EVDS_Propagator_Heun_Register(EVDS_SYSTEM* system);
// Runge-Kutta 4th order propagator
EVDS_API int EVDS_Propagator_RK4_Register(EVDS_SYSTEM* system);


// Check if material is oxidier
EVDS_API int EVDS_Material_IsOxidizer(EVDS_SYSTEM* system, const char* name);
// Check if material is fuel
EVDS_API int EVDS_Material_IsFuel(EVDS_SYSTEM* system, const char* name);
// Load default databases
EVDS_API int EVDS_Common_LoadDatabase(EVDS_SYSTEM* system);


////////////////////////////////////////////////////////////////////////////////
/// Register all common objects
#define EVDS_Common_Register(system) \
EVDS_RigidBody_Register(system); \
EVDS_RocketEngine_Register(system); \
EVDS_FuelTank_Register(system); \
EVDS_Gimbal_Register(system); \
EVDS_Planet_Register(system); \
EVDS_Wiring_Register(system); \
EVDS_Propagator_ForwardEuler_Register(system); \
EVDS_Propagator_Heun_Register(system); \
EVDS_Propagator_RK4_Register(system);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
#endif