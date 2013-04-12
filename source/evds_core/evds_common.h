////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief External Vessel Dynamics Simulator - Common Objects
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2013, Black Phoenix
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///   - Redistributions of source code must retain the above copyright
///     notice, this list of conditions and the following disclaimer.
///   - Redistributions in binary form must reproduce the above copyright
///     notice, this list of conditions and the following disclaimer in the
///     documentation and/or other materials provided with the distribution.
///   - Neither the name of the author nor the names of the contributors may
///     be used to endorse or promote products derived from this software without
///     specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
/// - @subpage EVDS_Solver_FuelTank "'fuel_tank'":
///		Default fuel tank object, provides basic model of a fuel tank with
///		a variable center of mass and sloshing support.
/// - @subpage EVDS_Solver_Planet "'planet'"
///		Planet or moon with built-in coordinate systems. Planetary positions must
///		be updated manually from an ephemeris - otherwise their motion will be
///		physically simulated.
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
// Get the stability frame for this vessel
//EVDS_API int EVDS_RigidBody_GetStabilityReferenceFrame(EVDS_OBJECT* object, EVDS_OBJECT* frame);

// Rocket engine (creates force along it's forward axis)
EVDS_API int EVDS_RocketEngine_Register(EVDS_SYSTEM* system);

// Fuel tank (stores fuel)
EVDS_API int EVDS_FuelTank_Register(EVDS_SYSTEM* system);


// Planet (represents a planetary body or a star)
//EVDS_API int EVDS_Planet_Register(EVDS_SYSTEM* system);


// Heun propagator-corrector solver
EVDS_API int EVDS_Propagator_Heun_Register(EVDS_SYSTEM* system);
// Runge-Kutta 4th order propagator
EVDS_API int EVDS_Propagator_RK4_Register(EVDS_SYSTEM* system);


// Load default databases
EVDS_API int EVDS_Common_LoadDatabase(EVDS_SYSTEM* system);


////////////////////////////////////////////////////////////////////////////////
/// Register all common objects
#define EVDS_Common_Register(system) \
EVDS_RigidBody_Register(system); \
EVDS_RocketEngine_Register(system); \
EVDS_FuelTank_Register(system); \
EVDS_Propagator_Heun_Register(system); \
EVDS_Propagator_RK4_Register(system);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
#endif