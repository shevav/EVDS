////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief External Vessel Dynamics Simulator (internal data structures)
/////////////////////////////////////////////////////////////////////////////////
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
#ifndef EVDS_INTERNAL_H
#define EVDS_INTERNAL_H
#ifdef __cplusplus
extern "C" {
#endif

/// Internal macro to check function for errors
#define EVDS_ERRCHECK(expr) { int error_code = expr; if (error_code != EVDS_OK) return error_code; }

/// Compatibility with Windows systems
#ifdef _WIN32
#define snprintf _snprintf
#define snscanf _snscanf
#define alloca _alloca
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_VARIABLE
/// @struct EVDS_VARIABLE
/// @brief Single variable inside an object or an entry in a data structure.
///
/// Variables can only be created before object is initialized. Here's an example:
/// ~~~{.c}
///		EVDS_OBJECT* object;
///		EVDS_VARIABLE* variable;
///		EVDS_Object_Create(system,parent,&object);
///		EVDS_Object_AddVariable(object,"center_of_mass",EVDS_VARIABLE_TYPE_VECTOR,&variable);
///		EVDS_Object_AddFloatVariable(object,"mass",5000.0,&variable);
/// ~~~
///
/// If variable already exists when adding, the function simply returns that variable (and does not change
/// its value or create a new one).
///
/// All work with variables (before object is initialized) must be done from the initializing thread.
/// If the initalizing thread must be changed in runtime (for example to pass an uninitialized object
/// to another thread) the EVDS_Object_TransferInitialization() call can be used.
///
/// A variable may have one of the following types:
/// Type								| Description
/// ------------------------------------|--------------------------------------
/// @c EVDS_VARIABLE_TYPE_FLOAT			| Floating point value
/// @c EVDS_VARIABLE_TYPE_STRING		| Null-terminated string
/// @c EVDS_VARIABLE_TYPE_VECTOR		| Vector
/// @c EVDS_VARIABLE_TYPE_QUATERNION	| Quaternion
/// @c EVDS_VARIABLE_TYPE_NESTED		| Data structure (contains a list of nested variables, list of attributes)
/// @c EVDS_VARIABLE_TYPE_DATA			| Stores pointer to custom data (some C structure)
/// @c EVDS_VARIABLE_TYPE_FUNCTION		| Stores a function/callback pointer. Function signature depends on variable name
///
/// The EVDS_VARIABLE_TYPE_NESTED variable type may contain nested variables and
/// nested attributes inside it. This variable type is used to load complex data structures
/// from the XML files. The best approach is to create a new variable of EVDS_VARIABLE_TYPE_DATA type,
/// which will store parsed data from EVDS_VARIABLE_TYPE_NESTED in the vessel initializer.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_VARIABLE_TABLE1D_ENTRY_TAG {
	EVDS_REAL x;						//X value
	EVDS_REAL f;						//Variable value
} EVDS_VARIABLE_TABLE1D_ENTRY;  

typedef struct EVDS_VARIABLE_TABLE1D_TAG {
	EVDS_REAL constant;					//Constant representing this table
	EVDS_VARIABLE_TABLE1D_ENTRY* data;	//Table of values
	int count;							//Size of the values table

	EVDS_REAL data_min;					//Min X value in the table
	EVDS_REAL data_max;					//Max X value in the table
	EVDS_REAL data_length;				//= data_max - data_min
	EVDS_REAL data_length1;				//= 1.0/data_length

	EVDS_REAL* data_fast;				//Pre-calculated table of interpolated values
	int data_fast_count;				//Size of the pre-calculated table
} EVDS_VARIABLE_TABLE1D;

struct EVDS_VARIABLE_TAG {
#ifndef EVDS_SINGLETHREADED
	SIMC_LOCK_ID lock;					//Read/write lock (only for non-float variables)
#endif

	char name[64];						//Parameter name
	EVDS_VARIABLE_TYPE type;			//Variable type
	void* value;						//Variable value
	size_t value_size;					//Size of variable (size of string if string variable)

	//EVDS_OBJECT* modifier;				//Reference to modifier this variable belongs to
	//EVDS_VARIABLE* source_variable;		//Source variable, used instead of original one if the object is not unique

	SIMC_LIST* attributes;				//Attributes of this variable (for arbitrary data only)
	SIMC_LIST* list;					//List of nested variables
	SIMC_LIST_ENTRY* attribute_entry;	//Entry in parent variables attributes list
	SIMC_LIST_ENTRY* list_entry;		//Entry in parent variables nested list (or objects variables list)

	EVDS_VARIABLE* parent;				//Variable this variable belongs to (if nested)
	EVDS_OBJECT* object;				//Object this parameter belongs to (0 if not a parameter)
	EVDS_SYSTEM* system;				//System this variable belongs to

	// User-defined data
	void* userdata;
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_OBJECT
/// @struct EVDS_OBJECT
/// @brief Single object or a coordinate system.
///
/// EVDS object can be used to define:
///  - Propagators (coordinate systems in which children states are propagated)
///  - Vessels/moving objects
///  - Planets and planet-like bodies
///  - Objects which create forces (engines, wings)
///  - Objects which have special meaning for their parent objects (IVSS objects)
///
/// The object behavior depends on its parent. Some objects are only meaningful
/// when their parent is some specific object type (for example engine nozzles will
/// only affect behavior of the engine object, and do not have any behavior otherwise).
///
/// Objects are distinct by their type and name.
/// Two objects may not share the same name.
/// Each object has an unique identifier that can be defined by user at any time.
/// It is preferred that this identifier stays unique amongst objects, but it can only
/// be enforced by the user. By default objects are created with no unique identifier.
///
/// See EVDS_Object_SetName(), EVDS_Object_SetType(), EVDS_Object_SetUID() for more information
/// about object names, types, unique identifiers.
///
/// Object may contain a pointer to user data (any data structure user wishes to
/// attach to the object), and a pointer to solver data (any data structure that was
/// assigned to the object by the solver). Both features work the same way, but
/// solver data is used exclusively by the objects solver:
/// ~~~{.c}
///		void* userdata = malloc(128);
///		EVDS_Object_SetUserdata(object,userdata);
/// ~~~
///
/// Using objects in multithreaded envrionment:
///  - Objects can be created and destroyed in any threads
///  - A single object must not be deleted from more than one thread at once
///  - Objects must be reference counted explicitly by the application using
///     EVDS_Object_Store() and EVDS_Object_Release()
///  - Object data will be retained in memory until no more references to it exist,
///     and physically removed after EVDS_System_CleanupObjects() is called
///  - Objects must be initialized after loading. Initialization and all operations which
///     alter list of variables must be done within the initializing thread.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_OBJECT_TAG {
	//Unique ID (numeric identifier for the object)
	unsigned int uid;						//00000 - 99999 reserved for normal vessels

	// Object coordinates and state in space
	EVDS_STATE_VECTOR state;
	// Previous state vector (used for interpolation when rendering)
	EVDS_STATE_VECTOR previous_state;
	// State in which object must be rendered
	EVDS_STATE_VECTOR render_state;

	// Public object state, used by functions which are not the integrating thread
#ifndef EVDS_SINGLETHREADED
	SIMC_SRW_ID state_lock;
	SIMC_SRW_ID previous_state_lock;
	EVDS_STATE_VECTOR private_state;
	SIMC_THREAD_ID integrate_thread;		//Thread that's integrating position 
											// (makes transformations use "state" and not "public_state")
	SIMC_THREAD_ID render_thread;			//Rendering thread (overrides coordinate conversions)
#endif

	// Fixed object information
	char name[256];							//Object name
	char type[256];							//Object type
	EVDS_OBJECT* parent;					//Objects parent
	EVDS_SOLVER* solver;					//Objects solver
	EVDS_SYSTEM* system;					//Objects system
	int parent_level;						//How many nodes away from root (0 for root)
	//EVDS_MODIFIER* modifier;				//Reference to modifier this object belongs to (if generated by one)
	//int unique_modified;					//Object existed before modifier was applied
	//EVDS_OBJECT* source;					//Object that this object is based on, if created via modifier

	// Object variables/parameters
	SIMC_LIST* variables;					//List of variables
	SIMC_LIST* children;					//Children objects
	SIMC_LIST* raw_children;				//Children objects (raw list, including the uninitialized ones)

	// Initialization-related information
	int initialized;						//Is object initialized
#ifndef EVDS_SINGLETHREADED
	SIMC_THREAD_ID initialize_thread;		//Thread that performs initialization
	SIMC_THREAD_ID create_thread;			//Thread in which object was created
#endif

	// Information for destroying the object
#ifndef EVDS_SINGLETHREADED
	int stored_counter;						//Instance counter (how many times object was stored elsewhere)
	int destroyed;							//Object is destroyed and must be removed from storage ASAP
#endif
	SIMC_LIST_ENTRY* object_entry;			//Entry in "system->objects" linked list (used for removing it from list)
	SIMC_LIST_ENTRY* parent_entry;			//Entry in "parent->children" linked list (used for removing it from list)
	SIMC_LIST_ENTRY* rparent_entry;			//Entry in "parent->raw_children" linked list (used for removing it from list)
	SIMC_LIST_ENTRY* type_entry;			//Entry in "type_list" linked list (used for removing it from list)
	SIMC_LIST* type_list;					//List in which type is stored (or 0)

	// Callbacks
	EVDS_Callback_Solve*		solve;		//Solve object/step state forward
	EVDS_Callback_Integrate*	integrate;	//Return derivative of state vector for integration

	// User-defined data
	void* userdata;
	void* solverdata;
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_SYSTEM
/// @struct EVDS_SYSTEM
/// @brief Object that represents a self-contained system of objects.
///
/// The EVDS system represents one instance of the simulator. It contains a tree
/// of EVDS_OBJECT data structures (which represent objects and coordinate systems),
/// and a set of EVDS_SOLVER data structures (which represent the different object types
/// and object behaviors).
///
/// EVDS system always contains a single "root" object, which represents the inertial space
/// of the simulated universe.
///
/// The tree of objects must not contain any loops. The API does not provide checks against loops,
/// creating a loop (e.g. root node is a child of an object that's a child of root node) will put
/// the program into undefined state.
///
/// Destroying the EVDS_SYSTEM object will clean up all relevant data, including
/// all EVDS_OBJECT data structures. The following data will not be cleaned up:
///  - Custom data structures within object variables (must be cleaned up by solvers)
///  - All userdata objects must be cleaned up manually by their user
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_INTERNAL_TYPE_ENTRY_TAG {
	char type[256];			//Type name
	SIMC_LIST* objects;		//List of objects with this type
} EVDS_INTERNAL_TYPE_ENTRY;

struct EVDS_SYSTEM_TAG {
	// Object data management
#ifndef EVDS_SINGLETHREADED
	SIMC_LOCK_ID cleanup_working;				// Delete thread working
	SIMC_LIST* deleted_objects;					// List of deleted objects
#endif
	SIMC_LIST* objects;							// List of objects
	SIMC_LIST* object_types;					// List of object types

	// Other lists
	SIMC_LIST* solvers;							// List of solvers
	SIMC_LIST* databases;						// List of databases (each an EVDS_VARIABLE)

	// Global callbacks
	EVDS_Callback_Initialize* OnInitialize;		// Global initialization callback

	// Unique ID counter (for objects without a defined UID)
	unsigned int uid_counter;

	// Root inertial space
	EVDS_OBJECT* inertial_space;

	// User-defined data
	void* userdata;
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_SYSTEM
/// @struct EVDS_MATERIAL
/// @brief Single material database entry
///
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_MATERIAL_TAG {
	char name[256];			//Material name
	char print_name[256];	//Material print name
	SIMC_LIST* parameters;	//Materials parameters
};
#endif




////////////////////////////////////////////////////////////////////////////////
// Internal API
////////////////////////////////////////////////////////////////////////////////
// Destroy object internal data
int EVDS_InternalObject_DestroyData(EVDS_OBJECT* object);
// Destroy variable internal data
int EVDS_InternalVariable_DestroyData(EVDS_VARIABLE* variable);
// Creates a new variable
int EVDS_Variable_Create(EVDS_SYSTEM* system, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable);
// Creates a new variable as a copy of existing one
int EVDS_Variable_Copy(EVDS_VARIABLE* source, EVDS_VARIABLE* variable);
// Destroy a material
int EVDS_Material_Destroy(EVDS_MATERIAL* material);

#ifndef EVDS_SINGLETHREADED
// Set private state vector
int EVDS_InternalObject_SetPrivateStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector);
#endif

#ifdef __cplusplus
}
#endif
#endif
