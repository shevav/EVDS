#include <GL/glfw.h>
#include <stdio.h>
#include <math.h>
#include "evds.h"

#include "../foxworks/fw_matrix.c"

int screen_width,screen_height;
void GLFWCALL glfw_resize(int width, int height) {
	screen_width = width;
	screen_height = height;
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glViewport(0,0,width,height);
}

void glfw_init() {
	glfwInit();
	glfwOpenWindow(1024,600,8,8,8,8,24,0,GLFW_WINDOW);
	glfwSetWindowSizeCallback(glfw_resize);
	glfwSwapInterval(0);
}

void glfw_setup_3d() {
	float fov = 30.0f;
	matrix lookat_matrix;
	matrix ogl_matrix;
	matrix perspective_matrix;

	matrix_build_perspective(fov,1.0f*screen_width/screen_height,1.0f,1e9f,perspective_matrix);
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(perspective_matrix);

	matrix_build_lookat(
		 2.0f,0.0f,2.0f,
		 0.0f,0.0f,0.0f,
		 0.0f,0.0f,1.0f,
		lookat_matrix);
	matrix_toOGL(ogl_matrix,lookat_matrix);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(ogl_matrix);

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f,0.0f,0.0f,0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void glfw_draw_cube(float width, float length, float height) {
	glBegin(GL_QUADS);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f( width*0.5f, length*0.5f ,-height*0.5f );
		glVertex3f(-width*0.5f, length*0.5f ,-height*0.5f );
		glVertex3f(-width*0.5f, length*0.5f , height*0.5f );
		glVertex3f( width*0.5f, length*0.5f , height*0.5f );
		glColor3f(1.0f,0.5f,0.0f);				  
		glVertex3f( width*0.5f,-length*0.5f , height*0.5f );
		glVertex3f(-width*0.5f,-length*0.5f , height*0.5f );
		glVertex3f(-width*0.5f,-length*0.5f ,-height*0.5f );
		glVertex3f( width*0.5f,-length*0.5f ,-height*0.5f );
		glColor3f(1.0f,0.0f,0.0f);				  
		glVertex3f( width*0.5f, length*0.5f , height*0.5f );
		glVertex3f(-width*0.5f, length*0.5f , height*0.5f );
		glVertex3f(-width*0.5f,-length*0.5f , height*0.5f );
		glVertex3f( width*0.5f,-length*0.5f , height*0.5f );
		glColor3f(1.0f,1.0f,0.0f);				  
		glVertex3f( width*0.5f,-length*0.5f ,-height*0.5f );
		glVertex3f(-width*0.5f,-length*0.5f ,-height*0.5f );
		glVertex3f(-width*0.5f, length*0.5f ,-height*0.5f );
		glVertex3f( width*0.5f, length*0.5f ,-height*0.5f );
		glColor3f(0.0f,0.0f,1.0f);				  
		glVertex3f(-width*0.5f, length*0.5f , height*0.5f );
		glVertex3f(-width*0.5f, length*0.5f ,-height*0.5f );
		glVertex3f(-width*0.5f,-length*0.5f ,-height*0.5f );
		glVertex3f(-width*0.5f,-length*0.5f , height*0.5f );
		glColor3f(1.0f,0.0f,1.0f);				  
		glVertex3f( width*0.5f, length*0.5f ,-height*0.5f );
		glVertex3f( width*0.5f, length*0.5f , height*0.5f );
		glVertex3f( width*0.5f,-length*0.5f , height*0.5f );
		glVertex3f( width*0.5f,-length*0.5f ,-height*0.5f );
	glEnd();
}




void main() {
	//First rotor
	float width  = 1.0f; //m, X
	float length = 1.0f; //m, Y
	float height = 0.1f; //m, Z
	float yaw_rpm = 40.0f; //revolutions per minute

	//Test mass (cube)
	float side = 0.2f; //m
	float dx = 0.5f; //X offset from center
	float dy = 0.0f; //Y offset from center
	float dz = 0.2f; //Z offset from center

	EVDS_SYSTEM* system;
	EVDS_OBJECT* inertial_system;
	EVDS_OBJECT* object;
	EVDS_OBJECT* object2;

	printf("Tutorial 3: Rigid Body Dynamics\n");
	EVDS_System_Create(&system);
	EVDS_Common_Register(system);

	//Create inertial sysetm
	EVDS_Object_Create(system,0,&inertial_system);
	EVDS_Object_SetType(inertial_system,"propagator_rk4");
	EVDS_Object_Initialize(inertial_system,1);

	//Create rigid body (rotor in vaccum)
	EVDS_Object_Create(system,inertial_system,&object);
	EVDS_Object_SetName(object,"Rotor");
	EVDS_Object_SetType(object,"vessel");
	EVDS_Object_AddFloatVariable(object,"jxx",(1.0/12.0)*(height*height + length*length),0);
	EVDS_Object_AddFloatVariable(object,"jyy",(1.0/12.0)*(height*height + width*width),0);
	EVDS_Object_AddFloatVariable(object,"jzz",(1.0/12.0)*(width*width   + length*length),0);
	EVDS_Object_AddFloatVariable(object,"mass",1000*width*height*length,0);
	EVDS_Object_SetOrientation(object,inertial_system,0,0,0);
	EVDS_Object_SetAngularVelocity(object,inertial_system,0,0,(yaw_rpm/60.0)*2.0*EVDS_PI);

	EVDS_Object_Create(system,object,&object2);
	EVDS_Object_SetName(object2,"Second rotor");
	EVDS_Object_SetType(object2,"vessel");
	//EVDS_Object_AddFloatVariable(object2,"jxx",(1.0/12.0)*(height*height + length*length),0);
	//EVDS_Object_AddFloatVariable(object2,"jyy",(1.0/12.0)*(height*height + width*width),0);
	//EVDS_Object_AddFloatVariable(object2,"jzz",(1.0/12.0)*(width*width   + length*length),0);
	//EVDS_Object_AddFloatVariable(object2,"mass",1000*width*height*length,0);
	//EVDS_Object_SetOrientation(object2,object,EVDS_RAD(90.0),0,0);

	EVDS_Object_AddFloatVariable(object2,"jxx",(1.0/6.0)*(side*side),0);
	EVDS_Object_AddFloatVariable(object2,"jyy",(1.0/6.0)*(side*side),0);
	EVDS_Object_AddFloatVariable(object2,"jzz",(1.0/6.0)*(side*side),0);
	EVDS_Object_AddFloatVariable(object2,"mass",1000*side*side*side,0);
	EVDS_Object_SetPosition(object2,object,dx,dy,dz);

	//EVDS_Object_Initialize(object2,1);
	EVDS_Object_Initialize(object,1);

	//Initialize OpenGL
	glfw_init();

	//Main simulation loop
	while (1) {
		int iteration;
		EVDS_STATE_VECTOR state;
		EVDS_MATRIX Qmatrix;
		matrix Qmatrix_gl;
		matrix Qmatrix_gl2;

		//Propagate state
		for (iteration = 0; iteration < 10; iteration++) {
			EVDS_Object_Solve(inertial_system,0.002);
		}

		//Generate OpenGL rotation matrix
		EVDS_Object_GetStateVector(object,&state);
		EVDS_Quaternion_ToMatrix(&state.orientation,Qmatrix);
		matrix_convert(Qmatrix_gl,Qmatrix);
		matrix_toOGL(Qmatrix_gl2,Qmatrix_gl);
		EVDS_Vector_Normalize(&state.angular_velocity,&state.angular_velocity);
		printf("%.3f %.3f %.3f\r",
				state.angular_velocity.x,
				state.angular_velocity.y,
				state.angular_velocity.z);

		//Draw object
		glfw_setup_3d();
		glBegin(GL_LINES);
			glColor4f(1,1,1,1);
			glVertex3f(0,0,0);
			glVertex3f(
				state.angular_velocity.x,
				state.angular_velocity.y,
				state.angular_velocity.z);
		glEnd();
		glMultMatrixf(Qmatrix_gl2);
		glfw_draw_cube(width,length,height);
		glBegin(GL_LINES);
			glColor4f(1.0,1.0,0.0,1);
			glVertex3f(0,0,0);
			glVertex3f(0,0,1);
		glEnd();

		glTranslatef(dx,dy,dz);
		glfw_draw_cube(side,side,side);
		glfwSwapBuffers();
		
		//Add a small delay
		SIMC_Thread_Sleep(0.02);
	}

	EVDS_System_Destroy(system);
}
