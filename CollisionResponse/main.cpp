// Skeleton program for GLUT applications.

// Link with: opengl32.lib, glu32.lib, glut32.lib.

#include <math.h>
#include <iostream>
#include <strstream>
#include <iomanip>
#include <glut.h>
#include "../FPSCamera/FPSCamera.h"
#include "../FPSCamera/CameraController.h"
#include "Plane.h"
#include "ParticleSystem.h"
#include "PhysicsSystem.h"
#include "GravitationalForce.h"
#include "CentralForce.h"
#include "Box.h"
#include "DefaultDebugDrawer.h"
#include "Tetrahedron.h"
#include "GUIBase.h"
#include "GUITray.h"
#include "VarWatch.h"

using namespace std;

extern void DrawArrow(Vec3& point, Vec3& direction, Vec3& colour);

// Initial size of graphics window.
const int WIDTH  = 800;
const int HEIGHT = 600;

// Current size of window.
int width  = WIDTH;
int height = HEIGHT;

float xMouse = (float)width / 2.0f;
float yMouse = (float)height / 2.0f;

float dMouseX = 0.0f;
float dMouseY = 0.0f;

bool keystate[256];
bool lastKeystate[256];

bool breakEnable = false;

// Bounds of viewing frustum.
double nearPlane =  0.1f;
double farPlane  = 1000.0f;

// Viewing angle.
double fovy = 40.0;

// Variables.
double alpha = 0;                                  // Set by idle function.
double beta = 0;                                   // Set by mouse X.
double dist = - (farPlane - nearPlane) / 2;    // Set by mouse Y.

CameraController* cameraController;
FPSCamera* camera;

int fps = 60;
bool physicsActive = false;

Plane * groundPlane;
ColouredParticleSystem* particleSystem;

Vec3 maxPos(25, 30, 25);
Vec3 minPos(-25, 10, -25);

AABB bounds(minPos, maxPos);

Box* testBox;

std::vector<Box*> boxes;
int numBoxes = 1;
Vec3 testVel;

int impulseCount = 0;
int numContacts;
float relativeVel = 0.0f;
bool breakOnImpulse = false;
bool handleMouse = false;

GUIBase* gui;

// This function is called to display the scene.

void AddBox()
{
	Box* box = new Box(ColouredParticleSystem::RandomVector(60.0) + Vec3(0, 15, 0), ColouredParticleSystem::RandomVector(10.0) + Vec3(5, 5, 5));
	//box->ApplyImpulse(Vec3(0, -0.05, 0));
	box->ApplyAngularMomentum(ColouredParticleSystem::RandomVector(1), ((float)rand() * 0.01) / RAND_MAX);
	box->ConvexPolyhedron::SetDebugColour(Vec4(ColouredParticleSystem::RandomVector(1), 1));
	boxes.push_back(box);
	PhysicsSystem::GetCurrentInstance()->AddRigidBody(box);
}

void AddTetra()
{
	Tetrahedron* tetra = new Tetrahedron(ColouredParticleSystem::RandomVector(30.0) + Vec3(0, 15, 0), (float)rand() * 10.0f / RAND_MAX);
	tetra->ApplyImpulse(ColouredParticleSystem::RandomVector(0.05f));
	tetra->ApplyAngularImpulse(ColouredParticleSystem::RandomVector(0.05f));
	tetra->ConvexPolyhedron::SetDebugColour(Vec4(ColouredParticleSystem::RandomVector(1), 1));
	PhysicsSystem::GetCurrentInstance()->AddRigidBody(tetra);
}

void setup()
{
	srand(time(NULL));
	camera = new FPSCamera();
	cameraController = new CameraController();
	cameraController->SetCamera(camera);
	camera->Position = Vec3(0.0, 0.0, -5.0f);
	cameraController->HasMomentum = true;
	glutWarpPointer(width / 2, height/ 2);
	cameraController->HasAngularMomentum = true;
	cameraController->MaxSpeed = 0.25f;
	for (int i = 0; i < 255; ++i)
	{
		keystate[i] = false;
		lastKeystate[i] = false;
	}
	if (handleMouse)
		glutSetCursor(GLUT_CURSOR_NONE);

	for (int i = 0; i < numBoxes; ++i)
	{
		AddBox();
	}

	Box* groundBox = new Box(Vec3(0, 0, 0), Vec3(100, 1, 100));
	groundBox->SetKinematic(true);
	PhysicsSystem::GetCurrentInstance()->AddRigidBody(groundBox);

	testBox = boxes[0];
	testVel = testBox->GetVelocity();
	testBox->RigidBody::SetDebugColour(Vec4(1, 0, 0, 1));

	PhysicsSystem::GetCurrentInstance()->SetDebugDrawer(new DefaultDebugDrawer());
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	float spec[4];
	spec[0] = 1.0f;
	spec[1] = 1.0f;
	spec[2] = 1.0f;
	spec[3] = 1.0f;
	Vec4 amb(0.01f, 0.01f, 0.01f, 0.01f);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb.Ref());
	glEnable(GL_LIGHT0);
	Vec4 diffColour(1.0f, 0.3f, 0.3f, 1.0f);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffColour.Ref());	

	gui = new GUIBase();
	GUITray* tray = gui->CreateTray(0, 0, 0);
	tray->AddTextBox("blah");
	tray->AddElement(new VarWatch<float>("RelativeVel", &relativeVel, new VarPrint<float>()));
	tray->AddElement(new VarWatch<Vec3>("Cam Pos", &camera->Position, new VarPrint<Vec3>()));
}

int lastTime = 0;
int frameCounter = 0;
int frameTimeCount = 0;

void display ()
{
	int currentTime = glutGet(GLUT_ELAPSED_TIME);
	int elapsedTime = (currentTime - lastTime);

	lastTime = currentTime;

	glEnable(GL_DEPTH_TEST);	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(camera->GetProjectionMatrix().Ref());

	++frameCounter;
	frameTimeCount += elapsedTime;

	if (frameTimeCount > 1000)
	{
		frameTimeCount = 0;
		fps = frameCounter;
		frameCounter = 0;
	}

	cameraController->Update((float)elapsedTime);
	if (physicsActive)
		PhysicsSystem::GetCurrentInstance()->Integrate((float)elapsedTime / 1000.0f);
	Vec4 lightPos(0.0f, 100.0f, 10.0f, 1.0f);

 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glMultMatrixf(camera->GetViewTransform().Ref()); //apply camera transform
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos.Ref());
	
	glPushMatrix();

	glDisable(GL_LIGHTING);
	PhysicsSystem::GetCurrentInstance()->DrawDebug();
	PhysicsSystem::GetCurrentInstance()->GetDebugDrawer()->DrawAABB(bounds, Vec4(0, 0, 1, 1));

	glPopMatrix();

	Vec3 origin(0, 2, 0);

	DrawArrow(origin, Vec3(5, 0, 0), Vec3(1, 0, 0));
	DrawArrow(origin, Vec3(0, 5, 0), Vec3(0, 1, 0));
	DrawArrow(origin, Vec3(0, 0, 5), Vec3(0, 0, 1));

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, (GLdouble)width, (GLdouble)height, 0.0);

	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gui->Draw();
	glutSwapBuffers();

	while (glutGet(GLUT_ELAPSED_TIME) - currentTime < 16) {}	

}

void HandleInput()
{
	if (keystate['w'])
	{
		cameraController->MoveForward();
	}
	else if (keystate['s'])
	{
		cameraController->MoveBackward();
	}

	if (keystate['a'])
	{
		cameraController->MoveLeft();
	}
	else if (keystate['d'])
	{
		cameraController->MoveRight();
	}

	if (keystate[' '])
	{
		cameraController->MoveUp();
	}
	else if (keystate['c'])
	{
		cameraController->MoveDown();
	}

	if (keystate['m'] && !lastKeystate['m'])
	{
		if (handleMouse)
		{
			handleMouse = false;
			glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
		}
		else
		{
			handleMouse = true;
			glutWarpPointer(width / 2, height / 2);
			glutSetCursor(GLUT_CURSOR_NONE);
		}
	}

	if (keystate['o'] && !lastKeystate['o'])
		breakEnable = true;

	if (keystate['b'] && !lastKeystate['b'])
	{
		AddBox();		
	}
	if (keystate['q'] && !lastKeystate['q'])
		AddTetra();
	if (keystate['p'] && !lastKeystate['p'])
		physicsActive = !physicsActive;

	if (keystate['l'])
	{
		testBox->ApplyForceAtPoint(Vec3(0.001, 0, 0), testBox->GetPosition() + Vec3(0, 0, 1));
	}
	if (keystate['k'])
		testBox->ApplyForceAtPoint(Vec3(0.0, 0.0, 0.001), testBox->GetPosition() + Vec3(0, 1, 0));
	
	if (keystate[27])
		exit(0);
	if (keystate['h'])
	{
		int i = 0;
		testBox = testBox;
	}
	if (keystate['t'] && !lastKeystate['t'])
		PhysicsSystem::GetCurrentInstance()->Integrate(0.016f);

	memcpy(lastKeystate, keystate, sizeof(bool) * 256);

}

// This function is called when there is nothing else to do.
void idle ()
{

	HandleInput();

	const double STEP = 0.1;
	const double ALL_ROUND = 360;
	alpha += STEP;
	if (alpha > ALL_ROUND)
		alpha -= ALL_ROUND;

	// Display normalized coordinates in title bar.

	Vec3 axis = qAxisAngle(testBox->GetAngularVelocity());
	float magnitude = len(axis);
	if (magnitude > 0.0000001f)
		axis /= magnitude;

	const int BUFSIZE = 200;
	static char buffer[BUFSIZE];
	ostrstream s(buffer, BUFSIZE);
	s << 
		resetiosflags(ios::floatfield) << 
		setprecision(3) << "Camera pitch, yaw: " << camera->Pitch << ", " << camera->Yaw << 
		setprecision(3) << ").  BoxAngularVel=" << setw(3) << magnitude << ", " << axis[0] << ", " << axis[1] << ", " << axis[2] <<
		setprecision(3) << ".  fps=" << fps <<
		"." << 
		" BoxVel=" << testBox->GetVelocity()[0] << ", " << testBox->GetVelocity()[1] << ", " << testBox->GetVelocity()[2] << ". ImpulseCount=" 
		<< impulseCount << " NumContacts=" << numContacts << " RelativeVel=" << relativeVel << ends;
	glutSetWindowTitle(buffer);

	dMouseX = (xMouse - width / 2.0f);
	dMouseY = (yMouse - height / 2.0f);


	if (handleMouse)
	{
		cameraController->ChangePitch(-dMouseY);
		cameraController->ChangeYaw(-dMouseX);
		glutWarpPointer(width / 2, height / 2);
	}

	glutPostRedisplay();
}

void mouseMovement (int mx, int my)
{
   // Normalize mouse coordinates.
   xMouse = float(mx);
   yMouse = float(my);   
}

// Respond to window resizing, preserving proportions.
// Parameters give new window size in pixels.
void reshapeMainWindow (int newWidth, int newHeight)
{
	width = newWidth;
	height = newHeight;
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(camera->GetProjectionMatrix().Ref());
}

// Display help.
void help()
{
   cout << 
      "'h'   display help" << endl <<
      endl;
}

// Respond to graphic character keys.
// Parameters give key code and mouse coordinates.
void KeyDown (unsigned char key, int x, int y)
{
	keystate[key] = true;
}

void KeyUp (unsigned char key, int x, int y)
{
	keystate[key] = false;
}

// Respond to function keys.
// Parameters give key code and mouse coordinates.
void functionKeys (int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_F1:
		cout << "F1" << endl;
		break;
	case GLUT_KEY_UP:
		cout << "Up" << endl;
		break;	  
	}
}


void main (int argc, char **argv)
{
	// GLUT initialization.
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(width, height);
	glutCreateWindow("GLUT Skeleton Program");
	setup();
	// Register call backs.
	glutDisplayFunc(display);
	glutReshapeFunc(reshapeMainWindow);
	glutKeyboardFunc(KeyDown);
	glutKeyboardUpFunc(KeyUp);
	glutSpecialFunc(functionKeys);
	glutMotionFunc(mouseMovement);
	glutPassiveMotionFunc(mouseMovement);
	glutIdleFunc(idle);

	// OpenGL initialization
	glEnable(GL_DEPTH_TEST);

	// Display help.
	help();

	// Enter GLUT loop.
	glutMainLoop();
}
