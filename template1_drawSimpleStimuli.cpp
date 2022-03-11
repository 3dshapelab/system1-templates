// This file is used to test visual stimuli. It builds and draws stimuli
// it does not go from trial to trial. But you can use keypress to change some variable
// to change the display distance, change it from the variable declaration


#include <cstdlib>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
//#include "Optotrak.h"
//#include "Optotrak2.h"
#include "Marker.h"
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "Util.h"
#define BROWN 
#ifdef BROWN
#include "BrownMotorFunctions.h"
#else
#include "RoveretoMotorFunctions.h"
#endif
/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;

/********* VISUALIZATION VARIABLES *****************/
Screen screen;
#include <Eigen/Core>
#define TIMER_MS 11
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
extern const double SCREEN_WIDE_SIZE = 383; //306;    // millimeters
extern const double SCREEN_HIGH_SIZE = SCREEN_WIDE_SIZE*SCREEN_HEIGHT/SCREEN_WIDTH; //306;    // millimeters
// Alignment between optotrak z axis and screen z axis
double alignmentX =  8.7;//24; //33.5; 
double alignmentY =  37.;//34; // 33;
double homeFocalDistance=-270.0, focalDistance= homeFocalDistance, baseFocalDistance= homeFocalDistance;

static const bool gameMode = true;
static const bool stereo = true;

bool resetScreen_betweenRuns = false;
/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
//Optotrak2 *optotrak;
//CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/
bool isStimulusDrawn = true;


/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
//vector <Marker> markers;
double interoculardistance = 60;


/********* DISPLAY VARIABLES *********/
double display_distance = -400;
double theta = 0;

/********* CURRENT STIMULI *********/
std::vector<Vector3d> dotContainer_cube;
int dot_num = 100;
double cube_size = 40;

/********** FUNCTION PROTOTYPES *****/
void beepOk(int tone);
void drawGLScene();
void handleKeypress(unsigned char key, int x, int y);
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void update(int value);
void idle();
void initMotors();
void initGLVariables();
void initVariables();
void initStreams();
void initRendering();
void initTrial();
void drawStimulus();

std::vector<Vector3d> buildRandomDotsCube(int dotNum, double cubeSize);
void drawRandomDots(std::vector<Vector3d> dot_container);
void drawSimpleOjbect();


/*************************** FUNCTIONS ***********************************/
/***** SOUND THINGS *****/
void beepOk(int tone)
{
	switch (tone)
	{
	case 0:
		// Remember to put double slash \\ to specify directories!!!
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-1.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 1:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-6.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	}
	return;
}


void initGLVariables()
{

}



void drawStimulus()
{
	drawRandomDots(dotContainer_cube);
	//drawSimpleOjbect();

}

std::vector<Vector3d> buildRandomDotsCube(int dotNum, double cubeSize)
{
	std::vector<Vector3d> dot_container;

	for (int dots_placed = 0; dots_placed < dotNum; dots_placed++)
	{
		double x_axis = double(rand() % 1001) / 1000.0 * cubeSize - cubeSize / 2.0;
		double y_axis = double(rand() % 1001) / 1000.0 * cubeSize - cubeSize / 2.0;
		double z_axis = double(rand() % 1001) / 1000.0 * cubeSize - cubeSize / 2.0;

		dot_container.push_back(Vector3d(x_axis, y_axis, z_axis));
	}

	return dot_container;

}

void drawRandomDots(std::vector<Vector3d> dot_container)
{
	glLoadIdentity();
	glTranslated(0, 0, display_distance);
	glRotatef(theta, 1.0, 1.0, 1.0); // 0, 0, 1

	for (int i = 0; i < int(dot_container.size()); i++)
	{
		Vector3d dot_vector = dot_container.at(i);
		double x_axis = dot_vector[0];
		double y_axis = dot_vector[1];
		double z_axis = dot_vector[2];

		glColor3f(1.0f, 0.0f, 0.0f);
		glPushMatrix();
		glTranslated(x_axis, y_axis, z_axis);
		glutSolidSphere(0.3, 8, 5);

		glPopMatrix();
	}

}

void drawSimpleOjbect()
{
	glColor3f(1.0f, 0.0f, 0.0f);

	glLoadIdentity();
	glPushMatrix();
	glTranslated(0, 0, display_distance);
	glRotatef(theta, 0.0, 1.0, 1.0); // 0, 0, 1

	glutSolidSphere(0.5, 20, 20);
	glutWireCube(30.0);

	glPopMatrix();

}

void drawGLScene()
{
	if (stereo)
	{
		// Draw left eye view
		glDrawBuffer(GL_BACK_LEFT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		cam.setEye(eyeRight);// we need to flip the eye because the image is mirrored
		drawStimulus();


		// Draw right eye view
		glDrawBuffer(GL_BACK_RIGHT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		cam.setEye(eyeLeft);// we need to flip the eye because the image is mirrored
		drawStimulus();

		glutSwapBuffers();
	}
	else
	{
		glDrawBuffer(GL_BACK);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		cam.setEye(eyeMiddle);
		drawStimulus();

		glutSwapBuffers();
	}
}

void initTrial()
{
	isStimulusDrawn = false;
	theta = 30;
	dotContainer_cube = buildRandomDotsCube(dot_num, cube_size);
	isStimulusDrawn = true;
	initProjectionScreen(display_distance);
}

// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{
	switch (key)
	{   //Quit program
	case 'x':

		//trial.next();
		isStimulusDrawn = false;
		drawGLScene();
		initProjectionScreen(display_distance);
		isStimulusDrawn = true;
		break;

	case 27:	//corrisponde al tasto ESC
	{
		if (resetScreen_betweenRuns)
			homeEverything(5000, 4500);

		exit(0);
	}
	break;

	// Enter key: press to make the final calibration



	case 'a':
	{
		//theta -= M_PI/2.0;
		theta -= 5;
	}
	break;
	case 's':
	{
		//theta += M_PI/2.0;
		theta += 5;
	}
	break;

	case 'm':
	{
		dot_num += 100;
		dotContainer_cube = buildRandomDotsCube(dot_num, cube_size);

	}
	break;

	case 'n':
	{
		if (dot_num > 100)
			dot_num -= 100;

		dotContainer_cube = buildRandomDotsCube(dot_num, cube_size);

	}
	break;


	case 'r':
	{
		display_distance = display_distance + 10;
	}
	break;




	}
}

// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}


void initProjectionScreen(double _focalDist, const Affine3d& _transformation, bool synchronous)
{
	focalDistance = _focalDist;
	screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE * SCREEN_HEIGHT / SCREEN_WIDTH);
	screen.setOffset(alignmentX, alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if (synchronous)
		moveScreenAbsolute(_focalDist, homeFocalDistance, 3500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 3500);
}


// Questa funzione si occupa di fare il refresh della schermata ed e' chiamata ogni TIMER_MS millisecond, tienila cosi'
void update(int value)
{
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}

void idle()
{





}


void initRendering()
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Set depth buffer clear value */
	glClearDepth(1.0);
	/* Enable depth test */
	glEnable(GL_DEPTH_TEST);
	/* Set depth function */
	glDepthFunc(GL_LEQUAL);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLineWidth(1.5);
}

void initVariables()
{
	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
	eyeMiddle = Vector3d(0, 0, 0);
	initTrial();
}


// Inizializza gli stream, apre il file per poi scriverci
void initStreams()
{

}
// Porta tutti i motori nella posizione di home e azzera i contatori degli steps
void initMotors()
{
	if (resetScreen_betweenRuns)
		homeEverything(5000, 4500);
}

int main(int argc, char* argv[])
{
	mathcommon::randomizeStart();
	glutInit(&argc, argv);
	if (stereo)
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
	else
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	if (gameMode == false)
	{
		glutInitWindowSize(640, 480);
		glutCreateWindow("EXP WEXLER");
		//glutFullScreen();
	}
	else
	{
		glutGameModeString("1024x768:32@85");
		glutEnterGameMode();
		glutFullScreen();
	}
	initMotors();
	initRendering();
	initGLVariables();
	initStreams();
	initVariables();
	glutDisplayFunc(drawGLScene);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutIdleFunc(idle);
	glutTimerFunc(TIMER_MS, update, 0);
	glutSetCursor(GLUT_CURSOR_NONE);
	boost::thread initVariablesThread(&initVariables);
	glutMainLoop();

	return 0;
}
