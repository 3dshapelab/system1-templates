// This file is used to used to generate trials from a FACTORIAL design (BalanceFactor)

// It goes like this:
// starts with a constant stimulus that does not go away (constantPres). can use keypress to change its depth
// pressing '+': 'constantPres' -> 'training', which is 2IFC
// pressing 't': 'training' -> 'experiment' which is the real experiment and we start recording responses on txt file


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
#include <algorithm>
#include <queue>

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
#include "ParametersLoader.h"
#include "BalanceFactor.h"
#include "GLText.h"
#include "Util.h"
#define BROWN 
#ifdef BROWN
#include "BrownMotorFunctions.h"
#else
#include "RoveretoMotorFunctions.h"
#endif

/***** CALIBRATION FILE *****/
#include "LatestCalibration.h"

/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;

const float DEG2RAD = M_PI / 180;
Screen screen;
/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode = true;
static const bool stereo = true;
bool resetScreen_betweenRuns = false;
/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
//Optotrak2 *optotrak;
CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/

/********** STREAMS **************/
ofstream responseFile;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60;

/********* DISPLAY VARIABLES *********/
double display_distance = -400;

/********** TRIAL SPECIFIC PARAMETERS ***************/
ParametersLoader parameters;
BalanceFactor<double> trial; //if using staircase: TrialGenerator<double> trial;

bool trial_prepared = false; 
bool training = true;
int totalBlkNum = 2;
int blkNum = 1;
int trialNum = 0;
bool screenBack = false; //has the screen been placed at projection distance
bool visibleInfo = true;


// Time variables 
Timer trial_time;
enum Stages {constantPresent, preparation, firstFixation, firstPresent, interStimBreak, secondPresent, respond, breakTime, expFinished };
Stages currentStage = constantPresent; // if just want to look at the stimuli, use the constant present stage
double ElapsedTime = 0;
//double interTrialTime = 600, presentationTime = 600, firstFixationTime = 600;
double interTrialTime = 400, presentationTime = 500, firstFixationTime = 200;
/********* CURRENT STIMULI *********/
double cube_size = 40;
bool stdFirst = true;
bool resp_stdLarger = true;
double size_constPres = 40.0, size_std = 20, size_comp = 20, size_first = 20, size_second = 20;


/*************************** EXPERIMENT SPECS ****************************/
string subjectName;
string experiment_directory = "C:/Users/visionlab/Documents/data/Templates/template3-generate-trials/";
// paramters file directory and name
string parametersFile_directory = experiment_directory + "/parameters_template3_balanceFactor.txt";
// response file headers
string responseFile_headers = "subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tstdSize\tcompSize\tresp_stdDeeper\tRT";


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
void initBlock();
void initTrial();
void online_trial();
void advanceTrial();
void drawStimulus();
void drawCube(double CubeSize);
void drawFixation();
void drawInfo();
void beepOk(int tone);
void shutdown();



/*

#pragma once
// The following macros define the minimum required platform.  The minimum required platform
// is the earliest version of Windows, Internet Explorer etc. that has the necessary features to run 
// your application.  The macros work by enabling all features available on platform versions up to and 
// including the version specified.

// Modify the following defines if you have to target a platform prior to the ones specified below.
// Refer to MSDN for the latest info on corresponding values for different platforms.
#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif


*/



//static const bool gameMode=true;

/*************************** FUNCTIONS ***********************************/

// This function seems to be used to shut down the system after use
void shutdown(){

	responseFile.close(); // close this object
		if(resetScreen_betweenRuns){
		homeEverything(5000,4500);
	}
	exit(0);
}

void initProjectionScreen(double _focalDist, const Affine3d &_transformation, bool synchronous)
{
	focalDistance = _focalDist;	
    screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE*SCREEN_HEIGHT/SCREEN_WIDTH);
    screen.setOffset(alignmentX,alignmentY);
    screen.setFocalDistance(_focalDist);
    screen.transform(_transformation);
    cam.init(screen);
	if ( synchronous )
		moveScreenAbsolute(_focalDist,homeFocalDistance,4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist,homeFocalDistance,4500);
}

// Initialize motors for moving screen around
void initMotors()
{
	if(resetScreen_betweenRuns){
		homeEverything(5000,4500);
	}
}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
void initRendering()
{   
glClearColor(0.0,0.0,0.0,1.0);
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
/* Set size buffer clear value */
glClearDepth(1.0);
/* Enable depth test */
glEnable(GL_DEPTH_TEST);
/* Set size function */
glDepthFunc(GL_LEQUAL);
// scommenta solo se vuoi attivare lo shading degli stimoli

glMatrixMode(GL_MODELVIEW);
glLoadIdentity();
// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
// ballerine (le vedi vibrare)
glLineWidth(1.5);

}

// Initialize the streams, open the file and write to it
void initStreams()
{
	// Initialize the parameter file starting from the file parameters.txt, if the file does not exist, it tells you
	ifstream parametersFile;
	parametersFile.open(parametersFile_directory.c_str());
	parameters.loadParameterFile(parametersFile);

	// Subject name
	subjectName = parameters.find("SubjectName");
 
	string responseFileName = experiment_directory +"/"+ subjectName  + "_factorial.txt";
	// Principal streams files
	if (util::fileExists(experiment_directory +"/"+subjectName+"_factorial.txt") && subjectName != "junk")
	{
		string error_on_file_io = experiment_directory+"/"+subjectName+"_factorial.txt" + string(" already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.",NULL, NULL);
		shutdown();
	}

	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;
		
}

void initBlock()
{
	trialNum = 1;
	trial.init(parameters);
	trial.next();
}

//Extracts constants from parameters file. Used in the method defined below
void initVariables()
{  
	// initialize the trial matrix
	initBlock();
	cout << "initialize parameter file" << endl;
	
	//initProjectionScreen(display_distance);	
	interoculardistance = atof(parameters.find("IOD").c_str());
	// eye coordinates
	eyeRight = Vector3d(interoculardistance/2,0,0);
	eyeLeft = Vector3d(-interoculardistance/2,0,0);
	eyeMiddle = Vector3d(0,0,0);
	display_distance = atof(parameters.find("dispDistance").c_str());//str2num<double>(parameters.find("randomDotssize"));
	initProjectionScreen(display_distance);
	
}

//Central function for projecting image onto the screen
void drawGLScene()
{
	// Note that in this experiment, there is no difference between drawing left and right because
	// we are displaying to the middle (there is no stereo)
	// Draw left eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0,0.0,0.0,1.0);
	cam.setEye(eyeLeft);//cam.setEye(eyeMiddle); 
	drawStimulus();
	drawInfo();


	// Draw right eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0,0.0,0.0,1.0);
	cam.setEye(eyeRight); //cam.setEye(eyeMiddle);
	drawStimulus();
	drawInfo();
	
	glutSwapBuffers();
}
void update(int value)
{
    glutPostRedisplay();
    glutTimerFunc(TIMER_MS, update, 0);
}


void drawInfo()
{
	// displays relevant information to the screen
	if ( visibleInfo )
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);

		GLText text;

		if ( gameMode )
			text.init(SCREEN_WIDTH,SCREEN_HEIGHT,glWhite,GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640,480,glWhite,GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();

		if (currentStage == expFinished) {

			glColor3fv(glWhite);
			text.draw("The experiment is over. Thank you! :)");

		}
		else if (currentStage == breakTime) {
			glColor3fv(glRed);
			text.draw("Break time! Press + to continue");
		}
		else if(currentStage == constantPresent) {
			glColor3fv(glWhite);
			text.draw("# Name: " + subjectName);		
			text.draw("# IOD: " +stringify<double>(interoculardistance));
			text.draw(" "); 
			text.draw("press + to start");
		}
		else {	
			glColor3fv(glRed);				

			text.draw(" ");text.draw(" "); text.draw(" ");

			text.draw("# trial: " +stringify<int>(trialNum));
			text.draw("# std size: " + stringify<double>(size_std));
			text.draw("# comp size: " + stringify<double>(size_comp));
			text.draw("# current stage: " +stringify<int>(currentStage));
			text.draw("# time: " +stringify<double>(ElapsedTime));			
		}
			text.leaveTextInputMode();
			glEnable(GL_COLOR_MATERIAL);
			glEnable(GL_BLEND);
	}
}

/***** SOUNDS *****/
void beepOk(int tone)
{

	switch(tone)
	{
	case 0:
		// Remember to put double slash \\ to specify directories!!!
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 1:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-success.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	}
	return;
}
void idle()
{
	online_trial();
	ElapsedTime = trial_time.getElapsedTimeInMilliSec();
	
}




void drawFixation() {
	// draws a small fixation cross at the center of the display
	glDisable(GL_TEXTURE_2D);
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(2.f);
	glLoadIdentity();
	glTranslated(0, 0, display_distance);
	double cross_length = 5;
	double circle_radius = 1;
	glBegin(GL_LINES);
	glVertex3d(cross_length / 2. + circle_radius / 2., 0, 0);
	glVertex3d(-cross_length / 2. + circle_radius / 2., 0, 0);
	glVertex3d(circle_radius / 2, -cross_length / 2. , 0);
	glVertex3d(circle_radius / 2, cross_length / 2. , 0);
	glEnd();


}

void drawCube(double CubeSize)
{
	glColor3f(1.0f, 0.0f, 0.0f);
	glLoadIdentity();
	glPushMatrix();
	glTranslated(0, 0, display_distance);
	glutWireCube(CubeSize);
	glPopMatrix();



}

void drawStimulus()
{

	double cross_length = 5;
	double circle_radius = 1;
	switch (currentStage) {

	case constantPresent:

	drawCube(size_constPres);

	glLoadIdentity();
	glPushMatrix();

	glColor3f(1.0f, 1.0f, 1.0f);
	glLineWidth(2.f);

	glBegin(GL_LINES);
	glVertex3d(cross_length / 2. + circle_radius / 2., 0, 0);
	glVertex3d(-cross_length / 2. + circle_radius / 2., 0, 0);
	glVertex3d(circle_radius / 2, -cross_length / 2. , 0);
	glVertex3d(circle_radius / 2, cross_length / 2. , 0);
	glEnd();

	glPopMatrix();

	
		break;


	case firstFixation:
		drawFixation();
		break;

	case firstPresent:
		
		drawCube(size_first);

		break;

	case interStimBreak:
		drawFixation();
		break;

	case secondPresent:
		drawCube(size_second);
		break;

	}

}

void initTrial()
{
	trial_prepared = false;
	initProjectionScreen(display_distance);
	if(!trial_prepared){

		if (currentStage == constantPresent) { 

			trial_prepared = true;
			currentStage = preparation;

		}else{

			currentStage = preparation;

			if (training) {
	
				size_first = rand() % 30 + 10;
				size_second = rand() % 30 + 10;
			}
			else {
				size_std = trial.getCurrent()["stdsize"];
				size_comp = trial.getCurrent()["compsize"];
		
				if (rand() % 2 == 0){
					stdFirst = true;
					size_first = size_std;
					size_second = size_comp;

				}else{
					stdFirst = false;
					size_first = size_comp;
					size_second = size_std;
				}

			}
			trial_prepared = true;
			currentStage = preparation;

		}		
	}
}


void online_trial() {
	
	switch (currentStage) {

	case preparation:
		
		if(trial_prepared){
			// reset TrialTime variables
			ElapsedTime = 0;
			trial_time.reset();
			trial_time.start(); // start the timer
			currentStage = firstFixation;
		}		
		break;

	case firstFixation:
		
		if (ElapsedTime > firstFixationTime) {
			currentStage = firstPresent;
		}
		break;

	case firstPresent:

		if (ElapsedTime > firstFixationTime + presentationTime) {
			currentStage = interStimBreak;

		}
		break;

	case interStimBreak:

		if (ElapsedTime > firstFixationTime + presentationTime + interTrialTime) {
			currentStage = secondPresent;
		}
		break;

	case secondPresent:

		if (ElapsedTime > firstFixationTime + 2 * presentationTime + interTrialTime) {
			currentStage = respond;
		}
		break;
	}

}



void advanceTrial()
{
	// "subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tstdSize\tcompSize\tcompSign\tcompDelta\tresp_firstDeeper\tresp_stdDeeper\tRT";

	if(training){
			//beepOk(0);
			initTrial();
	}else{
		responseFile << fixed <<
		subjectName << "\t" <<		//subjName
		interoculardistance << "\t" <<
		blkNum << "\t" <<
		trialNum << "\t" <<
		display_distance << "\t" <<
		size_std << "\t" <<
		size_comp << "\t" <<
		resp_stdLarger << "\t" <<
		ElapsedTime << endl;

		if (!trial.isEmpty()){			
			trialNum++;
			trial.next();
			trial_prepared = false;
			initTrial();

		}else{
			if(blkNum < totalBlkNum){
				currentStage = breakTime;
				visibleInfo=true;
				blkNum++;
				initBlock();
				}else{
				beepOk(1);
				responseFile.close();
				visibleInfo=true;
				currentStage = expFinished;
				}
		}
		
	}

}





// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,SCREEN_WIDTH, SCREEN_HEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

}
// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{   
	//cout << "listening for keypress" << endl;
	switch (key)
    {   
		case 'i':
			visibleInfo=!visibleInfo;
		break;

		case '7':
			ElapsedTime = 0;
			trial_time.reset();
			trial_time.start();
			currentStage = firstFixation;
		break;

		case 'Q':
		case 'q':
		case 27:	//corrisponde al tasto ESC
		{   
			// Ricorda quando chiami una funzione exit() di chiamare prima cleanup cosi
			// spegni l'Optotrak ed i markers (altrimenti loro restano accesi e li rovini) 
			shutdown();
		}
		break;


		case '+':
		{
			if(currentStage == constantPresent){
				beepOk(0);
				visibleInfo=false;
				currentStage = preparation;
				initTrial();	
			}
			if (currentStage == breakTime) {
				beepOk(0);
				visibleInfo = false;
				currentStage = preparation;
				initTrial();
			}

		}
		break;


		case '1':
			if (currentStage == respond) {
				
				beepOk(0);
				if (stdFirst)
					resp_stdLarger = true;
				else
					resp_stdLarger = false;

				advanceTrial();
			}
			break;

		case '2':
			if (currentStage == respond) {

				beepOk(0);
				//if the second presented stimulus was a test stimulus (first presented stimulus was a reference)
				if (stdFirst)
					resp_stdLarger = false;
				else
					resp_stdLarger = true;

				advanceTrial();
			}
			break;


		case 'a': // change texture
		{
			if(size_constPres < 69)
				size_constPres = size_constPres + 2 ;
		}			
		break;

		case 's':
		{
			if (size_constPres > 5)
				size_constPres = size_constPres - 2 ;
		}
		break;

		case 't': // change texture
		{
			if(training){
				training = false;
				beepOk(1);
				initTrial();
				trialNum = 1;
				visibleInfo = true;
				currentStage = breakTime;
			}
		}
		break;
	}
}


// this is run at compilation because it's titled 'main'
int main(int argc, char*argv[])  
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();
	
	// initializes optotrak and velmex motors
	initMotors();
	
	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();
	

	// initializing experiment's parameters
	
	initRendering(); // initializes the openGL parameters needed for creating the stimuli
	
	initStreams(); // streams as in files for writing data

	initVariables();
	// glut callback, OpenGL functions that are infinite loops to constantly run 

	glutDisplayFunc(drawGLScene); // keep drawing the stimuli

	glutKeyboardFunc(handleKeypress); // check for keypress

	glutReshapeFunc(handleResize);

	glutIdleFunc(idle);

	glutTimerFunc(TIMER_MS, update, 0);

	glutSetCursor(GLUT_CURSOR_NONE);

	boost::thread initVariablesThread(&initVariables); 

	glutMainLoop();

	return 0;
}
