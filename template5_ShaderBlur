// This template is for producing blurred images with shaders, with adjustable blurring parameters

// Prep:
 
// new header file (has to come BEFORE the gl.h stuffs
// #include CShaderProgram.h 

// new global variables:
//int Width = SCREEN_WIDTH; int Height = SCREEN_HEIGHT;
//GLuint sceneBuffer;
//GLuint FBO;
//CShaderProgram BlurH, BlurV;
//float* data;

// new functions
// void initBlur();
// void blurPass(CShaderProgram blur_shader, unsigned int texID); 
// void drawBlur();


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

#include <cstdlib>
#include <cmath>
#include <math.h>
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
#include <string>


/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include "CShaderProgram.h" // shader program

#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "CylinderPointsStimulus.h"
#include "EllipsoidPointsStimulus.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "TrialGenerator.h"
#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"
#include "BalanceFactor.h"
#include "ParStaircase.h"
#include "Staircase.h"
#include "BrownPhidgets.h"
#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"

//Library for texture mapping
#include "SOIL.h"
/***** CALIBRATION FILE *****/
#include "LatestCalibration.h"

/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;


#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"
#include "BrownMotorFunctions.h"
using namespace BrownMotorFunctions;
using namespace BrownPhidgets;



/*************** Variable Declarations ********************/
static const bool gameMode = true;
const float DEG2RAD = M_PI / 180;
/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
Optotrak2* optotrak;
Screen screen;
CoordinatesExtractor headEyeCoords;
/********** STREAMS **************/
ofstream responseFile;
// experiment directory
string experiment_directory = "C:/Users/visionlab/Documents/data/Templates/template5-ShaderBlur/";
// response file headers
string responseFile_headers = "IOD\tdisplayDistance\tblurBoxWidth\tblurExtraPasses\tblurDotSize";


/********* November 2019   CALIBRATION ON CHIN REST *****/

static const Vector3d center(0, 0, focalDistance);
double mirrorAlignment = 0.0, screenAlignmentY = 0.0, screenAlignmentZ = 0.0;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;

/********** TRIAL SPECIFIC PARAMETERS ***************/
//controling bool
bool visibleInfo = true;
bool stimlus_built = false;
bool blur_left_eye = true;
bool blur_right_eye = false;

bool resetScreen_betweenRuns = false; // do we want to reset the screen between blocks

/********** STIMULUS SPECIFIC PARAMETERS ***************/
double display_distance = -400;

double visual_angle = 7; // stim diangonal size
double height_stimulus, width_stimulus; // height and width
double depth_stimulus = 40; // depth
int cylHorizontal = 1;

// dot specs
float brightness_clear_dot = 0.6;
int dot_per_col = 16, dot_per_row = 16; //16, 19
int dot_number = (dot_per_col * dot_per_row) / (1.5 * 1.5);//1.5 is the ration of temp_height / height_stimulus
double dot_visangle = .1333;
double jitter_x_max = 2.5, jitter_y_max = 2.5;
std::vector<Vector3d> dotContainer_stimulus;

/**********	BLUR PARAMETERS ***************/
int blurringBox_Width = 3;
int blur_extraPasses = 3;
double dot_size_bfBlur = 1.14; // when extrapass is 4 -> 1.24

/*************************** Blur Shader ********************************************/
// important, don't forget that you include the CShaderProgram.h at the very top of the experiment file
int Width = SCREEN_WIDTH; int Height = SCREEN_HEIGHT;
GLuint sceneBuffer;
GLuint FBO;
CShaderProgram BlurH, BlurV;
float* data;


/*************************** FUNCTIONS ***********************************/
void initOptotrak();
void initMotors();
void initRendering();
void initVariables();
void initStreams();
void handleResize(int w, int h);
void drawStimulus_clear_eye();
void drawStimulus_blur_eye();
void initTrial();
void updateTheMarkers();
void drawInfo();
void beepOk(int tone);
void cleanup();
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void online_apparatus_alignment();
std::vector<Vector3d> buildRandomDotsA(double dispDepth);
std::vector<Vector3d> buildRandomDotsB(double dispDepth);
void drawRandomDots(float dotBrightness, double dispDepth, std::vector<Vector3d> dot_container, double dotRadius_RAD);
void drawFixation(double dispDistJitter);
void online_trial();


// blur stuff
void initBlur();
void drawBlurred(); // to test the blur implementation
void blurPass(CShaderProgram blur_shader, unsigned int texID); // does one pass through using blur with a frame buffer
void drawBlurInput();


// test original stimuli in the probe test 'fall20-ailin-brightnessDepth'
std::vector<Vector3d> buildRandomDotsA(double dispDepth)
{
	std::vector<Vector3d> dot_container;

	double edge = tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance) + dispDepth) ;
	double this_cylinder_width = 1.2 * edge;

	//height_stimulus = 55;
	for (int dots_placed = 0; dots_placed < 550; dots_placed++)
	{
		double x_axis = rand() % int(this_cylinder_width) - this_cylinder_width/2;
		double y_axis = rand() % int(edge) - edge/2;

		double z_axis = dispDepth * sqrt(1 - pow(y_axis/(edge/2.),2));

		dot_container.push_back(Vector3d(x_axis,y_axis,z_axis));


	}

	return dot_container;
}



void initBlur()
{
	// Load in the vertex and frament shader files to create two shader programs, one for horz blur and one for vert blur
	BlurH.Load((char*)(experiment_directory + "Shaders/blur.vs").c_str(), (char*)(experiment_directory + "/Shaders/blurh.fs").c_str());
	BlurV.Load((char*)(experiment_directory + "Shaders/blur.vs").c_str(), (char*)(experiment_directory + "/Shaders/blurv.fs").c_str());

	// set up uniforms for the horz and vert blur shaders
	BlurH.UniformLocations = new GLuint[2];
	BlurH.UniformLocations[0] = glGetUniformLocation(BlurH, "Width");
	BlurH.UniformLocations[1] = glGetUniformLocation(BlurH, "odw");

	BlurV.UniformLocations = new GLuint[2];
	BlurV.UniformLocations[0] = glGetUniformLocation(BlurV, "Width");
	BlurV.UniformLocations[1] = glGetUniformLocation(BlurV, "odh");

	glUseProgram(BlurH);
	glUniform1i(BlurH.UniformLocations[0], blurringBox_Width);
	glUseProgram(BlurV);
	glUniform1i(BlurV.UniformLocations[0], blurringBox_Width);
	glUseProgram(0);


	// not sure yet
	//glGenTextures(3, &sceneBuffer);
	glGenTextures(1, &sceneBuffer);
	glGenFramebuffers(1, &FBO);
	data = new float[4096];
}

// adapted from the drawBlur() in other scripts
void drawBlurred()
{
	// 1. draw stimulus and aperture --------------------------------------------- // 
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	// generate the fbo
	GLuint fbo;
	glGenFramebuffers(1, &fbo);

	//bind the fbo
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	//create a texture
	unsigned int firstPass;
	glGenTextures(1, &firstPass);
	glBindTexture(GL_TEXTURE_2D, firstPass);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SCREEN_WIDTH, SCREEN_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// attached texture to the framebuffer
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, firstPass, 0);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {

		////////////////////////////////// draw what you want to blur /////////////////
		drawBlurInput();
		//drawRandomDots(1.0, depth_stimulus, dotContainer_stimulus, atan(dot_size_bfBlur / abs(display_distance)));
		///////////////////////////////////////////////////////////////////////////////
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glDeleteFramebuffers(1, &fbo);
	}
	else {
		cout << "Framebuffer not complete" << endl;
	}
	///// we have an texture image of the scene called texture

	// 2. apply the (first) horizontal blur --------------------------------------------- // 
	blurPass(BlurH, firstPass);

	// 3. repeat vertical+horizontal blurs to increase the blur magnitude -------------- // 

	//int blur_mag = 1;
	if (blur_extraPasses > 0) {
		for (int passes = 0; passes < blur_extraPasses; passes++) {
			blurPass(BlurV, firstPass);
			blurPass(BlurH, firstPass);
		}
	}

	// 4. apply (final) vertical blur and draw ----------------------------------------- // 
	// this time, we actually draw the observable scene, instead of drawing it into frame buffer
	// apply vert blur pass and draw

	glUseProgram(BlurV);
	double ds = 1.0;
	glUniform1f(BlurV.UniformLocations[1], 1.0f / (float)(Width / ds));

	// Draw final plane with blurred image
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, firstPass); // is this the 2 buffer?

	//double horzOffset = 1.2 * tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance))/2.0; // ~30
	//double vertOffset = 2.4 * tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance))/3.0;
	double horzOffset = 0;
	double vertOffset = 0;

	glBegin(GL_QUADS);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-SCREEN_WIDE_SIZE / 2 - horzOffset, SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(SCREEN_WIDE_SIZE / 2 - horzOffset, SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(SCREEN_WIDE_SIZE / 2 - horzOffset, -SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-SCREEN_WIDE_SIZE / 2 - horzOffset, -SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	glDeleteTextures(1, &firstPass);
	glUseProgram(0);

}




void blurPass(CShaderProgram blur_shader, unsigned int texID)
{
	// generate the fbo
	GLuint fbo;
	glGenFramebuffers(1, &fbo);

	//bind the fbo
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	// attached texture to the framebuffer
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texID, 0);

	glUseProgram(blur_shader);
	float ds = 1.0;

	glUniform1f(blur_shader.UniformLocations[1], 1.0f / (float)(Width / ds));

	// Draw plane with blurred image
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texID); // is this the 2 buffer?

	//double horzOffset = 1.2 * tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance))/2.0;
	//double vertOffset = 2.4 * tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance))/3.0;
	double horzOffset = 0;
	double vertOffset = 0;

	//draw the plane with the texture saved by the frame buffer on it

	glBegin(GL_QUADS);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-SCREEN_WIDE_SIZE / 2 - horzOffset, SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(SCREEN_WIDE_SIZE / 2 - horzOffset, SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(SCREEN_WIDE_SIZE / 2 - horzOffset, -SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-SCREEN_WIDE_SIZE / 2 - horzOffset, -SCREEN_HIGH_SIZE / 2 - vertOffset, 0.0f);
	glEnd();



	// unbind texture and shader program after drawing
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	glUseProgram(0);

	// unbind and delete the framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteFramebuffers(1, &fbo);

}


// This function seems to be used to shut down the system after use
void shutdown() {

	responseFile.close(); // close this object
	if (resetScreen_betweenRuns)
		homeEverything(5000, 4500);
	cleanup();
	exit(0);
}
void cleanup()
{
	// Stop the optotrak
	optotrak->stopCollection();
	delete optotrak;
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
		moveScreenAbsolute(_focalDist, homeFocalDistance, 4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 4500);
}
// Initialize Optotrak for use in the experiment
void initOptotrak()
{
	optotrak = new Optotrak2(); //intiailize the Optotrak object
	optotrak->setTranslation(calibration);

	//define Optotrak-specific variables
	int numMarkers = 22;
	float frameRate = 85.0f;
	float markerFreq = 4600.0f;
	float dutyCycle = 0.4f;
	float voltage = 7.0f;

	// run the intiailization method for the Optotrak, checking to see if ever (if == 0) and catch the error if so
	if (optotrak->init("C:/cncsvisiondata/camerafiles/Aligned20111014", numMarkers, frameRate, markerFreq, dutyCycle, voltage) != 0)
	{
		cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
		cin.ignore(1E6, '\n');
		exit(0);
	}

	// Read 10 frames of coordinates and fill the markers vector
	for (int i = 0; i < 10; i++)
	{
		updateTheMarkers();
	}
}
// run a method to define a vector that holds marker positions 
void updateTheMarkers()
{
	optotrak->updateMarkers();
	markers = optotrak->getAllMarkers();

}
// Initialize motors for moving screen around
void initMotors()
{
	//specify the speed for (objects,screen)
	if (resetScreen_betweenRuns)
		homeEverything(5000, 4500);
}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
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
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
	// ballerine (le vedi vibrare)
	glLineWidth(1.5);

}


void initStreams()
{
	// trialFile directory
	string dirName = experiment_directory + subjectName;
	mkdir(dirName.c_str()); // windows syntax

	// Throw an error if the subject name is already used. Don't want to overwrite good data!
	if (util::fileExists(dirName + "/" + subjectName + "_s" + parameters.find("session") + ".txt")) // when writing distractor task data to file, use this style of filename
	{
		string error_on_file_io = subjectName + string(" already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
		exit(0);
	}
	string responseFileName = dirName + "blurParameterRecords.txt";

	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;

}



void initVariables()
{

	// eye coordinates
	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
	eyeMiddle = Vector3d(0, 0, 0);

	initTrial();
	initProjectionScreen(display_distance);

}

//Central function for projecting image onto the screen
void drawGLScene()
{
	// Draw left eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeLeft);

	if (blur_left_eye) {
		drawStimulus_blur_eye();
	}
	else {
		drawStimulus_clear_eye();
	}

	drawInfo();


	// Draw right eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeRight);

	if (blur_right_eye) {
		drawStimulus_blur_eye();
	}
	else {
		drawStimulus_clear_eye();
	}

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
	if (visibleInfo)
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);

		GLText text;

		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();


		glColor3fv(glRed);
		text.draw("or press m n a s to change the stimulus");
		text.draw(" ");
		text.draw("# IOD: " + stringify<double>(interoculardistance));
		text.draw("# depth: " + stringify<double>(depth_stimulus));

		// check if mirror is calibrated
		if (abs(mirrorAlignment - 45.0) > 0.2)
			text.draw("# Mirror Alignment = " + stringify<double>(mirrorAlignment));

		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}

/***** SOUNDS *****/
void beepOk(int tone)
{

	switch (tone)
	{
	case 0:
		PlaySound((LPCSTR)"C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 1:
		PlaySound((LPCSTR)"C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-success.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 3:
		PlaySound((LPCSTR)"C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-one.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 4:
		PlaySound((LPCSTR)"C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-two.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;


	}
	return;
}
void idle()
{
	online_trial();
	online_apparatus_alignment();
	ElapsedTime = trial_time.getElapsedTimeInMilliSec();

}


/*** Online operations ***/
void online_apparatus_alignment()
{
	updateTheMarkers();
	// mirror alignment check
	mirrorAlignment = asin(
		abs((markers.at(mirror1).p.z() - markers.at(mirror2).p.z())) /
		sqrt(
			pow(markers.at(mirror1).p.x() - markers.at(mirror2).p.x(), 2) +
			pow(markers.at(mirror1).p.z() - markers.at(mirror2).p.z(), 2)
		)
	) * 180 / M_PI;

	// screen Y alignment check
	screenAlignmentY = asin(
		abs((markers.at(screen1).p.y() - markers.at(screen3).p.y())) /
		sqrt(
			pow(markers.at(screen1).p.x() - markers.at(screen3).p.x(), 2) +
			pow(markers.at(screen1).p.y() - markers.at(screen3).p.y(), 2)
		)
	) * 180 / M_PI;

	// screen Z alignment check
	screenAlignmentZ = asin(
		abs(markers.at(screen1).p.z() - markers.at(screen2).p.z()) /
		sqrt(
			pow(markers.at(screen1).p.x() - markers.at(screen2).p.x(), 2) +
			pow(markers.at(screen1).p.z() - markers.at(screen2).p.z(), 2)
		)
	) * 180 / M_PI *
		abs(markers.at(screen1).p.x() - markers.at(screen2).p.x()) /
		(markers.at(screen1).p.x() - markers.at(screen2).p.x());

}



void drawFixation(double dispDistJitter) {
	// draws a small fixation cross at the center of the display
	glDisable(GL_TEXTURE_2D);
	glColor3f(0.7f, 0.0f, 0.0f);
	glLineWidth(2.f);
	glLoadIdentity();
	glTranslated(0, 0, display_distance + dispDistJitter);

	double cross_x = 4;
	double cross_y = 4;
	glBegin(GL_LINES);
	glVertex3d(cross_x / 2., 0, 0);
	glVertex3d(-cross_x / 2, 0, 0);
	glVertex3d(0, -cross_y / 2., 0);
	glVertex3d(0, cross_y / 2., 0);
	glEnd();

}


std::vector<Vector3d> buildRandomDotsB(double dispDepth)
{
	std::vector<Vector3d> dot_container;


	dot_per_col = 16, dot_per_row = 16;
	height_stimulus = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance));
	

	double temp_height = 1.5 * height_stimulus, temp_width = temp_height;
	double step_x = temp_width / (dot_per_row - 1), step_y = (temp_height) / (dot_per_col - 1); //step around 3.7

	double theta = rand() % 51 + 20.0;
	double cos_theta = cos(DEG2RAD * theta), sin_theta = sin(DEG2RAD * theta);

	if (height_stimulus / 2. < dispDepth) {

		double x_temp, y_temp, x, y, z;

		for (int i_y = 0; i_y < dot_per_col; i_y++) {

			for (int i_x = 0; i_x < dot_per_row; i_x++) {

				x_temp = i_x * step_x - temp_width / 2 +
					(rand() % 17) / 16.0 * jitter_x_max - jitter_x_max / 2;

				y_temp = i_y * step_y - temp_height / 2 +
					(rand() % 17) / 16.0 * jitter_y_max - jitter_y_max / 2;

				x = x_temp * cos_theta + y_temp * sin_theta;
				y = -x_temp * sin_theta + y_temp * cos_theta;

				if (abs(y) < height_stimulus / 2) {

					z = dispDepth * sqrt(1 - pow(y / (height_stimulus / 2.), 2));

					dot_container.push_back(Vector3d(x, y, z));

				}
			}
		}
	}
	else {
		double R = (pow(dispDepth, 2) + pow((height_stimulus / 2.), 2)) / (2 * dispDepth);

		double x_temp, y_temp, x, y, z;

		for (int i_y = 0; i_y < dot_per_col; i_y++) {

			for (int i_x = 0; i_x < dot_per_row; i_x++) {

				x_temp = i_x * step_x - temp_width / 2 +
					(rand() % 17) / 16.0 * jitter_x_max - jitter_x_max / 2;

				y_temp = i_y * step_y - temp_height / 2 +
					(rand() % 17) / 16.0 * jitter_y_max - jitter_y_max / 2;

				x = x_temp * cos_theta + y_temp * sin_theta;
				y = -x_temp * sin_theta + y_temp * cos_theta;

				if (abs(y) < height_stimulus / 2) {

					z = sqrt(pow(R, 2) - pow(y, 2)) - R + dispDepth;

					dot_container.push_back(Vector3d(x, y, z));

				}

			}
		}

	}
	return dot_container;
}



void drawRandomDots(float dotBrightness, double dispDepth, std::vector<Vector3d> dot_container, double dotRadius_RAD)
{
	if (stimlus_built) {
		glLoadIdentity();
		glColor3f(dotBrightness, 0.0f, 0.0f);

			glTranslated(0, 0, display_distance - dispDepth);
			if (cylHorizontal == 0) {
				glRotatef(90.0, 0.0, 0.0, 1.0); // 0, 0, 1
			}

			//glColor3f(dotBrightness, 0.0f, 0.0f);
			for (int i = 0; i < int(dot_container.size()); i++)
			{
				Vector3d dot_vector = dot_container.at(i);
				double x_axis = dot_vector[0];
				double y_axis = dot_vector[1];
				double z_axis = dot_vector[2];

				glPushMatrix();
				glTranslated(x_axis, y_axis, z_axis);

				double dot_size = tan(dotRadius_RAD) * abs(display_distance - dispDepth  + z_axis);
				glutSolidSphere(dot_size, 10, 10);
				glPopMatrix();
			}

	}

}



void drawStimulus_clear_eye()
{
	drawRandomDots(brightness_clear_dot, depth_stimulus, dotContainer_stimulus, DEG2RAD * dot_visangle / 2.);
}

// the content needs to be blurred
void drawBlurInput() {
	drawRandomDots(1.0, depth_stimulus, dotContainer_stimulus, atan(dot_size_bfBlur / abs(display_distance)));
}

void drawStimulus_blur_eye()
{
	drawBlurred();
}



void initTrial()
{
	stimlus_built = false;

	dotContainer_stimulus = buildRandomDotsB(depth_stimulus);

	stimlus_built = true;


}



// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

}
// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{
	//cout << "listening for keypress" << endl;
	switch (key)
	{

	case 'Q':
	case 'q':
	case 27:	//corrisponde al tasto ESC
	{

		shutdown();
	}
	break;


	case 'i':
		visibleInfo = !visibleInfo;
		break;

	case '+':
		// "IOD\tdisplayDistance\tblurBoxWidth\tblurExtraPasses\tblurDotSize";
		responseFile << fixed <<
			interoculardistance << "\t" <<
			display_distance << "\t" <<
			blurringBox_Width << "\t" <<
			blur_extraPasses << "\t" <<
			dot_size_bfBlur << endl;

			break;

	case '0':
		blur_right_eye = !blur_right_eye;
		break;


	case '9':
		blur_left_eye = !blur_left_eye;
		break;


	// change depth
	case '1':
	{
		if (depth_stimulus > 5) {
			depth_stimulus = depth_stimulus - 2;
			initTrial();
		}
	}
	break;

	// change depth
	case '2':
	{
		if (depth_stimulus < 72) {
			depth_stimulus = depth_stimulus + 2;
			initTrial();
		}
	}
	break;


	// change dot size
	case '4':
		if (dot_size_bfBlur > 0.3) {
			dot_size_bfBlur = dot_size_bfBlur - 0.02;
			//initTrial();
		}
		break;

	// change dot size
	case '5':
		dot_size_bfBlur = dot_size_bfBlur + 0.02;
		//initTrial();
		
		break;


	// change box_width
	case '3': // change box_width
	{
		stimlus_built = false;
		if(blurringBox_Width > 1)
			blurringBox_Width = blurringBox_Width - 1;

		glUseProgram(BlurH);
		glUniform1i(BlurH.UniformLocations[0], blurringBox_Width);
		glUseProgram(BlurV);
		glUniform1i(BlurV.UniformLocations[0], blurringBox_Width);
		glUseProgram(0);
		stimlus_built = true;

	}
	break;

	// change box_width
	case '6': // change box_width
	{
		stimlus_built = false;
		blurringBox_Width = blurringBox_Width + 1;

		glUseProgram(BlurH);
		glUniform1i(BlurH.UniformLocations[0], blurringBox_Width);
		glUseProgram(BlurV);
		glUniform1i(BlurV.UniformLocations[0], blurringBox_Width);
		glUseProgram(0);
		stimlus_built = true;

	}
	break;


	// change extra blurring passes
	case '7':
		if (blur_extraPasses > 0)
			blur_extraPasses = blur_extraPasses - 1;
		break;

	// change extra blurring passes
	case '8':
		blur_extraPasses = blur_extraPasses + 1;
		break;


		/*

		////////////////////// other changes we can make with the keypresses //////////////////

		// change shape type
		case '5':
			expShape = (ShapeTypes)((expShape + 1) % 3);
			initTrial();
			break;



		// change dot density
		case '7':
		{
			dot_per_col++;
			dot_per_row++;
			initTrial();
		}
		break;

		// change dot jitter
			case '8':
		{
			jitter_x_max = jitter_x_max + 0.1;
			jitter_y_max = jitter_y_max + 0.1;
			initTrial();
		}
		break;

		*/

	}
}


// this is run at compilation because it's titled 'main'
int main(int argc, char* argv[])
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();

	// initializes optotrak and velmex motors
	initOptotrak();
	initMotors();

	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);

	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();


	// initializing experiment's parameters

	initRendering(); // initializes the openGL parameters needed for creating the stimuli

	glewInit();
	initBlur();

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
	// When the program exists, clean up
	cleanup();
	return 0;
}
