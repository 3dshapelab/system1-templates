// This is groud zero: building textured stereo cylinder. Not conflict between texture and disparity yet.
// to change the display distance, change it from the variable declaration
// to save time, we can set bool resetScreen_betweenRuns to false

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
#include "CylinderPointsStimulus.h"
#include "EllipsoidPointsStimulus.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "Util.h"
#include "SOIL.h"
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


/***** CALIBRATION FILE *****/
#include "LatestCalibration.h"
/*
#define TIMER_MS 11 // 85 hz
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
static const double SCREEN_WIDE_SIZE = 306;    // millimeters


static const Vector3d calibration(160, 179, -75);
//static const Vector3d objCalibration(199.1, -149.2, -319.6);
// Alignment between optotrak z axis and screen z axis
double alignmentX = 33.5;
double alignmentY = 33;
double focalDistance = -270.0, homeFocalDistance = -270.0;
*/
const float DEG2RAD = M_PI / 180;
Screen screen;
/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode = true;
bool resetScreen_betweenRuns = false;
bool apertureOn = false;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
//Optotrak2 *optotrak;
CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 64;
double eyeOffsetX = 0;
double omega = 0;
//double beta = 0;

/********* VISUAL STIMULI *********/
double display_distance = -420;
double visual_angle = 7.5; // stim diangonal size
double cylinder_height = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance));
double cylinder_width = 1.2 * cylinder_height;

// stimulus depth
double depth_disparity = 30;
double depth_texture = 30;
bool text_disp_conflict = false;

// For texture:
double normalizer_base = 95; // maximum distance along the side on the sine wave fluctuating in depth
double normalizer = normalizer_base; //will vary based on display distance
int texnum = 2;

//TEXTURE LOADING VARIABLE
GLuint texture0[51];
//GLuint* texture[51];

double nr_points_width = 501; // nr of points in x direction
double nr_points_height = 801;
// Cylinder Buffers (for method 1)
GLfloat vertices[2000000]; // the vertices on the base(e.g. texture) surface. rougly 3 * nr_points_width * nr_points_height
GLfloat texcoors[2000000]; // for uv coordinates on the texture mapping
GLfloat colors[2000000];//color arrays, where texture is defined
GLfloat normals[2000000];
GLuint indices[4000000]; // tracking indices number    /// roughly 6 * nr_points_width * nr_points_height 
int total_ind = 0, nr_vertices = 0;


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
void drawInfo();
void drawStimulus();
double calculateZfromY(double cylDepth, double current_y);
Vector3d projectedPoint_cyclopeanEye(Vector3d originalPoint, double newDepth);
void buildVertices(double DispDepth, double TextDepth);
void drawVertices(int texNum);
int LoadGLTextures();
void drawAperture();

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

void drawInfo()
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
	text.draw("#IOD: " + stringify<double>(interoculardistance) );
	text.draw("#DISPARITY depth: " + stringify<double>(depth_disparity) + " mm. Press  keys to change");
	text.draw("#TEXTURE depth: " + stringify<double>(depth_texture) + " mm. Press keys to change");
	//text.draw("__________");
	text.draw(" - - - - -");
	text.draw("| +D    +T |");
	text.draw("|               |");
	text.draw("| -D    -T |");
	text.draw(" - - - - -");
	//text.draw("__________");
	//text.draw(" - - - - -");
	text.leaveTextInputMode();
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
}

void drawStimulus()
{

	drawVertices(texnum);
	if(apertureOn){
		drawAperture();
	}
}

double calculateZfromY(double cylDepth, double current_y) {
	
	double current_z;

	if (abs(current_y) >= cylinder_height / 2) {
		current_z = 0;
	}
	else {
		current_z = cylDepth * sqrt(1 - pow(current_y / (cylinder_height / 2.), 2));
	}
	return (current_z);
}

Vector3d projectedPoint_cyclopeanEye(Vector3d originalPoint, double newDepth) {
	// the original point is one vertex on the original surface
	// we want to find the intersection between the ray (connecting the cyclopean eye and the original vertex) and the new surface
	// see the details in the lab materials/coding an experiment/cueConflictCylinder_TextureDisparity.pptx
	Vector3d newPoint;

	if (abs(originalPoint[1]) == cylinder_height / 2) {

		newPoint = originalPoint; // x_original

	}
	else {

		double h = cylinder_height / 2;
		double l = -display_distance;
		double a = originalPoint[1] / ((originalPoint[2] - l) * h);
		double b = 1 / pow(newDepth, 2);

		double z_projected = (sqrt(pow(a * l, 2) * (-b) + pow(a, 2) + b ) + pow(a, 2) * l) / (pow(a,2) + b);

		newPoint[0] = originalPoint[0]; // x_original
		newPoint[1] = originalPoint[1] * (z_projected - l)/(originalPoint[2] - l);
		newPoint[2] = z_projected;
	}

	return newPoint;

}


void drawAperture() {

	double panel_w = 30, panel_h = 40, panel_separation = 45;
	panel_separation = 1.1 * tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance  + 7));

	glLoadIdentity();

	glTranslated(0, 0, display_distance  + 7);
	
	//glRotatef(90.0, 0.0, 0.0, 1.0); // 0, 0, 1
	


	glColor3f(0.0f, 0.0f, 0.0f);
	
	glBegin(GL_QUADS);
	glVertex3f(-panel_separation / 2 - panel_w, panel_h, 0.0f);
	glVertex3f(-panel_separation / 2, panel_h, 0.0f);
	glVertex3f(-panel_separation / 2, -panel_h, 0.0f);
	glVertex3f(-panel_separation / 2 - panel_w, -panel_h, 0.0f);
	glEnd();

	glBegin(GL_QUADS);
	glVertex3f(panel_separation / 2, panel_h, 0.0f);
	glVertex3f(panel_separation / 2 + panel_w, panel_h, 0.0f);
	glVertex3f(panel_separation / 2 + panel_w, -panel_h, 0.0f);
	glVertex3f(panel_separation / 2, -panel_h, 0.0f);
	glEnd();
}




void buildVertices(double DispDepth, double TextDepth) {

	cylinder_height = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance - DispDepth));
	cylinder_width = 1.2 * cylinder_height;

	double step_size_width = (cylinder_width / (nr_points_width - 1));
	double step_size_height = (cylinder_height / (nr_points_height - 1));

	double total_distance_v = 2; //tracks the distance along y/z axis, approximate the "diameter" of the ellipse


	// step 1: we build the meshgrid	
	/*


		(8)---------(9)---------(10)--------(11)
		 |           |           |           |
		 |           |           |           |
		 |           |           |           |
		(4)---------(5)---------(6)---------(7)
		 |           |           |           |
		 |           |           |           |
		 |           |           |           |
		(0)---------(1)---------(2)---------(3)

		triangle 1: 0 1 4;   triangle 2: 1 5 4 ...
	*/

	// project the points on the shallower surface onto the deeper surface

	if (DispDepth == TextDepth) {

		double cylinderDepth = DispDepth;

		int v_index = 0, t_index = 0; // vertices[r_strip][v_index], texcoors[r_strip][t_index]
		double x, y, z, y_prev, z_prev;
		y_prev = -cylinder_height / 2;
		z_prev = 0;

		for (y = -cylinder_height / 2; y < (cylinder_height / 2 + step_size_height / 2); y = y + step_size_height) {  // 

			if (abs(y) >= cylinder_height / 2) {
				z = 0;
			}
			else {
				z = cylinderDepth * sqrt(1 - pow(y / (cylinder_height / 2.), 2));
			}

			total_distance_v = total_distance_v + sqrt(pow(y - y_prev, 2) + pow(z - z_prev, 2));

			for (x = -cylinder_width / 2; x < (cylinder_width / 2 + step_size_width / 2); x = x + step_size_width) { //

				// current vertex
				vertices[v_index] = x;
				colors[v_index] = 1;
				//normals[v_index] = 0;
				v_index++;
				vertices[v_index] = y;
				colors[v_index] = 0;
				//normals[v_index] = y * cylinderDepth * cylinderDepth;
				v_index++;
				vertices[v_index] = z;
				colors[v_index] = 0;
				//normals[v_index] = z * cylinder_height * cylinder_height /4;
				v_index++;

				texcoors[t_index] = (x + cylinder_width / 2) / normalizer; //u coordinate. 
				t_index++;
				texcoors[t_index] = total_distance_v / normalizer; //v coordinate
				t_index++;

			}

			y_prev = y; z_prev = z;
		}
		nr_vertices = v_index / 3;
	}
	else {

		int v_index = 0, t_index = 0; // vertices[r_strip][v_index], texcoors[r_strip][t_index]
		double x, y, z;
		Vector3d DispPoint, TextPoint, TextPoint_prev;
		TextPoint_prev = Vector3d(0, -cylinder_height / 2, 0);

		for (y = -cylinder_height / 2; y < (cylinder_height / 2 + step_size_height / 2); y = y + step_size_height) {  // 

			if(DispDepth < TextDepth){

				z = calculateZfromY(DispDepth, y);

				DispPoint = Vector3d(0, y, z); //x remains the same so just take the value of 0 for convenience
				TextPoint = projectedPoint_cyclopeanEye(DispPoint, TextDepth);
			}
			else {

				z = calculateZfromY(TextDepth, y);

				TextPoint = Vector3d(0, y, z); //x remains the same so just take the value of 0 for convenience
				DispPoint = projectedPoint_cyclopeanEye(TextPoint, DispDepth);
			}

			total_distance_v = total_distance_v + sqrt(pow(TextPoint[1] - TextPoint_prev[1], 2) + pow(TextPoint[2] - TextPoint_prev[2], 2));

			for (x = -cylinder_width / 2; x < (cylinder_width / 2 + step_size_width / 2); x = x + step_size_width) { //

				// current vertex
				vertices[v_index] = x;
				colors[v_index] = 1;
				//normals[v_index] = 0;
				v_index++;
				vertices[v_index] = DispPoint[1];
				colors[v_index] = 0;
				//normals[v_index] = TextPoint[1] * TextDepth * TextDepth;
				v_index++;
				vertices[v_index] = DispPoint[2];
				colors[v_index] = 0;
				//normals[v_index] = TextPoint[2] * cylinder_height * cylinder_height /4;
				v_index++;

				texcoors[t_index] = (x + cylinder_width / 2) / normalizer; //u coordinate. 
				t_index++;
				texcoors[t_index] = total_distance_v / normalizer; //v coordinate
				t_index++;

			}

			TextPoint_prev = TextPoint;
		}
	
		nr_vertices = v_index / 3;
	}


	// step 2: indices is the array that stores the triangle vertices. Every three vertices form a triangle [0,4,5,  0,5,1, ...]
	/*

		 |     |     |     |
		 |     |     |     |
		(4)---(5)---(6)---(7)
		 |\    |     |     |
		 | \   |     |     |
		 |  \  |     |     |
		 |   \ |     |     |
		(0)---(1)---(2)---(3)


	 triangle 1: 0 1 4;   triangle 2: 1 5 4
	 triangle 3: 1 2 5;   triangle 2: 2 6 5 ...
	*/

	int nr_vertices_per_row = nr_points_width;
	int ind = 0; // the count of the ind

	for (int vi = 0; vi < nr_vertices - nr_vertices_per_row; vi++) { // not from the last row
		if ((vi % nr_vertices_per_row) != (nr_vertices_per_row - 1)) { // meaning it's not the last vertex of the row, or not from the last column
			indices[ind] = vi; ind++;
			indices[ind] = vi + 1; ind++;
			indices[ind] = vi + nr_vertices_per_row; ind++;

			indices[ind] = vi + 1; ind++;
			indices[ind] = vi + nr_vertices_per_row + 1; ind++;
			indices[ind] = vi + nr_vertices_per_row; ind++;
		}
	}
	total_ind = ind;

}

void drawVertices(int texNum) {


	glLoadIdentity();
	glTranslated(0, 0, display_distance - depth_disparity);
	//glTranslated(0, 0, display_distance);

	// enable matrices for use in drawing below
	//glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);

	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces

	// bind the texture

	glBindTexture(GL_TEXTURE_2D, texture0[texNum]);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	//glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, vertices);
	glTexCoordPointer(2, GL_FLOAT, 0, texcoors);
	//glNormalPointer(GL_FLOAT, 0, normals); //
	glColorPointer(3, GL_FLOAT, 0, colors);

	glDrawElements(GL_TRIANGLES, total_ind, GL_UNSIGNED_INT, indices);

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	//glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

}



void initTrial() {

	buildVertices(depth_disparity, depth_texture);

	initProjectionScreen(display_distance);

}



void drawGLScene()
{

	//glDrawBuffer(GL_BACK);

	 // Draw left eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);

	cam.setEye(eyeRight);
	drawStimulus();
	drawInfo();

	// Draw right eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);

	cam.setEye(eyeLeft);
	drawStimulus();
	drawInfo();

	glutSwapBuffers();


}

int LoadGLTextures()  // Load PNG And Convert To Textures
{

	for (int i = 1; i <= 5; i++) {
		std::stringstream ss;
		ss << i;

		string texturePath = "fall16-ailin-stimulusTest/0/polkadots" + ss.str() + ".png";
		texture0[i] = SOIL_load_OGL_texture
		(
			texturePath.c_str(),
			SOIL_LOAD_AUTO,
			SOIL_CREATE_NEW_ID,
			SOIL_FLAG_MULTIPLY_ALPHA
		);
	}


	return true; // Return Success
}




// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{
	switch (key)
	{   //Quit program


	case 27:	//corrisponde al tasto ESC
	{
		if (resetScreen_betweenRuns)
			homeEverything(5000, 4500);

		exit(0);
	}
	break;

	// Enter key: press to make the final calibration

	case '4':
	{
		depth_disparity = depth_disparity + 2;
		buildVertices(depth_disparity, depth_texture);
	



	}
	break;

	case '1':
	{
		depth_disparity = depth_disparity - 2;

		buildVertices(depth_disparity, depth_texture);


	}
	break;

	case '5':
	{
		depth_texture = depth_texture + 2;

		buildVertices(depth_disparity, depth_texture);



	}
	break;

	case '2':
	{
		depth_texture = depth_texture - 2;

		buildVertices(depth_disparity, depth_texture);

	}
	break;


	case 'r':
	{
		display_distance = display_distance + 10;
	}
	break;

	case '+':
		{
		apertureOn = !apertureOn;
		}
		break;




	}
}


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
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
	// ballerine (le vedi vibrare)
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

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);


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
	LoadGLTextures();
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
