// This file is used to test visual stimuli. It is also an illustration for how to build textured surface with triangles
// Method 1 is closer to the modern openGL
// Method 2 uses glBegin(TRIANGLES...). It's more straightforward for openGL novices.

// set resetScreen_betweenRuns = false to stop the monitor from moving between runs
// to change the display distance, change it from the variable declaration


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

/********* #DEFINE DIRECTIVES **************************/
#define TIMER_MS 11 // 85 hz
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
static const double SCREEN_WIDE_SIZE = 306;    // millimeters
const float DEG2RAD = M_PI / 180;
/********* 18 October 2011   CALIBRATION ON CHIN REST *****/
static const Vector3d calibration(160,179,-75);
//static const Vector3d objCalibration(199.1, -149.2, -319.6);
// Alignment between optotrak z axis and screen z axis
double alignmentX =  33.5;
double alignmentY =  33;
double focalDistance= -270.0, homeFocalDistance=-270.0;

Screen screen;
/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode=true;
static const bool stereo=true;
bool resetScreen_betweenRuns = false;
bool UseMethod1 = false;
/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
//Optotrak2 *optotrak;
CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance=60;
double eyeOffsetX = 0;
double omega = 0;
//double beta = 0;

/********* VISUAL STIMULI *********/
double display_distance = -400;
double visual_angle = 6; // stim diangonal size
double cylinder_height = tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double cylinder_width = 1.2 * cylinder_height;

// stimulus depth
double depth = 20;

// For texture:
double normalizer_base = 100; // maximum distance along the side on the sine wave fluctuating in depth
double normalizer = normalizer_base; //will vary based on display distance
int texnum = 2;

//TEXTURE LOADING VARIABLE
GLuint texture0[51];
//GLuint* texture[51];

double nr_points_width = 1001; // nr of points in x direction
double nr_points_height = 1001;
// Cylinder Buffers (for method 1)
GLfloat vertices[50000000]; // the vertices on the base(e.g. texture) surface
GLfloat texcoors[50000000]; // for uv coordinates on the texture mapping
GLfloat colors[50000000];//color arrays, where texture is defined
GLfloat normals[50000000];
GLuint indices[50000000]; // tracking indices number
int total_ind = 0, nr_vertices = 0;

// Cylinder Buffers (for method 2)
// the first dimension is the number of triangle strips. 
// the second dimension is the vertices on that triangle strip
GLfloat vertices_byStrip[2000][20000];
GLfloat texcoors_byStrip[2000][20000];
int total_strip = 0;

/********** FUNCTION PROTOTYPES *****/
void beepOk(int tone);
void drawGLScene();
void handleKeypress(unsigned char key, int x, int y);
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d &_transformation=Affine3d::Identity(),bool synchronous=true);
void update(int value);
void idle();
void initMotors();
void initGLVariables();
void initVariables();
void initStreams();
void initRendering();
void initTrial();
void drawStimulus();
double calculateZfromY(double cylDepth, double current_y);
void buildVertices1(double cylinderDepth);
void drawVertices1(int texNum);
void buildVertices2(double cylinderDepth);
void drawVertices2(int texNum);
int LoadGLTextures();

/*************************** FUNCTIONS ***********************************/
/***** SOUND THINGS *****/
void beepOk(int tone)
{
	switch(tone)
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
	if(UseMethod1)
		drawVertices1(texnum);
	else
		drawVertices2(texnum);
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


void buildVertices1(double cylinderDepth) {


	double step_size_width = (cylinder_width / (nr_points_width - 1));
	double step_size_height = (cylinder_height / (nr_points_height - 1));
	
	double total_distance_v = 0; //tracks the distance along y/z axis, approximate the "diameter" of the ellipse

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

	int v_index = 0, t_index = 0; // vertices[r_strip][v_index], texcoors[r_strip][t_index]
	double x, y, z, y_prev, z_prev;
	y_prev = -cylinder_height / 2; 
	z_prev = calculateZfromY(cylinderDepth, y_prev);

	for (y = -cylinder_height / 2; y < (cylinder_height / 2 + step_size_height / 2); y = y + step_size_height) {  // 

		z = calculateZfromY(cylinderDepth, y);
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

void drawVertices1(int texNum) {


	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	// enable matrices for use in drawing below
	//glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	//glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces

	// bind the texture

	glBindTexture(GL_TEXTURE_2D, texture0[texNum]);

	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
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

void buildVertices2(double cylinderDepth) {

	double step_size_width = (cylinder_width / (nr_points_width - 1));
	double step_size_height = (cylinder_height / (nr_points_height - 1));

	double total_distance_v = 0; //tracks the distance along y/z axis, approximate the "diameter" of the ellipse
	
	/*

		(8)---(9)---(10)--(11)
		 |     |     |     |
		 |     |     |     |
		(4)---(5)---(6)---(7)
		 |     |     |     |
		 |     |     |     |
		(0)---(1)---(2)---(3)

		4  0  5  1   6  2   7  3
		8  4  9  5  10  6  11  7

	*/

	int i_strip = 0;
	int v_index = 0, t_index = 0; // vertices[i_strip][v_index], texcoors[i_strip][t_index]
	double y_u = 0, y_b = 0, z_u = 0, z_b = 0; // u for upper, b for bottom
	double local_distance_v = 0;

	double z_u_prev = calculateZfromY(cylinderDepth, -cylinder_height / 2); // set its initial value

	for (y_b = -cylinder_height / 2; y_b < cylinder_height / 2; y_b = y_b + step_size_height) {  
		// y_bottom goes from -cylinder_height to the row below cylinder_height / 2
		
		v_index = 0;
		t_index = 0;
		
		z_b = z_u_prev; // the previous upper z becomes the current bottom z

		y_u = y_b + step_size_height;
		z_u = calculateZfromY(cylinderDepth, y_u);

		local_distance_v = sqrt(pow(y_b - y_u, 2) + pow(z_b - z_u, 2));

		for (double x = -cylinder_width / 2; x <= cylinder_width / 2; x = x + step_size_width) { //

			// upper vertex
			vertices_byStrip[i_strip][v_index] = x;
			v_index++;
			vertices_byStrip[i_strip][v_index] = y_u; //y
			v_index++;
			vertices_byStrip[i_strip][v_index] = z_u; //z
			v_index++;
			texcoors_byStrip[i_strip][t_index] = (x + cylinder_width / 2) / normalizer; //u coordinate
			t_index++;
			texcoors_byStrip[i_strip][t_index] = (total_distance_v + local_distance_v)/ normalizer; //v coordinate
			
			t_index++;

			// bottom vertex
			vertices_byStrip[i_strip][v_index] = x;
			v_index++;
			vertices_byStrip[i_strip][v_index] = y_b; //y
			v_index++;
			vertices_byStrip[i_strip][v_index] = z_b; //z
			v_index++;
			texcoors_byStrip[i_strip][t_index] = (x + cylinder_width / 2) / normalizer; //u coordinate
			t_index++;
			texcoors_byStrip[i_strip][t_index] = total_distance_v / normalizer; //v coordinate
			t_index++;
		}
		total_distance_v = total_distance_v + local_distance_v;
		i_strip++;
		z_u_prev = z_u;
	}

	total_strip = i_strip;

}


void drawVertices2(int texNum)
{

	int v_index = 0;//vertices[i_strip][v_index];
	int t_index = 0; //texcoors[i_strip][v_index]

	glColor3f(1.0f, 0.0f, 0.0f);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture0[texNum]);               // Select Our Texture
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glLoadIdentity();
	glTranslated(0,0,display_distance);

	for (int i_vrt = 0; i_vrt < total_strip; i_vrt++) {

		v_index = 0;
		t_index = 0; 

		// draw triangle fan and strips	
		glBegin(GL_TRIANGLE_STRIP);
		for (int i = 0; i < int(nr_points_width); i++) {

			glVertex3f(vertices_byStrip[i_vrt][v_index], vertices_byStrip[i_vrt][v_index + 1], vertices_byStrip[i_vrt][v_index + 2]);
			glTexCoord2f(texcoors_byStrip[i_vrt][t_index], texcoors_byStrip[i_vrt][t_index + 1]);
			
			v_index = v_index + 3;
			t_index = t_index + 2;
			
			glVertex3f(vertices_byStrip[i_vrt][v_index], vertices_byStrip[i_vrt][v_index + 1], vertices_byStrip[i_vrt][v_index + 2]);
			glTexCoord2f(texcoors_byStrip[i_vrt][t_index], texcoors_byStrip[i_vrt][t_index + 1]);
			v_index = v_index + 3;
			t_index = t_index + 2;

		}
		glEnd();

	}
}



void initTrial() {

	if(UseMethod1)
		buildVertices1(depth);
	else
		buildVertices2(depth);

	initProjectionScreen(display_distance);


}



void drawGLScene()
{
    if (stereo)

    {   //glDrawBuffer(GL_BACK);

        // Draw left eye view
        glDrawBuffer(GL_BACK_LEFT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0,0.0,0.0,1.0);
		
        cam.setEye(eyeRight); // we need to flip the eye because the image is mirrored
        drawStimulus();
		

        // Draw right eye view
        glDrawBuffer(GL_BACK_RIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0,0.0,0.0,1.0);
	
        cam.setEye(eyeLeft); // we need to flip the eye because the image is mirrored
        drawStimulus();
		
        glutSwapBuffers();
    }
    else
    {   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0,0.0,0.0,1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        cam.setEye(eyeMiddle);
        drawStimulus();
		
        glutSwapBuffers();
    }
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
{   switch (key)
    {   //Quit program

	
    case 27:	//corrisponde al tasto ESC
    {   
		if(resetScreen_betweenRuns)
			homeEverything(5000, 4500);
        exit(0);
    }
    break;

    // Enter key: press to make the final calibration

	  case 'm':
	  {		
		  depth = depth + 2;

		  if(UseMethod1)
			buildVertices1(depth);
		  else
			buildVertices2(depth);

	  }
	  break;

	  case 'n':
	  {
		   depth = depth - 2;

		   if(UseMethod1)
			  buildVertices1(depth);
		   else
			  buildVertices2(depth);

	  }
	  break;


	  case 'r':
	  {
		 display_distance = display_distance + 10;
	  }
	  break;

	  


	}
}


void handleResize(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,SCREEN_WIDTH, SCREEN_HEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
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
		moveScreenAbsolute(_focalDist,homeFocalDistance,3500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist,homeFocalDistance,3500);
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
glClearColor(0.0,0.0,0.0,1.0);
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
	if(resetScreen_betweenRuns)
		homeEverything(5000, 4500);
}

int main(int argc, char*argv[])
{
	mathcommon::randomizeStart();
    glutInit(&argc, argv);
    if (stereo)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    if (gameMode==false)
    {   glutInitWindowSize( 640,480 );
        glutCreateWindow("EXP WEXLER");
//glutFullScreen();
    }
    else
	{   glutGameModeString("1024x768:32@85");
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
