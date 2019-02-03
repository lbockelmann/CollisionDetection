//Liam Bockelmann ID: 108613490

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glut.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <cstdio>
#include <vector>
#include "myHeader.h"
#include "shapes.h"

//Window Size
GLsizei winWidth = 1200, winHeight = 800;
const int MAXSHAPES = 2;
void* shapePointers[MAXSHAPES] = { 0 };
void* OBBPointers[MAXSHAPES] = { 0 };
bool wireframe = false;
bool displayOBB = false;
int objDragSetter = -1;

GLfloat colorDraw[3] = { 1.0,1.0,1.0 };
GLfloat colorBackground[3] = { 0.0,0.0,0.0 };

#define PI 3.14159265358979323846  // pi
#define X .525731112119133606 * 20
#define Z .850650808352039932 * 20
#define DEGREE 0.0174533

static Vector icoVData[12] = {
	{ Vector(-X,0.0,Z) },{ Vector(X,0.0,Z) },{ Vector(-X,0.0,-Z) },{ Vector(X,0.0,-Z) },
	{ Vector(0.0,Z,X) },{ Vector(0.0,Z,-X) },{ Vector(0.0,-Z,X) },{ Vector(0.0,-Z,-X) },
	{ Vector(Z,X,0.0) },{ Vector(-Z,X,0.0) },{ Vector(Z,-X,0.0) },{ Vector(-Z,-X,0) }
};

static GLuint icoTIndices[20][3] = {
	{ 1,4,0 },{ 4,9,0 },{ 4,5,9 },{ 8,5,4 },{ 1,8,4 },
	{ 1,10,8 },{ 10,3,8 },{ 8,3,5 },{ 3,2,5 },{ 3,7,2 },
	{ 3,10,7 },{ 10,6,7 },{ 6,11,7 },{ 6,0,11 },{ 6,1,0 },
	{ 10,1,6 },{ 11,0,9 },{ 2,11,9 },{ 5,2,9 },{ 11,2,7 }
};

//Ellipse variables
Vector ellipseBound = Vector(1.0, 1.0, 1.0);

static GLfloat elipseVData[12][3];

float triangleSize = 2;

Vector shapePosition = Vector(0.0, 0.0, 0.0);
float maxSpeed = 0.2;
float minSpeed = 0.01;
Vector velocity(0.1, 0.0, 0.0);

Vector vData[12] = {
	{ Vector(0.0+5,triangleSize,0.0) },{ Vector(-triangleSize+5,-triangleSize,0.0) },{ Vector(triangleSize+5,-triangleSize,0.0) },{ Vector(0.0+5,-triangleSize,-triangleSize) },
	{ Vector(0.0,triangleSize+5,0.0) },{ Vector(-triangleSize,-triangleSize+5,0.0) },{ Vector(triangleSize,-triangleSize+5,0.0) },{ Vector(0.0,-triangleSize+5,-triangleSize) },
	{ Vector(0.0,triangleSize,0.0+5) },{ Vector(-triangleSize,-triangleSize,0.0+5) },{ Vector(triangleSize,-triangleSize,0.0+5) },{ Vector(0.0,-triangleSize,-triangleSize+5) }
};


//Helper function to draw triangle and set normals
void drawTriangle(Vector v1, Vector v2, Vector v3)
{
	glBegin(GL_TRIANGLES);
	glNormal3f(v1.x,v1.y,v1.z);
	glVertex3f(v1.x, v1.y, v1.z);
	glNormal3f(v2.x,v2.y,v2.z);
	glVertex3f(v2.x, v2.y, v2.z);
	glNormal3f(v3.x,v3.y,v3.z);
	glVertex3f(v3.x, v3.y, v3.z);
	glEnd();
}

//Function to draw an icosehedron, uses the subdivide function and passes its own depth parameter to it
void drawIco()
{
	int i;
	for (i = 0; i < 20; i++)
	{
		drawTriangle(icoVData[icoTIndices[i][0]], icoVData[icoTIndices[i][1]], icoVData[icoTIndices[i][2]]);
	}
}


//Function to draw an ellipse using the parametric equation with triangle strips. 
//Concentrics and slices determine definition(num of triangles) for the shape
void drawEllipsoid(unsigned int concentrics, unsigned int slices, float rx, float ry, float rz, float xShift, float yShift, float zShift)
{
	float tStep = (PI) / (float)slices;
	float sStep = (PI) / (float)concentrics;
	for (float t = -PI / 2; t <= (PI / 2) + .0001; t += tStep)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for (float s = -PI; s <= PI + .0001; s += sStep)
		{
			elipseVData[0][0] = rx * cos(t) * cos(s) + xShift;
			elipseVData[0][1] = ry * cos(t) * sin(s) + yShift;
			elipseVData[0][2] = rz * sin(t) + zShift;
			glNormal3fv(&elipseVData[0][0]);
			glVertex3f(rx * cos(t) * cos(s) + xShift, ry * cos(t) * sin(s) + yShift, rz * sin(t) + zShift);
			elipseVData[0][0] = rx * cos(t + tStep) * cos(s) + xShift;
			elipseVData[0][1] = ry * cos(t + tStep) * sin(s) + yShift;
			elipseVData[0][2] = rz * sin(t + tStep) + zShift;
			glNormal3fv(&elipseVData[0][0]);
			glVertex3f(rx * cos(t + tStep) * cos(s) + xShift, ry * cos(t + tStep) * sin(s) + yShift, rz * sin(t + tStep) + zShift);
		}
		glEnd();
	}
}

void init(void)
{
	glClearColor(colorBackground[0], colorBackground[1], colorBackground[2], 0.0);

	GLfloat lightPosition[] = { -5.0,1.0,5.0,0.0 };
	GLfloat modelAmbient[] = { 0.5,0.5,0.5,1.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, modelAmbient);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);

	glLoadIdentity();
	gluLookAt(0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glScalef(1.0, 1.0, 1.0);


	shapePointers[0] = new Teapot();
	OBBPointers[0] = new OBB();
	createOBBData((OBB*)OBBPointers[0], ((Teapot*)shapePointers[0])->vData, ((Teapot*)shapePointers[0])->vertexNum);
	splitOBB((OBB*)OBBPointers[0], ((Teapot*)shapePointers[0])->vData, ((Teapot*)shapePointers[0])->vertexNum);
	shapePointers[1] = new Bunny();
	OBBPointers[1] = new OBB();
	createOBBData((OBB*)OBBPointers[1], ((Bunny*)shapePointers[1])->vData, ((Bunny*)shapePointers[1])->vertexNum);
	splitOBB((OBB*)OBBPointers[1], ((Bunny*)shapePointers[1])->vData, ((Bunny*)shapePointers[1])->vertexNum);
	//splitOBB((OBB*)OBBPointers[0], ((Teapot*)shapePointers[0])->vData, ((Teapot*)shapePointers[0])->vertexNum);
	//splitOBB((OBB*)OBBPointers[0], ((Bunny*)shapePointers[0])->vData, ((Bunny*)shapePointers[0])->vertexNum);
	//shapePointers[0] = new Diamond();
	//OBBPointers[0] = new OBB();
	//createOBBData((OBB*)OBBPointers[0], ((Diamond*)shapePointers[0])->vData, ((Diamond*)shapePointers[0])->vertexNum);
	//splitOBB((OBB*)OBBPointers[0], ((Diamond*)shapePointers[0])->vData, ((Diamond*)shapePointers[0])->vertexNum);
	//shapePointers[1] = new Teapot();
	//shapePointers[2] = new Diamond();
	//shapePointers[3] = new Teapot();
	//shapePointers[4] = new Teapot();
	//shapePointers[5] = new Diamond();



}

void winReshapeFcn(GLint newWidth, GLint newHeight)
{
	glViewport(0, 0, newWidth, newHeight);

	glMatrixMode(GL_PROJECTION);
	glFrustum(-1.0, 1.0, -1.0, 1.0, 2.0, 50.0);

	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void drawEnviornment(void)
{
	/*
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glScalef(20.0,20.0,20.0);
	glutSolidTetrahedron();
	glScalef(0.05, 0.05, 0.05);
	glTranslatef(5.0, 0.0, 0.0);
	glutSolidTetrahedron();
	glTranslatef(-5.0, 5.0, 0.0);
	glutSolidTetrahedron();
	glTranslatef(0.0, -5.0, 5.0);
	glutSolidTetrahedron();
	glPopMatrix();
	*/
	glColor3f(0.0, 0.0, 1.0);
	drawTriangle(vData[0], vData[1], vData[2]);
	drawTriangle(vData[0], vData[1], vData[3]);
	drawTriangle(vData[0], vData[2], vData[3]);
	drawTriangle(vData[1], vData[2], vData[3]);

	drawTriangle(vData[4], vData[5], vData[6]);
	drawTriangle(vData[4], vData[5], vData[7]);
	drawTriangle(vData[4], vData[6], vData[7]);
	drawTriangle(vData[5], vData[6], vData[7]);

	drawTriangle(vData[8], vData[9], vData[10]);
	drawTriangle(vData[8], vData[9], vData[11]);
	drawTriangle(vData[8], vData[10], vData[11]);
	drawTriangle(vData[9], vData[10], vData[11]);

	glColor3f(0.0, 0.0, 0.2);
	drawIco();
	glColor3f(colorDraw[0], colorDraw[1], colorDraw[2]);
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(colorDraw[0], colorDraw[1], colorDraw[2]);
	glShadeModel(GL_SMOOTH);
	if(wireframe == true)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glLoadIdentity();
	//gluLookAt(0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	//glScalef(1.0, 1.0, 1.0);
	//drawEllipsoid(5, 5, ellipseBound.x, ellipseBound.y, ellipseBound.z, shapePosition.x, shapePosition.y, shapePosition.z);

	for (int i = 0; (i < MAXSHAPES) && (shapePointers[i] != 0); i++)
	{
		if (((Teapot*)shapePointers[i])->vertexNum == 530)
			drawTeapot((Teapot*)shapePointers[i]);
		else if (((Diamond*)shapePointers[i])->vertexNum == 6)
			drawDiamond((Diamond*)shapePointers[i]);
		else if (((Bunny*)shapePointers[i])->vertexNum == 2503)
			drawBunny((Bunny*)shapePointers[i]);
		if(displayOBB == true)
			drawOBBTree((OBB*)OBBPointers[i]);
		//drawOBB(((OBB*)OBBPointers[0])->child1);
		//drawOBB(((OBB*)OBBPointers[0])->child2);
	}

	//drawEnviornment();


	glFlush();
}

void onKey(unsigned char key, int x, int y)
{
	/*if (key == 's')
	{
		//while ((velocity.x != 0.0) || (velocity.y != 0.0) || (velocity.z != 0.0))
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			drawEnviornment();
	
			CollisionPacket check = CollisionPacket(ellipseBound, velocity, shapePosition, velocity, normalize2(velocity), shapePosition, false);
			
			
			checkTriangle(&check,vData[0], vData[1], vData[2]);
			checkTriangle(&check, vData[0], vData[1], vData[3]);
			checkTriangle(&check, vData[0], vData[2], vData[3]);
			checkTriangle(&check, vData[1], vData[2], vData[3]);

			checkTriangle(&check, vData[4], vData[5], vData[6]);
			checkTriangle(&check, vData[4], vData[5], vData[7]);
			checkTriangle(&check, vData[4], vData[6], vData[7]);
			checkTriangle(&check, vData[5], vData[6], vData[7]);

			checkTriangle(&check, vData[8], vData[9], vData[10]);
			checkTriangle(&check, vData[8], vData[9], vData[11]);
			checkTriangle(&check, vData[8], vData[10], vData[11]);
			checkTriangle(&check, vData[9], vData[10], vData[11]);
			
			for (int i = 0; i < 20; i++)
			{
				checkTriangle(&check,icoVData[icoTIndices[i][0]], icoVData[icoTIndices[i][1]], icoVData[icoTIndices[i][2]]);
			}

			if (check.foundCollision == true)
			{
				printf("COLLISION FOUND\n\n");
				if(velocity.x > 0)
					velocity.x = -maxSpeed + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (-minSpeed - -maxSpeed)));
				else
					velocity.x = minSpeed + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxSpeed - minSpeed)));
				if (velocity.y > 0)
					velocity.y = -maxSpeed + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (-minSpeed - -maxSpeed)));
				else
					velocity.y = minSpeed + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxSpeed - minSpeed)));
				if (velocity.z > 0)
					velocity.z = -maxSpeed + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (-minSpeed - -maxSpeed)));
				else
					velocity.z = minSpeed + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxSpeed - minSpeed)));
			}

			//drawEllipsoid(5, 5, ellipseBound.x, ellipseBound.y, ellipseBound.z,shapePosition.x,shapePosition.y,shapePosition.z);
			
			for (int i = 0; i < 530; i++)
			{
				((Teapot*)shapePointers[0])->vData[i][0] = ((Teapot*)shapePointers[0])->vData[i][0] + velocity.x;
				((Teapot*)shapePointers[0])->vData[i][1] = ((Teapot*)shapePointers[0])->vData[i][1] + velocity.y;
				((Teapot*)shapePointers[0])->vData[i][2] = ((Teapot*)shapePointers[0])->vData[i][2] + velocity.z;
			}
			for (int i = 0; (i < MAXSHAPES) && (shapePointers[i] != 0); i++)
			{
				if (((Teapot*)shapePointers[i])->vertexNum == 530)
					drawTeapot((Teapot*)shapePointers[i]);
				else if (((Diamond*)shapePointers[i])->vertexNum == 6)
					drawDiamond((Diamond*)shapePointers[i]);
			}


			shapePosition.x = shapePosition.x + velocity.x;
			shapePosition.y = shapePosition.y + velocity.y;
			shapePosition.z = shapePosition.z + velocity.z;

			glFlush();
		}
	}*/
	if (key == 'z')
	{
		if (wireframe == true)
		{
			wireframe = false;
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
		else
		{
			wireframe = true;
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		glutPostRedisplay();
	}
	else if (key == 'x')
	{
		if (displayOBB == true)
			displayOBB = false;
		else
			displayOBB = true;
		glutPostRedisplay();
	}
}

void rotateOBBTreeX(OBB* obb, double changeY, double changeZ, double theta)
{
	if (obb != NULL)
	{
		for (int i = 0; i < 8; i++)
		{
			obb->vData[i][1] = obb->vData[i][1] - changeY;
			obb->vData[i][2] = obb->vData[i][2] - changeZ;
			obb->vData[i][1] = obb->vData[i][1] * cos(theta)
				- obb->vData[i][2] * sin(theta);

			obb->vData[i][2] = obb->vData[i][1] * sin(theta)
				+ obb->vData[i][2] * cos(theta);
			obb->vData[i][1] = obb->vData[i][1] + changeY;
		}
		rotateOBBTreeX(obb->child1, changeY, changeZ, theta);
		rotateOBBTreeX(obb->child2, changeY, changeZ, theta);
	}
}

void rotateShapeX(double verts[][3], OBB* obb, int numVerts,double original[3], double theta)
{
	double changeY = verts[0][1] - original[1];
	double changeZ = verts[0][2] - original[2];
	for (int i = 0; i < numVerts; i++)
	{
		verts[i][1] = verts[i][1] - changeY;
		verts[i][2] = verts[i][2] - changeZ;
		verts[i][1] = verts[i][1] * cos(theta)
			- verts[i][2] * sin(theta);

		verts[i][2] = verts[i][1] * sin(theta)
			+ verts[i][2] * cos(theta);
		verts[i][1] = verts[i][1] + changeY;
	}
	rotateOBBTreeX(obb, changeY, changeZ, theta);
}

void rotateOBBTreeY(OBB* obb, double changeX, double changeZ, double theta)
{
	if (obb != NULL)
	{
		for (int i = 0; i < 8; i++)
		{
			obb->vData[i][0] = obb->vData[i][0] - changeX;
			obb->vData[i][2] = obb->vData[i][2] - changeZ;
			obb->vData[i][2] = obb->vData[i][2] * cos(theta)
				- obb->vData[i][0] * sin(theta);

			obb->vData[i][0] = obb->vData[i][2] * sin(theta)
				+ obb->vData[i][0] * cos(theta);
			obb->vData[i][0] = obb->vData[i][0] + changeX;
		}
		rotateOBBTreeY(obb->child1, changeX, changeZ, theta);
		rotateOBBTreeY(obb->child2, changeX, changeZ, theta);
	}
}

void rotateShapeY(double verts[][3], OBB* obb, int numVerts, double original[3], double theta)
{
	double changeX = verts[0][0] - original[0];
	double changeZ = verts[0][2] - original[2];
	for (int i = 0; i < numVerts; i++)
	{
		verts[i][0] = verts[i][0] - changeX;
		verts[i][2] = verts[i][2] - changeZ;
		verts[i][2] = verts[i][2] * cos(theta)
			- verts[i][0] * sin(theta);

		verts[i][0] = verts[i][2] * sin(theta)
			+ verts[i][0] * cos(theta);
		verts[i][0] = verts[i][0] + changeX;
	}
	rotateOBBTreeY(obb, changeX, changeZ, theta);
}

//This function handles arrow key inputs, which will couse the displayed object to rotate on the x and y axis
void onArrow(int key, int x, int y)
{
	if (objDragSetter != -1)
	{

		if (key == GLUT_KEY_RIGHT)
		{
			if (((Teapot*)shapePointers[objDragSetter])->vertexNum == 530)
			{
				rotateShapeX(((Teapot*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Teapot*)shapePointers[objDragSetter])->vertexNum,
					((Teapot*)shapePointers[objDragSetter])->original, DEGREE);
				glutPostRedisplay();
			}
			if (((Diamond*)shapePointers[objDragSetter])->vertexNum == 6)
			{
				rotateShapeX(((Diamond*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Diamond*)shapePointers[objDragSetter])->vertexNum,
					((Diamond*)shapePointers[objDragSetter])->original, DEGREE);
				glutPostRedisplay();
			}
			if (((Bunny*)shapePointers[objDragSetter])->vertexNum == 2503)
			{
				rotateShapeX(((Bunny*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Bunny*)shapePointers[objDragSetter])->vertexNum,
					((Bunny*)shapePointers[objDragSetter])->original, DEGREE);
				glutPostRedisplay();
			}
		}
		else if (key == GLUT_KEY_LEFT)
		{
			if (((Teapot*)shapePointers[objDragSetter])->vertexNum == 530)
			{
				rotateShapeX(((Teapot*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Teapot*)shapePointers[objDragSetter])->vertexNum,
					((Teapot*)shapePointers[objDragSetter])->original, -DEGREE);
				glutPostRedisplay();
			}
			if (((Diamond*)shapePointers[objDragSetter])->vertexNum == 6)
			{
				rotateShapeX(((Diamond*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Diamond*)shapePointers[objDragSetter])->vertexNum,
					((Diamond*)shapePointers[objDragSetter])->original, -DEGREE);
				glutPostRedisplay();
			}
			if (((Bunny*)shapePointers[objDragSetter])->vertexNum == 2503)
			{
				rotateShapeX(((Bunny*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Bunny*)shapePointers[objDragSetter])->vertexNum,
					((Bunny*)shapePointers[objDragSetter])->original, -DEGREE);
				glutPostRedisplay();
			}
		}
		else if (key == GLUT_KEY_UP)
		{
			if (((Teapot*)shapePointers[objDragSetter])->vertexNum == 530)
			{
				rotateShapeY(((Teapot*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Teapot*)shapePointers[objDragSetter])->vertexNum,
					((Teapot*)shapePointers[objDragSetter])->original, DEGREE);
				glutPostRedisplay();
			}
			if (((Diamond*)shapePointers[objDragSetter])->vertexNum == 6)
			{
				rotateShapeY(((Diamond*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Diamond*)shapePointers[objDragSetter])->vertexNum,
					((Diamond*)shapePointers[objDragSetter])->original, DEGREE);
				glutPostRedisplay();
			}
			if (((Bunny*)shapePointers[objDragSetter])->vertexNum == 2503)
			{
				rotateShapeY(((Bunny*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Bunny*)shapePointers[objDragSetter])->vertexNum,
					((Bunny*)shapePointers[objDragSetter])->original, DEGREE);
				glutPostRedisplay();
			}
		}
		else if (key == GLUT_KEY_DOWN)
		{
			if (((Teapot*)shapePointers[objDragSetter])->vertexNum == 530)
			{
				rotateShapeY(((Teapot*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Teapot*)shapePointers[objDragSetter])->vertexNum,
					((Teapot*)shapePointers[objDragSetter])->original, -DEGREE);
				glutPostRedisplay();
			}
			if (((Diamond*)shapePointers[objDragSetter])->vertexNum == 6)
			{
				rotateShapeY(((Diamond*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Diamond*)shapePointers[objDragSetter])->vertexNum,
					((Diamond*)shapePointers[objDragSetter])->original, -DEGREE);
				glutPostRedisplay();
			}
			if (((Bunny*)shapePointers[objDragSetter])->vertexNum == 2503)
			{
				rotateShapeY(((Bunny*)shapePointers[objDragSetter])->vData, ((OBB*)OBBPointers[objDragSetter]), ((Bunny*)shapePointers[objDragSetter])->vertexNum,
					((Bunny*)shapePointers[objDragSetter])->original, -DEGREE);
				glutPostRedisplay();
			}
		}
	}
}

Vector mouse(int x, int y)
{
	//return Vector(float(x) / winWidth, 1.0 - float(y) / winHeight, 0);
	GLint viewport[4]; //hold the viewport info
	GLdouble modelview[16]; //hold the modelview info
	GLdouble projection[16]; //hold the projection matrix info
	GLfloat winX, winY, winZ; //hold screen x,y,z coordinates
	GLdouble worldX, worldY, worldZ; //hold world x,y,z coordinates

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); //get the modelview info
	glGetDoublev(GL_PROJECTION_MATRIX, projection); //get the projection matrix info
	glGetIntegerv(GL_VIEWPORT, viewport); //get the viewport info

	winX = (float)x;
	winY = (float)viewport[3] - (float)y;

	//get the world coordinates from the screen coordinates
	gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &worldX, &worldY, &worldZ);
	gluUnProject(winX, winY, 1.0, modelview, projection, viewport, &worldX, &worldY, &worldZ);
	return Vector(worldX, worldY, worldZ);
}

// drag function
void drag(int x, int y)
{
	// int x and y of mouse converts to screen coordinates
	// returns the point as mousePt
	Vector mousePt = mouse(x, y);
	//create pointer to window point coordinates
	Vector* mouse = &mousePt;
	if (objDragSetter == -1)
	{
		for (int i = 0; (i < MAXSHAPES) && (shapePointers[i] != 0); i++)
		{
			if (((Teapot*)shapePointers[i])->vertexNum == 530)
			{
				float minX = ((Teapot*)shapePointers[i])->vData[0][0];
				float maxX = ((Teapot*)shapePointers[i])->vData[0][0];
				float minY = ((Teapot*)shapePointers[i])->vData[0][1];
				float maxY = ((Teapot*)shapePointers[i])->vData[0][1];
				for (int j = 0; j < 530; j++)
				{
					if (((Teapot*)shapePointers[i])->vData[j][0] < minX)
						minX = ((Teapot*)shapePointers[i])->vData[j][0];
					else if (((Teapot*)shapePointers[i])->vData[j][0] > maxX)
						maxX = ((Teapot*)shapePointers[i])->vData[j][0];
					if (((Teapot*)shapePointers[i])->vData[j][1] < minY)
						minY = ((Teapot*)shapePointers[i])->vData[j][1];
					else if (((Teapot*)shapePointers[i])->vData[j][1] > maxY)
						maxY = ((Teapot*)shapePointers[i])->vData[j][1];
				}
				if ((mouse->x >= minX) && (mouse->x <= maxX) && (mouse->y >= minY) && (mouse->y <= maxY))
				{
					objDragSetter = i;
					dragObj(shapePointers[i],(OBB*)OBBPointers[i], mouse, (OBB**)OBBPointers);
					glutPostRedisplay();
					break;
				}
			}
			else if (((Diamond*)shapePointers[i])->vertexNum == 6)
			{
				float minX = ((Diamond*)shapePointers[i])->vData[0][0];
				float maxX = ((Diamond*)shapePointers[i])->vData[0][0];
				float minY = ((Diamond*)shapePointers[i])->vData[0][1];
				float maxY = ((Diamond*)shapePointers[i])->vData[0][1];
				for (int j = 0; j < 6; j++)
				{
					if (((Diamond*)shapePointers[i])->vData[j][0] < minX)
						minX = ((Diamond*)shapePointers[i])->vData[j][0];
					else if (((Diamond*)shapePointers[i])->vData[j][0] > maxX)
						maxX = ((Diamond*)shapePointers[i])->vData[j][0];
					if (((Diamond*)shapePointers[i])->vData[j][1] < minY)
						minY = ((Diamond*)shapePointers[i])->vData[j][1];
					else if (((Diamond*)shapePointers[i])->vData[j][1] > maxY)
						maxY = ((Diamond*)shapePointers[i])->vData[j][1];
				}
				if ((mouse->x >= minX) && (mouse->x <= maxX) && (mouse->y >= minY) && (mouse->y <= maxY))
				{
					objDragSetter = i;
					dragObj(shapePointers[i], (OBB*)OBBPointers[i], mouse, (OBB**)OBBPointers);
					glutPostRedisplay();
					break;
				}
			}
			else if (((Bunny*)shapePointers[i])->vertexNum == 2503)
			{
				float minX = ((Bunny*)shapePointers[i])->vData[0][0];
				float maxX = ((Bunny*)shapePointers[i])->vData[0][0];
				float minY = ((Bunny*)shapePointers[i])->vData[0][1];
				float maxY = ((Bunny*)shapePointers[i])->vData[0][1];
				for (int j = 0; j < 6; j++)
				{
					if (((Bunny*)shapePointers[i])->vData[j][0] < minX)
						minX = ((Bunny*)shapePointers[i])->vData[j][0];
					else if (((Bunny*)shapePointers[i])->vData[j][0] > maxX)
						maxX = ((Bunny*)shapePointers[i])->vData[j][0];
					if (((Bunny*)shapePointers[i])->vData[j][1] < minY)
						minY = ((Bunny*)shapePointers[i])->vData[j][1];
					else if (((Bunny*)shapePointers[i])->vData[j][1] > maxY)
						maxY = ((Bunny*)shapePointers[i])->vData[j][1];
				}
				if ((mouse->x >= minX) && (mouse->x <= maxX) && (mouse->y >= minY) && (mouse->y <= maxY))
				{
					objDragSetter = i;
					dragObj(shapePointers[i], (OBB*)OBBPointers[i], mouse, (OBB**)OBBPointers);
					glutPostRedisplay();
					break;
				}
			}
		}
	}
	else
	{
		dragObj(shapePointers[objDragSetter], (OBB*)OBBPointers[objDragSetter],mouse, (OBB**)OBBPointers);
		glutPostRedisplay();
	}
}

void onMouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		//printf("%d		%d\n", x, y);
		//Vector coords = mouse(x, y);
		//printf("%f		%f\n", coords.x,coords.y);
		//printf("%f		%f\n", ((Teapot*)shapePointers[0])->vData[0][0], ((Teapot*)shapePointers[0])->vData[0][1]);
	}
	else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		objDragSetter = -1;
	}
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(winWidth, winHeight);
	glutCreateWindow("Final Project");

	init();
	glutDisplayFunc(display);
	glutKeyboardFunc(onKey);
	glutReshapeFunc(winReshapeFcn);
	glutMouseFunc(onMouse);
	glutSpecialFunc(onArrow);
	glutMotionFunc(drag);

	glutMainLoop();
	return 0;
}