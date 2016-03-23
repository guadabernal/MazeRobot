#ifndef GLPRIMITIVES_H
#define GLPRIMITIVES_H

#include <cmath>
#include <GL/freeglut.h>


//#include "common.h"

#define RADPERDEG 0.0174533

int NoColor = false;



void drawArrow(float x1, float y1, float z1, float x2, float y2,
	float z2, float D) {
	float x = x2 - x1;
	float y = y2 - y1;
	float z = z2 - z1;
	float L = sqrt(x * x + y * y + z * z);

	GLUquadricObj *quadObj;

	glPushMatrix();

	glTranslated(x1, y1, z1);

	if ((x != 0.) || (y != 0.)) {
		glRotated(atan2(y, x) / RADPERDEG, 0., 0., 1.);
		glRotated(atan2(sqrt(x * x + y * y), z) / RADPERDEG, 0., 1., 0.);
	}
	else if (z < 0) {
		glRotated(180, 1., 0., 0.);
	}

	glTranslatef(0, 0, L - 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, 2 * D, 0.0, 4 * D, 32, 4);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, 2 * D, 32, 4);
	gluDeleteQuadric(quadObj);

	glTranslatef(0, 0, -L + 4 * D);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, D, D, L - 4 * D, 32, 4);
	gluDeleteQuadric(quadObj);

	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, D, 32, 4);
	gluDeleteQuadric(quadObj);

	glPopMatrix();
}

void drawCylinder(float h, float r) {
	glPushMatrix();
	GLUquadricObj *quadObj;
	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluCylinder(quadObj, r, r, h, 32, 4);
	gluDeleteQuadric(quadObj);
	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, r, 32, 4);
	gluDeleteQuadric(quadObj);
	glTranslatef(0, 0, h);
	quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);
	gluDisk(quadObj, 0.0, r, 32, 4);
	gluDeleteQuadric(quadObj);
	glPopMatrix();
}


void drawSphere(float r, float slices , float stacks) {
	GLUquadricObj *quadObj;
	quadObj = gluNewQuadric();
	gluSphere(quadObj, r, slices, stacks);
	gluDeleteQuadric(quadObj);
}


void draw_axes_arrow(float width, float height, float depth, float r)
{
	glColor3f(1, 0, 0);
	drawArrow(-width / 2, 0, 0, width / 2, 0, 0, r);
	glColor3f(0, 1, 0);
	drawArrow(0, -height / 2, 0, 0, height / 2, 0, r);
	glColor3f(0, 0, 1);
	drawArrow(0, 0, -depth / 2, 0, 0, depth / 2, r);
}


void draw_axes(float width, float height, float depth)
{
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(-width / 2, 0, 0);
	glVertex3f(width / 2, 0, 0);
	glEnd();
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, -height / 2, 0);
	glVertex3f(0, height / 2, 0);
	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, -depth / 2);
	glVertex3f(0, 0, depth / 2);
	glEnd();
}


void draw_axes_positive(float width, float height, float depth)
{
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(width, 0, 0);
	glEnd();
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, height, 0);
	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, depth);
	glEnd();
}

void drawCube(float dx, float dy, float dz) {
	glPushMatrix();
	glTranslatef(-dx / 2, -dy / 2, -dz / 2);

	glBegin(GL_QUADS);
	// back face
	glNormal3f(0, 0, -1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, dy, 0);
	glVertex3f(dx, dy, 0);
	glVertex3f(dx, 0, 0);

	// flront face
	glNormal3f(0, 0, 1);
	glVertex3f(0, 0, dz);
	glVertex3f(0, dy, dz);
	glVertex3f(dx, dy, dz);
	glVertex3f(dx, 0, dz);

	// bottom face  (z = 1.0f)
	glNormal3f(0, -1, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, dz);
	glVertex3f(dx, 0, dz);
	glVertex3f(dx, 0, 0);

	// top face (z = -1.0f)
	glNormal3f(0, 1, 0);
	glVertex3f(0, dy, 0);
	glVertex3f(0, dy, dz);
	glVertex3f(dx, dy, dz);
	glVertex3f(dx, dy, 0);

	// Left face (x = -1.0f)
	glNormal3f(-1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, dz);
	glVertex3f(0, dy, dz);
	glVertex3f(0, dy, 0);

	// // Right face (x = 1.0f)
	glNormal3f(1, 0, 0);
	glVertex3f(dx, 0, 0);
	glVertex3f(dx, 0, dz);
	glVertex3f(dx, dy, dz);
	glVertex3f(dx, dy, 0);

	glEnd();  // End of drawing color-cube
	glPopMatrix();
}

#endif  // GLPRIMITIVES_H
