#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <GL/freeglut.h>
#include "glprimitives.h"

#define _USE_MATH_DEFINES
#include <math.h>


using namespace std;

class Robot {
public:
	Robot(float robotR, float robotH, float wheelR, float wheelW, float x = 0, float y = 0, float theta = 0)
		: robotR(robotR), robotH(robotH), wheelR(wheelR), wheelW(wheelW), x(x), y(y), theta(theta), VR(0), VL(0), maxMotorRev(1.5) {
		T = 0;
		Tvel = 0;
		setPhoneCoordinates(); 
	}

	void setPhoneCoordinates() {
		phex = 0;
		phey = 0;
		phez = 15;
		phcx = 0;
		phcy = -25;
		phcz = 0;
		phfov = 52;
	}

	void setVelocity(float tl, float tr) {
		Tvel = T;
		VL = tl/255 * maxMotorRev * 2 * M_PI * wheelR;
		VR = tr/255 * maxMotorRev * 2 * M_PI * wheelR;
	}

	void update(float dt) {
		if (VR != VL) { // case where it rotates on a finite circle
			float xr = x, yr = y;	//position of robot
			float R = robotR * (VR + VL) / (VL - VR);	//distance from robot to Icc
			float Iccx = xr - R * cos(theta);	//Icc x : center of rotation
			float Iccy = yr + R * sin(theta);	//Icc y
			float w = VR / (R + robotR);  //omega angular velocity
			float dtheta = w * dt;  //angle change
			x = (cos(dtheta) * (xr - Iccx) - sin(dtheta) * (yr - Iccy)) + Iccx;
			y = (sin(dtheta) * (xr - Iccx) + cos(dtheta) * (yr - Iccy)) + Iccy;
			theta = theta + dtheta;	//theta prime
		} else { // case where it goes straight
			float d = VL * dt;
			x = x + d * sin(theta);
			y = y + d * cos(theta);
		}
	}

	void simulate(float dt) {
		T = T + dt;
		std::cout << "T = " << T << std::endl;
		if (T - Tvel > 0.5) {
			setVelocity(0, 0);
		}
		update(dt);
	}

	void printStatus() {
		std::cout << "robotR = " << robotR << " robotH = " << robotH << " wheelR = " << wheelR <<std::endl;
		std::cout << "x = " << x << "  y = " << y << "  theta = " << theta <<std::endl;
	}
	
	void renderWheel(float wx, float wy, float wz) {
		glPushMatrix();
		glTranslatef(wx - wheelW / 2, wy, wz);
		glRotatef(90, 0, 1, 0);
		glColor3f(0, 0 ,0);
		drawCylinder(wheelW, wheelR);
		glColor3f(0.8, 0.8, 0.7);
		glTranslatef(0, 0, -0.1);
		drawCylinder(wheelW+0.2, wheelR-1);
		glPopMatrix();
	}

	void renderBase(float wx, float wy, float wz) {
		glPushMatrix();
		glTranslatef(wx, wy, wz);
		glColor3f(0, 0.5, 0.6);
		drawCylinder(robotH, robotR - 1);
		glColor3f(0.8, 0.2, 0.0);
		glTranslatef(0, 0, -0.1);
		drawCylinder(robotH + 0.2, robotR - 3);
		glTranslatef(0, -(robotR - 2), -0.1);
		glColor3f(0.0, 0.8, 0.0);
		drawCylinder(robotH + 0.4, 0.5);
		glPopMatrix();
	}

	void renderPhone() {
		glPushMatrix();
		glTranslatef(phex, phey, phez);
		glPushMatrix();
		glColor3f(0.4, 0, 0.8);
		glTranslatef(0, 3, -6);
		glRotatef(25, 1, 0, 0);
		drawCube(7, 0.8, 15);
		glPopMatrix();
		glColor3f(0.8, 1, 0);
		drawSphere(1, 10, 10);
		glPopMatrix();
		glPushMatrix();
		
	}
	void renderCameraFrustrum() {
		glColor3f(0, 0, 0);
		glPushMatrix();
		float dx = phex - phcx;
		float dy = phey - phcy;
		float dz = phez - phcz;
		float d = sqrt(dx * dx + dy * dy + dz * dz);
		float h = tan(phfov / 180.0f * M_PI / 2) * d *480.0f/640.0f;
		float theta = atan2(dy, dz);
		float k1 = phez * (tan(theta) - tan(theta - phfov / 180.0f * M_PI / 2));
		float k2 = -phez * (tan(theta + phfov / 180.0f * M_PI / 2) - tan(theta));

		glTranslatef(0, 0, 0.3);
		glBegin(GL_LINES);
		glVertex3f(phex, phey, phez);
		glVertex3f(phcx, phcy, phcz);
		glEnd();
		glColor4f(0.8, 0.8, 0.8, 0.9);
		glBegin(GL_QUADS);
		glVertex3f(phcx - h, phcy + k2, phcz);
		glVertex3f(phcx - h, phcy + k1, phcz);
		glVertex3f(phcx + h, phcy + k1, phcz);
		glVertex3f(phcx + h, phcy + k2, phcz);
		glEnd();

		glPopMatrix();
	}

	void render(bool phone = true) {
		glPushMatrix();
		glTranslatef(-x, 0, y);
		glRotatef(-90, 0, 1, 0);
		glRotatef(-90, 1, 0, 0);
		//renderCameraFrustrum();
		glRotatef(90 - theta * 180.0f / M_PI, 0, 0, 1);
		glTranslatef(0, 0, wheelR);
		
		renderWheel(robotR, 0, 0);
		renderWheel(-robotR, 0, 0);
		renderBase(0, 0, 0);
		if (phone)
		  renderPhone();
		glPopMatrix();
	}

	float getAngle() { return theta; }
	float getX() { return x; }
	float getY() { return y; }

private:	
	const float robotR;
	const float robotH;
	const float wheelR;
	const float wheelW;
	const float maxMotorRev;
	float x;
	float y;
	float theta;
	float Tvel;
	float T;

	float phex, phey, phez; // phone eye x, y, z
	float phcx, phcy, phcz; // phone target center x, y, z
	float phfov;			// phone phfov

	float VR, VL;
};

#endif