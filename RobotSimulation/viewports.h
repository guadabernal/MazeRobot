#ifndef VIEWPORTS_H
#define VIEWPORTS_H

#include <GL/freeglut.h>
#include "robot.h"
#include "gridpath.h"
#include "svector.h"

class ViewPort3D {
public:
	ViewPort3D(float x, float y, float w, float h, float fov, Robot& r, GridPath &grid) 
		: x(x), y(y), w(w), h(h), fov(fov), robot(r), grid(grid) {
		qCamera(0, 0, 0, 1);
		mouse_vx = -5;
		mouse_vy = 20;
		mouse_vz = -30;
		//svector::float4 qNew(0, 0, 0, 1);
		//qNew.euler(3.1415 / 2, 0, 0);
		//qCamera.quaternion_mult(qNew);
	}

	void render() {
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glViewport(x, y, w, h);
		gluPerspective(fov, w / h, 1, 2000);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glPushMatrix();

		glTranslatef(mouse_vx, -mouse_vy, mouse_vz);
		float pi = 3.14159265358;
		svector::float4 qNew(0, 0, 0, 1);
		qNew.euler(mouse_dy / 320.0f, mouse_dx / 320.0f, 0.0);
		qCamera.quaternion_mult(qNew);
		svector::float4 qAxis = qCamera.axis();
		glRotatef(qAxis.w / pi * 180, qAxis.x, qAxis.y, qAxis.z);

		glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
		draw_axes_positive(50, 50, 50);
		robot.render();
		grid.render(200, 200);

		glPopMatrix();
	}
	void mouse_button(int button, int status, int xm, int ym) {
		if (xm < x || xm > x + w || ym < y || ym > y + h) return;
		left_button_status = GLUT_UP;
		right_button_status = GLUT_UP;
		if ((button == 3) || (button == 4)) {
			if (status == GLUT_DOWN) {
				mouse_vz += button == 3 ? 1.9 : -1.9;
				if (mouse_vz > 40) mouse_vz = 30;
				if (mouse_vz < -200) mouse_vz = -200;
			}
		}
		else {
			if (button == GLUT_LEFT_BUTTON) {
				if (status == GLUT_DOWN) {
					left_button_status = GLUT_DOWN;
					left_button_down_x = xm;
					left_button_down_y = ym;
				}
			}
			if (button == GLUT_RIGHT_BUTTON) {
				if (status == GLUT_DOWN) {
					right_button_status = GLUT_DOWN;
					left_button_down_x = xm;
					left_button_down_y = ym;
				}
			}
		}
	}

	void mouse_active_motion(int xm, int ym) {
		if (xm < x || xm > x + w || ym < y || ym > y + h) return;
		if (left_button_status == GLUT_DOWN) {
			mouse_dx = (xm - left_button_down_x);
			mouse_dy = -(ym - left_button_down_y);
			if (fabs(mouse_dx) < 2) mouse_dx = 0;
			if (fabs(mouse_dy) < 2) mouse_dy = 0;
		}
		if (right_button_status == GLUT_DOWN) {
			float dx = (xm - left_button_down_x) / float(50);
			float dy = -(ym - left_button_down_y) / float(50);
			mouse_vy += dy;
			mouse_vx += dx;
		}

		left_button_down_x = xm;
		left_button_down_y = ym;
	}

	void mouse_passive_motion(int xm, int ym) {
		if (xm < x || xm > x + w || ym < y || ym > y + h) return;
		left_button_down_y = ym;
		left_button_down_x = xm;
	}
private:
	float x, y, w, h;
	float fov;
	Robot &robot;
	GridPath &grid;

	svector::float4 qCamera;
	int left_button_status;
	int right_button_status;
	int left_button_down_x;
	int left_button_down_y;
	float mouse_dx;
	float mouse_dy;
	float mouse_vx;
	float mouse_vy;
	float mouse_vz;
};

class ViewPortFix3D {
public:
	ViewPortFix3D(float x, float y, float w, float h, float fov, Robot& r, GridPath &grid)
		: x(x), y(y), w(w), h(h), fov(fov), robot(r), grid(grid) {
	}

	void render() {		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glViewport(x, y, w, h);
		gluPerspective(fov, float(w) / h, 0.001f, 1000);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glPushMatrix();
		float R = 35;
		float ex = -robot.getX();
		float ey = 21;
		float ez = robot.getY();
		float cx = ex + cos(3.1415/2 + robot.getAngle()) * R;
		float cy = 0;
		float cz = robot.getY() + sin(3.1415/2 + robot.getAngle()) * R;
		gluLookAt(ex, ey, ez, cx, cy, cz, 0.0, 1.0, 0.0);
		robot.render(false);
		grid.render(200, 200);

		glPopMatrix();
	}
private:
	float x, y, w, h;
	float fov;
	Robot &robot;
	GridPath &grid;
};

class ViewPort2D {
public:
	ViewPort2D(float x, float y, float w, float h, float WW, float WH, Robot& r, GridPath& grid)
		: x(x), y(y), w(w), h(h), WW(WW), WH(WH), robot(r), grid(grid) {
		mouse_vx = 0;
		mouse_vy = 0;
		mouse_vz = 100;
	}

	void render() {
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glViewport(x, y, w, h);
		glRotatef(-90, 1, 0, 0);
		float k = h / w;
		glOrtho(-mouse_vz, mouse_vz, -mouse_vz, mouse_vz, -mouse_vz*k, mouse_vz * k);
		glTranslatef(mouse_vx, 0, mouse_vy);
		glRotatef(180, 0, 1, 0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glPushMatrix();

		glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
		draw_axes_positive(50, 50, 50);
		robot.render();
		grid.render(200, 200);
		glPopMatrix();
	}

	void mouse_button(int button, int status, int xm, int ym) {
		if (xm < x || xm > x + w || ym < y || ym > y + h) return;
		left_button_status = GLUT_UP;
		right_button_status = GLUT_UP;
		if ((button == 3) || (button == 4)) {
			if (status == GLUT_DOWN) {
				mouse_vz += button == 3 ? 3 : -3;
				if (mouse_vz < 10) mouse_vz = 10;
				if (mouse_vz > 200) mouse_vz = 200;
			}
		}
		else {
			if (button == GLUT_LEFT_BUTTON) {
				if (status == GLUT_DOWN) {
					left_button_status = GLUT_DOWN;
					left_button_down_x = xm;
					left_button_down_y = ym;
				}
			}
		}
	}
	void mouse_active_motion(int xm, int ym) {
		if (xm < x || xm > x + w || ym < y || ym > y + h) return;
		if (left_button_status == GLUT_DOWN) {
			mouse_dx = (xm - left_button_down_x);
			mouse_dy = -(ym - left_button_down_y);
			mouse_vx += mouse_dx / WW * 2 * mouse_vz;
			mouse_vy += mouse_dy / WH * 2 * mouse_vz;
		}

		left_button_down_x = xm;
		left_button_down_y = ym;
	}

	void mouse_passive_motion(int xm, int ym) {
		if (xm < x || xm > x + w || ym < y || ym > y + h) return;
		left_button_down_y = ym;
		left_button_down_x = xm;
	}
private:
	float x, y, w, h;
	Robot &robot;
	GridPath &grid;

	int left_button_status;
	int right_button_status;
	int left_button_down_x;
	int left_button_down_y;
	float mouse_dx;
	float mouse_dy;
	float mouse_vx;
	float mouse_vy;
	float mouse_vz;
	float WW, WH;
};



#endif // VIEWPORTS_H
