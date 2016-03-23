#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <GL/freeglut.h>
#include "robot.h"
#include "glprimitives.h"
#include "viewports.h"
#include "gridpath.h"

const size_t WindowsWidth = 480*2;
const size_t WindowsHeight = 640;

Robot dragon(10, 2, 4.5, 0.8);
GridPath grid(200, 200);

ViewPort3D viewLeftTop(0, WindowsHeight / 2 + 1, WindowsWidth / 2 - 2, WindowsHeight / 2, 60, dragon, grid);
ViewPort2D viewLeftBottom(0, 0, WindowsWidth / 2 - 2, WindowsHeight / 2 - 1, WindowsWidth/2 , WindowsHeight/2, dragon, grid);
ViewPortFix3D viewRight(WindowsWidth / 2, 0, 480, 640, 52, dragon, grid);

void display() {
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	viewLeftTop.render();
	viewLeftBottom.render();
	viewRight.render();
	dragon.simulate(0.03);
	glutSwapBuffers();
}

void init_display() {
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glLineWidth(1.5);
}

void reshape(int w, int h) {
	init_display();
	display();
}

void mouse_wheel(int wheel, int direction, int x, int y) {
	int y1 = WindowsHeight - y;
	viewLeftTop.mouse_button(direction > 0 ? 3 : 4, GLUT_DOWN, x, y1);
	viewLeftBottom.mouse_button(direction > 0 ? 3 : 4, GLUT_DOWN, x, y1);
}

void mouse_button(int button, int state, int x, int y) {
	int y1 = WindowsHeight - y;
	viewLeftTop.mouse_button(button, state, x, y1);
	viewLeftBottom.mouse_button(button, state, x, y1);
}

void mouse_active_motion(int x, int y) { 
	int y1 = WindowsHeight - y;
	viewLeftTop.mouse_active_motion(x, y1); 
	viewLeftBottom.mouse_active_motion(x, y1);
}

void mouse_passive_motion(int x, int y) {
	int y1 = WindowsHeight - y;
	viewLeftTop.mouse_passive_motion(x, y1); 
	viewLeftBottom.mouse_passive_motion(x, y1);
}

void normal_keys(unsigned char key, int x, int y) {
	switch (key) {
	case 'a':
		dragon.setVelocity(150, -150);
		break;
	case 'w':
		dragon.setVelocity(150, 150);
		break;
	case 'd':
		dragon.setVelocity(-150, 150);
		break;
	case 'b':
		dragon.simulate(0.01);
		break;
	case 27:
		glutLeaveMainLoop();
		break;
	default:
		break;
	}
}

void init_glut_window(int argc, char *argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(WindowsWidth, WindowsHeight);
	glutCreateWindow("Robot Sim");

	glutDisplayFunc(display);
	glutIdleFunc(display);
	glutKeyboardFunc(normal_keys);
	//glutSpecialFunc(special_keys);
	glutMouseFunc(mouse_button);
	glutMotionFunc(mouse_active_motion);
	glutPassiveMotionFunc(mouse_passive_motion);
	glutMouseWheelFunc(mouse_wheel);
	glutReshapeFunc(reshape);

	glutMainLoop();
}

int main(int argc, char *argv[]) {
	init_glut_window(argc, argv);
	

  //using namespace cv;
  //cv::VideoCapture cap(0);

  //while (true) {
  //  Mat frame;
  //  cap >> frame;
  //  imshow("Frame", frame);
  //  if(cv::waitKey(30) > 0) break;
  //}
}
