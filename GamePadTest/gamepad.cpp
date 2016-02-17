#define _USE_MATH_DEFINES
#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>
#include "include/glheaders.h"
#include "include/glprimitives.h"
#include "gl2dgamepad.h"
#include "gamepad.h"
#include "SerialPort.h"

namespace {
  int WinWidth = 1100;
  int WinHeight = 1100;
  int ViewWidth = 100;
  float WinRatio = WinHeight / float(WinWidth);
  int ViewHeight = ViewWidth * WinRatio;
  std::string WinTitle = "GamePad";
  Gamepad gpad(0);
  SerialPort serial;
}


float px0 = -10, py0 = 0;
float px1 = 10, py1 = 0;
void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  set2DMode(ViewWidth, ViewHeight);
 
  gpad.update();
  float lx = scaleInput(gpad.getStickLeftX());
  float ly = scaleInput(gpad.getStickLeftY());
  float rx = scaleInput(gpad.getStickRightX());
  float ry = scaleInput(gpad.getStickRightY());

  glColor3f(1, 0, 0);
  printText(10, 10, format("lx = %f - ly = %f", lx, ly));
  glColor3f(0, 1, 0);
  printText(10, 15, format("lx = %f - ly = %f", rx, ry));

  px0 += lx;
  py0 -= ly;

  px1 += rx;
  py1 -= ry;

  glTranslatef(50, 50, 0);


  if (gpad.leftButtonPress())
    glColor3f(0, 0, 1);
  else 
    glColor3f(1, 0, 0);
  drawDisk(px0, py0, 5, 100);
  glColor3f(1, 0, 0);
  drawDisk(px1, py1, 5, 100);

  char v[2];
  v[0] = -ly * 100;
  v[1] = -ry * 100;
  serial.WriteData(v, 2);

  



  glutSwapBuffers();
}

void init_display() {
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glLineWidth(1.5);
}


void reshape(int w, int h) {
  WinWidth = w;
  WinHeight = h;
  WinRatio = WinHeight / float(WinWidth);
  ViewHeight = ViewWidth * WinRatio;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  init_display();
  display();
}

void mouse_wheel(int wheel, int direction, int x, int y) {
//  glGamepad.mouse_wheel(wheel, direction, x, y);
}

void mouse_button(int button, int status, int x, int y) {
//  glGamepad.mouse_button(button, status, x, y);
}

void mouse_active_motion(int x, int y) {// glGamepad.mouse_active_motion(x, y); 
}

void mouse_passive_motion(int x, int y) { //glGamepad.mouse_passive_motion(x, y); 
}

void special_keys(int key, int x, int y) {
  switch (key) {
  case GLUT_KEY_UP:
    break;
  case GLUT_KEY_DOWN:
    break;
  case GLUT_KEY_RIGHT:
    break;
  case GLUT_KEY_LEFT:
    break;
  case GLUT_KEY_F10:
    glutPostRedisplay();
    break;
  default:
    break;
  }
}

void normal_keys(unsigned char key, int x, int y) {
  switch (key) {
  case 'a':
    serial.connect("\\\\.\\COM11");
    break;
  case '1': {
    char v[2] = { 100, 100 };
    serial.WriteData(v, 2);
    break;
  }
  case '2': {
    char v[2] = { 0, 0 };
    serial.WriteData(v, 2);
    break;
  }
  case 't':
    break;
  case 32: {
    glutPostRedisplay();
    break;
  }
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
  glutInitWindowSize(WinWidth, WinHeight);
  glutCreateWindow(WinTitle.c_str());

  glutDisplayFunc(display);
  glutIdleFunc(display);
  glutReshapeFunc(reshape);

  glutKeyboardFunc(normal_keys);
  glutSpecialFunc(special_keys);

  glutMouseFunc(mouse_button);
  glutMotionFunc(mouse_active_motion);
  glutPassiveMotionFunc(mouse_passive_motion);
#ifdef WIN32
  glutMouseWheelFunc(mouse_wheel);
#endif

  glutMainLoop();
}

int main(int argc, char **argv) {
  init_glut_window(argc, argv);

  return 0;
}