//#include <windows.h>
//#include <stdint.h>
//
//#include <iostream>
//#include "gamepad.h"
//#include "utils.h"
//#include "SerialPort.h"
//
//
//
//int main() {
//  Gamepad gpad(0);
//  SerialPort serial;
//  Sleep(1000);
//  serial.connect("\\\\.\\COM11");
//
//  while (true) {
//    gpad.update();
//    float lx = scaleInput(gpad.getStickLeftX());
//    float ly = scaleInput(gpad.getStickLeftY());
//    float rx = scaleInput(gpad.getStickRightX());
//    float ry = scaleInput(gpad.getStickRightY());
//    printf("lx = %f ly =%f rx = %f ry = %f\n", lx, ly, rx, ry);
//    int16_t v[2];
//    v[0] = ly * 255;
//    v[1] = ry * 255;
//    int ret = serial.WriteData((char*)v, 4);
//    Sleep(20);
//  }
//
//}

#include <windows.h>
#include <stdint.h>

#include <iostream>
#include "gamepad.h"
#include "utils.h"
#include "SerialPort.h"
#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    Gamepad gpad(0);
    SerialPort serial;
    Sleep(1000);
    serial.connect("\\\\.\\COM11");

  VideoCapture cap; // open the default camera
  const std::string videoStreamAddress = "http://10.0.0.5:8080/video?x.mjpeg";
  if (!cap.open(videoStreamAddress)) {
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }

  Mat edges;
  namedWindow("edges", 1);
  namedWindow("Original", 1);
  for (;;)
  {
    Mat frame;
    cap >> frame; // get a new frame from camera
    cv::flip(frame, frame, 1);
    cvtColor(frame, edges, CV_BGR2GRAY);
    //cv::transpose(edges, edges);
    
    GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
    Canny(edges, edges, 0, 30, 3);
    imshow("edges", edges);
    imshow("Original", frame);
        gpad.update();
        float lx = scaleInput(gpad.getStickLeftX());
        float ly = scaleInput(gpad.getStickLeftY());
        float rx = scaleInput(gpad.getStickRightX());
        float ry = scaleInput(gpad.getStickRightY());
        printf("lx = %f ly =%f rx = %f ry = %f\n", lx, ly, rx, ry);
        int16_t v[2];
        v[0] = ly * 128;
        v[1] = ry * 128;
        int ret = serial.WriteData((char*)v, 4);
    if (waitKey(10) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}