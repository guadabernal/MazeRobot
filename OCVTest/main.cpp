#include <windows.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "gamepad.h"
#include "utils.h"
#include "SerialPort.h"
#include "opencv2/opencv.hpp"

using namespace cv;

SerialPort serial;

void setThrottle(int16_t *v) {
  serial.WriteData((char*)v, 4);
}

void setArray(int16_t *v, int16_t left, int16_t right) {
  v[0] = left; 
  v[1] = right;
}

void moveRobot(LARGE_INTEGER t0, LARGE_INTEGER frequency, const Mat &frame, std::ofstream &outfile) {
  LARGE_INTEGER t1;
  QueryPerformanceCounter(&t1);
  int16_t v[2] = { 0 };

  float dt = (t1.QuadPart - t0.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
  if (dt > 2000 && dt <= 2600) setArray(v, 240, 250); // forward + turn
  if (dt > 2600 && dt <= 3100) setArray(v, 228, 250); // forward + more turn
  if (dt > 3100 && dt <= 3600) setArray(v, 180, 250); // forward + ALOT more turn
  if (dt > 3600 && dt <= 4100) setArray(v, 180, 250); // forward + continue
  if (dt > 4100 && dt <= 4600) setArray(v, 140, 250); // forward + last grand turn
  if (dt > 4600 && dt <= 5100) setArray(v, 175, 200); // forward
  if (dt > 5100 && dt <= 5600) setArray(v, 160, 200); // forward
  if (dt > 5600 && dt <= 5900) setArray(v, 130, 200); // forward
  if (dt > 6100 && dt <= 6200) setArray(v, -70, -70); // stop 
  if (dt <= 2000 || dt > 6200) setArray(v, 0, 0);

  setThrottle(v);
  printf("dt=%f v0=%d v1=%d\n", dt, v[0], v[1]);

  if (dt > 2000 && dt < 6200) {
    outfile.write(reinterpret_cast<const char*>(&dt), sizeof(float));
    outfile.write(reinterpret_cast<const char*>(&v[0]), sizeof(int16_t));
    outfile.write(reinterpret_cast<const char*>(&v[1]), sizeof(int16_t));
    int size = frame.rows * frame.cols * frame.channels();
    outfile.write((char*)frame.data, size);
  }
}


void playFile(const std::string& Filename, Mat &frame) {
  std::ifstream infile("samples.nn", std::ifstream::binary);
  int rows, cols, channels;
  infile.read(reinterpret_cast<char*>(&rows), sizeof(int));
  infile.read(reinterpret_cast<char*>(&cols), sizeof(int));
  infile.read(reinterpret_cast<char*>(&channels), sizeof(int));
  int size = rows * cols * channels;
  
  printf("Rows = %d Cols = %d Channels = %d\n", rows, cols, channels);

  //Mat frame(rows, cols, CV_8UC3);
  namedWindow("Original", 1); 
  
  float t0 = 2000;
  while (!infile.eof()) {
    int16_t v[2];
    float t1;
    
    infile.read(reinterpret_cast<char*>(&t1), sizeof(float));
    infile.read(reinterpret_cast<char*>(&v[0]), sizeof(int16_t));
    infile.read(reinterpret_cast<char*>(&v[1]), sizeof(int16_t));

    printf("dt = %f t1=%f v0=%d v1=%d\n",t1 - t0, t1, v[0], v[1]);

    infile.read((char*)frame.data, size);
    imshow("Original", frame);
    Sleep(t1 - t0);
    t0 = t1;
  }

}


int main(int, char**)
{
  VideoCapture cap; // open the default camera
  const std::string videoStreamAddress = "http://10.0.0.5:8080/video?x.mjpeg";
  if (!cap.open(videoStreamAddress)) {
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }

  Mat frame;
  cap >> frame;

  playFile("samples.nn", frame);
  return 0;
#if 0
  std::ofstream outfile("samples.nn", std::ofstream::binary);

  serial.connect("\\\\.\\COM11");
  if (!serial.IsConnected()) {
    std::cout << "Error Connecting bluethoot" << std::endl;
    return -1;
  }
  Sleep(1000);
  
  VideoCapture cap; // open the default camera
  const std::string videoStreamAddress = "http://10.0.0.5:8080/video?x.mjpeg";
  if (!cap.open(videoStreamAddress)) {
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }

  Mat frame;
  cap >> frame;
  int cols = frame.cols;
  int rows = frame.rows;
  int channels = frame.channels();
  outfile.write(reinterpret_cast<const char*>(&cols), sizeof(int));
  outfile.write(reinterpret_cast<const char*>(&rows), sizeof(int));
  outfile.write(reinterpret_cast<const char*>(&channels), sizeof(int));

  for (int i = 0; i < 100; ++i) {
    cap >> frame;
  }

  namedWindow("Original", 1);

  LARGE_INTEGER frequency;
  LARGE_INTEGER t0;
  QueryPerformanceCounter(&t0);
  QueryPerformanceFrequency(&frequency);

  while (true) {
    cap >> frame;
    imshow("Original", frame);
    moveRobot(t0, frequency, frame, outfile);
    if (waitKey(10) >= 0) break;
  }


  outfile.close();
#endif
  return 0;
}



//  Gamepad gpad(0);
//gpad.update();
//float lx = scaleInput(gpad.getStickLeftX());
//float ly = scaleInput(gpad.getStickLeftY());
//float rx = scaleInput(gpad.getStickRightX());
//float ry = scaleInput(gpad.getStickRightY());
//printf("lx = %f ly =%f rx = %f ry = %f\n", lx, ly, rx, ry);
