#include <windows.h>
#include "gamepad.h"
#include <stdint.h>
#include "utils.h"
#include "SerialPort.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <fstream>

SerialPort serial;

bool getMassCenter(cv::Mat& mat, cv::Rect& r, cv::Point2f &p) {
  cv::Moments m = cv::moments(mat(r));
  if (m.m00 == 0 || m.m00 == 0) return false;
  p = cv::Point2f(r.x + m.m10 / m.m00, r.y + m.m01 / m.m00);
  return true;
}

void setThrottle(int16_t l, int16_t r) {
  int16_t v[2] = { l, r };
  serial.WriteData((char*)v, 4);
}


int main() {
  using namespace cv;
  cv::VideoCapture cap; // open camera
  if (!cap.open("http://192.168.0.198:8080/video?x.mjpeg")) {
    cap.release();
    std::cout << "Error opening video stream or file" << std::endl;
    exit(-1);
  }

  serial.connect("\\\\.\\COM11");
  if (!serial.IsConnected()) {
    std::cout << "Error Connecting bluethoot" << std::endl;
    return -1;
  }

  int iLowH = 85;
  int iHighH = 132;
  int iLowS = 23;
  int iHighS = 255;
  int iLowV = 0;
  int iHighV = 255;
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);
  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  Gamepad gpad(0);

  Mat frame;
  for (int i = 0; i < 300; ++i)
    cap.read(frame);

  bool isDone = false;
  bool gamePadEnable = true;
  bool isRotating = true;
  bool isMazeSolver = false;

  LARGE_INTEGER t0, t1, t2, trot0, trot1;
  LARGE_INTEGER frequency;
  QueryPerformanceCounter(&t0);
  QueryPerformanceFrequency(&frequency);

  float P0 = 0;
  float I = 0;


  while (!isDone) {
    t2 = t1;
    QueryPerformanceCounter(&t0);
    if (!cap.read(frame)) {
      std::cout << "Error Reading camera\n";
      exit(-1);
    }
    QueryPerformanceCounter(&t1);
    float dtcam = (t1.QuadPart - t0.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
    float dtpro = (t0.QuadPart - t2.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
    if (dtcam > 80 || dtpro > 80)
      printf("dtcam = %f  dtpro = %f\n", dtcam, dtpro);

    resize(frame, frame, Size(480, 640));
    Mat hsv_image;
    cvtColor(frame, hsv_image, COLOR_BGR2HSV); // conver to HSV
    Mat blue_range;
    inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), blue_range);

    GaussianBlur(blue_range, blue_range, Size(9, 9), 2, 2);
    threshold(blue_range, blue_range, 100, 255, THRESH_BINARY);
    erode(blue_range, blue_range, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)), Point(-1, -1), 1);


    gpad.update();
    if (gpad.rightButtonPress()) gamePadEnable = !gamePadEnable;
    if (gpad.leftButtonPress()) isMazeSolver = !isMazeSolver;
    if (gamePadEnable) {
      float lx = scaleInput(gpad.getStickLeftX());
      float ly = scaleInput(gpad.getStickLeftY());
      float rx = scaleInput(gpad.getStickRightX());
      float ry = scaleInput(gpad.getStickRightY());
      setThrottle(ly * 200, ry * 200);
    }

    Point2f p;

    int iband = 3; //3
    bool found = getMassCenter(blue_range, Rect(0, blue_range.rows - 50 * (iband + 1), blue_range.cols, 50), p);
    line(frame, Point(0, frame.rows - 50 * iband), Point(frame.cols, frame.rows - 50 * iband), Scalar(0, 0, 0), 1);
    line(frame, Point(0, frame.rows - 50 * (iband + 1)), Point(frame.cols, frame.rows - 50 * (iband + 1)), Scalar(0, 0, 0), 1);
    if (found)
      circle(frame, p, 9, Scalar(0, 0, 255), 3);
    if (isMazeSolver) {
      int iband_next = iband + 1;
      line(frame, Point(0, frame.rows - 50 * (iband_next + 1)), Point(frame.cols, frame.rows - 50 * (iband_next + 1)), Scalar(0, 0, 0), 1);
      Point2f pl, pc, pr;
      bool found_next_left = getMassCenter(blue_range, Rect(0, blue_range.rows - 50 * (iband_next + 1), blue_range.cols / 3.0f, 50), pl);
      bool found_next_center = getMassCenter(blue_range, Rect(blue_range.cols / 3.0f, blue_range.rows - 50 * (iband_next + 1),
        blue_range.cols / 3.0f, 50), pc);
      bool found_next_right = getMassCenter(blue_range, Rect(2 * blue_range.cols / 3.0f, blue_range.rows - 50 * (iband_next + 1),
        blue_range.cols / 3.0f, 50), pr);

      if (found_next_left)   circle(frame, pl, 9, Scalar(0, 0, 255), 3);
      if (found_next_center)   circle(frame, pc, 9, Scalar(0, 0, 255), 3);
      if (found_next_right)   circle(frame, pr, 9, Scalar(0, 0, 255), 3);
      if (found_next_right && pr.x - p.x > 60) {
        found = false;
        circle(frame, pr, 9, Scalar(0, 255, 0), 3);
      }
    }

    if (found && !gamePadEnable && !isRotating) {
      float ly = 1;
      float ry = 1;
      float P = p.x - blue_range.cols / 2.0f;
      float D = P - P0;
      I += P;
      P0 = P;

      float kp = 1 / 3.0f; // 1/3
      float ki = 1 / 10000.0f * 1;
      float kd = 3 / 2.0f;
      int pd = kp * P + ki * I + kd * D;

      const int max = 120; //120
      pd = pd > max ? max : pd < -max ? -max : pd;
      ly = pd < 0 ? max + pd : max;
      ry = pd > 0 ? max - pd : max;
      setThrottle(ly, ry);

      std::cout << "-------------- " << ly << " " << ry << "  kp = " << kp << " " << P << std::endl;
    }
    if (isMazeSolver) {
      if (!found && !gamePadEnable && !isRotating) {
        QueryPerformanceCounter(&trot0);
        isRotating = true;
      }
      if (isRotating) {
        QueryPerformanceCounter(&trot1);
        float dtrot = (trot1.QuadPart - trot0.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
        if (100 < dtrot && dtrot < 500)
          setThrottle(100, -100);
        else if (dtrot < 600)
          setThrottle(0, 0);
        else
          isRotating = false;
      }
    }
    cv::imshow("RobotView", blue_range);
    cv::imshow("Camera", frame);

    int key = cv::waitKey(30);
    switch (key) {
    case 'm':
      isMazeSolver = !isMazeSolver;
      std::cout << "Maze Solver enabled\n";
      break;
    case 'g':
      gamePadEnable = !gamePadEnable;
      std::cout << "GamePad enabled\n";
      break;
    case 27:
      isDone = true;
      break;
    }
  }
}


int main1() {
  using namespace cv;
  cv::VideoCapture cap; // open camera
  if (!cap.open("http://192.168.0.198:8080/video?x.mjpeg")) {
    cap.release();
    std::cout << "Error opening video stream or file" << std::endl;
    exit(-1);
  }

  serial.connect("\\\\.\\COM11");
  if (!serial.IsConnected()) {
    std::cout << "Error Connecting bluethoot" << std::endl;
    return -1;
  }

  int iLowH = 85;
  int iHighH = 132;
  int iLowS = 23;
  int iHighS = 255;
  int iLowV = 0;
  int iHighV = 255;
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);
  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  Gamepad gpad(0);

  Mat frame;
  for (int i = 0; i < 300; ++i)
    cap.read(frame);

  bool isDone = false;
  bool gamePadEnable = true;
  bool isRotating = true;

  LARGE_INTEGER t0, t1, t2, trot0, trot1;
  LARGE_INTEGER frequency;
  QueryPerformanceCounter(&t0);
  QueryPerformanceFrequency(&frequency);

  float P0 = 0;
  float I = 0;


  while (!isDone) {
    t2 = t1;
    QueryPerformanceCounter(&t0);
    if (!cap.read(frame)) {
      std::cout << "Error Reading camera\n";
      exit(-1);
    }
    QueryPerformanceCounter(&t1);
    float dtcam = (t1.QuadPart - t0.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
    float dtpro = (t0.QuadPart - t2.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
    if (dtcam > 80 || dtpro > 80)
      printf("dtcam = %f  dtpro = %f\n", dtcam, dtpro);

    resize(frame, frame, Size(480, 640));
    Mat hsv_image;
    cvtColor(frame, hsv_image, COLOR_BGR2HSV); // conver to HSV
    Mat blue_range;
    inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), blue_range);

    GaussianBlur(blue_range, blue_range, Size(9, 9), 2, 2);
    threshold(blue_range, blue_range, 100, 255, THRESH_BINARY);
    erode(blue_range, blue_range, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)), Point(-1, -1), 1);


    gpad.update();
    if (gpad.rightButtonPress()) gamePadEnable = !gamePadEnable;
    if (gamePadEnable) {
      float lx = scaleInput(gpad.getStickLeftX());
      float ly = scaleInput(gpad.getStickLeftY());
      float rx = scaleInput(gpad.getStickRightX());
      float ry = scaleInput(gpad.getStickRightY());
      setThrottle(ly * 200, ry * 200);
    }

    Point2f p;

    int iband = 4; //3
    bool found = getMassCenter(blue_range, Rect(0, blue_range.rows - 50 * (iband + 1), blue_range.cols, 50), p);
    line(frame, Point(0, frame.rows - 50 * iband), Point(frame.cols, frame.rows - 50 * iband), Scalar(0, 0, 0), 1);
    line(frame, Point(0, frame.rows - 50 * (iband + 1)), Point(frame.cols, frame.rows - 50 * (iband + 1)), Scalar(0, 0, 0), 1);
    if (found)
      circle(frame, p, 9, Scalar(0, 0, 255), 3);

    int iband_next = iband;
    line(frame, Point(0, frame.rows - 50 * (iband + 2)), Point(frame.cols, frame.rows - 50 * (iband + 2)), Scalar(0, 0, 0), 1);
    Point2f pl, pc, pr;
    bool found_next_left = getMassCenter(blue_range, Rect(0, blue_range.rows - 50 * (iband + 2), blue_range.cols / 3.0f, 50), pl);
    bool found_next_center = getMassCenter(blue_range, Rect(blue_range.cols / 3.0f, blue_range.rows - 50 * (iband_next + 1),
      blue_range.cols / 3.0f, 50), pc);
    bool found_next_right = getMassCenter(blue_range, Rect(2 * blue_range.cols / 3.0f, blue_range.rows - 50 * (iband_next + 1),
      blue_range.cols / 3.0f, 50), pr);

    if (found_next_left)   circle(frame, pl, 9, Scalar(0, 0, 255), 3);
    if (found_next_center)   circle(frame, pc, 9, Scalar(0, 0, 255), 3);
    if (found_next_right)   circle(frame, pr, 9, Scalar(0, 0, 255), 3);
    if (found_next_right) found = false;

    if (found && !gamePadEnable && !isRotating) {
      float ly = 1;
      float ry = 1;
      float P = p.x - blue_range.cols / 2.0f;
      float D = P - P0;
      I += P;
      P0 = P;

      float kp = 1 / 3.0f; // 1/3
      float ki = 1 / 10000.0f * 1;
      float kd = 3 / 2.0f;
      int pd = kp * P + ki * I + kd * D;

      const int max = 120; //120
      pd = pd > max ? max : pd < -max ? -max : pd;
      ly = pd < 0 ? max + pd : max;
      ry = pd > 0 ? max - pd : max;
      setThrottle(ly, ry);

      std::cout << "-------------- " << ly << " " << ry << "  kp = " << kp << " " << P << std::endl;
    }
    if (!found && !gamePadEnable && !isRotating) {
      QueryPerformanceCounter(&trot0);
      isRotating = true;
    }
    if (isRotating) {
      QueryPerformanceCounter(&trot1);
      float dtrot = (trot1.QuadPart - trot0.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
      if (100 < dtrot && dtrot < 200)
        setThrottle(100, -100);
      else if (dtrot < 300)
        setThrottle(0, 0);
      else
        isRotating = false;
    }

    cv::imshow("RobotView", blue_range);
    cv::imshow("Camera", frame);

    int key = cv::waitKey(30);
    switch (key) {
    case 'g':
      gamePadEnable = !gamePadEnable;
      break;
    case 27:
      isDone = true;
      break;
    }
  }
}
