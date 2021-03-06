#if 0
#include <windows.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "gamepad.h"
#include "utils.h"
#include "SerialPort.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#define SERIAL 0

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


void moveRobot(float d) {
  int16_t v[2] = { 120, 120 };
  if (d > 0)
    v[1] *= (1-d);
  else
    v[0] *= (1-fabs(d));


  if (d < -1) {
    v[0] = 0;
    v[1] = 0;
  }
  printf("Left = %d  Right = %d d=%f\n", v[0], v[1], d);
  setThrottle(v);
}

void playFile(const std::string& Filename) {
  std::ifstream infile("samples.nn", std::ifstream::binary);
  int rows, cols, channels;
  infile.read(reinterpret_cast<char*>(&cols), sizeof(int));
  infile.read(reinterpret_cast<char*>(&rows), sizeof(int));
  infile.read(reinterpret_cast<char*>(&channels), sizeof(int));
  int size = rows * cols * channels;
  
  printf("Rows = %d Cols = %d Channels = %d\n", rows, cols, channels);

  Mat frame(rows, cols, CV_8UC1);
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
    if (waitKey(t1 - t0) >= 0) break;
    //Sleep(t1 - t0);
    t0 = t1;
  }

}

cv::Point getCenter(Mat& mat) {
  Moments m = moments(mat, &moments);
  cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
  return center;
}






int main(int, char**)
{
#if 0
  playFile("samples.nn");
  return 0;
#endif

#if 0

  VideoCapture cap; // open the default camera
  const std::string videoStreamAddress = "http://10.0.0.4:8080/video?x.mjpeg";
  if (!cap.open(videoStreamAddress)) {
    cap.release();
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }
#if SERIAL
  serial.connect("\\\\.\\COM11");
  if (!serial.IsConnected()) {
    std::cout << "Error Connecting bluethoot" << std::endl;
    return -1;
  }
#endif

  int iLowH = 70;
  int iHighH = 144;

  int iLowS = 0;
  int iHighS = 255;

  int iLowV = 75;
  int iHighV = 255;

  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  Mat frame;

  for (int i = 0; i < 20; ++i) {
    cap >> frame;
  }

  cap >> frame;
  while (true) {
    cap >> frame;
    Mat hsv_image;
    cvtColor(frame, hsv_image, COLOR_BGR2HSV);
    //erode(hsv_image, hsv_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    //dilate(hsv_image, hsv_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    //dilate(hsv_image, hsv_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    //erode(hsv_image, hsv_image, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    Mat hue_range;
    inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), hue_range); //Threshold the image
    GaussianBlur(hue_range, hue_range, Size(9, 9), 2, 2);
    threshold(hue_range, hue_range, 100, 255, THRESH_BINARY);

    Mat l0 = hue_range(Range(hue_range.rows-200, hue_range.rows - 100), Range(0, hue_range.cols));
    Point p = getCenter(l0);
    p.y += hue_range.rows - 100;
    circle(frame, p, 10, Scalar(255, 255, 255));

#if SERIAL
    moveRobot(2*(p.x / 480.0f - 0.5f));
#endif

    imshow("hue", hue_range);
    imshow("lower", l0);
    imshow("Original", frame);
    if (waitKey(10) >= 0) break;
  }
  cap.release();

#endif

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

  for (int i = 0; i < 20; ++i) {
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


#endif


#ifdef LINEFOLLOW

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "SerialPort.h"
#include <stdint.h>
#include <vector>
#include <iostream>

  SerialPort serial;

  void setThrottle(int16_t *v) {
    serial.WriteData((char*)v, 4);
  }

  void setThrottle(int16_t l, int16_t r) {
    int16_t v[2] = { l, r };
    serial.WriteData((char*)v, 4);
  }

  void setArray(int16_t *v, int16_t left, int16_t right) {
    v[0] = left;
    v[1] = right;
  }

  void moveRobot(float d) {
    int16_t v[2] = { 120, 120 };
    if (d > 0)
      v[1] *= (1 - d);
    else
      v[0] *= (1 - fabs(d));

    if (d < -1) {
      v[0] = 0;
      v[1] = 0;
    }
    printf("Left = %d  Right = %d d=%f\n", v[0], v[1], d);
    setThrottle(v);
  }
  void stopRobot() {
    setThrottle(0);
  }

  cv::Point2f getMassCenter(cv::Mat& mat, cv::Rect& r, bool &found) {
    cv::Moments m = cv::moments(mat(r));
    cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
    center.x += r.x;
    center.y += r.y;
    found = true;
    if (m.m00 == 0 || m.m00 == 0)
      found = false;
    return center;
  }


#define SERIAL 0
  int main() {
    using namespace cv;
    cv::VideoCapture cap; // open camera
    if (!cap.open("http://10.0.0.5:8080/video?x.mjpeg")) {
      cap.release();
      std::cout << "Error opening video stream or file" << std::endl;
      exit(-1);
    }
#if SERIAL
    serial.connect("\\\\.\\COM11");
    if (!serial.IsConnected()) {
      std::cout << "Error Connecting bluethoot" << std::endl;
      return -1;
    }
#endif

    int iLowH = 85;
    int iHighH = 132;

    int iLowS = 12;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    RNG rng(12345);
    namedWindow("Camera");
    bool notDone = true;
    Matx33f H = { 243.4f, -82.8f, 120.0f, -0.0280f, 69.6f,
      234.0f, -0.0044f, -0.337f, 246.0f };
    Mat frame, birdEye, hsv_image;
    while (notDone) {
      cap >> frame;
      warpPerspective(frame, birdEye, H, frame.size(), CV_INTER_LINEAR |
        CV_WARP_INVERSE_MAP |
        CV_WARP_FILL_OUTLIERS);

      birdEye = birdEye(Rect(0, 0, birdEye.cols, birdEye.rows - 105));
      line(birdEye, Point(birdEye.cols / 2, 0), Point(birdEye.cols / 2, birdEye.rows), Scalar(0, 0, 0), 1);
      cvtColor(birdEye, hsv_image, COLOR_BGR2HSV); // conver to HSV


      Mat hue_range;
      inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), hue_range);
      GaussianBlur(hue_range, hue_range, Size(9, 9), 2, 2);
      threshold(hue_range, hue_range, 100, 255, THRESH_BINARY);
      // hue_range = hue_range(Rect(0, 0, hue_range.cols, hue_range.rows - 105));
      erode(hue_range, hue_range, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)), Point(-1, -1), 5);
      std::vector<Point2f> points;
      points.push_back(Point2f(0, 0));
      float k = 1.4f;
      for (int i = 0; i < 5; ++i) {
        bool found = false;
        Point p = getMassCenter(hue_range, Rect(0, hue_range.rows - 50 * (i + 1), hue_range.cols, 50), found);
        line(birdEye, Point(0, birdEye.rows - 50 * (i + 1)), Point(birdEye.cols, birdEye.rows - 50 * (i + 1)), Scalar(0, 0, 0), 1);
        if (found) {
          circle(birdEye, p, 9, Scalar(0, 0, 255), 3);
          p.x = (p.x - hue_range.cols / 2) * k;
          p.y = (75 + hue_range.rows - p.y) * k;
          points.push_back(p);
        }
        else break;
      }
      if (!points.empty()) {
        int ip = 0;
        for (int i = 1; i < points.size(); ++i){
          float theta = atan2(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
          std::cout << fabs(theta / 3.1415 * 180) << " ";
          if (fabs(theta / 3.1415 * 180) < 20) ip = i;
        }
        std::cout << "\ngoto point " << ip << "\n";
      }

#if SERIAL
      setThrottle(v[0], v[1]);
#endif


      imshow("DR", hue_range);
      cv::imshow("Birds_Eye", birdEye);
      cv::imshow("Camera", frame);
      int key = cv::waitKey(20);
      switch (key) {
      case 27:
        notDone = false;
        break;
      }
    }
    cap.release();
  }


#endif //LINEFOLLOW

#if 1

#include <windows.h>
#include "gamepad.h"
#include <stdint.h>
#include "utils.h"
#include "SerialPort.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "homography.h"
#include <fstream>

#define SERIAL 0

#if SERIAL
  SerialPort serial;
#endif

cv::Point2f getMassCenter(cv::Mat& mat, cv::Rect& r, bool &found) {
  cv::Moments m = cv::moments(mat(r));
  cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
  center.x += r.x;
  center.y += r.y;
  found = true;
  if (m.m00 == 0 || m.m00 == 0)
    found = false;
  return center;
}
#if SERIAL
void setThrottle(int16_t *v) {
  serial.WriteData((char*)v, 4);
}

void setThrottle(int16_t l, int16_t r) {
  int16_t v[2] = { l, r };
  serial.WriteData((char*)v, 4);
}

void setArray(int16_t *v, int16_t left, int16_t right) {
  v[0] = left;
  v[1] = right;
}

void moveRobot(LARGE_INTEGER t0, LARGE_INTEGER frequency, const cv::Mat &frame, std::ofstream &outfile) {
  LARGE_INTEGER t1;
  QueryPerformanceCounter(&t1);
  int16_t v[2] = { 0 };

  float dt = (t1.QuadPart - t0.QuadPart) * 1000.0f / frequency.QuadPart; // [ms]
  if (dt > 2000 && dt <= 2600) setArray(v, 120, 120); // forward + turn
  if (dt > 2600 && dt <= 2650) setArray(v, -120, -120); // stop
  if (dt > 2650 && dt <= 3500) setArray(v, 0, 0); // stop
  if (dt > 3500 && dt <= 3800) setArray(v, -120, 120);
  if (dt >= 3800) setArray(v, 0, 0); // forward + more turn
  
  setThrottle(v);
  printf("dt=%f v0=%d v1=%d\n", dt, v[0], v[1]);

  if (dt > 2000 && dt < 3900) {
    outfile.write(reinterpret_cast<const char*>(&dt), sizeof(float));
    outfile.write(reinterpret_cast<const char*>(&v[0]), sizeof(int16_t));
    outfile.write(reinterpret_cast<const char*>(&v[1]), sizeof(int16_t));
    int size = frame.rows * frame.cols * frame.channels();
    outfile.write((char*)frame.data, size);
  }
}

#endif

void playFile(const std::string& Filename) {
  using namespace cv;
  std::ifstream infile("samples.nn", std::ifstream::binary);
  int rows, cols, channels;
  infile.read(reinterpret_cast<char*>(&cols), sizeof(int));
  infile.read(reinterpret_cast<char*>(&rows), sizeof(int));
  infile.read(reinterpret_cast<char*>(&channels), sizeof(int));
  int size = rows * cols;

  printf("Rows = %d Cols = %d Channels = %d\n", rows, cols, channels);

  Mat frame(rows, cols, CV_8UC1);
  namedWindow("Original", 1);

  float t0 = 2000;
  while (!infile.eof()) {
    int16_t v[2];
    float t1;
    infile.read(reinterpret_cast<char*>(&t1), sizeof(float));
    infile.read(reinterpret_cast<char*>(&v[0]), sizeof(int16_t));
    infile.read(reinterpret_cast<char*>(&v[1]), sizeof(int16_t));
    printf("dt = %f t1=%f v0=%d v1=%d\n", t1 - t0, t1, v[0], v[1]);
    infile.read((char*)frame.data, size);
    imshow("Original", frame);
    if (waitKey(t1 - t0) >= 0) break;
    //Sleep(t1 - t0);
    t0 = t1;
  }
}



int main() {
  //calibrateHomography("http://192.168.0.198:8080/video?x.mjpeg");
  //return 0;

  //playFile("");
  //return 0;

  using namespace cv;
  cv::VideoCapture cap; // open camera
  //if (!cap.open("http://admin:123456@192.168.0.154/video.cgi?.mjpg")) {
  if (!cap.open("http://192.168.0.198:8080/video?x.mjpeg")) {
    //  if (!cap.open("http://192.168.0.156:8000/video?x.mjpeg")) {
    cap.release();
    std::cout << "Error opening video stream or file" << std::endl;
    exit(-1);
  }

  int iLowH = 85;
  int iHighH = 132;

  int iLowS = 23;
  int iHighS = 255;

  int iLowV = 0;
  int iHighV = 255;

  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  Gamepad gpad(0);
#if SERIAL
  serial.connect("\\\\.\\COM11");
  if (!serial.IsConnected()) {
    std::cout << "Error Connecting bluethoot" << std::endl;
    return -1;
  }
#endif

  Matx33f H = { 225.50311f, -76.164383f, 130.0f, 7.7888198f, 30.851748f, 254.0f, 0.018633541f, -0.31853649f, 222.0f };

  //  Matx33f H = { 201.16461f, -78.65583f, 141.0f, 0.42764893f, 29.763798f, 234.0f, -0.0024355366f, -0.31489924f, 197.0f };

  //Zenfone 2  Matx33f H = { 243.4f, -82.8f, 120.0f, -0.0280f, 69.6f,  234.0f, -0.0044f, -0.337f, 246.0f };

  Mat frame, birdEye, hsv_image;
  for (int i = 0; i < 200; ++i)
    cap.read(frame);

  bool isDone = false;
  float dxlast = 0;
  int state = 0;
  LARGE_INTEGER t0, t1, t2;
  LARGE_INTEGER frequency;
  QueryPerformanceCounter(&t0);
  QueryPerformanceFrequency(&frequency);

  //std::ofstream outfile("samples.nn", std::ofstream::binary);
  //cap >> frame;
  //int cols = 256;
  //int rows = 256;
  //int channels = 1;
  //outfile.write(reinterpret_cast<const char*>(&cols), sizeof(int));
  //outfile.write(reinterpret_cast<const char*>(&rows), sizeof(int));
  //outfile.write(reinterpret_cast<const char*>(&channels), sizeof(int));
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

    line(frame, Point(0, frame.rows / 2 - 270), Point(frame.cols, frame.rows / 2 - 270), Scalar(0, 0, 0), 3);

    //warpPerspective(frame, birdEye, H, frame.size(), CV_INTER_LINEAR |
//      CV_WARP_INVERSE_MAP |
      //CV_WARP_FILL_OUTLIERS);

    //birdEye = birdEye(Rect((birdEye.cols - 300) / 2, 237, 300, birdEye.rows - 340)); //480 x 274
    //resize(birdEye, birdEye, Size(512, 512));
    //line(birdEye, Point(birdEye.cols / 2, 0), Point(birdEye.cols / 2, birdEye.rows), Scalar(0, 0, 0), 1);

    //cvtColor(birdEye, hsv_image, COLOR_BGR2HSV); // conver to HSV
    //Mat hue_range;
    //inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), hue_range);
    //GaussianBlur(hue_range, hue_range, Size(9, 9), 2, 2);
    //threshold(hue_range, hue_range, 100, 255, THRESH_BINARY);
    //erode(hue_range, hue_range, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)), Point(-1, -1), 1);

    cvtColor(frame, hsv_image, COLOR_BGR2HSV); // conver to HSV
    Mat hue_range;
    inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), hue_range);
    GaussianBlur(hue_range, hue_range, Size(9, 9), 2, 2);
    threshold(hue_range, hue_range, 100, 255, THRESH_BINARY);
    erode(hue_range, hue_range, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)), Point(-1, -1), 1);

    bool found = false;
    int iband = 4; //3
    Point2f p = getMassCenter(hue_range, Rect(0, hue_range.rows - 50 * (iband + 1), hue_range.cols, 50), found);
    line(frame, Point(0, frame.rows - 50 * (iband + 1)), Point(frame.cols, frame.rows - 50 * (iband + 1)), Scalar(0, 0, 0), 1);
    if (found) {
      circle(frame, p, 9, Scalar(0, 0, 255), 3);
    }

    int16_t v[2];
#if GAMEPAD
    gpad.update();
    float lx = scaleInput(gpad.getStickLeftX());
    float ly = scaleInput(gpad.getStickLeftY());
    float rx = scaleInput(gpad.getStickRightX());
    float ry = scaleInput(gpad.getStickRightY());
    v[0] = ly * 200;
    v[1] = ry * 200;
#endif

    if (found) {
      float ly = 1;
      float ry = 1;
      float P = p.x - hue_range.cols / 2.0f;
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

      v[0] = ly;
      v[1] = ry;
      std::cout << "-------------- " <<  ly << " " << ry << "  kp = " << kp << " " << P <<  std::endl;
#if SERIAL
      int ret = serial.WriteData((char*)v, 4); // send data to arduino 
#endif    
    }


    //// printf("lx = %f ly =%f rx = %f ry = %f\n", lx, ly, rx, ry);

    cv::imshow("RobotView", hue_range);
    cv::imshow("Camera", frame);
    //cv::imshow("Bird Eye View", birdEye);

    int key = cv::waitKey(30);
    switch (key) {
    case 27:
      isDone = true;
      break;
    }
  }
  // outfile.close();


}
#endif


//  Gamepad gpad(0);
// gpad.update();
// float lx = scaleInput(gpad.getStickLeftX());
// float ly = scaleInput(gpad.getStickLeftY());
// float rx = scaleInput(gpad.getStickRightX());
// float ry = scaleInput(gpad.getStickRightY());
// printf("lx = %f ly =%f rx = %f ry = %f\n", lx, ly, rx, ry);

// Zenfone2
// Homography Matrix :
//[243.40329, -82.878532, 120;
//-0.028021142, 69.600586, 234;
//-0.0043745581, -0.33711663, 245]
