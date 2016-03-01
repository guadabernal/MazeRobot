#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>


struct MouseZoom {
  std::vector<cv::Point2f>* pImgPoints;
  cv::Mat* pFrame;
};

void onMouseSelectPoints(int event, int x, int y, int a, void *p) {
  using namespace cv;
  MouseZoom *z = (MouseZoom*)(p);
  if (event == CV_EVENT_LBUTTONDOWN) {
    if (z->pImgPoints->size() < 4)
      z->pImgPoints->push_back(Point2f(x, y));
  }
  if (event == CV_EVENT_RBUTTONDOWN) {
    z->pImgPoints->clear();
  }
  if (event == EVENT_MOUSEMOVE) {
    cv::Mat& img = *(z->pFrame);
    float x0 = x - 10 < 0 ? 0 : x - 10;
    float y0 = y - 10 < 0 ? 0 : y - 10;
    if (x + 10 < img.cols && y + 10 < img.rows) {
      cv::Mat zoom = img(cv::Rect(x0, y0, 20, 20));
      cv::resize(zoom, zoom, cv::Size(200, 200));
      line(zoom, Point2f(zoom.cols / 2, 0), Point2f(zoom.cols / 2, zoom.rows), Scalar(0, 0, 255), 1);
      line(zoom, Point2f(0, zoom.rows / 2), Point2f(zoom.cols, zoom.rows / 2), Scalar(0, 0, 255), 1);

      cv::imshow("Zoom", zoom);
    }
  }
}

void calibrateHomography(std::string Camera) {
  using namespace cv;
  // open camera
  cv::VideoCapture cap;
  if (!cap.open(Camera.c_str())) { // change to 0 when using webcam
    cap.release();
    std::cout << "Error opening video stream or file" << std::endl;
    exit(-1);
  }

  // define image and projection points
  std::vector<Point2f> imgPoints;
  std::vector<Point2f> dstPoints = { Point2f(0, 0), Point2f(0, 1), Point2f(1, 1),
    Point2f(1, 0) };
  std::vector<Scalar> pointColors = { Scalar(0, 0, 255), Scalar(0, 255, 0),
    Scalar(255, 0, 0), Scalar(0, 255, 255) };
  cv::Mat frame;

  MouseZoom param;
  param.pImgPoints = &imgPoints;
  param.pFrame = &frame;

  namedWindow("Camera");
  setMouseCallback("Camera", onMouseSelectPoints, &param);

  float Z = 10;
  bool notDone = true;
  Matx33f H;
  while (notDone) {
    cap >> frame;
    resize(frame, frame, Size(480, 640));
    for (int i = 0; i < imgPoints.size(); ++i)
      circle(frame, imgPoints[i], 9, pointColors[i], 3);
    line(frame, Point2f(frame.cols / 2, 0), Point2f(frame.cols / 2, frame.rows), Scalar(0, 0, 0), 2);
    line(frame, Point2f(0, frame.rows / 2), Point2f(frame.cols, frame.rows / 2), Scalar(0, 0, 0), 2);
    if (imgPoints.size() == 4) {
      H = getPerspectiveTransform(dstPoints, imgPoints);
      H(2, 2) = Z;
      Mat birdEye;
      warpPerspective(frame, birdEye, H, frame.size(),
        CV_INTER_LINEAR | CV_WARP_INVERSE_MAP |
        CV_WARP_FILL_OUTLIERS);
      line(birdEye, Point2f(birdEye.cols / 2, 0), Point2f(birdEye.cols / 2, birdEye.rows), Scalar(0, 0, 255), 1);
      cv::imshow("Birds_Eye", birdEye);
    }

    cv::imshow("Camera", frame);
    int key = cv::waitKey(20);

    switch (key) {
    case 'u':
      Z += 1;
      std::cout << Z << std::endl;
      break;
    case 'd':
      Z -= 1;
      std::cout << Z << std::endl;
      break;
    case 27:
      notDone = false;
      break;
    }
  }
  std::cout << "Homography Matrix:\n";
  std::cout << H << std::endl;
  cap.release();
}