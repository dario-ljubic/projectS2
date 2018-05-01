#include "LineDetection.h"

cv::Mat LineDetection::edges_;
cv::Mat LineDetection::standard_hough_;
cv::Mat LineDetection::probabilistic_hough_;
std::string LineDetection::standard_name_ = "Standard Hough Lines Testing";
std::string LineDetection::probabilistic_name_ = "Probabilistic Hough Lines Testing";
int LineDetection::min_threshold_ = 50;
int LineDetection::max_trackbar_ = 150;
int LineDetection::s_trackbar_ = max_trackbar_;
int LineDetection::p_trackbar_ = max_trackbar_;

void LineDetection::StandardHough(cv::Mat& edge_img,
                                  cv::Mat& standard_hough) {

  std::vector<cv::Vec2f> s_lines;
  cv::cvtColor(edge_img, standard_hough, cv::COLOR_GRAY2BGR);

  // TODO: Add option to choose rho and theta for accumulator array!
  // Currently rho=1 and theta=1 degree
  cv::HoughLines(edge_img, s_lines, 1, CV_PI/180, threshold_, 0, 0);

  for(cv::Vec2f line : s_lines) {
    float r = line[0], t = line[1];
    double cos_t = std::cos(t), sin_t = std::sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    cv::Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
    cv::Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));

    // Draw a line defined by points pt1 and pt2 on a standard_hough image
    // with blue color, thickness 1 and antialiased line.
    cv::line(standard_hough, pt1, pt2, cv::Scalar(255,0,0), 1, cv::LINE_AA);
  }
}

void LineDetection::ProbabilisticHough(cv::Mat& edge_img,
                                       cv::Mat& probabilistic_hough) {
  std::vector<cv::Vec4i> p_lines;
  cv::cvtColor(edge_img, probabilistic_hough, cv::COLOR_GRAY2BGR);

  // TODO: Add option to choose minLineLength and maxLineGap!
  // Currently minLineLength=20 and maxLineGap=10
  cv::HoughLinesP(edge_img, p_lines, 1, CV_PI/180, threshold_, 20, 10 );

  for(const cv::Vec4i& line : p_lines)
  {
    cv::line(probabilistic_hough, cv::Point(line[0], line[1]),
             cv::Point(line[2], line[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);
  }
}

void LineDetection::StandardHoughTest(int, void*) {
  std::vector<cv::Vec2f> s_lines;
  cv::cvtColor(edges_, standard_hough_, cv::COLOR_GRAY2BGR);

  cv::HoughLines(edges_, s_lines, 1, CV_PI/180, min_threshold_ + s_trackbar_,
                  0, 0);

  for(cv::Vec2f line : s_lines) {
    float r = line[0], t = line[1];
    double cos_t = std::cos(t), sin_t = std::sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    cv::Point pt1(cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t));
    cv::Point pt2(cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t));
    cv::line(standard_hough_, pt1, pt2, cv::Scalar(255,0,0), 1, cv::LINE_AA);
  }

  cv::imshow(standard_name_, standard_hough_);
}

void LineDetection::ProbabilisticHoughTest( int, void* ) {
  std::vector<cv::Vec4i> p_lines;
  cv::cvtColor(edges_, probabilistic_hough_, cv::COLOR_GRAY2BGR);

  cv::HoughLinesP(edges_, p_lines, 1, CV_PI/180, min_threshold_ + p_trackbar_,
                  20, 10);

  for(const cv::Vec4i& line : p_lines) {
    cv::line(probabilistic_hough_, cv::Point(line[0], line[1]),
             cv::Point(line[2], line[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);
  }

  cv::imshow(probabilistic_name_, probabilistic_hough_);
}

void LineDetection::RunHoughThresholdTesting(const cv::Mat& edges) {

  char thresh_label[50];
  // Since edges image is not altered, deep copy is not required.

  // In case of standard and probabilistic hough images, if one should do a
  // shallow copy i.e. just do standard_hough_ = edges_ , then the data would
  // not be copied to the new matrix. A pointer would point to the original
  // data and if the data is altered as it is the case here when adding lines
  // in an image, the original data would also be altered. Therefore, deep
  // copy is required. Also, it should be done every time the bar has changed
  // its position. It will be done with cvtColor in order to have the ability
  // to color the found lines.
  LineDetection::edges_ = edges;

  sprintf(thresh_label, "Thres: %d + input", LineDetection::min_threshold_);

  cv::namedWindow(standard_name_, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar( thresh_label,
                      standard_name_,
                      &s_trackbar_,
                      max_trackbar_,
                      LineDetection::StandardHoughTest);

  cv::namedWindow(probabilistic_name_, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar( thresh_label,
                      probabilistic_name_,
                      &p_trackbar_,
                      max_trackbar_,
                      LineDetection::ProbabilisticHoughTest);

  LineDetection::StandardHoughTest(0, nullptr);
  LineDetection::ProbabilisticHoughTest(0, nullptr);
  cv::waitKey(0);
}
