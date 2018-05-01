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
  cv::cvtColor( edge_img, standard_hough, cv::COLOR_GRAY2BGR );

  cv::HoughLines( edge_img, s_lines, 1, CV_PI/180, threshold_, 0, 0 );

  for(cv::Vec2f line : s_lines) {
    float r = line[0], t = line[1];
    double cos_t = std::cos(t), sin_t = std::sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    cv::Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
    cv::Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
    cv::line( standard_hough, pt1, pt2, cv::Scalar(255,0,0), 3, cv::LINE_AA);
  }
}

void LineDetection::ProbabilisticHough(cv::Mat& edge_img,
                                       cv::Mat& probabilistic_hough) {
  std::vector<cv::Vec4i> p_lines;
  cv::cvtColor( edge_img, probabilistic_hough, cv::COLOR_GRAY2BGR );

  HoughLinesP( edge_img, p_lines, 1, CV_PI/180, threshold_, 20, 10 );

  for(const cv::Vec4i& line : p_lines)
  {
    cv::Vec4i l = line;
    cv::line( probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
              cv::Scalar(255,0,0), 3, cv::LINE_AA);
  }
}

void LineDetection::StandardHoughTest(int, void*) {
  std::vector<cv::Vec2f> s_lines;
  cv::cvtColor( edges_, standard_hough_, cv::COLOR_GRAY2BGR );

  HoughLines( edges_, s_lines, 1, CV_PI/180, min_threshold_ +
                  s_trackbar_, 0,
              0 );
  for( cv::Vec2f line : s_lines) {
    float r = line[0], t = line[1];
    double cos_t = std::cos(t), sin_t = std::sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    cv::Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
    cv::Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
    cv::line( standard_hough_, pt1, pt2, cv::Scalar(255,0,0), 3, cv::LINE_AA);
  }

  cv::imshow( standard_name_, standard_hough_ );
}

void LineDetection::ProbabilisticHoughTest( int, void* ) {
  std::vector<cv::Vec4i> p_lines;
  cv::cvtColor( edges_, probabilistic_hough_, cv::COLOR_GRAY2BGR );

  HoughLinesP( edges_, p_lines, 1, CV_PI/180, min_threshold_ +
                   p_trackbar_, 30,
               10 );

  for(const cv::Vec4i& line : p_lines) {
    cv::Vec4i l = line;
    cv::line( probabilistic_hough_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
              cv::Scalar(255,0,0), 3, cv::LINE_AA);
  }

  cv::imshow(probabilistic_name_, probabilistic_hough_);
}

void LineDetection::RunHoughThresholdTesting(const cv::Mat& edges) {

  char thresh_label[50];
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
