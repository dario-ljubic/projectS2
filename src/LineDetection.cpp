#include "LineDetection.h"

#include "opencv2/highgui.hpp"

void LineDetection::StandardHough(cv::Mat& edge_img,
                                  cv::Mat& standard_hough) {

  std::vector<cv::Vec2f> s_lines;
  cv::cvtColor( edge_img, standard_hough, cv::COLOR_GRAY2BGR );

  /// 1. Use Standard Hough Transform
  cv::HoughLines( edge_img, s_lines, 1, CV_PI/180, threshold_, 0, 0 );

  /// Show the result
  for( size_t i = 0; i < s_lines.size(); i++ )
  {
    float r = s_lines[i][0], t = s_lines[i][1];
    double cos_t = cos(t), sin_t = sin(t);
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

  /// 2. Use Probabilistic Hough Transform
  HoughLinesP( edge_img, p_lines, 1, CV_PI/180, threshold_, 20, 10 );

  /// Show the result
  for( size_t i = 0; i < p_lines.size(); i++ )
  {
    cv::Vec4i l = p_lines[i];
    cv::line( probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
              cv::Scalar(255,0,0), 3, cv::LINE_AA);
  }
}
