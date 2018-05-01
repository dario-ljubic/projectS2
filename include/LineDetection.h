#ifndef PROJECTS2_LINEDETECTION_H
#define PROJECTS2_LINEDETECTION_H

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

class LineDetection {

public:
  static void RunHoughThresholdTesting(const cv::Mat& edges);

  void StandardHough(cv::Mat& edge_img,
                     cv::Mat& standard_hough);

  void ProbabilisticHough(cv::Mat& edge_img,
                          cv::Mat& standard_hough);

  inline int GetThreshold() {
    return threshold_;
  }

  void SetThreshold(int thres){
    threshold_ = thres;
  }

private:

  int threshold_;

  static void ProbabilisticHoughTest(int, void*);
  static void StandardHoughTest(int, void*);

  static cv::Mat edges_;
  static cv::Mat standard_hough_;
  static cv::Mat probabilistic_hough_;
  static std::string standard_name_;
  static std::string probabilistic_name_;
  static int min_threshold_;
  static int max_trackbar_;
  static int s_trackbar_;
  static int p_trackbar_;
};

#endif // PROJECTS2_LINEDETECTION_H
