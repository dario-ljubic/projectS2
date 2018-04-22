#ifndef PROJECTS2_LINEDETECTION_H
#define PROJECTS2_LINEDETECTION_H

#include "opencv2/imgproc.hpp"

class LineDetection {

public:
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
};

#endif // PROJECTS2_LINEDETECTION_H
