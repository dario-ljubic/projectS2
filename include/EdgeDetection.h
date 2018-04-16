#ifndef PROJECTS2_EDGEDETECTION_H
#define PROJECTS2_EDGEDETECTION_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

class EdgeDetection {
 public:

  // TODO: Otsu method
  //double OtsuThreshold(cv::Mat& src, cv::Mat& dst, double thresh, double
  // maxval, int type);

  int CannyEdgeDetection(cv::Mat &input_image, cv::Mat &output_edge_map,
                         int aperature_size = 3, bool L2gradient = false);

  inline double GetGaussianSigmaX() {
    return gaussian_sigma_x_;
  }

  inline void SetGaussianSigmaX(double sigma) {
    gaussian_sigma_x_ = sigma;
  }

  inline double GetGaussianSigmaY() {
    return gaussian_sigma_y_;
  }

  inline void SetGaussianSigmaY(double sigma) {
    gaussian_sigma_y_ = sigma;
  }

  inline cv::Size GetGaussianKernelSize() {
    return gaussian_kernel_size_;
  }

  inline void SetGaussianKernelSize(cv::Size size) {
    gaussian_kernel_size_ = size;
  }

  inline int GetSobelSize() {
    return sobel_size_;
  }

  inline void SetSobelSize(int size) {
    sobel_size_ = size;
  }

 private:
  cv::Size gaussian_kernel_size_{cv::Size(3, 3)};
  double gaussian_sigma_x_{sqrt(2)};
  double gaussian_sigma_y_{sqrt(2)};
  int sobel_size_;

  std::pair<double, double> FindThresholdValue(double include_factor,
                                               double img_median,
                                               int num_bits);

  double CalculateImageMedian(cv::Mat &Input, int num_bits);

  int ParseImgDepth(int img_depth);

};

#endif //PROJECTS2_EDGEDETECTION_H
