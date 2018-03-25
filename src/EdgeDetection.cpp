#include "EdgeDetection.h"

std::pair<double, double> EdgeDetection::FindThresholdValue(double include_factor, double img_median,
                                                            int num_bits) {

    double upper_threshold = std::fmin(std::pow(2, num_bits), (1+include_factor) * img_median);
    double lower_threshold = std::fmax(0, (1-include_factor) * img_median);
    return std::make_pair(upper_threshold, lower_threshold);
}

double EdgeDetection::CalculateImageMedian(cv::Mat& Input, int num_bits) {

    if (Input.channels() != 1) {
        return -1;
    }

    int nVals;
    nVals = std::pow(2, num_bits);

    // Create histogram
    float range[] = { 0, nVals };
    const float* histRange =  range ;
    bool uniform = true;
    bool accumulate = false;

    cv::Mat hist;
    // Calculate histogram of only 1-st image, and just the 0-th channel, do not use the mask
    // output histogram, histogram dimensionality,
    calcHist(&Input, 1, 0, cv::Mat(), hist, 1, &nVals, &histRange, uniform, accumulate);

    // Compute Cumulative Distribution Function
    cv::Mat cdf;
    hist.copyTo(cdf);
    for (int i = 1; i <= nVals-1; i++){
        cdf.at<float>(i) += cdf.at<float>(i - 1);
    }
    // Returns the total number of array elements.
    cdf /= Input.total();

    // Compute median value.
    double medianVal{0};
    for (int i = 0; i <= nVals-1; i++) {
        if (cdf.at<float>(i) >= 0.5) {
            medianVal = i;
            break;
        }
    }
    return medianVal;
}

int EdgeDetection::ParseImgDepth(int img_depth) {

    switch(img_depth) {
        case 0 : return 8; //#define CV_8U   0
        case 1 : return 8; //#define CV_8S   1
        case 2 : return 16; //#define CV_16U  2
        case 3 : return 16; //#define CV_16S  3
        case 4 : return 32; //#define CV_32S  4
        case 5 : return 32; //#define CV_32F  5
        case 6 : return 64; //#define CV_64F  6
                            //#define CV_USRTYPE1 7
        default: return -1;
    }
}

int EdgeDetection::CannyEdgeDetection(cv::Mat &input_image, cv::Mat &output_edge_map,
                                       int aperature_size, bool L2gradient) {

    cv::Mat blurred_image;
    cv::GaussianBlur(input_image, blurred_image, gaussian_kernel_size_, gaussian_sigma_x_,
                     gaussian_sigma_y_, cv::BORDER_DEFAULT);

    int num_bits = EdgeDetection::ParseImgDepth(input_image.depth());
    if (num_bits < 0) {
        return -1;
    }

    double img_median = CalculateImageMedian(blurred_image, num_bits);

    std::pair<double, double> canny_thresholds = FindThresholdValue(0.2, img_median, num_bits);
    cv::Canny(blurred_image, output_edge_map, canny_thresholds.first, canny_thresholds.second,
              aperature_size, L2gradient);

    // check if image has depth ot whatever and return 0 if everything is okay
}
