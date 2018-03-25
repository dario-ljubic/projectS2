#include <iostream>
#include "EdgeDetection.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

int main(int argc, char **argv) {

    cv::Mat input_image_raw;
    std::string filename;

    if (argc != 2) {
        std::cout << "Image path has not been given as an argument. Using default image..." << std::endl;
        std::cout << "Usage: ./projectS2 [image_name --default ../data/Lenna.png]" << std::endl;
        filename = "../data/image_0003.jpg";
    }
    else {
        filename = argv[1];
    }

    input_image_raw = cv::imread( filename, cv::IMREAD_COLOR );

    if(input_image_raw.empty()){
        std::cout << "Error opening image." << std::endl;
        return 1;
    }

    // TODO: Image preparation - to grayscale, to float
    // TODO: create a class for image processing

    // Convert the image to grayscale
    cv::Mat input_image_gray;
    cv::cvtColor(input_image_raw, input_image_gray, CV_BGR2GRAY);

    EdgeDetection test;

    cv::Mat edges;
    test.CannyEdgeDetection(input_image_gray, edges);

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", edges );

    cv::waitKey(0);

    return EXIT_SUCCESS;
}
