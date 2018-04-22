#include <iostream>
#include "EdgeDetection.h"
#include "LineDetection.h"
#include "InputParser.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

int main(int argc, char **argv) {

  InputParser parser(argc, argv);
  std::string filename;
  uint8_t camera_num;
  cv::Mat input_image_raw;

  if (argc < 2) {
    std::cout << "Not enough input arguments! Add [-h] to see help. \n";
    return EXIT_FAILURE;
  }

  if (parser.CmdOptionExists("-h")) {
    std::cout << "Usage instructions: \n";
    std::cout
        << "  -f : offline usage. You should provide a path to the image. If"
            " the path is not provided, default image is used.\n";
    std::cout << "  -l : live usage. You should choose which camera to use. In"
        " contrary, build in camera is used.\n";
    return EXIT_SUCCESS;
  }

  if (parser.CmdOptionExists("-f")) {

    filename = parser.GetCmdOption("-f");

    if (filename.empty()) {
      filename = "../data/Lenna.jpg";
    }

    input_image_raw = cv::imread(filename, cv::IMREAD_COLOR);
  }

  if (parser.CmdOptionExists("-l")) {

    filename = parser.GetCmdOption("-l");

    if (filename.empty()) {
      filename = "0";
    }

    camera_num = std::stoi(filename);

    cv::VideoCapture capture(camera_num);
    if (!capture.isOpened()) {
      std::cout << "Not able to access the camera!\n";
      return EXIT_FAILURE;
    }

    while (true) {
      cv::Mat frame;
      capture >> frame; // get a new frame from camera
      cv::imshow("Stream", frame);

      int key_pressed = cv::waitKey(1);

      if (key_pressed % 256 == 27) {
        std::cout << "Escape hit, closing... \n";
        break;
      }

      if (key_pressed % 256 == 32) {
        std::cout << "Space hit, taking an image... \n";
        input_image_raw = frame;
        break;
      }
    }
  }

  if (input_image_raw.empty()) {
    std::cout << "Error opening image." << std::endl;
    return EXIT_FAILURE;
  }

  // TODO: Image preparation - to grayscale, to float
  // TODO: create a class for image processing

  // Convert the image to grayscale
  cv::Mat input_image_gray;
  cv::cvtColor(input_image_raw, input_image_gray, CV_BGR2GRAY);

  // Edge detection
  EdgeDetection test;

  cv::Mat edges;
  test.CannyEdgeDetection(input_image_gray, edges);

  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display window", edges);

  cv::waitKey(0);

  // Line detection
  /// Create Trackbars for Thresholds
  LineDetection lines;
  cv::Mat standard_hough, probabilistic_hough;

  /// Initialize
  lines.SetThreshold(70);

  lines.StandardHough(edges, standard_hough);
  cv::imshow("Standard Hough", standard_hough);
  cv::waitKey(0);


  lines.ProbabilisticHough(edges, probabilistic_hough);
  cv::imshow("Probabilistic Hough", probabilistic_hough);
  cv::waitKey(0);

  return EXIT_SUCCESS;
}
