#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {
    const std::string output_path =
        argc > 1 ? argv[1] : "opencv-headless-check.png";

    std::cout << "Hello, OpenCV!" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Create an image in memory; no X11 display is required.
    cv::Mat image(300, 400, CV_8UC3, cv::Scalar(255, 0, 0));

    cv::rectangle(image, cv::Point(50, 50), cv::Point(350, 250), cv::Scalar(255, 255, 255), 3);
    cv::putText(image, "Hello OpenCV", cv::Point(60, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    if (!cv::imwrite(output_path, image)) {
        std::cerr << "Failed to write image: " << output_path << std::endl;
        return 1;
    }

    const cv::Scalar mean_bgr = cv::mean(image);
    std::cout << "Image size: " << image.cols << "x" << image.rows << std::endl;
    std::cout << "Mean BGR: " << mean_bgr[0] << ", " << mean_bgr[1]
              << ", " << mean_bgr[2] << std::endl;
    std::cout << "Wrote: " << output_path << std::endl;

    return 0;
}
