#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    std::cout << "Hello, OpenCV!" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Create a simple colored image (blue background)
    cv::Mat image(300, 400, CV_8UC3, cv::Scalar(255, 0, 0));

    // Draw a white rectangle and green text
    cv::rectangle(image, cv::Point(50, 50), cv::Point(350, 250), cv::Scalar(255, 255, 255), 3);
    cv::putText(image, "Hello OpenCV", cv::Point(60, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    // Display the image
    cv::imshow("Demo", image);
    cv::waitKey(0);

    return 0;
}
