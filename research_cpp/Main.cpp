#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
	cv::Mat image = cv::Mat::zeros(64, 64, CV_8UC3);
	cv::imshow("demo", image);
	cv::waitKey(0);

	return 0;
}

