#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

namespace CarsDetector
{
	using Image = cv::Mat;

	Image SubtractImage(Image& input, Image& backImg, Image& roadMask);
	Image ExtractShadow(Image& input, Image& roadMask);
	Image ReExtractShadow(Image& shadow, const int& areaThr, const float& aspectThr);
	Image ExtractCars(Image& input, Image& shadow);
	//Image DrawRectangle(Image& input, std::vector::)
};

