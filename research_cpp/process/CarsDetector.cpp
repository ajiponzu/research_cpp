#include "CarsDetector.h"

using Image = cv::Mat;

Image CarsDetector::SubtractImage(Image& input, Image& backImg, Image& roadMask)
{
	return Image();
}

Image CarsDetector::ExtractShadow(Image& input, Image& roadMask)
{
	return Image();
}

Image CarsDetector::ReExtractShadow(Image& shadow, const int& areaThr, const float& aspectThr)
{
	return Image();
}

Image CarsDetector::ExtractCars(Image& input, Image& shadow)
{
	return Image();
}