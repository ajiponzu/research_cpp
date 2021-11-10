#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

namespace CarsDetector
{
	using Image = cv::Mat;

	//背景差分
	void SubtractImage(Image& input, Image& output, Image& backImg, Image& roadMask);
	//車影抽出
	void ExtractShadow(Image& input, Image& output, Image& roadMask);
	//車影再抽出
	void ReExtractShadow(Image& shadow, Image& output, const int& areaThr, const float& aspectThr);
	void ExtractCars(Image& input, Image& output, Image& shadow);
	//Image DrawRectangle(Image& input, std::vector::)
};
