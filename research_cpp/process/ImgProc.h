#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

namespace ImgProc
{
	using Image = cv::Mat;

	class CarsDetector;
	class CarsTracer;

	static std::vector<Image> gTemplates;
};
