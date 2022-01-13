#pragma once

#include "CarsExtractor.h"

class ImgProc::CarsExtractor::BackImageHandle
{
private:
	static Image sSubtracted; // グレースケール二値画像
	static Image sBackImgFloat;
	static Image sFrameFloat;
	static Image sMoveCarsMask; // グレースケール二値画像
	static Image sMoveCarsMaskPrev; // グレースケール二値画像
	static bool sIsExistPreBackImg;

	static cv::VideoWriter mVideoWriterBack;

public:
	static Image& GetSubtracted() { return sSubtracted; }

	static void CreatePreBackImg();
	static void UpdateBackground();
};

