#pragma once

#include "CarsExtractor.h"

class ImgProc::CarsExtractor::BackImageHandle
{
private:
	static Image sSubtracted; // グレースケール二値画像
	static Image sBackImg;
	static Image sBackImgFloat;
	static Image sFrameFloat;
	static Image sMoveCarsMask; // グレースケール二値画像
	static bool sIsExistPreBackImg;

public:
	static const Image& GetBackImg() { return sBackImg; }
	static Image& GetSubtracted() { return sSubtracted; }

	static void CreatePreBackImg();
	static void UpdateBackground();
};

