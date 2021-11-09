#include "CarsDetector.h"

using Image = cv::Mat;

/// <summary>
/// 背景差分
/// </summary>
/// <param name="input">入力フレーム</param>
/// <param name="backImg">背景画像</param>
/// <param name="roadMask">マスク画像</param>
/// <returns>移動物体画像</returns>
Image CarsDetector::SubtractImage(Image& input, Image& backImg, Image& roadMask)
{
	Image subtracted, src1, src2, retImg;

	input.convertTo(src1, CV_32F); //浮動小数はcv_32fを使う -> cv_16fだと謎のエラー
	backImg.convertTo(src2, CV_32F);
	subtracted = cv::abs(src1 - src2);
	subtracted.convertTo(subtracted, CV_8U); // 型を戻す

	cv::cvtColor(subtracted, subtracted, cv::COLOR_BGR2GRAY); //グレースケール化
	cv::threshold(subtracted, subtracted, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); //大津の二値化, thrは必要ないので0を渡す

	cv::cvtColor(subtracted, subtracted, cv::COLOR_GRAY2BGR); // チャンネル数を戻す
	cv::bitwise_and(subtracted, roadMask, retImg); // マスキング処理

	return retImg;
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