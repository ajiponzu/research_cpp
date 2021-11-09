#include "CarsDetector.h"

#include <vector>

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
	cv::absdiff(src1, src2, subtracted);
	subtracted.convertTo(subtracted, CV_8U); // 型を戻す

	cv::cvtColor(subtracted, subtracted, cv::COLOR_BGR2GRAY); //グレースケール化
	cv::threshold(subtracted, subtracted, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); //大津の二値化, thrは必要ないので0を渡す

	cv::cvtColor(subtracted, subtracted, cv::COLOR_GRAY2BGR); // チャンネル数を戻す
	cv::bitwise_and(subtracted, roadMask, retImg); // マスキング処理

	return retImg;
}

/// <summary>
/// 車影抽出
/// </summary>
/// <param name="input">入力フレーム</param>
/// <param name="roadMask">マスク画像</param>
/// <returns>車影(仮)</returns>
Image CarsDetector::ExtractShadow(Image& input, Image& roadMask)
{
	Image shadow, lab, retImg;

	// [0], [1], [2]にl, a, bが分割して代入される動的配列
	std::vector<Image> vLab;

	cv::cvtColor(input, lab, cv::COLOR_BGR2Lab); //labに変換
	cv::split(lab, vLab); //split: チャンネルごとに分割する関数
	
	/* 参照型でリソース削減しつつ, わかりやすいエイリアスを定義 */
	auto& l = vLab[0];
	auto& a = vLab[1];
	auto& b = vLab[2];
	/* end */

	/* 統計量導出 */
	cv::Scalar meanLScalar, stdLScalar;
	double meanA = cv::mean(a)[0]; //cv::Scalarはdoubleの二次元配列なのかも?
	double meanB = cv::mean(b)[0];
	cv::meanStdDev(l, meanLScalar, stdLScalar);
	double meanL = meanLScalar[0];
	double stdL = stdLScalar[0];
	/* end */

	/* L値を決定する処理 */
	// np.where -> matA cmpop matB で代替
	// 戻り値はMatExprだが, Matと互換性あり
	// しかもcmpopがtrueの要素が255, それ以外の要素が0の行列として帰ってくる -> まんまwhere
	if ((meanA + meanB) <= 256)
	{
		auto thr = meanL - stdL / 3;
		l = (l <= thr);
	}
	else
	{
		int thr_l = 10, thr_b = 5;
		auto idxGroupL = (l <= thr_l); 
		auto idxGroupB = (b <= thr_b);
		l = idxGroupL.mul(idxGroupB);
	}
	/* end */

	/* a, b値を128で埋めてグレースケール化 */
	a = Image::ones(cv::Size(a.cols, a.rows), CV_8U) * 128;
	b = Image::ones(cv::Size(b.cols, b.rows), CV_8U) * 128;
	/* end */

	//統合処理
	cv::merge(vLab, retImg);
	cv::cvtColor(retImg, retImg, cv::COLOR_Lab2BGR);

	return retImg;
}

Image CarsDetector::ReExtractShadow(Image& shadow, const int& areaThr, const float& aspectThr)
{
	return Image();
}

Image CarsDetector::ExtractCars(Image& input, Image& shadow)
{
	return Image();
}