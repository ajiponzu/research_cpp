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
	auto& meanA = cv::mean(a)[0]; //cv::Scalarはdoubleの二次元配列なのかも?
	auto& meanB = cv::mean(b)[0];
	cv::meanStdDev(l, meanLScalar, stdLScalar);
	auto& meanL = meanLScalar[0];
	auto& stdL = stdLScalar[0];
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
	cv::merge(vLab, shadow);
	cv::cvtColor(shadow, shadow, cv::COLOR_Lab2BGR);

	cv::bitwise_and(shadow, roadMask, retImg); //マスキング処理

	return retImg;
}

/// <summary>
/// 車影再抽出
/// </summary>
/// <param name="shadow">車影画像</param>
/// <param name="areaThr">面積の閾値</param>
/// <param name="aspectThr">アスペクト比の閾値</param>
/// <returns></returns>
Image CarsDetector::ReExtractShadow(Image& shadow, const int& areaThr, const float& aspectThr)
{
	Image retImg, gray, labels, stats, _centroids;

	cv::cvtColor(shadow, gray, cv::COLOR_BGR2GRAY);
	//ラベル数
	auto labelNum = cv::connectedComponentsWithStats(gray, labels, stats, _centroids, 4); //ラベリング
	//車影でない部分のバッファ
	Image glasses = Image::zeros(cv::Size(gray.cols, gray.rows), CV_8U);

	/* 各領域ごとの処理, 0番は背景 */
	for (int label = 1; label < labelNum; label++)
	{
		/* 統計情報分割 */
		auto statsPtr = stats.ptr<int>(label);
		auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
		/* end */

		auto aspect = static_cast<float>(width) / height; //アスペクト比の導出
		auto tAreaThr = (y - 230) / 12 + areaThr; // 位置に応じた面積の閾値

		bool condArea = area < tAreaThr;
		bool condAspect = aspect > aspectThr;
		if (condArea || condAspect)
		{
			auto idxGroup = (labels == label);
			glasses.setTo(255, idxGroup); //車影でない部分を抽出
		}
	}
	/* end */

	cv::bitwise_xor(glasses, gray, retImg); //車影のみ抽出, xorなので先ほど作ったマスク以外の部分を車影として残す
	cv::cvtColor(retImg, retImg, cv::COLOR_GRAY2BGR);

	return retImg;
}

Image CarsDetector::ExtractCars(Image& input, Image& shadow)
{
	return Image();
}