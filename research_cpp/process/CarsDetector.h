#pragma once
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "ImgProc.h"

class ImgProc::CarsDetector
{
private:
	int frameWid = 0; //入力フレームの横幅
	int frameHigh = 0; //入力フレームの縦幅

	/* 出力画像バッファ */
	Image subtracted; //背景差分画像
	Image shadow; //車影画像
	Image reshadow;//車影再抽出画像
	Image cars; //車両画像
	Image carRects; //車両検出矩形画像
	/* end */

	/* 画像処理に用いるバッファ */
	Image tmp; //バッファ
	Image gray; //グレースケール化のためのバッファ
	Image binary; //二値化のためのバッファ
	Image lab128; //グレースケール化のために, L*a*b*のa値とb値を128にするためのバッファ, チャンネル数1
	Image labels; //ラベル画像
	Image stats; //ラベリングにおける統計情報
	Image centroids; //ラベリングにおける中心点座標群
	Image exceptedShadows; //除外すべき影画像
	Image morphKernel; // モルフォロジで使用するカーネル
	/* end */

	/* 外部リソース */
	Image& backImg; //背景画像
	Image& roadMask; //道路マスク画像
	std::vector<Image> roadMasks; //車線マスク画像群
	/* end */

public:

	/// <summary>
	/// コンストラクタ, メンバの初期化, 渡した画像によって作成する画像サイズが決定する
	/// </summary>
	/// <param name="backImg">背景画像</param>
	/// <param name="roadMask">道路マスク画像</param>
	/// <param name="roadMasks">車線マスク画像群</param>
	CarsDetector(Image& backImg, Image& roadMask, std::vector<Image>& _roadMasks)
		: backImg(backImg),
		roadMask(roadMask)
	{
		frameWid = backImg.cols;
		frameHigh = backImg.rows;

		cv::Size imgSize(frameWid, frameHigh);

		/* 3チャンネル画像の初期化 */
		subtracted = Image::zeros(imgSize, CV_8UC3);
		shadow = Image::zeros(imgSize, CV_8UC3);
		reshadow = Image::zeros(imgSize, CV_8UC3);
		cars = Image::zeros(imgSize, CV_8UC3);
		carRects = Image::zeros(imgSize, CV_8UC3);
		exceptedShadows = Image::zeros(imgSize, CV_8UC1);
		/* end */

		/* 1チャンネル画像の初期化 */
		lab128 = Image::ones(imgSize, CV_8U) * 128;
		/* end */

		/* モルフォロジカーネルの初期化 */
		int kernelList[9] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
		morphKernel = Image(3, 3, CV_8U, kernelList);
		/* end */

		/* 車線マスク画像群のグレースケール化, 車両検出ではグレースケールのマスクしか用いない */
		roadMasks.resize(_roadMasks.size());
		for (int idx = 0; idx < _roadMasks.size(); idx++)
		{
			cv::cvtColor(_roadMasks[idx], tmp, cv::COLOR_BGR2GRAY);
			roadMasks[idx] = tmp.clone();
		}
		/* end */
	}

	~CarsDetector() {}

	//背景差分, 移動物体検出
	void SubtractBackImage(Image& frame);
	//車影抽出
	void ExtractShadow(Image& frame);
	//車影再抽出
	void ReExtractShadow(const int& areaThr, const float& aspectThr);
	// 車両抽出
	void ExtractCars();
	// 車両を矩形で囲む
	void DrawRectangle(Image& frame, const int& areaThr);

	void WriteOutImgs(std::vector<std::string> pathList);
	void ShowOutImgs(const int& interval);
	const Image GetCarsRect() const { return carRects; }

private:
	CarsDetector(const CarsDetector& other) = delete;
};
