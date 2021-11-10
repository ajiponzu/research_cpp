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
	Image gray; //グレースケール化のためのバッファ
	Image binary; //二値化のためのバッファ
	Image src1C3; //二項演算のためのバッファ1, チャンネル数3
	Image src2C3; //二項演算のためのバッファ2, チャンネル数3
	Image src1C1; //二項演算のためのバッファ1, チャンネル数1
	Image src2C1; //二項演算のためのバッファ2, チャンネル数1
	Image fSrc1C3; //小数を含む二項演算のためのバッファ1, チャンネル数3
	Image fSrc2C3; //小数を含む二項演算のためのバッファ2, チャンネル数3
	Image fSrc1C1; //小数を含む二項演算のためのバッファ1, チャンネル数1
	Image fSrc2C1; //小数を含む二項演算のためのバッファ2, チャンネル数1
	Image labels; //ラベル画像
	Image stats; //ラベリングにおける統計情報
	Image centroids; //ラベリングにおける中心点座標群
	Image exceptedShadows; //除外すべき影画像
	/* end */

	/* 外部リソース */
	Image& backImg; //背景画像
	Image& roadMask; //道路マスク画像
	std::vector<Image>& roadMasks; //車線マスク画像群
	/* end */

public:

	/// <summary>
	/// コンストラクタ, メンバの初期化, 渡した画像によって作成する画像サイズが決定する
	/// </summary>
	/// <param name="backImg">背景画像</param>
	/// <param name="roadMask">道路マスク画像</param>
	/// <param name="roadMasks">車線マスク画像群</param>
	CarsDetector(Image& backImg, Image& roadMask, std::vector<Image>& roadMasks)
		: backImg(backImg),
		roadMask(roadMask),
		roadMasks(roadMasks)
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
		src1C3 = Image::zeros(imgSize, CV_8UC3);
		src2C3 = Image::zeros(imgSize, CV_8UC3);
		fSrc1C3 = Image::zeros(imgSize, CV_32FC3);
		fSrc2C3 = Image::zeros(imgSize, CV_32FC3);
		exceptedShadows = Image::zeros(imgSize, CV_8UC1);
		/* end */

		/* 1チャンネル画像の初期化 */
		src1C1 = Image::zeros(imgSize, CV_8UC1);
		src2C1 = Image::zeros(imgSize, CV_8UC1);
		fSrc1C1 = Image::zeros(imgSize, CV_32FC1);
		fSrc2C1 = Image::zeros(imgSize, CV_32FC1);
		/* end */
	}

	~CarsDetector() {}

	//背景差分
	void SubtractBackImage(Image& frame);
	//車影抽出
	void ExtractShadow(Image& frame);
	//車影再抽出
	void ReExtractShadow(const int& areaThr, const float& aspectThr);
	void ExtractCars();
	//void DrawRectangle()

	void WriteOutImgs(std::vector<std::string> pathList);
	void ShowOutImgs(const int& interval);

private:
	CarsDetector(const CarsDetector& other) = delete;
};
