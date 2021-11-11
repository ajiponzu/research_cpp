#pragma once
#include "ImgProc.h"

class ImgProc::CarsDetector
{
private:
	int mFrameWid = 0; //入力フレームの横幅
	int mFrameHigh = 0; //入力フレームの縦幅

	/* 出力画像バッファ */
	Image mSubtracted; //背景差分画像
	Image mShadow; //車影画像
	Image mReShadow;//車影再抽出画像
	Image mCars; //車両画像
	Image mCarRects; //車両検出矩形画像
	/* end */

	/* 画像処理に用いるバッファ */
	Image mTemp; //バッファ
	Image mGray; //グレースケール化のためのバッファ
	Image mBinary; //二値化のためのバッファ
	Image mLab128; //グレースケール化のために, L*a*b*のa値とb値を128にするためのバッファ, チャンネル数1
	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群
	Image mExceptedShadows; //除外すべき影画像
	Image mMorphKernel; // モルフォロジで使用するカーネル
	/* end */

	/* 外部リソース */
	Image& m_rBackImg; //背景画像
	Image& m_rRoadMask; //道路マスク画像
	std::vector<Image> mRoadMasks; //車線マスク画像群
	/* end */

public:

	/// <summary>
	/// コンストラクタ, メンバの初期化, 渡した画像によって作成する画像サイズが決定する
	/// </summary>
	/// <param name="backImg">背景画像</param>
	/// <param name="roadMask">道路マスク画像</param>
	/// <param name="roadMasks">車線マスク画像群</param>
	CarsDetector(Image& backImg, Image& roadMask, std::vector<Image>& roadMasks)
		: m_rBackImg(backImg),
		m_rRoadMask(roadMask)
	{
		mFrameWid = backImg.cols;
		mFrameHigh = backImg.rows;

		cv::Size imgSize(mFrameWid, mFrameHigh);

		/* 3チャンネル画像の初期化 */
		mSubtracted = Image::zeros(imgSize, CV_8UC3);
		mShadow = Image::zeros(imgSize, CV_8UC3);
		mReShadow = Image::zeros(imgSize, CV_8UC3);
		mCars = Image::zeros(imgSize, CV_8UC3);
		mCarRects = Image::zeros(imgSize, CV_8UC3);
		mExceptedShadows = Image::zeros(imgSize, CV_8UC1);
		/* end */

		/* 1チャンネル画像の初期化 */
		mLab128 = Image::ones(imgSize, CV_8U) * 128;
		/* end */

		/* モルフォロジカーネルの初期化 */
		int kernelList[9] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
		mMorphKernel = Image(3, 3, CV_8U, kernelList);
		/* end */

		/* 車線マスク画像群のグレースケール化, 車両検出ではグレースケールのマスクしか用いない */
		mRoadMasks.resize(roadMasks.size());
		for (int idx = 0; idx < roadMasks.size(); idx++)
		{
			cv::cvtColor(roadMasks[idx], mTemp, cv::COLOR_BGR2GRAY);
			roadMasks[idx] = mTemp.clone();
		}
		/* end */
	}

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
	const Image GetCarsRect() const { return mCarRects; }

private:
	CarsDetector(const CarsDetector& other) = delete;
};

