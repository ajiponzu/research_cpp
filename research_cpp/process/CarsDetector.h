#pragma once
#include "ImgProc.h"

class ImgProc::CarsDetector
{
private:
	/* 出力画像バッファ */
	Image mSubtracted; //背景差分画像, 1チャンネル固定
	Image mShadow; //車影画像, 1チャンネル固定
	Image mReShadow;//車影再抽出画像, 1チャンネル固定
	Image mCars; //車両画像, 1チャンネル固定
	Image mCarRects; //車両検出矩形画像, 3チャンネル固定
	/* end */

	/* 画像処理に用いるバッファ */
	Image mTemp; //バッファ
	Image mLab128; //グレースケール化のために, L*a*b*のa値とb値を128にするためのバッファ, チャンネル数1
	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群
	Image mExceptedShadows; //除外すべき影画像
	Image mMorphKernel; // モルフォロジで使用するカーネル
	/* end */

public:
	CarsDetector()
	{
		cv::Size imgSize(ImgProcToolkit::sVideoWidth, ImgProcToolkit::sVideoHeight);

		/* メンバ画像の初期化 */
		mCarRects = Image::zeros(imgSize, CV_8UC3);
		mExceptedShadows = Image::zeros(imgSize, CV_8U);
		mLab128 = Image::ones(imgSize, CV_8U) * 128;
		/* end */

		/* モルフォロジカーネルの初期化 */
		int kernelList[9] = { 0, 1, 0, 1, 1, 1, 0, 1, 0 };
		mMorphKernel = Image(3, 3, CV_8U, kernelList);
		/* end */
	}

	/// <summary>
	/// 背景差分, 移動物体検出
	/// </summary>
	void SubtractBackImage();

	/// <summary>
	/// 車影抽出
	/// </summary>
	void ExtractShadow();

	/// <summary>
	/// 車影再抽出
	/// </summary>
	/// <param name="areaThr">面積の閾値</param>
	/// <param name="aspectThr">アスペクト比の閾値</param>
	void ReExtractShadow(const int& areaThr = 5, const float& aspectThr = 1.6);

	/// <summary>
	/// 車両抽出
	/// </summary>
	void ExtractCars();

	/// <summary>
	/// 車両を矩形で囲む
	/// </summary>
	/// <param name="areaThr">面積の閾値</param>
	void DrawRectangle(const int& areaThr = 30);

	/// <summary>
	/// 出力画像一括保存
	/// </summary>
	/// <param name="pathList">パスの集合, 処理順にパス名を保存しておくこと</param>
	void WriteOutImgs(std::vector<std::string> pathList);

	/// <summary>
	/// 出力画像順次表示
	/// </summary>
	/// <param name="interval">待機時間[ms]</param>
	void ShowOutImgs(const int& interval = 1500);

	/// <summary>
	/// 車両検出画像の取得
	/// </summary>
	/// <returns>車両検出画像</returns>
	const Image& GetCarsRect() const { return mCarRects; }

private:
	CarsDetector(const CarsDetector& other) = delete;
};
