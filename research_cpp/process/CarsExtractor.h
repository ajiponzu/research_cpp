#pragma once
#include "ImgProc.h"

class ImgProc::CarsExtractor
{
private:
	class BackImageHandle;

	/* 出力画像バッファ */
	Image mSubtracted; //背景差分画像, 1チャンネル固定
	Image mShadow; //車影画像, 1チャンネル固定
	Image mReShadow;//車影再抽出画像, 1チャンネル固定
	/* end */

	/* 画像処理に用いるバッファ */
	Image mTemp; //バッファ
	Image mLab128; //グレースケール化のために, L*a*b*のa値とb値を128にするためのバッファ, チャンネル数1
	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群
	Image mExceptedShadows; //除外すべき影画像
	Image mCloseKernel; // クロージングで使用するカーネル
	Image mOpenKernel; // クロージングで使用するカーネル
	/* end */

	int mCloseCount = 3; // クロージング回数

public:
	CarsExtractor()
	{
		cv::Size imgSize(ImgProcToolkit::sVideoWidth, ImgProcToolkit::sVideoHeight);

		/* メンバ画像の初期化 */
		mExceptedShadows = Image::zeros(imgSize, CV_8U);
		mLab128 = Image::ones(imgSize, CV_8U) * 128;
		/* end */

		/* モルフォロジカーネルの初期化 */
		mCloseKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // モルフォロジカーネル取得関数, RECTのほかにCROSS, ELIPSEがある
		/* end */
	}

	/// <summary>
	/// 初期背景画像を作成(500フレーム使用)
	/// </summary>
	void InitBackgroundImage();

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
	/// 出力画像順次表示
	/// </summary>
	/// <param name="interval">待機時間[ms]</param>
	void ShowOutImgs(const int& interval = 1500);

	const Image& GetSubtracted() const { return mSubtracted; }
	const Image& GetShadow() const { return mShadow; }
	const Image& GetReShadow() const { return mReShadow; }

private:
	CarsExtractor(const CarsExtractor& other) = delete;
};
