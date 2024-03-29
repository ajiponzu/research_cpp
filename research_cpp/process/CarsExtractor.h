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
	Image mPreCars; // モルフォロジかけない車両抽出画像, 1チャンネル固定
	/* end */

	cv::VideoWriter mVideoWriterSub;
	cv::VideoWriter mVideoWriterShadow;
	cv::VideoWriter mVideoWriterReShadow;
	cv::VideoWriter mVideoWriterCars;
	cv::VideoWriter mVideoWriterPreCars;

	/* 画像処理に用いるバッファ */
	Image mTemp; //バッファ
	Image mLab128; //グレースケール化のために, L*a*b*のa値とb値を128にするためのバッファ, チャンネル数1
	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群
	Image mCloseKernel; // クロージングで使用するカーネル
	Image mOpenKernel; // クロージングで使用するカーネル
	/* end */

public:
	CarsExtractor()
	{
		const auto& [crefVideoWidth, crefVideoHeight] = ImgProcToolkit::GetVideoWidAndHigh();
		cv::Size imgSize(crefVideoWidth, crefVideoHeight);

		/* メンバ画像の初期化 */
		mLab128 = Image::ones(imgSize, CV_8U) * 128;
		/* end */

		/* モルフォロジカーネルの初期化 */
		const auto& kernelSize = ImgProcToolkit::GetExtractorParams().kernelSize;
		mCloseKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize)); // モルフォロジカーネル取得関数, RECTのほかにCROSS, ELIPSEがある
		/* end */

		auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
		auto videoFps = ImgProcToolkit::GetVideoCapture().get(cv::CAP_PROP_FPS);

		std::string outputPath = ImgProcToolkit::GetOutputBasePath() + "_Sub.mp4";
		mVideoWriterSub.open(outputPath, fourcc, videoFps, cv::Size(crefVideoWidth, crefVideoHeight));
		if (!mVideoWriterSub.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}

		outputPath = ImgProcToolkit::GetOutputBasePath() + "_Shadow.mp4";
		mVideoWriterShadow.open(outputPath, fourcc, videoFps, cv::Size(crefVideoWidth, crefVideoHeight));
		if (!mVideoWriterShadow.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}

		outputPath = ImgProcToolkit::GetOutputBasePath() + "_ReShadow.mp4";
		mVideoWriterReShadow.open(outputPath, fourcc, videoFps, cv::Size(crefVideoWidth, crefVideoHeight));
		if (!mVideoWriterReShadow.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}

		outputPath = ImgProcToolkit::GetOutputBasePath() + "_PreCars.mp4";
		mVideoWriterPreCars.open(outputPath, fourcc, videoFps, cv::Size(crefVideoWidth, crefVideoHeight));
		if (!mVideoWriterPreCars.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}

		outputPath = ImgProcToolkit::GetOutputBasePath() + "_Cars.mp4";
		mVideoWriterCars.open(outputPath, fourcc, videoFps, cv::Size(crefVideoWidth, crefVideoHeight));
		if (!mVideoWriterCars.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}
	}

	const Image& GetSubtracted() const { return mSubtracted; }
	const Image& GetShadow() const { return mShadow; }
	const Image& GetReShadow() const { return mReShadow; }

	/// <summary>
	/// 初期背景画像を作成(500フレーム使用)
	/// </summary>
	void InitBackgroundImage();

	/// <summary>
	/// 車両抽出
	/// </summary>
	void ExtractCars();

private:
	CarsExtractor(const CarsExtractor& other) = delete;

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
	void ReExtractShadow();

	/// <summary>
	/// 出力画像順次表示
	/// </summary>
	/// <param name="interval">待機時間[ms]</param>
	void ShowOutImgs(const int& interval = 1500);

	/// <summary>
	/// 各処理過程の結果画像を出力
	/// </summary>
	void OutputProcessVideo();
};
