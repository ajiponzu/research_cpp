#include <iostream>
#include <string>
#include <array>
#include <opencv2/opencv.hpp>

#include "process/CarsDetector.h"

using Image = cv::Mat;

enum VideoType
{
	HARE = 0,
	KUMORI,
	AME,
};

/* ファイルパス関連 */
const static std::string gVideoPathList[3] = { "./resource/hare/input.mp4", "./resource/kumori/input.mp4", "./resource/ame/input.mp4" };
const static std::string gOutputPathList[3] = { "./output/hare/output.mp4", "./output/kumori/output.mp4", "./output/ame/output.mp4" };
const static VideoType gVideoType = VideoType::HARE;
const static std::string gBackImgPathList[3] = { "./resource/hare/back.png", "./resource/kumori/back.mp4", "./resource/ame/back.mp4" };
const static std::string gRoadMaskPath = "./resource/back_kaikai.png";
const static std::string gRoadMasksBasePath = "./resource/back_kai";
constexpr static auto gRoadMasksNum = 4;
/* end */

/* ループ回数決定 */
constexpr static auto startCount = 1;
constexpr static auto endCount = 10000;
/* end */

/// <summary>
/// ビデオリソース読み込み・初期設定
/// </summary>
/// <param name="videoCapture"></param>
/// <param name="videoWriter"></param>
/// <returns></returns>
bool CreateVideoResource(cv::VideoCapture& videoCapture, cv::VideoWriter& videoWriter)
{
	videoCapture.open(gVideoPathList[gVideoType]);
	if (!videoCapture.isOpened())
	{
		std::cout << gVideoPathList[gVideoType] << ": doesn't exist" << std::endl;
		return false;
	}

	auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
	auto videoWidth = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
	auto videoHeight = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
	auto videoFps = videoCapture.get(cv::CAP_PROP_FPS);
	videoWriter.open(gOutputPathList[gVideoType], fourcc, videoFps, cv::Size(videoWidth, videoHeight));
	if (!videoWriter.isOpened())
	{
		std::cout << gVideoPathList[gVideoType] << ": can't create or overwrite" << std::endl;
		return false;
	}

	return true;
}

/// <summary>
/// 二値化されているか怪しいマスク画像を二値化
/// </summary>
/// <param name="inputImg">マスク画像</param>
void binarizeMask(Image& inputImg)
{
	cv::cvtColor(inputImg, inputImg, cv::COLOR_BGR2GRAY);
	cv::threshold(inputImg, inputImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	cv::cvtColor(inputImg, inputImg, cv::COLOR_GRAY2BGR);
}

/// <summary>
/// 背景画像・道路マスク画像等, 画像リソース読み込み
/// </summary>
/// <param name="backImg"></param>
/// <param name="roadMask"></param>
/// <param name="roadMasks"></param>
/// <returns></returns>
template<size_t N> bool CreateImageResource(Image& backImg, Image& roadMask, std::array<Image, N>& roadMasks)
{
	backImg = cv::imread(gBackImgPathList[gVideoType]);
	if (backImg.empty())
	{
		std::cout << gBackImgPathList[gVideoType] << ": can't read this." << std::endl;
		return false;
	}
	binarizeMask(backImg);

	roadMask = cv::imread(gRoadMaskPath);
	if (roadMask.empty())
	{
		std::cout << gRoadMaskPath << ": can't read this." << std::endl;
		return false;
	}
	binarizeMask(roadMask);

	for (size_t idx = 0; idx < N; idx++)
	{
		const auto filePath = gRoadMasksBasePath + std::to_string(idx) + ".png";
		roadMasks[idx] = cv::imread(filePath);
		if (roadMasks[idx].empty())
		{
			std::cout << filePath << ": can't read this." << std::endl;
			return false;
		}
		binarizeMask(roadMasks[idx]);
	}

	return true;
}

int main()
{
	// デバッグ出力高速化
	std::ios::sync_with_stdio(false);

	/* リソース読み込み */

	// 入力ビデオキャプチャ
	cv::VideoCapture videoCapture;
	// ビデオレコーダー
	cv::VideoWriter videoWriter;
	// リソース登録
	auto isCreatedVideo = CreateVideoResource(videoCapture, videoWriter);
	if (!isCreatedVideo)
		return 0;

	// 背景画像
	Image backImg;
	// 道路マスク画像
	Image roadMask;
	// 道路マスク画像(テンプレートマッチング)
	std::array<Image, gRoadMasksNum> roadMasks;
	// リソース登録
	auto isCreatedImages = CreateImageResource(backImg, roadMask, roadMasks);
	if (!isCreatedImages)
		return 0;

	///* リソース読み込みデバッグ */
	//cv::imshow("", backImg);
	//cv::waitKey(1500);
	//cv::imshow("", roadMask);
	//cv::waitKey(1500);
	//for (auto itr = roadMasks.begin(); itr != roadMasks.end(); itr++)
	//{
	//	cv::imshow("", *itr);
	//	cv::waitKey(1500);
	//}
	///* end */

	/* end */

	// フレームカウント
	int count = 0;
	// フレーム
	Image frame;

	/* 実行時間計測変数 */
	double f = 1000.0 / cv::getTickFrequency();
	uint64_t time = cv::getTickCount();
	/* end */

	// ビデオ読み込みループ
	while (true)
	{
		count++;
		// ビデオフレーム読み込み
		videoCapture >> frame;
		if (frame.empty())
			break;

		if (count < startCount)
			continue;

		if (count > endCount)
			break;

		/* 画像処理 */

		// 背景差分画像
		auto subtracted = CarsDetector::SubtractImage(frame, backImg, roadMask);
		//cv::imshow("", subtracted);
		//cv::waitKey(2000);

		//// 車影抽出
		auto shadow = CarsDetector::ExtractShadow(frame, roadMask);
		//cv::imshow("", shadow);
		//cv::waitKey(2000);

		// 車影再抽出
		auto reshadow = CarsDetector::ReExtractShadow(shadow, 5, 1.6);
		//cv::imshow("", reshadow);
		//cv::waitKey(20000);

		/* end */

		// ビデオ書き出し
		videoWriter << frame;

		std::cout << count << std::endl;
	}

	/* 計測時間表示 */
	auto ms = (cv::getTickCount() - time) * f;
	auto s = ms / 1000;
	auto m = s / 60;
	std::cout << ms << " [ms]" << std::endl;
	std::cout << s << " [s]" << std::endl;
	std::cout << m << " [m]" << std::endl;
	std::cout << count / s << "[fps]" << std::endl;
	/* end */

	return 0;
}