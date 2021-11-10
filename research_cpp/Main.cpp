#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "process/CarsDetector.h"

using Image = ImgProc::Image;

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
		std::cout << gOutputPathList[gVideoType] << ": can't create or overwrite" << std::endl;
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
bool CreateImageResource(Image& backImg, Image& roadMask, std::vector<Image>& roadMasks)
{
	backImg = cv::imread(gBackImgPathList[gVideoType]);
	if (backImg.empty())
	{
		std::cout << gBackImgPathList[gVideoType] << ": can't read this." << std::endl;
		return false;
	}

	roadMask = cv::imread(gRoadMaskPath);
	if (roadMask.empty())
	{
		std::cout << gRoadMaskPath << ": can't read this." << std::endl;
		return false;
	}
	binarizeMask(roadMask);

	for (size_t idx = 0; idx < gRoadMasksNum; idx++)
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

	size_t idx = 0;
	for (auto& mask : roadMasks)
	{
		const auto filePath = gRoadMasksBasePath + std::to_string(idx) + ".png";
		mask = cv::imread(filePath);
		if (roadMasks[idx].empty())
		{
			std::cout << filePath << ": can't read this." << std::endl;
			return false;
		}
		binarizeMask(mask);
		idx++;
	}

	return true;
}

int main()
{
	std::ios::sync_with_stdio(false); // デバッグ出力高速化

	/* リソース読み込み */

	// 入力ビデオキャプチャ
	cv::VideoCapture videoCapture;
	// ビデオレコーダー
	cv::VideoWriter videoWriter;
	auto isCreatedVideo = CreateVideoResource(videoCapture, videoWriter); // リソース登録
	if (!isCreatedVideo)
		return 0;

	// 背景画像
	Image backImg;
	// 道路マスク画像
	Image roadMask;
	// 道路マスク画像(テンプレートマッチング)
	std::vector<Image> roadMasks(gRoadMasksNum);
	auto isCreatedImages = CreateImageResource(backImg, roadMask, roadMasks); // リソース登録
	if (!isCreatedImages)
		return 0;

	/* リソース読み込みデバッグ */
	//cv::imshow("", backImg);
	//cv::waitKey(1500);
	//cv::imshow("", roadMask);
	//cv::waitKey(1500);
	//for (auto itr = roadMasks.begin(); itr != roadMasks.end(); itr++)
	//{
	//	cv::imshow("", *itr);
	//	cv::waitKey(1500);
	//}
	/* end */

	/* end */

	// フレームカウント
	int count = 0;
	// フレーム
	Image frame;

	/* 処理実行変数 */
	ImgProc::CarsDetector detector(backImg, roadMask, roadMasks);
	/* end */

	// 1秒あたりのフレーム数
	double tick = cv::getTickFrequency();

	/* ビデオ読み込みループ */
	while (true)
	{
		// 実行時間計測開始
		auto startTime = cv::getTickCount();

		count++;
		videoCapture >> frame; // ビデオフレーム読み込み
		if (frame.empty())
			break;

		if (count < startCount)
			continue;

		if (count > endCount)
			break;

		/* 画像処理 */

		/* 車両検出 */
		detector.SubtractBackImage(frame);
		detector.ExtractShadow(frame);
		detector.ReExtractShadow(5, 1.6f);
		detector.ExtractCars();
		detector.DrawRectangle(frame, 30);
		/* end */

		/* end */

		//videoWriter << frame; // ビデオ書き出し
		videoWriter << detector.GetCarsRect(); // ビデオ書き出し
		std::cout << count << std::endl; // カウントアップ

		/* デバッグ */
		//detector.ShowOutImgs(1000);
		/* end */

		/* 計測時間表示 */
		auto endTime = cv::getTickCount();
		auto fps = (endTime - startTime) / tick;
		std::cout << fps << "[s]" << std::endl;
		/* end */
	}
	/* end */

	return 0;
}