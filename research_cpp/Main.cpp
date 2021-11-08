#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "process/CarsDetector.h"

using Image = cv::Mat;

int main()
{
	// デバッグ出力高速化
	std::ios::sync_with_stdio(false);

	const std::string videoPath = "./resource/img_1fps/input.mp4";

	// 入力ビデオキャプチャ
	cv::VideoCapture videoCapture;
	videoCapture.open(videoPath);
	if (!videoCapture.isOpened())
	{
		std::cout << videoPath << ": doesn't exist" << std::endl;
		return 0;
	}

	// ビデオレコーダー
	cv::VideoWriter videoWriter;
	auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
	auto videoWidth = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
	auto videoHeight = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
	auto videoFps = videoCapture.get(cv::CAP_PROP_FPS);
	videoWriter.open("output.mp4", fourcc, videoFps, cv::Size(videoWidth, videoHeight));

	// フレームカウント
	int count = 0;
	// フレーム
	Image frame;

	// ビデオ読み込みループ
	while (true)
	{
		count++;
		// ビデオフレーム読み込み
		videoCapture >> frame;
		if (frame.empty())
			break;

		/* 画像処理 */

		/* end */

		// ビデオ書き出し
		videoWriter << frame;

		std::cout << count << std::endl;
	}

	return 0;
}

