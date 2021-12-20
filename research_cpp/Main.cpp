#include "process/CarsExtractor.h"
#include "process/CarsTracer.h"

using namespace ImgProc;

/* ループ回数決定 */
constexpr static auto gStartCount = 1;
//constexpr static auto gStartCount = 1300;

//constexpr static auto gEndCount = 1;
//constexpr static auto gEndCount = 2000;
//constexpr static auto gEndCount = 1;
constexpr static auto gEndCount = 100000;
/* end */

/// <summary>
/// 実行用関数, メインループ, 実行時間計測, 結果出力
/// </summary>
/// <returns>テンプレ</returns>
int main()
{
	std::ios::sync_with_stdio(false); // デバッグ出力高速化

	/* リソース読み込み */
	ImgProcToolkit::SetVideoType(VideoType::YU); // リソース指定

	auto isCreatedVideo = ImgProcToolkit::CreateVideoResource(); // リソース登録
	if (!isCreatedVideo)
		return 0;

	auto isCreatedImages = ImgProcToolkit::CreateImageResource(); // リソース登録
	if (!isCreatedImages)
		return 0;

	/* 車線方向の設定 */
	std::unordered_map<size_t, RoadDirect> directions;
	directions[0] = RoadDirect::LEAVE;
	directions[2] = RoadDirect::LEAVE;
	directions[1] = RoadDirect::APPROACH;
	directions[3] = RoadDirect::APPROACH;
	ImgProcToolkit::SetRoadCarsDirections(std::move(directions));
	/* end */

	/* 検出範囲の設定 */
	ImgProcToolkit::SetDetectTop(400);
	ImgProcToolkit::SetDetectBottom(700);
	ImgProcToolkit::SetDetectMergin(16);
	ImgProcToolkit::SetDetectMerginPad(24);
	ImgProcToolkit::SetDetectedNearOffset(16);
	/* end */

	//ImgProcToolkit::ShowResourceImgs(1500); // リソース表示

	auto& frame = ImgProcToolkit::GetFrame();
	auto& videoCapture = ImgProcToolkit::GetVideoCapture();
	auto& videoWriter = ImgProcToolkit::GetVideoWriter();
	/* end */

	/* 処理実行変数 */
	ImgProc::CarsExtractor extractor;
	ImgProc::CarsTracer tracer;
	/* end */

	// 背景画像初期化
	extractor.InitBackgroundImage();

	// フレームカウント
	uint64_t& count = ImgProcToolkit::GetFrameCount();
	// 1秒あたりのフレーム数
	double tick = cv::getTickFrequency();
	// 実行結果画像
	auto& result = ImgProcToolkit::GetResult();
	//auto& result = ImgProcToolkit::GetCars();

	/* 検出台数監視 */
	auto& carsNum = ImgProcToolkit::GetCarsNum();
	auto& carsFrameNum = ImgProcToolkit::GetFrameCarsNum();
	auto& carsNumPrev = ImgProcToolkit::GetCarsNumPrev();
	/* end */

	ImgProcToolkit::SetStartFrame(gStartCount);
	/* ビデオ読み込みループ */
	while (true)
	{
		// 実行時間計測開始
		auto startTime = cv::getTickCount();

		count++;
		videoCapture >> frame; // ビデオフレーム読み込み
		if (frame.empty())
			break;

		extractor.SubtractBackImage();

		if (count < gStartCount)
			continue;

		if (count > gEndCount)
			break;

		/* 車両検出の準備 */
		extractor.ExtractShadow();
		extractor.ReExtractShadow(20, 1.8f);
		extractor.ExtractCars();
		/* end */

		/* 車両検出・車両追跡 */
		tracer.DetectCars();
		/* end */

		videoWriter << result; // ビデオ書き出し
		std::cout << count << std::endl; // カウントアップ
		//std::string path = "./frame_" + std::to_string(count) + ".png";
		//cv::imwrite(path, result);

		/* 結果表示 */
		//cv::imshow("result2", ImgProcToolkit::GetCars());
		//cv::imshow("result2", extractor.GetSubtracted());
		//cv::imshow("result", result);
		//cv::waitKey(1);
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