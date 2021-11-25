#include "process/CarsDetector.h"
#include "process/CarsTracer.h"

using namespace ImgProc;

/* ループ回数決定 */
//constexpr static auto gStartCount = 1;
constexpr static auto gStartCount = 338;

//constexpr static auto gEndCount = 1;
//constexpr static auto gEndCount = 60;
constexpr static auto gEndCount = 345;
//constexpr static auto gEndCount = 10000;
/* end */

/// <summary>
/// 実行用関数, メインループ, 実行時間計測, 結果出力
/// </summary>
/// <returns>テンプレ</returns>
int main()
{
	std::ios::sync_with_stdio(false); // デバッグ出力高速化

	/* リソース読み込み */
	ImgProcToolkit::SetVideoType(ImgProcToolkit::VIDEO_TYPE_HARE); // リソース指定

	auto isCreatedVideo = ImgProcToolkit::CreateVideoResource(); // リソース登録
	if (!isCreatedVideo)
		return 0;

	auto isCreatedImages = ImgProcToolkit::CreateImageResource(); // リソース登録
	if (!isCreatedImages)
		return 0;

	/* 車線方向の設定 */
	std::unordered_map<size_t, int> directions;
	directions[0] = ImgProcToolkit::CARS_LEAVE_ROAD;
	directions[2] = ImgProcToolkit::CARS_LEAVE_ROAD;
	directions[1] = ImgProcToolkit::CARS_APPROACH_ROAD;
	directions[3] = ImgProcToolkit::CARS_APPROACH_ROAD;
	ImgProcToolkit::SetRoadCarsDirections(std::move(directions));
	/* end */

	/* 検出範囲の設定 */
	ImgProcToolkit::SetDetectTop(235);
	ImgProcToolkit::SetDetectBottom(530);
	ImgProcToolkit::SetDetectMergin(0);
	/* end */

	//ImgProcToolkit::ShowResourceImgs(1500); // リソース表示

	auto& frame = ImgProcToolkit::GetFrame();
	auto& videoCapture = ImgProcToolkit::GetVideoCapture();
	auto& videoWriter = ImgProcToolkit::GetVideoWriter();
	/* end */

	/* 処理実行変数 */
	ImgProc::CarsDetector detector;
	ImgProc::CarsTracer tracer;
	/* end */

	// フレームカウント
	uint64_t& count = ImgProcToolkit::GetFrameCount();
	// 1秒あたりのフレーム数
	double tick = cv::getTickFrequency();
	// 実行結果画像
	auto& result = ImgProcToolkit::GetResult();

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

		if (count < gStartCount)
			continue;

		if (count > gEndCount)
			break;

		/* 車両検出の準備 */
		detector.SubtractBackImage();
		detector.ExtractShadow();
		detector.ReExtractShadow(5, 1.6f);
		detector.ExtractCars();
		/* end */

		/* 車両追跡 */
		tracer.FindCarsTemplates();
		/* end */

		/* 新規車両検出 */
		detector.DrawRectangle(30);
		/* end */

		//detector.ShowOutImgs(1000); // 出力結果表示
		/* end */

		//Image temp;
		//cv::cvtColor(detector.GetCars(), temp, cv::COLOR_GRAY2BGR);
		//videoWriter << temp;

		videoWriter << result; // ビデオ書き出し
		std::cout << count << std::endl; // カウントアップ
		std::string path = "./frame_" + std::to_string(count) + ".png";
		cv::imwrite(path, result);

		/* 結果表示 */
		cv::imshow("result", result);
		cv::waitKey(1);
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