#include "process/CarsDetector.h"
#include "process/CarsTracer.h"

using namespace ImgProc;

/* ループ回数決定 */
constexpr static auto gStartCount = 1;
//constexpr static auto gEndCount = 1;
constexpr static auto gEndCount = 60;
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
	int count = 0;
	// 1秒あたりのフレーム数
	double tick = cv::getTickFrequency();
	// 実行結果画像
	auto& result = detector.GetCarsRect();

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

		/* 車両検出 */
		detector.SubtractBackImage();
		detector.ExtractShadow();
		detector.ReExtractShadow(5, 1.6f);
		detector.ExtractCars();
		detector.DrawRectangle(30);
		/* end */

		videoWriter << result; // ビデオ書き出し
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