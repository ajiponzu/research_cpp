#include "ImgProc.h"
#include "CarsExtractor.h"
#include "CarsTracer.h"

namespace ImgProc
{
	/* ImgProcToolkit */

	/* static変数再宣言 */
	// 入力ビデオキャプチャ
	cv::VideoCapture ImgProcToolkit::sVideoCapture;
	// ビデオレコーダー
	cv::VideoWriter ImgProcToolkit::sVideoWriter;
	// 入力ビデオの横幅
	int ImgProcToolkit::sVideoWidth = 0;
	// 入力ビデオの縦幅
	int ImgProcToolkit::sVideoHeight = 0;
	// 入力フレーム
	Image ImgProcToolkit::sFrame;
	// 結果画像
	Image ImgProcToolkit::sResultImg;
	// 車両二値画像
	Image ImgProcToolkit::sCarsImg;
	// 背景画像
	Image ImgProcToolkit::sBackImg;
	// 道路マスク画像
	Image ImgProcToolkit::sRoadMaskGray;
	// 道路マスク画像(テンプレートマッチング)
	std::vector<Image> ImgProcToolkit::sRoadMasksGray;
	/* end */

	/* テンプレート処理に用いる変数 */
	// 抽出したテンプレートを保存, 車線ごとに保存
	std::vector<std::unordered_map<uint64_t, Image>> ImgProcToolkit::sTemplatesList;
	// テンプレートの抽出位置を保存, 車線ごとに保存
	std::vector<std::unordered_map<uint64_t, cv::Rect2d>> ImgProcToolkit::sTemplatePositionsList;
	// 車線ごとの車の移動方向を保存
	std::unordered_map<size_t, RoadDirect> ImgProcToolkit::sRoadCarsDirections;
	// 車線ごとに検出境界に最も近い(高さの大小)車両IDを保存
	std::vector<std::unordered_set<uint64_t>> ImgProcToolkit::sBoundaryCarIdLists;
	/* end */

	// 道路数
	size_t ImgProcToolkit::sRoadMasksNum = 0;

	// 読み込んだフレーム数
	uint64_t ImgProcToolkit::sFrameCount = 0;
	// 初期フレーム
	uint64_t ImgProcToolkit::sStartFrame = 0;
	// 終了フレーム
	uint64_t ImgProcToolkit::sEndFrame = 0;
	// 検出・追跡中車両台数
	uint64_t ImgProcToolkit::sCarsNum = 0;
	// 全フレーム中の検出・追跡中車両台数(前フレームのもの)
	uint64_t ImgProcToolkit::sCarsNumPrev = 0;
	// 現在のフレーム中の車両台数
	uint64_t ImgProcToolkit::sFrameCarsNum = 0;
	// 検出車両のうち, もっとも最初に検出した車両のID
	uint64_t ImgProcToolkit::sFrontCarId = 0;
	/* end */

	/* パラメータ構造体初期化 */
	DetectAreaInf ImgProcToolkit::sDetectAreaInf{};
	ExtractorParams ImgProcToolkit::sExtractorParams{};
	TracerParams ImgProcToolkit::sTracerParams{};
	TemplateHandleParams ImgProcToolkit::sTemplateHandleParams{};
	BackImgHandleParams ImgProcToolkit::sBackImgHandleParams{};
	/* end */

	/// <summary>
	/// ビデオリソース読み込み・書き出し設定
	/// </summary>
	/// <param name="inputPath">入力ビデオパス</param>
	/// <param name="outputPath">出力パス</param>
	void ImgProcToolkit::CreateVideoResource(const std::string& inputPath, const std::string& outputPath)
	{
		sVideoCapture.open(inputPath);
		if (!sVideoCapture.isOpened())
		{
			std::cout << inputPath << ": doesn't exist" << std::endl;
			assert("failed to read video");
		}

		auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
		sVideoWidth = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
		sVideoHeight = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
		auto videoFps = sVideoCapture.get(cv::CAP_PROP_FPS);

		sVideoWriter.open(outputPath, fourcc, videoFps, cv::Size(sVideoWidth, sVideoHeight));
		if (!sVideoWriter.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}
	}

	/// <summary>
	/// マスク画像等読み込み
	/// </summary>
	/// <param name="roadMaskPath">マスク画像（全体）パス</param>
	/// <param name="roadMasksBasePath">道路マスク画像ベースパス</param>
	void ImgProcToolkit::CreateImageResource(const std::string& roadMaskPath, const std::string& roadMasksBasePath)
	{
		sRoadMaskGray = cv::imread(roadMaskPath);
		if (sRoadMaskGray.empty())
		{
			std::cout << roadMaskPath << ": can't read this." << std::endl;
			assert("failed to read roadMask");
		}
		binarizeImage(sRoadMaskGray);

		size_t idx = 0;
		while (true)
		{
			const auto filePath = roadMasksBasePath + std::to_string(idx) + ".png";
			auto mask = cv::imread(filePath);
			if (mask.empty())
			{
				if (idx == 0)
				{
					std::cout << filePath << ": can't read this." << std::endl;
					assert("failed to read some roadMasks");
				}
				break;
			}
			binarizeImage(mask);
			sRoadMasksGray.push_back(mask.clone());
			idx++;
		}
		sRoadMasksNum = idx;
		sBoundaryCarIdLists.resize(idx);
		sTemplatesList.resize(idx);
		sTemplatePositionsList.resize(idx);
	}

	/// <summary>
	/// 車線ごとの車の移動方向を設定
	/// </summary>
	/// <param name="directions">"L"か"R"が格納された配列</param>
	void ImgProcToolkit::SetRoadCarsDirections(const std::vector<std::string>& directions)
	{
		size_t idx = 0;
		RoadDirect directTemp{};
		for (const auto& direct : directions)
		{
			if (direct == "L")
				directTemp = RoadDirect::LEAVE;
			else
				directTemp = RoadDirect::APPROACH;

			sRoadCarsDirections[idx] = directTemp;
			idx++;
		}
	}

	/// <summary>
	/// リソース画像表示
	/// </summary>
	/// <param name="interval">待機時間[ms]</param>
	void ImgProcToolkit::ShowResourceImgs(const int& interval)
	{
		cv::imshow("", sBackImg);
		cv::waitKey(interval);
		cv::imshow("", sRoadMaskGray);
		cv::waitKey(interval);
		for (auto itr = sRoadMasksGray.begin(); itr != sRoadMasksGray.end(); itr++)
		{
			cv::imshow("", *itr);
			cv::waitKey(interval);
		}
	}

	/// <summary>
	/// リソース読み込み
	/// </summary>
	void ImgProcToolkit::SetResourcesAndParams()
	{
		cv::FileStorage fstorage("./execute.json", 0); // json読み込み
		const auto testCaseNum = static_cast<int>(fstorage["executeCaseNum"]); // 実行テストケース番号
		const auto root = fstorage["TestCases"][testCaseNum]; // テストケースパラメータハッシュ

		/* 処理フレーム指定 */
		sStartFrame = static_cast<uint64_t>(root["startFrame"].real());
		sEndFrame = static_cast<uint64_t>(root["endFrame"].real());
		/* end */
		
		/* リソース指定 */
		const auto resources = root["Resources"];

		const auto inputPath = resources["video"].string();
		const auto outputPath = resources["result"].string();
		const auto roadMaskPath = resources["mask"].string();
		const auto roadMasksBasePath = resources["roadMasksBase"].string();

		std::vector<std::string> directions{};
		const auto roadDirections = resources["roadDirections"];
		for (int i = 0; i < roadDirections.size(); i++)
			directions.push_back(roadDirections[i].string());

		CreateVideoResource(inputPath, outputPath);
		CreateImageResource(roadMaskPath, roadMasksBasePath);
		SetRoadCarsDirections(directions);
		/* end */

		/* パラメータ指定 */
		/* その1 */
		const auto detectAreaInf = root["DetectAreaInf"];
		sDetectAreaInf.top = static_cast<int>(detectAreaInf["top"].real());
		sDetectAreaInf.bottom = static_cast<int>(detectAreaInf["bottom"].real());
		sDetectAreaInf.mergin = static_cast<int>(detectAreaInf["mergin"].real());
		sDetectAreaInf.merginPad = static_cast<int>(detectAreaInf["merginPad"].real());
		sDetectAreaInf.nearOffset = static_cast<int>(detectAreaInf["nearOffset"].real());
		/* end */

		/* その2 */
		const auto extractorParams = root["ExtractorParams"];
		sExtractorParams.shadowThrL = static_cast<int>(extractorParams["shadowThrL"].real());
		sExtractorParams.shadowThrB = static_cast<int>(extractorParams["shadowThrB"].real());
		sExtractorParams.closeCount = static_cast<int>(extractorParams["closeCount"].real());
		sExtractorParams.kernelSize = static_cast<int>(extractorParams["kernelSize"].real());
		sExtractorParams.reshadowAreaThr = static_cast<int>(extractorParams["reshadowAreaThr"].real());
		sExtractorParams.reshadowAspectThr = static_cast<float>(extractorParams["reshadowAspectThr"].real());
		/* end */

		/* その3 */
		const auto tracerParams = root["TracerParams"];
		sTracerParams.minAreaRatio = tracerParams["minAreaRatio"].real();
		sTracerParams.detectAreaThr = static_cast<int>(tracerParams["detectAreaThr"].real());
		sTracerParams.minMatchingThr = tracerParams["minMatchingThr"].real();
		/* end */

		/* その4 */
		const auto templateHandleParams = root["TemplateHandleParams"];
		sTemplateHandleParams.mergin = static_cast<int>(templateHandleParams["mergin"].real());
		sTemplateHandleParams.magni = templateHandleParams["magni"].real();
		sTemplateHandleParams.kernelSize = static_cast<int>(templateHandleParams["kernelSize"].real());
		sTemplateHandleParams.closeCount = static_cast<int>(templateHandleParams["closeCount"].real());
		sTemplateHandleParams.minAreaRatio = templateHandleParams["minAreaRatio"].real();
		sTemplateHandleParams.areaThr = static_cast<int>(templateHandleParams["areaThr"].real());
		sTemplateHandleParams.nlDenoising = (templateHandleParams["nlDenoising"].string() == "on");
		/* end */

		/* その5 */
		const auto backImgHandleParams = root["BackImgHandleParams"];
		sBackImgHandleParams.blendAlpha = backImgHandleParams["blendAlpha"].real();
		/* end */
		/* end */
	}

	/// <summary>
	/// リソース確認
	/// </summary>
	void ImgProcToolkit::ShowResourcesAndParams()
	{
		std::cout << sStartFrame << std::endl;
		std::cout << sEndFrame << std::endl;

		//cv::imshow("", sRoadMaskGray);
		//cv::waitKey(1000);
	}

	/// <summary>
	/// 処理実行
	/// </summary>
	void ImgProcToolkit::RunImageProcedure()
	{
		std::ios::sync_with_stdio(false); // デバッグ出力高速化

		double tick = cv::getTickFrequency(); // 1秒あたりのフレーム数

		CarsExtractor extractor; // 抽出器
		CarsTracer tracer; // 検出器

		extractor.InitBackgroundImage();

		while (true)
		{
			// 実行時間計測開始
			auto startTime = cv::getTickCount();

			/* ビデオフレーム読み込み */
			sFrameCount++;
			sVideoCapture >> sFrame;
			if (sFrame.empty())
				break;
			/* end */

			if (sFrameCount < sStartFrame)
				continue;
			else if (sFrameCount > sEndFrame)
				break;

			/* メイン処理 */
			extractor.ExtractCars(); // 車両抽出
			tracer.DetectCars(); // 車両検出・追跡
			/* end */

			/* 結果出力・実行時間計測 */
			sVideoWriter << sResultImg;
			std::cout << sFrameCount << std::endl;
			auto endTime = cv::getTickCount();
			std::cout << (double)(endTime - startTime) / tick << std::endl;
			/* end */
		}
	}

	/* ImgProcToolkit外 */
	/// <summary>
	/// 画像の二値化
	/// </summary>
	/// <param name="inputImg">二値化画像, 1チャンネル</param>
	void binarizeImage(Image& inputImg)
	{
		if (inputImg.channels() == 3)
			cv::cvtColor(inputImg, inputImg, cv::COLOR_BGR2GRAY);
		cv::threshold(inputImg, inputImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	}

	/// <summary>
	///	画像の部分参照
	/// </summary>
	/// <param name="inputImg">参照元画像</param>
	/// <param name="x">参照範囲矩形の左上座標のx成分</param>
	/// <param name="y">参照範囲矩形の左上座標のy成分</param>
	/// <param name="width">参照範囲矩形の横幅</param>
	/// <param name="height">参照範囲矩形の縦幅</param>
	/// <returns>参照範囲の部分画像(</returns>
	Image GetImgSlice(const Image& inputImg, const int& x, const int& y, const int& width, const int& height)
	{
		return inputImg(cv::Rect(x, y, width + 1, height + 1)); // 参照カウントのため, シャローコピーでもデータが解放されない.
	}

	/// <summary>
	///	画像の部分参照
	/// </summary>
	/// <param name="inputImg">参照元画像</param>
	/// <param name="x">参照範囲矩形の左上座標のx成分</param>
	/// <param name="y">参照範囲矩形の左上座標のy成分</param>
	/// <param name="width">参照範囲矩形の横幅</param>
	/// <param name="height">参照範囲矩形の縦幅</param>
	/// <returns>参照範囲の部分画像(</returns>
	Image GetImgSlice(const Image& inputImg, const double& x, const double& y, const double& width, const double& height)
	{
		return inputImg(cv::Rect(static_cast<int>(x), static_cast<int>(y), static_cast<int>(width), static_cast<int>(height)));
	}

	/// <summary>
	///	画像の部分参照
	/// </summary>
	/// <param name="inputImg">参照元画像</param>
	/// <param name="x">参照範囲矩形</param>
	/// <returns>参照範囲の部分画像(</returns>
	Image GetImgSlice(const Image& inputImg, const cv::Rect2d& rect)
	{
		return inputImg(rect);
	}

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="x">抽出範囲矩形の左上座標のx成分</param>
	/// <param name="y">抽出範囲矩形の左上座標のy成分</param>
	/// <param name="width">抽出範囲矩形の横幅</param>
	/// <param name="height">抽出範囲矩形の縦幅</param>
	/// <returns>指定範囲の抽出画像(クローン後にムーブ)</returns>
	Image ExtractTemplate(const Image& inputImg, const int& x, const int& y, const int& width, const int& height)
	{
		return inputImg(cv::Rect(x, y, width + 1, height + 1)).clone();
	}

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="x">抽出範囲矩形の左上座標のx成分</param>
	/// <param name="y">抽出範囲矩形の左上座標のy成分</param>
	/// <param name="width">抽出範囲矩形の横幅</param>
	/// <param name="height">抽出範囲矩形の縦幅</param>
	/// <returns>指定範囲の抽出画像(クローン後にムーブ)</returns>
	Image ExtractTemplate(const Image& inputImg, const double& x, const double& y, const double& width, const double& height)
	{
		return inputImg(cv::Rect(static_cast<int>(x), static_cast<int>(y), static_cast<int>(width), static_cast<int>(height))).clone();
	}

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="rect">抽出範囲矩形</param>
	/// <returns>指定範囲の抽出画像(クローン後)</returns>
	Image ExtractTemplate(const Image& inputImg, const cv::Rect2d& rect)
	{
		return inputImg(rect).clone();
	}
	/* end */
};