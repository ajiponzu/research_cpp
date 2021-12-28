#include "ImgProc.h"

namespace ImgProc
{
	/* グローバル変数 */
	/* ファイルパス関連 */
	static const std::string gsVideoPathList[4] = { "./resource/hare/input.mp4", "./resource/kumori/input.mp4", "./resource/ame/input.mp4", "./resource/yugata/input.mp4" };
	static const std::string gsBackImgPathList[4] = { "./resource/hare/back.png", "./resource/kumori/back.png", "./resource/ame/back.png", "./resource/yugata/back.png"};
	static const std::string gsOutputPathList[4] = { "./output/hare/output.mp4", "./output/kumori/output.mp4", "./output/ame/output.mp4", "./output/yugata/output.mp4" };
	static const std::string gsTemplatesPathList[4] = { "./output/hare/template/", "./output/kumori/template/", "./output/ame/template/", "./output/yugata/template/" };
	//static const std::string gsRoadMaskPath = "./resource/back_kaikai.png";
	static const std::string gsRoadMaskPath = "./resource/2k/back_kaikai.png";
	//static const std::string gsRoadMasksBasePath = "./resource/back_kai";
	static const std::string gsRoadMasksBasePath = "./resource/2k/back_kai";
	/* end */
	/* end */

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

	size_t ImgProcToolkit::sRoadMasksNum = 0;
	VideoType ImgProcToolkit::sVideoType = VideoType::HARE;

	// 読み込んだフレーム数
	uint64_t ImgProcToolkit::sFrameCount = 0;
	// 初期フレーム
	uint64_t ImgProcToolkit::sStartFrame = 0;
	// 検出・追跡中車両台数
	uint64_t ImgProcToolkit::sCarsNum = 0;
	// 全フレーム中の検出・追跡中車両台数(前フレームのもの)
	uint64_t ImgProcToolkit::sCarsNumPrev = 0;
	// 現在のフレーム中の車両台数
	uint64_t ImgProcToolkit::sFrameCarsNum = 0;
	// 検出車両のうち, もっとも最初に検出した車両のID
	uint64_t ImgProcToolkit::sFrontCarId = 0;

	/* 検出範囲指定 */
	int ImgProcToolkit::sDetectTop = 0;
	int ImgProcToolkit::sDetectBottom = 0;
	int ImgProcToolkit::sDetectMergin = 0;
	int ImgProcToolkit::sDetectMerginPad = 0;
	int ImgProcToolkit::sDetectedNearOffset = 0;
	/* end */

	/* end */

	/// <summary>
	/// ビデオリソース読み込み・初期設定
	/// </summary>
	/// <returns>処理成功の真偽</returns>
	bool ImgProcToolkit::CreateVideoResource()
	{
		sVideoCapture.open(gsVideoPathList[static_cast<int>(sVideoType)]);
		if (!sVideoCapture.isOpened())
		{
			std::cout << gsVideoPathList[static_cast<int>(sVideoType)] << ": doesn't exist" << std::endl;
			return false;
		}

		auto fourcc = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FOURCC));
		sVideoWidth = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
		sVideoHeight = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
		auto videoFps = sVideoCapture.get(cv::CAP_PROP_FPS);
		sVideoWriter.open(gsOutputPathList[static_cast<int>(sVideoType)], fourcc, videoFps, cv::Size(sVideoWidth, sVideoHeight));
		if (!sVideoWriter.isOpened())
		{
			std::cout << gsOutputPathList[static_cast<int>(sVideoType)] << ": can't create or overwrite" << std::endl;
			return false;
		}

		return true;
	}

	/// <summary>
	/// 背景画像・道路マスク画像等, 画像リソース読み込み
	/// </summary>
	/// <returns>処理成功の真偽</returns>
	bool ImgProcToolkit::CreateImageResource()
	{
		sBackImg = cv::imread(gsBackImgPathList[static_cast<int>(sVideoType)]);
		if (sBackImg.empty())
		{
			std::cout << gsBackImgPathList[static_cast<int>(sVideoType)] << ": can't read this." << std::endl;
			return false;
		}

		sRoadMaskGray = cv::imread(gsRoadMaskPath);
		if (sRoadMaskGray.empty())
		{
			std::cout << gsRoadMaskPath << ": can't read this." << std::endl;
			return false;
		}
		binarizeImage(sRoadMaskGray);

		size_t idx = 0;
		while (true)
		{
			const auto filePath = gsRoadMasksBasePath + std::to_string(idx) + ".png";
			auto mask = cv::imread(filePath);
			if (mask.empty())
			{
				if (idx == 0)
				{
					std::cout << filePath << ": can't read this." << std::endl;
					return false;
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

		return true;
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
	/// 車線ごとの車の移動方向を設定
	/// </summary>
	/// <param name="directions">key: 車線番号(0~), value: CARS_~_ROADマクロのハッシュ</param>
	void ImgProcToolkit::SetRoadCarsDirections(const std::unordered_map<size_t, RoadDirect>&& directions)
	{
		sRoadCarsDirections = directions;
	}
	/* end */

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