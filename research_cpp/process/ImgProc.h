#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <unordered_set>

namespace ImgProc
{
	using Image = cv::Mat;

	enum class RoadDirect
	{
		LEAVE = 0,
		APPROACH = 1,
	};

	struct DetectAreaInf
	{
		int top = 0;
		int bottom = 0;
		int mergin = 0;
		int merginPad = 0;
		int nearOffset = 0;
	};

	struct ExtractorParams
	{
		int shadowThrL = 0;
		int shadowThrB = 0;
		int closeCount = 0;
		int kernelSize = 0;
		int reshadowAreaThr = 0;
		float reshadowAspectThr = 0.0f;
	};

	struct TracerParams
	{
		double minAreaRatio = 0.0;
		double minMatchingThr = 0.0;
		int detectAreaThr = 0;
	};

	struct TemplateHandleParams
	{
		int mergin = 0;
		double magni = 0.0;
		int kernelSize = 0;
		int closeCount = 0;
		double minAreaRatio = 0.0;
		int areaThr = 0;
	};

	struct BackImgHandleParams
	{
		double blendAlpha = 0.0;
	};

	class CarsExtractor;
	class CarsTracer;

	class ImgProcToolkit
	{
		ImgProcToolkit() = delete; //staticクラスなので
	private:
		// 入力ビデオキャプチャ
		static cv::VideoCapture sVideoCapture;
		// ビデオレコーダー
		static cv::VideoWriter sVideoWriter;
		// 入力ビデオの横幅
		static int sVideoWidth;
		// 入力ビデオの縦幅
		static int sVideoHeight;
		// 入力フレーム
		static Image sFrame;
		// 結果画像
		static Image sResultImg;
		// 車両二値画像
		static Image sCarsImg;
		// 背景画像
		static Image sBackImg;
		// 道路マスク画像
		static Image sRoadMaskGray;
		// 道路マスク画像(テンプレートマッチング)
		static std::vector<Image> sRoadMasksGray;
		/* end */

		/* テンプレート処理に用いる変数 */
		// 抽出したテンプレートを保存, 車線ごとに保存
		static std::vector<std::unordered_map<uint64_t, Image>> sTemplatesList;
		// テンプレートの抽出位置を保存, 車線ごとに保存
		static std::vector<std::unordered_map<uint64_t, cv::Rect2d>> sTemplatePositionsList;
		// 車線ごとの車の移動方向を保存
		static std::unordered_map<size_t, RoadDirect> sRoadCarsDirections;
		// 車線ごとに検出境界に最も近い(高さの大小)車両IDを保存
		static std::vector<std::unordered_set<uint64_t>> sBoundaryCarIdLists;
		/* end */

		/* ファイルパス関連 */
		// sRoadMasksのsize数
		static size_t sRoadMasksNum;
		/* end */

		// 読み込んだフレーム数
		static uint64_t sFrameCount;
		// 初期フレーム
		static uint64_t sStartFrame;
		// 終了フレーム
		static uint64_t sEndFrame;
		// 全フレーム中の検出・追跡中車両台数
		static uint64_t sCarsNum;
		// 全フレーム中の検出・追跡中車両台数(前フレームのもの)
		static uint64_t sCarsNumPrev;
		// 現在のフレーム中の車両台数
		static uint64_t sFrameCarsNum;
		// 検出車両のうち, もっとも最初に検出した車両のID
		static uint64_t sFrontCarId;

		/* パラメータ構造体 */
		static DetectAreaInf sDetectAreaInf; // 検出範囲
		static ExtractorParams sExtractorParams; // 車両抽出パラメータ
		static TracerParams sTracerParams; // 車両追跡パラメータ
		static TemplateHandleParams sTemplateHandleParams; // テンプレート操作パラメータ
		static BackImgHandleParams sBackImgHandleParams; // 背景処理パラメータ
		/* end */

		static std::string sOutputBasePath; // 出力動画のベースパス

	private:
		/// <summary>
		/// ビデオリソース読み込み・書き出し設定
		/// </summary>
		/// <param name="inputPath">入力ビデオパス</param>
		/// <param name="outputPath">出力パス</param>
		static void CreateVideoResource(const std::string& inputPath, const std::string& outputPath);

		/// <summary>
		/// マスク画像等読み込み
		/// </summary>
		/// <param name="roadMaskPath">マスク画像（全体）パス</param>
		/// <param name="roadMasksBasePath">道路マスク画像ベースパス</param>
		static void CreateImageResource(const std::string& roadMaskPath, const std::string& roadMasksBasePath);

		/// <summary>
		/// 車線ごとの車の移動方向を設定
		/// </summary>
		/// <param name="directions">"L"か"R"が格納された配列</param>
		static void SetRoadCarsDirections(const std::vector<std::string>& directions);

		/// <summary>
		/// リソース画像表示
		/// </summary>
		/// <param name="interval">待機時間[ms]</param>
		static void ShowResourceImgs(const int& interval);

	public:
		/// <summary>
		/// リソース読み込み
		/// </summary>
		static void SetResourcesAndParams();

		/// <summary>
		/// リソース確認
		/// </summary>
		static void ShowResourcesAndParams();

		/// <summary>
		/// 処理実行
		/// </summary>
		static void RunImageProcedure();

		/* セッタ・ゲッタ */
		/* セッタ */
		static void SetCarsNum(const uint64_t& carsNum) { sCarsNum = carsNum; }
		static void SetCarsNumPrev(const uint64_t& carsNumPrev) { sCarsNumPrev = carsNumPrev; }
		static void SetFrameCarsNum(const uint64_t& frameCarsNum) { sFrameCarsNum = frameCarsNum; }
		/* end */
		/* ゲッタ */
		static cv::VideoCapture& GetVideoCapture() { return sVideoCapture; }
		static cv::VideoWriter& GetVideoWriter() { return sVideoWriter; }
		static std::pair<int, int> GetVideoWidAndHigh() { return std::make_pair(sVideoWidth, sVideoHeight); }
		static uint64_t& GetStartFrame() { return sStartFrame; }
		static uint64_t& GetEndFrame() { return sEndFrame; }
		static Image& GetFrame() { return sFrame; }
		static Image& GetResult() { return sResultImg; }
		static Image& GetCars() { return sCarsImg; }
		static Image& GetBackImg() { return sBackImg; }
		static const size_t& GetRoadMasksNum() { return sRoadMasksNum; }
		static Image& GetRoadMaskGray() { return sRoadMaskGray; }
		static std::vector<Image>& GetRoadMasksGray() { return sRoadMasksGray; }
		static uint64_t& GetFrameCount() { return sFrameCount; }
		static uint64_t& GetFrontCarId() { return sFrontCarId; }
		static uint64_t& GetCarsNum() { return sCarsNum; }
		static uint64_t& GetFrameCarsNum() { return sFrameCarsNum; }
		static const uint64_t& GetCarsNumPrev() { return sCarsNumPrev; }
		static std::vector<std::unordered_map<uint64_t, Image>>& GetTemplatesList() { return sTemplatesList; }
		static std::vector<std::unordered_map<uint64_t, cv::Rect2d>>& GetTemplatePositionsList() { return sTemplatePositionsList; }
		static std::unordered_map<size_t, RoadDirect>& GetRoadCarsDirections() { return sRoadCarsDirections; }
		static std::vector<std::unordered_set<uint64_t>>& GetBoundaryCarIdLists() { return sBoundaryCarIdLists; }
		static const DetectAreaInf& GetDetectAreaInf() { return sDetectAreaInf; }
		static const ExtractorParams& GetExtractorParams() { return sExtractorParams; }
		static const TracerParams& GetTracerParams() { return sTracerParams; }
		static const TemplateHandleParams& GetTemplateHandleParams() { return sTemplateHandleParams; }
		static const BackImgHandleParams& GetBackImgHandleParams() { return sBackImgHandleParams; }
		static const std::string& GetOutputBasePath() { return sOutputBasePath; }
		/* end */
		/* end */
	};

	/// <summary>
	/// BGR二値化
	/// </summary>
	/// <param name="inputImg">二値化画像</param>
	void binarizeImage(Image& inputImg);

	/// <summary>
	///	画像の部分参照
	/// </summary>
	/// <param name="inputImg">参照元画像</param>
	/// <param name="x">参照範囲矩形の左上座標のx成分</param>
	/// <param name="y">参照範囲矩形の左上座標のy成分</param>
	/// <param name="width">参照範囲矩形の横幅</param>
	/// <param name="height">参照範囲矩形の縦幅</param>
	/// <returns>参照範囲の部分画像</returns>
	Image GetImgSlice(const Image& inputImg, const int& x, const int& y, const int& width, const int& height);

	/// <summary>
	///	画像の部分参照
	/// </summary>
	/// <param name="inputImg">参照元画像</param>
	/// <param name="x">参照範囲矩形の左上座標のx成分</param>
	/// <param name="y">参照範囲矩形の左上座標のy成分</param>
	/// <param name="width">参照範囲矩形の横幅</param>
	/// <param name="height">参照範囲矩形の縦幅</param>
	/// <returns>参照範囲の部分画像</returns>
	Image GetImgSlice(const Image& inputImg, const double& x, const double& y, const double& width, const double& height);

	/// <summary>
	///	画像の部分参照
	/// </summary>
	/// <param name="inputImg">参照元画像</param>
	/// <param name="x">参照範囲矩形</param>
	/// <returns>参照範囲の部分画像(</returns>
	Image GetImgSlice(const Image& inputImg, const cv::Rect2d& rect);

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="x">抽出範囲矩形の左上座標のx成分</param>
	/// <param name="y">抽出範囲矩形の左上座標のy成分</param>
	/// <param name="width">抽出範囲矩形の横幅</param>
	/// <param name="height">抽出範囲矩形の縦幅</param>
	/// <returns>指定範囲の抽出画像(クローン後)</returns>
	Image ExtractTemplate(const Image& inputImg, const int& x, const int& y, const int& width, const int& height);

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="x">抽出範囲矩形の左上座標のx成分</param>
	/// <param name="y">抽出範囲矩形の左上座標のy成分</param>
	/// <param name="width">抽出範囲矩形の横幅</param>
	/// <param name="height">抽出範囲矩形の縦幅</param>
	/// <returns>指定範囲の抽出画像(クローン後)</returns>
	Image ExtractTemplate(const Image& inputImg, const double& x, const double& y, const double& width, const double& height);

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="rect">抽出範囲矩形</param>
	/// <returns>指定範囲の抽出画像(クローン後)</returns>
	Image ExtractTemplate(const Image& inputImg, const cv::Rect2d& rect);
};
