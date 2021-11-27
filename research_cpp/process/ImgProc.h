#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <unordered_set>

namespace ImgProc
{
	using Image = cv::Mat;

	class CarsExtractor;
	class CarsTracer;

	class ImgProcToolkit
	{
		ImgProcToolkit() = delete; //staticクラスなので
		friend class CarsExtractor;
		friend class CarsTracer;
	public:
		constexpr static int VIDEO_TYPE_HARE = 0;
		constexpr static int VIDEO_TYPE_KUMORI = 1;
		constexpr static int VIDEO_TYPE_AME = 2;
		constexpr static int CARS_LEAVE_ROAD = 0;
		constexpr static int CARS_APPROACH_ROAD = 1;
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
		static Image sResutImg;
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
		static std::unordered_map<size_t, int> sRoadCarsDirections;
		// 車線ごとに検出境界に最も近い(高さの大小)車両IDを保存
		static std::vector<std::unordered_set<uint64_t>> sBoundaryCarIdLists;
		/* end */

		/* ファイルパス関連 */
		const static std::string sVideoPathList[3];
		const static std::string sOutputPathList[3];
		const static std::string sBackImgPathList[3];
		const static std::string sTemplatesPathList[3];
		const static std::string sRoadMaskPath;
		const static std::string sRoadMasksBasePath;
		// sRoadMasksのsize数
		static size_t sRoadMasksNum;
		// 使用リソースの指定
		static int sVideoType;
		/* end */

		// 読み込んだフレーム数
		static uint64_t sFrameCount;
		static uint64_t sStartFrame;
		// 全フレーム中の検出・追跡中車両台数
		static uint64_t sCarsNum;
		// 全フレーム中の検出・追跡中車両台数(前フレームのもの)
		static uint64_t sCarsNumPrev;
		// 現在のフレーム中の車両台数
		static uint64_t sFrameCarsNum;
		// 検出車両のうち, もっとも最初に検出した車両のID
		static uint64_t sFrontCarsId;

		/* 検出範囲指定 */
		static int sDetectTop;
		static int sDetectBottom;
		static int sDetectMergin;
		/* end */

	public:
		/// <summary>
		/// ビデオリソース読み込み・初期設定
		/// </summary>
		/// <returns>処理成功の真偽</returns>
		static bool CreateVideoResource();

		/// <summary>
		/// 背景画像・道路マスク画像等, 画像リソース読み込み
		/// </summary>
		/// <returns>処理成功の真偽</returns>
		static bool CreateImageResource();

		/// <summary>
		/// 車線ごとの車の移動方向を設定
		/// </summary>
		/// <param name="directions">key: 車線番号(0~), value: CARS_~_ROADマクロのハッシュ</param>
		static void SetRoadCarsDirections(const std::unordered_map<size_t, int>&& directions);

		/// <summary>
		/// リソース画像表示
		/// </summary>
		/// <param name="interval">待機時間[ms]</param>
		static void ShowResourceImgs(const int& interval);

		static void SetVideoType(const int& videoType) { sVideoType = videoType; }
		static void SetDetectTop(const int& detectTop) { sDetectTop = detectTop; }
		static void SetDetectBottom(const int& detectBottom) { sDetectBottom = detectBottom; }
		static void SetDetectMergin(const int& detectMergin) { sDetectMergin = detectMergin; }
		static void SetStartFrame(const int& startFrame) { sStartFrame = startFrame; }
		static cv::VideoCapture& GetVideoCapture() { return sVideoCapture; }
		static cv::VideoWriter& GetVideoWriter() { return sVideoWriter; }
		static Image& GetFrame() { return sFrame; }
		static uint64_t& GetFrameCount() { return sFrameCount; }
		static const uint64_t& GetCarsNum() { return sCarsNum; }
		static const uint64_t& GetCarsNumPrev() { return sCarsNumPrev; }
		static const uint64_t& GetFrameCarsNum() { return sFrameCarsNum; }
		static const Image& GetResult() { return sResutImg; }
	};

	/// <summary>
	/// BGR二値化
	/// </summary>
	/// <param name="inputImg">二値化画像</param>
	void binarizeImage(const Image& inputImg);

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
	template<class T> Image GetImgSlice(const Image& inputImg, const cv::Rect_<T>& rect);

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
	template<class T> Image ExtractTemplate(const Image& inputImg, const cv::Rect_<T>& rect);
};
