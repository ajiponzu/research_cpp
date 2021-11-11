#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

namespace ImgProc
{
	using Image = cv::Mat;
	using Point = cv::Point;

	class CarsDetector;
	class CarsTracer;

	/* リソース変数staticクラス */
	static class ImgProcToolkit
	{
		ImgProcToolkit() = delete; //staticクラスなので
		friend class CarsDetector;
		friend class CarsTracer;
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
		// 背景画像
		static Image sBackImg;
		// 道路マスク画像
		static Image sRoadMask;
		static Image sRoadMaskGray;
		// 道路マスク画像(テンプレートマッチング)
		static std::vector<Image> sRoadMasks;
		static std::vector<Image> sRoadMasksGray;
		/* end */

		/* テンプレート処理に用いる変数 */
		//抽出したテンプレートを保存
		static std::vector<std::vector<Image>> sTemplatesList;
		//テンプレートの抽出位置を保存
		static std::vector<Point> sTemplatePositionsList;
		/* end */

	/* ファイルパス関連 */
		constexpr static int VIDEO_TYPE_HARE = 0;
		constexpr static int VIDEO_TYPE_KUMORI = 1;
		constexpr static int VIDEO_TYPE_AME = 2;

		const static std::string sVideoPathList[3];
		const static std::string sOutputPathList[3];

		constexpr static int sVideoType = VIDEO_TYPE_HARE;
		const static std::string sBackImgPathList[3];

		const static std::string sRoadMaskPath;
		const static std::string sRoadMasksBasePath;
		static int sRoadMasksNum;
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
		/// リソース画像表示
		/// </summary>
		/// <param name="interval">待機時間[ms]</param>
		static void ShowResourceImgs(const int& interval);

		static cv::VideoCapture& GetVideoCapture() { return sVideoCapture; }
		static cv::VideoWriter& GetVideoWriter() { return sVideoWriter; }
		static Image& GetFrame() { return sFrame; }
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
	Image GetImgSlice(Image& inputImg, const int& x, const int& y, const int& width, const int& height);

	/// <summary>
	/// テンプレート抽出
	/// </summary>
	/// <param name="inputImg">抽出元画像</param>
	/// <param name="x">抽出範囲矩形の左上座標のx成分</param>
	/// <param name="y">抽出範囲矩形の左上座標のy成分</param>
	/// <param name="width">抽出範囲矩形の横幅</param>
	/// <param name="height">抽出範囲矩形の縦幅</param>
	/// <returns>指定範囲の抽出画像(クローン後にムーブ)</returns>
	Image&& ExtractTemplate(Image& inputImg, const int& x, const int& y, const int& width, const int& height);
};
