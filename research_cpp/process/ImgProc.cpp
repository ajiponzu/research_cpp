#include "ImgProc.h"

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
	// 背景画像
	Image ImgProcToolkit::sBackImg;
	// 道路マスク画像
	Image ImgProcToolkit::sRoadMask;
	Image ImgProcToolkit::sRoadMaskGray;
	// 道路マスク画像(テンプレートマッチング)
	std::vector<Image> ImgProcToolkit::sRoadMasks;
	std::vector<Image> ImgProcToolkit::sRoadMasksGray;
	/* end */

	/* テンプレート処理に用いる変数 */
	//抽出したテンプレートを保存
	static std::vector<std::vector<Image>> sTemplatesList;
	//テンプレートの抽出位置を保存
	static std::vector<Point> sTemplatePositionsList;
	/* end */

	/* ファイルパス関連 */
	const std::string ImgProcToolkit::sVideoPathList[3] = { "./resource/hare/input.mp4", "./resource/kumori/input.mp4", "./resource/ame/input.mp4" };
	const std::string ImgProcToolkit::sOutputPathList[3] = { "./output/hare/output.mp4", "./output/kumori/output.mp4", "./output/ame/output.mp4" };
	const std::string ImgProcToolkit::sBackImgPathList[3] = { "./resource/hare/back.png", "./resource/kumori/back.mp4", "./resource/ame/back.mp4" };
	const std::string ImgProcToolkit::sRoadMaskPath = "./resource/back_kaikai.png";
	const std::string ImgProcToolkit::sRoadMasksBasePath = "./resource/back_kai";
	int ImgProcToolkit::sRoadMasksNum = 0;
	int ImgProcToolkit::sVideoType = ImgProcToolkit::VIDEO_TYPE_HARE;
	/* end */

	/* end */

	/// <summary>
	/// ビデオリソース読み込み・初期設定
	/// </summary>
	/// <returns>処理成功の真偽</returns>
	bool ImgProcToolkit::CreateVideoResource()
	{
		sVideoCapture.open(sVideoPathList[sVideoType]);
		if (!sVideoCapture.isOpened())
		{
			std::cout << sVideoPathList[sVideoType] << ": doesn't exist" << std::endl;
			return false;
		}

		auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
		sVideoWidth = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
		sVideoHeight = static_cast<int>(sVideoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
		auto videoFps = sVideoCapture.get(cv::CAP_PROP_FPS);
		sVideoWriter.open(sOutputPathList[sVideoType], fourcc, videoFps, cv::Size(sVideoWidth, sVideoHeight));
		if (!sVideoWriter.isOpened())
		{
			std::cout << sOutputPathList[sVideoType] << ": can't create or overwrite" << std::endl;
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
		sBackImg = cv::imread(sBackImgPathList[sVideoType]);
		if (sBackImg.empty())
		{
			std::cout << sBackImgPathList[sVideoType] << ": can't read this." << std::endl;
			return false;
		}

		sRoadMaskGray = cv::imread(sRoadMaskPath);
		if (sRoadMaskGray.empty())
		{
			std::cout << sRoadMaskPath << ": can't read this." << std::endl;
			return false;
		}
		binarizeImage(sRoadMaskGray);
		cv::cvtColor(sRoadMaskGray, sRoadMask, cv::COLOR_GRAY2BGR);

		size_t idx = 0;
		while (true)
		{
			const auto filePath = sRoadMasksBasePath + std::to_string(idx) + ".png";
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
			cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
			sRoadMasks.push_back(mask.clone());
			idx++;
		}
		sRoadMasksNum = idx;

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
		cv::imshow("", sRoadMask);
		cv::waitKey(interval);
		for (auto itr = sRoadMasks.begin(); itr != sRoadMasks.end(); itr++)
		{
			cv::imshow("", *itr);
			cv::waitKey(interval);
		}
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
	Image GetImgSlice(Image& inputImg, const int& x, const int& y, const int& width, const int& height)
	{
		return inputImg(cv::Range(y, y + height), cv::Range(x, x + width));
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
	Image&& ExtractTemplate(Image& inputImg, const int& x, const int& y, const int& width, const int& height)
	{
		return std::move(inputImg(cv::Range(y, y + height), cv::Range(x, x + width)).clone());
	}
	/* end */
};