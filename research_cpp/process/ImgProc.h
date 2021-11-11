#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

namespace ImgProc
{
	using Image = cv::Mat;
	using Point = cv::Point;

	class CarsDetector;
	class CarsTracer;

	//抽出したテンプレートを保存
	static std::vector<std::vector<Image>> gTemplatesList;
	//テンプレートの抽出位置を保存
	static std::vector<Point> gTemplatePositionsList;

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
};
