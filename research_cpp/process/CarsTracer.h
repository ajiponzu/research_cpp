#pragma once

#include "ImgProc.h"

class ImgProc::CarsTracer
{
private:
	class TemplateHandle;
	friend class TemplateHandle;

	std::vector<std::pair<size_t, uint64_t>> mDeleteLists;
	Image mTempFrame;
	Image mTemp;
	Image mDataTemp;

	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群

	int mLabelNum; // ラベル数

	cv::Point mMaxLoc;
	cv::Rect2d mNearRect;

	std::vector<std::unordered_set<uint64_t>> mMatchLabels;
public:
	CarsTracer() 
	{
		mLabelNum = 0;
		mMatchLabels.resize(ImgProcToolkit::sRoadMasksNum);
	}

	/// <summary>
	/// 車両検出
	/// </summary>
	void DetectCars();
private:
	CarsTracer(const CarsTracer& other) = delete;

	void TraceCars(const size_t& idx);
	void DetectNewCars(const size_t& idx);

	template<class T> void TraceByLabels(const size_t& idx, cv::Rect_<T>& carPos);
};
