#pragma once

#include "ImgProc.h"

class ImgProc::CarsTracer
{
private:
	class TemplateHandle;

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
public:
	CarsTracer() : mLabelNum(0) {}

	/// <summary>
	/// 車両検出
	/// </summary>
	void DetectCars();
private:
	CarsTracer(const CarsTracer& other) = delete;

	void TraceCars(const size_t& idx);
	void TraceByEdge() {}
	void JudgeStopTraceAndDetect(const size_t& idx, const uint64_t& carId, const cv::Rect2d& carPos);
	void DestructTracedCars();

	void DetectNewCars(const size_t& idx);
	bool IsntDetectedCars(const size_t& idx, const cv::Rect2d& carPos);
	bool DoesntAddBoundCar(const size_t& idx, const cv::Rect2d& carPosRect);
	void ReExtractTemplate(cv::Rect2d& carPos);
};
