#pragma once

#include "ImgProc.h"

class ImgProc::CarsTracer
{
private:
	std::vector<std::pair<size_t, uint64_t>> mDeleteLists;
	Image mTempFrame;
	Image mTemp;
	Image mDataTemp;
public:
	CarsTracer() {}

	void FindCarsTemplates();
private:
	CarsTracer(const CarsTracer& other) = delete;

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="orgMagni">拡大・縮小倍率</param>
	/// <param name="orgMergin">車両の1フレーム後における推定移動幅</param>
	/// <returns>制限された領域をあらわす矩形</returns>
	cv::Rect2d ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId, const double& orgMagni=1.001, const int& orgMergin = 8);
};
