#pragma once

#include "ImgProc.h"

class ImgProc::CarsTracer
{
public:
	void FindCarsTemplates();
private:
	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <returns>制限された領域画像(画像データについてはシャローコピー)</returns>
	Image ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId);
};
