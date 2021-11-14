#include "CarsTracer.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	void CarsTracer::FindCarsTemplates()
	{
	}

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <returns>制限された領域画像(画像データについてはシャローコピー)</returns>
	Image CarsTracer::ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId)
	{
		double magni = 1.002;
		int mergin = 8;
		if (Tk::sRoadCarsDirections[maskId] == Tk::CARS_LEAVE_ROAD)
		{
			magni = 1 / magni;
			mergin *= -1;
		}

		auto& rect = Tk::sTemplatePositionsList[maskId][carId];
		rect.width *= magni;
		rect.height *= magni;
		auto& carTemplate = Tk::sTemplatesList[maskId][carId];
		cv::resize(carTemplate, carTemplate, carTemplate.size(), magni, magni, cv::INTER_CUBIC);

		auto findRectX = std::clamp(rect.x - std::abs(mergin), 0, Tk::sFrame.cols);
		auto& findRectY = rect.y;

		auto findRectXR = std::clamp(rect.x + rect.width + std::abs(mergin), 0, Tk::sFrame.cols);
		auto findRectYB = std::clamp(rect.y + (mergin / std::abs(mergin)) * (rect.height + mergin), 0, Tk::sFrame.rows);

		auto findRectWidth = std::abs(findRectXR - findRectX);
		auto findRectHeight = std::abs(findRectYB - findRectY);

		findRectX = std::min(findRectX, findRectXR);
		findRectY = std::min(findRectY, findRectYB);

		return GetImgSlice(Tk::sFrame, findRectX, findRectY, findRectWidth, findRectHeight);
	}
};