#include "CarsTracer.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	void CarsTracer::FindCarsTemplates()
	{
		Tk::sFrame.copyTo(Tk::sResutImg);
		if (Tk::sFrameCount == 1)
			return;

		double maxValue = 0.0;
		float magni = 1.001f;
		int mergin = 8;
		cv::Point maxLoc;
		for (size_t idx = 0; idx < Tk::sRoadMasksNum; idx++)
		{
			cv::bitwise_and(Tk::sFrame, Tk::sRoadMasks[idx], mTempFrame);
			auto& templates = Tk::sTemplatesList[idx];
			auto& templatePositions = Tk::sTemplatePositionsList[idx];

			for (auto carId = Tk::sFrontCarsId; carId < Tk::sCarsNum; carId++)
			{
				if (templates.find(carId) == templates.end())
					continue;

				auto& carImg = templates[carId];
				auto& carPos = templatePositions[carId];
				mTemp = ExtractCarsNearestArea(idx, carId, magni, mergin);
				//cv::imshow("", mTemp);
				//cv::waitKey(2000);
				//cv::imshow("", carImg);
				//cv::waitKey(5000);
				cv::matchTemplate(mTemp, carImg, mDataTemp, cv::TM_CCOEFF_NORMED);
				//cv::matchTemplate(mTempFrame, carImg, mDataTemp, cv::TM_CCOEFF_NORMED);
				cv::minMaxLoc(mDataTemp, nullptr, &maxValue, nullptr, &maxLoc);
				if (maxValue < 0.45)
				{
					mDeleteLists.push_back(std::pair(idx, carId));
					continue;
				}

				ExtractTemplate(mTemp, maxLoc.x, maxLoc.y, carPos.width, carPos.height).copyTo(carImg);
				//ExtractTemplate(mTempFrame, maxLoc.x, maxLoc.y, carPos.width, carPos.height).copyTo(carImg);
				//carPos.x = maxLoc.x;
				//carPos.y = maxLoc.y;
				//carImg = ExtractTemplate(Tk::sFrame, maxLoc.x, maxLoc.y, carPos.width, carPos.height);
				carPos.x += maxLoc.x - mergin;
				carPos.y += maxLoc.y - mergin;
				cv::rectangle(Tk::sResutImg, carPos, cv::Scalar(255, 0, 0), 2);

				//cv::imshow("", carImg);
				//cv::waitKey(2000);
			}
		}

		for (const auto& [roadIdx, carId] : mDeleteLists)
		{
			Tk::sTemplatesList[roadIdx].erase(carId);
			Tk::sTemplatePositionsList[roadIdx].erase(carId);
			if (carId == Tk::sFrontCarsId)
				Tk::sFrontCarsId++;
		}
		mDeleteLists.clear();
	}

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="orgMagni">拡大・縮小倍率</param>
	/// <param name="orgMergin">車両の1フレーム後における推定移動幅</param>
	/// <returns>制限された領域画像(画像データについてはシャローコピー)</returns>
	Image CarsTracer::ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId, const float& orgMagni, const int& orgMergin)
	{
		float magni = orgMagni; 
		int mergin = orgMergin;
		auto& rect = Tk::sTemplatePositionsList[maskId][carId];
		auto& carTemplate = Tk::sTemplatesList[maskId][carId];
		auto basePos = cv::Point(rect.x, rect.y);

		/* 車両が遠ざかっていくとき */
		if (Tk::sRoadCarsDirections[maskId] == Tk::CARS_LEAVE_ROAD)
		{
			magni = 1 / magni; // 縮小するために逆数にする
			mergin *= -1; // マイナスの方向に移動していくため
			basePos.y += rect.height;
		}
		/* end */

		/* テンプレートの拡大・縮小処理と, 座標矩形の縦横の変更 */
		rect.width = static_cast<int>(magni * rect.width);
		rect.height = static_cast<int>(magni * rect.height);
		cv::resize(carTemplate, carTemplate, cv::Size(rect.width, rect.height), magni, magni, cv::INTER_CUBIC);
		/* end */

		/* テンプレートマッチングの対象領域の限定 */
		/* 原点の設定 */
		auto findRectX = std::clamp(rect.x - std::abs(mergin), 0, Tk::sFrame.cols);
		auto& findRectY = basePos.y;
		/* end */

		/* 移動後に予想される到達地点の最大値を算出 */
		auto findRectXR = std::clamp(rect.x + rect.width + std::abs(mergin), 0, Tk::sVideoWidth);
		auto findRectYB = std::clamp(rect.y + (mergin / std::abs(mergin)) * (rect.height + mergin), 0, Tk::sVideoHeight);
		/* end */

		/* 最終的な探索領域の縦横の幅を算出 */
		auto findRectWidth = std::abs(findRectXR - findRectX);
		auto findRectHeight = std::abs(findRectYB - findRectY);
		/* end */

		/* 小さいほうを原点として最終的に採用 */
		findRectX = std::min(findRectX, findRectXR);
		findRectY = std::min(findRectY, findRectYB);
		/* end */
		/* end */

		return GetImgSlice(Tk::sFrame, findRectX, findRectY, findRectWidth, findRectHeight);
	}
	cv::Rect CarsTracer::CreateApproachRect(const size_t& maskId, const uint64_t& carId, const float& orgMagni, const int& orgMergin)
	{
		float magni = orgMagni; 
		int mergin = orgMergin;
		auto& rect = Tk::sTemplatePositionsList[maskId][carId];
		auto& carTemplate = Tk::sTemplatesList[maskId][carId];
		auto basePos = cv::Point(rect.x, rect.y);

		/* テンプレートの拡大・縮小処理と, 座標矩形の縦横の変更 */
		rect.width = static_cast<int>(magni * rect.width);
		rect.height = static_cast<int>(magni * rect.height);
		cv::resize(carTemplate, carTemplate, carTemplate.size(), magni, magni, cv::INTER_CUBIC);
		/* end */

		/* テンプレートマッチングの対象領域の限定 */
		/* 原点の設定 */
		auto findRectX = std::clamp(rect.x - std::abs(mergin), 0, Tk::sFrame.cols);
		auto& findRectY = rect.y;
		/* end */

		/* 移動後に予想される到達地点の最大値を算出 */
		auto findRectXR = std::clamp(basePos.x + rect.width + std::abs(mergin), 0, Tk::sVideoWidth);
		auto findRectYB = std::clamp(basePos.y + (mergin / std::abs(mergin)) * (rect.height + mergin), 0, Tk::sVideoHeight);
		/* end */

		/* 最終的な探索領域の縦横の幅を算出 */
		auto findRectWidth = std::abs(findRectXR - findRectX);
		auto findRectHeight = std::abs(findRectYB - findRectY);
		/* end */

		/* 小さいほうを原点として最終的に採用 */
		findRectX = std::min(findRectX, findRectXR);
		findRectY = std::min(findRectY, findRectYB);
		/* end */
		/* end */

		return cv::Rect(findRectX, findRectY, findRectWidth, findRectHeight);
	}

	cv::Rect CarsTracer::CreateLeaveRect(const size_t& maskId, const uint64_t& carId, const float& orgMagni, const int& orgMergin)
	{
		float magni = 1 / orgMagni; 
		int mergin = -orgMergin;
		auto& rect = Tk::sTemplatePositionsList[maskId][carId];
		auto& carTemplate = Tk::sTemplatesList[maskId][carId];
		auto basePos = cv::Point(rect.x, rect.y + rect.height);

		/* テンプレートの拡大・縮小処理と, 座標矩形の縦横の変更 */
		rect.width = static_cast<int>(magni * rect.width);
		rect.height = static_cast<int>(magni * rect.height);
		cv::resize(carTemplate, carTemplate, carTemplate.size(), magni, magni, cv::INTER_CUBIC);
		/* end */

		/* テンプレートマッチングの対象領域の限定 */
		/* 原点の設定 */
		auto findRectX = std::clamp(rect.x - std::abs(mergin), 0, Tk::sFrame.cols);
		auto& findRectY = basePos.y;
		/* end */

		/* 移動後に予想される到達地点の最大値を算出 */
		auto findRectXR = std::clamp(basePos.x + rect.width + std::abs(mergin), 0, Tk::sVideoWidth);
		auto findRectYB = std::clamp(basePos.y + (mergin / std::abs(mergin)) * (rect.height + mergin), 0, Tk::sVideoHeight);
		/* end */

		/* 最終的な探索領域の縦横の幅を算出 */
		auto findRectWidth = std::abs(findRectXR - findRectX);
		auto findRectHeight = std::abs(findRectYB - findRectY);
		/* end */

		/* 小さいほうを原点として最終的に採用 */
		findRectX = std::min(findRectX, findRectXR);
		findRectY = std::min(findRectY, findRectYB);
		/* end */
		/* end */

		return cv::Rect(findRectX, findRectY, findRectWidth, findRectHeight);
	}
};