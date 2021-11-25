#include "CarsTracer.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	/// <summary>
	/// 車両追跡
	/// </summary>
	void CarsTracer::FindCarsTemplates()
	{
		Tk::sFrame.copyTo(Tk::sResutImg);
		if (Tk::sFrameCount == Tk::sStartFrame)
			return;

		//double maxValue = 0.0, magni = 1.0009;
		double maxValue = 0.0, magni = 1.0015;
		int mergin = 10;
		cv::Point maxLoc;
		cv::Rect2d nearRect;
		/* 各道路ごとに処理 */
		for (size_t idx = 0; idx < Tk::sRoadMasksNum; idx++)
		{
			cv::bitwise_and(Tk::sFrame, Tk::sRoadMasks[idx], mTempFrame);
			auto& templates = Tk::sTemplatesList[idx];
			auto& templatePositions = Tk::sTemplatePositionsList[idx];

			/* 検出済み車両ごとに処理 */
			for (auto carId = Tk::sFrontCarsId; carId < Tk::sCarsNum; carId++)
			{
				if (templates.find(carId) == templates.end())
					continue;

				/* テンプレートマッチング */
				auto& carImg = templates[carId];
				auto& carPos = templatePositions[carId];
				nearRect = ExtractCarsNearestArea(idx, carId, magni, mergin);
				mTemp = GetImgSlice(Tk::sFrame, nearRect);
				cv::matchTemplate(mTemp, carImg, mDataTemp, cv::TM_CCOEFF_NORMED);
				cv::minMaxLoc(mDataTemp, nullptr, &maxValue, nullptr, &maxLoc);

				if (maxValue < 0.35)
				{
					mDeleteLists.push_back(std::pair(idx, carId));
					continue;
				}
				/* end */

				//if ((carPos.x != (nearRect.x + maxLoc.x)) && (carPos.y != (nearRect.y + maxLoc.y)))
				//if (maxValue > 0.90)
				//	carImg = ExtractTemplate(mTemp, maxLoc.x, maxLoc.y, static_cast<int>(carPos.width), static_cast<int>(carPos.height));

				/* 車両位置更新, 矩形描画 */
				carPos.x = nearRect.x + maxLoc.x;
				carPos.y = nearRect.y + maxLoc.y;
				cv::rectangle(Tk::sResutImg, carPos, cv::Scalar(255, 0, 0), 1);
				/* end */

				/* 追跡終了位置かどうかの判別 */
				switch (Tk::sRoadCarsDirections[idx])
				{
				case Tk::CARS_APPROACH_ROAD:
					if (carPos.y + carPos.height > Tk::sDetectBottom)
						mDeleteLists.push_back(std::pair(idx, carId));
					break;
				case Tk::CARS_LEAVE_ROAD:
					if (carPos.y < Tk::sDetectTop)
						mDeleteLists.push_back(std::pair(idx, carId));
					break;
				default:
					break;
				}
				/* end */
			}
			/* end */
		}
		/* end */

		/* 追跡終了車両をデータから除外 */
		for (const auto& [roadIdx, carId] : mDeleteLists)
		{
			Tk::sTemplatesList[roadIdx].erase(carId);
			Tk::sTemplatePositionsList[roadIdx].erase(carId);
			if (carId == Tk::sFrontCarsId)
				Tk::sFrontCarsId++;
		}
		mDeleteLists.clear();
		/* end */
	}

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="orgMagni">拡大・縮小倍率</param>
	/// <param name="orgMergin">車両の1フレーム後における推定移動幅</param>
	/// <returns>制限された領域をあらわす矩形</returns>
	cv::Rect2d CarsTracer::ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId, const double& orgMagni, const int& orgMergin)
	{
		auto magni = orgMagni;
		double mergin = orgMergin;
		auto& rect = Tk::sTemplatePositionsList[maskId][carId];
		auto& carTemplate = Tk::sTemplatesList[maskId][carId];

		/* 車両が遠ざかっていくとき */
		if (Tk::sRoadCarsDirections[maskId] == Tk::CARS_LEAVE_ROAD)
			magni = 1 / magni; // 縮小するために逆数にする
		/* end */

		/* テンプレートの拡大・縮小処理と, 座標矩形の縦横の変更 */
		rect.width *= magni;
		rect.height *= magni;
		cv::resize(carTemplate, carTemplate, cv::Size(), magni, magni, cv::INTER_NEAREST_EXACT);
		/* end */

		/* テンプレートマッチングの対象領域の限定 */
		/* 原点の設定 */
		auto findRectX = std::round(std::clamp((rect.x - mergin), 0.0, static_cast<double>(Tk::sVideoWidth)));
		auto findRectY = std::round(std::clamp((rect.y - mergin), 0.0, static_cast<double>(Tk::sVideoHeight)));
		/* end */

		/* 移動後に予想される到達地点の最大値を算出 */
		auto findRectXR = std::round(std::clamp(findRectX + rect.width + mergin * 2.0, 0.0, static_cast<double>(Tk::sVideoWidth)));
		auto findRectYB = std::round(std::clamp(findRectY + rect.height + mergin * 2.0, 0.0, static_cast<double>(Tk::sVideoHeight)));
		/* end */

		/* 最終的な探索領域の縦横の幅を算出 */
		auto findRectWidth = std::abs(findRectXR - findRectX) + 1.0;
		auto findRectHeight = std::abs(findRectYB - findRectY) + 1.0;
		/* end */
		/* end */

		return cv::Rect2d(findRectX, findRectY, findRectWidth, findRectHeight);
	}
};