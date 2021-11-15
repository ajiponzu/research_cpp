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
	/// <param name="orgMagni">拡大・縮小倍率</param>
	/// <param name="orgMergin">車両の1フレーム後における推定移動幅</param>
	/// <returns>制限された領域画像(画像データについてはシャローコピー)</returns>
	Image CarsTracer::ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId, const float& orgMagni, const int& orgMergin)
	{
		float magni = orgMagni; 
		int mergin = orgMergin;
		/* 車両が遠ざかっていくとき */
		if (Tk::sRoadCarsDirections[maskId] == Tk::CARS_LEAVE_ROAD)
		{
			magni = 1 / magni; // 縮小するために逆数にする
			mergin *= -1; // マイナスの方向に移動していくため
		}
		/* end */

		/* テンプレートの拡大・縮小処理と, 座標矩形の縦横の変更 */
		auto& rect = Tk::sTemplatePositionsList[maskId][carId];
		rect.width = static_cast<int>(magni * rect.width);
		rect.height = static_cast<int>(magni * rect.height);
		auto& carTemplate = Tk::sTemplatesList[maskId][carId];
		cv::resize(carTemplate, carTemplate, carTemplate.size(), magni, magni, cv::INTER_CUBIC);
		/* end */

		/* テンプレートマッチングの対象領域の限定 */
		/* 原点の設定 */
		auto findRectX = std::clamp(rect.x - std::abs(mergin), 0, Tk::sFrame.cols);
		auto& findRectY = rect.y;
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
};