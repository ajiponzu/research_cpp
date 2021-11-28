#include "CarsTracer.h"
#include "TemplateHandle.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	/// <summary>
	/// 車両検出
	/// </summary>
	void CarsTracer::DetectCars()
	{
		Tk::sFrame.copyTo(Tk::sResutImg);
		Tk::sCarsNumPrev = Tk::sCarsNum; // 前フレームの車両台数を保持

		for (size_t idx = 0; idx < Tk::sRoadMasksNum; idx++)
		{
			cv::bitwise_and(Tk::sCarsImg, Tk::sRoadMasksGray[idx], mTemp); // マスキング処理
			mLabelNum = cv::connectedComponentsWithStats(mTemp, mLabels, mStats, mCentroids); // ラベリング

			if (Tk::sFrameCount == Tk::sStartFrame)
			{
				DetectNewCars(idx);
				continue;
			}

			TraceCars(idx);
			DetectNewCars(idx);
		}
	}

	void CarsTracer::TraceCars(const size_t& idx)
	{
		auto& templates = Tk::sTemplatesList[idx];
		auto& templatePositions = Tk::sTemplatePositionsList[idx];
		auto& boundaryCarIdList = Tk::sBoundaryCarIdLists[idx];
		auto& roadCarsDirection = Tk::sRoadCarsDirections[idx];

		double maxValue = 0.0, magni = 1.0016;
		int mergin = 8;

		/* 検出済み車両ごとに処理 */
		for (auto carId = Tk::sFrontCarsId; carId < Tk::sCarsNum; carId++)
		{
			if (templates.find(carId) == templates.end())
				continue;

			/* テンプレートマッチング */
			auto& carImg = templates[carId];
			auto& carPos = templatePositions[carId];
			TemplateHandle::ExtractCarsNearestArea(mNearRect, idx, carId, magni, mergin);
			mTemp = GetImgSlice(Tk::sFrame, mNearRect);
			cv::matchTemplate(mTemp, carImg, mDataTemp, cv::TM_CCOEFF_NORMED);
			cv::minMaxLoc(mDataTemp, nullptr, &maxValue, nullptr, &mMaxLoc);

			if (maxValue < 0.4)
			{
				mDeleteLists.push_back(std::pair(idx, carId));
				continue;
			}

			carPos.x = mNearRect.x + mMaxLoc.x;
			carPos.y = mNearRect.y + mMaxLoc.y;
			cv::rectangle(Tk::sResutImg, carPos, cv::Scalar(255, 0, 0), 1);
			//templates[carId] = ExtractTemplate(Tk::sFrame, carPos);

			JudgeStopTraceAndDetect(idx, carId, carPos);
		}
		/* end */
		DestructTracedCars();
	}

	void CarsTracer::JudgeStopTraceAndDetect(const size_t& idx, const uint64_t& carId, const cv::Rect2d& carPos)
	{
		/* 追跡終了位置かどうかの判別 */
		switch (Tk::sRoadCarsDirections[idx])
		{
		case Tk::CARS_APPROACH_ROAD:
			if ((carPos.y + carPos.height) > Tk::sDetectBottom)
				mDeleteLists.push_back(std::pair(idx, carId));
			if (carPos.y > (Tk::sDetectTop + Tk::sDetectMergin + Tk::sDetectMerginPad))
				Tk::sBoundaryCarIdLists[idx].erase(carId);
			break;
		case Tk::CARS_LEAVE_ROAD:
			if (carPos.y < Tk::sDetectTop)
				mDeleteLists.push_back(std::pair(idx, carId));
			if ((carPos.y + carPos.height) < (Tk::sDetectTop - Tk::sDetectMergin - Tk::sDetectMerginPad))
				Tk::sBoundaryCarIdLists[idx].erase(carId);
			break;
		default:
			break;
		}
		/* end */
	}

	void CarsTracer::DestructTracedCars()
	{
		/* 追跡終了車両をデータから除外 */
		for (const auto& [roadIdx, carId] : mDeleteLists)
		{
			Tk::sTemplatesList[roadIdx].erase(carId);
			Tk::sTemplatePositionsList[roadIdx].erase(carId);
			Tk::sBoundaryCarIdLists[roadIdx].erase(carId);
			if (carId == Tk::sFrontCarsId)
				Tk::sFrontCarsId++;
		}
		mDeleteLists.clear();
		/* end */
	}

	void CarsTracer::DetectNewCars(const size_t& idx)
	{
		auto& templates = Tk::sTemplatesList[idx];
		auto& templatePositions = Tk::sTemplatePositionsList[idx];
		auto& boundaryCarIdList = Tk::sBoundaryCarIdLists[idx];

		/* 各領域ごとの処理, 0番は背景 */
		for (int label = 1; label < mLabelNum; label++)
		{
			/* 統計情報分割 */
			auto statsPtr = mStats.ptr<int>(label);
			auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
			/* end */

			if (width * height < 70)
				continue;

			/* 検出位置チェック */
			// 検出開始位置近傍の車両を特定, 未検出車両なら車両IDを保存
			// 1フレーム目は, 車両として検出しても, IDを保存しないものもあることに注意
			cv::Rect2d carPosRect(x, y, width, height);
			bool doesntDetectCar = false;

			/* 1フレーム目で検出されない領域を除外 */
			if (Tk::sFrameCount == Tk::sStartFrame)
			{
				auto bottomY = carPosRect.y + carPosRect.height;
				doesntDetectCar = (carPosRect.y < (Tk::sDetectTop + Tk::sDetectMergin + Tk::sDetectMerginPad)) || (bottomY > (Tk::sDetectBottom - Tk::sDetectMergin - Tk::sDetectMerginPad));
			}
			/* end */
			else
				doesntDetectCar = IsntDetectedCars(idx, carPosRect);

			if (doesntDetectCar)
				continue;

			if (DoesntAddBoundCar(idx, carPosRect))
				continue;

			boundaryCarIdList.insert(Tk::sCarsNum); 
			/* end */

			ReExtractTemplate(carPosRect); // テンプレート再抽出
			mTemp = ExtractTemplate(Tk::sFrame, carPosRect);

			cv::rectangle(Tk::sResutImg, carPosRect, cv::Scalar(0, 0, 255), 1); // 矩形を描く

			/* テンプレート抽出等 */
			templates.insert(std::pair(Tk::sCarsNum, mTemp));
			templatePositions.insert(std::pair(Tk::sCarsNum, carPosRect));
			/* end */

			/* 検出台数を更新 */
			Tk::sCarsNum++;
			Tk::sFrameCarsNum++;
			/* end */
		}
		/* end */
	}

	bool CarsTracer::IsntDetectedCars(const size_t& idx, const cv::Rect2d& carPos)
	{
		/* 検出位置チェック */
		bool doesntDetectCar = true;
		auto bottomY = carPos.y + carPos.height;

		/* 検出開始地点から遠い領域かをチェック */
		switch (Tk::sRoadCarsDirections[idx])
		{
		case Tk::CARS_APPROACH_ROAD:
			doesntDetectCar = (carPos.y < Tk::sDetectTop) || (carPos.y > (Tk::sDetectTop + Tk::sDetectMergin));
			return doesntDetectCar;
		case Tk::CARS_LEAVE_ROAD:
			doesntDetectCar = (bottomY < (Tk::sDetectBottom - Tk::sDetectMergin)) || (bottomY > Tk::sDetectBottom);
			return doesntDetectCar;
		default:
			break;
		}
		/* end */

		/* 2フレーム目以降は, 検出開始地点から遠い車両を検出しない */
		if (Tk::sFrameCount > Tk::sStartFrame && doesntDetectCar)
			return true;

		return false;
	}

	bool CarsTracer::DoesntAddBoundCar(const size_t& idx, const cv::Rect2d& carPosRect)
	{
		bool retFlag = false;
		for (const auto& elem : Tk::sBoundaryCarIdLists[idx])
		{
			const auto& carPos = Tk::sTemplatePositionsList[idx][elem];
			auto diffPosX = carPosRect.x - carPos.x;
			auto diffPosY = carPosRect.y - carPos.y;
			retFlag = (std::abs(diffPosX) < Tk::sDetectedNearOffset) && (std::abs(diffPosY) < Tk::sDetectedNearOffset);
			if (retFlag)
				break;

			diffPosX = (carPosRect.x + carPosRect.width) - (carPos.x + carPos.width);
			diffPosY = (carPosRect.y + carPosRect.height) - (carPos.y + carPos.height);
			retFlag = (std::abs(diffPosX) < Tk::sDetectedNearOffset) && (std::abs(diffPosY) < Tk::sDetectedNearOffset);
			if (retFlag)
				break;
		}

		return retFlag;
	}

	void CarsTracer::ReExtractTemplate(cv::Rect2d& carPos)
	{
		/* テンプレート再抽出 */
		{
			mTemp = GetImgSlice(Tk::sFrame, carPos);
			auto cutY = TemplateHandle::ExtractAreaByEdgeH(mTemp);
			auto cutPairX = TemplateHandle::ExtractAreaByEdgeV(mTemp);

			if (cutY <= (carPos.height * 0.35))
				cutY = static_cast<int>(carPos.height) - 1;

			auto wid = cutPairX.second - cutPairX.first + 1;
			if (wid < (carPos.width * 0.35))
			{
				cutPairX.first = 0;
				wid = static_cast<int>(carPos.width);
			}

			carPos.x += cutPairX.first;
			carPos.width = wid;
			carPos.height = cutY + 1;
		}
		/* end */
	}
};