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

			mMatchLabels[idx].clear();
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

		double maxValue = 0.0, magni = 1.0015;
		int mergin = 6;

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

			/* 検出ラベルとの擦り合わせ */
			carPos.x = mNearRect.x + mMaxLoc.x;
			carPos.y = mNearRect.y + mMaxLoc.y;
			TraceByLabels(idx, carPos);
			cv::rectangle(Tk::sResutImg, carPos, cv::Scalar(255, 0, 0), 1);
			templates[carId] = ExtractTemplate(Tk::sFrame, carPos);
			/* end */
		}
		/* end */

		for (int label = 1; label < mLabelNum; label++)
		{
			auto statsPtr = mStats.ptr<int>(label);
			auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];

			cv::rectangle(Tk::sResutImg, cv::Rect(x, y, width, height), cv::Scalar(255, 255, 0), 1);
		}
	}

	void CarsTracer::DetectNewCars(const size_t& idx)
	{
		auto& templates = Tk::sTemplatesList[idx];
		auto& templatePositions = Tk::sTemplatePositionsList[idx];
		auto& boundaryCarIdList = Tk::sBoundaryCarIdLists[idx];

		/* 各領域ごとの処理, 0番は背景 */
		for (int label = 1; label < mLabelNum; label++)
		{
			if (mMatchLabels[idx].find(label) != mMatchLabels[idx].end())
				continue;

			/* 統計情報分割 */
			auto statsPtr = mStats.ptr<int>(label);
			auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
			/* end */

			if (area < 20)
				continue;

			/* 検出位置チェック */
			cv::Rect carRect(x, y, width, height);
			bool doesntDetectCar = true;
			auto bottomY = carRect.y + carRect.height;

			/* 1フレーム目で検出されない領域を除外 */
			if (Tk::sFrameCount == Tk::sStartFrame)
			{
				doesntDetectCar = (carRect.y < Tk::sDetectTop) || (bottomY > Tk::sDetectBottom);
				if (doesntDetectCar)
					continue;
			}
			/* end */

			/* 検出開始地点から遠い領域かをチェック */
			switch (Tk::sRoadCarsDirections[idx])
			{
			case Tk::CARS_APPROACH_ROAD:
				doesntDetectCar = (carRect.y < Tk::sDetectTop) || (carRect.y > (Tk::sDetectTop + Tk::sDetectMergin));
				break;
			case Tk::CARS_LEAVE_ROAD:
				doesntDetectCar = (bottomY < (Tk::sDetectBottom - Tk::sDetectMergin)) || (bottomY > Tk::sDetectBottom);
				break;
			default:
				break;
			}
			/* end */

			/* 2フレーム目以降は, 検出開始地点から遠い車両を検出しない */
			if (Tk::sFrameCount > Tk::sStartFrame && doesntDetectCar)
				continue;

			/* 検出開始位置近傍の車両を特定, 未検出車両なら車両IDを保存 */
			bool continueFlag = false;
			if (!doesntDetectCar)
			{
				for (const auto& elem : boundaryCarIdList)
				{
					const auto& carPos = templatePositions[elem];
					auto diffPosX = carRect.x - carPos.x;
					auto diffPosY = carRect.y - carPos.y;
					continueFlag = (std::abs(diffPosX) < 4) && (std::abs(diffPosY) < 4);
					if (continueFlag)
						break;

					diffPosX = (carRect.x + carRect.width) - (carPos.x + carPos.width);
					diffPosY = (carRect.y + carRect.height) - (carPos.y + carPos.height);
					continueFlag = (std::abs(diffPosX) < 4) && (std::abs(diffPosY) < 4);
					if (continueFlag)
						break;
				}

				if (continueFlag)
					continue;

				boundaryCarIdList.insert(Tk::sCarsNum); // 1フレーム目は, 車両として検出しても, ここでIDを保存しないものもあることに注意
			}
			/* end */
			/* end */

			/* テンプレート再抽出 */
			{
			//	mTemp = GetImgSlice(Tk::sFrame, carRect);
			//	auto cutY = ExtractAreaByEdgeH(mTemp);
			//	auto cutPairX = ExtractAreaByEdgeV(mTemp);

			//	if (cutY <= (carRect.height * 0.35))
			//		cutY = carRect.height - 1;

				//auto wid = cutPairX.second - cutPairX.first + 1;
				//if (wid < (carRect.width * 0.35))
				//{
				//	cutPairX.first = 0;
				//	wid = carRect.width;
				//}

				//carRect.x += cutPairX.first;
				//carRect.width = wid;
				//carRect.height = cutY + 1;
				mTemp = ExtractTemplate(Tk::sFrame, carRect);
			}
			/* end */

			cv::rectangle(Tk::sResutImg, carRect, cv::Scalar(0, 0, 255), 1); // 矩形を描く

			/* テンプレート抽出等 */
			templates.insert(std::pair(Tk::sCarsNum, mTemp));
			templatePositions.insert(std::pair(Tk::sCarsNum, carRect));
			/* end */

			/* 検出台数を更新 */
			Tk::sCarsNum++;
			Tk::sFrameCarsNum++;
			/* end */
		}
		/* end */
	}

	template<class T> void CarsTracer::TraceByLabels(const size_t& idx, cv::Rect_<T>& carPos)
	{
		const auto& label = mLabels.at<int>(static_cast<int>(carPos.y), static_cast<int>(carPos.x));
		if (label == 0)
			return;

		/* 統計情報分割 */
		auto statsPtr = mStats.ptr<int>(label);
		const auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		const auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		const auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		const auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		const auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
		/* end */

		auto labelRect = cv::RotatedRect(cv::Point2f(x, y), cv::Point2f(x + width, y + height), 0);
		auto carRotatedRect = cv::RotatedRect(cv::Point2f(carPos.x, carPos.y), cv::Point2f(carPos.x + carPos.width, carPos.y + carPos.height), 0);
		cv::Rect2d resultRect;

		std::vector<cv::Point2f> vertices;
		cv::rotatedRectangleIntersection(carRotatedRect, labelRect, vertices);
		if (vertices.size() == 4)
			resultRect = cv::Rect2d(vertices[0], vertices[3]);
		else
			resultRect = carPos;

		if (std::abs(resultRect.width - carPos.width) > carPos.width * 0.3)
			resultRect.width = std::min(resultRect.width, carPos.width);

		if (std::abs(resultRect.height - carPos.height) > carPos.height * 0.3)
			resultRect.height = std::min(resultRect.height, carPos.height);

		carPos = resultRect;
		mMatchLabels[idx].insert(label);
	}
};