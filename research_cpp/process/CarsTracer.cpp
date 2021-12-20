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

	/// <summary>
	/// 車両追跡
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	void CarsTracer::TraceCars(const size_t& idx)
	{
		auto& templates = Tk::sTemplatesList[idx];
		auto& templatePositions = Tk::sTemplatePositionsList[idx];
		auto& boundaryCarIdList = Tk::sBoundaryCarIdLists[idx];
		auto& roadCarsDirection = Tk::sRoadCarsDirections[idx];

		double maxValue = 0.0, magni = 1.0016;
		int mergin = 16;

		/* 検出済み車両ごとに処理 */
		for (auto carId = Tk::sFrontCarsId; carId < Tk::sCarsNum; carId++)
		{
			if (templates.find(carId) == templates.end())
				continue;

			/* テンプレートマッチング */
			auto& carImg = templates[carId];
			auto& carPos = templatePositions[carId];
			TemplateHandle::ExtractCarsNearestArea(mNearRect, idx, carId, magni, mergin);
			mTemp = GetImgSlice(Tk::sFrame, mNearRect).clone();

			//Image edge, edgeTempl;
			//cv::cvtColor(mTemp, mTemp, cv::COLOR_BGR2GRAY);
			//cv::Laplacian(mTemp, edge, CV_8U);
			//cv::cvtColor(edge, edge, cv::COLOR_GRAY2BGR);

			//cv::cvtColor(carImg, mTemp, cv::COLOR_BGR2GRAY);
			//cv::Laplacian(mTemp, edgeTempl, CV_8U);
			//cv::cvtColor(edgeTempl, edgeTempl, cv::COLOR_GRAY2BGR);
			//cv::matchTemplate(edge, edgeTempl, mDataTemp, cv::TM_CCOEFF_NORMED);

			cv::matchTemplate(mTemp, carImg, mDataTemp, cv::TM_CCOEFF_NORMED);
			cv::minMaxLoc(mDataTemp, nullptr, &maxValue, nullptr, &mMaxLoc);

			if (maxValue < 0.4)
			{
				mDeleteLists.push_back(std::pair(idx, carId));
				continue;
			}
			/* end */

			carPos.x = mNearRect.x + mMaxLoc.x;
			carPos.y = mNearRect.y + mMaxLoc.y;
			cv::rectangle(Tk::sResutImg, carPos, cv::Scalar(255, 0, 0), 1);

			//if (carId % 5 == 0)
			//{
			//	const std::string path = Tk::sTemplatesPathList[Tk::sVideoType] + "template_" + std::to_string(carId) + "_" + std::to_string(Tk::sFrameCount) + ".png";
			//	cv::imwrite(path, templates[carId]);
			//}
			//templates[carId] = ExtractTemplate(Tk::sFrame, carPos);

			JudgeStopTraceAndDetect(idx, carId, carPos); // 追跡終了判定
		}
		/* end */
		DestructTracedCars(); // 追跡終了処理
	}

	/// <summary>
	/// 車両追跡の停止・新規検出車両判定の停止を判断する
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="carPos">車両位置</param>
	void CarsTracer::JudgeStopTraceAndDetect(const size_t& idx, const uint64_t& carId, const cv::Rect2d& carPos)
	{
		/* 追跡終了位置か, 新規車両かどうかの判別 */
		switch (Tk::sRoadCarsDirections[idx])
		{
		case Tk::CARS_APPROACH_ROAD:
			if ((carPos.y + carPos.height) > Tk::sDetectBottom)
				mDeleteLists.push_back(std::pair(idx, carId)); // 追跡停止判定
			if (carPos.y > (Tk::sDetectTop + Tk::sDetectMergin + Tk::sDetectMerginPad))
				Tk::sBoundaryCarIdLists[idx].erase(carId);
			break;
		case Tk::CARS_LEAVE_ROAD:
			if (carPos.y < Tk::sDetectTop)
				mDeleteLists.push_back(std::pair(idx, carId)); // 追跡停止判定
			if ((carPos.y + carPos.height) < (Tk::sDetectTop - Tk::sDetectMergin - Tk::sDetectMerginPad))
				Tk::sBoundaryCarIdLists[idx].erase(carId);
			break;
		default:
			break;
		}
		/* end */
	}

	/// <summary>
	/// 追跡停止処理
	/// </summary>
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

	/// <summary>
	/// 車両検出, 開始フレームとそれ以降で検出範囲が変化
	/// </summary>
	/// <param name="idx"></param>
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

			if (area < width * height * 0.3) // 外周や直線だけで面積を稼いでるラベルを除外
				continue;

			if (area < (y - Tk::sDetectTop) / 4 + 30)
				continue;

			cv::Rect2d carPosRect(x, y, width, height);
			bool doesntDetectCar = false;

			///* 検出位置チェック */
			//// 検出開始位置近傍の車両を特定, 未検出車両なら車両IDを保存
			//// 1フレーム目は, 車両として検出しても, IDを保存しないものもあることに注意
			///* 1フレーム目で検出されない領域を除外 */
			//if (Tk::sFrameCount == Tk::sStartFrame)
			//{
			//	auto bottomY = carPosRect.y + carPosRect.height;
			//	doesntDetectCar = (carPosRect.y < (Tk::sDetectTop + Tk::sDetectMergin + Tk::sDetectMerginPad)) || (bottomY > (Tk::sDetectBottom - Tk::sDetectMergin - Tk::sDetectMerginPad));
			//}
			///* end */
			//else
			//	doesntDetectCar = IsntDetectedCars(idx, carPosRect); // 検出範囲にあるか判定

			//if (doesntDetectCar) // 検出しない場合スキップ
			//	continue;

			//if (DoesntAddBoundCar(idx, carPosRect)) // 検出範囲にある車両に対し検出するか判定. その後, 検出しない場合スキップ
			//	continue;
			///* end */

			ReExtractTemplate(carPosRect); // テンプレート再抽出

			for (const auto& finPos : mFinCarPosList)
			{
				/* 検出位置チェック */
				// 検出開始位置近傍の車両を特定, 未検出車両なら車両IDを保存
				// 1フレーム目は, 車両として検出しても, IDを保存しないものもあることに注意
				/* 1フレーム目で検出されない領域を除外 */
				if (Tk::sFrameCount == Tk::sStartFrame)
				{
					auto bottomY = finPos.y + finPos.height;
					doesntDetectCar = (finPos.y < (Tk::sDetectTop + Tk::sDetectMergin + Tk::sDetectMerginPad)) || (bottomY > (Tk::sDetectBottom - Tk::sDetectMergin - Tk::sDetectMerginPad));
				}
				/* end */
				else
					doesntDetectCar = IsntDetectedCars(idx, finPos); // 検出範囲にあるか判定

				if (doesntDetectCar) // 検出しない場合スキップ
					continue;

				if (DoesntAddBoundCar(idx, finPos)) // 検出範囲にある車両に対し検出するか判定. その後, 検出しない場合スキップ
					continue;
				/* end */

				boundaryCarIdList.insert(Tk::sCarsNum); // 新規検出車両として登録
				mTemp = ExtractTemplate(Tk::sFrame, finPos);

				cv::rectangle(Tk::sResutImg, finPos, cv::Scalar(0, 0, 255), 1); // 矩形を描く

				/* テンプレート抽出・保存 */
				templates.insert(std::pair(Tk::sCarsNum, mTemp));
				templatePositions.insert(std::pair(Tk::sCarsNum, finPos));
				/* end */

				/* 検出台数を更新 */
				Tk::sCarsNum++;
				Tk::sFrameCarsNum++;
				/* end */
			}
			mFinCarPosList.clear();
		}
		/* end */
	}

	/// <summary>
	/// 車両が検出範囲に存在するか判定
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carPos">車両位置</param>
	/// <returns>判定結果, trueなら検出しない</returns>
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

	/// <summary>
	/// 新規車両として検出するか判定. 以前検出した新規車両との位置関係を考える.
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carPosRect">車両位置</param>
	/// <returns>判定結果, trueなら検出しない</returns>
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

	/// <summary>
	/// テンプレート再抽出
	/// </summary>
	/// <param name="carPos">車両位置</param>
	void CarsTracer::ReExtractTemplate(const cv::Rect2d& carPos)
	{
		mTempTempTemp = ExtractTemplate(Tk::sFrame, carPos);
		mTemp = ExtractTemplate(Tk::sBackImg, carPos);
		cv::fastNlMeansDenoisingColored(mTempTempTemp, mTempTemp, 10.0f, 10.0f, 3);
		cv::fastNlMeansDenoisingColored(mTemp, mTempTempTemp, 10.0f, 10.0f, 3);

		cv::absdiff(mTempTemp, mTempTempTemp, mTemp);
		binarizeImage(mTemp);
		cv::morphologyEx(mTemp, mTempTemp, cv::MORPH_CLOSE, mCloseKernel, cv::Point(-1, -1), mCloseCount);

		Image labels, stats, centroids;

		//ラベリングによって求められるラベル数
		auto labelNum = cv::connectedComponentsWithStats(mTempTemp, labels, stats, centroids, 8);
		/* 各領域ごとの処理, 0番は背景 */
		for (int label = 1; label < labelNum; label++)
		{
			/* 統計情報分割 */
			auto statsPtr = stats.ptr<int>(label);
			auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
			/* end */

			auto tAreaThr = (carPos.y - Tk::sDetectTop) / 4 + 10; // 位置に応じた面積の閾値
			if (area < tAreaThr)
				continue;

			mFinCarPosList.push_back(cv::Rect(carPos.x + x, carPos.y + y, width, height));
		}
		/* end */
	}
};