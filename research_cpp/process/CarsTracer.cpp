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
		const auto& crefFrame = Tk::GetFrame();
		auto& refResultImg = Tk::GetResult();
		const auto& crefCarsImg = Tk::GetCars();
		const auto& crefRoadMasksGray = Tk::GetRoadMasksGray();

		crefFrame.copyTo(refResultImg);
		Tk::SetCarsNumPrev(Tk::GetCarsNum()); // 前フレームの車両台数を保持

		for (size_t idx = 0; idx < Tk::GetRoadMasksNum(); idx++)
		{
			cv::bitwise_and(crefCarsImg, crefRoadMasksGray[idx], mTemp); // マスキング処理
			mLabelNum = cv::connectedComponentsWithStats(mTemp, mLabels, mStats, mCentroids); // ラベリング

			if (Tk::GetFrameCount() == Tk::GetStartFrame())
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
		const auto& crefFrame = Tk::GetFrame();
		auto& refResultImg = Tk::GetResult();
		auto& refTemplates = Tk::GetTemplatesList()[idx];
		auto& refTemplatePositions = Tk::GetTemplatePositionsList()[idx];
		auto& refBoundaryCarIdList = Tk::GetBoundaryCarIdLists()[idx];
		auto& refRoadCarsDirection = Tk::GetRoadCarsDirections()[idx];

		double maxValue = 0.0;

		/* 検出済み車両ごとに処理 */
		for (auto carId = Tk::GetFrontCarId(); carId < Tk::GetCarsNum(); carId++)
		{
			if (refTemplates.find(carId) == refTemplates.end())
				continue;

			/* テンプレートマッチング */
			auto& refCarImg = refTemplates[carId];
			auto& refCarPos = refTemplatePositions[carId];
			TemplateHandle::ExtractCarsNearestArea(mNearRect, idx, carId);
			mTemp = GetImgSlice(crefFrame, mNearRect).clone();

			//Image edge, edgeTempl;
			//cv::cvtColor(mTemp, mTemp, cv::COLOR_BGR2GRAY);
			//cv::Laplacian(mTemp, edge, CV_8U);
			//cv::cvtColor(edge, edge, cv::COLOR_GRAY2BGR);

			//cv::cvtColor(carImg, mTemp, cv::COLOR_BGR2GRAY);
			//cv::Laplacian(mTemp, edgeTempl, CV_8U);
			//cv::cvtColor(edgeTempl, edgeTempl, cv::COLOR_GRAY2BGR);
			//cv::matchTemplate(edge, edgeTempl, mDataTemp, cv::TM_CCOEFF_NORMED);

			cv::matchTemplate(mTemp, refCarImg, mDataTemp, cv::TM_CCOEFF_NORMED);
			cv::minMaxLoc(mDataTemp, nullptr, &maxValue, nullptr, &mMaxLoc);

			if (maxValue < 0.4)
			{
				mDeleteLists.push_back(std::pair(idx, carId));
				continue;
			}
			/* end */

			refCarPos.x = mNearRect.x + mMaxLoc.x;
			refCarPos.y = mNearRect.y + mMaxLoc.y;
			cv::rectangle(refResultImg, refCarPos, cv::Scalar(255, 0, 0), 1);

			//if (carId % 5 == 0)
			//{
			//	const std::string path = Tk::sTemplatesPathList[Tk::sVideoType] + "template_" + std::to_string(carId) + "_" + std::to_string(Tk::sFrameCount) + ".png";
			//	cv::imwrite(path, templates[carId]);
			//}
			//templates[carId] = ExtractTemplate(Tk::sFrame, carPos);

			JudgeStopTraceAndDetect(idx, carId, refCarPos); // 追跡終了判定
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
		const auto& crefRoadCarsDirection = Tk::GetRoadCarsDirections()[idx];
		auto& refBoundaryCarIdList = Tk::GetBoundaryCarIdLists()[idx];
		/* 追跡終了位置か, 新規車両かどうかの判別 */
		switch (crefRoadCarsDirection)
		{
		case static_cast<int>(RoadDirect::APPROACH):
			if ((carPos.y + carPos.height) > Tk::GetDetectBottom())
				mDeleteLists.push_back(std::pair(idx, carId)); // 追跡停止判定
			if (carPos.y > (Tk::GetDetectTop() + Tk::GetDetectMergin() + Tk::GetDetectMerginPad()))
				refBoundaryCarIdList.erase(carId);
			break;
		case static_cast<int>(RoadDirect::LEAVE):
			if (carPos.y < Tk::GetDetectTop())
				mDeleteLists.push_back(std::pair(idx, carId)); // 追跡停止判定
			if ((carPos.y + carPos.height) < (Tk::GetDetectTop() - Tk::GetDetectMergin() - Tk::GetDetectMerginPad()))
				refBoundaryCarIdList.erase(carId);
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
		auto& refTemplatesList = Tk::GetTemplatesList();
		auto& refTemplatePositionsList = Tk::GetTemplatePositionsList();
		auto& refBoundaryCarIdLists = Tk::GetBoundaryCarIdLists();
		auto& refFrontCarId = Tk::GetFrontCarId();
		/* 追跡終了車両をデータから除外 */
		for (const auto& [roadIdx, carId] : mDeleteLists)
		{
			refTemplatesList[roadIdx].erase(carId);
			refTemplatePositionsList[roadIdx].erase(carId);
			refBoundaryCarIdLists[roadIdx].erase(carId);
			if (carId == refFrontCarId)
				refFrontCarId++;
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
		const auto& crefFrame = Tk::GetFrame();
		auto& refCarsNum = Tk::GetCarsNum();
		auto& refFrameCarsNum = Tk::GetFrameCarsNum();
		auto& refResultImg = Tk::GetResult();
		auto& refTemplates = Tk::GetTemplatesList()[idx];
		auto& refTemplatePositions = Tk::GetTemplatePositionsList()[idx];
		auto& refBoundaryCarIdList = Tk::GetBoundaryCarIdLists()[idx];

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

			if (area < (y - Tk::GetDetectTop()) / 4 + 30)
				continue;

			cv::Rect2d carPosRect(x, y, width, height);
			bool doesntDetectCar = false;
			ReExtractTemplate(carPosRect); // テンプレート再抽出

			for (const auto& finPos : mFinCarPosList)
			{
				/* 検出位置チェック */
				// 検出開始位置近傍の車両を特定, 未検出車両なら車両IDを保存
				// 1フレーム目は, 車両として検出しても, IDを保存しないものもあることに注意
				/* 1フレーム目で検出されない領域を除外 */
				if (Tk::GetFrameCount() == Tk::GetStartFrame())
				{
					auto bottomY = finPos.y + finPos.height;
					doesntDetectCar = (finPos.y < (Tk::GetDetectTop() + Tk::GetDetectMergin() + Tk::GetDetectMerginPad()))
						|| (bottomY > (Tk::GetDetectBottom() - Tk::GetDetectMergin() - Tk::GetDetectMerginPad()));
				}
				/* end */
				else
					doesntDetectCar = IsntDetectedCars(idx, finPos); // 検出範囲にあるか判定

				if (doesntDetectCar) // 検出しない場合スキップ
					continue;

				if (DoesntAddBoundCar(idx, finPos)) // 検出範囲にある車両に対し検出するか判定. その後, 検出しない場合スキップ
					continue;
				/* end */

				refBoundaryCarIdList.insert(refCarsNum); // 新規検出車両として登録
				mTemp = ExtractTemplate(crefFrame, finPos);

				cv::rectangle(refResultImg, finPos, cv::Scalar(0, 0, 255), 1); // 矩形を描く

				/* テンプレート抽出・保存 */
				refTemplates.insert(std::pair(refCarsNum, mTemp));
				refTemplatePositions.insert(std::pair(refCarsNum, finPos));
				/* end */

				/* 検出台数を更新 */
				refCarsNum++;
				refFrameCarsNum++;
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
		const auto& crefRoadCarsDirection = Tk::GetRoadCarsDirections()[idx];
		bool doesntDetectCar = true;
		auto bottomY = carPos.y + carPos.height;

		/* 検出開始地点から遠い領域かをチェック */
		switch (crefRoadCarsDirection)
		{
		case static_cast<int>(RoadDirect::APPROACH):
			doesntDetectCar = (carPos.y < Tk::GetDetectTop())
				|| (carPos.y > (Tk::GetDetectTop() + Tk::GetDetectMergin()));
			return doesntDetectCar;
		case static_cast<int>(RoadDirect::LEAVE):
			doesntDetectCar = (bottomY < (Tk::GetDetectBottom() - Tk::GetDetectMergin()))
				|| (bottomY > Tk::GetDetectBottom());
			return doesntDetectCar;
		default:
			break;
		}
		/* end */

		/* 2フレーム目以降は, 検出開始地点から遠い車両を検出しない */
		if (doesntDetectCar && (Tk::GetFrameCount() > Tk::GetStartFrame()))
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
		const auto& crefBoundaryCarIdList = Tk::GetBoundaryCarIdLists()[idx];
		auto& crefTemplatePositions = Tk::GetTemplatePositionsList()[idx];
		bool retFlag = false;
		for (const auto& elem : crefBoundaryCarIdList)
		{
			const auto& crefCarPos = crefTemplatePositions[elem];
			auto diffPosX = carPosRect.x - crefCarPos.x;
			auto diffPosY = carPosRect.y - crefCarPos.y;
			retFlag = (std::abs(diffPosX) < Tk::GetDetectedNearOffset())
				&& (std::abs(diffPosY) < Tk::GetDetectedNearOffset());
			if (retFlag)
				break;

			diffPosX = (carPosRect.x + carPosRect.width) - (crefCarPos.x + crefCarPos.width);
			diffPosY = (carPosRect.y + carPosRect.height) - (crefCarPos.y + crefCarPos.height);
			retFlag = (std::abs(diffPosX) < Tk::GetDetectedNearOffset())
				&& (std::abs(diffPosY) < Tk::GetDetectedNearOffset());
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
		TemplateHandle::ReLabelingTemplate(mFinCarPosList, carPos);
	}
};