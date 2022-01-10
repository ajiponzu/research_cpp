#include "CarsTracer.h"
#include "TemplateHandle.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	CarsTracer::CarsTracer()
	{
		TemplateHandle::MakeCloseKernel();
	}

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
			mLabelNum = cv::connectedComponentsWithStats(mTemp, mLabels, mStats, mCentroids, 4); // ラベリング

			if (Tk::GetFrameCount() == Tk::GetStartFrame())
			{
				DetectNewCars(idx);
				continue;
			}

			TraceCars(idx);
			DetectNewCars(idx);
			DrawRectangles(idx);
		}
		const auto& detect = Tk::GetDetectAreaInf();
		cv::line(refResultImg, cv::Point(0, detect.top), cv::Point(Tk::GetVideoWidAndHigh().first, detect.top), cv::Scalar(0, 255, 0), 3);
		cv::line(refResultImg, cv::Point(0, detect.bottom), cv::Point(Tk::GetVideoWidAndHigh().first, detect.bottom), cv::Scalar(0, 255, 0), 3);
	}

	/// <summary>
	/// 車両追跡
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	void CarsTracer::TraceCars(const size_t& idx)
	{
		const auto& crefFrame = Tk::GetFrame();
		const auto& crefParams = Tk::GetTracerParams();
		auto& refResultImg = Tk::GetResult();
		auto& refTemplates = Tk::GetTemplatesList()[idx];
		auto& refTemplatePositions = Tk::GetTemplatePositionsList()[idx];
		auto& refRoadCarsDirection = Tk::GetRoadCarsDirections()[idx];

		double maxValueArray[2] = { 0.0, 0.0 };
		double maxValue = 0.0;
		Image gray, edge, edgeTempl;
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

			/* エッジによるテンプレートマッチング */
			cv::cvtColor(mTemp, gray, cv::COLOR_BGR2GRAY);
			cv::Laplacian(gray, edge, CV_8U);
			cv::cvtColor(edge, edge, cv::COLOR_GRAY2BGR);

			cv::cvtColor(refCarImg, gray, cv::COLOR_BGR2GRAY);
			cv::Laplacian(gray, edgeTempl, CV_8U);
			cv::cvtColor(edgeTempl, edgeTempl, cv::COLOR_GRAY2BGR);
			cv::matchTemplate(edge, edgeTempl, mDataTemp, cv::TM_CCORR_NORMED); // cos類似度
			cv::minMaxLoc(mDataTemp, nullptr, &maxValueArray[0], nullptr, &mMaxLocArray[0]);
			/* end */

			/* カラーによるテンプレートマッチング */
			cv::matchTemplate(mTemp, refCarImg, mDataTemp, cv::TM_CCOEFF_NORMED); // ZNCC
			cv::minMaxLoc(mDataTemp, nullptr, &maxValueArray[1], nullptr, &mMaxLocArray[1]);
			/* end */

			if (maxValueArray[0] <= maxValueArray[1])
			{
				mMaxLoc = mMaxLocArray[1];
				maxValue = maxValueArray[1];
			}
			else
			{
				mMaxLoc = mMaxLocArray[0];
				maxValue = maxValueArray[0];
			}

			if (maxValue < crefParams.minMatchingThr)
			{
				mDeleteLists.push_back(std::pair(idx, carId));
				continue;
			}
			/* end */

			refCarPos.x = mNearRect.x + mMaxLoc.x;
			refCarPos.y = mNearRect.y + mMaxLoc.y;
			JudgeStopTraceAndDetect(idx, carId, refCarPos); // 追跡終了判定
			//std::string path = "./template_" + std::to_string(Tk::GetFrameCount()) + "_" + std::to_string(carId) + ".png";
			//cv::imwrite(path, refTemplates[carId]);
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
		const auto& crefDetectArea = Tk::GetDetectAreaInf();
		auto carPosBottom = carPos.br();
		/* 追跡終了位置か, 新規車両かどうかの判別 */
		switch (crefRoadCarsDirection)
		{
		case RoadDirect::APPROACH:
			if (carPos.y > crefDetectArea.bottom)
				mDeleteLists.push_back(std::pair(idx, carId)); // 追跡停止判定
			break;
		case RoadDirect::LEAVE:
			if (carPosBottom.y < crefDetectArea.top)
				mDeleteLists.push_back(std::pair(idx, carId)); // 追跡停止判定
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
		auto& refFrontCarId = Tk::GetFrontCarId();
		/* 追跡終了車両をデータから除外 */
		for (const auto& [roadIdx, carId] : mDeleteLists)
		{
			refTemplatesList[roadIdx].erase(carId);
			refTemplatePositionsList[roadIdx].erase(carId);
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
		const auto& crefDetectArea = Tk::GetDetectAreaInf();
		const auto& crefParams = Tk::GetTracerParams();
		auto& refCarsNum = Tk::GetCarsNum();
		auto& refFrameCarsNum = Tk::GetFrameCarsNum();
		auto& refResultImg = Tk::GetResult();
		auto& refTemplates = Tk::GetTemplatesList()[idx];
		auto& refTemplatePositions = Tk::GetTemplatePositionsList()[idx];

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

			if (area < width * height * crefParams.minAreaRatio) // 外周や直線だけで面積を稼いでるラベルを除外
				continue;

			if (area < (y - crefDetectArea.top) / 4 + crefParams.detectAreaThr)
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
					doesntDetectCar = (finPos.y < (crefDetectArea.top + crefDetectArea.mergin + crefDetectArea.merginPad))
						|| (finPos.br().y >(crefDetectArea.bottom - crefDetectArea.mergin - crefDetectArea.merginPad));
				}
				/* end */
				else
					doesntDetectCar = IsntDetectedCars(idx, finPos); // 検出範囲にあるか判定

				if (doesntDetectCar) // 検出しない場合スキップ
					continue;

				if (DoesntAddCar(idx, finPos)) // 検出範囲にある車両に対し検出するか判定. その後, 検出しない場合スキップ
					continue;
				/* end */

				mTemp = ExtractTemplate(crefFrame, finPos);
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
		const auto& crefDetectArea = Tk::GetDetectAreaInf();
		bool doesntDetectCar = true;
		auto carPosBottom = carPos.br();

		/* 検出開始地点から遠い領域かをチェック */
		switch (crefRoadCarsDirection)
		{
		case RoadDirect::APPROACH:
			doesntDetectCar = (carPos.y < crefDetectArea.top)
				|| (carPos.y > (crefDetectArea.top + crefDetectArea.mergin));
			return doesntDetectCar;
		case RoadDirect::LEAVE:
			doesntDetectCar = (carPosBottom.y < (crefDetectArea.bottom - crefDetectArea.mergin))
				|| (carPosBottom.y > crefDetectArea.bottom);
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
	/// 新規車両として検出するか判定. 以前検出した車両との位置関係を考える.
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carPosRect">車両位置</param>
	/// <returns>判定結果, trueなら検出しない</returns>
	bool CarsTracer::DoesntAddCar(const size_t& idx, const cv::Rect2d& carPosRect)
	{
		const auto& crefDetectArea = Tk::GetDetectAreaInf();
		auto& refResultImg = Tk::GetResult();
		auto& refTemplates = Tk::GetTemplatesList()[idx];
		auto& crefTemplatePositions = Tk::GetTemplatePositionsList()[idx];
		bool retFlag = false;
		for (auto& [refCarId, refCarPos] : crefTemplatePositions)
		{
			//auto diffPosX = carPosRect.x - refCarPos.x;
			//auto diffPosY = carPosRect.y - refCarPos.y;
			//retFlag = (std::abs(diffPosX) < crefDetectArea.nearOffset)
			//	&& (std::abs(diffPosY) < crefDetectArea.nearOffset);
			//if (retFlag)
			//	return true;

			//auto carPosRectBottom = carPosRect.br();
			//auto carPosBottom = refCarPos.br();
			//diffPosX = carPosRectBottom.x  - carPosBottom.x;
			//diffPosY = carPosRectBottom.y - carPosBottom.y;
			//retFlag = (std::abs(diffPosX) < crefDetectArea.nearOffset)
			//	&& (std::abs(diffPosY) < crefDetectArea.nearOffset);
			//if (retFlag)
			//	return true;

			if (carPosRect.area() < refCarPos.area())
			{
				const auto bottom = carPosRect.br();
				auto center = cv::Point2d((carPosRect.x + bottom.x) / 2, (carPosRect.y + bottom.y) / 2);
				retFlag = refCarPos.contains(center);
				if (retFlag)
					return true;
			}
			else
			{
				const auto bottom = refCarPos.br();
				auto center = cv::Point2d((refCarPos.x + bottom.x) / 2, (refCarPos.y + bottom.y) / 2);
				if (carPosRect.contains(center))
				{
					refCarPos = carPosRect;
					refTemplates[refCarId] = ExtractTemplate(refResultImg, refCarPos);
					return true;
				}
			}
		}

		return retFlag;
	}

	/// <summary>
	/// テンプレート再抽出
	/// </summary>
	/// <param name="carPos">車両位置</param>
	void CarsTracer::ReExtractTemplate(const cv::Rect2d& carPos)
	{
		//TemplateHandle::ReLabelingTemplate(mFinCarPosList, carPos);
		TemplateHandle::ReLabelingTemplateContours(mFinCarPosList, carPos);
	}

	/// <summary>
	/// 車両矩形描画
	/// </summary>
	void CarsTracer::DrawRectangles(const size_t& idx)
	{
		auto& refResultImg = Tk::GetResult();
		auto& crefTemplatePositions = Tk::GetTemplatePositionsList()[idx];

		for (const auto& [crefCarId, crefCarPos] : crefTemplatePositions)
		{
			const auto bottom = crefCarPos.br();
			auto center = cv::Point2d((crefCarPos.x + bottom.x) / 2, (crefCarPos.y + bottom.y) / 2);
			cv::circle(refResultImg, center, 2, cv::Scalar(255, 0, 0)); // 中心点を描く
			cv::rectangle(refResultImg, crefCarPos, cv::Scalar(0, 0, 255), 2); // 矩形を描く
		}
	}
};