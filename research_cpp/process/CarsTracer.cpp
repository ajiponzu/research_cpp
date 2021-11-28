#include "CarsTracer.h"

#include <opencv2/core/core_c.h>

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
			mLabelNum = cv::connectedComponentsWithStats(mTemp,
				mLabels, mStats, mCentroids); // ラベリング

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
			ExtractCarsNearestArea(idx, carId, magni, mergin);
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

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="orgMagni">拡大・縮小倍率</param>
	/// <param name="orgMergin">車両の1フレーム後における推定移動幅</param>
	void CarsTracer::ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId, const double& orgMagni, const int& orgMergin)
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
		//rect.width *= magni;
		//rect.height *= magni;
		//cv::resize(carTemplate, carTemplate, cv::Size(), magni, magni, cv::INTER_NEAREST_EXACT);
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

		/* 制限区域確定 */
		mNearRect.x = findRectX;
		mNearRect.y = findRectY;
		mNearRect.width = findRectWidth;
		mNearRect.height = findRectHeight;
		/* end */
	}

	/// <summary>
	/// 横方向の負エッジをy方向微分によって求め, 切りだすy座標を処理によって選択
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすy座標</returns>
	int CarsTracer::ExtractAreaByEdgeH(const Image& inputImg)
	{
		Image gray, temp, sumElems;

		/* エッジ抽出部. cv関数では, 場合によってsrc=dstのように引数を与えると動作しないことがあるので注意 */
		cv::cvtColor(inputImg, temp, cv::COLOR_BGR2GRAY);
		cv::Sobel(temp, gray, CV_32F, 0, 1);
		/* end */

		/* 負のエッジのみ抽出 */
		auto whereMask = gray > 0; // 0より大きい要素の添え字を残すマスクを作成
		gray.setTo(cv::Scalar(0), whereMask); // マスク部分の添え字のみ要素を0にする
		cv::convertScaleAbs(gray, temp); // 各要素の絶対値をとって8bitに変換するらしい
		/* end */

		/* 二値化とエッジの頻度計算 */
		cv::threshold(temp, gray, 0, 1, cv::THRESH_BINARY | cv::THRESH_OTSU); // 二値化
		cv::reduce(gray, sumElems, 1, CV_REDUCE_SUM, CV_32F); // ビット深度をCV_8Uにすると何故か動作しない, 公式ドキュメントでは全部CV_32Fを指定
		sumElems = sumElems.reshape(1, 1); // 各y(行)について値を計算し収縮すると, n行1列になる. 1行n列の方が配列っぽいので直す. チャンネルは1のままにする
		sumElems.convertTo(sumElems, CV_8U); // で, CV_32Fだと二値化が動かないので, CV_8Uに戻す
		/* end */

		/* 頻度の二値化マスクと積算し, 有力なy座標のみ残す */
		cv::threshold(sumElems, temp, 0, 1, cv::THRESH_BINARY | cv::THRESH_OTSU);
		sumElems = temp.mul(sumElems);
		/* end */

		std::vector<int> yFreqs;
		SplitLineByEdge(sumElems, yFreqs);

		Image output;
		cv::Point a(0, 0), b(inputImg.cols, 0);
		output = inputImg.clone();
		for (const auto& elem : yFreqs)
		{
			a.y = elem;
			b.y = elem;
			cv::line(output, a, b, cv::Scalar(0, 0, 255));
		}
		const std::string path = Tk::sTemplatesPathList[Tk::sVideoType] + "template_" + std::to_string(Tk::sCarsNum) + ".png";
		cv::imwrite(path, output);

		return *yFreqs.rbegin(); // 縦方向は最下部のエッジがわかればよいので, 候補の一番最後の添え字が最下部のy座標
	}

	/// <summary>
	/// 縦方向の正負両エッジをそれぞれx方向微分によって求め, 切りだすx座標二つを処理によって選択
	/// 正エッジの左端, 負エッジの右端が出力されるはず
	/// 逆の場合, 負エッジの左端, 正エッジの右端が出力される
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすx座標二つを一組にして返す</returns>
	std::pair<int, int> CarsTracer::ExtractAreaByEdgeV(const Image& inputImg)
	{
		Image gray[2], temp[2], sumElems[2];
		/* エッジ抽出部. cv関数では, 場合によってsrc=dstのように引数を与えると動作しないことがあるので注意 */
		cv::cvtColor(inputImg, temp[0], cv::COLOR_BGR2GRAY);
		cv::Sobel(temp[0], gray[0], CV_32F, 1, 0);
		gray[0].copyTo(gray[1]);
		/* end */

		/* 正負のエッジをそれぞれ抽出 */
		/* 負のエッジ */
		auto whereMask = gray[0] > 0; // 0より大きい要素の添え字を残すマスクを作成
		gray[0].setTo(cv::Scalar(0), whereMask); // マスク部分の添え字のみ要素を0にする
		cv::convertScaleAbs(gray[0], temp[0]); // 各要素の絶対値をとって8bitに変換するらしい
		/* end */
		/* 正のエッジ */
		whereMask = gray[1] < 0;
		gray[1].setTo(cv::Scalar(0), whereMask);
		gray[1].convertTo(temp[1], CV_8U);
		/* end */
		/* end */

		/* 二値化とエッジの頻度計算 */
		/* 負エッジについて */
		cv::threshold(temp[0], gray[0], 0, 1, cv::THRESH_BINARY | cv::THRESH_OTSU); // 二値化
		cv::reduce(gray[0], sumElems[0], 0, CV_REDUCE_SUM, CV_32F); // ビット深度をCV_8Uにすると何故か動作しない, 公式ドキュメントでは全部CV_32Fを指定
		sumElems[0].convertTo(sumElems[0], CV_8U); // で, CV_32Fだと二値化が動かないので, CV_8Uに戻す
		/* end */
		/* 正エッジについて */
		cv::threshold(temp[1], gray[1], 0, 1, cv::THRESH_BINARY | cv::THRESH_OTSU); // 二値化
		cv::reduce(gray[1], sumElems[1], 0, CV_REDUCE_SUM, CV_32F); // ビット深度をCV_8Uにすると何故か動作しない, 公式ドキュメントでは全部CV_32Fを指定
		sumElems[1].convertTo(sumElems[1], CV_8U); // で, CV_32Fだと二値化が動かないので, CV_8Uに戻す
		/* end */
		/* end */

		/* 頻度の二値化マスクと積算し, 有力なx座標のみ残す */
		/* 負エッジについて */
		cv::threshold(sumElems[0], temp[0], 0, 1, cv::THRESH_BINARY | cv::THRESH_OTSU);
		sumElems[0] = temp[0].mul(sumElems[0]);
		/* end */
		/* 正エッジについて */
		cv::threshold(sumElems[1], temp[1], 0, 1, cv::THRESH_BINARY | cv::THRESH_OTSU);
		sumElems[1] = temp[1].mul(sumElems[1]);
		/* end */
		/* end */

		std::vector<int> xFreqs[2];
		SplitLineByEdge(sumElems[0], xFreqs[0]);
		SplitLineByEdge(sumElems[1], xFreqs[1]);

		/* 左端・右端チェック */
		auto ret = std::make_pair(*xFreqs[1].begin(), *xFreqs[0].rbegin());
		/* 正エッジの左端と負エッジの右端の大小チェックを行い, 逆なら適切な値を再代入する */
		if (ret.first > ret.second)
		{
			ret.first = *xFreqs[0].begin();
			ret.second = *xFreqs[1].rbegin();
		}
		/* end */
		/* end */

		return ret;
	}

	/// <summary>
	/// 頻度値データを, 一行n列の1チャンネル(グレースケール)画像として考え, 極大値をもつインデックスを保存
	/// </summary>
	/// <param name="inputData">入力データ, 必ずcolsをn, rowを1にしておく</param>
	/// <param name="retData">取得したいデータを格納するコンテナの参照</param>
	void CarsTracer::SplitLineByEdge(const Image& inputData, std::vector<int>& retData)
	{
		int nearMax = -1, nearMaxIdx = -1; // あえて自然数にとっての最小値?をいれておく
		bool flag = false; // 0から単調増加するとtrue, 0以外の値から0に単調減少するとfalseになる

		const auto& ptrInputData = inputData.data;
		for (int idx = 0; idx < inputData.cols; idx++)
		{
			const auto& elem = ptrInputData[idx];
			/* 登山中 */
			if (flag)
			{
				/* 0を平野とすると, 値の山が終わったとき */
				if (elem <= 0)
				{
					retData.push_back(nearMaxIdx); // 極大値をもつ添え字を保存
					flag = false;
					nearMax = -1; // 最大値をまたみつけるために, 最小値でリセット
				}
				/* end */
				/* 山を探索中, より大きな峰をみつけた */
				else if (elem > nearMax)
				{
					nearMaxIdx = idx; // 極大値を与える添え字を更新
					nearMax = elem; // 極大値の更新
				}
				/* end */
			}
			/* end */
			/* 平野を散歩中に山の麓をみつけた */
			else if (elem > 0)
			{
				flag = true;
				nearMaxIdx = idx;
				nearMax = elem;
			}
			/* end */
		}
	}
};