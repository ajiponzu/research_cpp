#include "CarsTracer.h"

#include <opencv2/core/core_c.h>

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
			auto& boundaryCarIdList = Tk::sBoundaryCarIdLists[idx];

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
					if ((carPos.y + carPos.height) > Tk::sDetectBottom)
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

				/* 検出ほやほや車両かどうかの判別 */
				switch (Tk::sRoadCarsDirections[idx])
				{
				case Tk::CARS_APPROACH_ROAD:
					if (carPos.y > (Tk::sDetectTop + Tk::sDetectMergin))
						boundaryCarIdList.erase(carId);
					break;
				case Tk::CARS_LEAVE_ROAD:
					if ((carPos.y + carPos.height) < (Tk::sDetectTop - Tk::sDetectMergin))
						boundaryCarIdList.erase(carId);
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
			Tk::sBoundaryCarIdLists[roadIdx].erase(carId);
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

	/// <summary>
	/// 横方向の負エッジをy方向微分によって求め, 切りだすy座標を処理によって選択
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすy座標</returns>
	int CarsTracer::ExtractAreaByEdgeH(Image& inputImg)
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
	std::pair<int, int> CarsTracer::ExtractAreaByEdgeV(Image& inputImg)
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
	void CarsTracer::SplitLineByEdge(Image& inputData, std::vector<int>& retData)
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