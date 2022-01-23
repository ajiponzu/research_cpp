#include "TemplateHandle.h"

#include <opencv2/core/core_c.h>

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	/* スタティック変数 */
	Image CarsTracer::TemplateHandle::mLabels; //ラベル画像
	Image CarsTracer::TemplateHandle::mStats; //ラベリングにおける統計情報
	Image CarsTracer::TemplateHandle::mCentroids; //ラベリングにおける中心点座標群
	Image CarsTracer::TemplateHandle::mTemp1;
	Image CarsTracer::TemplateHandle::mTemp2;
	Image CarsTracer::TemplateHandle::mTemp3;
	Image CarsTracer::TemplateHandle::mCloseKernel; // クロージングで使用するカーネル
	/* end */

	/// <summary>
	/// 頻度値データを, 一行n列の1チャンネル(グレースケール)画像として考え, 極大値をもつインデックスを保存
	/// </summary>
	/// <param name="inputData">入力データ, 必ずcolsをn, rowを1にしておく</param>
	/// <param name="retData">取得したいデータを格納するコンテナの参照</param>
	void CarsTracer::TemplateHandle::SplitLineByEdge(const Image& inputData, std::vector<int>& retData)
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
		
		if (retData.size() == 0)
			retData.push_back(-1);
	}

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="nearRect">制限区域矩形</param>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	void CarsTracer::TemplateHandle::ExtractCarsNearestArea(cv::Rect2d& nearRect, const size_t& maskId, const uint64_t& carId)
	{
		const auto& crefParams = Tk::GetTemplateHandleParams();
		auto magni = crefParams.magni;
		auto& refRect = Tk::GetTemplatePositionsList()[maskId][carId];
		auto& refCarTemplate = Tk::GetTemplatesList()[maskId][carId];
		const auto& crefRoadCarsDirection = Tk::GetRoadCarsDirections()[maskId];

		/* 車両が遠ざかっていくとき */
		if (crefRoadCarsDirection == RoadDirect::LEAVE)
			magni = 1 / crefParams.magni; // 縮小するために逆数にする
		/* end */

		/* テンプレートの拡大・縮小処理と, 座標矩形の縦横の変更 */
		refRect.width *= magni;
		refRect.height *= magni;
		cv::resize(refCarTemplate, refCarTemplate, refRect.size());
		/* end */

		/* テンプレートマッチングの対象領域の限定 */
		/* 原点の設定 */
		const auto& [crefVideoWidth, crefVideoHeight] = Tk::GetVideoWidAndHigh();
		auto findRectX = std::round(std::clamp((refRect.x - crefParams.mergin), 0.0, static_cast<double>(crefVideoWidth)));
		auto findRectY = std::round(std::clamp((refRect.y - crefParams.mergin), 0.0, static_cast<double>(crefVideoHeight)));
		/* end */

		/* 移動後に予想される到達地点の最大値を算出 */
		auto findRectXR = std::round(std::clamp(findRectX + refRect.width + crefParams.mergin * 2.0, 0.0, static_cast<double>(crefVideoWidth)));
		auto findRectYB = std::round(std::clamp(findRectY + refRect.height + crefParams.mergin * 2.0, 0.0, static_cast<double>(crefVideoHeight)));
		/* end */

		/* 最終的な探索領域の縦横の幅を算出 */
		auto findRectWidth = std::abs(findRectXR - findRectX) + 1.0;
		auto findRectHeight = std::abs(findRectYB - findRectY) + 1.0;
		/* end */
		/* end */

		/* 制限区域確定 */
		nearRect.x = findRectX;
		nearRect.y = findRectY;
		nearRect.width = findRectWidth;
		nearRect.height = findRectHeight;
		/* end */
	}

	/// <summary>
	/// テンプレートに対してもう一度ラベリングを行い, ラベルの左上座標を参照リストに入れる
	/// </summary>
	/// <param name="finCarPosList">ラベル座標を格納するために渡されたリストの参照</param>
	/// <param name="carPos">テンプレートの絶対座標</param>
	void CarsTracer::TemplateHandle::ReLabelingTemplate(std::vector<cv::Rect>& finCarPosList, const cv::Rect2d& carPos)
	{
		const auto& crefFrame = Tk::GetFrame();
		const auto& crefBackImg = Tk::GetBackImg();
		const auto& crefParams = Tk::GetTemplateHandleParams();
		const auto& crefDetectArea = Tk::GetDetectAreaInf();

		mTemp3 = ExtractTemplate(crefFrame, carPos);
		mTemp1 = ExtractTemplate(crefBackImg, carPos);

		cv::absdiff(mTemp3, mTemp1, mTemp2);
		binarizeImage(mTemp2);
		cv::morphologyEx(mTemp2, mTemp3, cv::MORPH_CLOSE, mCloseKernel, cv::Point(-1, -1), crefParams.closeCount);

		//ラベリングによって求められるラベル数
		auto labelNum = cv::connectedComponentsWithStats(mTemp3, mLabels, mStats, mCentroids, 8);
		/* 各領域ごとの処理, 0番は背景 */
		for (int label = 1; label < labelNum; label++)
		{
			/* 統計情報分割 */
			auto statsPtr = mStats.ptr<int>(label);
			auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
			/* end */

			auto tAreaThr = (carPos.y - crefDetectArea.top) / 4 + crefParams.areaThr; // 位置に応じた面積の閾値
			if (area < tAreaThr)
				continue;

			if (area < width * height * crefParams.minAreaRatio) // 外周や直線だけで面積を稼いでるラベルを除外
				continue;

			finCarPosList.push_back(cv::Rect(static_cast<int>(carPos.x) + x, static_cast<int>(carPos.y) + y, width, height));
		}
		/* end */
	}

	/// <summary>
	/// テンプレートに対してもう一度ラベリングを行い, ラベルの左上座標を参照リストに入れる
	/// </summary>
	/// <param name="finCarPosList">ラベル座標を格納するために渡されたリストの参照</param>
	/// <param name="carPos">テンプレートの絶対座標</param>
	void CarsTracer::TemplateHandle::ReLabelingTemplateContours(std::vector<cv::Rect>& finCarPosList, const cv::Rect2d& carPos)
	{
		const auto& crefFrame = Tk::GetFrame();
		const auto& crefBackImg = Tk::GetBackImg();
		const auto& crefParams = Tk::GetTemplateHandleParams();
		const auto& crefDetectArea = Tk::GetDetectAreaInf();
		const auto& crefFrameCount = Tk::GetFrameCount();

		mTemp3 = ExtractTemplate(crefFrame, carPos);
		mTemp1 = ExtractTemplate(crefBackImg, carPos);

		cv::absdiff(mTemp3, mTemp1, mTemp2);
		binarizeImage(mTemp2);
		cv::morphologyEx(mTemp2, mTemp3, cv::MORPH_CLOSE, mCloseKernel, cv::Point(-1, -1), crefParams.closeCount);

		//ラベリングによって求められるラベル数
		auto labelNum = cv::connectedComponentsWithStats(mTemp3, mLabels, mStats, mCentroids, 8);
		/* 各領域ごとの処理, 0番は背景 */
		for (int label = 1; label < labelNum; label++)
		{
			/* 統計情報分割 */
			auto statsPtr = mStats.ptr<int>(label);
			auto& x = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
			/* end */

			auto tAreaThr = (carPos.y - crefDetectArea.top) / 4 + crefParams.areaThr; // 位置に応じた面積の閾値
			if (area < tAreaThr)
				continue;

			if (area < width * height * crefParams.minAreaRatio) // 外周や直線だけで面積を稼いでるラベルを除外
				continue;

			finCarPosList.push_back(cv::Rect(static_cast<int>(carPos.x) + x, static_cast<int>(carPos.y) + y, width, height));
		}
		/* end */
	}

	/// <summary>
	/// 横方向の負エッジをy方向微分によって求め, 切りだすy座標を処理によって選択
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすy座標</returns>
	int CarsTracer::TemplateHandle::ExtractAreaByEdgeH(const Image& inputImg)
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

		//Image output;
		//cv::Point a(0, 0), b(inputImg.cols, 0);
		//output = inputImg.clone();
		//for (const auto& elem : yFreqs)
		//{
		//	a.y = elem;
		//	b.y = elem;
		//	cv::line(output, a, b, cv::Scalar(0, 0, 255));
		//}
		//const std::string path = Tk::sTemplatesPathList[Tk::sVideoType] + "template_" + std::to_string(Tk::sCarsNum) + ".png";
		//cv::imwrite(path, output);

		return *yFreqs.rbegin(); // 縦方向は最下部のエッジがわかればよいので, 候補の一番最後の添え字が最下部のy座標
	}

	/// <summary>
	/// 縦方向の正負両エッジをそれぞれx方向微分によって求め, 切りだすx座標二つを処理によって選択
	/// 正エッジの左端, 負エッジの右端が出力されるはず
	/// 逆の場合, 負エッジの左端, 正エッジの右端が出力される
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすx座標二つを一組にして返す</returns>
	std::pair<int, int> CarsTracer::TemplateHandle::ExtractAreaByEdgeV(const Image& inputImg)
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
};
