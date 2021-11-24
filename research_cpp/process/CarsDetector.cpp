#include "CarsDetector.h"

#include <opencv2/core/core_c.h>

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	/// <summary>
	/// 背景差分, 移動物体検出
	/// </summary>
	void CarsDetector::SubtractBackImage()
	{
		cv::absdiff(Tk::sFrame, Tk::sBackImg, mTemp); // 差分を取ってからその絶対値を画素値として格納
		binarizeImage(mTemp);
		cv::bitwise_and(mTemp, Tk::sRoadMaskGray, mSubtracted); // マスキング処理
	}

	/// <summary>
	/// 車影抽出
	/// </summary>
	void CarsDetector::ExtractShadow()
	{
		// [0], [1], [2]にl, a, bが分割して代入される動的配列
		std::vector<Image> vLab;

		cv::cvtColor(Tk::sFrame, mTemp, cv::COLOR_BGR2Lab); //l*a*b*に変換
		cv::split(mTemp, vLab); //split: チャンネルごとに分割する関数

		/* 参照型でリソース削減しつつ, わかりやすいエイリアスを定義 */
		auto& l = vLab[0];
		auto& a = vLab[1];
		auto& b = vLab[2];
		/* end */

		/* 統計量導出 */
		auto& meanA = cv::mean(a)[0]; //cv::Scalarはdoubleの二次元配列なのかも?
		auto& meanB = cv::mean(b)[0];
		cv::Scalar meanLScalar, stdLScalar;
		cv::meanStdDev(l, meanLScalar, stdLScalar);
		auto& meanL = meanLScalar[0];
		auto& stdL = stdLScalar[0];
		/* end */

		/* L値を決定する処理 */
		// np.where -> matA cmpop matB で代替
		// 戻り値はMatExprだが, Matと互換性あり
		// しかもcmpopがtrueの要素が255, それ以外の要素が0の行列として帰ってくる -> まんまwhere
		if ((meanA + meanB) <= 256)
		{
			auto thr = meanL - stdL / 3;
			l = (l <= thr);
		}
		else
		{
			int thr_l = 10, thr_b = 5;
			l = (l <= thr_l).mul(b <= thr_b);
		}
		/* end */

		/* a, b値を128で埋めてグレースケール化 */
		a = mLab128;
		b = mLab128;
		/* end */

		/* 統合処理 */
		cv::merge(vLab, mTemp);
		cv::cvtColor(mTemp, mTemp, cv::COLOR_Lab2BGR);
		/* end */

		cv::bitwise_and(mTemp, Tk::sRoadMask, mTemp); //マスキング処理
		cv::cvtColor(mTemp, mShadow, cv::COLOR_BGR2GRAY);
	}

	/// <summary>
	/// 車影再抽出
	/// </summary>
	/// <param name="areaThr">面積の閾値</param>
	/// <param name="aspectThr">アスペクト比の閾値</param>
	void CarsDetector::ReExtractShadow(const int& areaThr, const float& aspectThr)
	{
		//ラベリングによって求められるラベル数
		auto labelNum = cv::connectedComponentsWithStats(mShadow, mLabels, mStats, mCentroids, 4);

		mExceptedShadows.setTo(0); // 除外すべき影画像を0で初期化
		/* 各領域ごとの処理, 0番は背景 */
		for (int label = 1; label < labelNum; label++)
		{
			/* 統計情報分割 */
			auto statsPtr = mStats.ptr<int>(label);
			auto& y = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			auto& width = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
			auto& height = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
			auto& area = statsPtr[cv::ConnectedComponentsTypes::CC_STAT_AREA];
			/* end */

			auto aspect = static_cast<float>(width) / height; //アスペクト比の導出
			auto tAreaThr = (y - Tk::sDetectTop) / 12 + areaThr; // 位置に応じた面積の閾値

			bool condArea = area < tAreaThr;
			bool condAspect = aspect > aspectThr;
			if (condArea || condAspect)
			{
				auto idxGroup = (mLabels == label); // 車影出ない部分を抜き出す
				mExceptedShadows.setTo(255, idxGroup); //車影でない部分を白画素で塗る
			}
		}
		/* end */

		cv::bitwise_xor(mExceptedShadows, mShadow, mReShadow); //車影のみ抽出, xorなので先ほど作ったマスク以外の部分を車影として残す
	}

	/// <summary>
	/// 車両抽出
	/// </summary>
	void CarsDetector::ExtractCars()
	{
		mCars = mSubtracted - mReShadow; // 移動物体から車影を除去
		cv::morphologyEx(mCars, mTemp, cv::MORPH_CLOSE, mMorphKernel, cv::Point(-1, -1), mKernelCount);
		cv::bitwise_and(mTemp, Tk::sRoadMaskGray, mCars);
	}

	/// <summary>
	/// 車両を矩形で囲む, 未完成(未検出物体かどうかのチェックをあいまいにしている)
	/// </summary>
	void CarsDetector::DrawRectangle(const int& areaThr)
	{
		Tk::sFrame.copyTo(mCarRects);
		Tk::sCarsNumPrev = Tk::sCarsNum; // 前フレームの車両台数を保持

		/* 車線分繰り返す */
		for (size_t idx = 0; idx < Tk::sRoadMasksNum; idx++)
		{
			cv::bitwise_and(mCars, Tk::sRoadMasksGray[idx], mTemp); // マスキング処理
			//ラベリングによって求められるラベル数
			auto labelNum = cv::connectedComponentsWithStats(mTemp, mLabels, mStats, mCentroids);
			auto& templates = Tk::sTemplatesList[idx];
			auto& templatePositions = Tk::sTemplatePositionsList[idx];

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

				if (area < areaThr)
					continue;

				/* 検出位置チェック */
				cv::Rect carRect(x, y, width, height);
				bool doesntDetectCar = true;
				auto bottomY = carRect.y + carRect.height;

				/* 1フレーム目で検出されない領域を除外 */
				if (Tk::sFrameCount == 1)
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
				if (Tk::sFrameCount > 1 && doesntDetectCar)
					continue;

				/* 検出開始位置近傍の車両を特定, 未検出車両なら車両IDを保存 */
				bool continueFlag = false;
				if (!doesntDetectCar)
				{
					for (const auto& elem : Tk::sBoundaryCarIdLists[idx])
					{
						const auto& carPos = Tk::sTemplatePositionsList[idx][elem];
						auto diffPosX = carRect.x - carPos.x;
						auto diffPosY = carRect.y - carPos.y;
						auto diffWid = carRect.width - carPos.width;
						auto diffHigh = carRect.height - carPos.height;
						continueFlag = (std::abs(diffPosX) < 4) && (std::abs(diffPosY) < 4)
							|| (std::abs(diffPosX + diffWid) < 4) && (std::abs(diffPosY + diffHigh) < 4);
						if (continueFlag)
							break;
					}

					if (continueFlag)
						continue;

					Tk::sBoundaryCarIdLists[idx].insert(Tk::sCarsNum); // 1フレーム目は, 車両として検出しても, ここでIDを保存しないものもあることに注意
				}
				/* end */
				/* end */

				/* テンプレート再抽出 */
				{
					mTemp = GetImgSlice(Tk::sFrame, carRect);
					auto cutY = ExtractAreaByEdgeH(mTemp);
					auto cutPairX = ExtractAreaByEdgeV(mTemp);

					if (cutY <= (carRect.height * 0.35))
						cutY = carRect.height - 1;

					auto wid = cutPairX.second - cutPairX.first + 1;
					if (wid < (carRect.width * 0.35))
					{
						cutPairX.first = 0;
						wid = carRect.width;
					}

					carRect.x += cutPairX.first;
					carRect.width = wid;
					carRect.height = cutY + 1;
					mTemp = ExtractTemplate(Tk::sFrame, carRect);
				}
				/* end */

				cv::rectangle(mCarRects, carRect, cv::Scalar(0, 0, 255), 1); // 矩形を描く
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
		/* end */
	}

	/// <summary>
	/// 出力画像順次表示
	/// </summary>
	/// <param name="interval">待機時間[ms]</param>
	void CarsDetector::ShowOutImgs(const int& interval)
	{
		cv::imshow("detector", mSubtracted);
		cv::waitKey(interval);

		cv::imshow("detector", mShadow);
		cv::waitKey(interval);

		cv::imshow("detector", mReShadow);
		cv::waitKey(interval);

		cv::imshow("detector", mCars);
		cv::waitKey(interval);

		cv::imshow("detector", mCarRects);
		cv::waitKey(interval * 10);
	}

	/// <summary>
	/// 横方向の負エッジをy方向微分によって求め, 切りだすy座標を処理によって選択
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすy座</returns>
	int CarsDetector::ExtractAreaByEdgeH(Image& inputImg)
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
	std::pair<int, int> CarsDetector::ExtractAreaByEdgeV(Image& inputImg)
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
	void CarsDetector::SplitLineByEdge(Image& inputData, std::vector<int>& retData)
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