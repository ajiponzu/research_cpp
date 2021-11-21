#include "CarsDetector.h"

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

					if (!continueFlag)
						Tk::sBoundaryCarIdLists[idx].insert(Tk::sCarsNum); // 1フレーム目は, 車両として検出しても, ここでIDを保存しないものもあることに注意
					else
						continue;
				}
				/* end */
				/* end */

				cv::rectangle(mCarRects, carRect, cv::Scalar(0, 0, 255), 1); // 矩形を描く
				cv::rectangle(Tk::sResutImg, carRect, cv::Scalar(0, 0, 255), 1); // 矩形を描く

				/* テンプレート抽出等 */
				templates.insert(std::pair(Tk::sCarsNum, ExtractTemplate(Tk::sFrame, carRect)));
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
};