#include "CarsDetector.h"

namespace ImgProc
{
	/// <summary>
	/// 背景差分, 移動物体検出
	/// </summary>
	void CarsDetector::SubtractBackImage()
	{
		cv::absdiff(ImgProcToolkit::sFrame, ImgProcToolkit::sBackImg, mTemp); // 差分を取ってからその絶対値を画素値として格納
		binarizeImage(mTemp);
		cv::bitwise_and(mTemp, ImgProcToolkit::sRoadMaskGray, mSubtracted); // マスキング処理
	}

	/// <summary>
	/// 車影抽出
	/// </summary>
	void CarsDetector::ExtractShadow()
	{
		// [0], [1], [2]にl, a, bが分割して代入される動的配列
		std::vector<Image> vLab;

		cv::cvtColor(ImgProcToolkit::sFrame, mTemp, cv::COLOR_BGR2Lab); //l*a*b*に変換
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

		cv::bitwise_and(mTemp, ImgProcToolkit::sRoadMask, mTemp); //マスキング処理
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
			auto tAreaThr = (y - 230) / 12 + areaThr; // 位置に応じた面積の閾値

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
		cv::morphologyEx(mCars, mTemp, cv::MORPH_CLOSE, mMorphKernel);
		cv::bitwise_and(mTemp, ImgProcToolkit::sRoadMaskGray, mCars);
	}

	/// <summary>
	/// 車両を矩形で囲む
	/// </summary>
	void CarsDetector::DrawRectangle(const int& areaThr)
	{
		ImgProcToolkit::sFrame.copyTo(mCarRects);

		/* 車線分繰り返す */
		for (int idx = 0; idx < ImgProcToolkit::sRoadMasksNum; idx++)
		{
			cv::bitwise_and(mCars, ImgProcToolkit::sRoadMasksGray[idx], mTemp); // マスキング処理
			//ラベリングによって求められるラベル数
			auto labelNum = cv::connectedComponentsWithStats(mTemp, mLabels, mStats, mCentroids);

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

				/* 矩形を描く */
				auto topLeft = cv::Point(x, y);
				auto bottomRight = cv::Point(x + width, y + height);
				cv::rectangle(mCarRects, topLeft, bottomRight, cv::Scalar(0, 0, 255), 3);
				/* end */

				//bool doesDetectCar = true;
				//switch (ImgProcToolkit::sRoadCarsDirections[idx])
				//{
				//case ImgProcToolkit::CARS_APPROACH_ROAD:
				//{
				//	doesDetectCar
				//}
				//}
				//if (ImgProcToolkit::sFrameCount == 1)
				//{
				//}
				//else
				//{

				//}
				/* テンプレート抽出等 */
				ImgProcToolkit::sTemplatePositionsList.push_back(std::move(topLeft));
				ImgProcToolkit::sTemplatesList.push_back(ExtractTemplate(ImgProcToolkit::sFrame, x, y, width, height));
				/* end */
			}
			/* end */
		}
		/* end */
	}

	/// <summary>
	/// 出力画像一括保存
	/// </summary>
	/// <param name="pathList">パスの集合, 処理順にパス名を保存しておくこと</param>
	void CarsDetector::WriteOutImgs(std::vector<std::string> pathList)
	{
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