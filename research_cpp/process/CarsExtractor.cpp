#include "CarsExtractor.h"
#include "BackImageHandle.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	void CarsExtractor::ExtractCars()
	{
		SubtractBackImage();
		ExtractShadow();
		ReExtractShadow();

		const auto& crefParams = Tk::GetExtractorParams();
		auto& refCarsImg = Tk::GetCars();
		const auto& crefRoadMaskGray = Tk::GetRoadMaskGray();
		auto& refMorphPrevCars = Tk::GetMorphPrevCars();
		refMorphPrevCars = mSubtracted - mReShadow; // 移動物体から車影を除去
		cv::morphologyEx(refMorphPrevCars, refCarsImg, cv::MORPH_CLOSE, mCloseKernel, cv::Point(-1, -1), crefParams.closeCount);
		cv::bitwise_and(refCarsImg, crefRoadMaskGray, refCarsImg);
		OutputProcessVideo();
	}

	/// <summary>
	/// 初期背景画像を作成(500フレーム使用)
	/// </summary>
	void CarsExtractor::InitBackgroundImage()
	{
		BackImageHandle::CreatePreBackImg();
	}

	/// <summary>
	/// 背景差分, 移動物体検出
	/// </summary>
	void CarsExtractor::SubtractBackImage()
	{
		BackImageHandle::UpdateBackground();
		const auto& crefSubtracted = BackImageHandle::GetSubtracted();
		const auto& crefRoadMaskGray = Tk::GetRoadMaskGray();
		cv::bitwise_and(crefSubtracted, crefRoadMaskGray, mSubtracted); // マスキング処理
	}

	/// <summary>
	/// 車影抽出
	/// </summary>
	void CarsExtractor::ExtractShadow()
	{
		const auto& crefFrame = Tk::GetFrame();
		const auto& crefParams = Tk::GetExtractorParams();
		const auto& crefFrameCount = Tk::GetFrameCount();

		// [0], [1], [2]にl, a, bが分割して代入される動的配列
		std::vector<Image> vLab;

		cv::cvtColor(crefFrame, mTemp, cv::COLOR_BGR2Lab); //l*a*b*に変換
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
			l = (l <= crefParams.shadowThrL).mul(b <= crefParams.shadowThrB);
		/* end */

		/* a, b値を128で埋めてグレースケール化 */
		a = mLab128;
		b = mLab128;
		/* end */

		/* 統合処理 */
		cv::merge(vLab, mTemp);
		cv::cvtColor(mTemp, mTemp, cv::COLOR_Lab2BGR);
		/* end */

		cv::cvtColor(mTemp, mTemp, cv::COLOR_BGR2GRAY);
		if (crefFrameCount == 501 /*|| crefFrameCount == || ...*/)
		{
			std::string path = "./all_shadow" + std::to_string(crefFrameCount) + ".png";
			cv::imwrite(path, mTemp);
		}
		cv::bitwise_and(mTemp, mSubtracted, mShadow);
	}

	/// <summary>
	/// 車影再抽出
	/// </summary>
	/// <param name="areaThr">面積の閾値</param>
	/// <param name="aspectThr">アスペクト比の閾値</param>
	void CarsExtractor::ReExtractShadow()
	{
		//ラベリングによって求められるラベル数
		auto labelNum = cv::connectedComponentsWithStats(mShadow, mLabels, mStats, mCentroids, 8);
		const auto& crefDetectArea = Tk::GetDetectAreaInf();
		const auto& crefParams = Tk::GetExtractorParams();
		mShadow.copyTo(mReShadow);

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

			auto aspect = static_cast<float>(width) / height; //アスペクト比の導出

			if (aspect > crefParams.reshadowAspectThr)
			{
				auto idxGroup = (mLabels == label); // 車影でない部分をマスクとして抜き出す
				mReShadow.setTo(0, idxGroup); // 車影でない部分を0埋め
			}
		}
		/* end */
	}

	/// <summary>
	/// 出力画像順次表示
	/// </summary>
	/// <param name="interval">待機時間[ms]</param>
	void CarsExtractor::ShowOutImgs(const int& interval)
	{
		cv::imshow("detector", mSubtracted);
		cv::waitKey(interval);

		cv::imshow("detector", mShadow);
		cv::waitKey(interval);

		cv::imshow("detector", mReShadow);
		cv::waitKey(interval);
	}

	/// <summary>
	/// 書く処理過程の結果画像を出力
	/// </summary>
	void CarsExtractor::OutputProcessVideo()
	{
		cv::cvtColor(mSubtracted, mTemp, cv::COLOR_GRAY2BGR);
		mVideoWriterSub << mTemp;

		cv::cvtColor(mShadow, mTemp, cv::COLOR_GRAY2BGR);
		mVideoWriterShadow << mTemp;

		cv::cvtColor(mReShadow, mTemp, cv::COLOR_GRAY2BGR);
		mVideoWriterReShadow << mTemp;

		cv::cvtColor(Tk::GetMorphPrevCars(), mTemp, cv::COLOR_GRAY2BGR);
		mVideoWriterPreCars << mTemp;

		cv::cvtColor(Tk::GetCars(), mTemp, cv::COLOR_GRAY2BGR);
		mVideoWriterCars << mTemp;
	}
};