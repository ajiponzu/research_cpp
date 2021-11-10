#include "CarsDetector.h"

/// <summary>
/// 背景差分, 移動物体検出
/// </summary>
/// <param name="frame">入力フレーム</param>
void ImgProc::CarsDetector::SubtractBackImage(Image& frame)
{
	cv::absdiff(frame, backImg, tmp); // 差分を取ってからその絶対値を画素値として格納

	cv::cvtColor(tmp, gray, cv::COLOR_BGR2GRAY); //グレースケール化
	cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); //大津の二値化, thrは必要ないので0を渡す
	cv::cvtColor(binary, tmp, cv::COLOR_GRAY2BGR); // チャンネル数を戻す

	cv::bitwise_and(tmp, roadMask, subtracted); // マスキング処理
}

/// <summary>
/// 車影抽出
/// </summary>
void ImgProc::CarsDetector::ExtractShadow(Image& frame)
{
	// l*a*b*画像
	Image lab;

	// [0], [1], [2]にl, a, bが分割して代入される動的配列
	std::vector<Image> vLab;

	cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab); //l*a*b*に変換
	cv::split(lab, vLab); //split: チャンネルごとに分割する関数

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
	a = lab128;
	b = lab128;
	/* end */

	/* 統合処理 */
	cv::merge(vLab, shadow);
	cv::cvtColor(shadow, shadow, cv::COLOR_Lab2BGR);
	/* end */

	cv::bitwise_and(shadow, roadMask, shadow); //マスキング処理
}

/// <summary>
/// 車影再抽出
/// </summary>
/// <param name="areaThr">面積の閾値</param>
/// <param name="aspectThr">アスペクト比の閾値</param>
void ImgProc::CarsDetector::ReExtractShadow(const int& areaThr, const float& aspectThr)
{
	cv::cvtColor(shadow, gray, cv::COLOR_BGR2GRAY); // ラベリングにかけるためにはグレースケール化の必要がある
	//ラベリングによって求められるラベル数
	auto labelNum = cv::connectedComponentsWithStats(gray, labels, stats, centroids, 4);

	exceptedShadows.setTo(0); // 除外すべき影画像を0で初期化
	/* 各領域ごとの処理, 0番は背景 */
	for (int label = 1; label < labelNum; label++)
	{
		/* 統計情報分割 */
		auto statsPtr = stats.ptr<int>(label);
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
			auto idxGroup = (labels == label); // 車影出ない部分を抜き出す
			exceptedShadows.setTo(255, idxGroup); //車影でない部分を白画素で塗る
		}
	}
	/* end */

	cv::bitwise_xor(exceptedShadows, gray, reshadow); //車影のみ抽出, xorなので先ほど作ったマスク以外の部分を車影として残す
	cv::cvtColor(reshadow, reshadow, cv::COLOR_GRAY2BGR);
}

/// <summary>
/// 車両抽出
/// </summary>
void ImgProc::CarsDetector::ExtractCars()
{
	cars = subtracted - reshadow; // 移動物体から車影を除去
	cv::morphologyEx(cars, tmp, cv::MORPH_CLOSE, morphKernel);
	cv::bitwise_and(tmp, roadMask, cars);
}

/// <summary>
/// 車両を矩形で囲む
/// </summary>
/// <param name="frame">入力フレーム</param>
void ImgProc::CarsDetector::DrawRectangle(Image& frame, const int& areaThr)
{
	frame.copyTo(carRects);

	cv::cvtColor(cars, gray, cv::COLOR_BGR2GRAY); // ラベリングにかけるためにはグレースケール化の必要がある

	std::cout << roadMasks.size() << std::endl;

	/* 車線分繰り返す */
	for (int idx = 0; idx < roadMasks.size(); idx++)
	{
		cv::bitwise_and(gray, roadMasks[idx], tmp); // マスキング処理
		//ラベリングによって求められるラベル数
		auto labelNum = cv::connectedComponentsWithStats(tmp, labels, stats, centroids);

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

			if (area < areaThr)
				continue;

			/* 矩形を描く */
			auto topLeft = cv::Point(x, y);
			auto bottomRight = cv::Point(x + width, y + height);
			cv::rectangle(carRects, topLeft, bottomRight, cv::Scalar(0, 0, 255), 2);
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
void ImgProc::CarsDetector::WriteOutImgs(std::vector<std::string> pathList)
{
}

/// <summary>
/// 出力画像順次表示
/// </summary>
/// <param name="interval"></param>
void ImgProc::CarsDetector::ShowOutImgs(const int& interval)
{
	cv::imshow("detector", subtracted);
	cv::waitKey(interval);

	cv::imshow("detector", shadow);
	cv::waitKey(interval);

	cv::imshow("detector", reshadow);
	cv::waitKey(interval);

	cv::imshow("detector", cars);
	cv::waitKey(interval);

	cv::imshow("detector", carRects);
	cv::waitKey(interval * 10);
}