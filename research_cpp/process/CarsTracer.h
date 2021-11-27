#pragma once

#include "ImgProc.h"

class ImgProc::CarsTracer
{
private:
	std::vector<std::pair<size_t, uint64_t>> mDeleteLists;
	Image mTempFrame;
	Image mTemp;
	Image mDataTemp;

	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群

	cv::Point mMaxLoc;
	cv::Rect2d mNearRect;

	std::unordered_set<uint64_t> mMatchLabels;
public:
	CarsTracer() {}

	/// <summary>
	/// 車両検出
	/// </summary>
	void DetectCars();
private:
	CarsTracer(const CarsTracer& other) = delete;

	void TraceCars(const size_t& idx);

	template<class T> void MatchByLabels(cv::Rect_<T>& carPos);

	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="orgMagni">拡大・縮小倍率</param>
	/// <param name="orgMergin">車両の1フレーム後における推定移動幅</param>
	void ExtractCarsNearestArea(const size_t& maskId, const uint64_t& carId, const double& orgMagni = 1.001, const int& orgMergin = 8);

	/// <summary>
	/// 横方向の負エッジをy方向微分によって求め, 切りだすy座標を処理によって選択
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすy座標</returns>
	int ExtractAreaByEdgeH(const Image& inputImg);

	/// <summary>
	/// 縦方向の正負両エッジをそれぞれx方向微分によって求め, 切りだすx座標二つを処理によって選択
	/// 正エッジの左端, 負エッジの右端が出力されるはず
	/// 逆の場合, 負エッジの左端, 正エッジの右端が出力される
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすx座標二つを一組にして返す</returns>
	std::pair<int, int> ExtractAreaByEdgeV(const Image& inputImg);

	/// <summary>
	/// 頻度値データを, 一行n列の1チャンネル(グレースケール)画像として考え, 極大値をもつインデックスを保存
	/// </summary>
	/// <param name="inputData">入力データ, 必ずcolsをn, rowを1にしておく</param>
	/// <param name="retData">取得したいデータを格納するコンテナの参照</param>
	void SplitLineByEdge(const Image& inputData, std::vector<int>& retData);
};
