#pragma once
#include "ImgProc.h"

class ImgProc::CarsTracer
{
private:
	class TemplateHandle;

	std::vector<std::pair<size_t, uint64_t>> mDeleteLists;
	Image mTempFrame;
	Image mTemp;
	Image mDataTemp;

	Image mLabels; //ラベル画像
	Image mStats; //ラベリングにおける統計情報
	Image mCentroids; //ラベリングにおける中心点座標群

	int mLabelNum = 0; // ラベル数

	cv::Point mMaxLoc;
	cv::Point mMaxLocArray[2]{};
	cv::Rect2d mNearRect;

	std::vector<cv::Rect> mFinCarPosList;
public:
	CarsTracer();

	/// <summary>
	/// 車両検出
	/// </summary>
	void DetectCars();
private:
	CarsTracer(const CarsTracer& other) = delete;

	/// <summary>
	/// 車両追跡
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	void TraceCars(const size_t& idx);

	/// <summary>
	/// 車両追跡の停止・新規検出車両判定の停止を判断する
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	/// <param name="carPos">車両位置</param>
	void JudgeStopTraceAndDetect(const size_t& idx, const uint64_t& carId, const cv::Rect2d& carPos);

	/// <summary>
	/// 追跡停止処理
	/// </summary>
	void DestructTracedCars();

	/// <summary>
	/// 車両検出, 開始フレームとそれ以降で検出範囲が変化
	/// </summary>
	/// <param name="idx"></param>
	void DetectNewCars(const size_t& idx);

	/// <summary>
	/// 車両が検出範囲に存在するか判定
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carPos">車両位置</param>
	/// <returns>判定結果, trueなら検出しない</returns>
	bool IsntDetectedCars(const size_t& idx, const cv::Rect2d& carPos);

	/// <summary>
	/// 新規車両として検出するか判定. 以前検出した車両との位置関係を考える.
	/// </summary>
	/// <param name="idx">道路マスク番号</param>
	/// <param name="carPosRect">車両位置</param>
	/// <returns>判定結果, trueなら検出しない</returns>
	bool DoesntAddCar(const size_t& idx, const cv::Rect2d& carPosRect);

	/// <summary>
	/// テンプレート再抽出
	/// </summary>
	/// <param name="carPos">車両位置</param>
	void ReExtractTemplate(const cv::Rect2d& carPos);

	/// <summary>
	/// 車両矩形描画
	/// </summary>
	void DrawRectangles(const size_t& idx);
};
