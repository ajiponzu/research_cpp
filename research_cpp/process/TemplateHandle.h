#pragma once
#include "CarsTracer.h"

class ImgProc::CarsTracer::TemplateHandle
{
private:
	static Image mLabels; //ラベル画像
	static Image mStats; //ラベリングにおける統計情報
	static Image mCentroids; //ラベリングにおける中心点座標群
	static Image mTemp1;
	static Image mTemp2;
	static Image mTemp3;
	static Image mCloseKernel; // クロージングで使用するカーネル
private:
	/// <summary>
	/// 頻度値データを, 一行n列の1チャンネル(グレースケール)画像として考え, 極大値をもつインデックスを保存
	/// </summary>
	/// <param name="inputData">入力データ, 必ずcolsをn, rowを1にしておく</param>
	/// <param name="retData">取得したいデータを格納するコンテナの参照</param>
	static void SplitLineByEdge(const Image& inputData, std::vector<int>& retData);

public:
	/// <summary>
	/// テンプレートマッチングの対象領域を制限する
	/// </summary>
	/// <param name="nearRect">制限区域矩形</param>
	/// <param name="maskId">道路マスク番号</param>
	/// <param name="carId">車両番号</param>
	static void ExtractCarsNearestArea(cv::Rect2d& nearRect, const size_t& maskId, const uint64_t& carId);

	/// <summary>
	/// テンプレートに対してもう一度ラベリングを行い, ラベルの左上座標を参照リストに入れる
	/// </summary>
	/// <param name="finCarPosList">ラベル座標を格納するために渡されたリストの参照</param>
	/// <param name="carPos">テンプレートの絶対座標</param>
	static void ReLabelingTemplate(std::vector<cv::Rect>& finCarPosList, const cv::Rect2d& carPos);

	/// <summary>
	/// 横方向の負エッジをy方向微分によって求め, 切りだすy座標を処理によって選択
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすy座標</returns>
	static int ExtractAreaByEdgeH(const Image& inputImg);

	/// <summary>
	/// 縦方向の正負両エッジをそれぞれx方向微分によって求め, 切りだすx座標二つを処理によって選択
	/// 正エッジの左端, 負エッジの右端が出力されるはず
	/// 逆の場合, 負エッジの左端, 正エッジの右端が出力される
	/// </summary>
	/// <param name="inputImg">入力テンプレート画像</param>
	/// <returns>切りだすx座標二つを一組にして返す</returns>
	static std::pair<int, int> ExtractAreaByEdgeV(const Image& inputImg);
};

