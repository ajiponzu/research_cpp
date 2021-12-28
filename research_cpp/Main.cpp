#include "process/ImgProc.h"

using Tk = ImgProc::ImgProcToolkit;

/// <summary>
/// 実行時間計測, 結果出力
/// </summary>
/// <returns>テンプレ</returns>
int main()
{
	Tk::SetResourcesAndParams();
	//Tk::ShowResourcesAndParams();
	Tk::ImgProcToolkit::RunImageProcedure();

	return 0;
}