#include "BackImageHandle.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	Image CarsExtractor::BackImageHandle::sSubtracted;
	Image CarsExtractor::BackImageHandle::sBackImg;
	Image CarsExtractor::BackImageHandle::sBackImgFloat;
	Image CarsExtractor::BackImageHandle::sFrameFloat;
	Image CarsExtractor::BackImageHandle::sMoveCarsMask;
	bool CarsExtractor::BackImageHandle::sIsExistPreBackImg = false;

	void CarsExtractor::BackImageHandle::CreatePreBackImg()
	{
		auto& frame = Tk::sFrame;
		auto& videoCapture = ImgProcToolkit::GetVideoCapture();
		auto fgbg = cv::createBackgroundSubtractorMOG2();

		videoCapture >> frame;
		frame.convertTo(sBackImgFloat, CV_32FC3);
		sBackImgFloat.setTo(0.0);

		for (int count = 1; count <= fgbg->getHistory(); count++)
		{
			videoCapture >> frame;
			if (frame.empty())
				break;

			frame.convertTo(sFrameFloat, CV_32FC3);

			fgbg->apply(frame, sMoveCarsMask);
			binarizeImage(sMoveCarsMask);
			cv::bitwise_not(sMoveCarsMask, sMoveCarsMask);
			cv::accumulateWeighted(sFrameFloat, sBackImgFloat, 0.025, sMoveCarsMask);
		}

		sBackImgFloat.convertTo(sBackImg, CV_8UC3);
		sIsExistPreBackImg = true;
	}


	void CarsExtractor::BackImageHandle::UpdateBackground()
	{
		cv::absdiff(Tk::sFrame, sBackImg, sSubtracted); // 差分を取ってからその絶対値を画素値として格納
		binarizeImage(sSubtracted);
		
		/* 背景更新処理 */
		cv::bitwise_not(sSubtracted, sMoveCarsMask);
		Tk::sFrame.convertTo(sFrameFloat, CV_32FC3);
		cv::accumulateWeighted(sFrameFloat, sBackImgFloat, 0.025, sMoveCarsMask);
		sBackImgFloat.convertTo(sBackImg, CV_8UC3);
		/* end */
	}
};