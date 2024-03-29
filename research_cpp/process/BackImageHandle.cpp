#include "BackImageHandle.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	Image CarsExtractor::BackImageHandle::sSubtracted;
	Image CarsExtractor::BackImageHandle::sBackImgFloat;
	Image CarsExtractor::BackImageHandle::sFrameFloat;
	Image CarsExtractor::BackImageHandle::sMoveCarsMask;
	bool CarsExtractor::BackImageHandle::sIsExistPreBackImg = false;

	void CarsExtractor::BackImageHandle::CreatePreBackImg()
	{
		auto& refFrame = Tk::GetFrame();
		auto& refBackImg = Tk::GetBackImg();
		auto& refFrameCount = Tk::GetFrameCount();
		const auto& crefParams = Tk::GetBackImgHandleParams();

		auto& videoCapture = ImgProcToolkit::GetVideoCapture();
		auto fgbg = cv::createBackgroundSubtractorMOG2();

		videoCapture >> refFrame;
		refFrame.convertTo(sBackImgFloat, CV_32FC3);
		sBackImgFloat.setTo(0.0);

		uint64_t count = 0;
		for (int count = 1; count <= fgbg->getHistory(); count++)
		while (count <= fgbg->getHistory())
		{
			refFrameCount++;
			videoCapture >> refFrame;
			if (refFrame.empty())
				break;

			if (refFrameCount < Tk::GetStartFrame())
				continue;
			else if (refFrameCount > Tk::GetEndFrame())
				break;

			refFrame.convertTo(sFrameFloat, CV_32FC3);

			fgbg->apply(refFrame, sMoveCarsMask);
			binarizeImage(sMoveCarsMask);
			cv::bitwise_not(sMoveCarsMask, sMoveCarsMask);
			cv::accumulateWeighted(sFrameFloat, sBackImgFloat, crefParams.blendAlpha, sMoveCarsMask);

			std::cout << refFrameCount << std::endl;
			count++;
		}

		sBackImgFloat.convertTo(refBackImg, CV_8UC3);
		sIsExistPreBackImg = true;
		Tk::GetStartFrame() += fgbg->getHistory();
	}


	void CarsExtractor::BackImageHandle::UpdateBackground()
	{
		const auto& crefParams = Tk::GetBackImgHandleParams();
		const auto& crefFrame = Tk::GetFrame();
		const auto& refBackImg = Tk::GetBackImg();
		cv::absdiff(crefFrame, refBackImg, sSubtracted); // 差分を取ってからその絶対値を画素値として格納
		binarizeImage(sSubtracted);
		
		/* 背景更新処理 */
		cv::bitwise_not(sSubtracted, sMoveCarsMask);
		crefFrame.convertTo(sFrameFloat, CV_32FC3);
		cv::accumulateWeighted(sFrameFloat, sBackImgFloat, crefParams.blendAlpha, sMoveCarsMask);
		sBackImgFloat.convertTo(refBackImg, CV_8UC3);
		/* end */
	}
};