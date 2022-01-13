#include "BackImageHandle.h"

using Tk = ImgProc::ImgProcToolkit;

namespace ImgProc
{
	Image CarsExtractor::BackImageHandle::sSubtracted;
	Image CarsExtractor::BackImageHandle::sBackImgFloat;
	Image CarsExtractor::BackImageHandle::sFrameFloat;
	Image CarsExtractor::BackImageHandle::sMoveCarsMask;
	Image CarsExtractor::BackImageHandle::sMoveCarsMaskPrev;
	bool CarsExtractor::BackImageHandle::sIsExistPreBackImg = false;
	
	cv::VideoWriter CarsExtractor::BackImageHandle::mVideoWriterBack;

	void CarsExtractor::BackImageHandle::CreatePreBackImg()
	{
		auto& refFrame = Tk::GetFrame();
		auto& refBackImg = Tk::GetBackImg();
		auto& refFrameCount = Tk::GetFrameCount();
		const auto& crefParams = Tk::GetBackImgHandleParams();
		const auto& [crefVideoWidth, crefVideoHeight] = ImgProcToolkit::GetVideoWidAndHigh();

		auto& videoCapture = ImgProcToolkit::GetVideoCapture();
		auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
		auto videoFps = videoCapture.get(cv::CAP_PROP_FPS);

		std::string outputPath = ImgProcToolkit::GetOutputBasePath() + "_Back.mp4";
		mVideoWriterBack.open(outputPath, fourcc, videoFps, cv::Size(crefVideoWidth, crefVideoHeight));
		if (!mVideoWriterBack.isOpened())
		{
			std::cout << outputPath << ": can't create or overwrite" << std::endl;
			assert("failed to overwrite video");
		}

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
			if (refFrameCount != Tk::GetStartFrame())
				cv::bitwise_and(sMoveCarsMask, sMoveCarsMaskPrev, sMoveCarsMask);
			cv::accumulateWeighted(sFrameFloat, sBackImgFloat, crefParams.blendAlpha, sMoveCarsMask);
			sMoveCarsMaskPrev = sMoveCarsMask;

			sBackImgFloat.convertTo(refBackImg, CV_8UC3);
			mVideoWriterBack << refBackImg;
			std::cout << refFrameCount << std::endl;
			count++;
		}

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
		cv::bitwise_and(sMoveCarsMask, sMoveCarsMaskPrev, sMoveCarsMask);
		crefFrame.convertTo(sFrameFloat, CV_32FC3);
		cv::accumulateWeighted(sFrameFloat, sBackImgFloat, crefParams.blendAlpha, sMoveCarsMask);
		sBackImgFloat.convertTo(refBackImg, CV_8UC3);
		mVideoWriterBack << refBackImg;
		sMoveCarsMaskPrev = sMoveCarsMask;
		/* end */
	}
};