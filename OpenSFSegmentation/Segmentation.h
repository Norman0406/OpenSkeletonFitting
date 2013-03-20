#pragma once
#include "../OpenSF/Module.h"
#include <opencv2/opencv.hpp>

namespace osf
{
	class Segmentation
		: public Module
	{
	public:
		Segmentation(System*);
		virtual ~Segmentation(void);

		virtual bool isInit() const;

		const cv::Mat& getImgSegDepth() const;
		const cv::Mat& getImgSeg3d() const;

	protected:
		virtual void iInit();
		virtual void iProcessSeg() = 0;
		void iProcess();

		const cv::Mat* m_inImgDepth;
		const cv::Mat* m_inImg3d;
		cv::Mat m_segImgDepth;
		cv::Mat m_segImg3d;
	};
}