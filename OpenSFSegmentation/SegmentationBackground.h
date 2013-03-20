#pragma once
#include "Segmentation.h"
#include "Background.h"

namespace osf
{
	class SegmentationBackground
		: public Segmentation
	{
		MK_TYPE(SegmentationBackground);

	public:
		SegmentationBackground(System*);
		~SegmentationBackground(void);

		bool isInit() const;

		void setThreshold(float);
		void setErodingSize(int);
		void setMedianBlurSize(int);
		void setContoursFactor(float);
		float getThreshold() const;
		int getErodingSize() const;
		int getMedianBlurSize() const;
		float getContoursFactor() const;

	protected:
		void iInit();
		void iProcessSeg();

	private:
		void subtractBackground();
		void segmentContours();

		const cv::Mat* m_bgModel;
		cv::Mat m_diffImg;
		cv::Mat m_diffMask;
		cv::Mat m_procDiffMask;
		Background* m_background;

		// parameters
		float m_threshold;
		int m_erodingSize;
		int m_medianBlurSize;
		float m_contoursFactor;
	};
}
