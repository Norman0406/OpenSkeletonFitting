/***********************************************************************
*
* OpenSkeletonFitting
* Skeleton fitting by the use of energy minimization
* Copyright (C) 2012 Norman Link <norman.link@gmx.net>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***********************************************************************/

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
