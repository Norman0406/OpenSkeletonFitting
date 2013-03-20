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

#include "precompiled.h"
#include "SegmentationBackground.h"
#include "Background.h"
#include "BackgroundFirst.h"
#include "../OpenSF/Factory.h"

namespace osf
{
	SegmentationBackground::SegmentationBackground(System* sys)
		: Segmentation(sys)
	{
		m_threshold = 0.15f;
		m_erodingSize = 3;
		m_medianBlurSize = 3;
		m_contoursFactor = 300.0f;
	}

	SegmentationBackground::~SegmentationBackground(void)
	{
	}

	void SegmentationBackground::iInit()
	{
		Segmentation::iInit();

		// create background and assign input and output data
		m_background = Factory<Background>::createByType((const type_info&)BackgroundFirst::getType(), m_system);
		m_background->getInputData<cv::Mat>(0) = m_inImgDepth;
		m_bgModel = m_background->getOutputData<cv::Mat>(0);
		m_background->init();

		m_diffImg = m_bgModel->clone();
		m_diffMask = cv::Mat(m_diffImg.rows, m_diffImg.cols, CV_8UC1);
		m_procDiffMask = cv::Mat(m_diffImg.rows, m_diffImg.cols, CV_8UC1);
	}

	bool SegmentationBackground::isInit() const
	{
		return m_background && m_background->isInit() &&
			m_bgModel &&
			Segmentation::isInit();
	}
	
	void SegmentationBackground::setThreshold(float val)
	{
		m_threshold = val;
	}

	void SegmentationBackground::setErodingSize(int val)
	{
		m_erodingSize = val;
	}

	void SegmentationBackground::setMedianBlurSize(int val)
	{
		m_medianBlurSize = val;
	}

	void SegmentationBackground::setContoursFactor(float val)
	{
		m_contoursFactor = val;
	}

	float SegmentationBackground::getThreshold() const
	{
		return m_threshold;
	}

	int SegmentationBackground::getErodingSize() const
	{
		return m_erodingSize;
	}

	int SegmentationBackground::getMedianBlurSize() const
	{
		return m_medianBlurSize;
	}

	float SegmentationBackground::getContoursFactor() const
	{
		return m_contoursFactor;
	}
	
	void SegmentationBackground::subtractBackground()
	{
		// subtract running image from background
		for (int i = 0; i < m_bgModel->cols; i++) {
			for (int j = 0; j < m_bgModel->rows; j++) {
				float bg = m_bgModel->ptr<float>(j)[i];
				float depth = m_inImgDepth->ptr<float>(j)[i];
				float* diff = &m_diffImg.ptr<float>(j)[i];

				if (depth == 0)
					*diff = 0;
				else if (bg == 0)
					*diff = depth;
				else {
					*diff = fabs(bg - depth);
				}
			}
		}

		// filter out low values
		cv::Mat diffMask;
		cv::threshold(m_diffImg, diffMask, m_threshold, 255, CV_THRESH_BINARY);
		diffMask.convertTo(m_diffMask, CV_8U);
		
		m_segImgDepth.setTo(0);
		m_inImgDepth->copyTo(m_segImgDepth, m_diffMask);

		// closing
		cv::erode(m_segImgDepth, m_segImgDepth, cv::Mat(m_erodingSize, m_erodingSize, CV_8UC1));

		// median filtering
		cv::medianBlur(m_segImgDepth, m_segImgDepth, m_medianBlurSize);

		cv::Mat mask = m_segImgDepth.clone();
		cv::threshold(m_segImgDepth, mask, 0, 255, CV_THRESH_BINARY);
		mask.convertTo(m_procDiffMask, CV_8U);
	}

	void SegmentationBackground::segmentContours()
	{
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(m_procDiffMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		int index = -1;
		int size = 0;
		
		// the smaller the factor, the bigger the shape needs to be to be detected
		const int minSize = (int)((m_bgModel->cols * m_bgModel->rows) / m_contoursFactor);

		for (int i = 0; i < (int)contours.size(); i++) {
			int curSize = contours[i].size();
			if (curSize > size && curSize >= minSize) {
				size = curSize;
				index = i;
			}
		}

		// create a mask of the biggest contour
		m_procDiffMask.setTo(0);
		if (index >= 0)
			cv::drawContours(m_procDiffMask, contours, index, cv::Scalar(255), CV_FILLED);

		// use this mask to generate final segmented image
		cv::Mat temp;
		m_segImgDepth.copyTo(temp, m_procDiffMask);
		temp.copyTo(m_segImgDepth);

		// mask out 3d image
		m_segImg3d.setTo(0);
		m_inImg3d->copyTo(m_segImg3d, m_procDiffMask);
	}

	void SegmentationBackground::iProcessSeg()
	{
		if (!isInit())
			throw Exception("not init");

		m_background->process();

		subtractBackground();
		segmentContours();
	}
}
