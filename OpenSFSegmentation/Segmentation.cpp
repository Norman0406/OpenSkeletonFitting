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
#include <map>
#include <time.h>
#include "Segmentation.h"
#include "SegmentationFactory.h"

namespace osf
{
	Segmentation::Segmentation(System* sys)
		: Module(sys), m_inImgDepth(0), m_inImg3d(0)
	{
		m_segImgDepth.setTo(0);
		m_segImg3d.setTo(0);

		addInput(m_inImgDepth);
		addInput(m_inImg3d);
		addOutput(&m_segImgDepth);
		addOutput(&m_segImg3d);
	}

	Segmentation::~Segmentation(void)
	{
	}

	void Segmentation::iInit()
	{
		if (!m_inImgDepth || !m_inImg3d)
			throw Exception("invalid input data");

		m_segImgDepth = m_inImgDepth->clone();
		m_segImgDepth.setTo(0);

		m_segImg3d = m_inImg3d->clone();
		m_segImg3d.setTo(0);
	}

	bool Segmentation::isInit() const
	{
		return m_inImgDepth && !m_inImgDepth->empty() &&
			m_inImg3d && !m_inImg3d->empty() &&
			Module::isInit();
	}

	const cv::Mat& Segmentation::getImgSegDepth() const
	{
		return m_segImgDepth;
	}

	const cv::Mat& Segmentation::getImgSeg3d() const
	{
		return m_segImg3d;
	}

	void Segmentation::iProcess()
	{
		if (!isInit())
			throw Exception("not init");

		iProcessSeg();
	}
}
