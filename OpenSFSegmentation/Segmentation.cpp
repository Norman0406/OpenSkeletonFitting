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
