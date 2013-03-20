#include "precompiled.h"
#include "BackgroundFirst.h"

namespace osf
{
	BackgroundFirst::BackgroundFirst(System* sys)
		: Background(sys)
	{
		m_firstRun = true;
	}

	BackgroundFirst::~BackgroundFirst(void)
	{
	}

	void BackgroundFirst::iProcessBG()
	{
		if (m_firstRun) {
			m_inImgDepth->copyTo(m_bgModel);
			m_firstRun = false;
		}
	}
}
