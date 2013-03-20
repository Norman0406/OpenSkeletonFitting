#include "precompiled.h"
#include "Background.h"
#include "BackgroundFactory.h"

namespace osf
{
	Background::Background(System* sys)
		: Module(sys), m_inImgDepth(0)
	{
		addInput(m_inImgDepth);
		addOutput(&m_bgModel);
	}

	Background::~Background(void)
	{
	}

	bool Background::isInit() const
	{
		return m_inImgDepth &&
			Module::isInit();
	}

	void Background::iInit()
	{
		m_bgModel = m_inImgDepth->clone();
		m_bgModel.setTo(0);
	}

	void Background::iProcess()
	{
		iProcessBG();
	}
}