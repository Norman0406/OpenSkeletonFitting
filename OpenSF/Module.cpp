#include "precompiled.h"
#include "Module.h"

namespace osf
{
	Module::Module(System* sys)
		: m_system(sys), m_lastTime(0)
	{
	}

	Module::~Module(void)
	{
	}

	void Module::init()
	{
		iInit();
	}

	float Module::getLastTime() const
	{
		return m_lastTime;
	}

	bool Module::isInit() const
	{
		return true;
	}

	void Module::process()
	{
		// pre-process

		m_timer.reset();
		m_timer.start();

		iProcess();

		m_timer.stop();
		m_lastTime = m_timer.getDiffMS();

		// post-process
	}
}
