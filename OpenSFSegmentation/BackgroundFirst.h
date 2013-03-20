#pragma once
#include "Background.h"

namespace osf
{
	class BackgroundFirst
		: public Background
	{
		MK_TYPE(BackgroundFirst);

	public:
		BackgroundFirst(System* sys);
		~BackgroundFirst(void);
	
	protected:
		void iProcessBG();

	private:
		bool m_firstRun;
	};
}