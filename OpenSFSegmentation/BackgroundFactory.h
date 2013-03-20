#pragma once
#include "../OpenSF/Factory.h"
#include "BackgroundFirst.h"

namespace osf
{
	template<>
	Background* Factory<Background>::iCreateByType(const type_info& info, System* sys)
	{
		if (info == BackgroundFirst::getType())
			return new BackgroundFirst(sys);
		else
			return 0;
	}
}
