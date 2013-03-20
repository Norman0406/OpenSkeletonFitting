#pragma once
#include "../OpenSF/Factory.h"
#include "Features.h"

namespace osf
{
	template<>
	Features* Factory<Features>::iCreateByType(const type_info& info, System* sys)
	{
		if (info == Features::getType())
			return new Features(sys);
		else
			return 0;
	}
}
