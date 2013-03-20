#pragma once
#include "../OpenSF/Factory.h"
#include "Fitting.h"

namespace osf
{
	template<>
	Fitting* Factory<Fitting>::iCreateByType(const type_info& info, System* sys)
	{
		if (info == Fitting::getType())
			return new Fitting(sys);
		else
			return 0;
	}
}
