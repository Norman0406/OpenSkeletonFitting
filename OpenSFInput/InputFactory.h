#pragma once
#include "../OpenSF/Factory.h"
#include "InputKinect.h"
#include "InputNumbered.h"
#include "InputPlayerONI.h"

namespace osf
{
	class Input;

	template<>
	Input* Factory<Input>::iCreateByType(const type_info& info, System* sys)
	{
		if (info == InputKinect::getType())
			return new InputKinect(sys);
		else if (info == InputNumbered::getType())
			return new InputNumbered(sys);
		else if (info == InputPlayerONI::getType())
			return new InputPlayerONI(sys);
		else
			return 0;
	}
}
