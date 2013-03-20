#pragma once
#include "../OpenSF/Factory.h"
#include "SegmentationBackground.h"

namespace osf
{
	template<>
	Segmentation* Factory<Segmentation>::iCreateByType(const type_info& info, System* sys)
	{
		if (info == SegmentationBackground::getType())
			return new SegmentationBackground(sys);
		else
			return 0;
	}
}
