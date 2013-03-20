/***********************************************************************
*
* OpenSkeletonFitting
* Skeleton fitting by the use of energy minimization
* Copyright (C) 2012 Norman Link <norman.link@gmx.net>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***********************************************************************/

#pragma once
#include <iostream>
#include "Logging.h"

#if __APPLE__ & __MACH__
#include <typeinfo>
using namespace std;
#endif

namespace osf
{
	class System;

	template <class T>
	class Factory
	{
	public:
		~Factory();

		static T* createByType(const type_info& info, System* sys) {
			LOG << "creating object: " << info.name() << ENDL;

			if (!sys)
				throw Exception("system invalid");

			T* obj = iCreateByType(info, sys);

			if (!obj) {
				ERR << "could not create object: \"" << info.name() << "\"" << ENDL;
				throw Exception("could not create object");
			}

			return obj;
		}

	protected:
		Factory();

		static T* iCreateByType(const type_info&, System* sys);
	};
}
