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
