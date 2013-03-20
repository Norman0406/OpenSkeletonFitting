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
#include "../OpenSF/Module.h"
#include "../OpenSF/Factory.h"
#include "../OpenSF/Logging.h"

#if __APPLE__ & __MACH__
#include <typeinfo>
using namespace std;
#endif

namespace osf
{
	class Input;
	class Segmentation;
	class Features;
	class Fitting;

	class System
	{
	public:
		System();
		~System();
		
		void init();

		Input* createInput(const type_info&);
		Segmentation* createSegmentation(const type_info&);
		Features* createFeatures(const type_info&);
		Fitting* createFitting(const type_info&);
		bool getTerminate() const;
		void terminate(bool);

		const Input* getInput() const;
		const Segmentation* getSegmentation() const;
		const Features* getFeatures() const;
		const Fitting* getFitting() const;
		
		bool isInit() const;
		
		void prepare();
		void process();

	private:
		Input* m_input;
		Segmentation* m_segmentation;
		Features* m_features;
		Fitting* m_fitting;
		bool m_terminate;
	};
}
