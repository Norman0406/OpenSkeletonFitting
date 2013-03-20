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

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#elif __APPLE__ & __MACH__
#include <sys/time.h>
#endif

namespace osf
{
	class Timer
	{
	public:
		Timer();
		~Timer();

		void start();
		void stop();
		float getDiffMS() const;
		float getDiffS() const;
		void reset();
		static float getTimestamp();

	private:
#ifdef _WIN32
		LARGE_INTEGER m_start;
		LARGE_INTEGER m_stop;
		LARGE_INTEGER m_frequency;
#elif __APPLE__ & __MACH__
		timeval m_start;
		timeval m_stop;
#endif
		double m_elapsed;
	};
}
