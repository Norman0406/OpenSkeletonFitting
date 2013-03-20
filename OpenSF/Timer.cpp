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

#include "precompiled.h"
#include "Timer.h"

using namespace std;

namespace osf
{
	Timer::Timer()
	{
		reset();
#ifdef _WIN32
		QueryPerformanceFrequency(&m_frequency);
#elif __APPLE__ & __MACH__
		m_start.tv_sec = 0;
		m_start.tv_usec = 0;
		m_stop.tv_sec = 0;
		m_stop.tv_usec = 0;
#endif
	}

	Timer::~Timer()
	{
	}

	void Timer::start()
	{
#ifdef _WIN32
		QueryPerformanceCounter(&m_start);
#elif __APPLE__ & __MACH__
		gettimeofday(&m_start, NULL);
#endif
	}

	void Timer::stop()
	{
#ifdef _WIN32
		QueryPerformanceCounter(&m_stop);

		m_elapsed = (double)(m_stop.QuadPart - m_start.QuadPart);
		m_elapsed /= m_frequency.QuadPart;
#elif __APPLE__ & __MACH__
		gettimeofday(&m_stop, NULL);
		m_elapsed = m_stop.tv_sec - m_start.tv_sec;
		m_elapsed += (m_stop.tv_usec - m_start.tv_usec) / 1000000.0f;
#endif
	}
	
	float Timer::getDiffMS() const
	{
		return (float)(m_elapsed * 1000.0);
	}

	float Timer::getDiffS() const
	{
		return (float)m_elapsed;
	}

	float Timer::getTimestamp()
	{
#ifdef _WIN32
		LARGE_INTEGER timestamp;
		LARGE_INTEGER frq;
		QueryPerformanceCounter(&timestamp);
		QueryPerformanceFrequency(&frq);
		return timestamp.QuadPart / (float)frq.QuadPart;
#elif __APPLE__ & __MACH__
		// UNDONE
		return 0;
#endif
	}

	void Timer::reset()
	{
		m_elapsed = 0;
	}
}
