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
