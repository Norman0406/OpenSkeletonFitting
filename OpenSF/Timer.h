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
