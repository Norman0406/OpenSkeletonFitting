#pragma once
#include "Input.h"

namespace osf
{
	class InputNumbered
		: public Input
	{
		MK_TYPE(InputNumbered);

	public:
		InputNumbered(System*);
		~InputNumbered(void);

		bool isInit() const;

		// public interface
		void setFilePrefix(const std::string& prefix);
		void setLoopMode(bool);
		void setPauseMode(bool);
		void setStartFrame(int);

	protected:
		void iInit();
		void iGrabImages();

	private:
		std::string m_format;
		bool m_paused;
		bool m_loopMode;
		int m_frameIndex;
		int m_startIndex;
		float m_lastTimestamp;
		float m_lastTime;
		Timer m_thisTimer;
	};
}