#include "precompiled.h"
#include "InputNumbered.h"
#include "../OpenSFLib/System.h"

namespace osf
{
	/************************************************************************
	  This class reads a previously captured image stream from the disk.
	  Every image is saved seperately and loaded into memory. Because of this,
	  the reading speed highly depends on hard drive speed and is slowing
	  down during buffering. To reduce this problem, an efficient real-time
	  compression method could be used.
	*************************************************************************/

	InputNumbered::InputNumbered(System* sys)
		: Input(sys)
	{
		m_format.clear();
		m_loopMode = false;
		m_paused = false;
		m_startIndex = 0;
		m_frameIndex = 0;
		m_lastTimestamp = 0;
		m_thisTimer.reset();
	}

	InputNumbered::~InputNumbered(void)
	{
	}
	
	void InputNumbered::setFilePrefix(const std::string& prefix)
	{
		m_format = std::string(prefix) + "_%d_%s.cvm";
	}

	void InputNumbered::setLoopMode(bool loopMode)
	{
		m_loopMode = loopMode;
	}
	
	void InputNumbered::setPauseMode(bool paused)
	{
		m_paused = paused;
	}

	void InputNumbered::setStartFrame(int index)
	{
		m_startIndex = index;
	}

	void InputNumbered::iInit()
	{
		char buffer[512];
		bool successful = true;
		
		// read depth image
		sprintf(buffer, m_format.c_str(), m_frameIndex, "depth");
		successful &= loadCvMat(buffer, m_imgDepth);
		
		// read depth image
		sprintf(buffer, m_format.c_str(), m_frameIndex, "color");
		successful &= loadCvMat(buffer, m_imgColor);
		
		// read depth image
		sprintf(buffer, m_format.c_str(), m_frameIndex, "3d");
		successful &= loadCvMat(buffer, m_img3d);

		if (!successful)
			throw Exception("could not init");

		Input::iInit();
	}

	bool InputNumbered::isInit() const
	{
		return !m_format.empty() &&
			Input::isInit();
	}

	void InputNumbered::iGrabImages()
	{
		if (!m_paused) {
			if (m_frameIndex < m_startIndex)
				m_frameIndex = m_startIndex;

			m_thisTimer.stop();
			m_lastTime = m_thisTimer.getDiffS();

			m_thisTimer.reset();
			m_thisTimer.start();

			float timestamp;
			char buffer[512];
			bool successful = true;
		
			// read depth image
			sprintf(buffer, m_format.c_str(), m_frameIndex, "depth");
			successful &= loadCvMat(buffer, m_imgDepth, &timestamp);
		
			// read depth image
			sprintf(buffer, m_format.c_str(), m_frameIndex, "color");
			successful &= loadCvMat(buffer, m_imgColor);
		
			// read depth image
			sprintf(buffer, m_format.c_str(), m_frameIndex, "3d");
			successful &= loadCvMat(buffer, m_img3d);

			// loop or terminate
			if (!successful) {
				if (m_frameIndex > m_startIndex && m_loopMode)
					m_frameIndex = m_startIndex;
				else
					m_system->terminate(true);
			}

			m_frameIndex++;
		
			// wait (here or at the beginning of the function?)
			float diff = timestamp - m_lastTimestamp;

			m_lastTimestamp = timestamp;
		}
	}
}
