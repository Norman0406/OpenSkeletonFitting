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