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
#include "BackgroundFirst.h"

namespace osf
{
	BackgroundFirst::BackgroundFirst(System* sys)
		: Background(sys)
	{
		m_firstRun = true;
	}

	BackgroundFirst::~BackgroundFirst(void)
	{
	}

	void BackgroundFirst::iProcessBG()
	{
		if (m_firstRun) {
			m_inImgDepth->copyTo(m_bgModel);
			m_firstRun = false;
		}
	}
}
