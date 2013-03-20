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
#include "Background.h"
#include "BackgroundFactory.h"

namespace osf
{
	Background::Background(System* sys)
		: Module(sys), m_inImgDepth(0)
	{
		addInput(m_inImgDepth);
		addOutput(&m_bgModel);
	}

	Background::~Background(void)
	{
	}

	bool Background::isInit() const
	{
		return m_inImgDepth &&
			Module::isInit();
	}

	void Background::iInit()
	{
		m_bgModel = m_inImgDepth->clone();
		m_bgModel.setTo(0);
	}

	void Background::iProcess()
	{
		iProcessBG();
	}
}