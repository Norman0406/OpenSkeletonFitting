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
#include "Skeleton.h"
#include "Joint.h"
#include "../OpenSFitting/Fitting.h"

namespace osf
{
	/************************************************************************
	  Skeleton
	*************************************************************************/
	Skeleton::Skeleton(Fitting* fitting)
		: m_rootJoint(0), m_skeletonFitting(fitting)
	{
	}

	Skeleton::~Skeleton()
	{
		delete m_rootJoint;
	}

	Joint* Skeleton::getRoot()
	{
		return m_rootJoint;
	}

	void Skeleton::init()
	{
		m_rootJoint = iInit();
	}

	void Skeleton::updateFK()
	{
		if (!m_rootJoint)
			throw Exception("invalid data");

		updateFK(m_rootJoint);
	}
	
	void Skeleton::updateFK(Joint* joint)
	{
		joint->updateForward();
	}
}