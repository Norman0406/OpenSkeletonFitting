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

namespace osf
{
	class Fitting;
	class Skeleton;
	class FeaturePoint;
	class Joint;
	
	/************************************************************************
	  SkeletonManipulator
	*************************************************************************/
	class SkeletonManipulator
		: public Skeleton
	{
	public:
		SkeletonManipulator(Fitting*);
		~SkeletonManipulator();
	
		Joint* iInit();

	private:
		static bool classifyMostMoving(const FeaturePoint*, const Joint*,
			const std::vector<FeaturePoint*>&, Joint*);
		static bool extrapolation(Fitting* fitting, Joint* joint,
			const FeaturePoint* torso, const std::vector<FeaturePoint*>* featurePoints);
	};
}