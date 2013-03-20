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
#include "Skeleton.h"

namespace osf
{
	class Fitting;
	class FeaturePoint;
	class Joint;
	
	/************************************************************************
	  SkeletonHuman
	*************************************************************************/
	class SkeletonHuman
		: public Skeleton
	{
	public:
		~SkeletonHuman();

	protected:
		SkeletonHuman(Fitting*);
		
		static double energyGeodesicLine(const Joint*, void*);
		static bool classifyHead(const FeaturePoint*, const Joint*, const std::vector<FeaturePoint*>&, Joint*);
		static bool classifyHands(const FeaturePoint*, const Joint*, const std::vector<FeaturePoint*>&, Joint*);
		static bool extrapolationHead(Fitting* fitting, Joint* joint,
			const FeaturePoint* torso, const std::vector<FeaturePoint*>* featurePoints);
		static bool extrapolationHands(Fitting* fitting, Joint* joint,
			const FeaturePoint* torso, const std::vector<FeaturePoint*>* featurePoints);
	};

	/************************************************************************
	  SkeletonUpperBody
	*************************************************************************/
	class SkeletonUpperBody
		: public SkeletonHuman
	{
	public:
		SkeletonUpperBody(Fitting*);
		~SkeletonUpperBody();
	
		virtual Joint* iInit();
	};

	/************************************************************************
	  SkeletonLowerBody
	*************************************************************************/
	class SkeletonLowerBody
		: public SkeletonHuman
	{
	public:
		SkeletonLowerBody(Fitting*);
		~SkeletonLowerBody();
	
		virtual Joint* iInit();

	private:
		static bool classifyFeet(const FeaturePoint*, const Joint*, const std::vector<FeaturePoint*>&, Joint*);
	};

	/************************************************************************
	  SkeletonFullBody
	*************************************************************************/
	class SkeletonFullBody
		: public SkeletonHuman
	{
	public:
		SkeletonFullBody(Fitting*);
		~SkeletonFullBody();
	
		virtual Joint* iInit();
	};
	
	/************************************************************************
	  SkeletonSimple
	*************************************************************************/
	class SkeletonSimple
		: public SkeletonHuman
	{
	public:
		SkeletonSimple(Fitting*);
		~SkeletonSimple();
	
		Joint* iInit();
	};
}