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