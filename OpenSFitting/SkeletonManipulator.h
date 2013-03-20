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