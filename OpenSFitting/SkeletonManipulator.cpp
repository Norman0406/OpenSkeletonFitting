#include "precompiled.h"
#include "SkeletonManipulator.h"
#include "Skeleton.h"
#include "SkeletonCommon.h"
#include "SkeletonHuman.h"
#include "Joint.h"
#include "Constraint.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSF/Exception.h"

namespace osf
{
	/************************************************************************
	  SkeletonManipulator
	*************************************************************************/
	SkeletonManipulator::SkeletonManipulator(Fitting* fitting)
		: Skeleton(fitting)
	{
	}

	SkeletonManipulator::~SkeletonManipulator()
	{
	}

	Joint* SkeletonManipulator::iInit()
	{
		JointEndConnector* endAffector = new JointEndConnector(JT_LEFTHAND);
		JointBallAndSocket* midJoint = new JointBallAndSocket(JT_NECK, endAffector, 0.35);
		JointBallAndSocket* mainJoint = new JointBallAndSocket(JT_TORSO, midJoint, 0.35);
		JointEndConnector* root = new JointEndConnector(JT_TORSO);
		
		// set initial pose
		mainJoint->setOrientation(0, 0, 180);
		root->addSubJoint(mainJoint);
		root->setAsStandard();

		// set energy functions for end affectors
		root->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		endAffector->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
				
		// set classificator objects
		endAffector->setClassFunc(SkeletonManipulator::classifyMostMoving);
		
		// set extrapolator functions
		m_skeletonFitting->setExtrapolatorFunc(JT_LEFTHAND, extrapolation);

		return root;
	}

	bool SkeletonManipulator::classifyMostMoving(const FeaturePoint* featurePoint, const Joint* joint,
		const std::vector<FeaturePoint*>& ftPoints, Joint* skelRoot)
	{
		if (joint->getClass() != JC_ENDCONNECTOR) {
			WARN << "wrong joint class" << ENDL;
			return false;
		}
		
		// parameters
		const double torsoHeightTolerance = 0.2;
		const int minLifetime = 10;
		const double minSummedRelMeanSpeed = 0.03;
		const double minRelSpeed = 0.08;

		bool isFoot = featurePoint->getPosition3d().y > (skelRoot->getPos3d().y + torsoHeightTolerance);

		// condition for a feature point to be chosen
		bool trackingCondition = !isFoot &&
			featurePoint->getLatestConfLifetime() > minLifetime &&
			featurePoint->getFtSummedMeanRelSpd() > minSummedRelMeanSpeed &&
			featurePoint->getFtRelSpd() > minRelSpeed;

		if (trackingCondition)
			return true;

		return false;
	}
	
	bool SkeletonManipulator::extrapolation(Fitting* fitting, Joint* joint,
		const FeaturePoint* torso, const std::vector<FeaturePoint*>* featurePoints)
	{
		if (!fitting || !joint || !featurePoints || !torso)
			throw Exception("invalid input data");

		if (joint->getType() != JT_LEFTHAND && joint->getType() != JT_RIGHTHAND)
			return false;

		FeaturePoint* ftPoint = 0;
		for (int i = 0; i < (int)featurePoints->size(); i++) {
			if ((*featurePoints)[i]->getJointLabel() == joint->getType())
				ftPoint = (*featurePoints)[i];
		}

		const double torsoZTolerance = 0.1;
		if (ftPoint != 0 && !ftPoint->isConfirmed()) {	
			// only extrapolate if z distance of feature point is higher than z distance of torso
			if (ftPoint->getPosition3dFiltered().z > (torso->getPosition3dFiltered().z - torsoZTolerance))
				return false;

			cv::Point3d torsoPos = torso->getPosition3dFiltered();
			cv::Point3d startPos = ftPoint->getPosition3dFiltered();
			
			std::vector<cv::Point3d> neighbors;
			std::vector<float> neighborDistances;
			
			/*const float radius = 0.3f;
			fitting->computeDepthNNRadius(radius, startPos, neighbors, neighborDistances);*/

			const int k = 10;
			fitting->computeDepthKNN(k, startPos, neighbors, neighborDistances);

			if (neighbors.size() > 0) {
				// chose the foremost neighbor
				cv::Point3d newPoint3d(0, 0, std::numeric_limits<double>::infinity());
				for (int i = 0; i < (int)neighbors.size(); i++) {
					if (neighbors[i].z < newPoint3d.z)
						newPoint3d = neighbors[i];
				}

				cv::Point3d newPoint3dRel = newPoint3d - torsoPos;
				cv::Point2d newPoint2d(0, 0);
				pointXYZ2UV(fitting->getProjMat(), newPoint3d, newPoint2d);

				((FeaturePointAccessor*)ftPoint)->setPosition(newPoint3d, newPoint3dRel, newPoint2d);
				ftPoint->setConfirmState(FeaturePoint::CS_EXTRAPOLATED);
				return true;
			}
		}
		else if (!ftPoint)
			fitting->fixToStandardPose(joint);

		return false;
	}
}