#include "precompiled.h"
#include "SkeletonHuman.h"
#include "SkeletonCommon.h"
#include "Joint.h"
#include "Constraint.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSF/Exception.h"

namespace osf
{
	/************************************************************************
	  SkeletonHuman
	*************************************************************************/
	SkeletonHuman::SkeletonHuman(Fitting* fitting)
		: Skeleton(fitting)
	{
	}

	SkeletonHuman::~SkeletonHuman()
	{
	}
	
	double SkeletonHuman::energyGeodesicLine(const Joint* joint, void* cookie)
	{
		if (!cookie || !joint)
			throw Exception("invalid input data");

		if (joint->getType() != JT_LEFTELBOW &&
			joint->getType() != JT_RIGHTELBOW &&
			joint->getType() != JT_NECK &&
			joint->getType() != JT_LEFTKNEE &&
			joint->getType() != JT_RIGHTKNEE) {
			WARN << "energy function was assigned to the wrong end affector" << ENDL;
			return 0;
		}

		Fitting* fitting = (Fitting*)cookie;
		
		// energy is computed as nearest neighbor along the geodesic line. if corresponding joint is invalid,
		// simple nearest neighbor on the whole point cloud is performed.

		cv::Point3d jointPos = joint->getPos3d();
		const int numNN = 5;

		std::vector<cv::Point3d> nearestPoints;
		std::vector<float> nearestDist;
		
		// TODO: make this function independent from joint types: find the first end affector and use this one

		JointType jType = JT_UNKNOWN;
		if (joint->getType() == JT_LEFTELBOW)
			jType = JT_LEFTHAND;
		else if (joint->getType() == JT_RIGHTELBOW)
			jType = JT_RIGHTHAND;
		else if (joint->getType() == JT_NECK)
			jType = JT_HEAD;
		else if (joint->getType() == JT_LEFTKNEE)
			jType = JT_LEFTFOOT;
		else if (joint->getType() == JT_RIGHTKNEE)
			jType = JT_RIGHTFOOT;

		if (jType == JT_UNKNOWN)	// should never happen
			throw Exception("invalid joint type");

		if (!fitting->computeGeodesicKNN(jType, numNN, jointPos, nearestPoints, nearestDist))
			return cmEnergyNearestNeighbor(joint, cookie);

		double dist = 0;
		
		// get mean
		if (nearestDist.size() > 0) {
			for (int i = 0; i < (int)nearestDist.size(); i++) {
				dist += nearestDist[i];
			}
			dist /= nearestDist.size();
		}

		return dist;
	}

	bool SkeletonHuman::classifyHead(const FeaturePoint* featurePoint, const Joint* joint,
		const std::vector<FeaturePoint*>& ftPoints, Joint* skelRoot)
	{
		if (joint->getType() != JT_HEAD) {
			WARN << "classificator function was assigned to the wrong end affector" << ENDL;
			return false;
		}

		// head classification: distance to the torso in y-direction is minimal while absolute distance
		// in y-direction is maximal
		
		// parameters
		const double maxXZDistance = 0.1;
		const int minLifetime = 10;
		const double maxMeanSpeed = 0.2;

		cv::Point3d ftPos = featurePoint->getPosition3dFiltered();
		const Joint* torso = skelRoot->getJoint(JT_TORSO);

		cv::Point3d torsoPos = torso->getPos3d();
		cv::Point2d featPosXZ(ftPos.x, ftPos.z);
		cv::Point2d torsoPosXZ(torsoPos.x, torsoPos.z);
		
		// head position has to be situated above the torso
		if (ftPos.y > torsoPos.y)
			return false;

		// get the distance between feature positions on the xz-plane
		double xzDist = distanceP2d(featPosXZ, torsoPosXZ);

		bool trackingCondition = xzDist < maxXZDistance &&
			featurePoint->getLatestConfLifetime() > minLifetime &&
			featurePoint->getFtMeanRelSpd() < maxMeanSpeed;

		if (trackingCondition)
			return true;

		return false;
	}

	bool SkeletonHuman::classifyHands(const FeaturePoint* featurePoint, const Joint* joint,
		const std::vector<FeaturePoint*>& ftPoints, Joint* skelRoot)
	{
		if (joint->getType() != JT_LEFTHAND && joint->getType() != JT_RIGHTHAND) {
			WARN << "classificator function was assigned to the wrong end affector" << ENDL;
			return false;
		}

		// hand classification: head has already been classified, lifetime bog enough, getSummedRelativeSpeed()
		// reached threshold and position of feature point is on the same side as the joint position, compared
		// to the line from torso to head position

		// parameters
		const double torsoHeightTolerance = 0.3;
		const int minLifetime = 5;
		const double minSummedRelMeanSpeed = 0.05;	// mean summed speed (per frame in meters)

		// check if the head has already been classified
		bool headClassified = false;
		for (int i = 0; i < (int)ftPoints.size(); i++) {
			if (ftPoints[i]->getJointLabel() == JT_HEAD && ftPoints[i]->isConfirmed()) {
				headClassified = true;
				break;
			}
		}
		
		if (headClassified) {
			const Joint* torso = skelRoot->getJoint(JT_TORSO);
			const Joint* head = skelRoot->getJoint(JT_HEAD);

			// hand joints have to be situated above the torso,
			if (featurePoint->getPosition3dFiltered().y > (torso->getPos3d().y + torsoHeightTolerance))
				return false;
			
			// their position has to be on the same side as the joint to be assigned,
			// relative to the line from torso to head

			cv::Point2d ftPos2d = featurePoint->getPosition2d();

			// get line from torso to head
			cv::Point2d torsoPos2d = torso->getPos2d();
			cv::Point2d normal = head->getPos2d() - torsoPos2d;
			normal = normal * (1.0f / cv::norm(normal));
			
			// get correct normal direction, depending on the current joint
			if (joint->getType() == JT_LEFTHAND) {
				// get the the normal, pointing to the right image side, which
				// corresponds to the side of the left hand
				normal = cv::Point2d(-normal.y, normal.x);
			}
			else {
				// get the the normal, pointing to the left image side, which
				// corresponds to the side of the right hand
				normal = cv::Point2d(normal.y, -normal.x);
			}

			// get half space of feature position
			cv::Point2d featDir = ftPos2d - torsoPos2d;
			featDir = featDir * (1.0f / cv::norm(featDir));
			double dot = normal.dot(featDir);
			
			// left hand on the positive half space, right hand on the negative half space
			bool correctHalfSpace = dot > 0;

			// condition for a feature point to be chosen
			bool trackingCondition = correctHalfSpace &&
				featurePoint->getLatestConfLifetime() > minLifetime &&
				featurePoint->getFtSummedMeanRelSpd() > minSummedRelMeanSpeed &&
				featurePoint->getConfUnconfRatio() > 1.0;

			if (trackingCondition)
				return true;
		}

		return false;
	}

	bool SkeletonHuman::extrapolationHead(Fitting* fitting, Joint* joint,
		const FeaturePoint* torso, const std::vector<FeaturePoint*>* featurePoints)
	{
		// Idea: every joint has to define a way to extrapolation information. for example the head joints
		// extrapolation defines a nearest neighbor point whose y-coordinate is maximal, while the hand
		// joints extrapolation is defined by the maximal frontal nearest neighbor. If the return value is
		// true, an extrapolated point was found (?). Add the position of the extrapolated point to the
		// equivalent point in featurePoints and confirm the point. [2/13/2012 Norman]

		if (!fitting || !joint || !featurePoints || !torso)
			throw Exception("invalid input data");

		if (joint->getType() != JT_HEAD)
			return false;

		// get feature point assigned to this joint
		FeaturePoint* ftPoint = 0;
		for (int i = 0; i < (int)featurePoints->size(); i++) {
			if ((*featurePoints)[i]->getJointLabel() == joint->getType())
				ftPoint = (*featurePoints)[i];
		}

		// compute nearest neighbors nearest to the feature point
		if (ftPoint != 0 && !ftPoint->isConfirmed()) {
			cv::Point3d torsoPos = torso->getPosition3dFiltered();
			cv::Point3d startPos = ftPoint->getPosition3dFiltered();
			
			std::vector<cv::Point3d> neighbors;
			std::vector<float> neighborDistances;
			
			const int k = 5;
			fitting->computeDepthKNN(k, startPos, neighbors, neighborDistances);

			if (neighbors.size() > 0) {
				// chose the highest neighbor
				cv::Point3d newPoint3d(0, 0, 0);
				for (int i = 0; i < (int)neighbors.size(); i++) {
					if (neighbors[i].y < newPoint3d.y)
						newPoint3d = neighbors[i];
				}

				// update feature point data
				cv::Point3d newPoint3dRel = newPoint3d - torsoPos;
				cv::Point2d newPoint2d(0, 0);
				pointXYZ2UV(fitting->getProjMat(), newPoint3d, newPoint2d);

				((FeaturePointAccessor*)ftPoint)->setPosition(newPoint3d, newPoint3dRel, newPoint2d);
				ftPoint->setConfirmState(FeaturePoint::CS_EXTRAPOLATED);
				return true;
			}
		}

		return false;
	}
	
	bool SkeletonHuman::extrapolationHands(Fitting* fitting, Joint* joint,
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
		
		const double torsoZTolerance = 0.00;
		if (ftPoint != 0 && !ftPoint->isConfirmed()) {	
			// only extrapolate if z distance of feature point is higher than z distance of torso
			if (ftPoint->getPosition3dFiltered().z > (torso->getPosition3dFiltered().z - torsoZTolerance))
				return false;

			cv::Point3d torsoPos = torso->getPosition3dFiltered();
			cv::Point3d startPos = ftPoint->getPosition3dFiltered();
			
			std::vector<cv::Point3d> neighbors;
			std::vector<float> neighborDistances;
			
			const int k = 5;
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

	/************************************************************************
	  SkeletonUpperBody
	*************************************************************************/
	SkeletonUpperBody::SkeletonUpperBody(Fitting* fitting)
		: SkeletonHuman(fitting)
	{
	}

	SkeletonUpperBody::~SkeletonUpperBody()
	{
	}

	Joint* SkeletonUpperBody::iInit()
	{
		// create skeleton
		JointEndConnector* leftHand = new JointEndConnector(JT_LEFTHAND);
		JointEndConnector* rightHand = new JointEndConnector(JT_RIGHTHAND);
		JointEndConnector* head = new JointEndConnector(JT_HEAD);

		JointHinge* leftElbow = new JointHinge(JT_LEFTELBOW, leftHand, 0.35, cv::Point3d(1, 0, 0),
			new ConstraintHinge(cv::Point3d(1, 0, 0), 0, 160));
		JointBallAndSocket* leftShoulder = new JointBallAndSocket(JT_LEFTSHOULDER, leftElbow, 0.27,
			new ConstraintConeTwist(90, 0, 0));

		JointHinge* rightElbow = new JointHinge(JT_RIGHTELBOW, rightHand, 0.35, cv::Point3d(1, 0, 0),
			new ConstraintHinge(cv::Point3d(1, 0, 0), 0, 160));
		JointBallAndSocket* rightShoulder = new JointBallAndSocket(JT_RIGHTSHOULDER, rightElbow, 0.27,
			new ConstraintConeTwist(90, 0, 0));

		// multiple entries with the same JointType? [12/21/2011 Norman]
		JointConnector* neck = new JointConnector(JT_NECK);
		JointBallAndSocket* neckH = new JointBallAndSocket(JT_NECK, head, 0.27,
			new ConstraintConeTwist(40, 0, 0));
		JointBallAndSocket* neckL = new JointBallAndSocket(JT_NECK, leftShoulder, 0.17,
			new ConstraintFixed(0, 0, -90));
		JointBallAndSocket* neckR = new JointBallAndSocket(JT_NECK, rightShoulder, 0.17,
			new ConstraintFixed(0, 0, 90));
		neck->addSubJoint(neckL);
		neck->addSubJoint(neckR);
		neck->addSubJoint(neckH);

		JointBallAndSocket* torsoBaS = new JointBallAndSocket(JT_TORSO, neck, 0.35);
		JointEndConnector* torso = new JointEndConnector(JT_TORSO);
		torso->addSubJoint(torsoBaS);
		
		// set initial pose
		torsoBaS->setOrientation(0, 0, 180);
		neckL->setOrientation(0, 0, -90);
		neckR->setOrientation(0, 0, 90);
		leftShoulder->setOrientation(0, 0, -90);
		rightShoulder->setOrientation(0, 0, 90);
		torso->setAsStandard();	// set current pose as standard pose

		// set energy functions for end affectors
		torso->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		head->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		leftHand->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		rightHand->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		
		// set energy functions for middle joints
		neck->setEnergyFunction(SkeletonHuman::energyGeodesicLine, (void*)m_skeletonFitting);
		leftShoulder->setEnergyFunction(cmEnergyUnderlyingPoint, (void*)m_skeletonFitting);
		rightShoulder->setEnergyFunction(cmEnergyUnderlyingPoint, (void*)m_skeletonFitting);
		leftElbow->setEnergyFunction(SkeletonHuman::energyGeodesicLine, (void*)m_skeletonFitting);
		rightElbow->setEnergyFunction(SkeletonHuman::energyGeodesicLine, (void*)m_skeletonFitting);
		
		// set classificator objects
		head->setClassFunc(SkeletonHuman::classifyHead);
		leftHand->setClassFunc(SkeletonHuman::classifyHands);
		rightHand->setClassFunc(SkeletonHuman::classifyHands);
		
		// set extrapolator functions
		m_skeletonFitting->setExtrapolatorFunc(JT_HEAD, SkeletonHuman::extrapolationHead);
		m_skeletonFitting->setExtrapolatorFunc(JT_LEFTHAND, SkeletonHuman::extrapolationHands);
		m_skeletonFitting->setExtrapolatorFunc(JT_RIGHTHAND, SkeletonHuman::extrapolationHands);

		return torso;
	}

	/************************************************************************
	  SkeletonLowerBody
	*************************************************************************/
	SkeletonLowerBody::SkeletonLowerBody(Fitting* fitting)
		: SkeletonHuman(fitting)
	{
	}

	SkeletonLowerBody::~SkeletonLowerBody()
	{
	}

	Joint* SkeletonLowerBody::iInit()
	{
		// create skeleton
		JointEndConnector* leftFoot = new JointEndConnector(JT_LEFTFOOT);
		JointEndConnector* rightFoot = new JointEndConnector(JT_RIGHTFOOT);

		JointHinge* leftKnee = new JointHinge(JT_LEFTKNEE, leftFoot, 0.42, cv::Point3d(1, 0, 0),
			new ConstraintHinge(cv::Point3d(1, 0, 0), 0, 160));
		JointBallAndSocket* leftHip = new JointBallAndSocket(JT_LEFTHIP, leftKnee, 0.33);

		JointHinge* rightKnee = new JointHinge(JT_RIGHTKNEE, rightFoot, 0.42, cv::Point3d(1, 0, 0),
			new ConstraintHinge(cv::Point3d(1, 0, 0), 0, 160));
		JointBallAndSocket* rightHip = new JointBallAndSocket(JT_RIGHTHIP, rightKnee, 0.33);

		JointConnector* pelvis = new JointConnector(JT_PELVIS);
		JointBallAndSocket* pelvisL = new JointBallAndSocket(JT_PELVIS, leftHip, 0.09,
			new ConstraintFixed(0, 0, 90));
		JointBallAndSocket* pelvisR = new JointBallAndSocket(JT_PELVIS, rightHip, 0.09,
			new ConstraintFixed(0, 0, -90));
		pelvis->addSubJoint(pelvisL);
		pelvis->addSubJoint(pelvisR);

		JointBallAndSocket* torsoBaS = new JointBallAndSocket(JT_TORSO, pelvis, 0.12);
		JointEndConnector* torso = new JointEndConnector(JT_TORSO);
		torso->addSubJoint(torsoBaS);
		
		// set initial pose
		torsoBaS->setOrientation(0, 0, 0);
		pelvisL->setOrientation(0, 0, 90);
		pelvisR->setOrientation(0, 0, -90);
		leftHip->setOrientation(0, 0, -90);
		rightHip->setOrientation(0, 0, 90);
		torso->setAsStandard();	// set current pose as standard pose

		// set energy functions for end affectors
		torso->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		leftFoot->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		rightFoot->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		
		// set energy functions for middle joints
		pelvis->setEnergyFunction(cmEnergyUnderlyingPoint, (void*)m_skeletonFitting);
		leftHip->setEnergyFunction(cmEnergyUnderlyingPoint, (void*)m_skeletonFitting);
		rightHip->setEnergyFunction(cmEnergyUnderlyingPoint, (void*)m_skeletonFitting);
		leftKnee->setEnergyFunction(SkeletonHuman::energyGeodesicLine, (void*)m_skeletonFitting);
		rightKnee->setEnergyFunction(SkeletonHuman::energyGeodesicLine, (void*)m_skeletonFitting);
		
		// set classificator objects
		leftFoot->setClassFunc(classifyFeet);
		rightFoot->setClassFunc(classifyFeet);
		
		return torso;
	}
	
	bool SkeletonLowerBody::classifyFeet(const FeaturePoint* featurePoint, const Joint* joint,
		const std::vector<FeaturePoint*>& ftPoints, Joint* skelRoot)
	{
		if (joint->getType() != JT_LEFTFOOT && joint->getType() != JT_RIGHTFOOT) {
			WARN << "classificator function was assigned to the wrong end affector" << ENDL;
			return false;
		}

		// hand classification: head has already been classified, lifetime bog enough, getSummedRelativeSpeed()
		// reached threshold and position of feature point is on the same side as the joint position, compared
		// to the line from torso to head position

		// parameters
		const double torsoHeightTolerance = 0.3;
		const int minLifetime = 10;

		const Joint* torso = skelRoot->getJoint(JT_TORSO);
		const Joint* pelvis = skelRoot->getJoint(JT_PELVIS);

		// hand joints have to be situated above the torso,
		if (featurePoint->getPosition3dFiltered().y < (torso->getPos3d().y + torsoHeightTolerance))
			return false;
			
		// their position has to be on the same side as the joint to be assigned,
		// relative to the line from torso to head

		cv::Point2d ftPos2d = featurePoint->getPosition2d();

		// get line from torso to pelvis
		cv::Point2d torsoPos2d = torso->getPos2d();
		cv::Point2d normal = pelvis->getPos2d() - torsoPos2d;
		normal = normal * (1.0f / cv::norm(normal));
			
		// get correct normal direction, depending on the current joint
		if (joint->getType() == JT_RIGHTFOOT) {
			// get the the normal, pointing to the right image side, which
			// corresponds to the side of the left hand
			normal = cv::Point2d(-normal.y, normal.x);
		}
		else {
			// get the the normal, pointing to the left image side, which
			// corresponds to the side of the right hand
			normal = cv::Point2d(normal.y, -normal.x);
		}

		// get half space of feature position
		cv::Point2d featDir = ftPos2d - torsoPos2d;
		featDir = featDir * (1.0f / cv::norm(featDir));
		double dot = normal.dot(featDir);
			
		// left hand on the positive half space, right hand on the negative half space
		bool correctHalfSpace = dot > 0;

		// condition for a feature point to be chosen
		bool trackingCondition = correctHalfSpace &&
			featurePoint->getLatestConfLifetime() > minLifetime &&
			featurePoint->getConfUnconfRatio() > 1.0;

		if (trackingCondition)
			return true;

		return false;
	}

	/************************************************************************
	  SkeletonFullBody
	*************************************************************************/
	SkeletonFullBody::SkeletonFullBody(Fitting* fitting)
		: SkeletonHuman(fitting)
	{
	}

	SkeletonFullBody::~SkeletonFullBody()
	{
	}

	Joint* SkeletonFullBody::iInit()
	{
		Skeleton* upperBody = new SkeletonUpperBody(m_skeletonFitting);
		Skeleton* lowerBody = new SkeletonLowerBody(m_skeletonFitting);

		upperBody->init();
		lowerBody->init();

		// detach torso subjoints from upperBody and lowerBody skeletons, then free the memory
		Joint* upperTorso = upperBody->getRoot()->getSubJoints()[0];
		Joint* lowerTorso = lowerBody->getRoot()->getSubJoints()[0];
		upperBody->getRoot()->detachSubJoint(upperTorso);
		lowerBody->getRoot()->detachSubJoint(lowerTorso);

		delete upperBody;
		delete lowerBody;
		
		// create full body skeleton
		JointEndConnector* torso = new JointEndConnector(JT_TORSO);
		torso->addSubJoint(upperTorso);
		torso->addSubJoint(lowerTorso);
		torso->setAsStandard();
		
		// set main torso energy function
		torso->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());

		return torso;
	}	
	
	/************************************************************************
	  SkeletonSimple
	*************************************************************************/
	SkeletonSimple::SkeletonSimple(Fitting* fitting)
		: SkeletonHuman(fitting)
	{
	}

	SkeletonSimple::~SkeletonSimple()
	{
	}

	Joint* SkeletonSimple::iInit()
	{
		JointEndConnector* leftHand = new JointEndConnector(JT_LEFTHAND);
		JointEndConnector* rightHand = new JointEndConnector(JT_RIGHTHAND);
		JointEndConnector* head = new JointEndConnector(JT_HEAD);
		
		JointBallAndSocket* neck = new JointBallAndSocket(JT_NECK, head, 0.25,
			new ConstraintConeTwist(40, 0, 0));
		JointBallAndSocket* neckL = new JointBallAndSocket(JT_NECK, leftHand, 0.6);
		JointBallAndSocket* neckR = new JointBallAndSocket(JT_NECK, rightHand, 0.6);
		neck->addSubJoint(neckL);
		neck->addSubJoint(neckR);

		JointBallAndSocket* torsoBaS = new JointBallAndSocket(JT_TORSO, neck, 0.4);
		JointEndConnector* torso = new JointEndConnector(JT_TORSO);
		torso->addSubJoint(torsoBaS);
		
		// set initial pose
		torsoBaS->setOrientation(0, 0, 180);
		neckL->setOrientation(0, 0, -90);
		neckR->setOrientation(0, 0, 90);
		torso->setAsStandard();

		// set energy functions for end affectors
		torso->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		head->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		leftHand->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		rightHand->setEnergyFunction(cmEnergyAffectors, (void*)m_skeletonFitting->getFeatures());
		
		// set energy functions for middle joints
		neck->setEnergyFunction(SkeletonHuman::energyGeodesicLine, (void*)m_skeletonFitting);
		
		// set classificator objects
		head->setClassFunc(SkeletonHuman::classifyHead);
		leftHand->setClassFunc(SkeletonHuman::classifyHands);
		rightHand->setClassFunc(SkeletonHuman::classifyHands);
		
		// set extrapolator functions
		m_skeletonFitting->setExtrapolatorFunc(JT_HEAD, SkeletonHuman::extrapolationHead);
		m_skeletonFitting->setExtrapolatorFunc(JT_LEFTHAND, SkeletonHuman::extrapolationHands);
		m_skeletonFitting->setExtrapolatorFunc(JT_RIGHTHAND, SkeletonHuman::extrapolationHands);

		return torso;
	}
}