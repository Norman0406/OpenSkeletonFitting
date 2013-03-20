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
#include "Constraint.h"
#include "Joint.h"
#include "../OpenSF/Exception.h"
#include "../OpenSF/Utils.h"

namespace osf
{
	/************************************************************************
	  Constraint
	*************************************************************************/
	Constraint::Constraint()
	{
	}

	Constraint::~Constraint()
	{
	}

	void Constraint::check(Joint* joint)
	{
		iCheck(joint);
	}
	
	void Constraint::setLocalRot(Joint* joint, quaternion<double> locRotQuat)
	{
		joint->m_localRotQuat = locRotQuat;
	}
	
	/************************************************************************
	  ConstraintHinge
	*************************************************************************/
	ConstraintHinge::ConstraintHinge(cv::Point3d axis, double minAngle, double maxAngle)
		: m_axis(axis), m_minAngle(minAngle), m_maxAngle(maxAngle)
	{
		if (m_minAngle > m_maxAngle)
			throw Exception("minimum angle has to be smaller than maximum angle");
	}

	ConstraintHinge::~ConstraintHinge()
	{
	}

	void ConstraintHinge::iCheck(Joint* joint)
	{
		if (joint->getClass() == JC_HINGE) {
			JointHinge* hinge = (JointHinge*)joint;
			
			// create global rotation axis
			quaternion<double> global = joint->getGlobalQuat();
			cv::Point3d axis = m_axis;
			quatRotate(global, axis);
			axis *= 1.0 / cv::norm(axis);

			cv::Point3d bone = hinge->getGlobalDirection();
			cv::Point3d normal = ((Joint2Joint*)(hinge->getPrevJoint()))->getGlobalDirection();
			
			// create direction and normal for minimum plane
			cv::Point3d dirMin = normal;
			cv::Point3d normalMin = normal;
			quaternion<double> minQuat(0, 0, 0, 0);
			axis2Quat(minQuat, axis, m_minAngle);
			quatRotate(minQuat, dirMin);
			axis2Quat(minQuat, axis, m_minAngle + 90);
			quatRotate(minQuat, normalMin);
			
			// create direction and normal for maximum plane
			cv::Point3d dirMax = normal;
			cv::Point3d normalMax = normal;
			quaternion<double> maxQuat(0, 0, 0, 0);
			axis2Quat(maxQuat, axis, m_maxAngle);
			quatRotate(maxQuat, dirMax);
			axis2Quat(maxQuat, axis, m_maxAngle - 90);
			quatRotate(maxQuat, normalMax);

			// create middle normal
			cv::Point3d normalMiddle = normalMax + normalMin;
			normalMiddle *= 1.0 / cv::norm(normalMiddle);

			// dot product with direction vectors (pointing in the direction of the plane)
			double dotDirMax = dirMax.dot(bone);
			double dotDirMin = dirMin.dot(bone);

			// dot product with plane normal vectors
			double dotNormalMax = normalMax.dot(bone);
			double dotNormalMin = normalMin.dot(bone);

			// dot product with average vector
			double dotNormalMiddle = normalMiddle.dot(bone);

			// angular range
			double range = m_maxAngle - m_minAngle;

			bool outMax = false;
			bool outMin = false;
			bool outside = false;

			// handle special cases
			if (range < 180.0) {
				if (dotNormalMiddle < 0 || dotNormalMax < 0 || dotNormalMin < 0)
					outside = true;
			}
			else {
				if (dotNormalMiddle < 0) {
					outside = true;

					if (dotNormalMax > 0)
						outside = false;
					if (dotNormalMin > 0)
						outside = false;
				}
			}

			// detect, which limit is nearer
			if (outside) {
				if (1.0 - dotDirMax < 1.0 - dotDirMin)
					outMax = true;
				else
					outMin = true;
			}
			
			// set angle to limits
			if (outMax)
				hinge->setAngle(m_maxAngle);
			else if (outMin)
				hinge->setAngle(m_minAngle);
		}
	}

	/************************************************************************
	  ConstraintConeTwist
	*************************************************************************/
	ConstraintConeTwist::ConstraintConeTwist(double axisAngle, double minTwist, double maxTwist, cv::Point3d axis)
		: m_axis(axis), m_axisAngle(axisAngle), m_minTwist(minTwist), m_maxTwist(maxTwist)
	{
		if (m_minTwist > m_maxTwist)
			throw Exception("minimum twist has to be smaller than maximum twist");
	}

	ConstraintConeTwist::~ConstraintConeTwist()
	{
	}

	cv::Point3d prevNormal(0, 0, 0);
	void ConstraintConeTwist::iCheck(Joint* joint)
	{
		const double coneAngle = DEG2RAD(m_axisAngle);

		// create axis vector to specify cone main axis
		cv::Point3d axis(0, 0, 0);
		const Joint* prevJoint = joint->getPrevJoint();
		quaternion<double> axisQuat(0, 0, 0, 0);

		if (prevJoint) {
			// use coordinate system of previous bone
			axisQuat = prevJoint->getGlobalQuat();
		}
		else {
			// UNDONE: has some problems with root node [12/21/2011 Norman]

			// use standard coordinate system at root position
			euler2Quat(axisQuat, 0, 0, 0);
		}
		
		quaternion<double> axisTemp(0, m_axis.x, m_axis.y, m_axis.z);
		axisTemp = conj(axisQuat) * axisTemp * axisQuat;
		axis = cv::Point3d(axisTemp.R_component_2(), axisTemp.R_component_3(),
			axisTemp.R_component_4());
		axis *= 1.0 / cv::norm(axis);

		quaternion<double> anchorQuat = joint->getGlobalQuat();
		quaternion<double> locAnchorQuat = joint->getLocalQuat();
		
		// create anchor vector
		quaternion<double> anchorTemp(0, 0, 1, 0);
		anchorTemp = conj(anchorQuat) * anchorTemp * anchorQuat;
		cv::Point3d anchor(anchorTemp.R_component_2(), anchorTemp.R_component_3(),
			anchorTemp.R_component_4());
		anchor *= 1.0 / cv::norm(anchor);

		double dot = axis.dot(anchor);
		double angle = acos(dot);
		double cosConeAngle = cos(coneAngle);

		if (joint->getType() == JT_RIGHTSHOULDER)
			int k = 0;

		// clamp
		if (dot < std::numeric_limits<double>::epsilon())
			dot = 0;
		if (cosConeAngle < std::numeric_limits<double>::epsilon())
			cosConeAngle = 0;
		
		if (dot < cosConeAngle) {	// outside boundaries
			// get current local rotation axis and angle
			cv::Point3d axis(0, 0, 0);
			double axisAngle = 0;
			quat2Axis(locAnchorQuat, axis, axisAngle);

			// get new angle to restrict rotation
			double angleDiff = -RAD2DEG(abs(dot - cos(coneAngle)));

			// rotate quaternion back by angleDiff
			quaternion<double> newQuat(0, 0, 0, 0);
			axis2Quat(newQuat, axis, angleDiff);

			// NOTE: does somehow not work with root node [12/21/2011 Norman]
			locAnchorQuat = locAnchorQuat * newQuat;
			setLocalRot(joint, locAnchorQuat);
		}

		// NOTE: does not work fully yet. Problem: defining the (constant) normal
		// on the bone, which does not have any twist rotation and serves as a reference
		// to compare the actual rotation with. [1/11/2012 Norman]

		/*
		if (joint->getClass() == JC_BALLANDSOCKET) {
			JointBallAndSocket* ballAndSocket = (JointBallAndSocket*)joint;

			// TODO: clarify coordinate situation
			cv::Point3d bone = cv::Point3d(0, 0, -1);	// right?
			quatRotate(joint->getGlobalQuat(), bone);

			// remove twist to get stable reference normal
			cv::Point3d tEuler;
			quat2Euler(joint->getLocalQuat(), tEuler.x, tEuler.y, tEuler.z);
			quaternion<double> tLocQuat;
			euler2Quat(tLocQuat, tEuler.x, 0, tEuler.z);

			quaternion<double> tGlobQuat = tLocQuat * joint->getPrevJoint()->getGlobalQuat();
			
			cv::Point3d normal = cv::Point3d(1, 0, 0);	// right?
			quatRotate(tGlobQuat, normal);

			// NOTE: normal can be flipped at 180 degrees. If so, flip back.
			// not sure if this is working in all cases [1/11/2012 Norman]
			if (prevNormal.dot(normal) < 0)
				normal = -normal;
			prevNormal = normal;

			cv::Point3d axis(0, 1, 0);
			quatRotate(joint->getGlobalQuat(), axis);
		
			// create direction and normal for minimum plane
			cv::Point3d dirMin = normal;
			cv::Point3d normalMin = normal;
			quaternion<double> minQuat(0, 0, 0, 0);
			axis2Quat(minQuat, axis, m_minTwist);
			quatRotate(minQuat, dirMin);
			axis2Quat(minQuat, axis, m_minTwist + 90);
			quatRotate(minQuat, normalMin);
		
			// create direction and normal for maximum plane
			cv::Point3d dirMax = normal;
			cv::Point3d normalMax = normal;
			quaternion<double> maxQuat(0, 0, 0, 0);
			axis2Quat(maxQuat, axis, m_maxTwist);
			quatRotate(maxQuat, dirMax);
			axis2Quat(maxQuat, axis, m_maxTwist - 90);
			quatRotate(maxQuat, normalMax);

			// create middle normal
			cv::Point3d normalMiddle = normalMax + normalMin;
			normalMiddle *= 1.0 / cv::norm(normalMiddle);

			// dot product with direction vectors (pointing in the direction of the plane)
			double dot = normal.dot(bone);
			double dotDirMax = dirMax.dot(bone);
			double dotDirMin = dirMin.dot(bone);

			// dot product with plane normal vectors
			double dotNormalMax = normalMax.dot(bone);
			double dotNormalMin = normalMin.dot(bone);

			// dot product with average vector
			double dotNormalMiddle = normalMiddle.dot(bone);

			// angular range
			double range = m_maxTwist - m_minTwist;

			bool outMax = false;
			bool outMin = false;
			bool outside = false;

			// handle special cases
			if (range < 180.0) {
				if (dotNormalMiddle < 0 || dotNormalMax < 0 || dotNormalMin < 0)
					outside = true;
			}
			else {
				if (dotNormalMiddle < 0) {
					outside = true;

					if (dotNormalMax > 0)
						outside = false;
					if (dotNormalMin > 0)
						outside = false;
				}
			}

			// detect, which limit is nearer
			if (outside) {
				if (1.0 - dotDirMax < 1.0 - dotDirMin)
					outMax = true;
				else
					outMin = true;

				LOG << "outside" << ENDL;
			}

			double rotAngleMax = dotDirMax >= 1.0 ? 0 : RAD2DEG(acos(dotDirMax));
			double rotAngleMin = dotDirMin >= 1.0 ? 0 : RAD2DEG(acos(dotDirMin));

			// set angle to limits
			if (outMax)
				ballAndSocket->addOrientation(0, -rotAngleMax, 0);
			else if (outMin)
				ballAndSocket->addOrientation(0, rotAngleMin, 0);
		}*/
	}

	/************************************************************************
	  ConstraintFixed
	*************************************************************************/
	ConstraintFixed::ConstraintFixed(double angleX, double angleY, double angleZ)
		: m_angles(cv::Point3d(angleX, angleY, angleZ))
	{
	}

	ConstraintFixed::ConstraintFixed(cv::Point3d angles)
		: m_angles(angles)
	{
	}

	ConstraintFixed::~ConstraintFixed()
	{
	}

	void ConstraintFixed::iCheck(Joint* joint)
	{
		quaternion<double> fixedRot(0, 0, 0, 0);
		euler2Quat(fixedRot, m_angles.x, m_angles.y, m_angles.z);
		setLocalRot(joint, fixedRot);
	}

	/************************************************************************
	  ConstraintFunction
	*************************************************************************/
	ConstraintFunction::ConstraintFunction(ConstraintFunc func)
		: m_func(func)
	{
	}

	ConstraintFunction::~ConstraintFunction()
	{
	}

	void ConstraintFunction::iCheck(Joint* joint)
	{
		m_func(joint, this);
	}
}
