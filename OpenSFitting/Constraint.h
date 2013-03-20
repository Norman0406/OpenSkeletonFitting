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
#include <opencv2/opencv.hpp>

namespace osf
{
	class Joint2Joint;
	class JointBallAndSocket;
	class JointHinge;

	/************************************************************************
	  Constraint: base class for constraints
	*************************************************************************/
	class Constraint
	{
	public:
		Constraint();
		~Constraint();

		void check(Joint*);

	protected:
		void setLocalRot(Joint*, quaternion<double>);
		virtual void iCheck(Joint*) = 0;
	};

	/************************************************************************
	  ConstraintHinge: defines an axis to rotate around
	*************************************************************************/
	class ConstraintHinge
		: public Constraint
	{
	public:
		ConstraintHinge(cv::Point3d axis, double minAngle, double maxAngle);
		~ConstraintHinge();

	protected:
		void iCheck(Joint*);

	private:
		cv::Point3d m_axis;
		double m_minAngle;
		double m_maxAngle;
	};

	/************************************************************************
	  ConstraintConeTwist: defines a valid cone volume
	*************************************************************************/
	class ConstraintConeTwist
		: public Constraint
	{
	public:
		ConstraintConeTwist(double axisAngle, double minTwist, double maxTwist,
			cv::Point3d axis = cv::Point3d(0, 1, 0));
		~ConstraintConeTwist();
		
	protected:
		void iCheck(Joint*);

	private:
		cv::Point3d m_axis;
		double m_axisAngle;
		double m_minTwist;
		double m_maxTwist;
	};

	/************************************************************************
	  ConstraintFixed: allows only one rotation
	*************************************************************************/
	class ConstraintFixed
		: public Constraint
	{
	public:
		ConstraintFixed(double angleX, double angleY, double angleZ);
		ConstraintFixed(cv::Point3d angles);
		~ConstraintFixed();
		
	protected:
		void iCheck(Joint*);

	private:
		cv::Point3d m_angles;
	};

	/************************************************************************
	  ConstraintFunction: uses an arbitrary function to define constraints
	*************************************************************************/
	class ConstraintFunction
		: public Constraint
	{
	public:
		typedef void (*ConstraintFunc)(Joint*, Constraint*);

		ConstraintFunction(ConstraintFunc func);
		~ConstraintFunction();
		
	protected:
		void iCheck(Joint*);

	private:
		ConstraintFunc m_func;
	};
}