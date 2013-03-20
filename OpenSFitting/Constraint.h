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