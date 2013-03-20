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
#include <list>
#include <boost/math/quaternion.hpp>
#include <boost/thread.hpp>

using namespace boost::math;

namespace osf
{
	// joint types
	enum JointType {
		JT_UNKNOWN = 0,
		JT_HEAD,
		JT_NECK,
		JT_LEFTSHOULDER,
		JT_LEFTELBOW,
		JT_LEFTHAND,
		JT_RIGHTSHOULDER,
		JT_RIGHTELBOW,
		JT_RIGHTHAND,
		JT_TORSO,
		JT_PELVIS,
		JT_LEFTHIP,
		JT_LEFTKNEE,
		JT_LEFTFOOT,
		JT_RIGHTHIP,
		JT_RIGHTKNEE,
		JT_RIGHTFOOT
	};

	// degrees of freedom
	enum JointDOF {
		JD_X = 0,
		JD_Y,
		JD_Z,
		JD_XY,
		JD_XZ,
		JD_YZ,
		JD_XYZ
	};

	class Constraint;
	class Joint;
	class FeaturePoint;
	
	typedef double (*EnergyFunction)(const Joint*, void*);
	typedef bool (*ClassificatorFunction)(const FeaturePoint*, const Joint*, const std::vector<FeaturePoint*>&, Joint*);

	/************************************************************************
		Each joint can only have one destination joint, which will be connected
		to this joint with a bone. Additionally, it is possible to define
		subjoints, that are fixed at the same position but connect to different
		joints.
	*************************************************************************/
	enum JointClass
	{
		JC_HINGE,
		JC_BALLANDSOCKET,
		JC_CONNECTOR,
		JC_ENDCONNECTOR,
	};

	/************************************************************************
	  Joint: base class for all joints
	*************************************************************************/
	class Joint2Joint;
	class JointEndConnector;

	// NOTE: instead of using friend class, using the accessor pattern would probably
	// be better (see OpenSFFeatures::FeaturePoint)

	class Joint
	{
		friend class Joint2Joint;	// can it be avoided?
		friend class Constraint;

	public:
		virtual ~Joint();

		void addSubJoint(Joint*);
		bool detachSubJoint(Joint*);
		JointType getType() const;
		JointClass getClass() const;
		const cv::Point3d& getPos3d() const;
		const cv::Point2d& getPos2d() const;
		void setPos3d(cv::Point3d);
		static void setProjMat(const cv::Mat&);
		static const cv::Mat& getProjMat();
		void addPos3d(cv::Point3d);
		virtual void addBoneSize(double);
		virtual double getOverallBoneSize() const;
		double getBoneScaleFactor() const;
		Joint* getPrevJoint() const;
		void setEnergyFunction(EnergyFunction, void*);
		EnergyFunction getEnergyFunction() const;
		const std::vector<Joint*>& getSubJoints() const;
		bool isSubJoint() const;
		virtual Joint* getJoint(JointType);
		virtual void getSubEndConnectors(std::vector<JointEndConnector*>&);
		void setAssigned(bool);
		bool getAssigned() const;
		void setFixed(bool);
		bool isFixed() const;

		// quaternion rotation
		const quaternion<double>& getLocalQuat() const;
		void setLocalQuat(quaternion<double>);
		void getLocalQuatAngles(double& angleX, double& angleY, double& angleZ) const;
		cv::Point3d getLocalQuatAngles() const;
		quaternion<double> getGlobalQuat() const;
		void getGlobalQuatAngles(double& angleX, double& angleY, double& angleZ) const;
		cv::Point3d getGlobalQuatAngles() const;
		virtual void setAsStandard();
		virtual void setToStandard();
		bool energyChanged() const;
		void energyChanged();
		virtual void allEnergiesChanged();
		
		double computeEnergy() const;
		virtual double computeForwardEnergy();
		virtual void updateForward(bool check = true);

	protected:
		Joint(JointClass, JointType);
		void setPrevJoint(Joint*);
		void setPos3dIndep(cv::Point3d);
		
		quaternion<double> m_localRotQuat;

	private:
		quaternion<double>	m_stdLocalRotQuat;
		double				m_stdOverallBoneSize;
		bool				m_isSubJoint;
		bool				m_isAssigned;
		Joint*				m_prevJoint;
		std::vector<Joint*> m_subJoints;
		cv::Point3d			m_pos3d;
		cv::Point2d			m_pos2d;
		static cv::Mat		m_projMat;
		EnergyFunction		m_energyFunc;
		void*				m_energyFuncCookie;
		const JointType		m_jointType;
		const JointClass	m_jointClass;
		bool				m_isFixed;

		// performance: only compute energy once. if nothing changed, no need to recompute it...
		bool			m_useEnergyBoost;
		mutable bool	m_energyChanged;
		mutable double	m_lastEnergy;
	};

	/************************************************************************
	  Joint2Joint: a joint connecting to another joint with a bone and a
	  rotation quaternion
	*************************************************************************/
	class Joint2Joint
		: public Joint
	{
	public:
		virtual ~Joint2Joint();
		
		Joint* getDestination() const;
		double getBoneLength() const;
		cv::Point3d getLocalDirection() const;
		cv::Point3d getGlobalDirection() const;
		virtual Joint* getJoint(JointType);
		virtual void getSubEndConnectors(std::vector<JointEndConnector*>&);
		virtual void addBoneSize(double);
		virtual double getOverallBoneSize() const;
		
		virtual double computeForwardEnergy();
		virtual void updateForward(bool check = true);
		
		virtual void setAsStandard();
		virtual void setToStandard();
		virtual void allEnergiesChanged();

	protected:
		Joint2Joint(JointClass, JointType, Joint* destination, double boneLength);
		Joint2Joint(JointClass, JointType, Joint* destination, double boneLength, Constraint* constraint);

		Constraint* m_constraint;

	private:
		Joint* m_destination;
		double m_boneLength;
	};

	/************************************************************************
	  JointHinge: a hinge joint connecting to another joint, defined by an
	  axis to rotate around
	*************************************************************************/
	class JointHinge
		: public Joint2Joint
	{
	public:
		JointHinge(JointType, Joint* destination, double boneLength, cv::Point3d axis);
		JointHinge(JointType, Joint* destination, double boneLength, cv::Point3d axis, Constraint* constraint);
		~JointHinge();

		void setAngle(double angle);
		void addAngle(double angle);

	private:
		cv::Point3d m_axis;
	};

	/************************************************************************
	  JointBallAndSocket: a ball and socket joint connecting to another joint
	*************************************************************************/
	class JointBallAndSocket
		: public Joint2Joint
	{
	public:
		JointBallAndSocket(JointType, Joint* destination, double boneLength);
		JointBallAndSocket(JointType, Joint* destination, double boneLength, Constraint* constraint);
		~JointBallAndSocket();

		void setOrientation(double angleX, double angleY, double angleZ);
		void setOrientation(const cv::Point3d&);
		void addOrientation(double angleX, double angleY, double angleZ);
		void addOrientation(const cv::Point3d&);
	};

	/************************************************************************
	  JointConnector: a joint that does not connect to any other joint, may
	  only have subjoints and is also not an end-connector.
	*************************************************************************/
	class JointConnector
		: public Joint
	{
	public:
		JointConnector(JointType);
		~JointConnector();
	};

	/************************************************************************
	  JointEndConnector: a joint that does not connect to any other joint,
	  but may have subjoints
	*************************************************************************/
	class JointEndConnector
		: public Joint
	{
	public:
		JointEndConnector(JointType);
		~JointEndConnector();
		
		void setClassFunc(ClassificatorFunction);
		ClassificatorFunction getClassFunc() const;
		virtual void getSubEndConnectors(std::vector<JointEndConnector*>&);

	private:
		ClassificatorFunction m_classFunc;
	};
}
