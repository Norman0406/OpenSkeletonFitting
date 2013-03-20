#include "precompiled.h"
#include "Joint.h"
#include "Constraint.h"
#include "../OpenSF/Utils.h"
#include "../OpenSF/Exception.h"

namespace osf
{
	/************************************************************************
	  Joint
	*************************************************************************/
	cv::Mat Joint::m_projMat;

	Joint::Joint(JointClass jClass, JointType jType)
		: m_jointClass(jClass), m_jointType(jType), m_prevJoint(0)
	{
		m_pos3d = cv::Point3d(0, 0, 0);
		m_pos2d = cv::Point2d(0, 0);
		m_energyFunc = 0;
		m_energyFuncCookie = 0;
		m_isSubJoint = false;
		m_localRotQuat = quaternion<double>(0, 0, 0, 0);
		m_stdLocalRotQuat = quaternion<double>(0, 0, 0, 0);
		m_stdOverallBoneSize = 0;
		m_isAssigned = false;
		m_isFixed = false;
		m_lastEnergy = 0;
		m_useEnergyBoost = true;
		energyChanged();
		euler2Quat(m_localRotQuat, 0, 0, 0);
	}

	Joint::~Joint()
	{
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			delete m_subJoints[i];
		m_subJoints.clear();
	}

	void Joint::addSubJoint(Joint* subJoint)
	{
		if (subJoint->getType() != getType())
			throw Exception("types have to equal");
		
		if (subJoint->getType() != getType())
			throw Exception("types do not equal");

		subJoint->m_prevJoint = this;
		subJoint->m_isSubJoint = true;
		m_subJoints.push_back(subJoint);
	}

	bool Joint::detachSubJoint(Joint* subJoint)
	{
		// don't delete memory, just detach subjoint
		for (std::vector<Joint*>::iterator it = m_subJoints.begin(); it != m_subJoints.end(); it++) {
			if (*it == subJoint) {
				m_subJoints.erase(it);
				return true;
			}
		}

		return false;
	}

	JointType Joint::getType() const
	{
		return m_jointType;
	}

	JointClass Joint::getClass() const
	{
		return m_jointClass;
	}
	
	void Joint::setPrevJoint(Joint* prevJoint)
	{
		m_prevJoint = prevJoint;
	}
	
	Joint* Joint::getPrevJoint() const
	{
		return m_prevJoint;
	}

	void Joint::setPos3dIndep(cv::Point3d pos3d)
	{
		if (pos3d != m_pos3d) {
			m_pos3d = pos3d;
			m_energyChanged = true;
		}

		if (!m_projMat.empty())
			pointXYZ2UV(m_projMat, m_pos3d, m_pos2d);
	}
	
	void Joint::setEnergyFunction(EnergyFunction func, void* cookie)
	{
		m_energyFunc = func;
		m_energyFuncCookie = cookie;
		energyChanged();
	}

	EnergyFunction Joint::getEnergyFunction() const
	{
		return m_energyFunc;
	}

	const cv::Point3d& Joint::getPos3d() const
	{
		return m_pos3d;
	}

	const cv::Point2d& Joint::getPos2d() const
	{
		return m_pos2d;
	}
	
	void Joint::setPos3d(cv::Point3d pos3d)
	{
		if (!m_prevJoint) {
			if (pos3d != m_pos3d) {
				m_pos3d = pos3d;
				energyChanged();
			}

			if (!m_projMat.empty())
				pointXYZ2UV(m_projMat, m_pos3d, m_pos2d);
		}
		else
			WARN << "position of this joint cannot be set" << ENDL;
	}

	void Joint::setProjMat(const cv::Mat& projMat)
	{
		m_projMat = projMat;
	}

	const cv::Mat& Joint::getProjMat()
	{
		return m_projMat;
	}
	
	void Joint::addPos3d(cv::Point3d pos3d)
	{
		if (!m_prevJoint) {
			if (pos3d != cv::Point3d(0, 0, 0)) {
				m_pos3d += pos3d;
				energyChanged();
			}
			
			if (!m_projMat.empty())
				pointXYZ2UV(m_projMat, m_pos3d, m_pos2d);
		}
		else
			WARN << "position of this joint cannot be set" << ENDL;
	}
	
	void Joint::addBoneSize(double size)
	{
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			m_subJoints[i]->addBoneSize(size);
	}
	
	double Joint::getOverallBoneSize() const
	{
		double size = 0;
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			size += m_subJoints[i]->getOverallBoneSize();
		return size;
	}
	
	double Joint::getBoneScaleFactor() const
	{
		double boneSize = getOverallBoneSize();
		return m_stdOverallBoneSize > 0 ? boneSize / m_stdOverallBoneSize
			: 1.0;
	}
	
	const std::vector<Joint*>& Joint::getSubJoints() const
	{
		return m_subJoints;
	}

	bool Joint::isSubJoint() const
	{
		return m_isSubJoint;
	}
	
	Joint* Joint::getJoint(JointType type)
	{
		if (!isSubJoint() && getType() == type)
			return this;

		for (int i = 0; i < (int)m_subJoints.size(); i++) {
			Joint* found = m_subJoints[i]->getJoint(type);
			if (found)
				return found;
		}

		return 0;
	}
	
	void Joint::getSubEndConnectors(std::vector<JointEndConnector*>& joints)
	{
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			m_subJoints[i]->getSubEndConnectors(joints);
	}
	
	void Joint::setAssigned(bool assigned)
	{
		m_isAssigned = assigned;
	}

	bool Joint::getAssigned() const
	{
		return m_isAssigned;
	}

	void Joint::setFixed(bool fixed)
	{
		m_isFixed = fixed;
	}

	bool Joint::isFixed() const
	{
		return m_isFixed;
	}
	
	const quaternion<double>& Joint::getLocalQuat() const
	{
		return m_localRotQuat;
	}

	void Joint::setLocalQuat(quaternion<double> localQuat)
	{
		if (localQuat != m_localRotQuat) {
			m_localRotQuat = localQuat;
			energyChanged();
		}
	}

	void Joint::getLocalQuatAngles(double& angleX, double& angleY, double& angleZ) const
	{
		quat2Euler(getLocalQuat(), angleX, angleY, angleZ);
	}

	cv::Point3d Joint::getLocalQuatAngles() const
	{
		cv::Point3d result(0, 0, 0);
		getLocalQuatAngles(result.x, result.y, result.z);
		return result;
	}

	quaternion<double> Joint::getGlobalQuat() const
	{
		// NOTE: might be better to store the global quat only once instead of
		// recomputing it every time [12/16/2011 Norman]

		Joint* prevJoint = m_prevJoint;
		if (prevJoint)
			return getLocalQuat() * prevJoint->getGlobalQuat();

		return getLocalQuat();
	}

	void Joint::getGlobalQuatAngles(double& angleX, double& angleY, double& angleZ) const
	{
		quat2Euler(getGlobalQuat(), angleX, angleY, angleZ);
	}

	cv::Point3d Joint::getGlobalQuatAngles() const
	{
		cv::Point3d result(0, 0, 0);
		getGlobalQuatAngles(result.x, result.y, result.z);
		return result;
	}

	void Joint::setAsStandard()
	{
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			m_subJoints[i]->setAsStandard();

		m_stdLocalRotQuat = m_localRotQuat;
	}

	void Joint::setToStandard()
	{
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			m_subJoints[i]->setToStandard();

		m_localRotQuat = m_stdLocalRotQuat;
		m_stdOverallBoneSize = getOverallBoneSize();
		energyChanged();
	}
	
	bool Joint::energyChanged() const
	{
		if (!m_useEnergyBoost)
			return true;

		return m_energyChanged;
	}

	void Joint::energyChanged()
	{
		m_energyChanged = true;
	}

	void Joint::allEnergiesChanged()
	{
		for (int i = 0; i < (int)m_subJoints.size(); i++)
			m_subJoints[i]->energyChanged();

		energyChanged();
	}

	double Joint::computeEnergy() const
	{
		if (m_energyFunc) {
			if (energyChanged()) {
				m_lastEnergy = m_energyFunc(this, m_energyFuncCookie);
				m_energyChanged = false;
			}

			return m_lastEnergy;
		}

		return 0;
	}
	
	double Joint::computeForwardEnergy()
	{
		double energy = 0;

		for (int i = 0; i < (int)m_subJoints.size(); i++)
			energy += m_subJoints[i]->computeForwardEnergy();
		
		// get energy of this joint
		if (!isSubJoint())
			energy += computeEnergy();

		return energy;
	}

	void Joint::updateForward(bool check)
	{
		// sub joints are supposed to be at the position of the parent joint
		for (int i = 0; i < (int)m_subJoints.size(); i++) {
			m_subJoints[i]->setPos3dIndep(getPos3d());
			m_subJoints[i]->updateForward(check);
		}
	}

	/************************************************************************
	  Joint2Joint
	*************************************************************************/
	Joint2Joint::Joint2Joint(JointClass jClass, JointType jType, Joint* destination, double boneLength)
		: Joint(jClass, jType), m_destination(destination),
		m_boneLength(boneLength), m_constraint(0)
	{
		m_destination->setPrevJoint(this);
	}
	
	Joint2Joint::Joint2Joint(JointClass jClass, JointType jType, Joint* destination, double boneLength, Constraint* constraint)
		: Joint(jClass, jType), m_destination(destination),
		m_boneLength(boneLength), m_constraint(constraint)
	{
		m_destination->setPrevJoint(this);
		
		m_localRotQuat = quaternion<double>(0, 0, 0, 0);
		euler2Quat(m_localRotQuat, 0, 0, 0);
	}

	Joint2Joint::~Joint2Joint()
	{
	}

	Joint* Joint2Joint::getDestination() const
	{
		return m_destination;
	}

	double Joint2Joint::getBoneLength() const
	{
		return m_boneLength;
	}

	cv::Point3d Joint2Joint::getLocalDirection() const
	{
		quaternion<double> quat = getLocalQuat();
		quaternion<double> position(0, 0, 1, 0);
		quaternion<double> newPos = conj(quat) * position * quat;
		cv::Point3d vector(newPos.R_component_2(), newPos.R_component_3(), newPos.R_component_4());
		vector *= 1.0 / cv::norm(vector);

		return vector;
	}

	cv::Point3d Joint2Joint::getGlobalDirection() const
	{
		quaternion<double> quat = getGlobalQuat();
		quaternion<double> position(0, 0, 1, 0);
		quaternion<double> newPos = conj(quat) * position * quat;
		cv::Point3d vector(newPos.R_component_2(), newPos.R_component_3(), newPos.R_component_4());
		vector *= 1.0 / cv::norm(vector);

		return vector;
	}
	
	Joint* Joint2Joint::getJoint(JointType type)
	{
		Joint* found = m_destination->getJoint(type);
		return found ? found : Joint::getJoint(type);
	}
	
	void Joint2Joint::getSubEndConnectors(std::vector<JointEndConnector*>& joints)
	{
		m_destination->getSubEndConnectors(joints);
		Joint::getSubEndConnectors(joints);
	}

	void Joint2Joint::addBoneSize(double size)
	{
		Joint::addBoneSize(size);
		m_destination->addBoneSize(size);
		m_boneLength += size;
	}
	
	double Joint2Joint::getOverallBoneSize() const
	{
		// get sub joint sizes
		double size = Joint::getOverallBoneSize();

		// get this bone length
		size += m_boneLength;

		// get destination bone sizes
		size += m_destination->getOverallBoneSize();

		return size;
	}

	void Joint2Joint::setAsStandard()
	{
		m_destination->setAsStandard();
		Joint::setAsStandard();
	}

	void Joint2Joint::setToStandard()
	{
		m_destination->setToStandard();
		Joint::setToStandard();
	}

	void Joint2Joint::allEnergiesChanged()
	{
		m_destination->energyChanged();
		Joint::allEnergiesChanged();
	}

	double Joint2Joint::computeForwardEnergy()
	{
		double energy = Joint::computeForwardEnergy();
		energy += getDestination()->computeForwardEnergy();

		return energy;
	}

	void Joint2Joint::updateForward(bool check)
	{
		// update sub joints
		Joint::updateForward(check);
		
		quaternion<double> global = getGlobalQuat();

		// check constraint
		if (m_constraint && check)
			m_constraint->check(this);
		
		// rotate point to new position
		global = getGlobalQuat();
		quaternion<double> position(0, 0, m_boneLength, 0);
		quaternion<double> newPos = conj(global) * position * global;
		cv::Point3d vector(newPos.R_component_2(), newPos.R_component_3(), newPos.R_component_4());
		m_destination->setPos3dIndep(getPos3d() + vector);
		
		// update destination joint
		m_destination->updateForward(check);
	}

	/************************************************************************
	  JointHinge
	*************************************************************************/
	JointHinge::JointHinge(JointType jType, Joint* destination, double boneLength, cv::Point3d axis)
		: Joint2Joint(JC_HINGE, jType, destination, boneLength), m_axis(axis)
	{
	}

	JointHinge::JointHinge(JointType jType, Joint* destination, double boneLength, cv::Point3d axis, Constraint* constraint)
		: Joint2Joint(JC_HINGE, jType, destination, boneLength, constraint), m_axis(axis)
	{
	}

	JointHinge::~JointHinge()
	{
	}
	
	void JointHinge::setAngle(double angle)
	{
		if (!isFixed())
			axis2Quat(m_localRotQuat, m_axis, angle);
	}

	void JointHinge::addAngle(double angle)
	{
		if (!isFixed()) {
			quaternion<double> quat(0, 0, 0, 0);
			axis2Quat(quat, m_axis, angle);
			m_localRotQuat = m_localRotQuat * quat;
		}
	}

	/************************************************************************
	  JointBallAndSocket
	*************************************************************************/
	JointBallAndSocket::JointBallAndSocket(JointType jType, Joint* destination, double boneLength)
		: Joint2Joint(JC_BALLANDSOCKET, jType, destination, boneLength)
	{
	}

	JointBallAndSocket::JointBallAndSocket(JointType jType, Joint* destination, double boneLength, Constraint* constraint)
		: Joint2Joint(JC_BALLANDSOCKET, jType, destination, boneLength, constraint)
	{
	}

	JointBallAndSocket::~JointBallAndSocket()
	{
	}
	
	void JointBallAndSocket::setOrientation(double angleX, double angleY, double angleZ)
	{
		if (!isFixed())
			euler2Quat(m_localRotQuat, angleX, angleY, angleZ);
	}

	void JointBallAndSocket::setOrientation(const cv::Point3d& angles)
	{
		setOrientation(angles.x, angles.y, angles.z);
	}
	
	void JointBallAndSocket::addOrientation(double angleX, double angleY, double angleZ)
	{
		if (!isFixed()) {
			quaternion<double> quat(0, 0, 0, 0);
			euler2Quat(quat, angleX, angleY, angleZ);
			m_localRotQuat = quat * m_localRotQuat;
		}
	}

	void JointBallAndSocket::addOrientation(const cv::Point3d& angles)
	{
		addOrientation(angles.x, angles.y, angles.z);
	}

	/************************************************************************
	  JointConnector
	*************************************************************************/
	JointConnector::JointConnector(JointType jType)
		: Joint(JC_CONNECTOR, jType)
	{
	}

	JointConnector::~JointConnector()
	{
	}

	/************************************************************************
	  JointEndConnector
	*************************************************************************/	
	JointEndConnector::JointEndConnector(JointType jType)
		: Joint(JC_ENDCONNECTOR, jType), m_classFunc(0)
	{
	}

	JointEndConnector::~JointEndConnector()
	{
	}
	
	void JointEndConnector::setClassFunc(ClassificatorFunction func)
	{
		m_classFunc = func;
	}

	ClassificatorFunction JointEndConnector::getClassFunc() const
	{
		return m_classFunc;
	}
		
	void JointEndConnector::getSubEndConnectors(std::vector<JointEndConnector*>& joints)
	{
		joints.push_back(this);
		Joint::getSubEndConnectors(joints);
	}
}
