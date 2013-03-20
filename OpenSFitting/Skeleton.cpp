#include "precompiled.h"
#include "Skeleton.h"
#include "Joint.h"
#include "../OpenSFitting/Fitting.h"

namespace osf
{
	/************************************************************************
	  Skeleton
	*************************************************************************/
	Skeleton::Skeleton(Fitting* fitting)
		: m_rootJoint(0), m_skeletonFitting(fitting)
	{
	}

	Skeleton::~Skeleton()
	{
		delete m_rootJoint;
	}

	Joint* Skeleton::getRoot()
	{
		return m_rootJoint;
	}

	void Skeleton::init()
	{
		m_rootJoint = iInit();
	}

	void Skeleton::updateFK()
	{
		if (!m_rootJoint)
			throw Exception("invalid data");

		updateFK(m_rootJoint);
	}
	
	void Skeleton::updateFK(Joint* joint)
	{
		joint->updateForward();
	}
}