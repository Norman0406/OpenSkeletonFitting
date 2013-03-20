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
#include "Skeleton.h"
#include "SkeletonFitting.h"
#include "Constraint.h"
#include "Joint.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSF/Exception.h"

namespace osf
{
	/************************************************************************
	  SkeletonFitting
	*************************************************************************/
	SkeletonFitting::SkeletonFitting(Skeleton* skeleton)
	{
		m_rootJoint = skeleton->getRoot();
		if (!m_rootJoint)
			throw Exception("invalid data");

		m_technique = MT_SIMPLE_CCD;
	}

	SkeletonFitting::~SkeletonFitting()
	{
	}
	
	void SkeletonFitting::update(int maxIter, double changeThresh, bool minimizeSize)
	{
		if (!m_rootJoint)
			throw Exception("invalid data");
		
		double change = 0;
		int iter = 0;
		m_minimizeSize = minimizeSize;

		do {
			change = m_rootJoint->computeForwardEnergy();

			switch (m_technique) {
			case MT_SIMPLE_CCD:
			case MT_CMINPACK_CCD:
				updateIK(m_rootJoint);
				break;
			default:
				throw Exception("invalid parameter");
			}

			change = m_rootJoint->computeForwardEnergy() - change;
		} while (abs(change) > changeThresh && iter++ < maxIter);
	}

	void SkeletonFitting::setTechnique(MinimizationTechnique technique)
	{
		m_technique = technique;
	}

	/************************************************************************
	  Simple minimization:
	*************************************************************************/
	void SkeletonFitting::updateIK(Joint* joint)
	{
		// update sub-joints
		const std::vector<Joint*>& subJoints = joint->getSubJoints();
		for (int i = 0; i < (int)subJoints.size(); i++)
			updateIK(subJoints[i]);

		if (joint->getClass() == JC_ENDCONNECTOR) {
			if (m_technique == MT_SIMPLE_CCD)
				updateInverse(joint);
			else
				updateCMP(joint);
		}
		else if (joint->getClass() == JC_BALLANDSOCKET || joint->getClass() == JC_HINGE) {
			Joint2Joint* connector = (Joint2Joint*)joint;
						
			// update destination joint
			updateIK(connector->getDestination());
		}
	}
	
	void SkeletonFitting::updateInverse(Joint* joint)
	{
		// TODO: use same minimization method also with MT_CMINPACK_CCD, just exchange the function

		if (!joint) {
			WARN << "joint invalid" << ENDL;	// should never happen
			return;
		}

		// parameter for callback
		const int p1 = 0, p2 = 1, p3 = 2;
		
		// if this is the root joint, also update the position
		if (!joint->getPrevJoint()) {
			const int maxIter = 50;
			const double changeThresh = 0.00000001;
			//const double changeThresh = 0.00001;

			// minimze position
			const double initialPos = 0.1;
			minimizeSimpleCCD(joint, initialPos, jointCallbackPos, (void*)&p1, maxIter, changeThresh);
			minimizeSimpleCCD(joint, initialPos, jointCallbackPos, (void*)&p2, maxIter, changeThresh);
			minimizeSimpleCCD(joint, initialPos, jointCallbackPos, (void*)&p3, maxIter, changeThresh);
			
			// minimze skeleton size
			if (m_minimizeSize) {
				const double initialSize = 0.1;
				minimizeSimpleCCD(joint, initialSize, jointCallbackSize, 0, maxIter, changeThresh);
			}
		}

		// update joint angles
		const int maxIter = 50;
		const double changeThresh = 0.0000001;
		//const double changeThresh = 0.0001;
		const double initialAngle = 1.0;

		switch (joint->getClass()) {
		case JC_BALLANDSOCKET:
			{
				// update ball and socket joints
				JointBallAndSocket* jointBAS = (JointBallAndSocket*)joint;
				
				minimizeSimpleCCD(jointBAS, initialAngle, jointCallbackBAS, (void*)&p1, maxIter, changeThresh);
				minimizeSimpleCCD(jointBAS, initialAngle, jointCallbackBAS, (void*)&p2, maxIter, changeThresh);
				minimizeSimpleCCD(jointBAS, initialAngle, jointCallbackBAS, (void*)&p3, maxIter, changeThresh);

				break;
			}
		case JC_HINGE:
			{
				// update hinge joints
				JointHinge* jointHinge = (JointHinge*)joint;

				minimizeSimpleCCD(jointHinge, initialAngle, jointCallbackHinge, 0, maxIter, changeThresh);

				break;
			}
		case JC_ENDCONNECTOR:
			// no need to update end connector joints, their position is only
			// affected by their subordinate joints
			break;
		case JC_CONNECTOR:
			// does also not need to be updated
			break;
		default:
			WARN << "unknown joint type: " << (int)joint->getClass() << ENDL;
			break;
		}
		
		// if this is not the root joint, update the next previous joint
		if (joint->getPrevJoint())
			updateInverse(joint->getPrevJoint());
	}
	
	void SkeletonFitting::minimizeSimpleCCD(Joint* joint, double val, JointCallback cbFunc, void* cookie)
	{
		minimizeSimpleCCD(joint, val, cbFunc, cookie, 50, 0.000001);
	}

	void SkeletonFitting::minimizeSimpleCCD(Joint* joint, double val, JointCallback cbFunc, void* cookie,
		const int maxIter, const double changeThresh)
	{
		double change = 0.0;
		int iter = 0;

		// to remove rotating when energy function of sub joints is zero (bad code)
		const double internalThresh = changeThresh;

		do {
			// get energy before translation
			double energy1 = joint->computeForwardEnergy();
			
			// translate
			cbFunc(joint, cookie, val);
			joint->updateForward(true);

			// get energy after translation
			double energy2 = joint->computeForwardEnergy();
			change = energy2 - energy1;
						
			// restore and change direction, if previous energy was smaller
			if (abs(change) <= internalThresh || change > internalThresh) {
				// invert direction
				val *= -1;
				
				// compute new energy
				cbFunc(joint, cookie, val);
				joint->updateForward(true);
				change = joint->computeForwardEnergy() - energy2;
				
				// decrease step width
				float decFac = 2.0f;
				val /= decFac;
			}
		} while (abs(change) > changeThresh && iter++ < maxIter);
	}

	void SkeletonFitting::jointCallbackPos(Joint* joint, void* cookie, double val)
	{
		if (!cookie)
			throw Exception("no additional data set");
		
		if (!joint->getPrevJoint()) {
			const int param = *((const int*)(cookie));
			switch (param) {
			case 0:
				joint->addPos3d(cv::Point3d(val, 0, 0));
				break;
			case 1:
				joint->addPos3d(cv::Point3d(0, val, 0));
				break;
			case 2:
				joint->addPos3d(cv::Point3d(0, 0, val));
				break;
			default:
				WARN << "invalid parameter: " << param << ENDL;
				break;
			}
		}
		else
			WARN << "invalid joint type" << ENDL;
	}

	void SkeletonFitting::jointCallbackSize(Joint* joint, void* cookie, double val)
	{
		joint->addBoneSize(val);
	}
	
	void SkeletonFitting::jointCallbackBAS(Joint* joint, void* cookie, double val)
	{
		if (!cookie)
			throw Exception("no additional data set");
		
		if (joint->getClass() == JC_BALLANDSOCKET) {
			JointBallAndSocket* jointBAS = (JointBallAndSocket*)joint;

			const int param = *((const int*)(cookie));
			switch (param) {
			case 0:
				jointBAS->addOrientation(cv::Point3d(val, 0, 0));
				break;
			case 1:
				jointBAS->addOrientation(cv::Point3d(0, val, 0));
				break;
			case 2:
				jointBAS->addOrientation(cv::Point3d(0, 0, val));
				break;
			default:
				WARN << "invalid parameter: " << param << ENDL;
				break;
			}
		}
		else
			WARN << "invalid joint type" << ENDL;
	}

	void SkeletonFitting::jointCallbackHinge(Joint* joint, void* cookie, double val)
	{
		if (joint->getClass() == JC_HINGE) {
			JointHinge* jointHinge = (JointHinge*)joint;
			jointHinge->addAngle(val);
		}
		else
			WARN << "invalid joint type" << ENDL;
	}

	/************************************************************************
	  CMinPack minimzation:
	*************************************************************************/
	void SkeletonFitting::updateCMP(Joint* joint)
	{
		// TODO: use same minimization method also with MT_CMINPACK_CCD, just exchange the function

		if (!joint) {
			WARN << "joint invalid" << ENDL;	// should never happen
			return;
		}
		
		// TODO: implement simple steepest descent method using the same functions
		const int i = 0;
		
		// if this is the root joint, also update the position
		if (!joint->getPrevJoint()) {
			const double tol = 0.000001;
			const double factor = 0.01;
			const double eps = 0.00001;
			const int iterFac = 50;

			cv::Point3d addPos(0, 0, 0);
			double addSize = 0;
			
			if (i == 0) {
				minimize(&Payload(joint, 0), funcPos, 1, 1, &addPos.x, tol, iterFac, factor, eps);
				minimize(&Payload(joint, 1), funcPos, 1, 1, &addPos.y, tol, iterFac, factor, eps);
				minimize(&Payload(joint, 2), funcPos, 1, 1, &addPos.z, tol, iterFac, factor, eps);

				if (m_minimizeSize)
					minimize(&Payload(joint, 2), funcSize, 1, 1, &addSize, tol, iterFac, factor, eps);
			}
			else {
				/*minimizeSteepestDesc(&Payload(joint, 0), funcPos, 1, 1, &addPos.x);
				minimizeSteepestDesc(&Payload(joint, 1), funcPos, 1, 1, &addPos.y);
				minimizeSteepestDesc(&Payload(joint, 2), funcPos, 1, 1, &addPos.z);
				
				if (m_minimizeSize)
					minimizeSteepestDesc(&Payload(joint, 2), funcSize, 1, 1, &addSize);*/
			}
		
			joint->addPos3d(addPos);
			joint->addBoneSize(addSize);
			joint->updateForward(false);	// changing position doesn't need constraint checking
		}
		
		// update joint angles
		const double tol = 0.001;
		const int iterFac = 50;
		double factor = 100;
		double eps = 0.001;

		switch (joint->getClass()) {
		case JC_BALLANDSOCKET:
			{
				// update ball and socket joints
				JointBallAndSocket* jointBAS = (JointBallAndSocket*)joint;

				// update angles
				cv::Point3d addRot(0, 0, 0);
			
				if (i == 0) {
					minimize(&Payload(joint, 0), funcBAS, 1, 1, &addRot.x, tol, iterFac, factor, eps);
					minimize(&Payload(joint, 1), funcBAS, 1, 1, &addRot.y, tol, iterFac, factor, eps);
					minimize(&Payload(joint, 2), funcBAS, 1, 1, &addRot.z, tol, iterFac, factor, eps);
				}
				else {
					//minimizeSteepestDesc(&Payload(joint, 0), funcBAS, 1, 1, &addRot.x);
					//minimizeSteepestDesc(&Payload(joint, 1), funcBAS, 1, 1, &addRot.y);
					//minimizeSteepestDesc(&Payload(joint, 2), funcBAS, 1, 1, &addRot.z);
				}


				jointBAS->addOrientation(addRot);
				jointBAS->updateForward(true);

				break;
			}
		case JC_HINGE:
			{
				// update hinge joints
				JointHinge* jointHinge = (JointHinge*)joint;
				
				// update angles
				double addAngle = 0;
				
				if (i == 0) {
					minimize(&Payload(joint, 0), funcHinge, 1, 1, &addAngle, tol, iterFac, factor, eps);
				}
				else {
					//minimizeSteepestDesc(&Payload(joint, 0), funcHinge, 1, 1, &addAngle);
				}

				jointHinge->addAngle(addAngle);
				jointHinge->updateForward(true);

				break;
			}
		case JC_ENDCONNECTOR:
			// no need to update end connector joints, their position is only
			// affected by their subordinate joints
			break;
		case JC_CONNECTOR:
			// does also not need to be updated
			break;
		default:
			WARN << "unknown joint type: " << (int)joint->getClass() << ENDL;
			break;
		}
		
		// if this is not the root joint, update the next previous joint
		if (joint->getPrevJoint())
			updateCMP(joint->getPrevJoint());
	}
	
	int SkeletonFitting::funcPos(void* p, int m, int n, const double* x, double* fvec, int iflag)
	{
		if (!p)
			throw Exception("invalid data");

		Payload* thisPayload = (Payload*)p;
		Joint* joint = thisPayload->joint;
		
		// store state
		cv::Point3d pos = joint->getPos3d();
		
		switch (thisPayload->axis) {
		case 0:
			joint->addPos3d(cv::Point3d(x[0], 0, 0));
			break;
		case 1:
			joint->addPos3d(cv::Point3d(0, x[0], 0));
			break;
		case 2:
			joint->addPos3d(cv::Point3d(0, 0, x[0]));
			break;
		default:
				throw Exception("invalid parameter");
			break;
		}

		// compute energy
		joint->updateForward(false);
		fvec[0] = joint->computeForwardEnergy();

		// reset
		joint->setPos3d(pos);
		joint->updateForward(false);
		
		return iflag;
	}
	
	int SkeletonFitting::funcSize(void* p, int m, int n, const double* x, double* fvec, int iflag)
	{
		if (!p)
			throw Exception("invalid data");

		Payload* thisPayload = (Payload*)p;
		Joint* joint = (JointHinge*)thisPayload->joint;
				
		// compute energy
		joint->addBoneSize(x[0]);
		joint->updateForward(false);
		fvec[0] = joint->computeForwardEnergy();

		// reset
		joint->addBoneSize(-x[0]);
		joint->updateForward(false);
		
		return iflag;
	}
	
	int SkeletonFitting::funcBAS(void* p, int m, int n, const double* x, double* fvec, int iflag)
	{
		if (!p)
			throw Exception("invalid data");

		Payload* thisPayload = (Payload*)p;
		JointBallAndSocket* joint = (JointBallAndSocket*)thisPayload->joint;
		
		// store state
		quaternion<double> rot = joint->getLocalQuat();

		switch (thisPayload->axis) {
		case 0:
			joint->addOrientation(cv::Point3d(x[0], 0, 0));
			break;
		case 1:
			joint->addOrientation(cv::Point3d(0, x[0], 0));
			break;
		case 2:
			joint->addOrientation(cv::Point3d(0, 0, x[0]));
			break;
		default:
			throw Exception("invalid parameter");
		}
		
		// compute energy
		joint->updateForward(false);
		fvec[0] = joint->computeForwardEnergy();

		// reset
		joint->setLocalQuat(rot);
		joint->updateForward(false);
		
		return iflag;
	
	}

	int SkeletonFitting::funcHinge(void* p, int m, int n, const double* x, double* fvec, int iflag)
	{
		if (!p)
			throw Exception("invalid data");

		Payload* thisPayload = (Payload*)p;
		JointHinge* joint = (JointHinge*)thisPayload->joint;

		// store state
		quaternion<double> rot = joint->getLocalQuat();
		
		// compute energy
		joint->addAngle(x[0]);
		joint->updateForward(false);
		fvec[0] = joint->computeForwardEnergy();

		// reset
		joint->setLocalQuat(rot);
		joint->updateForward(false);
		
		return iflag;
	}

	int SkeletonFitting::minimizeSteepestDesc(void* p, minpack_func_mn fcn, int m, int n, double* vals)
	{
		// UNDONE

		double change = 0.0;
		int iter = 0;

		double val = 0.1;

		double changeThresh = 0.00001;
		double maxIter = 50;

		// to remove rotating when energy function of sub joints is zero (bad code)
		const double internalThresh = changeThresh;

		// create initial estimate
		double* x = new double[n];
		for (int i = 0; i < n; i++)
			x[i] = vals[i];

		// initialize function vector
		double* fvec = new double[m];
		for (int i = 0; i < m; i++)
			fvec[i] = 0;
		
		for (int i = 0; i < n; i++) {
			do {
				// get energy before translation
				//double energy1 = joint->computeForwardEnergy();
			
				// translate
				//cbFunc(joint, cookie, val);
				//joint->updateForward(true);

				// compute energy
				fcn(p, m, n, x, fvec, 0);
				
				// get energy after translation
				//double energy2 = joint->computeForwardEnergy();
				//change = energy2 - energy1;
						
				// restore and change direction, if previous energy was smaller
				if (abs(change) <= internalThresh || change > internalThresh) {
					// invert direction
					val *= -1;
				
					// compute new energy
					//cbFunc(joint, cookie, val);
					//joint->updateForward(true);
					//change = joint->computeForwardEnergy() - energy2;
				
					// decrease step width
					float decFac = 2.0f;
					val /= decFac;
				}
			} while (abs(change) > changeThresh && iter++ < maxIter);
		}

		delete[] x;
		delete[] fvec;

		return 0;
	}

	int SkeletonFitting::minimizeSimple(void* p, minpack_func_mn fcn, int m, int n, double* vals, double tol)
	{
		double* fvec = new double[m];
						
		// work array
		int* iwa = new int[n];
		int lwa = m * n + 5 * n + m;
		double* wa = new double[lwa];
		
		int info = lmdif1(fcn, p, m, n, vals, fvec, tol, iwa, wa, lwa);
		
		delete[] fvec;
		delete[] iwa;
		delete[] wa;

		return info;
	}
	
	int SkeletonFitting::minimize(void* p, minpack_func_mn fcn, int m, int n, double* vals, double tol, int iterFac, double factor, double eps)
	{
		double* fvec = new double[m];
		
		// work array
		int* iwa = new int[n];
		int lwa = m * n + 5 * n + m;
		double* wa = new double[lwa];
		
		double* fvec2 = fvec;
		int* iwa2 = iwa;
		double* wa2 = wa;
		
		int mp5n, mode, nfev;
		double ftol, gtol, xtol;
		double epsfcn;
		int maxfev, nprint;
		int info;
		--fvec2;
		--iwa2;
		--vals;
		--wa2;
	
		if (n <= 0 || m < n || tol < 0. || lwa < m * n + n * 5 + m)
			throw Exception("invalid data");

		maxfev = (n + 1) * iterFac;
		ftol = tol;
		xtol = tol;
		gtol = 0.;
		epsfcn = eps;
		mode = 1;
		nprint = 0;
		mp5n = m + n * 5;
		info = lmdif(fcn, p, m, n, &vals[1], &fvec2[1], ftol, xtol, gtol, maxfev,
			epsfcn, &wa2[1], mode, factor, nprint, &nfev, &wa2[mp5n + 
			1], m, &iwa2[1], &wa2[n + 1], &wa2[(n << 1) + 1], &wa2[n * 3 + 1], 
			&wa2[(n << 2) + 1], &wa2[n * 5 + 1]);

		if (info == 8) {
			info = 4;
		}
		
		delete[] fvec;
		delete[] iwa;
		delete[] wa;

		return info;
	}
}
