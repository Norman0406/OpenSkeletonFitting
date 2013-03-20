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
#include "../CMinPack/minpack.h"

extern "C" {
	typedef int (*minpack_func_mn)(void *p, int m, int n, const double *x, double *fvec,
								   int iflag );
	int lmdif1(minpack_func_mn fcn, void *p, int m, int n, double *x, 
		double *fvec, double tol, int *iwa, 
		double *wa, int lwa);
	int lmdif(minpack_func_mn fcn, void *p, int m, int n, double *x, 
		double *fvec, double ftol, double xtol, double
		gtol, int maxfev, double epsfcn, double *diag, int
		mode, double factor, int nprint, int *
		nfev, double *fjac, int ldfjac, int *ipvt, double *
		qtf, double *wa1, double *wa2, double *wa3, double *
		wa4);
	double dpmpar_(const int *i__);
}

namespace osf
{
	class Joint;
	class Skeleton;

	class SkeletonFitting
	{
	public:
		SkeletonFitting(Skeleton*);
		~SkeletonFitting();

		enum MinimizationTechnique
		{
			MT_SIMPLE_CCD = 0,	// recursive simple minimization based on cyclic coordinate descent
			MT_CMINPACK_CCD,	// minpack ccd minimization
		};

		void update(int maxIter = 1, double changeThresh = 0.001, bool minimizeSize = true);
		void setTechnique(MinimizationTechnique);
		
	private:
		// Simple minimization
		typedef void (*JointCallback)(Joint*, void*, double);
		void updateIK(Joint*);
		void updateInverse(Joint*);
		void minimizeSimpleCCD(Joint*, double, JointCallback, void*);
		void minimizeSimpleCCD(Joint*, double, JointCallback, void*,
			const int maxIter, const double changeThresh);
		
		static void jointCallbackPos(Joint* joint, void* cookie, double val);
		static void jointCallbackSize(Joint* joint, void* cookie, double val);
		static void jointCallbackBAS(Joint* joint, void* cookie, double val);
		static void jointCallbackHinge(Joint* joint, void* cookie, double val);
		
		// CMinPack minimization
		void updateCMP(Joint*);
		static int funcPos(void*, int, int, const double*, double*, int);
		static int funcSize(void*, int, int, const double*, double*, int);
		static int funcBAS(void*, int, int, const double*, double*, int);
		static int funcHinge(void*, int, int, const double*, double*, int);
		int minimizeSimple(void*, minpack_func_mn, int, int, double*, double tol = 0.);
		int minimize(void*, minpack_func_mn, int, int, double*, double tol = 0., int iterFac = 200,
			double factor = 100., double eps = 0.);

		int minimizeSteepestDesc(void*, minpack_func_mn, int, int, double*);
		
		// This struct is passed to the energy functions to avoid multiple functions for
		// every parameter. The member axis specifies, which parameter to evaluate. [12/19/2011 Norman]
		struct Payload {
			Joint* joint;
			int axis;

			Payload(Joint* j, int a) {
				joint = j;
				axis = a;
			}
		};
		
		Joint*	m_rootJoint;
		MinimizationTechnique m_technique;
		bool m_minimizeSize;
	};
}