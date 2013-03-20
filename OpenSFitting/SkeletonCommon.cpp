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
#include "SkeletonHuman.h"
#include "Skeleton.h"
#include "Joint.h"
#include "Constraint.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSF/Exception.h"

namespace osf
{
	double cmEnergyAffectors(const Joint* joint, void* cookie)
	{
		if (!cookie || !joint)
			throw Exception("invalid input data");
		
		const std::vector<FeaturePoint*>* trackedFeatures = (const std::vector<FeaturePoint*>*)cookie;

		// find feature point
		std::vector<FeaturePoint*>::const_iterator it = trackedFeatures->end();
		for (it = trackedFeatures->begin(); it != trackedFeatures->end(); it++) {
			if ((*it)->getJointLabel() == joint->getType())
				break;
		}
		
		double dist = 0;
		cv::Point3d jointPos = joint->getPos3d();

		if (it == trackedFeatures->end()) {
			// not found
			dist = 0;
		}
		else {
			// found, get feature point position
			cv::Point3d pointToAffect = (*it)->getPosition3dFiltered();
			dist = distanceP3dSq(pointToAffect, jointPos);
		}

		return dist;
	}
	
	// TODO: for all joints, also sample the bone connecting two successive joints into
	// sample points and compute a nearest neighbor energy function at each of the sample points.

	double cmEnergyNearestNeighbor(const Joint* joint, void* cookie)
	{
		if (!cookie || !joint)
			throw Exception("invalid input data");

		Fitting* fitting = (Fitting*)cookie;

		cv::Point3d jointPos = joint->getPos3d();
		const int numNN = 5;

		std::vector<cv::Point3d> nearestPoints;
		std::vector<float> nearestDist;
		fitting->computeDepthKNN(numNN, jointPos, nearestPoints, nearestDist);

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
	
	double cmEnergyUnderlyingPoint(const Joint* joint, void* cookie)
	{
		if (!cookie || !joint)
			throw Exception("invalid input data");

		Fitting* fitting = (Fitting*)cookie;

		const cv::Mat* img3d = fitting->getImg3d();
		if (!img3d)
			throw Exception("3d image invalid");
		
		// check if 2d position is valid
		cv::Point posImg((int)joint->getPos2d().x, (int)joint->getPos2d().y);
		if (posImg.x < 0 || posImg.x >= img3d->cols ||
			posImg.y < 0 || posImg.y >= img3d->rows)
			return 0;
		
		// get 3d point directly under the joint image position
		cv::Point3d underlyingPoint(img3d->ptr<float>(posImg.y)[posImg.x * 3 + 0],
			img3d->ptr<float>(posImg.y)[posImg.x * 3 + 1], img3d->ptr<float>(posImg.y)[posImg.x * 3 + 2]);

		double dist = 0;
		
		// get distance to underlying 3d point
		if (underlyingPoint.z > 0) {
			cv::Point3d pos3d = joint->getPos3d();
			dist = distanceP3dSq(pos3d, underlyingPoint);
		}
		else {
			// if point is invalid, use nearest neighbor
			dist = cmEnergyNearestNeighbor(joint, cookie);
		}

		return dist;
	}
}
