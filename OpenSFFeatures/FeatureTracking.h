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
#include "../OpenSFitting/Joint.h"

namespace osf
{
	class FeaturePointAccessor;

	class FeatureTracking
	{
	public:
		FeatureTracking(std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >&, const cv::Mat&);
		~FeatureTracking(void);

		void process();
		bool getPointByLabel(cv::Point3d&, int) const;
		const FeaturePoint* getPointByLabel(int) const;
		void getLabelList(std::vector<int>&) const;
		const std::vector<FeaturePoint*>& getFeaturePoints() const;
		
		// parameters
		void setSearchRadius(double);
		void setFeatureLifespan(int);
		double getSearchRadius() const;
		int getFeatureLifespan() const;

	private:
		void track();
		void extrapolate();
		void stepLifetime();
		void computeFeatureFeatures();
		void clearFeatures();
		void markFeature(int);
		std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >& m_inFeatures;
		std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > > m_prevInFeatures;

		// Tracking assigns every current feature point a previous feature point and sets
		// the same label to it. Features that existed in the previous frame but do not exist
		// int the current frame, or vice versa, are assigned a -1 label. [1/26/2012 Norman]
		
		struct SimpleTrackingItem {
			int trackingLabel;
			int listIndex;
			cv::Point3d globalPos;
			cv::Point3d relativePos;

			SimpleTrackingItem(cv::Point3d gPos, cv::Point3d rPos, int index, int label = -1) {
				globalPos = gPos;
				relativePos = rPos;
				listIndex = index;
				trackingLabel = label;
			}
		};

		// first item of the pair is the global, second the relative position
		std::vector<SimpleTrackingItem> m_curFeatures;
		std::vector<SimpleTrackingItem> m_prevFeatures;
		const cv::Mat& m_projMat;

		// contains a list of tracked feature points including additional data
		std::map<int, FeaturePointAccessor*> m_trackedFeaturesAcc;
		std::vector<FeaturePoint*> m_trackedFeatures;
		
		int		m_maxLabel;			// current maximum label
		cv::Point3d m_curTorsoPos;

		// temp
		static int counter;
		
		// parameters
		double	m_searchRadius;		// search radius to look for matching feature points
		int		m_featureLifespan;	// life span for extrapolation when a feature point was lost
	};
}
