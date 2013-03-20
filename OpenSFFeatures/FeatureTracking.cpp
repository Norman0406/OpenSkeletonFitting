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
#include "../OpenSFitting/Joint.h"
#include "FeatureTracking.h"
#include "FeaturePoint.h"

namespace osf
{
	int FeatureTracking::counter = 0;	// temp

	FeatureTracking::FeatureTracking(std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >& features,
		const cv::Mat& projMat)
		: m_inFeatures(features), m_projMat(projMat)
	{
		m_maxLabel = 0;
		m_searchRadius = 0.3;
		m_featureLifespan = 1;
	}

	FeatureTracking::~FeatureTracking(void)
	{
		clearFeatures();
	}

	void FeatureTracking::clearFeatures()
	{
		m_prevFeatures.clear();
		m_curFeatures.clear();

		for (std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end(); it++)
			delete it->second;
		m_trackedFeaturesAcc.clear();
	}

	const FeaturePoint* FeatureTracking::getPointByLabel(int label) const
	{
		if (label < 0) {
			WARN << "invalid label: " << label << ENDL;
			return 0;
		}

		// find tracked feature point
		std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.find(label);
		if (it == m_trackedFeaturesAcc.end()) {
			WARN << "invalid label: " << label << ENDL;
			return 0;
		}

		return it->second;
	}

	bool FeatureTracking::getPointByLabel(cv::Point3d& point, int label) const
	{
		const FeaturePoint* ftPoint = getPointByLabel(label);
		if (!ftPoint)
			return false;
		
		point = ftPoint->getPosition3d();

		return true;
	}

	void FeatureTracking::getLabelList(std::vector<int>& list) const
	{
		list.clear();
		for (std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end(); it++) {
			list.push_back(it->second->getTrackingLabel());
		}
	}

	void FeatureTracking::setSearchRadius(double searchRadius)
	{
		m_searchRadius = searchRadius;
	}

	void FeatureTracking::setFeatureLifespan(int lifespan)
	{
		m_featureLifespan = lifespan;
	}

	double FeatureTracking::getSearchRadius() const
	{
		return m_searchRadius;
	}

	int FeatureTracking::getFeatureLifespan() const
	{
		return m_featureLifespan;
	}

	const std::vector<FeaturePoint*>& FeatureTracking::getFeaturePoints() const
	{
		return m_trackedFeatures;
	}

	void FeatureTracking::stepLifetime()
	{
		// add up the lifetime values of every tracked feature point
		for (std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end(); it++) {
			it->second->stepLifetime();
		}
	}
	
	void FeatureTracking::markFeature(int label)
	{
		std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.find(label);
		if (it == m_trackedFeaturesAcc.end()) {
			WARN << "label not found: " << label << ENDL;
			return;
		}

		FeaturePointAccessor* ftPoint = m_trackedFeaturesAcc[label];
		
		// TODO: find a nicer approach for this

		// some problems here confusing assignment
		if (ftPoint->getJointLabel() == JT_TORSO)
			ftPoint->setJointLabel(JT_UNKNOWN);

		ftPoint->setConfirmState(FeaturePoint::CS_UNCONFIRMED);
	}

	void FeatureTracking::track()
	{
		/*if (counter == 42)
			WARN << "warn" << ENDL;*/

		// Step 1: try to find a best matching for every previously tracked feature point in
		// the list of current feature points
		for (std::vector<SimpleTrackingItem>::const_iterator itPrev = m_prevFeatures.begin();
				itPrev != m_prevFeatures.end(); itPrev++) {

			// skip point that have not been tracked before
			if (itPrev->trackingLabel < 0)
				continue;
			
			cv::Point3d prevPos = itPrev->relativePos;

			typedef std::vector<SimpleTrackingItem>::iterator iter;
			std::vector<iter> closestPoints;

			// find closest points to the current location
			for (std::vector<SimpleTrackingItem>::iterator itCur = m_curFeatures.begin();
					itCur != m_curFeatures.end(); itCur++) {
				cv::Point3d curPos = itCur->relativePos;
				double dist = distanceP3d(curPos, prevPos);
				
				if (dist < m_searchRadius) {

					// check if this point has already been assigned and chose whether a reassignment
					// is necessary
					bool pushToList = true;
					if (itCur->trackingLabel >= 0) {
						// get the label of the already assigned point
						int label = itCur->trackingLabel;

						// find the matching point in the list of previous features
						cv::Point3d assignedPos(0, 0, 0);
						bool found = false;
						for (std::vector<SimpleTrackingItem>::const_iterator it = m_prevFeatures.begin();
								it != m_prevFeatures.end(); it++) {
							if (it->trackingLabel == label) {
								assignedPos = it->relativePos;
								found = true;
								break;
							}
						}
						
						// should never happen
						if (!found) {
							WARN << "matching label not found, this should not happen" << ENDL;
							break;
						}

						// if the already assigned distance is smaller, then no assignment necessary
						double assignedDist = distanceP3d(curPos, assignedPos);
						if (assignedDist < dist)
							pushToList = false;
					}

					// add to list of closest neighboring points
					if (pushToList)
						closestPoints.push_back(itCur);
				}
			}

			// find nearest point
			double minDist = std::numeric_limits<double>::infinity();
			int minDistIndex = -1;
			for (int i = 0; i < (int)closestPoints.size(); i++) {
				double dist = distanceP3d(prevPos, closestPoints[i]->relativePos);

				if (dist < minDist) {
					minDist = dist;
					minDistIndex = i;
				}
			}
			
			// match with nearest point
			if (minDistIndex >= 0) {
				// if this point was already assigned a value, remove it first
				if (closestPoints[minDistIndex]->trackingLabel >= 0) {
					// mark feature "to be removed" and set label to JT_UNKNOWN
					markFeature(closestPoints[minDistIndex]->trackingLabel);
				}

				// assign the same label
				closestPoints[minDistIndex]->trackingLabel = itPrev->trackingLabel;

				// get feature point and set right values
				FeaturePointAccessor* ftPoint = m_trackedFeaturesAcc[itPrev->trackingLabel];
				if (!ftPoint || ftPoint->getTrackingLabel() != itPrev->trackingLabel)
					throw Exception("feature point invalid");
				
				if (closestPoints[minDistIndex]->listIndex >= 0)
					ftPoint->setConfirmState(FeaturePoint::CS_CONFIRMED);

				// set label
				JointType jLabel = JT_UNKNOWN;
				
				if (closestPoints[minDistIndex]->listIndex >= 0)
					jLabel = m_inFeatures[closestPoints[minDistIndex]->listIndex].first;
				else
					jLabel = ftPoint->getJointLabel();

				if (jLabel != JT_UNKNOWN)
					ftPoint->setJointLabel(jLabel);
			}
			else {
				// mark feature "to be removed"
				markFeature(itPrev->trackingLabel);
			}
		}

		/*if (counter >= 42)
			goto step3;*/

		// Step 2: assign every not-yet assigned current feature point a valid not-tracked
		// feature point in the list of previous features
		for (std::vector<SimpleTrackingItem>::iterator itCur = m_curFeatures.begin();
				itCur != m_curFeatures.end(); itCur++) {

			// skip feature points that are already tracked by previous step
			if (itCur->trackingLabel >= 0)
				continue;

			cv::Point3d curPos = itCur->relativePos;

			typedef std::vector<SimpleTrackingItem>::iterator iter;
			std::vector<iter> closestPoints;

			// find closest points to the current location
			for (std::vector<SimpleTrackingItem>::iterator itPrev = m_prevFeatures.begin();
					itPrev != m_prevFeatures.end(); itPrev++) {

				// skip previous features that are already tracked
				if (itPrev->trackingLabel >= 0)
					continue;

				cv::Point3d prevPos = itPrev->relativePos;
				double dist = distanceP3d(curPos, prevPos);
				
				if (dist < m_searchRadius) {
					// add to list of closest neighboring points
					closestPoints.push_back(itPrev);
				}
			}
			
			// create new feature point
			FeaturePointAccessor* ftPoint = new FeaturePointAccessor(m_maxLabel);
			ftPoint->setConfirmState(FeaturePoint::CS_CONFIRMED);			
			// set label
			JointType jLabel = JT_UNKNOWN;
			
			if (itCur->listIndex >= 0)
				jLabel = m_inFeatures[itCur->listIndex].first;

			if (jLabel != JT_UNKNOWN)
				ftPoint->setJointLabel(jLabel);

			if (!closestPoints.empty()) {
				// find nearest point
				double minDist = std::numeric_limits<double>::infinity();
				int minDistIndex = -1;
				for (int i = 0; i < (int)closestPoints.size(); i++) {
					double dist = distanceP3d(curPos, closestPoints[i]->relativePos);

					if (dist < minDist) {
						minDist = dist;
						minDistIndex = i;
					}
				}

				if (minDistIndex >= 0) {
					// match with nearest point
					closestPoints[minDistIndex]->trackingLabel = m_maxLabel;
				}
				else {
					// should never happen
					delete ftPoint;
					ftPoint = 0;
				}
			}
			
			// add feature point to list
			if (ftPoint) {
				itCur->trackingLabel = m_maxLabel;
				m_trackedFeaturesAcc[m_maxLabel] = ftPoint;
				m_maxLabel++;
			}
		}
		
		// Step 3: set data for tracked feature points
		for (int i = 0; i < (int)m_curFeatures.size(); i++) {
			int label = m_curFeatures[i].trackingLabel;

			if (label < 0)
				continue;

			std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.find(label);
			if (it == m_trackedFeaturesAcc.end())
				WARN << "label not found: " << label << ENDL;
			else {
				// set position
				cv::Point2d pos2d(0, 0);
				pointXYZ2UV(m_projMat, m_curFeatures[i].globalPos, pos2d);
				it->second->setPosition(m_curFeatures[i].globalPos, m_curFeatures[i].relativePos, pos2d);
			}
		}

		counter++;
	}

	void FeatureTracking::extrapolate()
	{
		// TODO: create virtual feature points, if tracking was lost.
		// There are two types of virtual feature points. When a feature point is obscured, just
		// use its last position in relation to the torso position and normal as virtual
		// feature point for the next x frames. If tracking was lost and the point is not obscured
		// (feature extraction is not working), use nearest neighbor from the point cloud to
		// generate the virtual feature point until a valid feature was found again (or for the
		// next x frames?).
		
		// TODO: if a previous feature is not found in the list of current features, add
		// its position and label to current features again for a limited time and remove
		// it, when the time is over (1 - 5 frames)
		
		// remove feature points whose unconfirmed lifetime is too high
		for (std::map<int, FeaturePointAccessor*>::iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end();) {
			if (it->second->getLatestUnconfLifetime() > m_featureLifespan) {
				// remove entry from m_curFeatures
				for (std::vector<SimpleTrackingItem>::iterator it2 = m_curFeatures.begin();
						it2 != m_curFeatures.end();) {
					if (it2->trackingLabel == it->first) {
						it2 = m_curFeatures.erase(it2);
					}
					else
						it2++;
				}

				// delete and remove feature point
				delete it->second;
#ifdef _WIN32
				it = m_trackedFeaturesAcc.erase(it);
#elif __APPLE__ & __MACH__
				// NOTE: not sure if correct
				m_trackedFeaturesAcc.erase(it++);
#endif
			}
			else
				it++;
		}

		// add unconfirmed feature points that are not yet removed to m_curFeatures. This means, that in the
		// next step, they will be added to the list of previous features and tried to be matched against.
		for (std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end(); it++) {
			if (!it->second->isConfirmed()) {
				cv::Point3d globPos = it->second->getPosition3d();
				cv::Point3d relPos = it->second->getRelativePosition();

				globPos = relPos + m_curTorsoPos;

				cv::Point2d imgPos(0, 0);
				pointXYZ2UV(m_projMat, globPos, imgPos);
				it->second->setPosition(globPos, relPos, imgPos); 

				m_curFeatures.push_back(SimpleTrackingItem(globPos, relPos, -1, it->second->getTrackingLabel()));
			}
		}
	}

	void FeatureTracking::computeFeatureFeatures()
	{
		for (std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end(); it++) {
			it->second->computeTemporalFeatures();

			// TOOD: when the feature point is not virtual at the moment, compute local
			// image features
		}
	}

	void FeatureTracking::process()
	{
		// get torso position
		bool torsoFound = false;
		for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::const_iterator it =
				m_inFeatures.begin(); it != m_inFeatures.end(); it++) {
			if (it->first == JT_TORSO) {
				m_curTorsoPos = it->second.first;
				torsoFound = true;
			}
		}

		// only track if a torso position has been found
		if (torsoFound) {
			// copy previous current features to previous features
			m_prevFeatures.clear();
			for (std::vector<SimpleTrackingItem>::const_iterator it = m_curFeatures.begin();
					it != m_curFeatures.end(); it++)
				m_prevFeatures.push_back(*it);

			// copy incoming features to current features
			m_curFeatures.clear();
			for (int i = 0; i < (int)m_inFeatures.size(); i++) {
				m_curFeatures.push_back(SimpleTrackingItem(m_inFeatures[i].second.first,
					m_inFeatures[i].second.first - m_curTorsoPos, i));
			}

			// track
			if (!m_prevFeatures.empty()) {
				track();
				stepLifetime();
				computeFeatureFeatures();
				extrapolate();
			}

			// copy previous current input features to previous input features
			m_prevInFeatures.clear();
			for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::const_iterator it =
					m_inFeatures.begin(); it != m_inFeatures.end(); it++)
				m_prevInFeatures.push_back(*it);
		}
		else {
			m_curTorsoPos = cv::Point3d(0, 0, -1);
			clearFeatures();
		}

		// add to public tracking list
		m_trackedFeatures.clear();
		for (std::map<int, FeaturePointAccessor*>::const_iterator it = m_trackedFeaturesAcc.begin();
				it != m_trackedFeaturesAcc.end(); it++)
			m_trackedFeatures.push_back(it->second);
	}
}
