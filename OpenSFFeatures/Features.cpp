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
#include "Features.h"
#include "FeatureTracking.h"
#include "FeaturesFactory.h"
#include "FeaturePoint.h"
#include "../OpenSFInput/Input.h"
#include "../OpenSFitting/Joint.h"
#include "../OpenSFLib/System.h"
#include "../OpenSF/Utils.h"
#include <cmath>

namespace osf
{
	Features::Features(System* sys)
		: Module(sys), m_inImgDepth(0), m_inImg3d(0), m_distMap(0),
		m_ftTracking(0)
	{
		m_imgDepth.setTo(0);
		m_img3d.setTo(0);
		m_imgGeodesic.setTo(0);

		addInput(m_inImgDepth);
		addInput(m_inImg3d);
		addOutput(&m_imgDepth);
		addOutput(&m_img3d);
		addOutput(&m_imgGeodesic);
		addOutput(&m_imgPredecessors);
		addOutput(&m_trackedFeatures);
		
		// standard parameters
		m_geoMaxZDistThreshold = 0.1f;
		m_geoNeighborPrecision = GeodesicDistMap::NP_8;
		m_isoPatchResizing = cv::Size(160, 120);	// sizes between 80x60 and 160x120 recommended
		m_isoPatchWidth = 0.2f;
		m_trSearchRadius = 0.3f;
		m_trFtLifespan = 10;
		m_trFtPointTempTimespan = 20;
		m_trFtKfMeasurementNoise = 1e-5;
		m_trFtKfProcessNoise = 1e-6;
	}

	Features::~Features(void)
	{
		delete m_distMap;
		m_distMap = 0;
		delete m_ftTracking;
		m_ftTracking = 0;
	}

	void Features::iInit()
	{
		// create geodesic distance map
		m_distMap = new GeodesicDistMap(*m_inImg3d);
		m_distMap->setMaxZDistThreshold(m_geoMaxZDistThreshold);
		m_distMap->setNeighborPrecision((GeodesicDistMap::NeighborPrecision)m_geoNeighborPrecision);
		
		// create feature tracking
		m_ftTracking = new FeatureTracking(m_features, m_system->getInput()->getProjMat());
		m_ftTracking->setSearchRadius(m_trSearchRadius);
		m_ftTracking->setFeatureLifespan(m_trFtLifespan);
		setTrFtPointTempTimespan(m_trFtPointTempTimespan);
		setTrFtKfMeasurementNoise(m_trFtKfMeasurementNoise);
		setTrFtKfProcessNoise(m_trFtKfProcessNoise);
	}

	bool Features::isInit() const
	{
		return m_inImgDepth && m_inImg3d &&
			m_distMap && m_distMap->isCreated() &&
			Module::isInit();
	}

	const cv::Mat& Features::getImgGeodesic() const
	{
		return m_imgGeodesic;
	}

	const cv::Mat& Features::getImgPredecessors() const
	{
		return m_imgPredecessors;
	}

	const std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >& Features::getFeatures() const
	{
		return m_features;
	}

	const std::vector<FeaturePoint*>& Features::getTrackedFeatures() const
	{
		return m_trackedFeatures;
	}
	
	const FeatureTracking* Features::getFeatureTracking() const
	{
		return m_ftTracking;
	}
	
	void Features::setGeoMaxZDistThreshold(float val)
	{
		m_geoMaxZDistThreshold = val;

		if (m_distMap)
			m_distMap->setMaxZDistThreshold(m_geoMaxZDistThreshold);
	}

	void Features::setGeoNeighborPrecision(int val)
	{
		m_geoNeighborPrecision = val;

		if (m_distMap)
			m_distMap->setNeighborPrecision((GeodesicDistMap::NeighborPrecision)m_geoNeighborPrecision);
	}
	
	void Features::setIsoPatchResizing(const cv::Size& val)
	{
		m_isoPatchResizing = val;
	}

	void Features::setIsoPatchWidth(float val)
	{
		if (val < 0)
			throw Exception("invalid iso-patch width");

		m_isoPatchWidth = val;
	}

	void Features::setTrSearchRadius(float val)
	{
		m_trSearchRadius = val;

		if (m_ftTracking)
			m_ftTracking->setSearchRadius(m_trSearchRadius);
	}

	void Features::setTrFtLifespan(int val)
	{
		m_trFtLifespan = val;

		if (m_ftTracking)
			m_ftTracking->setFeatureLifespan(m_trFtLifespan);
	}
	
	void Features::setTrFtPointTempTimespan(int val)
	{
		m_trFtPointTempTimespan = val;

		FeaturePointAccessor::setMaxTimespan(m_trFtPointTempTimespan);
	}
	
	void Features::setTrFtKfMeasurementNoise(double val)
	{
		m_trFtKfMeasurementNoise = val;

		FeaturePoint::setKfMeasurementNoise(m_trFtKfMeasurementNoise);
	}

	void Features::setTrFtKfProcessNoise(double val)
	{
		m_trFtKfProcessNoise = val;

		FeaturePoint::setKfProcessNoise(m_trFtKfProcessNoise);
	}

	float Features::getGeoMaxZDistThreshold() const
	{
		return m_geoMaxZDistThreshold;
	}

	int Features::getGeoNeighborPrecision() const
	{
		return m_geoNeighborPrecision;
	}

	const cv::Size& Features::getIsoPatchResizing() const
	{
		return m_isoPatchResizing;
	}

	float Features::getIsoPatchWidth() const
	{
		return m_isoPatchWidth;
	}

	float Features::getTrSearchRadius() const
	{
		return m_trSearchRadius;
	}

	int Features::getTrFtLifespan() const
	{
		return m_trFtLifespan;
	}

	int Features::getTrFtPointTempTimespan() const
	{
		return m_trFtPointTempTimespan;
	}
		
	double Features::getTrFtKfMeasurementNoise() const
	{
		return m_trFtKfMeasurementNoise;
	}

	double Features::getTrFtKfProcessNoise() const
	{
		return m_trFtKfProcessNoise;
	}

	void Features::getPoint3d(const cv::Mat& img3d, const cv::Point& imgPoint, cv::Point3d& point3d)
	{
		const float* row = img3d.ptr<float>(imgPoint.y);
		point3d.x = row[imgPoint.x * 3 + 0];
		point3d.y = row[imgPoint.x * 3 + 1];
		point3d.z = row[imgPoint.x * 3 + 2];
	}

	void Features::createMask(const cv::Mat& srcImg, cv::Mat& mask)
	{
		if (mask.empty())
			mask = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);

		int type = 0;
		if (srcImg.type() == CV_32FC3)
			type = 1;
		else if (srcImg.type() == CV_32FC1)
			type = 2;

		if (type == 0)
			throw Exception("invalid image type");

		mask.setTo(0);
		for (int i = 0; i < srcImg.cols; i++) {
			for (int j = 0; j < srcImg.rows; j++) {
				float maskVal = 0;
				
				if (type == 1)
					maskVal = srcImg.ptr<float>(j)[i * 3 + 2];
				else if (type == 2)
					maskVal = srcImg.ptr<float>(j)[i];

				if (maskVal > 0)
					mask.ptr<uchar>(j)[i] = 255;
			}
		}
	}

	void Features::geodesicDistances()
	{
		m_distMap->compute(*m_inImg3d, m_imgGeodesic, m_imgPredecessors);
	}

	void Features::fillRegion(const cv::Mat& img, cv::Mat& dstImg, cv::Point start, uchar newVal, float curVal)
	{
		// NOTE: this function has to do region growing, beginning from start point and filling each
		// pixel in dstImg with newVal, that has not yet been filled and has value curVal in img [12/9/2011 Norman]
		
		if (dstImg.type() != CV_8UC1 || img.type() != CV_32FC1 ||
			dstImg.rows != img.rows || dstImg.cols != img.cols)
			throw Exception("invalid input data");

		std::queue<cv::Point> queue;
		queue.push(start);

		do {
			cv::Point curPoint = queue.front();
			queue.pop();
						
			// push valid neighborhood to queue
			for (int k = -1; k <= 1; k++) {
				for (int l = -1; l <= 1; l++) {
					int iInd = curPoint.x + k;
					int jInd = curPoint.y + l;

					if (iInd < 0 || iInd >= img.cols ||
						jInd < 0 || jInd >= img.rows)
						continue;
					
					float srcVal = img.ptr<float>(jInd)[iInd];
					uchar dstVal = dstImg.ptr<uchar>(jInd)[iInd];

					if (srcVal == curVal && dstVal != newVal) {
						queue.push(cv::Point(iInd, jInd));
						dstImg.ptr<uchar>(jInd)[iInd] = newVal;
					}
				}
			}
		} while (!queue.empty());
	}

	void Features::findRegionMax(cv::Mat& neighborsMap, const cv::Mat& geodesicMap, const cv::Point& start, cv::Point& max)
	{
		if (neighborsMap.type() != CV_8UC1 || geodesicMap.type() != CV_32FC1 ||
			neighborsMap.rows != geodesicMap.rows || neighborsMap.cols != geodesicMap.cols)
			throw Exception("invalid input data");

		std::queue<cv::Point> queue;
		queue.push(start);

		float maxVal = 0;
		do {
			cv::Point curPoint = queue.front();
			queue.pop();

			uchar* curNeighborVal = &neighborsMap.ptr<uchar>(curPoint.y)[curPoint.x];

			if (*curNeighborVal == 1) {
				// remove point from neighbors
				neighborsMap.ptr<uchar>(curPoint.y)[curPoint.x] = 2;
			
				float distVal = geodesicMap.ptr<float>(curPoint.y)[curPoint.x];

				if (distVal > maxVal) {
					// select as current maximum
					maxVal = distVal;
					max = curPoint;
				}

				// push valid neighborhood to queue
				for (int k = -1; k <= 1; k++) {
					for (int l = -1; l <= 1; l++) {
						int iInd = curPoint.x + k;
						int jInd = curPoint.y + l;

						if (iInd < 0 || iInd >= neighborsMap.cols ||
							jInd < 0 || jInd >= neighborsMap.rows)
							continue;
					
						uchar neighborVal = neighborsMap.ptr<uchar>(jInd)[iInd];
						float distVal = geodesicMap.ptr<float>(jInd)[iInd];

						if (neighborVal == 1)
							queue.push(cv::Point(iInd, jInd));
					}
				}
			}
		} while (!queue.empty());
	}

	void Features::isoPatchFeatures()
	{
		// in this case, we don't need to process
		if (cv::mean(m_imgGeodesic) == cv::Scalar(0))
			return;
		
		// adjust size if necessary
		if (m_isoPatchResizing.width > m_imgGeodesic.cols)
			m_isoPatchResizing.width = m_imgGeodesic.cols;
		if (m_isoPatchResizing.height > m_imgGeodesic.rows)
			m_isoPatchResizing.height = m_imgGeodesic.rows;

		//cv::imshow("Geodesic", m_imgGeodesic * 0.8);
				
		// resize and filter geodesic distance map
		cv::Mat geodesic, geoTemp;
		cv::resize(m_imgGeodesic, geoTemp, m_isoPatchResizing, 0, 0, CV_INTER_NN);
		cv::medianBlur(geoTemp, geoTemp, 3);

		// also resize 3d image for feature detection
		cv::Mat resizedImg3d;
		cv::resize(m_img3d, resizedImg3d, m_isoPatchResizing, 0, 0, CV_INTER_NN);
		
		// crop geodesic image with image3d mask to avoid getting wrong 3d coordinates
		cv::Mat mask3d;
		createMask(resizedImg3d, mask3d);
		geoTemp.copyTo(geodesic, mask3d);

		// create iso patch as a discretized version of the distance map
		cv::Mat isoPatches(geodesic.rows, geodesic.cols, geodesic.type());
		isoPatches.setTo(0);

		// NOTE: center of gravity is not regarded, why? [12/9/2011 Norman]

		for (int i = 0; i < geodesic.cols; i++) {
			for (int j = 0; j < geodesic.rows; j++) {
				float geoDist = geodesic.ptr<float>(j)[i];
				float* discrDist = &isoPatches.ptr<float>(j)[i];
				
				if (geoDist > 0) {
					// clamp the values
					int clamped = (int)(geoDist / m_isoPatchWidth);

					// convert back to float and add mean value
					float newDist = (float)(clamped * m_isoPatchWidth) + (m_isoPatchWidth / 2.0f);

					*discrDist = newDist;
				}
			}
		}
		
		//cv::imshow("Geodesic Iso-Patches", isoPatches * 0.8);
		
		// labels: 0: invalid (out of user), 1: inside user, 2: deleted [12/12/2011 Norman]
		cv::Mat endRegions(isoPatches.rows, isoPatches.cols, CV_8UC1);
		endRegions.setTo(0);

		for (int i = 0; i < endRegions.cols; i++) {
			for (int j = 0; j < endRegions.rows; j++) {
				float distVal = isoPatches.ptr<float>(j)[i];
				uchar* nVal = &endRegions.ptr<uchar>(j)[i];
				
				// search in neighborhood for neighbors that have a higher distance
				if (*nVal < 2 && distVal > 0) {
					*nVal = 1;

					bool breakThisLoop = false;
					for (int k = -1; k <= 1 && !breakThisLoop; k++) {
						for (int l = -1; l <= 1 && !breakThisLoop; l++) {
							int iInd = i + k;
							int jInd = j + l;

							if (iInd < 0 || iInd >= isoPatches.cols ||
								jInd < 0 || jInd >= isoPatches.rows ||
								(iInd == i && jInd == j))
								continue;
						
							float nDistVal = isoPatches.ptr<float>(jInd)[iInd];

							// remove this region if there neighbors with a higher distance
							if (nDistVal > distVal) {
								fillRegion(isoPatches, endRegions, cv::Point(i, j), 2, distVal);
								breakThisLoop = true;
							}
						}
					}
				}
			}
		}

		//cv::imshow("End-Regions", endRegions * 50);
		
		const cv::Mat& projMat = m_system->getInput()->getProjMat();

		// find features in regions with label 1
		for (int i = 0; i < endRegions.cols; i++) {
			for (int j = 0; j < endRegions.rows; j++) {
				uchar neighborVal = endRegions.ptr<uchar>(j)[i];

				if (neighborVal == 1) {
					// find local maximum
					cv::Point max(0, 0);
					findRegionMax(endRegions, geodesic, cv::Point(i, j), max);
					
					// add feature point
					cv::Point3d point3d(0, 0, 0);
					getPoint3d(resizedImg3d, max, point3d);

					if (point3d.z > 0) {
						cv::Point newPoint2d(0, 0);
						cv::Point2d newPointTemp(0, 0);
						pointXYZ2UV(projMat, point3d, newPointTemp);
						newPoint2d = cv::Point((int)newPointTemp.x, (int)newPointTemp.y);

						if (newPoint2d.x >= 0 && newPoint2d.x < m_imgGeodesic.cols &&
							newPoint2d.y >= 0 && newPoint2d.y < m_imgGeodesic.rows)
							m_features.push_back(std::make_pair(JT_UNKNOWN, std::make_pair(point3d, newPoint2d)));
					}
					else
						WARN << "wrong feature 3d coordinate, should not happen anymore" << ENDL;
				}
			}
		}
	}

	void Features::relocateFeatures()
	{
		// Since the feature detection uses a downsampled version of the geodesic distance map,
		// the detected features are located not exactly on the edge of the original distance map.
		// This algorithm thus relocates the feature points by tracing back the path to the local
		// maximum near each feature point.

		const cv::Mat& projMat = m_system->getInput()->getProjMat();

		for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::iterator it =
				m_features.begin(); it != m_features.end(); it++) {
			
			// don't relocate torso position
			if (it->first == JT_TORSO)
				continue;
			
			// get image position
			cv::Point3d pos3d = it->second.first;
			cv::Point pos = it->second.second;

			double centerDist = m_imgGeodesic.ptr<float>(pos.y)[pos.x];
			bool stop = false;

			// starting from the initial feature position, look in the neighborhood for any
			// points in the distance map with a higher distance and only stop, when no more
			// valid points are found
			do {
				bool found = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						cv::Point nPos = pos + cv::Point(i, j);

						if (nPos.x < 0 || nPos.x >= m_imgGeodesic.cols ||
							nPos.y < 0 || nPos.y >= m_imgGeodesic.rows ||
							nPos == pos)
							continue;

						double nDist = m_imgGeodesic.ptr<float>(nPos.y)[nPos.x];

						if (nDist > centerDist) {
							pos = nPos;
							centerDist = nDist;
							found = true;
						}
					}
				}

				if (!found)
					stop = true;
			} while (!stop);
			
			// store new location and remove possible duplicates
			bool found = false;
			cv::Point3d newPos3d(0, 0, 0);
			getPoint3d(m_img3d, pos, newPos3d);

			// try to find a possibly already existing feature point at this location
			for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::const_iterator it2 =
					m_features.begin(); it2 != m_features.end(); it2++) {
				if (it2->second.first == newPos3d && it2 != it) {
					found = true;
					break;
				}
			}

			// if not found, update this one's position. if duplicate found, erase it from the list.
			if (!found) {
				it->second.first = newPos3d;
				it->second.second = pos;
			}
			else {
				it = m_features.erase(it);
				it--;
			}
		}
	}
	
	void Features::filterOutNearFeatures()
	{
		const float radius = 0.2f;

		std::vector<bool> toRemove(m_features.size());
		for (int i = 0; i < (int)toRemove.size(); i++)
			toRemove[i] = false;
		
		const cv::Mat& projMat = m_system->getInput()->getProjMat();

		int i = 0;
		for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::const_iterator it1 =
				m_features.begin(); it1 != m_features.end(); it1++, i++) {
			if (toRemove[i] == true)
				continue;

			cv::Point3d pos3d1 = it1->second.first;
			cv::Point pos2d1 = it1->second.second;
			float dist1 = m_imgGeodesic.ptr<float>(pos2d1.y)[pos2d1.x];
			
			if (dist1 == 0) {
				toRemove[i] = true;
				continue;
			}

			int j = 0;
			for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::const_iterator it2 =
					m_features.begin(); it2 != m_features.end(); it2++, j++) {
				if (it1 == it2 || toRemove[j] == true)
					continue;
				
				cv::Point3d pos3d2 = it2->second.first;
				cv::Point pos2d2 = it2->second.second;
				float dist2 = m_imgGeodesic.ptr<float>(pos2d2.y)[pos2d2.x];

				if (dist2 == 0) {
					toRemove[j] = true;
					continue;
				}
				
				double spatialDist = distanceP3d(pos3d1, pos3d2);
				
				// mark this point as to be removed
				if (spatialDist < radius && abs(dist2 - dist1) < radius && dist2 < dist1)
					toRemove[j] = true;
			}
		}

		// remove detected features
		i = 0;
		for (std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >::iterator it =
				m_features.begin(); it != m_features.end(); i++) {
			if (toRemove[i] == true) {
				it = m_features.erase(it);
			}
			else
				it++;
		}
	}

	void Features::torsoFeature()
	{
		m_torsoNormal = cv::Point3d(0, 0, 0);

		// get moments
		cv::Mat mask;
		createMask(m_img3d, mask);
		cv::Moments moments = cv::moments(mask, true);

		// label torso
		cv::Point3d torso(0, 0, 0);
		if (moments.m00 != 0) {
			cv::Point com((int)(moments.m10 / moments.m00), (int)(moments.m01 / moments.m00));
			getPoint3d(m_img3d, com, torso);

			if (torso.z > 0) {
				m_features.push_back(std::make_pair(JT_TORSO, std::make_pair(torso, com)));
				m_torsoNormal = computeTorsoNormal(torso, com);
			}
		}
	}

	cv::Point3d Features::computeTorsoNormal(const cv::Point3d& torso3d, const cv::Point& torso2d)
	{
		std::vector<cv::Point3d> neighbors;
		
		const int maskSize = 1;
		for (int i = -maskSize; i <= maskSize; i++) {
			for (int j = -maskSize; j <= maskSize; j++) {
				cv::Point3d point(0, 0, 0);
				cv::Point newPoint2d = torso2d + cv::Point(i, j);
				
				if ((i == 0 && j == 0) ||
					(newPoint2d.x < 0 || newPoint2d.x >= m_img3d.cols ||
					newPoint2d.y < 0 || newPoint2d.y >= m_img3d.rows))
					continue;

				getPoint3d(m_img3d, newPoint2d, point);

				if (point.z > 0)
					neighbors.push_back(point);
			}
		}
		
		// compute normals for every neighbor point
		std::vector<cv::Point3d> normals;
		for (int i = 0; i < (int)neighbors.size(); i++) {
			int neighborPoint1 = i;
			int neighborPoint2 = (i+1) % neighbors.size();

			cv::Point3d point1 = neighbors[neighborPoint1];
			cv::Point3d point2 = neighbors[neighborPoint2];

			if (point1.z == 0 || point2.z == 0)
				continue;

			cv::Point3d diff1 = point1 - torso3d;
			cv::Point3d diff2 = point2 - torso3d;

			cv::Point3d normal = diff2.cross(diff1);
			
			if (cv::norm(normal) > 0) {

				if (normal.z > 0)
					normal *= -1.0;

				normal *= 1.0f / cv::norm(normal);
				normals.push_back(normal);
			}
		}
		
		// get average normal
		if ((int)normals.size() > 0) {
			cv::Point3d avgNormal(0, 0, 0);
			for (int i = 0; i < (int)normals.size(); i++)
				avgNormal += normals[i];
			avgNormal *= 1.0f / (int)normals.size();
			avgNormal *= 1.0f / cv::norm(avgNormal);

			return avgNormal;
		}

		return cv::Point3d(0, 0, 0);
	}

	void Features::iProcess()
	{
		if (!isInit())
			throw Exception("not init");

		m_features.clear();

		// copy data
		m_inImgDepth->copyTo(m_imgDepth);
		m_inImg3d->copyTo(m_img3d);

		// extract features
		geodesicDistances();
		isoPatchFeatures();
		relocateFeatures();
		filterOutNearFeatures();
		torsoFeature();
		
		/*m_features.clear();
		m_features.push_back(std::make_pair(JT_TORSO, std::make_pair(cv::Point3d(0, 0, 1), cv::Point(0, 0))));
		m_features.push_back(std::make_pair(JT_LEFTFOOT, std::make_pair(cv::Point3d(1, 1, 1), cv::Point(0, 0))));
		m_features.push_back(std::make_pair(JT_RIGHTFOOT, std::make_pair(cv::Point3d(-1, 1, 1), cv::Point(0, 0))));*/

		// track and label features
		m_ftTracking->process();

		m_trackedFeatures = m_ftTracking->getFeaturePoints();
	}
}
