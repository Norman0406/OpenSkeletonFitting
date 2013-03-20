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
#include "Fitting.h"
#include "FittingFactory.h"
#include "Joint.h"
#include "SkeletonFitting.h"
#include "../OpenSFInput/Input.h"
#include "../OpenSFLib/System.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"

namespace osf
{
	Fitting::Fitting(System* sys)
		: Module(sys), m_inImgDepth(0), m_inImg3d(0), m_inImgGeodesic(0),
		m_inImgPredecessors(0), m_inFeatures(0), m_skeleton(0), m_skeletonFitting(0),
		m_flIndex(0)
	{
		addInput(m_inImgDepth);
		addInput(m_inImg3d);
		addInput(m_inImgGeodesic);
		addInput(m_inImgPredecessors);
		addInput(m_inFeatures);

		m_processFitting = false;
		
		m_NNDepthStep = 3;
		m_NNDepthMaxLeafSize = 15;
		m_NNGeoStep = 1;
		m_NNGeoMaxLeafSize = 15;
		m_NNGeoCutoffFactor = 0.5f;
		m_fitCCDMaxIter = 1;
		m_fitCCDChangeThresh = 0.001;
		m_fitCCDMinimizeSize = true;
	}

	Fitting::~Fitting(void)
	{
		delete m_skeleton;
		delete m_skeletonFitting;
		delete m_flIndex;

		for (std::map<JointType, flann::Index<flann::L2<float> > *>::iterator it = m_flGeoIndices.begin();
				it != m_flGeoIndices.end(); it++)
			delete it->second;
		m_flGeoIndices.clear();
	}

	void Fitting::iInit()
	{
		if (isInit())
			throw Exception("already init");
		
		// init skeleton
		if (!m_skeleton)
			throw Exception("skeleton not set, call createSkeleton() first!");
		m_skeleton->init();

		m_skeletonFitting = new SkeletonFitting(m_skeleton);
		
		// create flann params
		m_flSearchParams = flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED);
		m_flIndexParams = flann::KDTreeSingleIndexParams(m_NNDepthMaxLeafSize);

		m_flGeoSearchParams = flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED);
		m_flGeoIndexParams = flann::KDTreeSingleIndexParams(m_NNGeoMaxLeafSize);
	}

	bool Fitting::isInit() const
	{
		return m_inImgDepth && m_inImg3d &&
			m_inImgGeodesic && m_inFeatures &&
			m_skeleton && m_skeletonFitting &&
			Module::isInit();
	}
	
	const std::vector<FeaturePoint*>* Fitting::getFeatures()
	{
		return m_inFeatures;
	}

	const cv::Mat& Fitting::getProjMat() const
	{
		return m_system->getInput()->getProjMat();
	}

	const cv::Mat* Fitting::getImgDepth() const
	{
		return m_inImgDepth;
	}

	const cv::Mat* Fitting::getImg3d() const
	{
		return m_inImg3d;
	}

	const cv::Mat* Fitting::getImgGeo() const
	{
		return m_inImgGeodesic;
	}

	const cv::Mat* Fitting::getImgPred() const
	{
		return m_inImgPredecessors;
	}

	Skeleton* Fitting::getSkeleton()
	{
		if (!m_skeleton)
			throw Exception("invalid data");

		return m_skeleton;
	}

	Joint* Fitting::getRootJoint()
	{
		return getSkeleton()->getRoot();
	}

	bool Fitting::isTracking() const
	{
		return m_processFitting;
	}

	void Fitting::setNNDepthStep(int val)
	{
		m_NNDepthStep = val;
	}

	void Fitting::setNNDepthMaxLeafSize(int val)
	{
		m_NNDepthMaxLeafSize = val;
	}

	void Fitting::setNNGeoStep(int val)
	{
		m_NNGeoStep = val;
	}

	void Fitting::setNNGeoMaxLeafSize(int val)
	{
		m_NNGeoMaxLeafSize = val;
	}

	void Fitting::setNNGeoCutoffFactor(float val)
	{
		m_NNGeoCutoffFactor = val;
	}

	void Fitting::setFitCCDMaxIter(int val)
	{
		m_fitCCDMaxIter = val;
	}

	void Fitting::setFitCCDChangeThresh(double val)
	{
		m_fitCCDChangeThresh = val;
	}

	void Fitting::setFitCCDMinimzeSize(bool val)
	{
		m_fitCCDMinimizeSize = val;
	}

	int	Fitting::getNNDepthStep() const
	{
		return m_NNDepthStep;
	}

	int Fitting::getNNDepthMaxLeafSize() const
	{
		return m_NNDepthMaxLeafSize;
	}

	int Fitting::getNNGeoStep() const
	{
		return m_NNGeoStep;
	}

	int Fitting::getNNGeoMaxLeafSize() const
	{
		return m_NNGeoMaxLeafSize;
	}

	float Fitting::getNNGeoCutoffFactor() const
	{
		return m_NNGeoCutoffFactor;
	}

	int Fitting::getFitCCDMaxIter() const
	{
		return m_fitCCDMaxIter;
	}

	double Fitting::getFitCCDChangeThresh() const
	{
		return m_fitCCDChangeThresh;
	}

	bool Fitting::getFitCCDMinimizeSize() const
	{
		return m_fitCCDMinimizeSize;
	}

	void Fitting::setExtrapolatorFunc(JointType type, ExtrapolatorFunc func)
	{
		// if called twice for the same joint, the function will be overwritten
		m_extrapolatorFuncs[type] = func;
	}

	void Fitting::fixToStandardPose(Joint* joint) const
	{
		// set joint and all its predecessors to their standard pose, until
		// we reach a "junction", which connects to more than one end affector
		
		joint->setToStandard();

		std::vector<JointEndConnector*> endConnectors;
		joint->getSubEndConnectors(endConnectors);
		if ((int)endConnectors.size() > 1 || !joint->getPrevJoint())
			return;
		
		joint->setFixed(true);
		fixToStandardPose(joint->getPrevJoint());
	}
	
	void Fitting::releaseFromStandardPose(Joint* joint)
	{
		std::vector<JointEndConnector*> endConnectors;
		joint->getSubEndConnectors(endConnectors);
		if ((int)endConnectors.size() > 1 || !joint->getPrevJoint())
			return;
		
		joint->setFixed(false);
		releaseFromStandardPose(joint->getPrevJoint());
	}

	void Fitting::buildFlannIndexDepth()
	{
		//const int step = m_inImg3d->cols / 100;
		//const int step = 1;
		const int step = m_NNDepthStep;
		
		// create point cloud
		int oldSize = m_flPointCloud.size();
		m_flPointCloud.clear();
		m_flPointCloud.reserve(oldSize);
		for (int i = 0; i < m_inImg3d->cols; i+=step) {
			for (int j = 0; j < m_inImg3d->rows; j+=step) {
				cv::Point3f point(0, 0, 0);
				const float* row = m_inImg3d->ptr<float>(j);
				point.x = row[i * 3 + 0];
				point.y = row[i * 3 + 1];
				point.z = row[i * 3 + 2];

				if (point.z != 0 && m_inImgDepth->ptr<float>(j)[i] != 0)
					m_flPointCloud.push_back(point);
			}
		}

		// create flann index
		if (m_flPointCloud.size() > 0) {
			m_flDataset = flann::Matrix<float>(&m_flPointCloud[0].x, m_flPointCloud.size(), 3);
			delete m_flIndex;
			m_flIndex = new flann::Index<flann::L2<float> >(m_flDataset, m_flIndexParams);
			m_flIndex->buildIndex();
		}
	}
	
	void Fitting::buildFlannIndexGeodesic()
	{
		//const int step = 1;					// only add every i-th point
		const int step = m_NNGeoStep;
		const float cutoffFactor = m_NNGeoCutoffFactor;	// the geodesic line will be cut off by this value
		
		// for every feature point, create an entry in each of the maps to initialize flann
		for (int i = 0; i < (int)m_inFeatures->size(); i++) {
			FeaturePoint* ftPoint = m_inFeatures->at(i);
			
			JointType jType = ftPoint->getJointLabel();
			if (jType == JT_UNKNOWN)	// don't process unknown feature points
				continue;

			cv::Point2f point2d = ftPoint->getPosition2d();

			if (point2d.x < 0 || point2d.x >= m_inImgPredecessors->cols ||
				point2d.y < 0 || point2d.y >= m_inImgPredecessors->rows) {
					return;
			}

			// get path to origin and add the 3d points along this path to the flann point cloud
			cv::Point curPt = point2d;
			cv::Point lastPt = curPt;
			float curDist = 0;
			float maxDist = 0;
			bool hasPoints = false;
			
			int temp = 0;
			do {
				if (curPt.x < 0 || curPt.y >= m_inImgPredecessors->cols ||
					curPt.y < 0 || curPt.y >= m_inImgPredecessors->rows)
					break;			

				curDist = m_inImgGeodesic->ptr<float>(curPt.y)[curPt.x];

				// set maximum distance (geodesic distance of feature point)
				if (maxDist == 0)
					maxDist = curDist;
				else {
					// stop if we processed x % of points on the geodesic line
					if (curDist < maxDist - (maxDist * cutoffFactor))
						break;
					
					// skip every i-th point
					if (temp++ % step != 0)
						continue;

					// clear previous point cloud
					if (!hasPoints)
						m_flGeoPointClouds[jType].clear();

					// get 3d point
					cv::Point3f point3d(m_inImg3d->ptr<float>(curPt.y)[curPt.x * 3 + 0],
						m_inImg3d->ptr<float>(curPt.y)[curPt.x * 3 + 1],
						m_inImg3d->ptr<float>(curPt.y)[curPt.x * 3 + 2]);

					// add point to the list
					m_flGeoPointClouds[jType].push_back(point3d);
					hasPoints = true;
				}
			
				// get next point
				int x = m_inImgPredecessors->ptr<int>(curPt.y)[curPt.x * 2 + 0];
				int y = m_inImgPredecessors->ptr<int>(curPt.y)[curPt.x * 2 + 1];

				lastPt = curPt;
				curPt = cv::Point(x, y);
			} while (curPt != lastPt);
			
			// create flann index
			if (hasPoints && m_flGeoPointClouds[jType].size() > 0) {
				// create dataset matrix
				m_flGeoDatasets[jType] = flann::Matrix<float>(&m_flGeoPointClouds[jType][0].x,
					m_flGeoPointClouds[jType].size(), 3);

				// create index
				flann::Index<flann::L2<float> >** index = &m_flGeoIndices[jType];
				delete *index;
				*index = new flann::Index<flann::L2<float> >(m_flGeoDatasets[jType], m_flGeoIndexParams);

				// build index
				(*index)->buildIndex();
			}
		}
	}
	
	void Fitting::computeDepthKNN(int k, const cv::Point3d& searchPoint,
		std::vector<cv::Point3d>& nearestPoints, std::vector<float>& nearestDist)
	{
		knnSearch(m_flDataset, *m_flIndex, m_flSearchParams, k, searchPoint, nearestPoints, nearestDist);
	}

	bool Fitting::computeGeodesicKNN(JointType jointType, int k, const cv::Point3d& searchPoint,
		std::vector<cv::Point3d>& nearestPoints, std::vector<float>& nearestDist)
	{
		std::vector<cv::Point3f> points;
		
		// find feature point
		FeaturePoint* ftPoint = 0;
		for (int i = 0; i < (int)m_inFeatures->size(); i++) {
			if (m_inFeatures->at(i)->getJointLabel() == jointType)
				ftPoint = m_inFeatures->at(i);
		}

		if (!ftPoint || !ftPoint->isConfirmed())
			return false;

		JointType jType = ftPoint->getJointLabel();
		
		std::vector<cv::Point3f>& pointCloud = m_flGeoPointClouds[jType];
		flann::Matrix<float>& dataset = m_flGeoDatasets[jType];
		flann::Index<flann::L2<float> >* index = m_flGeoIndices[jType];
		
		knnSearch(dataset, *index, m_flGeoSearchParams, k, searchPoint, nearestPoints, nearestDist);

		return true;
	}

	void Fitting::knnSearch(const flann::Matrix<float>& pointCloud, flann::Index<flann::L2<float> >& index,
		const flann::SearchParams& searchParams, int k, const cv::Point3d& searchPoint,
		std::vector<cv::Point3d>& nearestPoints, std::vector<float>& nearestDist)
	{
		if (pointCloud.rows == 0)
			return;

		// adjust k if necessary
		if ((int)pointCloud.rows < k)
			k = pointCloud.rows;
		
		std::vector<int> pointsIdx(k);
		nearestDist.clear();

		if ((int)pointsIdx.size() < k)
			pointsIdx.resize(k);
		if ((int)nearestDist.size() < k) 
			nearestDist.resize(k);

		std::vector<float> queryPoint(3);
		queryPoint[0] = (float)searchPoint.x;
		queryPoint[1] = (float)searchPoint.y;
		queryPoint[2] = (float)searchPoint.z;
		flann::Matrix<int> indices(&pointsIdx[0], 1, k);
		flann::Matrix<float> dists(&nearestDist[0], 1, k);

		// perform knn search
		index.knnSearch(flann::Matrix<float>(&queryPoint[0], 1, 3), indices, dists, k, searchParams);
		
		nearestPoints.clear();
		for (int i = 0; i < (int)pointsIdx.size(); i++) {
			int idx = pointsIdx[i];
			cv::Point3d point(pointCloud[idx][0], pointCloud[idx][1], pointCloud[idx][2]);
			nearestPoints.push_back(point);
		}
	}

	void Fitting::runExtrapolation(const std::vector<JointEndConnector*>& endAffectors)
	{
		// find torso feature
		FeaturePoint* torsoFeat = 0;
		for (int i = 0; i < (int)m_inFeatures->size(); i++) {
			if ((*m_inFeatures)[i]->getJointLabel() == JT_TORSO) {
				torsoFeat = (*m_inFeatures)[i];
				break;
			}
		}

		if (torsoFeat) {
			// call extrapolator functions for every end affector
			for (int i = 0; i < (int)endAffectors.size(); i++) {
				JointEndConnector* joint = (JointEndConnector*)endAffectors[i];

				if (m_extrapolatorFuncs.find(joint->getType()) != m_extrapolatorFuncs.end()) {
					if (m_extrapolatorFuncs[joint->getType()](this, joint, torsoFeat, m_inFeatures))
						joint->energyChanged();
				}
			}
		}
	}
	
	void Fitting::runClassification(const std::vector<JointEndConnector*>& endAffectors)
	{
		// call classificator object for every end-affector
		for (int i = 0; i < (int)endAffectors.size(); i++) {
			ClassificatorFunction classFunc = endAffectors[i]->getClassFunc();

			// call classificator with every tracked feature point
			if (classFunc) {
				for (int j = 0; j < (int)m_inFeatures->size(); j++) {
					JointType jType = m_inFeatures->at(j)->getJointLabel();

					if (jType == JT_UNKNOWN) {
						// let joint end affectors classify the features and assign them
						if (classFunc(m_inFeatures->at(j), endAffectors[i], *m_inFeatures, m_skeleton->getRoot())) {
							JointType newLabel = endAffectors[i]->getType();

							// only assign, if there isn't a feature point with this label yet
							bool assign = true;
							for (int k = 0; k < (int)m_inFeatures->size(); k++) {
								if (m_inFeatures->at(k)->getJointLabel() == newLabel) {
									assign = false;
									break;
								}
							}
							
							if (assign) {
								m_inFeatures->at(j)->setJointLabel(endAffectors[i]->getType());
								endAffectors[i]->setAssigned(true);
								releaseFromStandardPose(endAffectors[i]);
							}
						}

						// NOTE: This approach does not concern about losing track of a previously classified joint. Once
						// a joint classified a tracked feature point, it is maintained until the tracking is lost. [2/6/2012 Norman]
					}
					else {
						// feature was already classified, keep tracking
					}
				}
			}
		}
	}

	bool Fitting::startTracking(const std::vector<JointEndConnector*>& endAffectors)
	{
		// start fitting if every end affector has an assigned feature point
		bool start = true;
		for (int i = 0; i < (int)endAffectors.size(); i++) {
			JointType jType = endAffectors[i]->getType();

			// if end affector does not have a classificator function, skip this one
			if (!endAffectors[i]->getClassFunc() || !endAffectors[i]->getEnergyFunction())
				continue;

			bool found = false;
			
			// try to find a mached feature point for the current end affector
			for (int j = 0; j < (int)m_inFeatures->size(); j++) {
				JointType trType = m_inFeatures->at(j)->getJointLabel();
				if (trType == jType && m_inFeatures->at(j)->isConfirmed()) {
					found = true;
					break;
				}
			}

			if (!found) {
				start = false;
				break;
			}
		}

		if (start)
			return true;

		return false;
	}

	bool Fitting::stopTracking(bool& hasTorso, cv::Point3d& torsoPos)
	{
		// stop tracking, if feature point for torso is lost
		hasTorso = false;
		torsoPos = cv::Point3d(0, 0, 0);
		for (int j = 0; j < (int)m_inFeatures->size(); j++) {
			JointType trType = m_inFeatures->at(j)->getJointLabel();
			if (trType == JT_TORSO) {
				hasTorso = true;
				torsoPos = m_inFeatures->at(j)->getPosition3d();
				break;
			}
		}

		if (!hasTorso)
			return true;

		return false;
	}

	void Fitting::fit()
	{
		m_skeleton->getRoot()->allEnergiesChanged();

		m_fittingTimer.reset();
		m_fittingTimer.start();

		// set projection matrix for joints
		const cv::Mat& projMat = m_system->getInput()->getProjMat();
		if (Joint::getProjMat().empty() && !projMat.empty())
			Joint::setProjMat(projMat);
		
		// build flann index for depth nearest neighbor search
		buildFlannIndexDepth();

		// get list of end affectors
		std::vector<JointEndConnector*> endAffectors;
		m_skeleton->getRoot()->getSubEndConnectors(endAffectors);
		
		// run extrapolator functions before tracking
		if (m_processFitting)
			runExtrapolation(endAffectors);

		// run classificator functions for end affectors
		runClassification(endAffectors);
		
		// build flann indices for geodesic nearest neighbor search
		buildFlannIndexGeodesic();
		
		// detect start of skeleton tracking
		if (startTracking(endAffectors) && !m_processFitting)
			m_processFitting = true;

		// detect stopping condition for skeleton tracking
		bool hasTorso = false;
		cv::Point3d torsoPos(0, 0, 0);
		if (stopTracking(hasTorso, torsoPos) && m_processFitting)
			m_processFitting = false;
		
		// adjust torso position
		if (!m_processFitting && hasTorso)
			m_skeleton->getRoot()->setPos3d(torsoPos);
		else if (!m_processFitting)
			m_skeleton->getRoot()->setPos3d(cv::Point3d(0, 0, 0));
		
		// update forward kinematics
		m_skeleton->updateFK();
		
		if (m_processFitting) {
			// run energy minimization
			m_skeletonFitting->setTechnique(SkeletonFitting::MT_SIMPLE_CCD);
			//m_skeletonFitting->setTechnique(SkeletonFitting::MT_CMINPACK_CCD);

			m_skeletonFitting->update(m_fitCCDMaxIter, m_fitCCDChangeThresh, m_fitCCDMinimizeSize);
		}
		else {
			// relax skeleton into standard pose
			m_skeleton->getRoot()->setToStandard();
		}

		m_fittingTimer.stop();
		float diff = m_fittingTimer.getDiffMS();
	}
	
	void Fitting::iProcess()
	{
		if (!isInit())
			throw Exception("not init");
		
		fit();
	}
}
