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
#include "../OpenSF/Module.h"
#include "Joint.h"
#include <opencv2/opencv.hpp>

#pragma warning(disable: 4996)
#include <flann/flann.hpp>
#pragma warning(default: 4996)

namespace osf
{
	class Joint;
	class JointEndConnector;
	class Skeleton;
	class SkeletonFitting;
	class FeaturePoint;
	class Fitting;

	typedef bool (*ExtrapolatorFunc)(Fitting*, Joint*, const FeaturePoint*,
		const std::vector<FeaturePoint*>*);

	class Fitting
		: public Module
	{
		MK_TYPE(Fitting);

	public:
		Fitting(System*);
		virtual ~Fitting(void);

		virtual bool isInit() const;

		template <typename T>
		Skeleton* createSkeleton();

		const std::vector<FeaturePoint*>* getFeatures();
		bool isTracking() const;
		const cv::Mat& getProjMat() const;
		const cv::Mat* getImgDepth() const;
		const cv::Mat* getImg3d() const;
		const cv::Mat* getImgGeo() const;
		const cv::Mat* getImgPred() const;
		Joint* getRootJoint();
		Skeleton* getSkeleton();
		void setExtrapolatorFunc(JointType, ExtrapolatorFunc);
		void fixToStandardPose(Joint*) const;
		void releaseFromStandardPose(Joint*);

		void computeDepthKNN(int, const cv::Point3d&, std::vector<cv::Point3d>&, std::vector<float>&);
		bool computeGeodesicKNN(JointType, int, const cv::Point3d&, std::vector<cv::Point3d>&, std::vector<float>&);
				
		// parameters
		void setNNDepthStep(int);
		void setNNDepthMaxLeafSize(int);
		void setNNGeoStep(int);
		void setNNGeoMaxLeafSize(int);
		void setNNGeoCutoffFactor(float);
		void setFitCCDMaxIter(int);
		void setFitCCDChangeThresh(double);
		void setFitCCDMinimzeSize(bool);
		
		int	getNNDepthStep() const;
		int getNNDepthMaxLeafSize() const;
		int getNNGeoStep() const;
		int getNNGeoMaxLeafSize() const;
		float getNNGeoCutoffFactor() const;
		int getFitCCDMaxIter() const;
		double getFitCCDChangeThresh() const;
		bool getFitCCDMinimizeSize() const;

	protected:
		virtual void iInit();
		void iProcess();

	private:
		void fit();
		void runClassification(const std::vector<JointEndConnector*>&);
		bool startTracking(const std::vector<JointEndConnector*>&);
		bool stopTracking(bool&, cv::Point3d&);
		void runExtrapolation(const std::vector<JointEndConnector*>&);
		void buildFlannIndexDepth();
		void buildFlannIndexGeodesic();
		void knnSearch(const flann::Matrix<float>&, flann::Index<flann::L2<float> >&, const flann::SearchParams&,
			int k, const cv::Point3d&, std::vector<cv::Point3d>&, std::vector<float>&);

		const cv::Mat* m_inImgDepth;
		const cv::Mat* m_inImg3d;
		const cv::Mat* m_inImgGeodesic;
		const cv::Mat* m_inImgPredecessors;
		const std::vector<FeaturePoint*>* m_inFeatures;
		
		Skeleton*			m_skeleton;
		SkeletonFitting*	m_skeletonFitting;
		bool				m_processFitting;
		Timer				m_fittingTimer;
		std::map<JointType, ExtrapolatorFunc>	m_extrapolatorFuncs;
		
		// flann
		std::vector<cv::Point3f>		m_flPointCloud;
		flann::Matrix<float>			m_flDataset;
		flann::SearchParams				m_flSearchParams;
		flann::IndexParams				m_flIndexParams;
		flann::Index<flann::L2<float> >*	m_flIndex;
		
		flann::SearchParams										m_flGeoSearchParams;
		flann::IndexParams										m_flGeoIndexParams;
		std::map<JointType, std::vector<cv::Point3f> >			m_flGeoPointClouds;
		std::map<JointType, flann::Matrix<float> >				m_flGeoDatasets;
		std::map<JointType, flann::Index<flann::L2<float> > *>	m_flGeoIndices;

		// parameters
		int m_NNDepthStep;			// step width for flann depth point cloud
		int m_NNDepthMaxLeafSize;	// max leaf size for depth flann
		int m_NNGeoStep;			// step width for flann geodesic lines
		int m_NNGeoMaxLeafSize;		// max leaf size for geodesic flann
		float m_NNGeoCutoffFactor;	// geodesic line will be cut off by this value
		int m_fitCCDMaxIter;		// maximum iterations of CCD algorithm
		double m_fitCCDChangeThresh;	// CCD will be stopped if no more changes above this threshold occur
		bool m_fitCCDMinimizeSize;	// also minimize skeleton size
	};
	
	template <typename T>
	Skeleton* Fitting::createSkeleton()
	{
		if (m_skeleton) {
			WARN << "skeleton already set" << ENDL;
			return 0;
		}

		m_skeleton = new T(this);
		return m_skeleton;
	}
}
