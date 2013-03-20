#pragma once
#include "../OpenSF/Module.h"
#include "../OpenSFitting/Joint.h"
#include "GeodesicDistMap.h"
#include <opencv2/opencv.hpp>

namespace osf
{
	class FeatureTracking;
	class FeatureLabelling;
	class FeaturePoint;

	class Features
		: public Module
	{
		MK_TYPE(Features);

	public:
		Features(System*);
		virtual ~Features(void);

		virtual bool isInit() const;

		const cv::Mat& getImgGeodesic() const;
		const cv::Mat& getImgPredecessors() const;
		const std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > >& getFeatures() const;
		const std::vector<FeaturePoint*>& getTrackedFeatures() const;
		const FeatureTracking* getFeatureTracking() const;

		// parameters
		void setGeoMaxZDistThreshold(float);
		void setGeoNeighborPrecision(int);
		void setIsoPatchResizing(const cv::Size&);
		void setIsoPatchWidth(float);
		void setTrSearchRadius(float);
		void setTrFtLifespan(int);
		void setTrFtPointTempTimespan(int);
		void setTrFtKfMeasurementNoise(double);
		void setTrFtKfProcessNoise(double);

		float getGeoMaxZDistThreshold() const;
		int getGeoNeighborPrecision() const;
		const cv::Size& getIsoPatchResizing() const;
		float getIsoPatchWidth() const;
		float getTrSearchRadius() const;
		int getTrFtLifespan() const;
		int getTrFtPointTempTimespan() const;
		double getTrFtKfMeasurementNoise() const;
		double getTrFtKfProcessNoise() const;

	protected:
		virtual void iInit();
		void iProcess();

	private:
		void getPoint3d(const cv::Mat& img3d, const cv::Point& imgPoint, cv::Point3d& point3d);
		void createMask(const cv::Mat&, cv::Mat&);
		void geodesicDistances();
		void torsoFeature();
		cv::Point3d computeTorsoNormal(const cv::Point3d&, const cv::Point&);

		// geodesic iso patches feature detection
		void fillRegion(const cv::Mat&, cv::Mat&, cv::Point, uchar, float);
		void findRegionMax(cv::Mat&, const cv::Mat&, const cv::Point&, cv::Point&);
		void isoPatchFeatures();
		void relocateFeatures();
		void filterOutNearFeatures();

		const cv::Mat* m_inImgDepth;
		const cv::Mat* m_inImg3d;
		
		cv::Mat m_imgDepth;
		cv::Mat m_img3d;

		// feature detection
		std::vector<std::pair<JointType, std::pair<cv::Point3d, cv::Point> > > m_features;
		std::vector<FeaturePoint*> m_trackedFeatures;
		
		// geodesic distance map
		cv::Mat m_imgGeodesic;
		cv::Mat m_imgPredecessors;
		GeodesicDistMap* m_distMap;

		// feature postprocessing
		FeatureTracking*	m_ftTracking;
		cv::Point3d			m_torsoNormal;

		// parameters
		float		m_geoMaxZDistThreshold;	// maximum distance of neighboring pixels
		int			m_geoNeighborPrecision;	// 4 or 8 neighborhood
		cv::Size	m_isoPatchResizing;		// downsample size for iso-patch generation
		float		m_isoPatchWidth;		// range of an iso-patch
		float		m_trSearchRadius;		// search radius for tracking
		int			m_trFtLifespan;			// maximum extrapolating feature lifespan
		int			m_trFtPointTempTimespan;	// timespan in which temporal features are computed for every feature point
		double		m_trFtKfMeasurementNoise;	// measurement noise for kalman filter
		double		m_trFtKfProcessNoise;		// process noise for kalman filter
	};
}
