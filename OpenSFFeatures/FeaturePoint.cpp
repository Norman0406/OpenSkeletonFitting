#include "precompiled.h"
#include "FeaturePoint.h"
#include "../OpenSFitting/Joint.h"

namespace osf
{
	/************************************************************************
	  FeaturePoint
	*************************************************************************/
	double FeaturePoint::m_kfMeasurementNoise = 1e-5;
	double FeaturePoint::m_kfProcessNoise = 1e-6;

	FeaturePoint::FeaturePoint(void)
	{
		m_position3d = cv::Point3d(0, 0, 0);
		m_position3dFiltered = cv::Point3d(0, 0, 0);
		m_position2d = cv::Point2d(0, 0);
		m_relativePosition = cv::Point3d(0, 0, 0);
		m_velocity = cv::Point3d(0, 0, 0);

		m_speed = 0;
		m_relSpeed = 0;
				
		m_summedRelSpd = 0;
		m_summedAbsSpd = 0;
		m_summedMeanRelSpd = 0;
		m_summedMeanAbsSpd = 0;
		m_meanRelVel = cv::Point3d(0, 0, 0);
		m_meanAbsVel = cv::Point3d(0, 0, 0);
		m_meanRelSpd = 0;
		m_meanAbsSpd = 0;

		m_lifetime = 0;
		m_overallConfLifetime = 0;
		m_overallUnconfLifetime = 0;
		m_latestConfLifetime = 0;
		m_latestUnconfLifetime = 0;

		m_jointLabel = JT_UNKNOWN;
		m_cfmState = CS_CONFIRMED;

		// kalman filter parameters
		m_kfUseSpeed = true;
		
		// init kalman filter
		if (m_kfUseSpeed) {
			m_kalmanFilter.init(6, 3, 0);
			m_kalmanFilter.transitionMatrix = *(cv::Mat_<float>(6, 6) <<
				1, 0, 0, 1, 0, 0,	0, 1, 0, 0, 1, 0,	0, 0, 1, 0, 0, 1,
				0, 0, 0, 1, 0, 0,	0, 0, 0, 0, 1, 0,	0, 0, 0, 0, 0, 1);
		}
		else {
			m_kalmanFilter.init(3, 3, 0);
			m_kalmanFilter.transitionMatrix = *(cv::Mat_<float>(3, 3) <<
				1, 0, 0,	0, 1, 0,	0, 0, 1);
		}

		m_kfMeasurements = cv::Mat(3, 1, CV_32F);
		m_kfMeasurements.setTo(0);
		
		setIdentity(m_kalmanFilter.measurementMatrix);
		setIdentity(m_kalmanFilter.processNoiseCov, cv::Scalar::all(m_kfProcessNoise));
		setIdentity(m_kalmanFilter.measurementNoiseCov, cv::Scalar::all(m_kfMeasurementNoise));
		setIdentity(m_kalmanFilter.errorCovPost, cv::Scalar::all(1));
		randn(m_kalmanFilter.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
	}

	FeaturePoint::~FeaturePoint(void)
	{
	}
	
	const cv::Point3d& FeaturePoint::getPosition3d() const
	{
		return m_position3d;
	}
	
	const cv::Point3d& FeaturePoint::getRelativePosition() const
	{
		return m_relativePosition;
	}

	const cv::Point3d& FeaturePoint::getVelocity() const
	{
		return m_velocity;
	}

	const cv::Point3d& FeaturePoint::getRelativeVelocity() const
	{
		return m_relativeVelocity;
	}

	const cv::Point2d& FeaturePoint::getPosition2d() const
	{
		return m_position2d;
	}

	const cv::Point3d& FeaturePoint::getPosition3dFiltered() const
	{
		return m_position3dFiltered;
	}

	int FeaturePoint::getTrackingLabel() const
	{
		return m_trackingLabel;
	}

	void FeaturePoint::setConfirmState(ConfirmState cfmState)
	{
		m_cfmState = cfmState;

		if (m_cfmState != CS_UNCONFIRMED)
			m_latestUnconfLifetime = 0;
		else
			m_latestConfLifetime = 0;
	}

	FeaturePoint::ConfirmState FeaturePoint::getConfirmState() const
	{
		return m_cfmState;
	}
	
	bool FeaturePoint::isConfirmed() const
	{
		return m_cfmState == CS_CONFIRMED;
	}

	void FeaturePoint::setJointLabel(JointType label)
	{
		m_jointLabel = label;
	}

	JointType FeaturePoint::getJointLabel() const
	{
		return m_jointLabel;
	}
	
	void FeaturePoint::trackFrom(const FeaturePoint& ftPoint)
	{
		((FeaturePointAccessor*)this)->setPosition(ftPoint.getPosition3d(),
			ftPoint.getRelativePosition(), ftPoint.getPosition2d());

		m_trackingLabel = ftPoint.getTrackingLabel();
		m_jointLabel = ftPoint.getJointLabel();
		m_lifetime = ftPoint.getLifetime();
		m_overallConfLifetime = ftPoint.getOverallConfLifetime();
		m_overallUnconfLifetime = ftPoint.getOverallUnconfLifetime();
		m_latestConfLifetime = ftPoint.getLatestConfLifetime();
		m_latestUnconfLifetime = ftPoint.getLatestUnconfLifetime();
	}

	void FeaturePoint::setKfMeasurementNoise(double val)
	{
		m_kfMeasurementNoise = val;
	}

	void FeaturePoint::setKfProcessNoise(double val)
	{
		m_kfProcessNoise = val;
	}

	double FeaturePoint::getKfMeasurementNoise()
	{
		return m_kfMeasurementNoise;
	}

	double FeaturePoint::getkfProcessNoise()
	{
		return m_kfProcessNoise;
	}
	
	double FeaturePoint::getFtSpd() const
	{
		return m_speed;
	}

	double FeaturePoint::getFtRelSpd() const
	{
		return m_relSpeed;
	}
	
	double FeaturePoint::getFtSummedRelSpd() const
	{
		return m_summedRelSpd;
	}

	double FeaturePoint::getFtSummedAbsSpd() const
	{
		return m_summedAbsSpd;
	}

	double FeaturePoint::getFtSummedMeanRelSpd() const
	{
		return m_summedMeanRelSpd;
	}

	double FeaturePoint::getFtSummedMeanAbsSpd() const
	{
		return m_summedMeanAbsSpd;
	}

	const cv::Point3d& FeaturePoint::getFtMeanRelVel() const
	{
		return m_meanRelVel;
	}

	const cv::Point3d& FeaturePoint::getFtMeanAbsVel() const
	{
		return m_meanAbsVel;
	}

	double FeaturePoint::getFtMeanRelSpd() const
	{
		return m_meanRelSpd;
	}

	double FeaturePoint::getFtMeanAbsSpd() const
	{
		return m_meanRelSpd;
	}

	int FeaturePoint::getLifetime() const
	{
		return m_lifetime;
	}
	
	int FeaturePoint::getOverallConfLifetime() const
	{
		return m_overallConfLifetime;
	}

	int FeaturePoint::getOverallUnconfLifetime() const
	{
		return m_overallUnconfLifetime;
	}

	int FeaturePoint::getLatestConfLifetime() const
	{
		return m_latestConfLifetime;
	}

	int FeaturePoint::getLatestUnconfLifetime() const
	{
		return m_latestUnconfLifetime;
	}

	float FeaturePoint::getConfUnconfRatio() const
	{
		return (float)m_overallConfLifetime / (float)m_overallUnconfLifetime;
	}

	/************************************************************************
	  FeaturePointAccessor
	*************************************************************************/
	int FeaturePointAccessor::m_maxTimespan = 20;

	FeaturePointAccessor::FeaturePointAccessor(int label)
		: m_firstFrame(true)
	{
		m_trackingLabel = label;
		setMaxTimespan(20);

		m_summedRelVelsCount = 0;
	}

	FeaturePointAccessor::~FeaturePointAccessor()
	{
	}

	void FeaturePointAccessor::setMaxTimespan(int timespan)
	{
		m_maxTimespan = timespan;
	}

	int FeaturePointAccessor::getMaxTimespan()
	{
		return m_maxTimespan;
	}

	void FeaturePointAccessor::setPosition(const cv::Point3d& position3d, const cv::Point3d& relativePosition, const cv::Point2d& position2d)
	{
		// fill velocity vectors with zeros if it is too small
		if ((int)m_relVelocities.size() > m_maxTimespan)
			m_relVelocities.resize(m_maxTimespan);
		else {
			while ((int)m_relVelocities.size() < m_maxTimespan)
				m_relVelocities.push_back(cv::Point3d(0, 0, 0));
		}

		if ((int)m_absVelocities.size() > m_maxTimespan)
			m_absVelocities.resize(m_maxTimespan);
		else {
			while ((int)m_absVelocities.size() < m_maxTimespan)
				m_absVelocities.push_back(cv::Point3d(0, 0, 0));
		}

		// init kalman filter pre-state
		if (m_firstFrame) {
			m_kalmanFilter.statePre.at<float>(0) = (float)relativePosition.x;
			m_kalmanFilter.statePre.at<float>(1) = (float)relativePosition.y;
			m_kalmanFilter.statePre.at<float>(2) = (float)relativePosition.z;

			if (m_kfUseSpeed) {
				m_kalmanFilter.statePre.at<float>(3) = 0;
				m_kalmanFilter.statePre.at<float>(4) = 0;
				m_kalmanFilter.statePre.at<float>(5) = 0;
			}
		}
		
		// set position
		m_position3d = position3d;
		m_position2d = position2d;
		
		if (m_firstFrame)
			m_firstFrame = false;
		
		// predict kalman filter
		cv::Mat prediction = m_kalmanFilter.predict();
		
		// create measurements
		m_kfMeasurements.at<float>(0) = (float)relativePosition.x;
		m_kfMeasurements.at<float>(1) = (float)relativePosition.y;
		m_kfMeasurements.at<float>(2) = (float)relativePosition.z;

		// correct state
		cv::Mat estimation = m_kalmanFilter.correct(m_kfMeasurements);

		cv::Point3d pos3dFilteredRel = cv::Point3d(estimation.at<float>(0),
			estimation.at<float>(1), estimation.at<float>(2));

		cv::Point3d pos3dFilteredAbs = pos3dFilteredRel +
			(position3d - relativePosition);
		
		// compute velocities
		if (!m_firstFrame) {
			m_velocity = pos3dFilteredAbs - m_position3dFiltered;
			m_relativeVelocity = pos3dFilteredRel - m_relativePosition;
			
			m_speed = cv::norm(m_velocity);
			m_relSpeed = cv::norm(m_relativeVelocity);
			
			m_absVelocities.push_back(m_velocity);
			while ((int)m_absVelocities.size() > m_maxTimespan)
				m_absVelocities.pop_front();
			
			m_relVelocities.push_back(m_relativeVelocity);
			while ((int)m_relVelocities.size() > m_maxTimespan)
				m_relVelocities.pop_front();
		}

		m_position3dFiltered = pos3dFilteredAbs;
		m_relativePosition = pos3dFilteredRel;
	}
	
	void FeaturePointAccessor::setVelocity(const cv::Point3d& velocity, const cv::Point3d& relativeVelocity)
	{
		m_velocity = velocity;
		m_relativeVelocity = relativeVelocity;
	}

	void FeaturePointAccessor::stepLifetime()
	{
		m_lifetime++;

		if (m_cfmState == CS_UNCONFIRMED) {
			m_latestUnconfLifetime++;
			m_overallUnconfLifetime++;
		}
		else {
			m_latestConfLifetime++;
			m_overallConfLifetime++;
		}
	}

	void FeaturePointAccessor::computeTemporalFeatures()
	{
		// create relative temporal features
		m_summedRelSpd = 0;
		m_meanRelVel = cv::Point3d(0, 0, 0);

		for (std::list<cv::Point3d>::const_iterator it = m_relVelocities.begin(); it != m_relVelocities.end(); it++) {
			m_summedRelSpd += cv::norm(*it);
			m_meanRelVel += *it;
		}

		m_summedMeanRelSpd = m_summedRelSpd / m_relVelocities.size();
		m_meanRelVel *= 1.0f / m_relVelocities.size();
		m_meanRelSpd = cv::norm(m_meanRelVel);
		
		// create absolute temporal features
		m_summedAbsSpd = 0;
		m_meanAbsVel = cv::Point3d(0, 0, 0);

		for (std::list<cv::Point3d>::const_iterator it = m_absVelocities.begin(); it != m_absVelocities.end(); it++) {
			m_summedAbsSpd += cv::norm(*it);
			m_meanAbsVel += *it;
		}

		m_summedMeanAbsSpd = m_summedAbsSpd / m_absVelocities.size();
		m_meanAbsVel *= 1.0f / m_absVelocities.size();
		m_meanAbsSpd = cv::norm(m_meanAbsVel);		
	}
}
