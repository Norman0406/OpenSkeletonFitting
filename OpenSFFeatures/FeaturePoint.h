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
#include <list>
#include "../OpenSFitting/Joint.h"

namespace osf
{
	/************************************************************************
	  FeaturePoint: provides reading access to the important features
	*************************************************************************/
	class FeaturePoint
	{
	public:
		enum ConfirmState
		{
			CS_CONFIRMED = 0,	// tracked point was assigned a valid feature point
			CS_EXTRAPOLATED,	// point is currently being extrapolated (by extrapolator function)
			CS_UNCONFIRMED		// point is unconfirmed and position is being maintained
		};

		FeaturePoint(void);
		virtual ~FeaturePoint(void);
		
		const cv::Point3d& getPosition3d() const;
		const cv::Point3d& getRelativePosition() const;
		const cv::Point3d& getVelocity() const;
		const cv::Point3d& getRelativeVelocity() const;
		const cv::Point2d& getPosition2d() const;
		const cv::Point3d& getPosition3dFiltered() const;
		int getTrackingLabel() const;
		void setConfirmState(ConfirmState);
		ConfirmState getConfirmState() const;
		bool isConfirmed() const;
		void setJointLabel(JointType);		// the only writing function that may be called from outside
		JointType getJointLabel() const;
		void trackFrom(const FeaturePoint&);

		static void setKfMeasurementNoise(double);
		static void setKfProcessNoise(double);
		static double getKfMeasurementNoise();
		static double getkfProcessNoise();

		// features
		double getFtSpd() const;
		double getFtRelSpd() const;

		double getFtSummedRelSpd() const;
		double getFtSummedAbsSpd() const;
		double getFtSummedMeanRelSpd() const;
		double getFtSummedMeanAbsSpd() const;
		const cv::Point3d& getFtMeanRelVel() const;
		const cv::Point3d& getFtMeanAbsVel() const;
		double getFtMeanRelSpd() const;
		double getFtMeanAbsSpd() const;
		
		int getLifetime() const;
		int getOverallConfLifetime() const;
		int getOverallUnconfLifetime() const;
		int getLatestConfLifetime() const;
		int getLatestUnconfLifetime() const;
		float getConfUnconfRatio() const;

	protected:
		cv::Point3d m_position3d;
		cv::Point3d m_position3dFiltered;
		cv::Point3d m_relativePosition;
		cv::Point3d m_velocity;
		cv::Point3d m_relativeVelocity;
		cv::Point2d m_position2d;
		int m_trackingLabel;

		ConfirmState m_cfmState;
		JointType m_jointLabel;

		// filtering
		cv::KalmanFilter	m_kalmanFilter;
		cv::Mat				m_kfMeasurements;
		bool				m_kfUseSpeed;
		static double		m_kfMeasurementNoise;
		static double		m_kfProcessNoise;

		// features
		double m_speed;				// current speed in 3d space
		double m_relSpeed;			// current speed in 3d space, with relation to reference point (torso)

		double m_summedRelSpd;		// sum over all previously stored relative speeds
		double m_summedAbsSpd;		// sum over all previously stored absolute speeds
		double m_summedMeanRelSpd;	// mean value of m_summedRelSpd
		double m_summedMeanAbsSpd;	// mean value of m_summedAbsSpd
		cv::Point3d m_meanRelVel;	// mean velocity vector over all previously stored relative velocities
		cv::Point3d m_meanAbsVel;	// mean velocity vector over all previously stored absolute velocities
		double m_meanRelSpd;		// norm of m_meanRelVel (speed)
		double m_meanAbsSpd;		// norm of m_meanAbsVel (speed)
				
		int m_lifetime;					// overall lifetime
		int m_overallConfLifetime;		// overall confirmed lifetime
		int m_overallUnconfLifetime;	// overall unconfirmed lifetime
		int m_latestConfLifetime;		// lifetime of the latest confirmed period
		int m_latestUnconfLifetime;		// lifetime of the latest unconfirmed period
	};

	/************************************************************************
	  FeaturePointAccessor: also provides writing access to the features
	*************************************************************************/
	class FeaturePointAccessor
		: public FeaturePoint
	{
	public:
		FeaturePointAccessor(int label);
		virtual ~FeaturePointAccessor();

		static void setMaxTimespan(int);
		static int getMaxTimespan();
		void setPosition(const cv::Point3d& worldPos, const cv::Point3d& relPos, const cv::Point2d&);
		void setVelocity(const cv::Point3d& vel, const cv::Point3d& relVel);
		std::vector<double>& getDepthFeatures();
		void stepLifetime();
		void computeTemporalFeatures();

	private:
		bool m_firstFrame;

		int m_summedRelVelsCount;
		static int m_maxTimespan;		// max time (frames) used to compute temporal features
		std::list<cv::Point3d> m_relVelocities;
		std::list<cv::Point3d> m_absVelocities;
	};
}
