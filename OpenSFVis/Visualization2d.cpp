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

#include <iostream>
#include "Visualization2d.h"
#include "../OpenSFInput/Input.h"
#include "../OpenSFSegmentation/Segmentation.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSF/Exception.h"
#include "../OpenSF/Utils.h"

namespace osf
{
	Visualization2d::Visualization2d(Input* input, Segmentation* seg, Features* feat, Fitting* fit)
		: m_input(input), m_segmentation(seg), m_features(feat), m_fitting(fit)
	{
		m_imgDepth = 0;
		m_img3d = 0;
		m_imgColor = 0;
		m_projMat = 0;
		m_imgSegDepth = 0;
		m_imgSeg3d = 0;
		m_imgGeo = 0;
		m_imgPred = 0;
		m_vecFeatures = 0;
		m_rootJoint = 0;
		m_isInit = false;

		if (!input)
			throw Exception("input invalid");
	}

	Visualization2d::~Visualization2d(void)
	{
		cv::destroyWindow("DepthMap");
		cv::destroyWindow("Segmentation");
		cv::destroyWindow("Geodesic Distances");
		cv::destroyWindow("Skeleton 2D");
	}
	
	void Visualization2d::recordDepthMap(std::string filename, int fourcc, int framerate)
	{
		m_writerDepthMap.filename = filename;
		m_writerDepthMap.fourcc = fourcc;
		m_writerDepthMap.framerate = framerate;
		m_writerDepthMap.write = true;
	}

	void Visualization2d::recordSegmentation(std::string filename, int fourcc, int framerate)
	{
		m_writerSegmentation.filename = filename;
		m_writerSegmentation.fourcc = fourcc;
		m_writerSegmentation.framerate = framerate;
		m_writerSegmentation.write = true;
	}

	void Visualization2d::recordGeodesicMap(std::string filename, int fourcc, int framerate)
	{
		m_writerGeodesic.filename = filename;
		m_writerGeodesic.fourcc = fourcc;
		m_writerGeodesic.framerate = framerate;
		m_writerGeodesic.write = true;
	}

	void Visualization2d::recordSkeleton(std::string filename, int fourcc, int framerate)
	{
		m_writerSkeleton.filename = filename;
		m_writerSkeleton.fourcc = fourcc;
		m_writerSkeleton.framerate = framerate;
		m_writerSkeleton.write = true;
	}

	void Visualization2d::init()
	{
		if (m_isInit) {
			WARN << "already init" << ENDL;
			return;
		}

		// init input vis
		m_imgDepth = &m_input->getImgDepth();
		m_img3d = &m_input->getImg3d();
		m_imgColor = &m_input->getImgColor();
		m_projMat = &m_input->getProjMat();
		
		cv::namedWindow("DepthMap");
		
		// create video writer
		if (m_writerDepthMap.write) {
			m_writerDepthMap.writer = cv::VideoWriter(m_writerDepthMap.filename, m_writerDepthMap.fourcc,
				m_writerGeodesic.framerate, cv::Size(m_imgDepth->cols, m_imgDepth->rows), true);
		}

		// init segmentation vis
		if (m_segmentation) {
			cv::namedWindow("Segmentation");

			m_imgSegDepth = &m_segmentation->getImgSegDepth();
			m_imgSeg3d = &m_segmentation->getImgSeg3d();
			
			// create video writer
			if (m_writerSegmentation.write) {
				m_writerSegmentation.writer = cv::VideoWriter(m_writerSegmentation.filename, m_writerSegmentation.fourcc,
					m_writerGeodesic.framerate, cv::Size(m_imgSegDepth->cols, m_imgSegDepth->rows), true);
			}
		}

		// init feature vis
		if (m_features) {
			cv::namedWindow("Geodesic Distances");

			m_imgGeo = &m_features->getImgGeodesic();
			m_imgPred = &m_features->getImgPredecessors();
			m_vecFeatures = &m_features->getTrackedFeatures();

			m_featVis = cv::Mat(m_imgDepth->rows, m_imgDepth->cols, CV_8UC3);
			
			// create video writer
			if (m_writerGeodesic.write) {
				m_writerGeodesic.writer = cv::VideoWriter(m_writerGeodesic.filename, m_writerGeodesic.fourcc,
					m_writerGeodesic.framerate, cv::Size(m_imgGeo->cols, m_imgGeo->rows), true);
			}
		}

		// init fitting vis
		if (m_fitting) {
			cv::namedWindow("Skeleton 2D");

			m_skeletonImg = cv::Mat(m_imgDepth->rows, m_imgDepth->cols, CV_8UC3);

			m_rootJoint = m_fitting->getRootJoint();
			if (!m_rootJoint)
				throw Exception("invalid root joint");
			
			// create video writer
			if (m_writerSkeleton.write) {
				m_writerSkeleton.writer = cv::VideoWriter(m_writerSkeleton.filename, m_writerSkeleton.fourcc,
					m_writerGeodesic.framerate, cv::Size(m_skeletonImg.cols, m_skeletonImg.rows), true);
			}
		}

		m_isInit = true;
	}

	bool Visualization2d::draw(bool& paused, bool& step)
	{
		if (!m_isInit) {
			WARN << "not init" << ENDL;
			return false;
		}

		int key = cv::waitKey(5);

		// process keyboard
		if (key == 32)	// space
			paused = !paused;

		if (key == 13)	// enter
			step = true;
		
		if (key == 27)	// escape
			return true;

		// draw depth map
		if (m_input) {
			if (!m_imgDepth->empty()) {
				cv::imshow("DepthMap", (*m_imgDepth) * 0.2);
				
				if (m_writerDepthMap.writer.isOpened())
					m_writerDepthMap.writer << (*m_imgDepth) * 0.2;
			}
		}

		// draw segmentation result
		if (m_segmentation) {
			if (!m_imgSegDepth->empty()) {
				cv::imshow("Segmentation", (*m_imgSegDepth) * 0.2);
				
				if (m_writerSegmentation.writer.isOpened())
					m_writerSegmentation.writer << (*m_imgSegDepth) * 0.2;
			}			
		}

		// draw geodesic distance map with features
		if (m_features) {
			m_featVis.setTo(0);

			if (!m_imgGeo->empty()) {
				cv::Mat temp;
				m_imgGeo->convertTo(temp, CV_8U, (int)(255 * 0.8f));
				cv::cvtColor(temp, m_featVis, CV_GRAY2BGR, 3);
			}

			// draw feature point geodesic paths
			for (int i = 0; i < (int)m_vecFeatures->size(); i++) {
				FeaturePoint* ftPoint = m_vecFeatures->at(i);
				if (!ftPoint->isConfirmed())
					continue;

				cv::Point2f point2d = ftPoint->getPosition2d();

				if (point2d.x < 0 || point2d.x >= m_imgPred->cols ||
					point2d.y < 0 || point2d.y >= m_imgPred->rows)
					continue;

				// draw path to origin
				cv::Point curPt = point2d;
				cv::Point lastPt = curPt;
				float curDist = 0;
				float lastDist = 0;
				do {
					if (curPt.x < 0 || curPt.y >= m_featVis.cols ||
						curPt.y < 0 || curPt.y >= m_featVis.rows)
						break;

					// draw point
					m_featVis.ptr<uchar>(curPt.y)[curPt.x * 3 + 0] = 0;
					m_featVis.ptr<uchar>(curPt.y)[curPt.x * 3 + 1] = 0;
					m_featVis.ptr<uchar>(curPt.y)[curPt.x * 3 + 2] = 255;

					// get next point
					int x = m_imgPred->ptr<int>(curPt.y)[curPt.x * 2 + 0];
					int y = m_imgPred->ptr<int>(curPt.y)[curPt.x * 2 + 1];

					curDist = m_imgGeo->ptr<float>(curPt.y)[curPt.x];

					lastPt = curPt;
					lastDist = curDist;
					curPt = cv::Point(x, y);
				} while (curPt != lastPt);
			}
			
			// draw feature point positions
			for (int i = 0; i < (int)m_vecFeatures->size(); i++) {
				FeaturePoint* ftPoint = m_vecFeatures->at(i);
				
				cv::Point2f point2d = ftPoint->getPosition2d();

				cv::Scalar color(255, 0, 0);

				// give prelabeled features a different color
				if (ftPoint->getJointLabel() != JT_UNKNOWN)
					color = cv::Scalar(0, 255, 255);
				
				if (!ftPoint->isConfirmed()) {
					//color = cv::Scalar(0, 255, 0);
					continue;
				}

				// draw feature point
				cv::line(m_featVis, point2d + cv::Point2f(-2, 0), point2d + cv::Point2f(2, 0), color);
				cv::line(m_featVis, point2d + cv::Point2f(0, -2), point2d + cv::Point2f(0, 2), color);
			}

			cv::imshow("Geodesic Distances", m_featVis);
			
			if (m_writerGeodesic.writer.isOpened())
				m_writerGeodesic.writer << m_featVis;
		}

		// draw skeleton
		if (m_fitting) {
			drawSkeleton();

			if (m_writerSkeleton.writer.isOpened())
				m_writerSkeleton.writer << m_skeletonImg;
		}

		return false;
	}
	
	void Visualization2d::drawSkeleton()
	{
		m_skeletonImg.setTo(0);
		
		if (m_imgSegDepth && !m_imgSegDepth->empty()) {
			m_imgSegDepth->convertTo(m_skeletonImg, CV_8U, 51);
			cv::cvtColor(m_skeletonImg, m_skeletonImg, CV_GRAY2BGR);
		}

		// draw feature points
		if (m_vecFeatures) {
			for (int i = 0; i < (int)m_vecFeatures->size(); i++) {
				FeaturePoint* ftPoint = m_vecFeatures->at(i);
				if (ftPoint->getConfirmState() == FeaturePoint::CS_UNCONFIRMED)
					continue;

				cv::Point3d ptXYZ = ftPoint->getPosition3d();
				cv::Point2f ptUV = ftPoint->getPosition2d();

				cv::Scalar color(255, 0, 0);

				// give prelabeled features a different color
				if (ftPoint->getJointLabel() != JT_UNKNOWN)
					color = cv::Scalar(0, 255, 255);

				cv::line(m_skeletonImg, ptUV + cv::Point2f(-2, 0), ptUV + cv::Point2f(2, 0), color);
				cv::line(m_skeletonImg, ptUV + cv::Point2f(0, -2), ptUV + cv::Point2f(0, 2), color);
			}
		}

		// draw bones
		if (m_rootJoint && m_fitting->isTracking())
			drawBone(m_rootJoint);
	
		cv::imshow("Skeleton 2D", m_skeletonImg);
	}

	void Visualization2d::drawBone(const Joint* joint)
	{
		// draw sub joints
		const std::vector<Joint*> subJoints = joint->getSubJoints();
		for (int i = 0; i < (int)subJoints.size(); i++)
			drawBone(subJoints[i]);

		cv::Point2f srcPos2d = joint->getPos2d();
		cv::Point3d srcPos3d = joint->getPos3d();
		double jointDist = cv::norm(srcPos3d);
		bool drawIt = jointDist > 0.1 && srcPos3d.z > 0.1;
		
		// draw bone
		if (joint->getClass() != JC_ENDCONNECTOR && joint->getClass() != JC_CONNECTOR) {
			Joint2Joint* src = (Joint2Joint*)joint;
			Joint* dst = src->getDestination();

			if (drawIt) {			
				cv::Point2f dstPos2d = dst->getPos2d();			
				float length = distanceP2f(srcPos2d, dstPos2d);
				
				//if (srcPos2d.x >= 0 && srcPos2d.y >= 0 &&
					//dstPos2d.x >= 0 && dstPos2d.y >= 0)
						cv::line(m_skeletonImg, srcPos2d, dstPos2d, cv::Scalar(255, 255, 255));
		
				// get normal
				cv::Point2f normal1 = dstPos2d - srcPos2d;
				normal1 = cv::Point2f(normal1.y, -normal1.x);
				normal1 *= 1.0f / (float)cv::norm(normal1);

				float boneSize = (length / 8.0f);
		
				// rotate 45 degrees
				cv::Point2f rotated1 = cv::Point2f(normal1.x * cosf(45) - normal1.y * sinf(45),
					normal1.x * sinf(45) + normal1.y * cosf(45));

				cv::Point2f newPoint1 = rotated1 * boneSize;

				cv::line(m_skeletonImg, srcPos2d, srcPos2d + newPoint1, cv::Scalar(255, 255, 255));
				cv::line(m_skeletonImg, srcPos2d + newPoint1, dstPos2d, cv::Scalar(255, 255, 255));

				cv::Point2f normal2 = -normal1;

				// rotate 45 degrees
				cv::Point2f rotated2 = cv::Point2f(normal2.x * cosf(-45) - normal2.y * sinf(-45),
					normal2.x * sinf(-45) + normal2.y * cosf(-45));

				cv::Point2f newPoint2 = rotated2 * boneSize;

				cv::line(m_skeletonImg, srcPos2d, srcPos2d + newPoint2, cv::Scalar(255, 255, 255));
				cv::line(m_skeletonImg, srcPos2d + newPoint2, dstPos2d, cv::Scalar(255, 255, 255));
			}

			drawBone(dst);
		}
		
		// draw joint location
		if (!joint->isSubJoint() && drawIt) {
			cv::Scalar color(0, 0, 0);
			
			if (joint->getClass() == JC_ENDCONNECTOR)
				color = cv::Scalar(0, 0, 255);
			else
				color = cv::Scalar(255, 255, 255);
			
			int circleSize = (int)((10.0 / jointDist) * (m_skeletonImg.cols / 320));
			if (circleSize < 2)
				circleSize = 2;
			cv::circle(m_skeletonImg, srcPos2d, circleSize, color);
		}
	}
}
