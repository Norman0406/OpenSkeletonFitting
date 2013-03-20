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
	class Input;
	class Segmentation;
	class Features;
	class FeaturePoint;
	class Fitting;
	class Joint;

	class Visualization2d
	{
	public:
		Visualization2d(Input*, Segmentation*, Features*, Fitting*);
		~Visualization2d(void);
		
		bool draw(bool&, bool&);
		void init();
		
		void recordDepthMap(std::string, int fourcc = 0, int framerate = 20);
		void recordSegmentation(std::string, int fourcc = 0, int framerate = 20);
		void recordGeodesicMap(std::string, int fourcc = 0, int framerate = 20);
		void recordSkeleton(std::string, int fourcc = 0, int framerate = 20);

	private:
		void drawSkeleton();
		void drawBone(const Joint*);

		Input* m_input;
		Segmentation* m_segmentation;
		Features* m_features;
		Fitting* m_fitting;
		bool m_isInit;
		
		// Input
		const cv::Mat* m_imgDepth;
		const cv::Mat* m_img3d;
		const cv::Mat* m_imgColor;
		const cv::Mat* m_projMat;

		// Segmentation
		const cv::Mat* m_imgSegDepth;
		const cv::Mat* m_imgSeg3d;

		// Features
		const cv::Mat* m_imgGeo;
		const cv::Mat* m_imgPred;
		const std::vector<FeaturePoint*>* m_vecFeatures;
		cv::Mat m_featVis;

		// Fitting
		Joint* m_rootJoint;
		cv::Mat m_skeletonImg;

		// recording
		struct VideoWriter
		{
			std::string filename;
			int fourcc;
			int framerate;
			bool write;
			cv::VideoWriter writer;

			VideoWriter() {
				filename.clear();
				fourcc = 0;
				framerate = 20;
				write = false;
			}
		};
		
		VideoWriter	m_writerDepthMap;
		VideoWriter	m_writerSegmentation;
		VideoWriter	m_writerGeodesic;
		VideoWriter	m_writerSkeleton;
	};
}
