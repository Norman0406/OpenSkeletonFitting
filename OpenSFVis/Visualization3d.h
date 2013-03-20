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

	class Visualization3d
	{
	public:
		Visualization3d(Input*, Segmentation*, Features*, Fitting*, cv::Size wS = cv::Size(640, 480));
		~Visualization3d(void);
		
		bool draw(bool&, bool&);
		void init();

	private:
		void render();
		void processKeyboard();
		void processMouse();
		void rotateCameraSphere(float, float);
		void renderSphere(float, int, int);
		void renderLookAt();
		void renderOrigin(float width = 1.0f);
		void renderScene();
		void renderBone(Joint*, float);
		
		Input* m_input;
		Segmentation* m_segmentation;
		Features* m_features;
		Fitting* m_fitting;
		bool m_isInit;
		
		// Input
		const cv::Mat* m_imgDepth;
		const cv::Mat* m_img3d;
		const cv::Mat* m_imgColor;

		// Segmentation
		const cv::Mat* m_imgSegDepth;
		const cv::Mat* m_imgSeg3d;

		// Features
		const cv::Mat* m_imgGeo;
		const cv::Mat* m_imgPred;
		const std::vector<FeaturePoint*>* m_vecFeatures;

		// Fitting
		Joint* m_rootJoint;
	
		// OpenGL
		std::vector<cv::Point3f> m_labelColors;
		cv::Size m_windowSize;
		cv::Point3f m_camPos;
		cv::Point3f m_camDir;
		cv::Point3f m_camUp;
		cv::Point3f m_camLookAt;
		float m_theta;
		float m_phi;
		float m_radius;
		cv::Point m_lastMousePos;
		int m_lastMouseWheel;
		bool m_fixedLookAt;
		int m_displayList;
		bool m_spacePressed;
		bool m_enterPressed;
	};
}
