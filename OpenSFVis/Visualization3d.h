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
