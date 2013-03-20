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
