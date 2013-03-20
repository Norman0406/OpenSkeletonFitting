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
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

namespace osf
{
	class Input
		: public Module
	{
	public:
		virtual ~Input(void);

		virtual bool isInit() const;
		
		void shutdown();
		void startCapture();
		void startCapture(const std::string& prefix);
		void stopCapture();
		
		void setResizing(const cv::Size&);
		const cv::Size& getResizing() const;

		const cv::Mat& getProjMat() const;
		const cv::Mat& getCameraMat() const;
		const cv::Mat& getRotMat() const;
		const cv::Mat& getTransMat() const;
		const cv::Mat& getImgDepth() const;
		const cv::Mat& getImg3d() const;
		const cv::Mat& getImgColor() const;

	protected:
		Input(System*);

		virtual void iInit();
		void iProcess();
		virtual void iGrabImages() = 0;
		
		cv::Mat	m_imgDepth;
		cv::Mat m_imgColor;
		cv::Mat m_img3d;
		cv::Mat m_outDepth;
		cv::Mat m_outColor;
		cv::Mat m_outImg3d;
		cv::Size m_resizing;

	private:
		void processThread();
		void computeProjectionMat();
		
		cv::Mat m_projMat;
		cv::Mat m_matCamera;
		cv::Mat m_matRot;
		cv::Mat m_matTrans;
		bool m_capturing;
		std::string m_capFormat;
		int m_frame;

		// thread variables
		boost::thread m_processThread;
		boost::mutex m_startMutex;
		boost::mutex m_condMutex1;
		boost::condition_variable_any m_condVar1;
		bool m_cond1;
		boost::mutex m_condMutex2;
		boost::condition_variable_any m_condVar2;
		bool m_cond2;
		boost::mutex m_shutdownMutex;
		bool m_shutdownCond;
	};
}
