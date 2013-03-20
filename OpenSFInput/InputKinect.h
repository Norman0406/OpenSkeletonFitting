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
#include "Input.h"

#pragma warning(disable:4005)
#include <XnCppWrapper.h>
#include <XnUSB.h> 
#pragma warning(default:4005)

namespace osf
{
	class InputKinect
		: public Input
	{
		MK_TYPE(InputKinect);

	public:
		InputKinect(System*);
		~InputKinect(void);

		bool isInit() const;

		// parameters
		void setDeviceIndex(int);
		void setRegisterDepth(bool);
		void setMotorAngle(int);
		void setDepthSize(cv::Size);	// to be called before initialization
		void setColorSize(cv::Size);	// to be called before initialization
		
		int getDeviceIndex() const;
		bool getRegisterDepth() const;
		int getMotorAngle() const;
		const cv::Size& getDepthSize() const;
		const cv::Size& getColorSize() const;

		void startRecording(std::string);
		void stopRecording();

	protected:
		void iInit();
		void iGrabImages();

	private:
		void initMotor();
		void moveMotor(int);
		void createDepthGenerator();
		void createImageGenerator();
		void createMapGeneratorNode(xn::MapGenerator& generator, const xn::NodeInfoList& list);
		void startRecording();
		void update();
		
		bool m_isInit;
		
		XN_USB_DEV_HANDLE	m_motorDev;
		xn::Context			m_context;
		xn::Recorder		m_recorder;
		xn::DepthGenerator	m_depthGenerator;
		xn::DepthMetaData	m_depthMetaData;
		xn::ImageGenerator	m_imageGenerator;
		xn::ImageMetaData	m_imageMetaData;
		int					m_maxDepth;
		cv::Size			m_sizeDepth;
		cv::Size			m_sizeColor;
		XnPoint3D*			m_imgPoints;
		XnPoint3D*			m_worldPoints;
		int					m_motorAngle;
		int					m_lastMotorAngle;

		// parameters
		int					m_deviceIndex;
		bool				m_registerDepth;
		bool				m_startRecording;
		bool				m_isRecording;
		std::string			m_recFilename;
	};
}
