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
	class InputPlayerONI
		: public Input
	{
		MK_TYPE(InputPlayerONI);

	public:
		InputPlayerONI(System*);
		~InputPlayerONI(void);

		bool isInit() const;

		// public interface
		void setFilename(std::string);
		void setRepeat(bool);
		void setPlaybackSpeed(double);
		void setPauseMode(bool);

		const cv::Size& getDepthSize() const;
		const cv::Size& getColorSize() const;

	protected:
		void iInit();
		void iGrabImages();

	private:
		void createDepthGenerator();
		void createImageGenerator();
		void update();
		
		bool m_isInit;
		
		xn::Context			m_context;
		xn::Player			m_player;
		xn::DepthGenerator	m_depthGenerator;
		xn::DepthMetaData	m_depthMetaData;
		xn::ImageGenerator	m_imageGenerator;
		xn::ImageMetaData	m_imageMetaData;
		int					m_maxDepth;
		cv::Size			m_sizeDepth;
		cv::Size			m_sizeColor;
		XnPoint3D*			m_imgPoints;
		XnPoint3D*			m_worldPoints;
		bool				m_firstRun;

		// parameters
		std::string			m_filename;
		bool				m_repeat;
		bool				m_paused;

		// <= 0: as fast as possible, ]0, 1[: slow motion, == 1: original speed, > 1: fast motion
		double				m_playbackSpeed;
	};
}
