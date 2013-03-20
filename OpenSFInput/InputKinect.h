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
