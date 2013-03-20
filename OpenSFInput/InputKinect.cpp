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

#include "precompiled.h"
#include "InputKinect.h"

namespace osf
{
	InputKinect::InputKinect(System* sys)
		: Input(sys), m_isInit(false), m_imgPoints(0), m_worldPoints(0)
	{
		m_deviceIndex = 0;
		m_registerDepth = false;
		m_motorAngle = 0;
		m_lastMotorAngle = 0;
		m_motorDev = 0;
		m_sizeDepth = cv::Size(640, 480);
		m_sizeColor = cv::Size(640, 480);
		m_startRecording = false;
		m_isRecording = false;
		m_recFilename = "rec.oni";
	}

	InputKinect::~InputKinect(void)
	{
		stopRecording();

		delete[] m_imgPoints;
		m_imgPoints = 0;
		delete[] m_worldPoints;
		m_worldPoints = 0;
		m_context.Release();
		m_depthGenerator.Release();
		m_imageGenerator.Release();
		if (m_motorDev)
			xnUSBCloseDevice(m_motorDev);
	}
	
	void InputKinect::setDeviceIndex(int index)
	{
		if (isInit())
			ERR << "already init" << ENDL;

		m_deviceIndex = index;
	}

	void InputKinect::setRegisterDepth(bool registerDepth)
	{
		m_registerDepth = registerDepth;
	}

	void InputKinect::setMotorAngle(int angle)
	{
		m_motorAngle = angle;
	}

	int InputKinect::getDeviceIndex() const
	{
		return m_deviceIndex;
	}

	bool InputKinect::getRegisterDepth() const
	{
		return m_registerDepth;
	}

	int InputKinect::getMotorAngle() const
	{
		return m_motorAngle;
	}
	
	const cv::Size& InputKinect::getDepthSize() const
	{
		return m_sizeDepth;
	}

	const cv::Size& InputKinect::getColorSize() const
	{
		return m_sizeColor;
	}

	void InputKinect::setDepthSize(cv::Size size)
	{
		if (m_isInit) {
			WARN << "this function can only be called before initialization" << ENDL;
			return;
		}

		m_sizeDepth = size;
	}

	void InputKinect::setColorSize(cv::Size size)
	{
		if (m_isInit) {
			WARN << "this function can only be called before initialization" << ENDL;
			return;
		}

		m_sizeColor = size;
	}

	void InputKinect::iInit()
	{
		// init motor
		initMotor();

		if (isInit())
			throw Exception("already init");

		XnStatus rc = XN_STATUS_OK;

		// get OpenNI version
		LOG << "using OpenNI " << XN_BRIEF_VERSION_STRING << ENDL;

		// init
		rc = m_context.Init();
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
		
		// create recorder
		m_recorder.Create(m_context);

		// create generator nodes
		createDepthGenerator();
		createImageGenerator();
	
		// register depth to color image
		if (m_registerDepth) {
			rc = m_depthGenerator.GetAlternativeViewPointCap().SetViewPoint(m_imageGenerator);
			if (rc != XN_STATUS_OK)
				WARN << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
		}
			
		// start generating
		rc = m_context.StartGeneratingAll();
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
		
		// sync map generators
		if (m_depthGenerator.GetFrameSyncCap().CanFrameSyncWith(m_imageGenerator) &&
			!m_depthGenerator.GetFrameSyncCap().IsFrameSyncedWith(m_imageGenerator)) {
				// NOTE: does not work at the moment [1/30/2012 Norman]
				rc = m_depthGenerator.GetFrameSyncCap().FrameSyncWith(m_imageGenerator);
				if (rc != XN_STATUS_OK)
					WARN << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
		}
		
		Input::iInit();

		m_isInit = true;
	}

	void InputKinect::initMotor()
	{
		const XnUSBConnectionString *paths;
		XnUInt32 count;
		XnStatus rc;

		// Init OpenNI USB
		rc = xnUSBInit(); 
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}

		// Open "Kinect motor" USB device
		rc = xnUSBEnumerateDevices(0x045E /* VendorID */, 0x02B0 /*ProductID */, &paths, &count); 
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}

		// Open first found device
		rc = xnUSBOpenDeviceByPath(paths[0], &m_motorDev); 
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			if (m_motorDev) {
				xnUSBCloseDevice(m_motorDev);
				m_motorDev = 0;
			}
			return;
		}

		// Init motor
		XnUChar buf[1]; // output buffer
		rc = xnUSBSendControl(m_motorDev, (XnUSBControlType)0xc0, 0x10, 0x00, 0x00, buf, sizeof(buf), 0); 
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			if (m_motorDev) {
				xnUSBCloseDevice(m_motorDev);
				m_motorDev = 0;
			}
			return;
		}

		rc = xnUSBSendControl(m_motorDev, XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 0, 0); 
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			if (m_motorDev) {
				xnUSBCloseDevice(m_motorDev);
				m_motorDev = 0;
			}
			return;
		}
	}

	bool InputKinect::isInit() const
	{
		return m_isInit &&
			Input::isInit();
	}
	
	void InputKinect::createDepthGenerator()
	{
		XnStatus rc = XN_STATUS_OK;
		xn::NodeInfoList depthNodes;
		int i = 0;

		// get list of depth nodes
		rc = m_context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, depthNodes, NULL);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
	
		// iterate over nodes
		for (xn::NodeInfoList::Iterator nodeIt = depthNodes.Begin(); nodeIt != depthNodes.End(); ++nodeIt) { 
			const xn::NodeInfo& info = *nodeIt; 
			const XnProductionNodeDescription& desc = info.GetDescription();

			LOG << "found device " << i << ": " << desc.strVendor << " " << desc.strName << ENDL;
			i++;
		}
		
		if (i == 0)
			throw Exception("no modules found");
	
		// create node
		createMapGeneratorNode(m_depthGenerator, depthNodes);

		/* Kinect does not yet support setting different map output modes
		// get resolutions
		XnUInt32 mapOutputModes = m_depthGenerator.GetSupportedMapOutputModesCount();
		XnMapOutputMode* modes = new XnMapOutputMode[mapOutputModes];
		rc = m_depthGenerator.GetSupportedMapOutputModes(modes, mapOutputModes);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}

		// enumerate resolutions and chose best one with the highest framerate
		int maxFPS = 0;
		int index = -1;
		int maxResIndex = -1;
		cv::Size maxRes(0, 0);
		for (int i = 0; i < (int)mapOutputModes; i++) {
			XnMapOutputMode mode = modes[i];
			
			// find appropriate resolution with highest framerate
			if (mode.nXRes == m_sizeDepth.width &&
				mode.nYRes == m_sizeDepth.height &&
				(int)mode.nFPS > maxFPS) {
					index = i;
					maxFPS = mode.nFPS;
			}

			// alternatively find highest resolution
			if ((int)mode.nXRes > maxRes.width && (int)mode.nYRes > maxRes.height) {
				maxResIndex = i;
				maxRes = cv::Size((int)mode.nXRes, (int)mode.nYRes);
			}
		}
		
		// choose best resolution
		XnMapOutputMode depthOutputMode;
		if (index < 0)
			depthOutputMode = modes[maxResIndex];
		else
			depthOutputMode = modes[index];*/

		XnMapOutputMode depthOutputMode;
		depthOutputMode.nXRes = XN_VGA_X_RES;
		depthOutputMode.nYRes = XN_VGA_Y_RES;
		depthOutputMode.nFPS = 30;
		
		// set output mode
		rc = m_depthGenerator.SetMapOutputMode(depthOutputMode);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
	
		// get additional meta data
		m_depthGenerator.GetMetaData(m_depthMetaData);
	
		// get size
		m_sizeDepth = cv::Size(m_depthMetaData.XRes(), m_depthMetaData.YRes());
		m_maxDepth = m_depthMetaData.ZRes();
	
		// create images
		m_imgDepth = cv::Mat(m_sizeDepth, CV_32FC1);
		m_img3d = cv::Mat(m_sizeDepth, CV_32FC3);
	
		// create conversion tables
		m_imgPoints = new XnPoint3D[m_sizeDepth.width * m_sizeDepth.height];
		m_worldPoints = new XnPoint3D[m_sizeDepth.width * m_sizeDepth.height];
	}

	void InputKinect::createImageGenerator()
	{
		XnStatus rc = XN_STATUS_OK;

		xn::NodeInfoList imageNodes;
		rc = m_context.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, NULL, imageNodes, NULL);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
		
		// create nodes
		createMapGeneratorNode(m_imageGenerator, imageNodes);
		
		/* Kinect does not yet support setting different map output modes
		// get resolutions
		XnUInt32 mapOutputModes = m_imageGenerator.GetSupportedMapOutputModesCount();
		XnMapOutputMode* modes = new XnMapOutputMode[mapOutputModes];
		rc = m_imageGenerator.GetSupportedMapOutputModes(modes, mapOutputModes);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
				
		// enumerate resolutions and chose best one with the highest framerate
		int maxFPS = 0;
		int index = -1;
		int maxResIndex = -1;
		cv::Size maxRes(0, 0);
		for (int i = 0; i < (int)mapOutputModes; i++) {
			XnMapOutputMode mode = modes[i];
			
			// find appropriate resolution with highest framerate
			if (mode.nXRes == m_sizeColor.width &&
				mode.nYRes == m_sizeColor.height &&
				(int)mode.nFPS > maxFPS) {
					index = i;
					maxFPS = mode.nFPS;
			}

			// alternatively find highest resolution
			if ((int)mode.nXRes > maxRes.width && (int)mode.nYRes > maxRes.height) {
				maxResIndex = i;
				maxRes = cv::Size((int)mode.nXRes, (int)mode.nYRes);
			}
		}
		
		// choose best resolution
		XnMapOutputMode colorOutputMode;
		if (index < 0)
			colorOutputMode = modes[maxResIndex];
		else
			colorOutputMode = modes[index];*/
		
		XnMapOutputMode colorOutputMode;
		colorOutputMode.nXRes = XN_VGA_X_RES;
		colorOutputMode.nYRes = XN_VGA_Y_RES;
		colorOutputMode.nFPS = 30;

		// set color output mode
		rc = m_imageGenerator.SetMapOutputMode(colorOutputMode);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}

		// get additional data
		m_imageGenerator.GetMetaData(m_imageMetaData);
	
		m_sizeColor = cvSize(m_imageMetaData.XRes(), m_imageMetaData.YRes());

		if (m_imageMetaData.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
			throw Exception("pixel format is not RGB24");
	
		// create image
		m_imgColor = cv::Mat(m_sizeColor, CV_8UC3);
	}

	void InputKinect::createMapGeneratorNode(xn::MapGenerator& generator, const xn::NodeInfoList& list)
	{
		XnStatus rc = XN_STATUS_OK;
		int i = 0;
		bool created = false;
		for (xn::NodeInfoList::Iterator nodeIt = list.Begin(); nodeIt != list.End(); ++nodeIt) { 
			const xn::NodeInfo& info = *nodeIt; 
			const XnProductionNodeDescription& desc = info.GetDescription();

			if (i == m_deviceIndex) {
				xn::ProductionNode node;
				rc = m_context.CreateProductionTree(const_cast<xn::NodeInfo&>(info), node);
				if (rc != XN_STATUS_OK) {
					ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
					throw Exception("could not init");
				}

				rc = info.GetInstance(generator);
				if (rc != XN_STATUS_OK) {
					ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
					throw Exception("could not init");
				}

				LOG << "created [generator] " << generator.GetName() << ENDL;

				created = true;
			}

			i++;
		}

		if (!created || !generator.IsValid())
			throw Exception("no generator created");
	}

	void InputKinect::moveMotor(int angle)
	{
		if (m_motorDev) {
			XnStatus rc; 
			// Send move control request 
			rc = xnUSBSendControl(m_motorDev, XN_USB_CONTROL_TYPE_VENDOR, 0x31, angle, 0x00, NULL, 0, 0); 
			if (rc != XN_STATUS_OK) {
				ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
				throw Exception("could not init");
			}
		}
	}

	void InputKinect::startRecording(std::string filename)
	{
		m_startRecording = true;
		m_recFilename = filename;
	}
	
	void InputKinect::startRecording()
	{
		XnStatus rc;

		rc = m_recorder.SetDestination(XN_RECORD_MEDIUM_FILE, m_recFilename.c_str());
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not start recording");
		}

		rc = m_recorder.AddNodeToRecording(m_depthGenerator);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not start recording");
		}

		rc = m_recorder.AddNodeToRecording(m_imageGenerator);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not start recording");
		}

		m_startRecording = false;
		m_isRecording = true;
	}

	void InputKinect::stopRecording()
	{
		if (m_isRecording) {
			XnStatus rc;

			rc = m_recorder.RemoveNodeFromRecording(m_depthGenerator);
			if (rc != XN_STATUS_OK)
				WARN << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;

			rc = m_recorder.RemoveNodeFromRecording(m_imageGenerator);
			if (rc != XN_STATUS_OK)
				WARN << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
		}
	}

	void InputKinect::update()
	{
		m_context.WaitAndUpdateAll();
		m_depthGenerator.GetMetaData(m_depthMetaData);
		m_imageGenerator.GetMetaData(m_imageMetaData);

		if (m_startRecording)
			startRecording();
		
		if (m_isRecording) {
			XnStatus rc = m_recorder.Record();
			if (rc != XN_STATUS_OK)
				WARN << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
		}
				
		// get depth map
		const XnDepthPixel* pDepth = m_depthMetaData.Data();
		const XnUInt8* pImage = m_imageMetaData.Data();

		// get depth pixels
		for (int i = 0; i < m_sizeDepth.height; i++) {
			for (int j = 0; j < m_sizeDepth.width; j++) {			
				XnDepthPixel pix = *(pDepth++);
				float dist = pix / 1000.0f;
				m_imgDepth.ptr<float>(i)[j] = dist;

				m_imgPoints[i * m_sizeDepth.width + j].X = (XnFloat)j;
				m_imgPoints[i * m_sizeDepth.width + j].Y = (XnFloat)i;
				m_imgPoints[i * m_sizeDepth.width + j].Z = (XnFloat)dist;
			}
		}

		// get color pixels
		for (int i = 0; i < m_sizeColor.height; i++) {
			for (int j = 0; j < m_sizeColor.width; j++) {
				XnUInt8 pixR = *(pImage++);
				XnUInt8 pixG = *(pImage++);
				XnUInt8 pixB = *(pImage++);

				uchar* row = m_imgColor.ptr<uchar>(i);
				row[j * 3 + 0] = pixB;
				row[j * 3 + 1] = pixG;
				row[j * 3 + 2] = pixR;
			}
		}

		// get 3d image
		m_depthGenerator.ConvertProjectiveToRealWorld(m_sizeDepth.width * m_sizeDepth.height,
			m_imgPoints, m_worldPoints);
		for (int i = 0; i < m_sizeDepth.height; i++) {
			for (int j = 0; j < m_sizeDepth.width; j++) {
				XnPoint3D worldPoint = m_worldPoints[i * m_sizeDepth.width + j];
				float* row = m_img3d.ptr<float>(i);
				row[j * 3 + 0] = worldPoint.X;
				row[j * 3 + 1] = -worldPoint.Y;
				row[j * 3 + 2] = worldPoint.Z;
			}
		}
	}

	void InputKinect::iGrabImages()
	{
		if (!isInit())
			throw Exception("not init");

		// move motor if change detected
		if (m_motorDev) {
			if (m_lastMotorAngle != m_motorAngle)
				moveMotor(m_motorAngle);
			m_lastMotorAngle = m_motorAngle;
		}

		update();

		/*saveCvMat("ImgDepth.cvm", m_imgDepth);
		saveCvMat("Img3d.cvm", m_img3d);*/
	}
}
