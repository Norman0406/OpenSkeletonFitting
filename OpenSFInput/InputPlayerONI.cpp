#include "precompiled.h"
#include "../OpenSFLib/System.h"
#include "InputPlayerONI.h"

namespace osf
{
	InputPlayerONI::InputPlayerONI(System* sys)
		: Input(sys), m_isInit(false), m_imgPoints(0), m_worldPoints(0)
	{
		m_sizeDepth = cv::Size(640, 480);
		m_sizeColor = cv::Size(640, 480);
		m_filename = "rec.oni";
		m_repeat = true;
		m_paused = false;
		m_playbackSpeed = 1.0;
		m_firstRun = true;
	}

	InputPlayerONI::~InputPlayerONI(void)
	{
		delete[] m_imgPoints;
		m_imgPoints = 0;
		delete[] m_worldPoints;
		m_worldPoints = 0;
		m_context.Release();
		m_depthGenerator.Release();
		m_imageGenerator.Release();
	}
	
	const cv::Size& InputPlayerONI::getDepthSize() const
	{
		return m_sizeDepth;
	}

	const cv::Size& InputPlayerONI::getColorSize() const
	{
		return m_sizeColor;
	}
	
	void InputPlayerONI::setFilename(std::string filename)
	{
		m_filename = filename;
	}
	
	void InputPlayerONI::setRepeat(bool repeat)
	{
		m_repeat = repeat;
	}

	void InputPlayerONI::setPlaybackSpeed(double speed)
	{
		m_playbackSpeed = speed;
	}

	void InputPlayerONI::setPauseMode(bool paused)
	{
		m_paused = paused;
	}

	void InputPlayerONI::iInit()
	{
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
		
		// create player
		rc = m_context.OpenFileRecording(m_filename.c_str(), m_player);
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
				
		// create generator nodes
		createDepthGenerator();
		createImageGenerator();

		if (m_playbackSpeed <= 0)
			m_playbackSpeed = XN_PLAYBACK_SPEED_FASTEST;

		m_player.SetRepeat(m_repeat);
		m_player.SetPlaybackSpeed(m_playbackSpeed);
			
		// start generating
		rc = m_context.StartGeneratingAll();
		if (rc != XN_STATUS_OK) {
			ERR << xnGetStatusName(rc) << ": " << xnGetStatusString(rc) << ENDL;
			throw Exception("could not init");
		}
		
		Input::iInit();

		m_isInit = true;
	}

	bool InputPlayerONI::isInit() const
	{
		return m_isInit &&
			Input::isInit();
	}
	
	void InputPlayerONI::createDepthGenerator()
	{
		XnStatus rc = XN_STATUS_OK;
		rc = m_context.FindExistingNode(XN_NODE_TYPE_DEPTH, m_depthGenerator);
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

	void InputPlayerONI::createImageGenerator()
	{
		XnStatus rc = XN_STATUS_OK;
		rc = m_context.FindExistingNode(XN_NODE_TYPE_IMAGE, m_imageGenerator);
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

	void InputPlayerONI::update()
	{
		if (!m_paused || m_firstRun) {
			m_context.WaitAndUpdateAll();
			
			m_depthGenerator.GetMetaData(m_depthMetaData);
			m_imageGenerator.GetMetaData(m_imageMetaData);

			if (m_firstRun)
				m_firstRun = false;
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
		
		if (m_player.IsEOF()) {
			m_system->terminate(true);
			return;
		}
	}

	void InputPlayerONI::iGrabImages()
	{
		if (!isInit())
			throw Exception("not init");

		update();
	}
}