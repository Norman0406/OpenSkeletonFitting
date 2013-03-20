#include "precompiled.h"
#include "Input.h"
#include "InputFactory.h"

namespace osf
{

	Input::Input(System* sys)
		: Module(sys), m_capturing(false), m_frame(0)
	{
		m_imgDepth.setTo(0);
		m_imgColor.setTo(0);
		m_img3d.setTo(0);
		m_capFormat.clear();
		
		// set initial resizing
		m_resizing = cv::Size(640, 480);

		addOutput(&m_outDepth);
		addOutput(&m_outColor);
		addOutput(&m_outImg3d);
		
		srand((int)time(NULL));

		m_cond1 = false;
		m_cond2 = false;
		m_shutdownCond = false;
	}
	
	Input::~Input(void)
	{
	}

	void Input::shutdown()
	{
		// set shutdown condition for thread
		m_shutdownMutex.lock();
		m_shutdownCond = true;
		m_shutdownMutex.unlock();

		// notify the thread
		m_condMutex2.lock();
		m_cond2 = true;
		m_condVar2.notify_all();
		m_condMutex2.unlock();

		// let the thread finish its work
		m_processThread.join();
	}
	
	void Input::iInit()
	{
		m_outDepth = cv::Mat(m_resizing, m_imgDepth.type());
		m_outColor = cv::Mat(m_resizing, m_imgColor.type());
		m_outImg3d = cv::Mat(m_resizing, m_img3d.type());

		m_outDepth.setTo(0);
		m_outColor.setTo(0);
		m_outImg3d.setTo(0);		
		
		// create input processing thread
		m_processThread = boost::thread(boost::bind(&Input::processThread, this));

		// NOTE: the thread will start working immediately, but this is okay, as it waits
		// for synchronization for the main thread until it continues its work. [2/24/2012 Norman]
	}

	bool Input::isInit() const
	{
		return !m_imgDepth.empty() &&
			!m_img3d.empty() &&
			Module::isInit();
	}

	void Input::startCapture()
	{
		startCapture("img");
	}

	void Input::startCapture(const std::string& prefix)
	{
		if (m_capturing) {
			WARN << "already capturing" << ENDL;
			return;
		}

		m_capturing = true;
		m_capFormat = std::string(prefix) + "_%d_%s.cvm";
	}

	void Input::stopCapture()
	{
		if (!m_capturing) {
			WARN << "not capturing" << ENDL;
			return;
		}

		m_capturing = false;
	}
	
	void Input::setResizing(const cv::Size& resizing)
	{
		if (isInit()) {
			WARN << "already init, cannot change resizing any longer" << ENDL;
			return;
		}

		m_resizing = resizing;
	}

	const cv::Size& Input::getResizing() const
	{
		return m_resizing;
	}

	const cv::Mat& Input::getProjMat() const
	{
		return m_projMat;
	}

	const cv::Mat& Input::getCameraMat() const
	{
		return m_matCamera;
	}

	const cv::Mat& Input::getRotMat() const
	{
		return m_matRot;
	}

	const cv::Mat& Input::getTransMat() const
	{
		return m_matTrans;
	}

	const cv::Mat& Input::getImgDepth() const
	{
		return m_outDepth;
	}

	const cv::Mat& Input::getImg3d() const
	{
		return m_outImg3d;
	}

	const cv::Mat& Input::getImgColor() const
	{
		return m_outColor;
	}

	void Input::computeProjectionMat()
	{
		if (m_outImg3d.empty())
			throw Exception("3d image invalid");

		std::vector<std::vector<float> > objPoints;
		std::vector<std::vector<float> > imgPoints;
		int count = 6;
		
		// create point correspondences with random points
		objPoints.reserve(count);
		imgPoints.reserve(count);

		int maxNumIters = count * 20;
		int numIters = 0;
		do {
			if (numIters > maxNumIters)
				throw Exception("could not compute projection matrix");

			numIters++;

			float val1 = rand() / (float)RAND_MAX;
			float val2 = rand() / (float)RAND_MAX;
			int x = (int)(val1 * m_outImg3d.cols);
			int y = (int)(val2 * m_outImg3d.rows);

			if (x < 0 || x >= m_outImg3d.cols ||
				y < 0 || y >= m_outImg3d.rows)
				continue;

			const float* row = m_outImg3d.ptr<float>(y);
				
			std::vector<float> imgPoint(3);
			imgPoint[0] = (float)x;
			imgPoint[1] = (float)y;
			imgPoint[2] = 1.0f;

			std::vector<float> objPoint(4);
			objPoint[0] = row[x * 3 + 0];
			objPoint[1] = row[x * 3 + 1];
			objPoint[2] = row[x * 3 + 2];
			objPoint[3] = 1.0f;

			if (objPoint[2] > 0) {
				objPoints.push_back(objPoint);
				imgPoints.push_back(imgPoint);
			}
		} while ((int)objPoints.size() < count);

		// reconstruction from given point correspondences
		// (see http://de.wikipedia.org/wiki/Projektionsmatrix#Berechnung_der_Projektionsmatrix_aus_Punktkorrespondenzen)
		
		// create Matrix A
		cv::Mat matA(2 * count, 12, CV_32FC1);
		for (int i = 0, j = 0; i < 2 * count; i+=2, j++) {	// cols
			// first row
			matA.ptr<float>(i)[0] = objPoints[j][0];
			matA.ptr<float>(i)[1] = objPoints[j][1];
			matA.ptr<float>(i)[2] = objPoints[j][2];
			matA.ptr<float>(i)[3] = objPoints[j][3];
			
			matA.ptr<float>(i)[4] = matA.ptr<float>(i)[5] =
				matA.ptr<float>(i)[6] = matA.ptr<float>(i)[7] = 0;
			
			matA.ptr<float>(i)[8] = - imgPoints[j][0] * objPoints[j][0];
			matA.ptr<float>(i)[9] = - imgPoints[j][0] * objPoints[j][1];
			matA.ptr<float>(i)[10] = - imgPoints[j][0] * objPoints[j][2];
			matA.ptr<float>(i)[11] = - imgPoints[j][0] * objPoints[j][3];
			
			// second row
			matA.ptr<float>(i+1)[0] = matA.ptr<float>(i+1)[1] =
				matA.ptr<float>(i+1)[2] = matA.ptr<float>(i+1)[3] = 0;

			matA.ptr<float>(i+1)[4] = objPoints[j][0];
			matA.ptr<float>(i+1)[5] = objPoints[j][1];
			matA.ptr<float>(i+1)[6] = objPoints[j][2];
			matA.ptr<float>(i+1)[7] = objPoints[j][3];			
			
			matA.ptr<float>(i+1)[8] = - imgPoints[j][1] * objPoints[j][0];
			matA.ptr<float>(i+1)[9] = - imgPoints[j][1] * objPoints[j][1];
			matA.ptr<float>(i+1)[10] = - imgPoints[j][1] * objPoints[j][2];
			matA.ptr<float>(i+1)[11] = - imgPoints[j][1] * objPoints[j][3];
		}

		// solve using SVD
		cv::Mat w;
		cv::Mat u;
		cv::Mat vt;

		cv::SVD mySVD;
		mySVD.compute(matA, w, u, vt);
		
		// create matrix
		m_projMat = cv::Mat(3, 4, CV_32FC1);
		for (int i = 0; i < vt.rows; i++) {
			m_projMat.ptr<float>(0)[i] = vt.ptr<float>(vt.cols - 1)[i] /
				vt.ptr<float>(vt.cols - 1)[vt.rows - 1];
		}

		// normalize projection matrix
		float sum = m_projMat.ptr<float>(2)[0] +
			m_projMat.ptr<float>(2)[1] +
			m_projMat.ptr<float>(2)[2];
		m_projMat = m_projMat * (1.0f / sum);
		
		// decompose
		cv::decomposeProjectionMatrix(m_projMat, m_matCamera, m_matRot, m_matTrans);
	}
		
	void Input::processThread()
	{
		m_startMutex.lock();

		while (true) {
			// check if this thread has to be closed
			m_shutdownMutex.lock();
			if (m_shutdownCond)
				break;
			m_shutdownMutex.unlock();

			// grab images
			iGrabImages();
			
			// notify the main thread that this thread has finished its work
			m_condMutex1.lock();
			m_cond1 = true;
			m_condVar1.notify_all();
			m_condMutex1.unlock();
			
			// wait for the main thread to be ready for the next pass
			m_condMutex2.lock();
			while (!m_cond2)
				m_condVar2.wait(m_condMutex2);
			m_cond2 = false;
			m_condMutex2.unlock();
		}

		m_shutdownMutex.unlock();
	}

	void Input::iProcess()
	{
		if (!isInit())
			throw Exception("not init");
		
		m_startMutex.unlock();

		// wait for the thread to be finished
		m_condMutex1.lock();
		while (!m_cond1)
			m_condVar1.wait(m_condMutex1);
		m_cond1 = false;
		m_condMutex1.unlock();
		
		// resize image data
		cv::resize(m_imgDepth, m_outDepth, m_resizing, 0, 0, CV_INTER_NN);
		cv::resize(m_imgColor, m_outColor, m_resizing, 0, 0, CV_INTER_NN);
		cv::resize(m_img3d, m_outImg3d, m_resizing, 0, 0, CV_INTER_NN);

		// compute projection matrix on first frame
		if (m_projMat.empty())
			computeProjectionMat();

		if (m_projMat.empty())
			throw Exception("projection matrix invalid");
		
		// capture data
		if (m_capturing) {
			char buffer[512];
						
			// save depth image
			sprintf(buffer, m_capFormat.c_str(), m_frame, "depth");
			saveCvMat(buffer, m_imgDepth);
			
			// save color image
			sprintf(buffer, m_capFormat.c_str(), m_frame, "color");
			saveCvMat(buffer, m_imgColor);
			
			// save 3d image
			sprintf(buffer, m_capFormat.c_str(), m_frame, "3d");
			saveCvMat(buffer, m_img3d);

			m_frame++;
		}
		
		// notify the thread to process the next frame
		m_condMutex2.lock();
		m_cond2 = true;
		m_condVar2.notify_all();
		m_condMutex2.unlock();
	}
}
