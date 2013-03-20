#include <iostream>
#include <time.h>
#include "Visualization3d.h"
#include "../OpenSFInput/Input.h"
#include "../OpenSFSegmentation/Segmentation.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"
#include "../OpenSFFeatures/FeatureTracking.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSF/Exception.h"
#include "../OpenSF/Utils.h"
#include <GL/glfw.h>

#ifdef __APPLE__ & __MACH__
#include <OpenGL/gl.h>
#endif

namespace osf
{	
	Visualization3d::Visualization3d(Input* input, Segmentation* seg, Features* feat, Fitting* fit,
		cv::Size windowSize)
		: m_input(input), m_segmentation(seg), m_features(feat), m_fitting(fit),
		m_windowSize(windowSize)
	{
		m_imgDepth = 0;
		m_img3d = 0;
		m_imgColor = 0;
		m_imgSegDepth = 0;
		m_imgSeg3d = 0;
		m_imgGeo = 0;
		m_imgPred = 0;
		m_vecFeatures = 0;
		m_rootJoint = 0;
		m_isInit = false;	
	
		m_camPos = cv::Point3f(0, 0, 0);
		m_camDir = cv::Point3f(0, 0, 1);
		m_camUp = cv::Point3f(0, 1, 0);
		m_camLookAt = cv::Point3f(0, 0, 0);
		m_theta = -90;
		m_phi = 0;
		m_radius = 2.5f;
		m_lastMousePos = cv::Point(0, 0);
		m_lastMouseWheel = 0;
		m_displayList = 0;
		m_fixedLookAt = true;
		m_spacePressed = false;
		m_enterPressed = false;

		if (!input)
			throw Exception("input invalid");
	}

	Visualization3d::~Visualization3d(void)
	{
		glDeleteLists(m_displayList, 1);
		glfwCloseWindow();
		glfwTerminate();
	}

	void Visualization3d::init()
	{
		if (m_isInit) {
			WARN << "already init" << ENDL;
			return;
		}

		// init input vis
		m_imgDepth = &m_input->getImgDepth();
		m_img3d = &m_input->getImg3d();
		m_imgColor = &m_input->getImgColor();

		// init segmentation vis
		if (m_segmentation) {
			m_imgSegDepth = &m_segmentation->getImgSegDepth();
			m_imgSeg3d = &m_segmentation->getImgSeg3d();
		}

		// init feature vis
		if (m_features) {
			m_imgGeo = &m_features->getImgGeodesic();
			m_imgPred = &m_features->getImgPredecessors();
			m_vecFeatures = &m_features->getTrackedFeatures();
		}

		// init fitting vis
		if (m_fitting) {
			m_rootJoint = m_fitting->getRootJoint();
			if (!m_rootJoint)
				throw Exception("invalid root joint");
		}

		// init OpenGL
		glfwInit();
		glfwOpenWindow(m_windowSize.width, m_windowSize.height, 8, 8, 8, 0, 16, 0, GLFW_WINDOW);
		glfwSetWindowTitle("Skeleton 3D");

		glEnable(GL_DEPTH_TEST);
		
		// create a display list containing a box
		m_displayList = glGenLists(1);
		if (!m_displayList)
			throw Exception("could not create display list");
		glNewList(m_displayList, GL_COMPILE);

		glBegin(GL_QUADS);
		glVertex3f(-1.0f, -1.0f, 1.0f);
		glVertex3f(1.0, -1.0f, 1.0f);
		glVertex3f(1.0f, 1.0f, 1.0f);
		glVertex3f(-1.0f, 1.0f, 1.0f);

		glVertex3f(-1.0f, -1.0f, -1.0f);
		glVertex3f(1.0, -1.0f, -1.0f);
		glVertex3f(1.0f, 1.0f, -1.0f);
		glVertex3f(-1.0f, 1.0f, -1.0f);

		glVertex3f(-1.0f, -1.0f, -1.0f);
		glVertex3f(1.0, -1.0f, -1.0f);
		glVertex3f(1.0f, -1.0f, 1.0f);
		glVertex3f(-1.0f, -1.0f, 1.0f);

		glVertex3f(-1.0f, 1.0f, -1.0f);
		glVertex3f(1.0, 1.0f, -1.0f);
		glVertex3f(1.0f, 1.0f, 1.0f);
		glVertex3f(-1.0f, 1.0f, 1.0f);
				
		glVertex3f(-1.0f, -1.0f, -1.0f);
		glVertex3f(-1.0, 1.0f, -1.0f);
		glVertex3f(-1.0f, 1.0f, 1.0f);
		glVertex3f(-1.0f, -1.0f, 1.0f);
		
		glVertex3f(1.0f, -1.0f, -1.0f);
		glVertex3f(1.0, 1.0f, -1.0f);
		glVertex3f(1.0f, 1.0f, 1.0f);
		glVertex3f(1.0f, -1.0f, 1.0f);
		glEnd();

		glEndList();

		// init random color list
		const int labelSize = 256;
		m_labelColors.resize(labelSize);
		srand((int)time(NULL));
		for (int i = 0; i < (int)m_labelColors.size(); i++) {
			m_labelColors[i] = cv::Point3f(rand() / (float)RAND_MAX,
				rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
		}

		m_isInit = true;
	}
	
	bool Visualization3d::draw(bool& paused, bool& step)
	{
		if (!m_isInit) {
			WARN << "not init" << ENDL;
			return false;
		}

		// process mouse movement
		processMouse();

		// render scene
		render();		
		
		glfwPollEvents();
		
		// process keyboard
		if (glfwGetKey(GLFW_KEY_SPACE) == GLFW_PRESS && !m_spacePressed) {
			m_spacePressed = true;
			if (!paused)
				paused = true;
			else
				paused = false;
		}
		else if (glfwGetKey(GLFW_KEY_SPACE) == GLFW_RELEASE && m_spacePressed)
			m_spacePressed = false;
		
		if (glfwGetKey(GLFW_KEY_ENTER) == GLFW_PRESS && !m_enterPressed) {
			m_enterPressed = true;
			step = true;
		}
		else if (glfwGetKey(GLFW_KEY_ENTER) == GLFW_RELEASE && m_enterPressed)
			m_enterPressed = false;
		
		if (glfwGetKey(GLFW_KEY_ESC) == GLFW_PRESS)
			return true;

		return false;
	}

	void Visualization3d::render()
	{
		if (m_fixedLookAt) {
			if (m_rootJoint)
				m_camLookAt = m_rootJoint->getPos3d();
			else if (m_vecFeatures) {
				for (int i = 0; i < (int)m_vecFeatures->size(); i++) {
					if (m_vecFeatures->at(i)->getJointLabel() == JT_TORSO) {
						m_camLookAt = m_vecFeatures->at(i)->getPosition3d();
						break;
					}
				}
			}
		}

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		
		// Reset transformations
		glLoadIdentity();

		// rotate coordinate system
		glRotatef(180, 0, 0, 1);

		// Set the camera
		gluPerspective(50.0, (float)m_windowSize.width / (float)m_windowSize.height, 0.1, 1000.0);

		gluLookAt(m_camPos.x, m_camPos.y, m_camPos.z,
			m_camLookAt.x, m_camLookAt.y, m_camLookAt.z,
			m_camUp.x, m_camUp.y, m_camUp.z);
		
		// render look at position
		renderLookAt();
	
		// render coordinate center
		glPushMatrix();
		glTranslatef(0, 0, 0);
		renderOrigin();
		glPopMatrix();
		
		// render scene
		glPushMatrix();
		renderScene();
		glPopMatrix();
		
		glPopMatrix();
		glfwSwapBuffers();
	}
	
	void Visualization3d::processMouse()
	{
		int x = 0, y = 0;
		glfwGetMousePos(&x, &y);
		
		// store last position for mouse speed calculation
		if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_1) == GLFW_PRESS ||
			glfwGetMouseButton(GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) {
				if (m_lastMousePos.x == 0 && m_lastMousePos.y == 0)
					m_lastMousePos = cv::Point(x, y);
		}
		else if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_1) == GLFW_RELEASE &&
			glfwGetMouseButton(GLFW_MOUSE_BUTTON_2) == GLFW_RELEASE) {
				m_lastMousePos = cv::Point(0, 0);
		}

		// rotate on left click
		if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
			m_theta -= (x - m_lastMousePos.x) * 1.0f;
			m_phi -= (y - m_lastMousePos.y) * 1.0f;
		}

		// move on right click
		if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) {
			if (m_fixedLookAt)
				m_fixedLookAt = false;

			cv::Point3f dirVec = m_camPos - m_camLookAt;
			cv::Point3f rightVec = m_camUp.cross(dirVec);
			rightVec *= 1.0f / cv::norm(rightVec);
			cv::Point3f upVec = m_camUp;
			upVec *= 1.0f / cv::norm(upVec);

			cv::Point2f movement((float)m_lastMousePos.x - x, (float)y - m_lastMousePos.y);
			movement.x *= m_radius / m_windowSize.width;
			movement.y *= m_radius / m_windowSize.height;
		
			cv::Point3f newRight = rightVec * movement.x;
			cv::Point3f newUp = upVec * movement.y;

			m_camLookAt -= (newRight + newUp);
		}

		// zoom with mouse wheel
		int mouseWheel = glfwGetMouseWheel();

		int newVal = mouseWheel - m_lastMouseWheel;
		m_radius -= (newVal / 10.0f);

		if (m_radius < 0.1f)
			m_radius = 0.1f;
		else if (m_radius > 15.0f)
			m_radius = 15.0f;

		m_lastMouseWheel = mouseWheel;

		// rotate view around look at position
		rotateCameraSphere(m_theta, m_phi);

		if (m_lastMousePos.x != 0 && m_lastMousePos.y != 0)
			m_lastMousePos = cv::Point(x, y);
	}

	void Visualization3d::rotateCameraSphere(float theta, float phi)
	{
		// get camera position on sphere around m_camLookAt
		m_camPos.x = m_camLookAt.x + m_radius * (float)(cos(DEG2RAD(theta)) * cos(DEG2RAD(m_phi)));
		m_camPos.y = m_camLookAt.y + m_radius * (float)(sin(DEG2RAD(phi)));
		m_camPos.z = m_camLookAt.z + m_radius * (float)(sin(DEG2RAD(theta)) * cos(DEG2RAD(phi)));

		// approximate up vector
		float dt = 0.001f;
		float eyeXtemp = m_camLookAt.x + m_radius * (float)(cos(DEG2RAD(theta)) * cos(DEG2RAD(phi) + dt));
		float eyeYtemp = m_camLookAt.y + m_radius * (float)(sin(DEG2RAD(phi) + dt));
		float eyeZtemp = m_camLookAt.z + m_radius * (float)(sin(DEG2RAD(theta)) * cos(DEG2RAD(phi) + dt));

		m_camUp.x = eyeXtemp - m_camPos.x;
		m_camUp.y = eyeYtemp - m_camPos.y;
		m_camUp.z = eyeZtemp - m_camPos.z;
	}

	void Visualization3d::renderSphere(float r, int lats, int longs)
	{
		int i, j;
		for (i = 0; i <= lats; i++) {
			float lat0 = (float)(M_PI * (-0.5 + (float)(i - 1) / lats));
			float z0  = sinf(lat0) * r;
			float zr0 =  cosf(lat0) * r;

			float lat1 = (float)(M_PI * (-0.5 + (float)i / lats));
			float z1 = sinf(lat1) * r;
			float zr1 = cosf(lat1) * r;

			glBegin(GL_QUAD_STRIP);
			for (j = 0; j <= longs; j++) {
				float lng = (float)(2 * M_PI * (float)(j - 1) / longs);
				float x = cosf(lng);
				float y = sinf(lng);

				glNormal3f(x * zr0, y * zr0, z0);
				glVertex3f(x * zr0, y * zr0, z0);
				glNormal3f(x * zr1, y * zr1, z1);
				glVertex3f(x * zr1, y * zr1, z1);
			}
			glEnd();
		}
	}

	void Visualization3d::renderLookAt()
	{
		glPushMatrix();
		glTranslatef(m_camLookAt.x, m_camLookAt.y, m_camLookAt.z);
		glScalef(0.05f, 0.05f, 0.05f);
		glColor3f(0, 1, 1);
		glLineWidth(1.0);

		glBegin(GL_LINES);
		glVertex3f(-1, 0, 0);
		glVertex3f(1, 0, 0);
		glEnd();
	
		glBegin(GL_LINES);
		glVertex3f(0, -1, 0);
		glVertex3f(0, 1, 0);
		glEnd();
	
		glBegin(GL_LINES);
		glVertex3f(0, 0, -1);
		glVertex3f(0, 0, 1);
		glEnd();

		renderSphere(0.5f, 10, 10);

		glPopMatrix();
	}

	void Visualization3d::renderOrigin(float lineWidth)
	{
		glLineWidth(lineWidth);

		// x axis
		glColor3f(1, 0, 0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);
		glEnd();
	
		// y axis
		glColor3f(0, 1, 0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);
		glEnd();
	
		// z axis
		glColor3f(0, 0, 1);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);
		glEnd();
	}

	void Visualization3d::renderScene()
	{
		// draw 3d image
		const bool drawColor = true;
		if (m_img3d) {
			const float depthScale = 1.0f / 5.0f;
			int step = m_img3d->cols / 120;
			step = step < 1 ? 1 : step;
			const float scale3d = (0.5f * step) / (float)m_img3d->cols;

			for (int i = 0; i < m_img3d->cols; i+=step) {
				for (int j = 0; j < m_img3d->rows; j+=step) {
					const float* row3d = m_img3d->ptr<float>(j);
					float depth = m_imgDepth->ptr<float>(j)[i];
					
					const float relScale = scale3d * depth;
					
					// set color to draw
					float color[3] = {0, 0, 0};
					if (drawColor && m_imgColor) {
						const uchar* rowColor = m_imgColor->ptr<uchar>(j);
						color[0] = rowColor[i * 3 + 2] / 255.0f;
						color[1] = rowColor[i * 3 + 1] / 255.0f;
						color[2] = rowColor[i * 3 + 0] / 255.0f;
					}
					else {
						color[0] = depth * 0.2f;
						color[1] = depth * 0.2f;
						color[2] = depth * 0.2f;
					}
					
					// draw background image darker
					if (m_imgSeg3d) {
						const float* rowUser3d = m_imgSeg3d->ptr<float>(j);
						if (rowUser3d[i * 3 + 2] == 0) {
							const float factor = 0.5f;
							color[0] *= factor;
							color[1] *= factor;
							color[2] *= factor;
						}
					}
					
					cv::Point3d point;
					point.x = row3d[i * 3 + 0];
					point.y = row3d[i * 3 + 1];
					point.z = row3d[i * 3 + 2];
					
					// draw box at sample position
					if (depth > 0) {
						depth *= depthScale;
						glPushMatrix();
						glTranslated(point.x, point.y, point.z);
						glScalef(relScale, relScale, relScale);
						glColor3f(color[0], color[1], color[2]);
						glCallList(m_displayList);
						glPopMatrix();
					}
				}
			}
		}

		
		// skeleton has to be always visible
		glDisable(GL_DEPTH_TEST);
		
		// draw tracked feature points
		if (m_features) {
			std::vector<int> list;
			m_features->getFeatureTracking()->getLabelList(list);
			
			for (int i = 0; i < (int)list.size(); i++) {
				const FeaturePoint* ftPoint = m_features->getFeatureTracking()->getPointByLabel(list[i]);

				if (ftPoint) {
					cv::Point3f color = m_labelColors[ftPoint->getTrackingLabel() % (int)m_labelColors.size()];
					cv::Point3d position = ftPoint->getPosition3dFiltered();
					float size = 0.04f;
					if (!ftPoint->isConfirmed())
						size = 0.03f;

					glPushMatrix();
					glTranslated(position.x, position.y, position.z);
					glColor3f(color.x, color.y, color.z);
					renderSphere(size, 10, 10);
					glPopMatrix();
				}
			}
		}
		/*
		// draw feature points
		if (m_vecFeatures) {
			for (int i = 0; i < (int)m_vecFeatures->size(); i++) {
				FeaturePoint* ftPoint = m_vecFeatures->at(i);

				glPushMatrix();
				glTranslated(ftPoint->getPosition3d().x, ftPoint->getPosition3d().y, ftPoint->getPosition3d().z);
			
				// give prelabeled features a different color
				if (ftPoint->getJointLabel() != JT_UNKNOWN)
					glColor3f(1, 1, 0);
				else
					glColor3f(0, 0, 1);

				gluSphere(m_sphere, 0.015f, 10, 10);
				glPopMatrix();
			}
		}*/

		if (m_rootJoint && m_fitting->isTracking()) {
			renderBone(m_rootJoint, (float)m_rootJoint->getBoneScaleFactor());
		}
		
		glEnable(GL_DEPTH_TEST);
	}

	void Visualization3d::renderBone(Joint* joint, float sizeFac)
	{		
		// draw sub joints
		const std::vector<Joint*>& subJoints = joint->getSubJoints();
		for (int i = 0; i < (int)subJoints.size(); i++)
			renderBone(subJoints[i], sizeFac);
		
		cv::Point3d srcPos = joint->getPos3d();
		
		// draw bone
		if (joint->getClass() != JC_ENDCONNECTOR && joint->getClass() != JC_CONNECTOR) {
			Joint2Joint* src = (Joint2Joint*)joint;
			Joint* dst = src->getDestination();

			cv::Point3d dstPos = dst->getPos3d();

			// compute line thickness
			cv::Point3d eyePos = m_camPos;
			double distToEye = cv::norm((srcPos + ((dstPos - srcPos) * 0.5)) - eyePos);
			distToEye = distToEye < 0.1 ? 0.1 : distToEye;	// must never get <= 0
			float lineWidth = 8.0f / (float)distToEye;
		
			// connect joints with a white line
			glLineWidth(lineWidth * sizeFac);
			glColor3f(1, 1, 1);
			glBegin(GL_LINES);
			glVertex3d(srcPos.x, srcPos.y, srcPos.z);
			glVertex3d(dstPos.x, dstPos.y, dstPos.z);
			glEnd();
		
			// get global euler rotation angles
			cv::Point3d angles = src->getGlobalQuatAngles();

			// draw global coordinate system for joints
			if (!src->isSubJoint()) {
				cv::Point3d posToDraw = src->getPos3d();
				glPushMatrix();
				glTranslated(posToDraw.x, posToDraw.y, posToDraw.z);
				glRotated(-angles.x, 1, 0, 0);
				glRotated(-angles.y, 0, 1, 0);
				glRotated(-angles.z, 0, 0, 1);
				glScalef(0.1f, 0.1f, 0.1f);
				renderOrigin(2.0f);
				glPopMatrix();
			}

			renderBone(dst, sizeFac);
		}

		// draw joint position
		if (!joint->isSubJoint()) {
			glPushMatrix();
			glTranslated(srcPos.x, srcPos.y, srcPos.z);
			if (joint->getClass() == JC_ENDCONNECTOR) {
				glColor3f(1, 0, 0);
				renderSphere(sizeFac / 40.0f, 10, 10);
			}
			else {
				glColor3f(1, 1, 1);
				renderSphere(sizeFac / 50.0f, 10, 10);
			}
			glPopMatrix();
		}
	}
}
