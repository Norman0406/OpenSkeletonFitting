#pragma once
#include "Exception.h"
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

namespace boost
{
	namespace math
	{
		template <typename T>
		class quaternion;
	}
}

using namespace boost::math;

namespace osf
{
	#define DEG2RAD(a) (a * (M_PI / 180.0))
	#define RAD2DEG(a) ((a * 180.0) / M_PI)

	bool loadCvMat(const char* filename, cv::Mat& image, float* timestamp = 0);
	bool saveCvMat(const char* filename, const cv::Mat& image);
	
	void matrix2Quat(const double* rot, double* quat);
	void matrix2Quat(const cv::Mat& rot, quaternion<double>& quat);
	void quat2Matrix(const double* quat, double* rot);
	void quat2Matrix(const quaternion<double>& quat, cv::Mat& rot);
	void euler2Quat(quaternion<double>& quat, double angleX, double angleY, double angleZ);
	void quat2Euler(const quaternion<double>& quat, double& angleX, double& angleY, double& angleZ);
	void axis2Quat(quaternion<double>& quat, const cv::Point3d& axis, double angle);
	void quat2Axis(const quaternion<double>& quat, cv::Point3d& axis, double& angle);
	void quatRotate(const quaternion<double>& quat, cv::Point3d& point);
	void quatRotateInv(const quaternion<double>& quat, cv::Point3d& point);

	inline bool areSizesEq(const cv::Size& size1, const cv::Size& size2)
	{
		return size1.width == size2.width &&
			size1.height == size2.height;
	}

	inline bool areSizesEq(const cv::Mat& mat1, const cv::Mat& mat2)
	{
		return mat1.cols == mat2.cols &&
			mat1.rows == mat2.rows;
	}
	
	inline float distanceP3f(const cv::Point3f point1, const cv::Point3f point2)
	{
		return sqrtf((point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y) +
			(point2.z - point1.z) * (point2.z - point1.z));
	}

	inline float distanceP2f(const cv::Point2f point1, const cv::Point2f point2)
	{
		return sqrtf((point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y));
	}

	inline float distanceP3fSq(const cv::Point3f point1, const cv::Point3f point2)
	{
		return (point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y) +
			(point2.z - point1.z) * (point2.z - point1.z);
	}

	inline float distanceP2fSq(const cv::Point2f point1, const cv::Point2f point2)
	{
		return (point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y);
	}

	inline double distanceP3d(const cv::Point3d point1, const cv::Point3d point2)
	{
		return sqrt((point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y) +
			(point2.z - point1.z) * (point2.z - point1.z));
	}

	inline double distanceP2d(const cv::Point2d point1, const cv::Point2d point2)
	{
		return sqrt((point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y));
	}

	inline double distanceP3dSq(const cv::Point3d point1, const cv::Point3d point2)
	{
		return (point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y) +
			(point2.z - point1.z) * (point2.z - point1.z);

		// more precise
		/*return (point2.x * point2.x - point2.x * point1.x - point1.x * point2.x + point1.x * point1.x) +
			(point2.y * point2.y - point2.y * point1.y - point1.y * point2.y + point1.y * point1.y) +
			(point2.z * point2.z - point2.z * point1.z - point1.z * point2.z + point1.z * point1.z);*/
	}

	inline double distanceP2dSq(const cv::Point2d point1, const cv::Point2d point2)
	{
		return (point2.x - point1.x) * (point2.x - point1.x) +
			(point2.y - point1.y) * (point2.y - point1.y);
	}

	inline cv::Mat createRotMat2d(double angleRad)
	{
		cv::Mat rotMat(2, 2, CV_64FC1);
		rotMat.ptr<double>(0)[0] = cos(angleRad);
		rotMat.ptr<double>(0)[1] = -sin(angleRad);
		rotMat.ptr<double>(1)[0] = sin(angleRad);
		rotMat.ptr<double>(1)[1] = cos(angleRad);
		return rotMat;
	}

	inline cv::Mat createRotMat3dX(double angleRad)
	{
		cv::Mat rotMat(3, 3, CV_64FC1);
		rotMat.setTo(0);
		rotMat.ptr<double>(0)[0] = 1.0;
		rotMat.ptr<double>(1)[1] = cos(angleRad);
		rotMat.ptr<double>(1)[2] = -sin(angleRad);
		rotMat.ptr<double>(2)[1] = sin(angleRad);
		rotMat.ptr<double>(2)[2] = cos(angleRad);
		return rotMat;
	}

	inline cv::Mat createRotMat3dY(double angleRad)
	{
		cv::Mat rotMat(3, 3, CV_64FC1);
		rotMat.setTo(0);
		rotMat.ptr<double>(0)[0] = cos(angleRad);
		rotMat.ptr<double>(0)[2] = sin(angleRad);
		rotMat.ptr<double>(1)[1] = 1.0;
		rotMat.ptr<double>(2)[0] = -sin(angleRad);
		rotMat.ptr<double>(2)[2] = cos(angleRad);
		return rotMat;
	}

	inline cv::Mat createRotMat3dZ(double angleRad)
	{
		cv::Mat rotMat(3, 3, CV_64FC1);
		rotMat.setTo(0);
		rotMat.ptr<double>(0)[0] = cos(angleRad);
		rotMat.ptr<double>(0)[1] = -sin(angleRad);
		rotMat.ptr<double>(1)[0] = sin(angleRad);
		rotMat.ptr<double>(1)[1] = cos(angleRad);
		rotMat.ptr<double>(2)[2] = 1.0;
		return rotMat;
	}

	inline cv::Mat createRotMat3d(double angleRadX, double angleRadY, double angleRadZ)
	{
		cv::Mat rotX = createRotMat3dX(angleRadX);
		cv::Mat rotY = createRotMat3dY(angleRadY);
		cv::Mat rotZ = createRotMat3dZ(angleRadZ);
	
		return rotX * rotY * rotZ;
	}

	inline void addRotation(cv::Mat& rotMat, double angleRad)
	{
		if (rotMat.cols != 2 || rotMat.rows != 2 ||
			rotMat.type() != CV_32FC1)
			throw Exception("invalid input data");
	
		cv::Mat newMat(2, 2, CV_64FC1);
	
		newMat.ptr<double>(0)[0] = cos(angleRad);
		newMat.ptr<double>(0)[1] = -sin(angleRad);
		newMat.ptr<double>(1)[0] = sin(angleRad);
		newMat.ptr<double>(1)[1] = cos(angleRad);

		rotMat *= newMat;
	}
	
	inline void pointXYZ2UV(const cv::Mat& projMat, const cv::Point3f& src, cv::Point2f& dst)
	{
		if (projMat.cols != 4 || projMat.rows != 3 ||
			projMat.type() != CV_32F || src.z <= 0)
			return;

		cv::Mat srcMat = cv::Mat(4, 1, CV_32FC1);
		cv::Mat dstMat = cv::Mat(3, 1, CV_32FC1);
		
		srcMat.ptr<float>(0)[0] = src.x;
		srcMat.ptr<float>(0)[1] = src.y;
		srcMat.ptr<float>(0)[2] = src.z;
		srcMat.ptr<float>(0)[3] = 1.0f;

		dstMat = projMat * srcMat;

		dst.x = dstMat.ptr<float>(0)[0] / dstMat.ptr<float>(0)[2];
		dst.y = dstMat.ptr<float>(0)[1] / dstMat.ptr<float>(0)[2];
	}
	
	inline void pointXYZ2UV(const cv::Mat& projMat, const cv::Point3d& src, cv::Point2d& dst)
	{
		cv::Point3f srcPt((float)src.x, (float)src.y, (float)src.z);
		cv::Point2f dstPt(0, 0);
		pointXYZ2UV(projMat, srcPt, dstPt);
		dst.x = (double)dstPt.x;
		dst.y = (double)dstPt.y;
	}
	
	inline double distancePoint2Line(const cv::Point3d& point, const cv::Point3d& origin, const cv::Point3d& direction)
	{
		cv::Point3d connection = point - origin;
		double t = connection.dot(direction);
		double dist = 0;
		if (t <= 0.0f) {
			dist = connection.dot(connection);
		} else {
			cv::Point3d Q = origin + (t / direction.dot(direction)) * direction;
			connection = point - Q;
			dist = connection.dot(connection);
		}
		return dist;
	}

	inline void clampAngle(double& angle)
	{
		if (angle >= 360)
			angle = angle - 360;
		else if (angle < 0)
			angle = 360 + angle;
	}
}
