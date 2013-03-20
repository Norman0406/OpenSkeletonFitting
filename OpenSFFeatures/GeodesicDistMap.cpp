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
#include "GeodesicDistMap.h"
#include "../OpenSF/Logging.h"
#include "../OpenSF/Utils.h"

namespace osf
{
	GeodesicDistMap::GeodesicDistMap(NeighborPrecision prec)
		: m_isCreated(false), m_precision(prec), m_centerOfMass(0, 0)
	{
		m_maxZDistThreshold = std::numeric_limits<float>::infinity();
	}

	GeodesicDistMap::GeodesicDistMap(const cv::Mat& imgUser3d, NeighborPrecision prec)
		: m_isCreated(false), m_precision(prec), m_centerOfMass(0, 0)
	{
		create(imgUser3d);
	}

	GeodesicDistMap::~GeodesicDistMap(void)
	{
	}
	
	void GeodesicDistMap::setMaxZDistThreshold(float threshold)
	{
		m_maxZDistThreshold = threshold;
	}

	void GeodesicDistMap::setNeighborPrecision(NeighborPrecision precision)
	{
		if (precision != NP_4 && precision != NP_8) {
			WARN << "unknown neighbor precision: " << precision << ENDL;
			return;
		}

		m_precision = precision;
		
		// recreate edge map if necessary
		if (!m_edges.empty() && !m_imgUser3d->empty()) {
			m_edges = cv::Mat(m_imgUser3d->rows, m_imgUser3d->cols, CV_MAKETYPE(CV_32F, (int)m_precision));
			m_edges.setTo(0);
		}
	}

	float GeodesicDistMap::getMaxZDistThreshold() const
	{
		return m_maxZDistThreshold;
	}

	GeodesicDistMap::NeighborPrecision GeodesicDistMap::getNeighborPrecision() const
	{
		return m_precision;
	}

	void GeodesicDistMap::create(const cv::Mat& imgUser3d)
	{
		if (m_isCreated)
			throw Exception("already created");

		m_imgUser3d = &imgUser3d;
	
		// create user mask image
		createMask(imgUser3d, m_imgUserMask);

		// create distance image
		m_distance = cv::Mat(imgUser3d.rows, imgUser3d.cols, CV_32FC1);
		m_distance.setTo(0);

		// create predecessors map
		m_predecessors = cv::Mat(imgUser3d.rows, imgUser3d.cols, CV_32SC2);
		m_predecessors.setTo(0);

		// create visited image
		m_visited = cv::Mat(imgUser3d.rows, imgUser3d.cols, CV_8SC1);
		m_visited.setTo(0);

		// create 8 channel edge map
		m_edges = cv::Mat(imgUser3d.rows, imgUser3d.cols, CV_MAKETYPE(CV_32F, (int)m_precision));
		m_edges.setTo(0);
		
		m_centerOfMass = cv::Point(0, 0);

		m_isCreated = true;
	}

	bool GeodesicDistMap::isCreated() const
	{
		return m_isCreated;
	}

	void GeodesicDistMap::createMask(const cv::Mat& imgUser3d, cv::Mat& mask)
	{
		if (mask.empty())
			mask = cv::Mat(imgUser3d.rows, imgUser3d.cols, CV_8UC1);

		mask.setTo(0);
		for (int i = 0; i < imgUser3d.cols; i++) {
			for (int j = 0; j < imgUser3d.rows; j++) {
				float valZ = imgUser3d.ptr<float>(j)[i * 3 + 2];
				if (valZ > 0)
					mask.ptr<uchar>(j)[i] = 255;
			}
		}
	}

	void GeodesicDistMap::compute(const cv::Mat& imgUser3d, cv::Mat& imgGeo, cv::Mat& imgPred)
	{
		if (!m_isCreated)
			throw Exception("not yet created");

		if (imgUser3d.type() != CV_32FC3 || imgUser3d.channels() != 3)
			throw Exception("invalid input data");
		
		// create user mask
		createMask(imgUser3d, m_imgUserMask);
		
		// compute center of mass
		cv::Moments moments = cv::moments(m_imgUserMask, true);

		m_distance.setTo(0);
		m_predecessors.setTo(0);

		if (moments.m00 != 0) {
			m_centerOfMass = cv::Point((int)(moments.m10 / moments.m00), (int)(moments.m01 / moments.m00));
	
			if (initialize(m_centerOfMass, imgUser3d)) {
				computeDijkstra();
			}

			// change infinity to 0
			cv::threshold(m_distance, m_distance, std::numeric_limits<float>::max(),
				0, CV_THRESH_TOZERO_INV);
		}
		
		imgGeo = m_distance;
		imgPred = m_predecessors;

		//cv::imshow("Geodesic", m_distance * 0.8);
	}
	
	std::vector<cv::Point> GeodesicDistMap::getNeighbors(const cv::Point& centerPos)
	{
		std::vector<cv::Point> neighbors((int)m_precision);
		switch (m_precision) {
		case NP_4:
			neighbors[0] = centerPos + cv::Point(-1, 0);
			neighbors[1] = centerPos + cv::Point(0, -1);
			neighbors[2] = centerPos + cv::Point(1, 0);
			neighbors[3] = centerPos + cv::Point(0, 1);
			break;
		case NP_8:
			neighbors[0] = centerPos + cv::Point(-1, 0);
			neighbors[1] = centerPos + cv::Point(-1, -1);
			neighbors[2] = centerPos + cv::Point(0, -1);
			neighbors[3] = centerPos + cv::Point(1, -1);
			neighbors[4] = centerPos + cv::Point(1, 0);
			neighbors[5] = centerPos + cv::Point(1, 1);
			neighbors[6] = centerPos + cv::Point(0, 1);
			neighbors[7] = centerPos + cv::Point(-1, 1);
			break;
		}
		return neighbors;
	}

	bool GeodesicDistMap::initialize(const cv::Point& start, const cv::Mat& imgUser3d)
	{
		if (start.x < 0 || start.x >= imgUser3d.cols ||
			start.y < 0 || start.y >= imgUser3d.rows)
			return false;

		// should not happen
		while (!m_queue.empty())
			m_queue.pop();

		m_imgUser3d = &imgUser3d;
	
		m_distance.setTo(std::numeric_limits<float>::infinity());
		m_predecessors.setTo(0);
		
		// init m_visited
		m_visited.setTo(0);
		bool foundStart = false;
		for (int i = 0; i < m_imgUser3d->cols; i++) {
			for (int j = 0; j < m_imgUser3d->rows; j++) {
				float valUserZ = m_imgUser3d->ptr<float>(j)[i * 3 + 2];

				if (valUserZ > 0)
					m_visited.ptr<char>(j)[i] = 1;	// 0: invalid, 1: not yet visited, 2: visited
			}
		}
		
		// NOTE: A problem might arise, if the center of gravity is outside the object [11/28/2011 Norman]

		// push start position to queue
		Elem* elem = new Elem;
		elem->distance = 0;
		elem->pos = start;
		m_queue.push(elem);

		// set predecessor of start position
		m_predecessors.ptr<int>(start.y)[start.x * 2 + 0] = start.x;
		m_predecessors.ptr<int>(start.y)[start.x * 2 + 1] = start.y;

		m_edges.setTo(0);		
		
		return true;
	}

	void GeodesicDistMap::computeDijkstra()
	{
		createEdgeMap();
		runDijkstra();
	}
	
	void GeodesicDistMap::createEdgeMap()
	{
		// create neighbors map with correct weights
		for (int i = 0; i < m_imgUser3d->cols; i++) {
			for (int j = 0; j < m_imgUser3d->rows; j++) {
				float valUserZ = m_imgUser3d->ptr<float>(j)[i * 3 + 2];

				if (valUserZ > 0) {
					cv::Point pos(i, j);
					
					// center position in world
					cv::Point3f posWorld = cv::Point3f(m_imgUser3d->ptr<float>(pos.y)[pos.x * 3 + 0],
						m_imgUser3d->ptr<float>(pos.y)[pos.x * 3 + 1],
						m_imgUser3d->ptr<float>(pos.y)[pos.x * 3 + 2]);

					// neighbors in clockwise order, beginning with most left neighbor
					std::vector<cv::Point> neighbors = getNeighbors(pos);
					if (m_edges.channels() != neighbors.size())
						throw Exception("invalid channel count");

					// fill in edge data structure with appropriate edge weights (distances)
					int channel = 0;
					for (std::vector<cv::Point>::const_iterator it = neighbors.begin();
							it != neighbors.end(); it++, channel++) {
						float* edgeWeight = &m_edges.ptr<float>(j)[i * m_edges.channels() + channel];
						
						// mark as invalid if necessary
						if ((*it).x < 0 || (*it).x >= m_imgUser3d->cols ||
							(*it).y < 0 || (*it).y >= m_imgUser3d->rows ||
							m_imgUser3d->ptr<float>((*it).y)[(*it).x * 3 + 2] == 0) {
								*edgeWeight = -2;
								continue;
						}

						// neighbor position in world
						cv::Point3f nPosWorld = cv::Point3f(m_imgUser3d->ptr<float>((*it).y)[(*it).x * 3 + 0],
							m_imgUser3d->ptr<float>((*it).y)[(*it).x * 3 + 1],
							m_imgUser3d->ptr<float>((*it).y)[(*it).x * 3 + 2]);

						*edgeWeight = distanceP3f(posWorld, nPosWorld);

						// remove edge if z-distance is above threshold
						if (fabs(nPosWorld.z - posWorld.z) > m_maxZDistThreshold)
							*edgeWeight = -1;
					}
				}
			}
		}
	}
	
	void GeodesicDistMap::runDijkstra()
	{
		// run dijkstra's algorithm on edge map
		while (!m_queue.empty()) {
			Elem* topElem = m_queue.top();
			m_queue.pop();

			float topDist = topElem->distance;
			cv::Point topPos = topElem->pos;
			delete topElem;

			char* visited = &m_visited.ptr<char>(topPos.y)[topPos.x];
			if (*visited == 1) {
				*visited = 2;

				float* valDist = &m_distance.ptr<float>(topPos.y)[topPos.x];
				*valDist = topDist;

				std::vector<cv::Point> neighbors = getNeighbors(topPos);

				for (int i = 0; i < (int)m_precision; i++) {
					float edgeWeight = m_edges.ptr<float>(topPos.y)[topPos.x * m_edges.channels() + i];
					
					if (edgeWeight > 0) {
						processEdge(topPos, topDist, neighbors[i], edgeWeight);
					}
				}
			}
		}
	}

	void GeodesicDistMap::processEdge(const cv::Point& topPos, float topDist, const cv::Point& nPos, float edgeWeight)
	{
		float altDist = topDist + edgeWeight;
		float* valDistN = &m_distance.ptr<float>(nPos.y)[nPos.x];

		if (altDist < *valDistN) {
			Elem* elem = new Elem;
			elem->distance = altDist;
			elem->pos = nPos;
			m_queue.push(elem);

			// store predecessor
			m_predecessors.ptr<int>(nPos.y)[nPos.x * 2 + 0] = topPos.x;
			m_predecessors.ptr<int>(nPos.y)[nPos.x * 2 + 1] = topPos.y;

			*valDistN = altDist;
		}
	}
}
