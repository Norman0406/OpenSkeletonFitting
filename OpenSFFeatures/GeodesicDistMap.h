#pragma once
#include <opencv2/opencv.hpp>
#include <queue>

namespace osf
{
	class GeodesicDistMap
	{
	public:
		enum NeighborPrecision
		{
			NP_4 = 4,	// 4 neighbors in cross neighborhood
			NP_8 = 8,	// 8 neighbors in full neighborhood
		};

		GeodesicDistMap(NeighborPrecision prec = NP_8);
		GeodesicDistMap(const cv::Mat&, NeighborPrecision prec = NP_8);
		~GeodesicDistMap(void);

		void create(const cv::Mat&);
		bool isCreated() const;
		void compute(const cv::Mat& imgUser3d, cv::Mat&, cv::Mat&);

		// parameters
		void setMaxZDistThreshold(float);
		void setNeighborPrecision(NeighborPrecision);
		float getMaxZDistThreshold() const;
		NeighborPrecision getNeighborPrecision() const;

	private:
		void createMask(const cv::Mat&, cv::Mat&);
		bool initialize(const cv::Point& start, const cv::Mat& img3d);
		void computeDijkstra();
		void createEdgeMap();
		void runDijkstra();
		void processEdge(const cv::Point&, float, const cv::Point&, float);
		std::vector<cv::Point> getNeighbors(const cv::Point&);
	
		bool m_isCreated;
		const cv::Mat* m_imgUser3d;
		cv::Mat m_imgUserMask;
		cv::Point m_centerOfMass;
		
		struct Elem {
			cv::Point pos;
			float distance;
		};

		class ElemComparator {
		public:
			bool operator() (const Elem* el1, const Elem* el2) const {
				return el1->distance > el2->distance;
			}
		};

		cv::Mat m_edges;	// 4/8 channel edge map, beginning from left neighbor (-1, 0) and proceeding clockwise
		cv::Mat m_distance;	// stores distance values
		cv::Mat m_predecessors;	// stores predecessors for each node
		cv::Mat m_visited;	// information about visited nodes
		std::priority_queue<Elem*, std::vector<Elem*>, ElemComparator> m_queue;	// priority queue
		
		// parameters
		float m_maxZDistThreshold;
		NeighborPrecision m_precision;
	};
}