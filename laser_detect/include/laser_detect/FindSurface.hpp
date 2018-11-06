#ifndef _FINDSURFACE_H
#define _FINDSURFACE_H

#include <vector>
#include <string>

#include "Line2D.hpp"
#include "Point2D.h"
#include "LineDetector.h"

class LineOffset {
public:
typedef std::vector<Point2D> PointsT;
typedef std::vector<bool> VoteT;
typedef std::string OffsetT;

public:
LineOffset(double off_thre = 1.) : m_off_thre(off_thre)
{
}

~LineOffset()
{
}


Line2D Execute(const PointsT points, const Line2D& line, int max_count = 10)
{
	PointsT all_points = points;
	PointsT outer_points;
	PointsT inner_points; VoteT vote;
	Line2D off_line = line;

	DEBUGLOG("begin to execute");
	for (int i = 0; i < max_count; ++i) {
		DEBUGLOG("step" << i);

		SeperatePoints(all_points, off_line, inner_points, outer_points);
		all_points = inner_points;

		FindLine(inner_points, off_line, vote);
		double err = Error(inner_points, off_line);
        auto len = inner_points.size();

		if (err < m_off_thre) {
			DEBUGLOG("find offset line");
			return off_line;
		}
	}

	off_line = line;
	return off_line;
}


private:

/**
 * Seperate points for inner points and outer points.
 * Inner points are judged by: dot(p-a, n)*dot(0-a, n) >= 0
 * Outer points are judged by: dot(p-a, n)*dot(0-a, n) < 0
 * @param points_input [description]
 * @param line         [description]
 * @param points_inner [description]
 * @param points_outer [description]
 */
void SeperatePoints(const PointsT& points_input, const Line2D& line,
		    PointsT& points_inner, PointsT& points_outer)
{
	// points_inner is reused
	points_inner.clear();

	PointsT::const_iterator iter = points_input.begin();

	for (; iter != points_input.end(); ++iter) {
		Point2D a = line.PointInLine();
		Point2D n = line.Normal();

		if (Line2D::DotProduct(*iter - a, n) * Line2D::DotProduct(-a, n) >= 0)
			points_inner.push_back(*iter);
		else
			points_outer.push_back(*iter);
	}
}

void FindLine(const PointsT& points, Line2D& line, VoteT& vote)
{
	m_line_detector.FindLine(points, line, vote);
}

double Error(const PointsT& points, const Line2D& line)
{
	// check container before dereference
	if (points.empty()) {
		return 100.;
	}

	double sum_dist = 0.;
	PointsT::const_iterator iter = points.begin();
	for (; iter != points.end(); ++iter) {
		double val = line.PointDist(*iter);
		sum_dist += val;
	}

	return sum_dist / points.size();
}

private:
PointsT m_points;
Line2D m_line;
OffsetT m_off_type;
LineDetector m_line_detector;
double m_off_thre;
};

class FindSurface : public LineOffset {
public:
FindSurface()
{
}

Line2D GetSurface(const LaserData& laser_data, const Line2D& line)
{
	PointsT points = laser_data.ScanToPoint();
	Line2D surf_line = Execute(points, line);

	return surf_line;
}
};



#endif
