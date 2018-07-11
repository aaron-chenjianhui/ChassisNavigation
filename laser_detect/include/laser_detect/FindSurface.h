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
LineOffset()
{
}

LineOffset(Line2D & init_line, PointsT & init_points, OffsetT& off_type, double off_thre = 1.) :
	m_line(init_line), m_points(init_points), m_off_type(off_type), m_off_thre(off_thre)
{
}

~LineOffset()
{
}


bool Execute(Line2D &line, int max_count = 10)
{
	PointsT outer_points;
	PointsT inner_points;
	VoteT vote;

	for (int i = 0; i < max_count; ++i) {
		SeperatePoints(m_points, m_line, inner_points, outer_points);
		m_points = inner_points;

		FindLine(inner_points, m_line, vote);
		if (Error() < m_off_thre) {
			line = m_line;
			return true;
		}
	}

	line = m_line;
	return false;
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

double Error()
{
	return m_line_detector(m_points, m_line);
}

private:
PointsT m_points;
Line2D m_line;
OffsetT m_off_type;
LineDetector m_line_detector;
double m_off_thre;
};

#endif
