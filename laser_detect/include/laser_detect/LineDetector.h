#ifndef _LINE_DETECTOR_H
#define _LINE_DETECTOR_H

#include "Line2D.hpp"
#include "Point2D.h"
#include "LaserDataType.hpp"

#include "LineParamEstimator.h"
#include "RANSAC.h"



class LineDetector {
public:
typedef std::vector<Point2D> PointsT;
typedef std::vector<bool> VoteT;
typedef std::vector<double> LineParamT;

public:
LineDetector(double delta = 40, double probability = 0.999) : m_estimator(delta), m_probability(probability)
{
}

void FindLine(const PointsT& points, Line2D& line, VoteT& vote)
{
	LineParamT line_param;
	double used_data = RANSAC<Point2D, double>::compute(line_param, &m_estimator,
							    const_cast<PointsT &>(points), m_probability, vote);
    // Line Parameter
    line.m_nx = line_param[0];
    line.m_ny = line_param[1];
    line.m_ax = line_param[2];
    line.m_ay = line_param[3];

}

/**
 *
 * @param  points [description]
 * @param  line   [description]
 * @return        [description]
 */
double Error(const PointsT& points, const Line2D& line)
{
	double sum = 0;

	PointsT::const_iterator iter = points.begin();

	for (; iter != points.end(); ++iter) {
		double err = Line2D::DotProduct(*iter - line.PointInLine(), line.Normal());
		sum += fabs(err);
	}

	return sum / (points.size());
}

private:
LineParamEstimator m_estimator;
double m_probability;
};

class LaserLineDetector : public LineDetector {
public:
void LaserFindLine(LaserData& raw_laser_data, Line2D& line, LaserData& select_laser_data)
{
	VoteT vote;

	FindLine(raw_laser_data.ScanToPoint(), line, vote);

	LaserData::AngSeqT ang_voted;
	LaserData::AngSeqT ang_raw = raw_laser_data.GetAngSeq();
	LaserData::AngSeqT::iterator iter = ang_raw.begin();
	VoteT::iterator vote_iter = vote.begin();

	for (; iter != ang_raw.end(); ++iter, ++vote_iter) {
		if (*vote_iter)
			ang_voted.push_back(*iter);
	}

	raw_laser_data.DataVoted(ang_voted, select_laser_data);
}
};


#endif
