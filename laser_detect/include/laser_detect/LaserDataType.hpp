#ifndef _LASER_DATA_TYPE_HPP
#define _LASER_DATA_TYPE_HPP

#include <map>
#include <iterator>
#include <algorithm>

#include "laser_msgs/UnitConvert.h"
#include "laser_msgs/DebugLog.hpp"

#include "Point2D.h"

class LaserRange {
public:
void InsertData(double min_angle, double max_angle)
{
	if (min_angle < max_angle) {
		m_min = min_angle;
		m_max = max_angle;
	} else {
		m_min = max_angle;
		m_max = min_angle;
	}
}

double MinAng()
{
	return m_min;
}

double MaxAng()
{
	return m_max;
}

private:
double m_min;
double m_max;
};

struct LaserLimit {
	double	ang_limit_min = DEG2RAD(-135);
	double	ang_limit_max = DEG2RAD(135);
	double	data_limit_min = M2MM(0.8);
	double	data_limit_max = M2MM(10);
};


class DoubleFinder {
public:
typedef std::map<double, double> LaserDataT;
typedef std::pair<const double, double> LaserPointT;

public:
DoubleFinder(double comp_val, double thre_val = 0.0001) :
	m_comp_val(comp_val),
	m_thre_val(thre_val)
{
}

bool operator ()(const LaserDataT::value_type &it)
{
	return fabs(it.first - m_comp_val) < m_thre_val;
}
private:
double m_comp_val;
double m_thre_val;
};


class LaserData {
public:
typedef std::map<double, double> LaserDataT;
typedef std::map<double, double> LaserIntensitiesT;
typedef std::pair<const double, double> LaserPointT;
typedef std::vector<double> AngSeqT;
typedef std::vector<double> IntenSeqT;
typedef std::vector<double> RangeSeqT;

typedef std::vector<Point2D> PointsT;
typedef std::vector<bool> VoteT;


public:

void InsertData(LaserDataT::const_iterator& iter_low, LaserDataT::const_iterator& iter_up)
{
	m_laser_data.insert(iter_low, iter_up);
}

void InsertData(const LaserPointT& single_data)
{
	m_laser_data.insert(single_data);
}


// virtual void UpdateData();
void EmptyData()
{
	m_laser_data.clear();
}

PointsT ScanToPoint() const
{
	Point2D point;
	PointsT points;

	LaserDataT::const_iterator it = m_laser_data.begin();

	for (; it != m_laser_data.end(); ++it) {
		point.x = (it->second) * cos(it->first);
		point.y = (it->second) * sin(it->first);

		points.push_back(point);
	}

	return points;
}


bool DataInRange(double		min_angle,
		 double		max_angle,
		 LaserData&	lidar_data_out) const
{
	if (min_angle > max_angle) {
		ROS_INFO_STREAM("min_angle is bigger than max_angle");
		return false;
	}
	if (min_angle < m_laser_limit.ang_limit_min ||
	    max_angle > m_laser_limit.ang_limit_max) {
		ROS_INFO_STREAM("select angular range is out of angular limitation");
		return false;
	}
	LaserDataT::const_iterator iter_low = m_laser_data.lower_bound(min_angle);
	LaserDataT::const_iterator iter_up = m_laser_data.upper_bound(max_angle);

	lidar_data_out.EmptyData();
	lidar_data_out.InsertData(iter_low, iter_up);

	return true;
}

void DataEliminated(const AngSeqT&	angle_seq,
		    LaserData&		laser_data)
{
	laser_data.EmptyData();

	// Sort angle sequence, in order to simplify the calculation
	AngSeqT ang_seq = angle_seq;
	std::sort(ang_seq.begin(), ang_seq.end());

	LaserDataT::iterator iter_front = m_laser_data.begin();

	AngSeqT::iterator ang_iter = ang_seq.begin();
	for (; ang_iter != ang_seq.end(); ++ang_iter) {
		LaserDataT::iterator iter_find = std::find_if(iter_front, m_laser_data.end()
							      , DoubleFinder(*ang_iter));
		if (iter_find == m_laser_data.end()) {
			DEBUGLOG("Ignored angle is not found");
			return;
		} else {
			// erase data and move iterator to the new position
			iter_front = m_laser_data.erase(iter_find);
		}
	}
}

void DataVoted(const AngSeqT& angle_seq, LaserData& laser_data)
{
	laser_data.EmptyData();

	AngSeqT ang_seq = angle_seq;
	std::sort(ang_seq.begin(), ang_seq.end());

	LaserDataT::iterator iter_front = m_laser_data.begin();

	AngSeqT::iterator ang_iter = ang_seq.begin();
	for (; ang_iter != ang_seq.end(); ++ang_iter) {
		LaserDataT::iterator iter_find = std::find_if(iter_front, m_laser_data.end()
							      , DoubleFinder(*ang_iter));
		if (iter_find == m_laser_data.end()) {
			DEBUGLOG("Voted angle is not found");
			return;
		} else {
			laser_data.InsertData(*iter_find);
		}
	}
}


void LimitLaserData(const std::initializer_list<LaserLimit>& list)
{
	for (auto it = list.begin(); it != list.end(); ++it) {
		double ang_min = it->ang_limit_min;
		double ang_max = it->ang_limit_max;
		double data_min = it->data_limit_min;
		double data_max = it->data_limit_max;

		LaserDataT::iterator iter = m_laser_data.begin();
		for (; iter != m_laser_data.end(); ++iter) {
			if ((iter->first < ang_max) && (iter->first < ang_max) &&
			    (iter->second > data_max || iter->second < data_min))
				iter = (--m_laser_data.erase(iter));
		}
	}
}

AngSeqT GetAngSeq()
{
	AngSeqT angle_seq;
	LaserDataT::iterator iter = m_laser_data.begin();

	for (; iter != m_laser_data.end(); ++iter) {
		angle_seq.push_back(iter->first);
	}

	return angle_seq;
}

RangeSeqT GetRangeData() const
{
	RangeSeqT range_seq;
	LaserDataT::const_iterator iter = m_laser_data.begin();

	for (; iter != m_laser_data.end(); ++iter) {
		range_seq.push_back(iter->second);
	}

	return range_seq;
}

IntenSeqT GetInten() const
{
	return IntenSeqT(DataSize());
}

unsigned int DataSize() const
{
	return m_laser_data.size();
}

void MinAngData(double& angle, double& data)
{
}

void MaxAngData(double& angle, double& data)
{
}

public:
std::string frame_id;
unsigned int seq;
double angle_min;
double angle_max;
double angle_increment;
double time_increment;
double scan_time;
double range_min;
double range_max;


protected:
LaserDataT m_laser_data;
LaserLimit m_laser_limit;
};


#endif // ifndef _LASER_DATA_TYPE_HPP
