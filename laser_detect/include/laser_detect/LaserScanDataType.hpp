#ifndef _LASER_SCAN_DATA_TYPE_HPP
#define _LASER_SCAN_DATA_TYPE_HPP

#include <sensor_msgs/LaserScan.h>

#include "LaserDataType.hpp"

#include "Line2D.hpp"

class LaserScanData : public LaserData {
public:
LaserScanData()
{
	m_laser_limit.ang_limit_min = -135;
	m_laser_limit.ang_limit_max = 135;
	m_laser_limit.data_limit_min = 0.8;
	m_laser_limit.data_limit_max = 10;
}

void UpdateData(const sensor_msgs::LaserScan& laser_data)
{
	frame_id = laser_data.header.frame_id;
	seq = laser_data.header.seq;
	angle_min = laser_data.angle_min;
	angle_max = laser_data.angle_max;
	angle_increment = laser_data.angle_increment;
	time_increment = laser_data.time_increment;
	scan_time = laser_data.scan_time;
	range_min = laser_data.range_min;
	range_max = laser_data.range_max;

	std::vector<float> ranges = laser_data.ranges;
	std::vector<float> intensities = laser_data.intensities;

	//
	// lidar_iter_type iter;
	std::vector<float>::iterator iter;
	double angle_now = angle_min;
	double lidar_data_now;

	for (iter = ranges.begin(); iter != ranges.end(); ++iter) {
		lidar_data_now = (double)(*iter);
		InsertData(std::pair<double, double>(angle_now, lidar_data_now));

		angle_now += angle_increment;
	}

	// eliminate Unreasonable data
	LimitLaserData({ m_laser_limit });
}

sensor_msgs::LaserScan ToLaserScan() const
{
	sensor_msgs::LaserScan msgs;

	msgs.header.frame_id = frame_id;
	msgs.header.seq = seq;
	msgs.angle_min = angle_min;
	msgs.angle_max = angle_max;
	msgs.angle_increment = angle_increment;
	msgs.time_increment = time_increment;
	msgs.scan_time = scan_time;
	msgs.range_min = range_min;
	msgs.range_max = range_max;

	RangeSeqT range = GetRangeData();
	msgs.ranges = std::vector<float>(range.begin(), range.end());
	IntenSeqT inten = GetInten();
	msgs.intensities = std::vector<float>(inten.begin(), inten.end());

	return msgs;
}




void ChooseBetterRange(Line2D& line, double min_range, double max_range)
{
}

private:
LaserLimit m_laser_limit;
};

#endif // ifndef _LASER_DATA_TYPE_HPP
