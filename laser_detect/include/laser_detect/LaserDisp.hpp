#ifndef _LASER_DISP_HPP
#define _LASER_DISP_HPP

#include <string>

#include <ros/ros.h>

#include "LaserDataType.hpp"



class LaserDisp {
public:
LaserDisp(ros::NodeHandle nh) : m_nh(nh)
{
	m_laser_puber = m_nh.advertise<sensor_msgs::LaserScan>("custom_scan", 1);
	m_line_puber = m_nh.advertise<sensor_msgs::PointCloud>("custom_line", 1);
};
~LaserDisp()
{
};
};


void DispLaserScan(const LaserData& laser_data)
{
	sensor_msgs::LaserScan msgs;

	msgs = laser_data.GetLaserScan(frame_id);

	m_laser_puber.publish(msgs);
}

void DispLine(const Line2D& line)
{
}


private:
ros::NodeHandle m_nh;

ros::Publisher m_laser_puber;
ros::Publisher m_line_puber;

#endif
