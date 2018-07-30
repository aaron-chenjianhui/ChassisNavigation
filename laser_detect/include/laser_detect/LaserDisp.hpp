#ifndef _LASER_DISP_HPP
#define _LASER_DISP_HPP

#include <string>
#include <initializer_list>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include "LaserScanDataType.hpp"
#include "Pose2D.hpp"
#include "Line2D.hpp"

#include "laser_msgs/UnitConvert.h"



class LaserDisp {
public:
typedef std::vector<Point2D> PointsT;

public:
LaserDisp(ros::NodeHandle nh) : m_nh(nh)
{
	m_laser_puber = m_nh.advertise<sensor_msgs::LaserScan>("custom_scan", 1);
	m_line_puber = m_nh.advertise<sensor_msgs::PointCloud>("custom_line", 1);
}

LaserDisp()
{
	m_laser_puber = m_nh.advertise<sensor_msgs::LaserScan>("custom_scan", 1);
	m_line_puber = m_nh.advertise<sensor_msgs::PointCloud>("custom_line", 1);
}

~LaserDisp()
{
}

void DispLaserScan(const LaserScanData& laser_data)
{
	sensor_msgs::LaserScan msgs;

	msgs = laser_data.ToLaserScan();

	m_laser_puber.publish(msgs);
}

void DispLine(const std::initializer_list<Line2D> &	list,
	      const std::initializer_list<double> &	inten_list)
{
	if (list.size() != inten_list.size()) {
		return;
	}

	sensor_msgs::PointCloud cloud;
	std::vector<geometry_msgs::Point32> points_ros;
	std::vector<float> inten;

	auto list_iter = list.begin();
	auto inten_iter = inten_list.begin();

	for (; list_iter != list.end(); ++list_iter, ++inten_iter) {
		Line2D line = *list_iter;
		PointsT points = line.GenPoints(line.m_ax - 4000 * line.m_ny,
						line.m_ax + 4000 * line.m_ny);

		PointsT::iterator iter = points.begin();

		for (; iter != points.end(); ++iter) {
			geometry_msgs::Point32 p;
			p.x = iter->x;
			p.y = iter->y;
			p.z = 0;

			points_ros.push_back(p);
			inten.push_back(*inten_iter);
		}
	}

	cloud.points = points_ros;
	cloud.channels[0].values = inten;

	m_line_puber.publish(cloud);
}

void BroadTransform(const Pose2D& pose, std::string& parent, std::string& child)
{
	tf::Transform transform;
	tf::Quaternion q;

	transform.setOrigin(tf::Vector3(MM2M(pose.m_x), MM2M(pose.m_y), 0));
	q.setRPY(0, 0, pose.m_theta);
	transform.setRotation(q);

	m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, child));
}


private:
ros::NodeHandle m_nh;

ros::Publisher m_laser_puber;
ros::Publisher m_line_puber;

tf::TransformBroadcaster m_br;
};





#endif
