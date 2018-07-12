#ifndef _LASER_DISP_HPP
#define _LASER_DISP_HPP

#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

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

void DispLine(std::initializer_list<Line2D &>	list,
	      std::initializer_list<double>	inten_list)
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
			points_ros.push_back(geometry_msgs::Point32(iter->x, iter->y, 0));
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

	transform.setOrigin(tf::Vector3(MM2M(pose.x), MM2M(pose.y), 0));
	q.setRPY(0, 0, pose.theta);
	transform.setRotation(q);

	m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now()), parent, child);
}


private:
ros::NodeHandle m_nh;

ros::Publisher m_laser_puber;
ros::Publisher m_line_puber;
ros::TransformBroadcaster m_br;

#endif
