#ifndef _MY_POSE_2D_ROS_HPP
#define _MY_POSE_2D_ROS_HPP

#include "Pose2D.hpp"

#include <geometry_msgs/Pose2D.h>

class MyPose2DRos : public MyPose2D {
public:

void UpdatePose(const geometry_msgs::Pose2D& my_pose)
{
	m_x = my_pose.x;
	m_y = my_pose.y;
	m_theta = my_pose.theta;
}
};

#endif // ifndef _MY_POSE_2D_ROS_HPP
