#ifndef _MY_POSE_2D_HPP
#define _MY_POSE_2D_HPP

// #include <ostream>
#include "laser_msgs/MyMatTypes.hpp"

class Pose2D {
public:

double m_x;
double m_y;
double m_theta;

public:
Pose2D(double x = 0, double y = 0, double theta = 0) :
	m_x(x),
	m_y(y),
	m_theta(theta)
{
}

Pose2D(mat3x3& pose_mat)
{
	m_x = pose_mat(0, 2);
	m_y = pose_mat(1, 2);
	m_theta = atan(pose_mat(1, 0) / pose_mat(0, 0));
}

Pose2D InvPose()
{
	return Pose2D(-m_x, -m_y, -m_theta);
}

mat3x3 ToMat()
{
	mat3x3 pose_mat;

	double pose_x = m_x;
	double pose_y = m_y;
	double pose_theta = m_theta;
	double sin_theta = sin(pose_theta);
	double cos_theta = cos(pose_theta);

	pose_mat(0, 2) = pose_x;
	pose_mat(1, 2) = pose_y;
	pose_mat(0, 0) = cos_theta;
	pose_mat(0, 1) = -sin_theta;
	pose_mat(1, 0) = sin_theta;
	pose_mat(1, 1) = cos_theta;
	pose_mat(2, 0) = 0;
	pose_mat(2, 1) = 0;
	pose_mat(2, 2) = 1;

	return pose_mat;
}



// inline std::ostream& operator<<(std::ostream& output, const MyPose2D& pose)
// {
//   output << pose.m_x << ' ' << pose.m_y << ' ' << pose.m_theta;
//   return output;
// }
};

#endif // ifndef _MY_POSE_2D_HPP
