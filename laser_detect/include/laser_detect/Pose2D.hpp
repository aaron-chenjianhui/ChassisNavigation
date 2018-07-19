#ifndef _MY_POSE_2D_HPP
#define _MY_POSE_2D_HPP

// #include <ostream>

class Pose2D {
public:

double m_x;
double m_y;
double m_theta;

public:
Pose2D(double x, double y, double theta) :
	m_x(x),
	m_y(y),
	m_theta(theta)
{
}

Pose2D InvPose()
{
	return Pose2D(-m_x, -m_y, -m_theta);
}

// inline std::ostream& operator<<(std::ostream& output, const MyPose2D& pose)
// {
//   output << pose.m_x << ' ' << pose.m_y << ' ' << pose.m_theta;
//   return output;
// }
};

#endif // ifndef _MY_POSE_2D_HPP
