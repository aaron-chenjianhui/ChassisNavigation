#ifndef _CONTAINER_DETECT_H
#define _CONTAINER_DETECT_H

#include "Pose2D.hpp"
#include "LaserDataType.hpp"
#include "Line2D.hpp"
#include "MatrixCal.h"
#include "MathType.h"
#include "LineDetector.h"

struct ContainerParam {
	double		width = 0;
	double		length = 0;

	Line2D		left_line;
	LaserData	left_data;
	double		left_min;
	double		left_max;

	Line2D		right_line;
	LaserData	right_data;
	double		right_min;
	double		right_max;

	Line2D		front_line;
	LaserData	front_data;
	double		front_min;
	double		front_max;
};

class ContainerDetect {
public:
ContainerDetect(Pose2D& pose, double container_width, double container_length)
{
	UpdateRange(pose, container_width, container_length, DEG2RAD(5));
}

~ContainerDetect()
{
}

void UpdateRange(const Pose2D& pose, double width, double length, double delta)
{
	Point2D left_min(0, 0);
	Point2D left_max(-length, 0);
	Point2D front_min(0, -width);
	Point2D front_max(0, 0);
	Point2D right_min(-length, -width);
	Point2D right_max(0, -width);

	double left_min_angle, left_max_angle;
	double front_min_angle, front_max_angle;
	double right_min_angle, right_max_angle;

	CalRange(left_min, left_max, pose, delta, m_container_param.left_min, m_container_param.left_max);
	CalRange(front_min, front_max, pose, delta, m_container_param.front_min, m_container_param.front_max);
	CalRange(right_min, right_max, pose, delta, m_container_param.right_min, m_container_param.right_max);

	m_container_param.width = width;
	m_container_param.length = length;
}

void FindWallLine(const LaserData& raw_laser_data, ContainerParam& container_param)
{
	LaserData left_laser_data, front_laser_data, right_laser_data;
	bool ret = false;

	if (raw_laser_data.DataInRange(m_param.left_min, m_param.left_max, left_laser_data)) {
		m_line_detector.LaserFindLine(left_laser_data, m_param.left_line, m_param.left_data);
	} else {
		DEBUGLOG("Can not select left laser data in range");
	}

	if (raw_laser_data.DataInRange(m_param.front_min, m_param.front_max, front_laser_data)) {
		m_line_detector.LaserFindLine(front_laser_data, m_param.front_line, m_param.front_data);
	} else {
		DEBUGLOG("Can not select front laser data in range");
	}

	if (raw_laser_data.DataInRange(m_param.right_min, m_param.right_max, right_laser_data)) {
		m_line_detector.LaserFindLine(right_laser_data, m_param.right_line, m_param.right_data);
	} else {
		DEBUGLOG("Can not select right laser data in range");
	}
}

void ChooseBetterRegion(const Line2D& line, const Pose2D& pose, const double& ref_min, const double& ref_max, double& min_range, double& max_range)
{
	Line2D v_line;

	v_line.ax = pose.x;
	v_line.ay = pose.y;
	v_line.nx = -line.ny;
	v_line.ny = line.nx;

	Point2D cross_point = Line2D::CrossPoint(v_line, line);
	double theta_in_ori = atan2(cross_point.y - pose.y, cross_point.x - pose.x);
	double theta_in_laser = theta_in_ori - pose.theta;

	min_range = std::min(theta_in_laser - DEG2RAD(20), ref_min);
	max_range = std::max(theta_in_laser + DEG2RAD(20), ref_max);
}


private:

void CalRange(const Point2D& min_p, const Point2D& max_p, const MyPose2D& pose,
	      double delta, double &min_angle, double &max_angle)
{
	MatrixCal mat_cal;

	mat3x3 T_laser_in_ori;
	MatrixCal::PoseT laser_pose;

	laser_pose.push_back(pose.x);
	laser_pose.push_back(pose.y);
	laser_pose.push_back(pose.theta);
	mat_cal.PoseToMat(laser_pose, T_laser_in_ori);

	mat3x1 min_in_ori, min_in_laser;
	mat3x1 max_in_ori, max_in_laser;
	min_in_ori(1) = min_p.x - pose.x;
	min_in_ori(2) = min_p.y - pose.y;
	min_in_ori(3) = 1;
	max_in_ori(1) = max_p.x - pose.x;
	max_in_ori(2) = max_p.y - pose.y;
	max_in_ori(3) = 1;

	min_in_laser = T_laser_in_ori * max_in_laser;
	max_in_laser = T_laser_in_ori * max_in_laser;

	min_angle = atan2(min_in_laser(2), min_in_laser(1)) - delta;
	max_angle = atan2(max_in_laser(2), max_in_laser(1)) + delta;
}




private:
ContainerParam m_param;
LaserLineDetector m_line_detecotr;
};

#endif