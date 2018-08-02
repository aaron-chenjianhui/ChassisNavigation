#ifndef _CONTAINER_DETECT_H
#define _CONTAINER_DETECT_H

#include "Pose2D.hpp"
#include "LaserDataType.hpp"
#include "Line2D.hpp"
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
ContainerDetect()
{
	double width = 2350;
	double length = 5650;
	Pose2D pose(-length, -width / 2.0, 0);

	UpdateRange(pose, width, length, DEG2RAD(5));
}

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

	CalRange(left_min, left_max, pose, delta, m_param.left_min, m_param.left_max);
	CalRange(front_min, front_max, pose, delta, m_param.front_min, m_param.front_max);
	CalRange(right_min, right_max, pose, delta, m_param.right_min, m_param.right_max);

	m_param.width = width;
	m_param.length = length;

	// debug
	DEBUGLOG("left min is: " << m_param.left_min);
	DEBUGLOG("left max is: " << m_param.left_max);
	DEBUGLOG("front min is: " << m_param.front_min);
	DEBUGLOG("front max is: " << m_param.front_max);
	DEBUGLOG("right min is: " << m_param.right_min);
	DEBUGLOG("right max is: " << m_param.right_max);
}

void FindWallLine(const LaserData& raw_laser_data, ContainerParam& param)
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

	// // Find original point
	// Point2D pos_in_laser = Line2D::CrossPoint(m_param.left_line, m_param.front_line);
	// double theta_in_laser = atan2(ori_in_laser.y - m_param.left_line.m_ay,
	//                            ori_in_laser.x - m_param.left_line.m_ax);
	// Pose2D ori_in_laser(pos_in_laser.m_x, pos_in_laser.m_y, theta_in_laser);
	// Pose2D laser_in_ori = ori_in_laser.InvPose();

	param = m_param;
	param.left_min = left_laser_data.GetAngSeq().front();
	param.left_max = left_laser_data.GetAngSeq().back();
	param.front_min = front_laser_data.GetAngSeq().front();
	param.front_max = front_laser_data.GetAngSeq().back();
	param.right_min = right_laser_data.GetAngSeq().front();
	param.right_max = right_laser_data.GetAngSeq().back();
}

void ChooseBetterRegion(const Line2D& line,
			const double& ref_min, const double& ref_max,
			double& min_range, double& max_range)
{
	Line2D v_line;

	v_line.m_ax = 0;
	v_line.m_ay = 0;
	v_line.m_nx = -line.m_ny;
	v_line.m_ny = line.m_nx;

	Point2D cross_point = Line2D::CrossPoint(v_line, line);
	double theta_in_laser = atan2(cross_point.y, cross_point.x);

	min_range = std::min(theta_in_laser - DEG2RAD(20), ref_min);
	max_range = std::max(theta_in_laser + DEG2RAD(20), ref_max);
}




private:

void CalRange(const Point2D& min_p, const Point2D& max_p,
	      const Pose2D& pose, double delta,
	      double &min_angle, double &max_angle)
{
	MatrixCal mat_cal;

	mat3x3 T_laser_in_ori;
	MatrixCal::PoseT laser_pose;

	laser_pose.push_back(pose.m_x);
	laser_pose.push_back(pose.m_y);
	laser_pose.push_back(pose.m_theta);
	mat_cal.PoseToMat(laser_pose, T_laser_in_ori);

	mat3x1 min_in_ori, min_in_laser;
	mat3x1 max_in_ori, max_in_laser;
	min_in_ori(0) = min_p.x;
	min_in_ori(1) = min_p.y;
	min_in_ori(2) = 1;
	max_in_ori(0) = max_p.x;
	max_in_ori(1) = max_p.y;
	max_in_ori(2) = 1;

	min_in_laser = T_laser_in_ori.inverse() * min_in_ori;
	max_in_laser = T_laser_in_ori.inverse() * max_in_ori;

	min_angle = atan2(min_in_laser(1), min_in_laser(0)) - delta;
	max_angle = atan2(max_in_laser(1), max_in_laser(0)) + delta;
}




private:
ContainerParam m_param;
LaserLineDetector m_line_detector;
};

#endif
