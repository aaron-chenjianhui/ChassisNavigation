#include "icp_intfc_node.h"



ICPIntfc::ICPIntfc()
{
	m_icp_client = m_nh.advertiseService("nav_status", &ICPIntfc::statusCallback, this);

	m_pose_suber = m_nh.subscribe("pose2D", 1, &ICPIntfc::ICPPoseCallback, this);

	m_init_suber = m_nh.subscribe("init_laser_pose", 1, &ICPIntfc::InitPoseCallback, this);
}

ICPIntfc::~ICPIntfc()
{
}

bool ICPIntfc::statusCallback(laser_msgs::nav_srv::Request &req, laser_msgs::nav_srv::Response& res)
{
	uint8_t sys_status_now = req.sys_status;
	uint8_t nav_status_now = req.nav_status;

	m_sys_status = (sys_status_t)sys_status_now;
	m_nav_status = (nav_status_t)nav_status_now;

	bool nav_conn_now = true;
	res.nav_conn = nav_conn_now;

	return true;
}

void ICPIntfc::InitPoseCallback()
{
}


void ICPIntfc::ICPPoseCallback()
{
}
