#include "laser_detect_node.h"


void CoarsePoseCallback(const geometry_msgs::Pose2D& pose)
{
	m_coarse_pose.x = pose.x;
	m_coarse_pose.y = pose.y;
	m_coarse_pose.theta = pose.theta;
}

LaserDetectNode::LaserCallback(const sensor_msgs::LaserScan& laser_data){
	//
	boost::mutex::scoped_lock lock(m_mutex);

	// collection count
	m_filter_count++;
}


LaserDetectNode::statusCallback(laser_msgs::laser_detect_srv::Request &req, laser_msgs::Laser_detect_srv::Response& res){
	// retrieve status from request
	uint8_t system_status_now = req.sys_status;
	uint8_t detect_status_now = req.detect_status;
	bool calc_request = req.calc_request;

	m_sys_status = static_cast<sys_status_t>system_status_now;
	m_detect_status = static_cast<detect_status_t>detect_status_now;

	// output parameter
	bool detect_conn = true;
	bool is_calcd = false;

	// status process
	if (DETECT_PREPARATION == m_detect_status) {
	} else if (DETECT_CAL_ING == m_detect_status) {
	} else if (DETECT_CAL_OK == m_detect_status) {
		is_calcd = true;
	} else if (DETECT_ERR == m_detect_status) {
		ROS_INFO("Laser Detect Node encounters error");
	}

	// return response
	res.detect_conn = detect_conn;
	res.filter_count = m_filter_count;
	res.is_calcd = is_calcd;
}
