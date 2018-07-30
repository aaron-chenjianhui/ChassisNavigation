#include "laser_detect_node.h"


LaserDetectNode::LaserDetectNode() :
	m_filter(20),
	m_disp(m_nh)
{
	m_laser_suber = m_nh.subscribe("scan_comp", 1, &LaserDetectNode::LaserCallback, this);
	m_coarse_pose_suber = m_nh.subscribe("coarse_pose", 1, &LaserDetectNode::CoarsePoseCallback, this);
	m_status_server = m_nh.advertiseService("detect_status", &LaserDetectNode::statusCallback, this);

	m_init_puber = m_nh.advertise<geometry_msgs::Pose2D>("init_laser_pose", 1);

	m_laser_buf.clear();
}

LaserDetectNode::~LaserDetectNode()
{
}

void LaserDetectNode::CoarsePoseCallback(const geometry_msgs::Pose2D& pose)
{
	m_coarse_pose.m_x = pose.x;
	m_coarse_pose.m_y = pose.y;
	m_coarse_pose.m_theta = pose.theta;
}

void LaserDetectNode::LaserCallback(const sensor_msgs::LaserScan& laser_data)
{
	//
	boost::mutex::scoped_lock lock(m_mutex);

	// update data
	m_tmp_laser_data.UpdateData(laser_data);

	// maintane the buffer vector
	m_laser_buf.push_back(m_tmp_laser_data);
	if (m_laser_buf.size() > 5) {
		m_laser_buf.pop_front();
	}
}


bool LaserDetectNode::statusCallback(laser_msgs::laser_detect_srv::Request &req, laser_msgs::laser_detect_srv::Response& res)
{
	// retrieve status from request
	uint8_t system_status_now = req.sys_status;
	uint8_t detect_status_now = req.detect_status;
	bool calc_request = req.calc_request;

	m_sys_status = static_cast<sys_status_t>(system_status_now);
	m_detect_status = static_cast<detect_status_t>(detect_status_now);

	// output parameter
	bool detect_conn = true;
	bool is_calcd = false;

	// status process
	if (DETECT_PREPARATION == m_detect_status) {
		if (!m_laser_buf.empty()) {
			LaserScanData laser_data = m_laser_buf.front();
			m_laser_buf.pop_front();

			m_filter.Filtering(laser_data, m_laser_data);

			m_filter_count++;
		} else {
		}
	} else if (DETECT_CAL_ING == m_detect_status) {
		m_filter_count = 0;

		double width = 2350;
		double length = 1700;

		m_detect.UpdateRange(m_coarse_pose, width, length, DEG2RAD(5));

		ContainerParam param;
		m_detect.FindWallLine(m_laser_data, param);

		double left_min, left_max, front_min, front_max, right_min, right_max;
		m_detect.ChooseBetterRegion(param.left_line, param.left_min, param.left_max, left_min, left_max);
		LaserScanData left_laser;
		param.left_data.DataInRange(left_min, left_max, left_laser);

		m_detect.ChooseBetterRegion(param.front_line, param.front_min, param.front_max, front_min, front_max);
		LaserScanData front_laser;
		param.front_data.DataInRange(front_min, front_max, front_laser);

		m_detect.ChooseBetterRegion(param.right_line, param.right_min, param.right_max, right_min, right_max);
		LaserScanData right_laser;
		param.right_data.DataInRange(right_min, right_max, right_laser);

		// local detection
		Line2D left_revise_line = m_surf_finder.GetSurface(left_laser, param.left_line);
		Line2D front_revise_line = m_surf_finder.GetSurface(front_laser, param.front_line);
		Point2D pos = Line2D::CrossPoint(left_revise_line, front_revise_line);
		double theta = atan2(pos.y - left_revise_line.m_ay, pos.x - left_revise_line.m_ax);
		Pose2D ori_in_laser(pos.x, pos.y, theta);
		m_laser_in_ori = ori_in_laser.InvPose();
	} else if (DETECT_CAL_OK == m_detect_status) {
		m_filter_count = 0;
		is_calcd = true;

		geometry_msgs::Pose2D pose;
		pose.x = m_laser_in_ori.m_x;
		pose.y = m_laser_in_ori.m_y;
		pose.theta = m_laser_in_ori.m_theta;
		m_init_puber.publish(pose);
	} else if (DETECT_ERR == m_detect_status) {
		m_filter_count = 0;
		ROS_INFO("Laser Detect Node encounters error");
	} else {
		m_filter_count = 0;
	}

	// return response
	res.detect_conn = detect_conn;
	res.filter_count = m_filter_count;
	res.is_calcd = is_calcd;

	return true;
}
