#include "icp_intfc/icp_intfc_node.h"

ICPIntfc::ICPIntfc()
{
	m_icp_server = m_nh.advertiseService("nav_status", &ICPIntfc::statusCallback, this);

	m_pose_suber = m_nh.subscribe("pose2D", 1, &ICPIntfc::ICPPoseCallback, this);

	m_init_suber = m_nh.subscribe("init_laser_pose", 1, &ICPIntfc::InitPoseCallback, this);

	m_pose_puber = m_nh.advertise<geometry_msgs::Pose2D>("laser_pose", 1000);

	paramInit();
}

void ICPIntfc::paramInit()
{
	m_pose_buf.clear();
}

ICPIntfc::~ICPIntfc()
{
}

bool ICPIntfc::statusCallback(laser_msgs::nav_srv::Request &req, laser_msgs::nav_srv::Response& res)
{
	boost::mutex::scoped_lock lock(m_mutex);

	uint8_t sys_status_now = req.sys_status;
	uint8_t nav_status_now = req.nav_status;

	m_sys_status = (sys_status_t)sys_status_now;
	m_nav_status = (nav_status_t)nav_status_now;

	if (SYS_GLOBAL_OK == m_sys_status || SYS_TRACK == m_sys_status) {
		if (!m_pose_buf.empty()) {
			m_laser_pose = m_pose_buf.front();
			m_pose_buf.clear();
		}

		mat3x3 T_Laser_in_Ori = m_coeff_mat * m_laser_pose.ToMat();

		// publish pose2d message
		geometry_msgs::Pose2D pose_msg;
		Pose2D pose(T_Laser_in_Ori);
		pose_msg.x = pose.m_x;
		pose_msg.y = pose.m_y;
		pose_msg.theta = pose.m_theta;
		m_pose_puber.publish(pose_msg);

		// for display
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pose_msg.x, pose_msg.y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, pose_msg.theta);
		transform.setRotation(q);
		m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
							"original", "dest"));
	} else {
		if (!m_pose_buf.empty()) {
			m_pose_buf.clear();
		}
	}

	bool nav_conn_now = true;
	res.nav_conn = nav_conn_now;

	return true;
}

void ICPIntfc::InitPoseCallback(const geometry_msgs::Pose2D& ori_pose)
{
	boost::mutex::scoped_lock lock(m_mutex);

	m_truth_pose = Pose2D(ori_pose.x, ori_pose.y, ori_pose.theta);

	// m_truth_pose: laser pose in original
	// m_laser_pose: laser pose in reference frame
	m_coeff_mat = m_truth_pose.ToMat() * (m_laser_pose.ToMat().inverse());

	m_b_init_pose = true;
}

void ICPIntfc::ICPPoseCallback(const geometry_msgs::Pose2D& pose)
{
	boost::mutex::scoped_lock lock(m_mutex);


	if (!m_pose_buf.empty()) {
		m_pose_buf.push_back(Pose2D(M2MM(pose.x), M2MM(pose.y), pose.theta));
	}
}
