#include "amcl_intfc/amcl_intfc.h"

#include <boost/array.hpp>

AMCLIntfc::AMCLIntfc()
{
	m_init_puber = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
		"initialpose",
		1);
	m_crs_puber = m_nh.advertise<geometry_msgs::Pose2D>("coarse_pose", 1);
	m_update_client =
		m_nh.serviceClient<std_srvs::Empty>("request_nomotion_update");
	m_laser_suber = m_nh.subscribe("scan", 1,
				       &AMCLIntfc::LaserCallback, this);
	m_amcl_suber =
		m_nh.subscribe("amcl_pose", 1, &AMCLIntfc::AMCLCallback, this);
	m_sys_server = m_nh.advertiseService("amcl_status", &AMCLIntfc::statusCallback,
					     this);

	paramInit();
}

AMCLIntfc::AMCLIntfc(double cov_pos_thre, double cov_ori_thre) : AMCLIntfc()
{
	m_pos_cov_thre = cov_pos_thre;
	m_ori_cov_thre = cov_ori_thre;
}

AMCLIntfc::~AMCLIntfc()
{
}

void AMCLIntfc::paramInit()
{
	m_sys_status = SYS_INIT;
	m_amcl_status = AMCL_READY;
	m_amcl_conn_status = AMCL_CONN;
	m_cov_is_small = false;

	m_pos_cov_thre = 0.01;
	m_ori_cov_thre = 0.01;
}

void AMCLIntfc::calcInitPose(
	const sensor_msgs::LaserScan &			laser_data,
	geometry_msgs::PoseWithCovarianceStamped&	init_pose_re)
{
	unsigned int seq = laser_data.header.seq;
	double angle_min = laser_data.angle_min;
	double angle_max = laser_data.angle_max;
	double angle_increment = laser_data.angle_increment;
	double time_increment = laser_data.time_increment;
	double scan_time = laser_data.scan_time;
	double range_min = laser_data.range_min;
	double range_max = laser_data.range_max;
	std::vector<float> ranges = laser_data.ranges;
	std::vector<float> intensities = laser_data.intensities;

	float len = ranges[500];

	geometry_msgs::PoseWithCovarianceStamped init_pose;

	init_pose.header.seq = 1;
	init_pose.header.stamp = ros::Time::now();
	init_pose.header.frame_id = "map";

	init_pose.pose.pose.position.x = -len;
	init_pose.pose.pose.position.y = -2.350 / 2;
	init_pose.pose.pose.position.z = 0;
	init_pose.pose.pose.orientation.x = 0;
	init_pose.pose.pose.orientation.y = 0;
	init_pose.pose.pose.orientation.z = 0;
	init_pose.pose.pose.orientation.w = 1;

	boost::array<double, 36> arr;
	arr.assign(0);
	arr[0] = 3;
	arr[7] = 0.5;
	arr[35] = 0.2;
	init_pose.pose.covariance = arr;

	init_pose_re = init_pose;
}

void AMCLIntfc::PubInitPose()
{
	geometry_msgs::PoseWithCovarianceStamped init_pose;

	calcInitPose(m_laser_data, init_pose);

	m_init_puber.publish(init_pose);
}

void AMCLIntfc::ForceUpdate()
{
	std_srvs::Empty nomotion_srv;

	if (~m_update_client.call(nomotion_srv)) {
		ROS_INFO("Force Update Failed\r\n");
	}
}

void AMCLIntfc::PubCrsPose()
{
	geometry_msgs::Pose2D crs_pose;

	CovPoseToPose2D(m_amcl_pose, crs_pose);

	m_crs_puber.publish(crs_pose);
}

void AMCLIntfc::LaserCallback(const sensor_msgs::LaserScan& laser_data)
{
	m_laser_data = laser_data;
}

void AMCLIntfc::AMCLCallback(
	const geometry_msgs::PoseWithCovarianceStamped& amcl_pose)
{
	m_amcl_pose = amcl_pose;
}

bool AMCLIntfc::isCovSmall(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
	double cov_x = pose.pose.covariance[0];
	double cov_y = pose.pose.covariance[7];
	double cov_theta = pose.pose.covariance[35];

	if ((cov_x < m_pos_cov_thre) && (cov_y < m_pos_cov_thre) &&
	    (cov_theta < m_ori_cov_thre)) {
		return true;
	} else {
		return false;
	}
}

void AMCLIntfc::CovPoseToPose2D(
	const geometry_msgs::PoseWithCovarianceStamped& cov_pose,
	geometry_msgs::Pose2D &				pose_re)
{
	double x = cov_pose.pose.pose.position.x;
	double y = cov_pose.pose.pose.position.y;
	double quat_x = cov_pose.pose.pose.orientation.x;
	double quat_y = cov_pose.pose.pose.orientation.y;
	double quat_z = cov_pose.pose.pose.orientation.z;
	double quat_w = cov_pose.pose.pose.orientation.w;

	double theta = 2 * atan2(quat_z, quat_w);

	pose_re.x = x;
	pose_re.y = y;
	pose_re.theta = theta;
}

// status callback function
bool AMCLIntfc::statusCallback(laser_msgs::amcl_srv::Request &	req,
			       laser_msgs::amcl_srv::Response&	res)
{
	uint8_t sys_status_now = req.sys_status;
	uint8_t amcl_status_now = req.amcl_status;

	m_sys_status = (sys_status_t)sys_status_now;
	m_amcl_status = (amcl_status_t)amcl_status_now;

	if (AMCL_SEND_INIT == amcl_status_now) {
		PubInitPose();
	} else if (AMCL_SEND_UPDATE == amcl_status_now) {
		ForceUpdate();
	} else if (AMCL_SEND_LOC == amcl_status_now) {
		PubCrsPose();
	}

	bool ret = isCovSmall(m_amcl_pose);
	m_cov_is_small = ret;
	m_amcl_conn_status = AMCL_CONN;

	res.amcl_conn = m_amcl_conn_status;
	res.cov_is_small = m_cov_is_small;

	return true;
}
