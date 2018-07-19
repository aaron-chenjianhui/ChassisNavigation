#ifndef _LASER_DETECT_NODE_H
#define _LASER_DETECT_NODE_H

#include "LaserDisp.hpp"
#include "Pose2D.hpp"
#include "LaserFilter.hpp"


class LaserDetectNode {
public:
LaserDetectNode();
~LaserDetectNode();


void CoarsePoseCallback(const geometry_msgs::Pose2D& pose);

// LiDAR data callback function
void LaserCallback(const sensor_msgs::LaserScan& laser_data);

// for state machine
bool statusCallback(laser_msgs::laser_detect_srv::Request &req, laser_msgs::laser_detect_srv::Response& res);

private:
ros::NodeHandle m_nh;
// global status
ContainerDetect m_container_detect;


// subscribe LiDAR data
ros::Subscriber m_laser_suber;

// subscribe coarse pose
ros::Subscriber m_coarse_pose_suber;

// for state machine
ros::ServiceServer m_status_server;

// for initial pose publish
ros::Publisher m_init_puber;

// for result display
LaserDisp m_disp;

// for node status
detect_status_t m_detect_status;
sys_status_t m_sys_status;
uint16_t m_filter_count;
bool m_is_calcd;

// coarse pose
Pose2D m_coarse_pose;

// subclass
LaserMAFilter m_filter;
LaserDisp m_disp;
ContainerDetect m_detect;
FindSurface m_surf_finder;

// data class
LaserScanData m_laser_data;
LaserScanData m_tmp_laser_data;
std::vector<LaserScanData> m_laser_buf;
Pose2D m_laser_in_ori;
};



#endif
