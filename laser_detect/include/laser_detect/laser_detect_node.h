#ifndef _LASER_DETECT_NODE_H
#define _LASER_DETECT_NODE_H

#include "LaserDisp.hpp"


class LaserDetectNode {
public:
LaserDetectNode();
~LaserDetectNode();



// LiDAR data callback function
void LaserCallback(const sensor_msgs::LaserScan& laser_data);

// for state machine
bool statusCallback(laser_msgs::laser_detect_srv::Request &req, laser_msgs::laser_detect_srv::Response& res);

private:
ros::NodeHandle m_nh;

// subscribe LiDAR data
ros::Subscriber m_laser_suber;

// for state machine
ros::ServiceServer m_status_server;

// for result display
LaserDisp m_disp;
};


#endif
