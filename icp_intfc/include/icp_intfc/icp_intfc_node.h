#ifndef _ICP_INTFC_NODE_H
#define _ICP_INTFC_NODE_H

#include "Pose2D.hpp"


class ICPIntfc {
public:
ICPIntfc();
~ICPIntfc();


// state machine callback
bool statusCallback(laser_msgs::nav_srv::Request &req, laser_msgs::nav_srv::Response& res);

// subscribe init pose
void InitPoseCallback(const geometry_msgs::Pose2D&);
// subscribe icp pose
void ICPPoseCallback(const geometry_msgs::Pose2D&);
// Init parameter
void paramInit();


private:
ros::NodeHandle m_nh;

// state callback
ros::ServiceClient m_icp_client;

// subscribe laser_scan_matcher pose
ros::Subscriber m_pose_suber;
// subscribe laser init pose
ros::Subscriber m_init_suber;
// publish calculated laser pose
ros::Publisher m_pose_puber;
// for display
ros::TransformBroadcaster m_br;


// for node status
sys_status_t m_sys_status;
nav_status_t m_nav_status;

// receive new pose from laser_scan_matcher
std::vector<Pose2D> m_pose_buf;
//
Pose2D m_truth_pose;
Pose2D m_laser_pose;
// coeff for transfering m_laser_pose to m_truth_pose
mat3x3 m_coeff_mat;
// whether receive init pose, and calculate the coeff
bool m_b_init_pose;

// for
boost::mutex m_mutex;
};


#endif
