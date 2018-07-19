#ifndef _ICP_INTFC_NODE_H
#define _ICP_INTFC_NODE_H


class ICPIntfc {
public:
ICPIntfc();
~ICPIntfc();


// state machine callback
bool statusCallback(laser_msgs::nav_srv::Request &req, laser_msgs::nav_srv::Response& res);

//
void InitPoseCallback();
//
void ICPPoseCallback();


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


// for node status
sys_status_t m_sys_status;
nav_status_t m_nav_status;
};


#endif
