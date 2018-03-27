#ifndef _AMCL_INTFC_H
#define _AMCL_INTFC_H

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>

#include "laser_msgs/amcl_srv.h"

#include "SysStateTypes.h"


class AMCLIntfc {
public:

  AMCLIntfc();
  AMCLIntfc(double pos_cov_thre,
            double ori_cov_thre);
  ~AMCLIntfc();

  void paramInit();

  void LaserCallback(const sensor_msgs::LaserScan& laser_data);
  void AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose);

  // status callback function
  bool statusCallback(laser_msgs::amcl_srv::Request & req,
                      laser_msgs::amcl_srv::Response& res);

  bool isCovSmall(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void calcInitPose(const sensor_msgs::LaserScan            & scan,
                    geometry_msgs::PoseWithCovarianceStamped& init_pose);
  void CovPoseToPose2D(const geometry_msgs::PoseWithCovarianceStamped& cov_pose,
                       geometry_msgs::Pose2D                         & pose);

private:

  ros::NodeHandle m_nh;
  ros::Publisher m_init_puber;
  ros::Publisher m_crs_puber;
  ros::ServiceClient m_update_client;
  ros::Subscriber m_laser_suber;
  ros::Subscriber m_amcl_suber;
  ros::ServiceServer m_sys_server;

  geometry_msgs::PoseWithCovarianceStamped m_amcl_pose;
  geometry_msgs::Pose2D m_crs_pose;
  sensor_msgs::LaserScan m_laser_data;

  sys_status_t m_sys_status;
  amcl_status_t m_amcl_status;
  amcl_conn_status_t m_amcl_conn_status;
  bool m_cov_is_small;

  double m_ori_cov_thre;
  double m_pos_cov_thre;
};

#endif // ifndef _AMCL_INTFC_H
