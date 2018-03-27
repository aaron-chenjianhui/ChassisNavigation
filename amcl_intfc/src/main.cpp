#include <ros/ros.h>
#include "amcl_intfc/amcl_intfc.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "amcl_intfc_node");

  ROS_INFO("Start AMCL Interface node\r\n");

  double cov_pos_thre, cov_ori_thre;
  ros::param::param<double>("~cov_pos_thre",
                            cov_pos_thre,
                            0.01);
  ros::param::param<double>("~cov_ori_thre",
                            cov_ori_thre,
                            0.01);

  AMCLIntfc amcl_node(cov_pos_thre, cov_ori_thre);

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  //  ros::spin();
  return 0;
}
