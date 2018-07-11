#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "amcl_test_nomotion");
  ROS_INFO("Start amcl testing\r\n");


  ros::NodeHandle nh;

  ros::ServiceClient nomotion_client = nh.serviceClient<std_srvs::Empty>(
    "request_nomotion_update");


  ros::Rate loop_rate(1);

  char ch = '0';

  while ((ros::ok()) && ('q' != ch)) {
    std_srvs::Empty nomotion_srv;

    if (nomotion_client.call(nomotion_srv)) {
      ROS_INFO("Force Update\r\n");
    }
    else {
      ROS_INFO("Force Update Failed\r\n");
    }

    ros::spinOnce();

    ch = std::cin.get();
  }

  ROS_INFO("ROS shutdown");

  ros::shutdown();

  return 0;
}
