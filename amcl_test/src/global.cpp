#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "amcl_test_global");
  ROS_INFO("Start amcl testing\r\n");


  ros::NodeHandle nh;

  ros::ServiceClient localization_client =
    nh.serviceClient<std_srvs::Empty>(
      "global_localization");

  ros::Rate loop_rate(1);

  char ch = '0';

  while ((ros::ok()) && ('q' != ch)) {
    std_srvs::Empty localization_srv;

    if (localization_client.call(localization_srv)) {
      ROS_INFO("Global location\r\n");
    }
    else {
      ROS_INFO("Global location failed\r\n");
    }

    ros::spinOnce();

    ch = std::cin.get();
  }

  ROS_INFO("ROS shutdown");

  ros::shutdown();

  return 0;
}
