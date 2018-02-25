#include <ros/ros.h>


#include "laser_detect.h"


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "laser_detect");

  ROS_INFO("Start Laser Detect node\r\n");

  lms::LaserDetect laser_detect_class;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  //  ros::spin();
  return 0;
}
