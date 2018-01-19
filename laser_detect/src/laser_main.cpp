#include <ros/ros.h>


#include "laser_detect.h"


int main(int argc, char *argv[]){
  ros::init(argc, argv, "laser_detect");

  lms::LaserDetect laser_detect_class;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

//  ros::spin();
  return 0;
}
