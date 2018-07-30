#include <ros/ros.h>


#include "laser_detect_node.h"


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "laser_detect");

	ROS_INFO("Start Laser Detect node\r\n");

	LaserDetectNode laser_detect_node;

	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();

	//  ros::spin();
	return 0;
}
