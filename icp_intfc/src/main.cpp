#include "ros/ros.h"
#include "icp_intfc_node.h"


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "icp_interface");

	ICPIntfc icp_intfc;

	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();

	return 0;
}
