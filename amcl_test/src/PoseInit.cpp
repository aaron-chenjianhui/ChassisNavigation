#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <boost/array.hpp>

unsigned int count = 0;

bool laser_update = false;
geometry_msgs::PoseWithCovarianceStamped init_pose;

void LaserScanCallback(const sensor_msgs::LaserScan& laser_data);

int  main(int argc, char *argv[]) {
  ros::init(argc, argv, "init_pose_pub");
  ROS_INFO("Start amcl testing");


  ros::NodeHandle nh;

  ros::Subscriber laser_suber = nh.subscribe("scan", 1000, LaserScanCallback);
  ros::Publisher  init_puber  =
    nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);

  char ch = '0';

  while ((ros::ok()) && ('q' != ch)) {
    if (true == laser_update) {
      init_puber.publish(init_pose);

      ROS_INFO("Publish initialized pose");

      laser_update = false;
    }

    ros::spinOnce();

    ch = std::cin.get();
  }

  ROS_INFO("ROS shutdown");

  ros::shutdown();

  return 0;
}

void LaserScanCallback(const sensor_msgs::LaserScan& laser_data) {
  unsigned int seq               = laser_data.header.seq;
  double angle_min               = laser_data.angle_min;
  double angle_max               = laser_data.angle_max;
  double angle_increment         = laser_data.angle_increment;
  double time_increment          = laser_data.time_increment;
  double scan_time               = laser_data.scan_time;
  double range_min               = laser_data.range_min;
  double range_max               = laser_data.range_max;
  std::vector<float> ranges      = laser_data.ranges;
  std::vector<float> intensities = laser_data.intensities;

  float len = ranges[500];

  count++;


  init_pose.header.seq      = count;
  init_pose.header.stamp    = ros::Time::now();
  init_pose.header.frame_id = "map";

  init_pose.pose.pose.position.x    = -len;
  init_pose.pose.pose.position.y    = -2.350 / 2;
  init_pose.pose.pose.position.z    = 0;
  init_pose.pose.pose.orientation.x = 0;
  init_pose.pose.pose.orientation.y = 0;
  init_pose.pose.pose.orientation.z = 0;
  init_pose.pose.pose.orientation.w = 1;


  boost::array<double, 36> arr;
  arr.assign(0);
  arr[0]  = 3;
  arr[7]  = 0.5;
  arr[35] = 0.2;

  init_pose.pose.covariance = arr;

  laser_update = true;
}
