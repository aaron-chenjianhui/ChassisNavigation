#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

#include "LaserScanDataType.hpp"
#include "LaserFilter.hpp"
#include "MyPose2DRos.hpp"

int main(int argc, char *argv[]) {
  geometry_msgs::Pose2D  pose_msgs;
  sensor_msgs::LaserScan custom_msgs;

  LaserMAFilter filter(100);

  // LaserScanData laser_data;
  // LaserScanData laser_data_filtered;
  LaserScanData laser_data;
  LaserScanData laser_data_filtered;

  laser_data.UpdateData(custom_msgs);
  filter.Filtering(laser_data, laser_data_filtered);

  MyPose2DRos my_pose;
  my_pose.UpdatePose(pose_msgs);


  // LaserDataFac fac;
  // LaserData   *laser_data = fac.Create("laser_scan");
  //
  // laser_data->UpdateData(custom_msgs);

  return 0;
}
