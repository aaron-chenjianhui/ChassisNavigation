#include <fstream>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>


void LmsSaveCallback(const sensor_msgs::LaserScan &laser_data);
void CustomSaveCallback(const sensor_msgs::LaserScan &custom_data);


int main(int argc, char* argv[]){
  ros::init(argc, argv, "lms_saver");
  ROS_INFO("Start Lidar data saving\r\n");

  ros::NodeHandle nh;
  ros::Subscriber lidar_suber;
  ros::Subscriber custom_suber;

  lidar_suber = nh.subscribe("scan", 1000, LmsSaveCallback);
  custom_suber = nh.subscribe("custom_scan", 1000, CustomSaveCallback);


  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();


  return 0;
}



void LmsSaveCallback(const sensor_msgs::LaserScan &laser_data){
  unsigned int seq = laser_data.header.seq;
  double angle_min = laser_data.angle_min;
  double angle_max = laser_data.angle_max;
  double angle_increment = laser_data.angle_increment;
  double time_increment = laser_data.time_increment;
  double scan_time = laser_data.scan_time;
  double range_min = laser_data.range_min;
  double range_max = laser_data.range_max;
  std::vector<float> ranges = laser_data.ranges;
  std::vector<float> intensities = laser_data.intensities;

  std::ofstream out_file;
  out_file.open("./data/lidar_data.txt", std::ofstream::out | std::ofstream::app);

  if(out_file.is_open()){
    ROS_INFO("Writing!!!\r\n");
    //
    out_file << "[seq]" << std::endl << seq << std::endl;
    //
    out_file << "[angle_min]" << std::endl << angle_min << std::endl;
    //
    out_file << "[angle_max]" << std::endl << angle_max << std::endl;
    //
    out_file << "[angle_increment]" << std::endl << angle_increment << std::endl;
    //
    out_file << "[time_increment]" << std::endl << time_increment << std::endl;
    //
    out_file << "[scan_time]" << std::endl << scan_time << std::endl;
    //
    out_file << "[range_min]" << std::endl << range_min << std::endl;
    //
    out_file << "[range_max]" << std::endl << range_max << std::endl;

    // 写入ranges
    std::vector<float>::iterator it;
    out_file << "[ranges]" << std::endl;
    for(it = ranges.begin(); it != ranges.end()-1; ++it){
      out_file << *it << ", ";
    }
    out_file << *it << std::endl;
    // 写入intensities
    out_file << "[intensities]" << std::endl;
    for(it = intensities.begin(); it != intensities.end()-1; ++it){
      out_file << *it << ", ";
    }
    out_file << *it << std::endl;
    //
    out_file << "[tail]" << std::endl << std::endl;

    // 关闭文件
    out_file.close();
  }
}


  void CustomSaveCallback(const sensor_msgs::LaserScan &custom_data){
      unsigned int seq = custom_data.header.seq;
      double angle_min = custom_data.angle_min;
      double angle_max = custom_data.angle_max;
      double angle_increment = custom_data.angle_increment;
      double time_increment = custom_data.time_increment;
      double scan_time = custom_data.scan_time;
      double range_min = custom_data.range_min;
      double range_max = custom_data.range_max;
      std::vector<float> ranges = custom_data.ranges;
      std::vector<float> intensities = custom_data.intensities;

      std::ofstream out_file;
      out_file.open("./data/custom_data.txt", std::ofstream::out | std::ofstream::app);

      if(out_file.is_open()){
        ROS_INFO("Writing!!!\r\n");
        //
        out_file << "[seq]" << std::endl << seq << std::endl;
        //
        out_file << "[angle_min]" << std::endl << angle_min << std::endl;
        //
        out_file << "[angle_max]" << std::endl << angle_max << std::endl;
        //
        out_file << "[angle_increment]" << std::endl << angle_increment << std::endl;
        //
        out_file << "[time_increment]" << std::endl << time_increment << std::endl;
        //
        out_file << "[scan_time]" << std::endl << scan_time << std::endl;
        //
        out_file << "[range_min]" << std::endl << range_min << std::endl;
        //
        out_file << "[range_max]" << std::endl << range_max << std::endl;

        // 写入ranges
        std::vector<float>::iterator it;
        out_file << "[ranges]" << std::endl;
        for(it = ranges.begin(); it != ranges.end()-1; ++it){
          out_file << *it << ", ";
        }
        out_file << *it << std::endl;
        // 写入intensities
        // out_file << "[intensities]" << std::endl;
        // for(it = intensities.begin(); it != intensities.end()-1; ++it){
        //   out_file << *it << ", ";
        // }
        // out_file << *it << std::endl;
        //
        out_file << "[tail]" << std::endl << std::endl;

        // 关闭文件
        out_file.close();
      }
  }
