#include <fstream>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <laser_msgs/detect_msg.h>

// typedef (unsigned int) uint32_t;

typedef std::vector<double>           vec_data_type;
typedef std::vector<double>::iterator vec_iter_type;


void LmsSaveCallback(const sensor_msgs::LaserScan& laser_data);
void CustomSaveCallback(const sensor_msgs::LaserScan& custom_data);
void DetectLineCallback(const laser_msgs::detect_msg& line_data);
void OriPoseCallback(const geometry_msgs::Pose2D& ori_pose);
void LaserPoseCallback(const geometry_msgs::Pose2D& laser_pose);
void LogSaveCallback(const sensor_msgs::LaserScan& laser_data);


template<typename S, typename P>
void DownSample(const S& s, const uint32_t interval, P& p) {
  p.clear();

  uint32_t s_size = s.size();

  if ((s_size % 2) == 1) {
    for (typename S::const_iterator it = s.begin(); it != s.end();) {
      p.push_back(*it);

      for (uint32_t i = 0; i < interval; ++i) {
        ++it;
      }
    }
  }
  else if ((s_size % 2) == 0) {}
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lms_saver");
  ROS_INFO("Start Lidar data saving\r\n");

  ros::NodeHandle nh;
  ros::Subscriber lidar_suber;
  ros::Subscriber custom_suber;
  ros::Subscriber pose_suber;
  ros::Subscriber line_suber;
  ros::Subscriber laser_pose_suber;
  ros::Subscriber log_pose_suber;


  lidar_suber  = nh.subscribe("scan", 1000, LmsSaveCallback);
  custom_suber = nh.subscribe("custom_scan", 1000, CustomSaveCallback);

  // log_pose_suber = nh.subscribe("scan", 1000, LogSaveCallback);

  // pose_suber   = nh.subscribe("laser_pose", 1000, OriPoseCallback);
  pose_suber       = nh.subscribe("debug_laser_pose", 1000, OriPoseCallback);
  line_suber       = nh.subscribe("line_param", 1000, DetectLineCallback);
  laser_pose_suber = nh.subscribe("laser_pose", 1000, LaserPoseCallback);


  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();


  return 0;
}

void LogSaveCallback(const sensor_msgs::LaserScan& laser_data) {
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

  uint32_t interval        = 2;
  double   angle_range     = angle_max - angle_min;
  double   range_increment = 0.001;
  int num                  = (int)((angle_max - angle_min) / angle_increment) + 1;

  std::vector<float> ranges_sample;

  DownSample(ranges, interval, ranges_sample);

  std::ofstream out_file;

  out_file.open("./data/custom.log", std::ofstream::out | std::ofstream::app);

  if (out_file.is_open()) {
    ROS_INFO("Writing laser data!!!\r\n");

    //
    out_file << "ROBOTLASER1 " << "0 ";
    out_file << angle_min << " " << angle_range << " ";
    out_file << angle_increment << " " << range_max << " ";
    out_file << range_increment << " " << num << " ";


    // 写入ranges
    std::vector<float>::iterator it;

    for (it = ranges_sample.begin(); it != ranges_sample.end() - 1; ++it) {
      out_file << *it << " ";
    }

    // remissionNumber
    out_file << "0 ";

    for (int i = 0; i < 11; ++i) {
      out_file << "0 ";
    }

    // timestamp
    out_file << seq << " " << "x" << std::endl;
  }
}

void LmsSaveCallback(const sensor_msgs::LaserScan& laser_data) {
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

  std::ofstream out_file;

  out_file.open("./data/lidar_data.txt", std::ofstream::out | std::ofstream::app);

  if (out_file.is_open()) {
    ROS_INFO("Writing laser data!!!\r\n");

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

    for (it = ranges.begin(); it != ranges.end() - 1; ++it) {
      out_file << *it << ", ";
    }
    out_file << *it << std::endl;

    // 写入intensities
    out_file << "[intensities]" << std::endl;

    for (it = intensities.begin(); it != intensities.end() - 1; ++it) {
      out_file << *it << ", ";
    }
    out_file << *it << std::endl;

    //
    out_file << "[tail]" << std::endl << std::endl;

    // 关闭文件
    out_file.close();
  }
}

void CustomSaveCallback(const sensor_msgs::LaserScan& custom_data) {
  unsigned int seq               = custom_data.header.seq;
  double angle_min               = custom_data.angle_min;
  double angle_max               = custom_data.angle_max;
  double angle_increment         = custom_data.angle_increment;
  double time_increment          = custom_data.time_increment;
  double scan_time               = custom_data.scan_time;
  double range_min               = custom_data.range_min;
  double range_max               = custom_data.range_max;
  std::vector<float> ranges      = custom_data.ranges;
  std::vector<float> intensities = custom_data.intensities;

  std::ofstream out_file;

  out_file.open("./data/custom_data.txt",
                std::ofstream::out | std::ofstream::app);

  if (out_file.is_open()) {
    ROS_INFO("Writing custom laser data!!!\r\n");

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

    for (it = ranges.begin(); it != ranges.end() - 1; ++it) {
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

void OriPoseCallback(const geometry_msgs::Pose2D& ori_pose) {
  double x, y, theta;

  x     = ori_pose.x;
  y     = ori_pose.y;
  theta = ori_pose.theta;

  std::ofstream out_file;
  out_file.open("./data/ori_data.txt", std::ofstream::out | std::ofstream::app);

  if (out_file.is_open()) {
    ROS_INFO("Writing pose data!!!\r\n");

    //
    out_file << "[X Y Theta]" << std::endl;
    out_file << x << " " << y << " " << " " << theta << std::endl;

    // 关闭文件
    out_file.close();
  }
}

void LaserPoseCallback(const geometry_msgs::Pose2D& laser_pose) {
  double x, y, theta;

  x     = laser_pose.x;
  y     = laser_pose.y;
  theta = laser_pose.theta;

  std::ofstream out_file;
  out_file.open("./data/laser_pose_data.txt",
                std::ofstream::out | std::ofstream::app);

  if (out_file.is_open()) {
    ROS_INFO("Writing laser pose data!!!\r\n");

    //
    out_file << "[X Y Theta]" << std::endl;
    out_file << x << " " << y << " " << " " << theta << std::endl;

    // 关闭文件
    out_file.close();
  }
}

void DetectLineCallback(const laser_msgs::detect_msg& line_data) {
  double r_n_x, r_n_y, r_a_x, r_a_y;
  double l_n_x, l_n_y, l_a_x, l_a_y;
  double f_n_x, f_n_y, f_a_x, f_a_y;

  vec_data_type r_line_param;
  vec_data_type l_line_param;
  vec_data_type f_line_param;

  r_line_param = line_data.right_line_param;
  l_line_param = line_data.left_line_param;
  f_line_param = line_data.front_line_param;

  r_n_x = r_line_param[0];
  r_n_y = r_line_param[1];
  r_a_x = r_line_param[2];
  r_a_y = r_line_param[3];
  l_n_x = l_line_param[0];
  l_n_y = l_line_param[1];
  l_a_x = l_line_param[2];
  l_a_y = l_line_param[3];
  f_n_x = f_line_param[0];
  f_n_y = f_line_param[1];
  f_a_x = f_line_param[2];
  f_a_y = f_line_param[3];

  std::ofstream out_file;
  out_file.open("./data/line_param.txt", std::ofstream::out | std::ofstream::app);

  if (out_file.is_open()) {
    ROS_INFO("Writing line parameter!!!\r\n");

    //
    out_file <<
      "[r_n_x r_n_y r_a_x r_a_y l_n_x l_n_y l_a_x l_a_y f_n_x f_n_y f_a_x f_a_y]"
             <<
      std::endl;
    out_file << r_n_x << " " << r_n_y << " " << r_a_x << " " << r_a_y << " ";
    out_file << l_n_x << " " << l_n_y << " " << l_a_x << " " << l_a_y << " ";
    out_file << f_n_x << " " << f_n_y << " " << f_a_x << " " << f_a_y << " ";
    out_file << std::endl;

    // 关闭文件
    out_file.close();
  }
}
