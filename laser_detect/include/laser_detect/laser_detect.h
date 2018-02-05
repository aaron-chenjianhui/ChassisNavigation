#include "LineParamEstimator.h"
#include "RANSAC.h"
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <laser_detect/laser_detect_srv.h>
#include <laser_detect/detect_msg.h>

#include <Eigen/Eigen>
#include <boost/thread/mutex.hpp>

#include <vector>
#include <map>
#include <algorithm>
#include <numeric>

#include "LineParamEstimator.h"
#include "RANSAC.h"

#define DEG2RAD(X) X * 3.1415926 / 180
#define RAD2DEG(X) X * 180 / 3.1415926
#define M2MM(X) X * 1000
#define MM2M(X) X / 1000

typedef std::vector<double>           vec_data_type;
typedef std::vector<double>::iterator vec_iter_type;

// <angle_seq, lidar_data>
typedef std::map<double, double>                      lidar_data_type;
typedef std::map<double, double>::iterator            lidar_iter_type;
typedef std::deque<lidar_data_type>                   lidar_filter_type;
typedef std::deque<lidar_data_type>::iterator         filter_iter_type;
typedef std::deque<lidar_data_type>::reverse_iterator filter_riter_type;

// Eigen type for least square
typedef Eigen::Matrix<double, 2, 2>mat2x2;
typedef Eigen::Matrix<double, 2, 1>mat2x1;
typedef Eigen::Matrix<double, 3, 3>mat3x3;

namespace lms {
class LaserDetect {
public:

  LaserDetect();
  ~LaserDetect();

  void ParamInit();

  void DataSelect(double           min_angle,
                  double           max_angle,
                  lidar_data_type& lidar_data_in,
                  lidar_data_type& lidar_data_out);
  void DataFilter(lidar_data_type& lidar_data_in,
                  lidar_data_type& lidar_data_out);
  void DataIgn(vec_data_type  & ign_angle,
               lidar_data_type& lidar_data_in,
               lidar_data_type& lidar_data_out);

  // path_param: (nx, ny, ax, ay);        dest_pose: (dest_x, dest_y,
  // dest_theta)
  bool WallDetect(lidar_data_type& lidar_data,
                  vec_data_type  & l_param,
                  vec_data_type  & r_param,
                  vec_data_type  & f_param);
  void DataProcess(vec_data_type& l_param,
                   vec_data_type& r_param,
                   vec_data_type& f_param,
                   vec_data_type& ori_pose);

  //    void

  bool LaserSrvCallback(laser_detect::laser_detect_srv::Request & req,
                        laser_detect::laser_detect_srv::Response& res);
  void LaserDetectCallback(const sensor_msgs::LaserScan& laser_data);
  bool ResultCheck(vec_data_type& l_param,
                   vec_data_type& r_param,
                   vec_data_type& f_param);
  void ResultDisp(lidar_data_type& lidar_data,
                  vec_data_type  & ori_pose,
                  vec_data_type  & left_line_param,
                  vec_data_type  & right_line_param,
                  vec_data_type  & front_line_param);
  void BWFilter(lidar_filter_type& filter_data_in,
                lidar_filter_type& filter_data_out,
                vec_data_type    & Az,
                vec_data_type    & Bz);
  void MAFilter(lidar_filter_type& filter_data_in,
                lidar_data_type  & filter_data_out,
                int                ave_num);

  //
  template<typename M, typename V>
  void SelectLidarData(const M& m, V& v) {
    for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
      v.push_back(it->second);
    }
  }

  template<typename M, typename V>
  void SelectAngleData(const M& m, V& v) {
    for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
      v.push_back(it->first);
    }
  }

  template<typename S, typename P>
  void ScanToPoint(const S& s, P& p) {
    p.clear();

    for (typename S::const_iterator it = s.begin(); it != s.end(); ++it) {
      // auto new_x = (it->second)*cos(it->first);
      // auto new_y = (it->second)*sin(it->first);

      p.push_back(Point2D((it->second) * cos(it->first) * 1000,
                          (it->second) * sin(it->first) * 1000));
    }
  }

  //
  void PoseToMat(double  pose[3],
                 mat3x3& pose_mat);
  void MatToPose(mat3x3& pose_mat,
                 double  pose[3]);

private:

  // Boolean
  bool detect_flag; //
  // Mutex
  boost::mutex m_mutex;

  // Unit: m
  double m_lidar_min;
  double m_lidar_max;
  double m_lidar_narr_min;
  double m_lidar_narr_max;

  // Unit: mm
  double m_front_thre; // Front Threshold
  double m_front_dist; // Stop Position,


  ros::NodeHandle m_nh;
  ros::Subscriber m_suber;
  ros::Publisher m_puber;
  ros::ServiceServer m_srver;

  // For Check out Error
  ros::Publisher m_pose_puber;
  ros::Publisher m_filter_lidar_puber;
  ros::Publisher m_line_param_puber;
  ros::Publisher m_laser_pose_puber;

  // tf broadcast
  tf::TransformBroadcaster m_br;

  lidar_data_type lidar_data_raw;
  lidar_data_type lidar_data_filter;
  vec_data_type lidar_ang;

  // Filter Deque
  lidar_filter_type lidar_filter_in;
  lidar_filter_type lidar_filter_out;
  vec_data_type Az;
  vec_data_type Bz;

  // The order of low-pass filter
  int m_filter_order;

  // The number of  Moving Average Filter
  int m_ave_num;
};
}
