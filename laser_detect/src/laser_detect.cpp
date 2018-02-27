#include "laser_detect.h"

// Used for detect debug
#define DETECT_DEBUG 1

#define X_COMP 5.5
#define Y_COMP -16

namespace lms {
LaserDetect::LaserDetect() {
  //
  ParamInit();

  //
  m_suber = m_nh.subscribe("scan", 1000, &LaserDetect::LaserDetectCallback, this);
  m_puber = m_nh.advertise<sensor_msgs::PointCloud>("point", 1000);

  m_pose_puber          = m_nh.advertise<geometry_msgs::Pose2D>("ori_pose", 1000);
  m_container_len_puber =
    m_nh.advertise<std_msgs::Float32>("container_len", 1000);

  // For checkout error
  m_filter_lidar_puber = m_nh.advertise<sensor_msgs::LaserScan>("custom_scan",
                                                                1000);
  m_line_param_puber = m_nh.advertise<laser_msgs::detect_msg>("line_param",
                                                              1000);
  m_laser_pose_puber =
    m_nh.advertise<geometry_msgs::Pose2D>("debug_laser_pose", 1000);

  // For node status
  m_detect_server = m_nh.advertiseService("detect_status",
                                          &LaserDetect::statusCallback,
                                          this);
}

LaserDetect::~LaserDetect() {}

void LaserDetect::ParamInit() {
  // Boolean
  detect_flag = 0;

  // Unit: m
  m_lidar_max      = 10;
  m_lidar_min      = 0.8;
  m_lidar_narr_max = 2;
  m_lidar_narr_min = 0.8;

  // Unit: mm
  m_front_dist = 1000;
  m_front_thre = m_front_dist * 1.2;

  // ori pose
  m_ori_pose_x     = 0;
  m_ori_pose_y     = 0;
  m_ori_pose_theta = 0;

  // wall length
  m_right_length = 0;
  m_left_length  = 0;

  //
  lidar_filter_in.erase(lidar_filter_in.begin(), lidar_filter_in.end());
  lidar_filter_out.erase(lidar_filter_out.begin(), lidar_filter_out.end());

  double coeff_Az[] =
  { 1, -2.999548768734757, 2.999097639268598, -0.999548870522360 };
  double coeff_Bz[] =
  { 8.673617379884036e-20, 5.741326415393087e-12, 5.740463101634766e-12, 0 };

  if (sizeof(coeff_Az) == sizeof(coeff_Bz)) {
    int num = sizeof(coeff_Az) / sizeof(double);
    m_filter_order = num - 1;

    Az.insert(Az.begin(), coeff_Az, coeff_Az + num);
    Bz.insert(Bz.begin(), coeff_Bz, coeff_Bz + num);
  }
  else {
    ROS_INFO("Initialization Error!\r\n");
  }

  m_ave_num = 200;

  // status parameter
  m_detect_status = detect_ready;
  m_sys_status    = sys_init;
  m_filter_count  = 0;
  m_detect_flag   = 0;
}

/**
 * Obtain necessary
 * @param  lidar_data_cal [description]
 * @return                [description]
 */
bool LaserDetect::LaserCal(lidar_data_type& lidar_data_cal)
{
  //
  boost::mutex::scoped_lock lock(m_mutex);

  // detect result
  vec_data_type path_param;
  vec_data_type ori_pose;
  vec_data_type l_line_param, r_line_param, f_line_param;
  vec_data_type enter_param;
  double left_len, right_len;

  //
  if (WallDetect(lidar_data_cal, l_line_param, r_line_param, f_line_param,
                 enter_param)) {
    FindOriPose(l_line_param, r_line_param, f_line_param, ori_pose);
    FindWallLen(ori_pose, enter_param, left_len, right_len);

    m_ori_pose_x     = ori_pose[0];
    m_ori_pose_y     = ori_pose[1];
    m_ori_pose_theta = ori_pose[2];

    m_left_length  = left_len;
    m_right_length = right_len;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(m_ori_pose_x / 1000.0, m_ori_pose_y / 1000.0,
                                    0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, m_ori_pose_theta);
    transform.setRotation(q);
    m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                            "laser", "original"));

    return true;
  }
  else {
    // Detect Failed
    return false;
  }
}

void LaserDetect::LaserDetectCallback(const sensor_msgs::LaserScan& laser_data) {
  //
  boost::mutex::scoped_lock lock(m_mutex);

  // collection count
  m_filter_count++;

  if (m_filter_count > 10000) {
    m_filter_count = 0;
  }

  lidar_data_raw.clear();
  lidar_data_filter.clear();

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

  //
  // lidar_iter_type iter;
  lidar_ang.erase(lidar_ang.begin(), lidar_ang.end());
  std::vector<float>::iterator iter;
  double angle_now = angle_min;
  double lidar_data_now;

  for (iter = ranges.begin(); iter != ranges.end(); ++iter) {
    lidar_data_now = (double)(*iter);
    lidar_data_raw.insert(std::pair<double, double>(angle_now, lidar_data_now));
    lidar_ang.push_back(angle_now);

    angle_now += angle_increment;
  }

  //    // Put lidar_data_raw into filter deque
  //    lidar_filter_in.push_back(lidar_data_raw);
  //    if ((m_filter_order+1) < lidar_filter_in.size()){
  //        lidar_filter_in.pop_front();
  //    }

  //    BWFilter(lidar_filter_in, lidar_filter_out, Az, Bz);
  //    lidar_data_filter = lidar_filter_out.back();


  // Put lidar_data_raw into filter deque
  lidar_filter_in.push_back(lidar_data_raw);

  if (m_ave_num < lidar_filter_in.size()) {
    lidar_filter_in.pop_front();
  }

  MAFilter(lidar_filter_in, lidar_data_filter, m_ave_num);


  //
  if ((m_sys_status == sys_global_ok) || (m_sys_status == sys_track)) {
    // Publish pose data
    geometry_msgs::Pose2D ori_pose_msg;
    ori_pose_msg.x     = m_ori_pose_x;
    ori_pose_msg.y     = m_ori_pose_y;
    ori_pose_msg.theta = m_ori_pose_theta;
    m_pose_puber.publish(ori_pose_msg);

    std_msgs::Float32 container_len;
    container_len.data = (m_left_length + m_right_length) / 2.0;
    m_container_len_puber.publish(container_len);
  }

#if DETECT_DEBUG

  // For Display
  lidar_data_type lidar_data_disp = lidar_data_filter;

  // publish custom data, here it is the lidar data after filted
  sensor_msgs::LaserScan custom_msgs;
  std::vector<float> custom_data;
  lidar_iter_type    custom_iter;

  for (custom_iter =
         lidar_data_disp.begin(); custom_iter != lidar_data_disp.end();
       ++custom_iter) {
    custom_data.push_back((float)(custom_iter->second));
  }
  custom_msgs.header.frame_id = "laser";
  custom_msgs.header.seq      = laser_data.header.seq;
  custom_msgs.angle_min       = laser_data.angle_min;
  custom_msgs.angle_max       = laser_data.angle_max;
  custom_msgs.angle_increment = laser_data.angle_increment;
  custom_msgs.time_increment  = laser_data.time_increment;
  custom_msgs.scan_time       = laser_data.scan_time;
  custom_msgs.range_min       = laser_data.range_min;
  custom_msgs.range_max       = laser_data.range_max;
  custom_msgs.ranges          = custom_data;
  custom_msgs.intensities     = laser_data.intensities;

  m_filter_lidar_puber.publish(custom_msgs);


  // detection for custom data
  vec_data_type path_param;
  vec_data_type ori_pose;
  vec_data_type l_line_param, r_line_param, f_line_param;
  vec_data_type enter_param;
  double left_len, right_len;

  if (WallDetect(lidar_data_disp, l_line_param, r_line_param, f_line_param,
                 enter_param)) {
    FindOriPose(l_line_param, r_line_param, f_line_param, ori_pose);
    FindWallLen(ori_pose, enter_param, left_len, right_len);
    ResultDisp(lidar_data_disp, ori_pose, l_line_param, r_line_param,
               f_line_param);

    // Publish original pose
    geometry_msgs::Pose2D ori_pose_msg;
    ori_pose_msg.x     = ori_pose[0];
    ori_pose_msg.y     = ori_pose[1];
    ori_pose_msg.theta = ori_pose[2];
    m_pose_puber.publish(ori_pose_msg);

    // Publish laser pose in original coordinate
    geometry_msgs::Pose2D laser_pose_msg;
    mat3x3 T_Ori_in_Laser;
    mat3x3 T_Laser_in_Ori;
    double pose_Ori_in_Laser[3];
    double pose_Laser_in_Ori[3];
    pose_Ori_in_Laser[0] = ori_pose[0];
    pose_Ori_in_Laser[1] = ori_pose[1];
    pose_Ori_in_Laser[2] = ori_pose[2];

    PoseToMat(pose_Ori_in_Laser, T_Ori_in_Laser);
    T_Laser_in_Ori = T_Ori_in_Laser.inverse();
    MatToPose(T_Laser_in_Ori, pose_Laser_in_Ori);
    laser_pose_msg.x     = pose_Laser_in_Ori[0];
    laser_pose_msg.y     = pose_Laser_in_Ori[1];
    laser_pose_msg.theta = pose_Laser_in_Ori[2];
    m_laser_pose_puber.publish(laser_pose_msg);

    // Publish line parameter
    laser_msgs::detect_msg line_msg;
    line_msg.right_line_param = r_line_param;
    line_msg.left_line_param  = l_line_param;
    line_msg.front_line_param = f_line_param;
    m_line_param_puber.publish(line_msg);


    // Broadcast original tf tree
    tf::Transform ori_transform;
    ori_transform.setOrigin(tf::Vector3(ori_pose[0] / 1000.0,
                                        ori_pose[1] / 1000.0,
                                        0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, ori_pose[2]);
    ori_transform.setRotation(q);
    m_br.sendTransform(tf::StampedTransform(ori_transform, ros::Time::now(),
                                            "laser",
                                            "ori_debug"));

    // container length
    double container_len = (left_len + right_len) / 2;
    std::cout << "Container length is: " << container_len << std::endl;

    // Broadcast enter tf tree
    double l_ang  = enter_param[0];
    double l_data = enter_param[1];
    double r_ang  = enter_param[2];
    double r_data = enter_param[3];
    double l_x    = l_data * cos(l_ang);
    double l_y    = l_data * sin(l_ang);
    double r_x    = r_data * cos(r_ang);
    double r_y    = r_data * sin(r_ang);

    //    std::cout << "l_data is: " << l_data << std::endl;
    //    std::cout << "l_ang is: " << l_ang << std::endl;

    // std::cout << "r_x is: " << r_x << std::endl;
    // std::cout << "r_x is: " << r_x << std::endl;

    tf::Transform r_enter_transform;
    r_enter_transform.setOrigin(tf::Vector3(r_x, r_y, 0.0));
    tf::Quaternion r_enter_q;
    r_enter_q.setRPY(0, 0, ori_pose[2]);
    r_enter_transform.setRotation(r_enter_q);
    m_br.sendTransform(tf::StampedTransform(r_enter_transform, ros::Time::now(),
                                            "laser",
                                            "r_enter_debug"));

    tf::Transform l_enter_transform;
    l_enter_transform.setOrigin(tf::Vector3(l_x, l_y, 0.0));
    tf::Quaternion l_enter_q;
    l_enter_q.setRPY(0, 0, ori_pose[2]);
    l_enter_transform.setRotation(r_enter_q);
    m_br.sendTransform(tf::StampedTransform(l_enter_transform, ros::Time::now(),
                                            "laser",
                                            "l_enter_debug"));
  }
  else {
    ROS_INFO("Detect Error\r\n");
  }
#endif // if DETECT_DEBUG
}

/**
 * [LaserDetect::BWFilter description]
 * @param filter_data_in  [description]
 * @param filter_data_out [description]
 * @param Az              [description]
 * @param Bz              [description]
 */
void LaserDetect::BWFilter(lidar_filter_type& filter_data_in,
                           lidar_filter_type& filter_data_out,
                           vec_data_type    & Az,
                           vec_data_type    & Bz) {
  if ((sizeof(Az) == sizeof(Bz)) &&
      ((m_filter_order + 1) == filter_data_in.size())) {
    lidar_data_type lidar_data_now;
    lidar_iter_type lidar_iter;

    //
    filter_riter_type filter_in_riter;
    filter_riter_type filter_out_riter;
    vec_iter_type     az_iter;
    vec_iter_type     bz_iter;

    double angle_now;
    double data_now;

    // Loop for every angle
    lidar_iter_type iter;
    int angle_index = 0;

    for (iter =
           filter_data_in.back().begin(); iter != filter_data_in.back().end();
         ++iter, ++angle_index) {
      double in_sum  = 0;
      double out_sum = 0;

      // Sum of filter_data_in
      filter_in_riter = filter_data_in.rbegin();
      bz_iter         = Bz.begin();

      for (; bz_iter != Bz.end(); ++bz_iter, ++filter_in_riter) {
        lidar_iter = (*filter_in_riter).begin();

        for (int i = 0; i < angle_index;
             ++i, ++lidar_iter) in_sum += (*bz_iter) * (lidar_iter->second);
      }

      // Sum of filter_data_out
      filter_out_riter = filter_data_out.rbegin();
      az_iter          = Az.begin() + 1;

      for (; az_iter != Az.end(); ++az_iter, ++filter_out_riter) {
        lidar_iter = (*filter_out_riter).begin();

        for (int i = 0; i < angle_index;
             ++i, ++lidar_iter) out_sum += (*az_iter) * (lidar_iter->second);
      }

      //
      angle_now = iter->first;
      data_now  = (in_sum - out_sum) / Az[0];

      lidar_data_now.insert(std::pair<double, double>(angle_now, data_now));
    }

    // Push lidar_data_now into filter_data_out deque
    filter_data_out.push_back(lidar_data_now);

    if ((m_filter_order + 1) < filter_data_out.size()) {
      filter_data_out.pop_front();
    }
  }

  // Mean Value filter for first m_filter_order+1 data
  else if ((sizeof(Az) == sizeof(Bz)) &&
           (m_filter_order >= filter_data_in.size())) {
    lidar_data_type lidar_data_now;
    lidar_iter_type iter;

    int filter_num = filter_data_in.size();

    //
    vec_data_type filter_data_sum(filter_data_in.back().size());
    vec_iter_type sum_iter;

    filter_iter_type filter_iter;

    for (filter_iter =
           filter_data_in.begin(); filter_iter != filter_data_in.end();
         ++filter_iter) {
      vec_data_type filter_data_select;
      vec_iter_type select_iter;

      SelectLidarData(*filter_iter, filter_data_select);

      sum_iter = filter_data_sum.begin();

      for (select_iter = filter_data_select.begin();
           select_iter != filter_data_select.end(); ++select_iter) {
        *sum_iter += *select_iter;
        ++sum_iter;
      }
    }

    sum_iter = filter_data_sum.begin();
    double angle_now;
    double data_now;

    for (iter =
           filter_data_in.back().begin(); iter != filter_data_in.back().end();
         ++iter) {
      angle_now = iter->first;
      data_now  = (*sum_iter) / filter_num;

      lidar_data_now.insert(std::pair<double, double>(angle_now, data_now));

      ++sum_iter;
    }

    // Push lidar_data_now into filter_data_out deque
    filter_data_out.push_back(lidar_data_now);

    if ((m_filter_order + 1) < filter_data_out.size()) {
      filter_data_out.pop_front();
    }
  }
  else {}
}

void LaserDetect::MAFilter(lidar_filter_type& filter_data_in,
                           lidar_data_type& filter_data_out, int ave_num) {
  filter_data_out.erase(filter_data_out.begin(), filter_data_out.end());

  if (ave_num >= filter_data_in.size()) {
    //
    vec_data_type lidar_sum;
    lidar_sum.resize(lidar_ang.size());
    vec_iter_type sum_iter;

    // Loop for every angle
    filter_iter_type num_iter;
    lidar_iter_type  ang_iter;

    for (num_iter = filter_data_in.begin(); num_iter != filter_data_in.end();
         ++num_iter) {
      sum_iter = lidar_sum.begin();

      for (ang_iter = (*num_iter).begin(); ang_iter != (*num_iter).end();
           ++ang_iter, ++sum_iter) {
        (*sum_iter) += ang_iter->second;
      }
    }

    //
    double angle_now;
    double data_now;
    vec_iter_type iter = lidar_ang.begin();

    for (sum_iter = lidar_sum.begin(); sum_iter != lidar_sum.end();
         ++sum_iter, ++iter) {
      angle_now = *iter;
      data_now  = (*sum_iter) / (filter_data_in.size());

      filter_data_out.insert(std::pair<double, double>(angle_now, data_now));
    }
  }
  else {}
}

void LaserDetect::DataSelect(double           min_angle,
                             double           max_angle,
                             lidar_data_type& lidar_data_in,
                             lidar_data_type& lidar_data_out) {
  if (min_angle < max_angle) {
    lidar_iter_type iter_low;
    lidar_iter_type iter_up;

    lidar_data_out.clear();

    iter_low = lidar_data_in.lower_bound(min_angle);
    iter_up  = lidar_data_in.upper_bound(max_angle);

    // if (iter_low < iter_up){
    lidar_data_out.insert(iter_low, iter_up);

    // }
  }
  else {
    ROS_ERROR("Angle Range Wrong!\r\n");
  }
}

// Ignore Unreasonable Data
void LaserDetect::DataFilter(lidar_data_type& lidar_data_in,
                             lidar_data_type& lidar_data_out) {
  lidar_iter_type iter;

  lidar_data_out.clear();

  for (iter = lidar_data_in.begin(); iter != lidar_data_in.end(); ++iter) {
    if ((iter->first > DEG2RAD(90)) || (iter->first < DEG2RAD(-90))) {
      if ((iter->second > m_lidar_narr_min) &&
          (iter->second < m_lidar_narr_max)) {
        lidar_data_out.insert(*iter);
      }
    }
    else {
      if ((iter->second > m_lidar_min) || (iter->second < m_lidar_max)) {
        lidar_data_out.insert(*iter);
      }
    }
  }
}

//
void LaserDetect::DataIgn(vec_data_type  & ign_angle,
                          lidar_data_type& lidar_data_in,
                          lidar_data_type& lidar_data_out) {
  vec_iter_type   max_iter;
  vec_iter_type   min_iter;
  vec_iter_type   vec_iter_now;
  lidar_iter_type iter;

  bool ign_flag = 0;

  lidar_data_out.clear();
  vec_iter_now = ign_angle.begin();

  min_iter = std::min_element(ign_angle.begin(), ign_angle.end());
  max_iter = std::max_element(ign_angle.begin(), ign_angle.end());

  for (iter = lidar_data_in.begin(); iter != lidar_data_in.end(); ++iter) {
    if ((iter->first > (*max_iter)) || (iter->first < (*min_iter))) {
      lidar_data_out.insert(*iter);
    }
    else {
      vec_iter_type vec_iter;

      for (vec_iter = vec_iter_now; vec_iter != ign_angle.end(); ++iter) {
        if (abs(*vec_iter - iter->first) < 0.001) {
          ign_flag = 1;
          break;
        }
      }

      if (0 == ign_flag) {
        lidar_data_out.insert(*iter);
      }
      else {
        ++vec_iter_now;
      }
      ign_flag = 0;
    }
  }
}

void LaserDetect::FindUltiData(const lidar_data_type  & lidar_data,
                               const std::vector<bool>& vote_index,
                               double                 & min_angle,
                               double                 & min_data,
                               double                 & max_angle,
                               double                 & max_data) {
  //
  vec_data_type vote_angle;
  vec_data_type vote_lidar_data;

  // Voted Angle
  const_lidar_iter_type iter                  = lidar_data.begin();
  std::vector<bool>::const_iterator vote_iter = vote_index.begin();

  for (; iter != lidar_data.end(); ++iter) {
    if (true == *vote_iter) {
      vote_angle.push_back(iter->first);
      vote_lidar_data.push_back(iter->second);
    }
    ++vote_iter;
  }

  min_angle = vote_angle.front();
  max_angle = vote_angle.back();
  min_data  = vote_lidar_data.front();
  max_data  = vote_lidar_data.back();


  // f_angle_max = *(std::max_element(f_vote_angle.begin(),
  // f_vote_angle.end()));
  // f_angle_min = *(std::min_element(f_vote_angle.begin(),
  // f_vote_angle.end()));
}

//
bool LaserDetect::ResultCheck(vec_data_type& left_line_param,
                              vec_data_type& right_line_param,
                              vec_data_type& front_line_param) {
  if (left_line_param.empty() || right_line_param.empty() ||
      front_line_param.empty()) {
    return false;
  }

  // Final Check
  double r_n_x = right_line_param[0];
  double r_n_y = right_line_param[1];
  double r_a_x = right_line_param[2];
  double r_a_y = right_line_param[3];
  double l_n_x = left_line_param[0];
  double l_n_y = left_line_param[1];
  double l_a_x = left_line_param[2];
  double l_a_y = left_line_param[3];
  double f_n_x = front_line_param[0];
  double f_n_y = front_line_param[1];
  double f_a_x = front_line_param[2];
  double f_a_y = front_line_param[3];

  // Left & Right Check
  double l_r_parrel_thre = 0.05;
  double l_r_parrel_err  = fabs(r_n_x * l_n_y - r_n_y * l_n_x);
  bool   l_r_pos_check   = l_a_y > r_a_y;
  bool   l_r_flag        = (l_r_parrel_err < l_r_parrel_thre) && (l_r_pos_check);

  // Left & front check
  double l_f_vert_thre = 0.1;
  double l_f_vert_err  = fabs(f_n_x * l_n_x + f_n_y * l_n_y);
  bool   l_f_flag      = l_f_vert_err < l_f_vert_thre;

  // Right & front check
  double r_f_vert_thre = 0.1;
  double r_f_vert_err  = fabs(r_n_x * f_n_x + r_n_y * f_n_y);
  bool   r_f_flag      = r_f_vert_err < r_f_vert_thre;

  return true;

  //  return l_r_flag & l_f_flag & r_f_flag;
}

void LaserDetect::FindWallLen(const vec_data_type& ori_pose,
                              const vec_data_type& enter_param,
                              double             & l_len,
                              double             & r_len) {
  double ori_x     = ori_pose[0];
  double ori_y     = ori_pose[1];
  double ori_theta = ori_pose[2];
  double l_ang     = enter_param[0];
  double l_data    = enter_param[1] * 1000;
  double r_ang     = enter_param[2];
  double r_data    = enter_param[3] * 1000;

  mat3x3 T_Ori_in_Laser;
  mat3x1 T_L_in_Laser;
  mat3x1 T_R_in_Laser;
  mat3x1 T_L_in_Ori;
  mat3x1 T_R_in_Ori;

  T_Ori_in_Laser(0, 0) = cos(ori_theta);
  T_Ori_in_Laser(0, 1) = -sin(ori_theta);
  T_Ori_in_Laser(0, 2) = ori_x;
  T_Ori_in_Laser(1, 0) = sin(ori_theta);
  T_Ori_in_Laser(1, 1) = cos(ori_theta);
  T_Ori_in_Laser(1, 2) = ori_y;
  T_Ori_in_Laser(2, 0) = T_Ori_in_Laser(2, 1) = 0;
  T_Ori_in_Laser(2, 2) = 1;

  T_L_in_Laser(0) = l_data * cos(l_ang);
  T_L_in_Laser(1) = l_data * sin(l_ang);
  T_L_in_Laser(2) = 1;

  T_R_in_Laser(0) = r_data * cos(r_ang);
  T_R_in_Laser(1) = r_data * sin(r_ang);
  T_R_in_Laser(2) = 1;

  T_L_in_Ori = T_Ori_in_Laser.inverse() * T_L_in_Laser;
  T_R_in_Ori = T_Ori_in_Laser.inverse() * T_R_in_Laser;

  l_len = abs(T_L_in_Ori(0));
  r_len = abs(T_R_in_Ori(0));
}

void LaserDetect::FindOriPose(vec_data_type& l_line_param,
                              vec_data_type& r_line_param,
                              vec_data_type& f_line_param,
                              vec_data_type& ori_pose) {
  double r_n_x, r_n_y, r_a_x, r_a_y;
  double l_n_x, l_n_y, l_a_x, l_a_y;
  double f_n_x, f_n_y, f_a_x, f_a_y;
  double r_theta, l_theta, ori_theta;

  //
  mat2x2 ori_A;
  mat2x1 ori_B;
  mat2x1 ori_pos; // original: (X, Y)

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

  // Original Position
  ori_A(0, 0) = f_n_x;
  ori_A(0, 1) = f_n_y;
  ori_A(1, 0) = l_n_x;
  ori_A(1, 1) = l_n_y;
  ori_B(0, 0) = f_n_x * f_a_x + f_n_y * f_a_y;
  ori_B(1, 0) = l_n_x * l_a_x + l_n_y * l_a_y;
  ori_pos     = ori_A.inverse() * ori_B;

  double raw_x  = ori_pos(0, 0);
  double raw_y  = ori_pos(1, 0);
  double real_x = raw_x + X_COMP;
  double real_y = raw_y + Y_COMP;

  ori_pose.push_back(real_x);
  ori_pose.push_back(real_y);

  // Original Orientation
  r_theta   = -atan(r_n_x / r_n_y);
  l_theta   = -atan(l_n_x / l_n_y);
  ori_theta = (r_theta + l_theta) / 2;
  ori_pose.push_back(ori_theta);
}

// path_param: (nx, ny, ax, ay);        dest_pose: (dest_x, dest_y, dest_theta)

/**
 * Detect walls
 * @param  lidar_data  [description]
 * @param  l_param     [description]
 * @param  r_param     [description]
 * @param  f_param     [description]
 * @param  enter_param (l_ang, l_data, r_ang, r_data)
 * @return             [description]
 */
bool LaserDetect::WallDetect(lidar_data_type& lidar_data,
                             vec_data_type  & l_param,
                             vec_data_type  & r_param,
                             vec_data_type  & f_param,
                             vec_data_type  & enter_param) {
  // Lidar Data
  lidar_data_type lidar_filter;
  lidar_data_type lidar_left;
  lidar_data_type lidar_right;
  lidar_data_type lidar_front;
  lidar_data_type lidar_judge;


  vec_data_type judge_data;

  // Lidar Data for final Ransac
  lidar_data_type front_select_lidar;
  lidar_data_type left_select_lidar;
  lidar_data_type right_select_lidar;

  // Left, Right, Front Range
  double left_min = DEG2RAD(0);

  //  double left_max  = DEG2RAD(134);
  double left_max = DEG2RAD(60);

  //  double right_min = DEG2RAD(-134);
  double right_min = DEG2RAD(-60);
  double right_max = DEG2RAD(0);
  double front_min = DEG2RAD(-90);
  double front_max = DEG2RAD(90);
  double judge_min = DEG2RAD(-45);
  double judge_max = DEG2RAD(45);

  // First Ransac Angle
  double r_angle_min, r_angle_max;
  double l_angle_min, l_angle_max;
  double f_angle_min, f_angle_max;

  // Final Ransac  Angle
  double r_fin_angle_min, r_fin_angle_max;
  double l_fin_angle_min, l_fin_angle_max;
  double f_fin_angle_min, f_fin_angle_max;

  // RANSAC Data
  std::vector<double>  right_line_param;
  std::vector<double>  left_line_param;
  std::vector<double>  front_line_param;
  std::vector<Point2D> right_points;
  std::vector<Point2D> left_points;
  std::vector<Point2D> front_points;

  //
  std::vector<bool> r_vote_index;
  std::vector<bool> l_vote_index;
  std::vector<bool> f_vote_index;
  std::vector<bool>::iterator r_vote_iter;
  std::vector<bool>::iterator l_vote_iter;
  std::vector<bool>::iterator f_vote_iter;

  vec_data_type r_vote_angle;
  vec_data_type l_vote_angle;
  vec_data_type f_vote_angle;

  // Final Parameter
  double r_n_x, r_n_y, r_a_x, r_a_y;
  double l_n_x, l_n_y, l_a_x, l_a_y;
  double f_n_x, f_n_y, f_a_x, f_a_y;

  // Left & Right "enter-point"
  double r_select_angle, r_select_data;
  double l_select_angle, l_select_data;

  // 0:left, right    1:left(right), front    2:front    3:error
  unsigned int chassis_pos_mode = 0;

  // RANSAC Parameters
  LineParamEstimator lpEstimator(40); // for a point to be on the line it has to
                                      // be closer than 0.5 units from the line
  double desiredProbabilityForNoOutliers = 0.999;

  // Data Pre-Processing
  DataFilter(lidar_data, lidar_filter);

  DataSelect(left_min,  left_max,  lidar_filter, lidar_left);
  DataSelect(right_min, right_max, lidar_filter, lidar_right);
  DataSelect(front_min, front_max, lidar_filter, lidar_front);
  DataSelect(judge_min, judge_max, lidar_filter, lidar_judge);

  ScanToPoint(lidar_right, right_points);
  ScanToPoint(lidar_left,  left_points);
  ScanToPoint(lidar_front, front_points);


  // Position Judgemen
  SelectLidarData(lidar_judge, judge_data);
  double judge_sum  = std::accumulate(judge_data.begin(), judge_data.end(), 0);
  double judge_mean = judge_sum / (judge_data.end() - judge_data.begin()) * 1000;

  if (judge_mean < m_front_thre) {
    chassis_pos_mode = 2;
  }


  // Find Lines
  if (2 == chassis_pos_mode) {
    front_select_lidar = lidar_front;
    double f_usedData = RANSAC<Point2D, double>::compute(front_line_param,
                                                         &lpEstimator,
                                                         front_points,
                                                         desiredProbabilityForNoOutliers,
                                                         f_vote_index);

    // Voted Angle
    lidar_iter_type iter;
    f_vote_iter = f_vote_index.begin();

    for (iter = lidar_front.begin(); iter != lidar_front.end(); ++iter) {
      if (true == *f_vote_iter) {
        f_vote_angle.push_back(iter->first);
      }
      ++f_vote_iter;
    }
    f_angle_max = *(std::max_element(f_vote_angle.begin(), f_vote_angle.end()));
    f_angle_min = *(std::min_element(f_vote_angle.begin(), f_vote_angle.end()));

    // Final left  & right Angle
    l_fin_angle_min = f_angle_max;
    l_fin_angle_max = left_max;
    r_fin_angle_min = right_min;
    r_fin_angle_max = f_angle_min;

    DataSelect(l_fin_angle_min, l_fin_angle_max, lidar_filter, left_select_lidar);
    DataSelect(r_fin_angle_min, r_fin_angle_max, lidar_filter,
               right_select_lidar);

    ScanToPoint(left_select_lidar,  left_points);
    ScanToPoint(right_select_lidar, right_points);

    double r_usedData = RANSAC<Point2D, double>::compute(right_line_param,
                                                         &lpEstimator,
                                                         right_points,
                                                         desiredProbabilityForNoOutliers,
                                                         r_vote_index);
    double l_usedData = RANSAC<Point2D, double>::compute(left_line_param,
                                                         &lpEstimator,
                                                         left_points,
                                                         desiredProbabilityForNoOutliers,
                                                         l_vote_index);
  }
  else {
    right_select_lidar = lidar_right;
    left_select_lidar  = lidar_left;
    double r_usedData = RANSAC<Point2D, double>::compute(right_line_param,
                                                         &lpEstimator,
                                                         right_points,
                                                         desiredProbabilityForNoOutliers,
                                                         r_vote_index);
    double l_usedData = RANSAC<Point2D, double>::compute(left_line_param,
                                                         &lpEstimator,
                                                         left_points,
                                                         desiredProbabilityForNoOutliers,
                                                         l_vote_index);

    // Voted Angle
    lidar_iter_type iter;
    r_vote_iter = r_vote_index.begin();

    for (iter = lidar_right.begin(); iter != lidar_right.end(); ++iter) {
      if (true == *r_vote_iter) {
        r_vote_angle.push_back(iter->first);
      }
      ++r_vote_iter;
    }
    r_angle_max = *(std::max_element(r_vote_angle.begin(), r_vote_angle.end()));
    r_angle_min = *(std::min_element(r_vote_angle.begin(), r_vote_angle.end()));

    l_vote_iter = l_vote_index.begin();

    for (iter = lidar_left.begin(); iter != lidar_left.end(); ++iter) {
      if (true == *l_vote_iter) {
        l_vote_angle.push_back(iter->first);
      }
      ++l_vote_iter;
    }
    l_angle_max = *(std::max_element(l_vote_angle.begin(), l_vote_angle.end()));
    l_angle_min = *(std::min_element(l_vote_angle.begin(), l_vote_angle.end()));

    // Line Parameter
    r_n_x = right_line_param[0];
    r_n_y = right_line_param[1];
    r_a_x = right_line_param[2];
    r_a_y = right_line_param[3];
    l_n_x = left_line_param[0];
    l_n_y = left_line_param[1];
    l_a_x = left_line_param[2];
    l_a_y = left_line_param[3];

    // Mode judgement
    if ((fabs(r_n_x * l_n_y - r_n_y * l_n_x) < 0.05) && (l_a_y > r_a_y)) {
      chassis_pos_mode = 0;
    }
    else if (fabs(r_n_x * l_n_x + r_n_y * l_n_y) < 0.1) {
      double l_angle_pos = fabs((l_angle_min + l_angle_max) / 2.0 - left_min) /
                           (left_max - left_min);
      double r_angle_pos = fabs((r_angle_min + r_angle_max) / 2.0 - right_min) /
                           (right_max - right_min);

      if (l_angle_pos < r_angle_pos) {
        chassis_pos_mode = 11; // front, right
      }
      else {
        chassis_pos_mode = 12; // left, front
      }
    }
    else {
      chassis_pos_mode = 3;
    }

    // Run Ransac
    if (0 == chassis_pos_mode) {
      f_fin_angle_min = r_angle_max;
      f_fin_angle_max = l_angle_min;
      DataSelect(f_fin_angle_min,
                 f_fin_angle_max,
                 lidar_filter,
                 front_select_lidar);

      //
      ScanToPoint(front_select_lidar, front_points);

      double f_usedData = RANSAC<Point2D, double>::compute(front_line_param,
                                                           &lpEstimator,
                                                           front_points,
                                                           desiredProbabilityForNoOutliers,
                                                           f_vote_index);
    }
    else if (11 == chassis_pos_mode) {
      left_line_param.clear();
      l_vote_index.clear();

      f_fin_angle_min = r_angle_max;
      f_fin_angle_max = l_angle_max;
      l_fin_angle_min = l_angle_max;
      l_fin_angle_max = left_max;

      DataSelect(f_fin_angle_min,
                 f_fin_angle_max,
                 lidar_filter,
                 front_select_lidar);
      DataSelect(l_fin_angle_min, l_fin_angle_max, lidar_filter,
                 left_select_lidar);

      ScanToPoint(front_select_lidar, front_points);
      ScanToPoint(left_select_lidar,  left_points);

      double f_usedData = RANSAC<Point2D, double>::compute(front_line_param,
                                                           &lpEstimator,
                                                           front_points,
                                                           desiredProbabilityForNoOutliers,
                                                           f_vote_index);
      double l_usedData = RANSAC<Point2D, double>::compute(left_line_param,
                                                           &lpEstimator,
                                                           left_points,
                                                           desiredProbabilityForNoOutliers,
                                                           l_vote_index);
    }
    else if (12 == chassis_pos_mode) {
      right_line_param.clear();
      r_vote_index.clear();

      f_fin_angle_min = r_angle_min;
      f_fin_angle_max = l_angle_min;
      r_fin_angle_min = r_angle_min;
      r_fin_angle_max = right_min;

      DataSelect(f_fin_angle_min,
                 f_fin_angle_max,
                 lidar_filter,
                 front_select_lidar);
      DataSelect(r_fin_angle_min,
                 r_fin_angle_max,
                 lidar_filter,
                 right_select_lidar);

      ScanToPoint(front_select_lidar, front_points);
      ScanToPoint(right_select_lidar, right_points);

      double f_usedData = RANSAC<Point2D, double>::compute(front_line_param,
                                                           &lpEstimator,
                                                           front_points,
                                                           desiredProbabilityForNoOutliers,
                                                           f_vote_index);
      double r_usedData = RANSAC<Point2D, double>::compute(right_line_param,
                                                           &lpEstimator,
                                                           right_points,
                                                           desiredProbabilityForNoOutliers,
                                                           r_vote_index);
    }
    else {
      f_fin_angle_min = r_angle_max;
      f_fin_angle_max = l_angle_min;
      DataSelect(f_fin_angle_min,
                 f_fin_angle_max,
                 lidar_filter,
                 front_select_lidar);

      //
      ScanToPoint(front_select_lidar, front_points);

      double f_usedData = RANSAC<Point2D, double>::compute(front_line_param,
                                                           &lpEstimator,
                                                           front_points,
                                                           desiredProbabilityForNoOutliers,
                                                           f_vote_index);
    }
  }


  // Result Checking
  bool re = ResultCheck(left_line_param, right_line_param, front_line_param);

  if (true == re) {
    //
    double r_min_angle, r_max_angle, l_min_angle, l_max_angle;
    double r_min_data, r_max_data, l_min_data, l_max_data;

    FindUltiData(right_select_lidar,
                 r_vote_index,
                 r_min_angle,
                 r_min_data,
                 r_max_angle,
                 r_max_data);
    FindUltiData(left_select_lidar,
                 l_vote_index,
                 l_min_angle,
                 l_min_data,
                 l_max_angle,
                 l_max_data);
    r_select_angle = r_min_angle;
    r_select_data  = r_min_data;
    l_select_angle = l_max_angle;
    l_select_data  = l_max_data;
    enter_param.push_back(l_select_angle);
    enter_param.push_back(l_select_data);
    enter_param.push_back(r_select_angle);
    enter_param.push_back(r_select_data);

    l_param = left_line_param;
    r_param = right_line_param;
    f_param = front_line_param;

    return true;
  }
  else {
    ROS_INFO("ERROR\r\n");
    return false;
  }
}

void LaserDetect::PoseToMat(double pose[], mat3x3& pose_mat) {
  double pose_x     = pose[0];
  double pose_y     = pose[1];
  double pose_theta = pose[2];
  double sin_theta  = sin(pose_theta);
  double cos_theta  = cos(pose_theta);

  pose_mat(0, 2) = pose_x;
  pose_mat(1, 2) = pose_y;
  pose_mat(0, 0) = cos_theta;
  pose_mat(0, 1) = -sin_theta;
  pose_mat(1, 0) = sin_theta;
  pose_mat(1, 1) = cos_theta;
  pose_mat(2, 0) = 0;
  pose_mat(2, 1) = 0;
  pose_mat(2, 2) = 1;
}

void LaserDetect::MatToPose(mat3x3& pose_mat, double pose[]) {
  double pose_x, pose_y, pose_theta;

  pose_x     = pose_mat(0, 2);
  pose_y     = pose_mat(1, 2);
  pose_theta = atan(pose_mat(1, 0) / pose_mat(0, 0));

  pose[0] = pose_x;
  pose[1] = pose_y;
  pose[2] = pose_theta;
}

void LaserDetect::ResultDisp(lidar_data_type& lidar_data,
                             vec_data_type  & ori_pose,
                             vec_data_type  & left_line_param,
                             vec_data_type  & right_line_param,
                             vec_data_type  & front_line_param) {
  // Display Results
  sensor_msgs::PointCloud cloud;
  geometry_msgs::Point32  single_point;
  std::vector<float> intensity_value;
  std::vector<geometry_msgs::Point32> points_value;

  cloud.header.stamp    = ros::Time::now();
  cloud.header.frame_id = "laser";
  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";


  //    lidar_data_type data_disp = front_select_lidar;
  lidar_data_type data_disp = lidar_data;
  lidar_iter_type iter;

  for (iter = data_disp.begin(); iter != data_disp.end(); ++iter) {
    single_point.x = (iter->second) * cos(iter->first);
    single_point.y = (iter->second) * sin(iter->first);
    single_point.z = 0;
    points_value.push_back(single_point);

    intensity_value.push_back(500);
  }


  //
  double n_x, n_y, a_x, a_y;
  double start_x, start_y, end_x, end_y, incre_x, incre_y;

  int num = 500;

  // Front
  n_x = front_line_param[0];
  n_y = front_line_param[1];
  a_x = front_line_param[2] / 1000.0;
  a_y = front_line_param[3] / 1000.0;

  start_x = a_x + 4 * n_y;
  start_y = a_y - 4 * n_x;
  end_x   = a_x - 4 * n_y;
  end_y   = a_y + 4 * n_x;
  incre_x = (end_x - start_x) / (double)num;
  incre_y = (end_y - start_y) / (double)num;

  for (int i = 0; i < num; ++i) {
    single_point.x = start_x + incre_x * i;
    single_point.y = start_y + incre_y * i;
    single_point.z = 0;

    points_value.push_back(single_point);
    intensity_value.push_back(100);
  }

  // Right
  n_x = right_line_param[0];
  n_y = right_line_param[1];
  a_x = right_line_param[2] / 1000.0;
  a_y = right_line_param[3] / 1000.0;

  start_x = a_x + 4 * n_y;
  start_y = a_y - 4 * n_x;
  end_x   = a_x - 4 * n_y;
  end_y   = a_y + 4 * n_x;
  incre_x = (end_x - start_x) / (double)num;
  incre_y = (end_y - start_y) / (double)num;

  for (int i = 0; i < num; ++i) {
    single_point.x = start_x + incre_x * i;
    single_point.y = start_y + incre_y * i;
    single_point.z = 0;

    points_value.push_back(single_point);
    intensity_value.push_back(200);
  }

  // Left
  n_x = left_line_param[0];
  n_y = left_line_param[1];
  a_x = left_line_param[2] / 1000.0;
  a_y = left_line_param[3] / 1000.0;

  start_x = a_x + 4 * n_y;
  start_y = a_y - 4 * n_x;
  end_x   = a_x - 4 * n_y;
  end_y   = a_y + 4 * n_x;
  incre_x = (end_x - start_x) / (double)num;
  incre_y = (end_y - start_y) / (double)num;

  for (int i = 0; i < num; ++i) {
    single_point.x = start_x + incre_x * i;
    single_point.y = start_y + incre_y * i;
    single_point.z = 0;

    points_value.push_back(single_point);
    intensity_value.push_back(300);
  }

  cloud.points             = points_value;
  cloud.channels[0].values = intensity_value;

  m_puber.publish(cloud);
}

bool LaserDetect::statusCallback(laser_msgs::laser_detect_srv::Request & req,
                                 laser_msgs::laser_detect_srv::Response& res) {
  // transfer system status to node
  uint8_t system_status_now = req.sys_status;
  uint8_t detect_status_now = req.detect_status;
  bool    cal_flag_input    = req.cal_flag_input;

  m_sys_status    = (sys_status_t)system_status_now;
  m_detect_status = (detect_status_t)detect_status_now;

  //
  if (cal_flag_input && (m_detect_status == detect_cal_ing)) {
    bool ret = LaserCal(lidar_data_filter);

    if (ret) {
      m_detect_flag = true;
    }
    else {
      m_detect_flag = false;
    }
  }

  // cal_flag for res
  if (detect_cal_ing != m_detect_status) {
    m_detect_flag = false;
  }
  bool cal_flag_output = cal_flag_input & (!m_detect_flag);

  // detect_conn for res
  bool detect_conn = true;

  // cal_count for res
  uint16_t cal_count = m_filter_count;

  // service return
  res.detect_conn     = detect_conn;
  res.cal_count       = m_filter_count;
  res.cal_flag_output = cal_flag_output;

  return true;
}
}
