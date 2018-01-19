#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <laser_detect/laser_detect_srv.h>

#include <Eigen/Eigen>
#include <boost/thread/mutex.hpp>

#include <fstream>


// 0: 停止； 1： 加速； 2： 匀速； 3： 减速； 4： 位置到位； 5：到位； 6： error
typedef enum ChassisStatus{
    STOP,
    SPEEDUP,
    SPEEDCONST,
    SPEEDSLOW,
    ADJUST,
    HOLDON,
    ERROR
}ChassisStatus;

typedef Eigen::Matrix<double, 3, 3> mat3x3;

namespace plan {
class ChassisPlanner{
public:
    ChassisPlanner();
    ChassisPlanner(double separation, double radius, double max_speed,
                   double accel_dist, double accel_time, double ctrl_period);
    ~ChassisPlanner();

    // State Machine
    void StatusDef();

    // Planning
    void PoseUpdateCallback(const geometry_msgs::Pose2D& chassis_pose);
    void AutoPlanner();

    // PID Algorithm
    double AngVelPID(double err, double err_integral, double last_err,
                     double kp, double ki, double kd);

    //
    void PoseToMat(double pose[3], mat3x3& pose_mat);
    void MatToPose(mat3x3& pose_mat, double pose[3]);

private:
    // 当前位姿
    double pose_x;
    double pose_y;
    double pose_theta;
    // 出发位姿
    double start_pose_x;
    double start_pose_y;
    double start_pose_theta;
    // 终点位姿
    double dest_pose_x;
    double dest_pose_y;
    double dest_pose_theta;
    // Pose Definition
    mat3x3 T_Ref_in_Ori;
    mat3x3 T_Laser_in_Ref;
    mat3x3 T_Laser_in_Ori;

    double head_dist;
    double m_ctrl_period;
    double m_separation, m_radius, m_max_speed, m_accel_dist, m_accel_time;
    double m_accel;   // 根据加减速时间计算的加速度
    double dist_thre, theta_thre;

    double last_linear_cmd, last_angular_cmd;
    double last_ang_dist_up, ang_integral_up;
    double last_ang_dist_slow, ang_integral_slow;
    double last_ang_dist_ave, ang_integral_ave;




    //
    ChassisStatus m_chassis_status;
    // 行驶方向
    int m_head_flag;
    // 根据启动该节点的时小车处在的位置
    bool m_b_pose_init;
    bool m_b_end_adjust;

    //
    ros::NodeHandle m_nh;
    ros::Subscriber m_pose_suber;
    ros::Publisher m_vel_puber;
    ros::ServiceClient m_client;
    geometry_msgs::Twist send_cmd;
    // tf broadcast
    tf::TransformBroadcaster m_br;

    //
    boost::mutex m_mutex;

    std::ofstream file;
};
}
