#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>


class SensorHandle {
private:

  ros::NodeHandle m_nh;
  ros::Subscriber m_imu_suber;
  ros::Subscriber m_pose_suber;
  ros::Publisher m_imu_puber;
  ros::Publisher m_pose_puber;

  double imu_ori_last;

public:

  SensorHandle() {
    m_imu_suber = m_nh.subscribe("xqserial_server/Twist", 1000,
                                 &SensorHandle::ImuCallback, this);
    m_pose_suber = m_nh.subscribe("xqserial_server/Pose2D", 1000,
                                  &SensorHandle::OdomCallback, this);
    m_imu_puber  = m_nh.advertise<sensor_msgs::Imu>("imu/data", 1000);
    m_pose_puber = m_nh.advertise<nav_msgs::Odometry>("odom", 1000);

    imu_ori_last = 0.0;
  }

  ~SensorHandle() {}

  void ImuCallback(const geometry_msgs::Twist& imu_data) {
    sensor_msgs::Imu imu_msgs;
    double imu_ang_vel = imu_data.angular.z;
    double imu_ori     = imu_ori_last + imu_ang_vel;

    // In ROS, Quaternion is represented as [x y z w]
    // double quat_ori[4];
    // quat_ori[3] = cos(imu_ori/2.0);
    // quat_ori[0] = 0.0;
    // quat_ori[1] = 0.0;
    // quat_ori[2] = sin(imu_ori/2.0);

    imu_msgs.orientation.x = 0.0;
    imu_msgs.orientation.y = 0.0;
    imu_msgs.orientation.z = sin(imu_ori / 2.0);
    imu_msgs.orientation.w = cos(imu_ori / 2.0);
    m_imu_puber.publish(imu_msgs);
  }

  void OdomCallback(const geometry_msgs::Pose2D& pose_data) {
    nav_msgs::Odometry odom_msgs;
    double odom_x   = pose_data.x;
    double odom_y   = pose_data.y;
    double odom_ori = pose_data.theta;

    odom_msgs.pose.pose.position.x    = odom_x;
    odom_msgs.pose.pose.position.y    = odom_y;
    odom_msgs.pose.pose.orientation.x = 0.0;
    odom_msgs.pose.pose.orientation.y = 0.0;
    odom_msgs.pose.pose.orientation.z = sin(odom_ori / 2.0);
    odom_msgs.pose.pose.orientation.w = cos(odom_ori / 2.0);
    m_pose_puber.publish(odom_msgs);
  }
};


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sensor_handle");

  ROS_INFO("Start sensor_handle_node\r\n");

  SensorHandle sensor_handle;

  ros::spin();
}
