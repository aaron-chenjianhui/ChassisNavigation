#include <ros/ros.h>

#include <unistd.h>
#include <vector>

#include "state/StateMachine.h"
#include "laser_msgs/status_srv.h"
#include "laser_msgs/driver_srv.h"

typedef enum {
  lidar_conn    = 1,
  lidar_disconn = 0
} lidar_status_t;
typedef enum {
  tcp_conn    = 1,
  tcp_disconn = 0
} tcp_status_t;

// TODO(CJH): add detect disconnected status
typedef enum {
  detect_ready   = 0,
  detect_cal_ing = 1,
  detect_cal_ok  = 2
} detect_status_t;

// TODO(CJH): add nav disconnected status
typedef enum {
  nav_ready = 0,
  nav_calib = 1,
  nav_ok    = 2
} nav_status_t;

// TODO(CJH): add msg disconnected status
typedef enum {
  msg_ready  = 0,
  msg_global = 1,
  msg_track  = 2
} msg_status_t;

// TODO(CJH): add error status
typedef enum {
  sys_init       = 0,
  sys_ready      = 1,
  sys_global_ing = 2,
  sys_global_ok  = 3,
  sys_track      = 4
} sys_status_t;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;

  int rate_hz = 50;
  ros::Rate loop_rate(rate_hz);

  // Monitoring service
  ros::ServiceClient lidar_client = nh.serviceClient<laser_msgs::status_srv>(
    "lidar_status");
  ros::ServiceClient detect_client = nh.serviceClient<laser_msgs::status_srv>(
    "detect_status");
  ros::ServiceClient nav_client = nh.serviceClient<laser_msgs::status_srv>(
    "nav_status");
  ros::ServiceClient driver_client = nh.serviceClient<laser_msgs::status_srv>(
    "driver_status");

  //
  laser_msgs::status_srv lidar_srv;
  laser_msgs::status_srv detect_srv;
  laser_msgs::status_srv nav_srv;
  laser_msgs::driver_srv driver_srv;

  lidar_status_t  lidar_status  = lidar_disconn;
  tcp_status_t    tcp_status    = tcp_disconn;
  detect_status_t detect_status = detect_ready;
  nav_status_t    nav_status    = nav_ready;
  msg_status_t    msg_status    = msg_ready;
  sys_status_t    sys_status    = sys_init;

  StateMachine_initialize();
  uint8_t system_state_init = StateMachine_Y.sys_state;
  sys_status = (sys_status_t)system_state_init;

  while (ros::ok()) {
    std::cout << "------------------ Loop Start------------------" << std::endl;

    //
    lidar_srv.request.sys_status  = sys_status;
    detect_srv.request.sys_status = sys_status;
    nav_srv.request.sys_status    = sys_status;
    driver_srv.request.sys_status = sys_status;

    // Monitoring lidar
    if (lidar_client.call(lidar_srv)) {
      uint8_t lidar_status_now = lidar_srv.response.node_status;
      lidar_status = (lidar_status_t)lidar_status_now;

      std::cout << "Lidar node status is: " << lidar_status << std::endl;
    }
    else {
      lidar_status = lidar_disconn;

      std::cout << "Lidar node encounters error" << std::endl;
    }

    // Monitoring detect node
    if (detect_client.call(detect_srv)) {
      uint8_t detect_status_now = lidar_srv.response.node_status;
      detect_status = (detect_status_t)detect_status_now;

      std::cout << "Detect node status is: " << detect_status << std::endl;
    }
    else {
      detect_status = detect_ready;

      // ROS_INFO("Detect node encounters error");
      std::cout << "Detect node encounters error" << std::endl;
    }

    // Monitoring nav node
    if (nav_client.call(nav_srv)) {
      uint8_t nav_status_now = nav_srv.response.node_status;
      nav_status = (nav_status_t)nav_status_now;
    }
    else {
      nav_status = nav_ready;
    }

    // Monitoring tcp & msg
    if (driver_client.call(driver_srv)) {
      std::vector<uint8_t> driver_status_now = driver_srv.response.driver_status;
      tcp_status = (tcp_status_t)driver_status_now[0];
      msg_status = (msg_status_t)driver_status_now[1];
    }
    else {
      tcp_status = tcp_disconn;
      msg_status = msg_ready;
    }

    // Update node status for StateMachine
    StateMachine_U.lidar_state       = lidar_status;
    StateMachine_U.tcp_state         = tcp_status;
    StateMachine_U.detect_node_state = detect_status;
    StateMachine_U.nav_node_state    = nav_status;
    StateMachine_U.recv_msg_state    = msg_status;

    // Update StateMachine
    StateMachine_step();

    // Output of StateMachine
    uint8_t system_status_now = StateMachine_Y.sys_state;
    sys_status = (sys_status_t)system_status_now;
    ROS_INFO("system state is: %d", system_status_now);

    std::cout << "------------------ Loop End ------------------" << std::endl;
    std::cout << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  StateMachine_terminate();

  return 0;
}
