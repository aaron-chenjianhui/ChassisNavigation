#include <ros/ros.h>

#include <unistd.h>
#include <vector>

#include "state/CoreStateMachine.h"
#include "state/SysStateTypes.h"

#include "laser_msgs/lidar_srv.h"
#include "laser_msgs/nav_srv.h"
#include "laser_msgs/tcp_srv.h"
#include "laser_msgs/laser_detect_srv.h"


#define CAL_TIMES 20

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;

  int sys_freq;

  // nh.param<int>("sys_freq", sys_freq, 10);

  ros::param::param<int>("/state_machine/sys_freq",
                         sys_freq,
                         10);

  ros::Rate loop_rate(sys_freq);

  // create client for service
  ros::ServiceClient lidar_client =
    nh.serviceClient<laser_msgs::lidar_srv>(
      "lidar_status");
  ros::ServiceClient detect_client =
    nh.serviceClient<laser_msgs::laser_detect_srv>(
      "detect_status");
  ros::ServiceClient nav_client = nh.serviceClient<laser_msgs::nav_srv>(
    "nav_status");
  ros::ServiceClient tcp_client = nh.serviceClient<laser_msgs::tcp_srv>(
    "tcp_status");

  // create instances of srv files
  laser_msgs::lidar_srv lidar_status_srv;
  laser_msgs::laser_detect_srv detect_status_srv;
  laser_msgs::nav_srv nav_status_srv;
  laser_msgs::tcp_srv tcp_status_srv;

  // system status parameters(input)
  lidar_status_t lidar_conn_status  = lidar_disconn;
  tcp_status_t   tcp_conn_status    = tcp_disconn;
  nav_conn_status_t nav_conn_status = nav_disconn;
  detect_input_status_t detect_input_status;
  detect_input_status.detect_conn_status = detect_disconn;
  detect_input_status.filter_count       = (uint16_t)0;
  detect_input_status.cal_flag           = false;
  msg_status_t msg_status = no_msg;

  // system status parameters (output)
  sys_status_t sys_status = sys_init;
  nav_status_t nav_status = nav_err;
  detect_output_status_t detect_output_status;
  detect_output_status.detect_status = detect_err;
  detect_output_status.cal_flag      = false;

  // model parameter: cal times
  CoreStateMachine_P.counter_times = (uint16_t)CAL_TIMES;
  CoreStateMachine_initialize();

  // system initialization status
  uint8_t system_status_now   = CoreStateMachine_Y.sys_status;
  bool    cal_flag_output_now = CoreStateMachine_Y.detect_flag_output;
  uint8_t detect_status_now   = CoreStateMachine_Y.detect_status;
  uint8_t nav_status_now      = CoreStateMachine_Y.nav_status;

  // transfer StateMachine type to custome defined type
  sys_status                         = (sys_status_t)system_status_now;
  nav_status                         = (nav_status_t)nav_status_now;
  detect_output_status.detect_status = (detect_status_t)detect_status_now;
  detect_output_status.cal_flag      = cal_flag_output_now;


  uint16_t detect_count = 0;
  bool     detect_flag  = false;


  while (ros::ok()) {
    std::cout << "------------------ Loop Start------------------" << std::endl;

    // Monitoring tcp & msg
    // srv input parameter for tcp_driver node
    tcp_status_srv.request.sys_status = sys_status;

    if (tcp_client.call(tcp_status_srv)) {
      bool tcp_conn_now      = tcp_status_srv.response.tcp_conn;
      uint8_t msg_status_now = tcp_status_srv.response.msg_status;

      tcp_conn_status = (tcp_status_t)tcp_conn_now;
      msg_status      = (msg_status_t)msg_status_now;

      std::cout << "TCP conn status is: " << tcp_conn_status << std::endl;
      std::cout << "Msg status is: " << msg_status << std::endl;
    }
    else {
      tcp_conn_status = tcp_disconn;
      msg_status      = no_msg;

      std::cout << "TCP node encounters error" << std::endl;
    }


    // Monitoring lidar
    // srv input parameters for LMS1xx node
    lidar_status_srv.request.sys_status = sys_status;

    std::cout << "--------------------------" << std::endl;

    if (lidar_client.call(lidar_status_srv)) {
      uint8_t lidar_status_now = lidar_status_srv.response.lidar_conn;
      lidar_conn_status = (lidar_status_t)lidar_status_now;

      std::cout << "Lidar conn status is: " << lidar_conn_status << std::endl;
    }
    else {
      lidar_conn_status = lidar_disconn;

      std::cout << "Lidar node encounters error" << std::endl;
    }


    // Monitoring detect node
    // srv input parameters for laser_detect node
    detect_status_srv.request.sys_status     = sys_status;
    detect_status_srv.request.detect_status  = detect_output_status.detect_status;
    detect_status_srv.request.cal_flag_input = detect_output_status.cal_flag;

    std::cout << "--------------------------" << std::endl;

    if (detect_client.call(detect_status_srv)) {
      bool detect_conn_now         = detect_status_srv.response.detect_conn;
      uint16_t cal_count_now       = detect_status_srv.response.cal_count;
      bool     cal_flag_output_now = detect_status_srv.response.cal_flag_output;

      detect_input_status.detect_conn_status =
        (detect_conn_status_t)detect_conn_now;
      detect_input_status.filter_count = (uint16_t)cal_count_now;
      detect_input_status.cal_flag     =
        (bool)cal_flag_output_now;

      std::cout << "Detect conn status is: " << detect_conn_now << std::endl;
      std::cout << "Calculation count is: " << cal_count_now << std::endl;
      std::cout << "Calculation flag is: " << cal_flag_output_now << std::endl;
    }
    else {
      detect_input_status.detect_conn_status = detect_disconn;
      detect_input_status.filter_count       = (uint16_t)0;
      detect_input_status.cal_flag           = false;

      // ROS_INFO("Detect node encounters error");
      std::cout << "Detect node encounters error" << std::endl;
    }


    // laser_match_nav node
    nav_status_srv.request.sys_status = sys_status;
    nav_status_srv.request.nav_status = nav_status;

    std::cout << "--------------------------" << std::endl;

    // Monitoring nav node
    if (nav_client.call(nav_status_srv)) {
      uint8_t nav_status_now = nav_status_srv.response.nav_conn;

      nav_conn_status = (nav_conn_status_t)nav_status_now;

      std::cout << "Nav node conn status is: " << nav_conn_status << std::endl;
    }
    else {
      nav_conn_status = nav_disconn;

      std::cout << "Nav node encounters error" << std::endl;
    }


    // Update node status for StateMachine
    CoreStateMachine_U.detect_flag_input = detect_input_status.cal_flag;
    CoreStateMachine_U.filter_count      = detect_input_status.filter_count;
    CoreStateMachine_U.detect_conn       = detect_input_status.detect_conn_status;
    CoreStateMachine_U.nav_conn          = nav_conn_status;
    CoreStateMachine_U.lidar_conn        = lidar_conn_status;
    CoreStateMachine_U.tcp_conn          = tcp_conn_status;
    CoreStateMachine_U.recv_msg          = msg_status;


    // Update StateMachine
    CoreStateMachine_step();

    // Output of StateMachine
    system_status_now   = CoreStateMachine_Y.sys_status;
    cal_flag_output_now = CoreStateMachine_Y.detect_flag_output;
    detect_status_now   = CoreStateMachine_Y.detect_status;
    nav_status_now      = CoreStateMachine_Y.nav_status;

    //
    sys_status                         = (sys_status_t)system_status_now;
    nav_status                         = (nav_status_t)nav_status_now;
    detect_output_status.detect_status = (detect_status_t)detect_status_now;
    detect_output_status.cal_flag      = cal_flag_output_now;

    std::cout << "--------------------------" << std::endl;
    std::cout << "System output status is: " << sys_status << std::endl;
    std::cout << "Nav node output status is: " << nav_status << std::endl;
    std::cout << "Detect node output status is: " <<
    (uint16_t)detect_status_now <<
      std::endl;
    std::cout << "Cal flag output is: " << (bool)cal_flag_output_now << std::endl;

    std::cout << "------------------ Loop End ------------------" << std::endl;
    std::cout << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  CoreStateMachine_terminate();

  return 0;
}
