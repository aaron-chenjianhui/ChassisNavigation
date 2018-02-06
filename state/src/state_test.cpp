#include <ros/ros.h>

#include <unistd.h>

#include "StateMachine.h"

int main(int argc, char *argv[]) {


  ros::init(argc, argv, "state_flow");
  ros::NodeHandle nh;

  int rate_hz = 50;
  ros::Rate loop_rate(rate_hz);

    StateMachine_initialize();
    int count = 0;

        uint8_t system_state_init = StateMachine_Y.sys_state;

  while (ros::ok()) {
      StateMachine_U.lidar_state       = 0;
      StateMachine_U.tcp_state         = 0;
      StateMachine_U.detect_node_state = 0;
      StateMachine_U.nav_node_state    = 0;
      StateMachine_U.recv_msg_state    = 0;
    // stateflow input data
     if (count >= 5 && count < 10){
         StateMachine_U.lidar_state       = 1;
         StateMachine_U.tcp_state         = 1;
         StateMachine_U.detect_node_state = 0;
         StateMachine_U.nav_node_state    = 0;
         StateMachine_U.recv_msg_state    = 0;
     }
     if (count >= 10 && count < 20){
         StateMachine_U.lidar_state       = 1;
         StateMachine_U.tcp_state         = 1;
         StateMachine_U.detect_node_state = 0;
         StateMachine_U.nav_node_state    = 0;
         StateMachine_U.recv_msg_state    = 1;
     }


    StateMachine_step();

    uint8_t system_state = StateMachine_Y.sys_state;
    ROS_INFO("system state is: %d", system_state);

    count ++;


    ros::spinOnce();
    loop_rate.sleep();
  }

  StateMachine_terminate();

  return 0;
}
