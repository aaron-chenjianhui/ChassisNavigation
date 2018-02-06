#include "ros/ros.h"

#include "tcp_driver/chassis_conn.h"

#include "tcp_driver/StateMachine.h"

#define _TCP_BUF_DEBUG 0

int test_count = 0;

typedef enum {
  lidar_conn    = 1,
  lidar_disconn = 0
} lidar_status_t;
typedef enum {
  tcp_conn    = 1,
  tcp_disconn = 0
} tcp_status_t;
typedef enum {
  detect_ready   = 0,
  detect_cal_ing = 1,
  detect_cal_ok  = 2
} detect_status_t;
typedef enum {
  nav_ready = 0,
  nav_calib = 1,
  nav_ok    = 2
} nav_status_t;
typedef enum {
  msg_ready  = 0,
  msg_global = 1,
  msg_track  = 2
} msg_status_t;
typedef enum {
  sys_init       = 0,
  sys_ready      = 1,
  sys_global_ing = 2,
  sys_global_ok  = 3,
  sys_track      = 4
} sys_status_t;


int main(int argc, char *argv[]) {
  // ROS related parameter
  ros::init(argc, argv, "chassis_conn");
  ros::NodeHandle n("~");

  //  ros::Rate loop(20);
  ros::Rate loop(10);


  StateMachine_initialize();


  // PLC related parameters
  conn::ChassisConn chassis;

  std::string host;
  int port;


  //  n.param<std::string>("chassis_host", host, "127.0.0.1");
  //  n.param<int>(        "chassis_port", port, 2000);

  n.param<std::string>("chassis_host", host, "192.168.10.30");
  n.param<int>(        "chassis_port", port, 4097);

  // Socket parameter


  while (ros::ok()) {
    // status Parameters
    lidar_status_t  lidar_status  = lidar_conn;
    tcp_status_t    tcp_status    = tcp_disconn;
    detect_status_t detect_status = detect_ready;
    nav_status_t    nav_status    = nav_ready;
    msg_status_t    msg_status    = msg_ready;

    sys_status_t sys_status = sys_init;

    int detect_tick = 0;

    // Statemachine update
    StateMachine_U.lidar_state       = lidar_status;
    StateMachine_U.tcp_state         = tcp_status;
    StateMachine_U.detect_node_state = detect_status;
    StateMachine_U.nav_node_state    = nav_status;
    StateMachine_U.recv_msg_state    = msg_status;
    StateMachine_step();

    uint8_t output = (uint8_t)StateMachine_Y.sys_state;
    sys_status = (sys_status_t)output;


    // Connecting
    ROS_INFO_STREAM("Connecting to PLC at " << host);
    std::cout << "Connecting to PLC at " << host << std::endl;

    chassis.connect(host, port);

    if (!chassis.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      std::cout << "Unable to connect, retrying." << std::endl;

      ros::Duration(1).sleep();
      continue;
    }
    ROS_INFO("Connected");
    std::cout << "Connected" << std::endl;

    //
    tcp_status = tcp_conn;

    // Send & Recv Data
    while (ros::ok()) {
      //
      std::cout <<
        "--------------------------- Loop Start ---------------------------" <<
        std::endl;

      // system status
      std::cout << "In this loop, system status is: " << sys_status << std::endl;

      // Update node status
      lidar_status = lidar_conn;

      // Detect node simu
      detect_status = detect_ready;
      nav_status    = nav_ready;

      if (sys_status == sys_global_ok) {
        detect_status = detect_cal_ok;
        nav_status    = nav_ok;
      }

      if (sys_status == sys_global_ing) {
        detect_status = detect_cal_ing;
        nav_status    = nav_calib;
        detect_tick++;

        if (detect_tick > 15) {
          detect_status = detect_cal_ok;
          nav_status    = nav_ok;
          detect_tick   = 0;
        }
      }


      // Communication status
      u_char send_buf[100];
      u_char recv_buf[100];

      int recv_ret = 1;
      int send_ret = 1;

      // char send_buf[] = {0x02, 0x11, 0x22, 0xAB, 0xCD, 0xEF};
      bool sendOK = false;
      bool recvOK = false;
      int  re     = chassis.WRSelect(recvOK, sendOK);

#if _TCP_BUF_DEBUG

      if (re > 0) {
        // If receive from socket is OK
        if (recvOK) {
          recv_ret = chassis.ReadBuf(recv_buf, sizeof(recv_buf));

          if (1 == recv_ret) {
            char int_buf[4];
            int  recv_data_first, recv_data_second, recv_data_three;

            memcpy((char *)&recv_data_first,  recv_buf,     4);
            memcpy((char *)&recv_data_second, recv_buf + 4, 4);
            memcpy((char *)&recv_data_three,  recv_buf + 8, 4);

            std::cout << "recv buf int is: " << recv_data_first <<
              std::endl;
            std::cout << "recv buf int is: " << recv_data_second <<
              std::endl;
            std::cout << "recv buf int is: " << recv_data_three <<
              std::endl;
          }
          else {
            std::cout << "ReadError" << std::endl;
          }
        }

        // If send to socket is OK
        if (sendOK) {
          int send_first  = 3400;
          int send_second = -23400;
          memcpy(send_buf,                      (u_char *)&send_first,
                 sizeof(send_first));
          memcpy(send_buf + sizeof(send_first), (u_char *)&send_second,
                 sizeof(send_second));

          send_ret = chassis.WriteBuf(send_buf, 8);

          if (1 == send_ret) {
            std::cout << "Send Buffer Success" << std::endl;
          }
          else {
            std::cout << "Send Buffer Failed" << std::endl;
          }
        }
      }
#else // if _TCP_BUF_DEBUG

      if (re > 0) {
        // If receive from socket is OK
        if (recvOK) {
          chassis_status_t chassis_status;
          OdomData odom_data;
          recv_ret = chassis.RecvMsg(chassis_status, odom_data);

          if (1 == recv_ret) {
            if (chassis_status == chassis_ready) {
              msg_status = msg_ready;

              std::cout << "Recv ready msg" << std::endl;
            }
            else if (chassis_status == global_cmd) {
              msg_status = msg_global;

              std::cout << "Recv global msg" << std::endl;
            }
            else if (chassis_status == track_cmd) { // track_cmd
              msg_status = msg_track;

              std::cout << "Recv track msg" << std::endl;
            }
            else {
              std::cout << "Recv other msg" << std::endl;
            }
          }
          else {
            std::cout << "Recv buffer number is: 0" << std::endl;
          }
        }

        // If send to socket is OK
        if (sendOK) {
          double x             = 988.234 + test_count * 0.5;
          double y             = 50.681 + test_count * 0.5;
          double theta         = 15.391 + test_count * 0.5;
          double container_len = 12000.810 + test_count * 0.5;


          pc_status_t pc_status;

          LocationData location_data;
          location_data.x     = (int)(x * 1000);
          location_data.y     = (int)(y * 1000);
          location_data.theta = (int)(theta * 1000);

          ContainerData container_data;
          container_data.container_length = (int)(container_len * 1000);


          if (sys_status == sys_ready) {
            pc_status = pc_ready;

            //              pc_status = global_cal;
            send_ret = chassis.SendMsg(pc_status);

            std::cout << "Send PC ready buffer success" << std::endl;
          }
          else if (sys_status == sys_global_ing) {
            pc_status = global_cal;
            send_ret  = chassis.SendMsg(pc_status);

            std::cout << "Send PC global cal buffer success" << std::endl;
          }
          else if (sys_status == sys_global_ok) {
            pc_status = global_ready;

            send_ret = chassis.SendMsg(pc_status, location_data, container_data);

            test_count++;

            std::cout << "Send PC global ready success" << std::endl;
            std::cout << "X is: " << location_data.x << "    "
                      << "Y is: " << location_data.y << "    "
                      << "Theta is: " << location_data.theta << std::endl;
            std::cout << "Container length is: " <<
              container_data.container_length << std::endl;
          }
          else if (sys_status == sys_track) {
            pc_status = track_ready;
            send_ret  = chassis.SendMsg(pc_status, location_data);

            test_count++;

            std::cout << "Send PC track ready success" << std::endl;
            std::cout << "X is: " << location_data.x << "    "
                      << "Y is: " << location_data.y << "    "
                      << "Theta is: " << location_data.theta << std::endl;
          }

          // return check
          if (1 == send_ret) {
            //            std::cout << "Send Buffer Success" << std::endl;
          }
          else {
            std::cout << "Send Buffer Failed" << std::endl;
          }
        }
      }
      else {
        if (0 == re) {
          ROS_WARN("Data transfer failure: Timeout\r\n");
          std::cout << "Data transfer failure: Timeout" << std::endl;
        }
        else {
          ROS_WARN("Data transfer failure: Error\r\n");
          std::cout << "Data transfer failure: Error" << std::endl;
        }

        tcp_status = tcp_disconn;

        // chassis.disconnect();
        // break;
      }

      #endif // ifndef _TCP_BUF_DEBUG

      // Disconnected, need reconnect
      if ((-1 == recv_ret) || (-1 == send_ret)) {
        tcp_status = tcp_disconn;

        // chassis.disconnect();
        // break;
      }

      // Statemachine update
      StateMachine_U.lidar_state       = lidar_status;
      StateMachine_U.tcp_state         = tcp_status;
      StateMachine_U.detect_node_state = detect_status;
      StateMachine_U.nav_node_state    = nav_status;
      StateMachine_U.recv_msg_state    = msg_status;
      StateMachine_step();

      // OUTPUT
      uint8_t output = (uint8_t)StateMachine_Y.sys_state;
      sys_status = (sys_status_t)output;

      std::cout << "Now, system status updates to " << sys_status << std::endl;

      if (sys_status == sys_init) {
        std::cout << "Socket disconnected" << std::endl;
        chassis.disconnect();
        break;
      }

      std::cout <<
        "--------------------------- Loop End ---------------------------" <<
        std::endl;
      std::cout << std::endl;

      //
      loop.sleep();
    }
  }

  chassis.disconnect();
  StateMachine_terminate();
  return 0;
}
