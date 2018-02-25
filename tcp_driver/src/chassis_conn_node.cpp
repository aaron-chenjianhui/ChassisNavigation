#include "ros/ros.h"

#include "SysStateTypes.h"

// Service headers
#include <geometry_msgs/Pose2D.h>
#include "laser_msgs/tcp_srv.h"

#include "chassis_conn.h"

#define _TCP_BUF_DEBUG 0


// State information
tcp_status_t tcp_status = tcp_disconn;
msg_status_t msg_status = no_msg;
sys_status_t sys_status = sys_init;

double pose_x, pose_y, pose_theta;

// status callback function
bool statusCallback(laser_msgs::tcp_srv::Request & req,
                    laser_msgs::tcp_srv::Response& res);
void simuCallback(const geometry_msgs::Pose2D& laser_pose);

int  main(int argc, char *argv[]) {
  // ROS related parameter
  ros::init(argc, argv, "chassis_conn");
  ros::NodeHandle n;

  ros::ServiceServer tcp_server =
    n.advertiseService("tcp_status", statusCallback);
  ros::Subscriber simu_server = n.subscribe("laser_pose", 1000, simuCallback);

  //  ros::Rate loop(20);
  ros::Rate loop_rate(10);

  // PLC related parameters
  conn::ChassisConn chassis;

  // Socket parameter
  std::string host;
  int port;

  //  n.param<std::string>("chassis_host", host, "127.0.0.1");
  //  n.param<int>(        "chassis_port", port, 2000);
  n.param<std::string>("chassis_host", host, "192.168.10.30");
  n.param<int>(        "chassis_port", port, 4097);


  while (ros::ok()) {
    // Connecting
    ROS_INFO_STREAM("Connecting to PLC at " << host);
    std::cout << "Connecting to PLC at " << host << std::endl;

    chassis.connect(host, port);

    if (!chassis.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      std::cout << "Unable to connect, retrying." << std::endl;

      tcp_status = tcp_disconn;
      msg_status = no_msg;

      ros::spinOnce();
      loop_rate.sleep();
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

      // Communication status
      u_char send_buf[100];
      u_char recv_buf[100];

      int recv_ret = -1;
      int send_ret = -1;

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
              msg_status = no_msg;

              std::cout << "Recv other msg" << std::endl;
            }
          }
          else {
            msg_status = no_msg;

            std::cout << "Recv buffer number is: 0" << std::endl;
          }
        }

        // If send to socket is OK
        if (sendOK) {
          int test_count       = 0;
          double x             = pose_x;
          double y             = pose_y;
          double theta         = pose_theta;
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
            send_ret  = chassis.SendMsg(pc_status);

            std::cout << "Send PC ready buffer success" << std::endl;
          }
          else if (sys_status == sys_global_ing) {
            pc_status = global_cal;
            send_ret  = chassis.SendMsg(pc_status);

            std::cout << "Send PC global cal buffer success" << std::endl;
          }
          else if (sys_status == sys_global_ok) {
            pc_status = global_ready;
            send_ret  = chassis.SendMsg(pc_status, location_data, container_data);

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
          tcp_status = tcp_conn;
          msg_status = no_msg;

          ROS_WARN("Data transfer failure: Timeout\r\n");
          std::cout << "Data transfer failure: Timeout" << std::endl;
        }
        else {
          tcp_status = tcp_conn;
          msg_status = no_msg;

          ROS_WARN("Data transfer failure: Error\r\n");
          std::cout << "Data transfer failure: Error" << std::endl;
        }
      }
      #endif // ifndef _TCP_BUF_DEBUG

      // Disconnected, need reconnect
      if ((-1 == recv_ret) || (-1 == send_ret)) {
        tcp_status = tcp_disconn;
        msg_status = no_msg;

        chassis.disconnect();

        //
        ros::spinOnce();
        loop_rate.sleep();
        break;
      }


      std::cout <<
        "--------------------------- Loop End ---------------------------" <<
        std::endl;
      std::cout << std::endl;

      //
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  chassis.disconnect();
  return 0;
}

bool statusCallback(laser_msgs::tcp_srv::Request & req,
                    laser_msgs::tcp_srv::Response& res) {
  std::cout << "in" << std::endl;
  uint16_t sys_status_now = req.sys_status;

  sys_status = (sys_status_t)sys_status_now;

  bool tcp_conn_now      = tcp_status;
  uint8_t msg_status_now = msg_status;

  res.tcp_conn   = tcp_conn_now;
  res.msg_status = msg_status_now;

  return true;
}

void simuCallback(const geometry_msgs::Pose2D& laser_pose) {
  pose_x     = laser_pose.x;
  pose_y     = laser_pose.y;
  pose_theta = laser_pose.theta;
}
