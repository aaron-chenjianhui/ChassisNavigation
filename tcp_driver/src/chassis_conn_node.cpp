#include "ros/ros.h"

#include "tcp_driver/chassis_conn.h"

#define _TCP_BUF_DEBUG 0


int main(int argc, char *argv[]) {
  // ROS related parameter
  ros::init(argc, argv, "chassis_conn");
  ros::NodeHandle n("~");
  ros::Rate loop(1);

  // PLC related parameters
  conn::ChassisConn chassis;

  std::string host;
  int port;


  n.param<std::string>("chassis_host", host, "127.0.0.1");
  n.param<int>(        "chassis_port", port, 2000);

  // n.param<std::string>("chassis_host", host, "192.168.10.30");
  // n.param<int>("chassis_port", port, 4097);
  // Socket parameter


  while (ros::ok()) {
    // Connecting
    ROS_INFO_STREAM("Connecting to PLC at " << host);
    chassis.connect(host, port);

    if (!chassis.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }
    ROS_INFO("Connected");

    // Send & Recv Data
    while (ros::ok()) {
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

          if (1 == recv_ret) {}
          else {}
        }

        // If send to socket is OK
        if (sendOK) {
          double x             = 988.234;
          double y             = 50.681;
          double theta         = 15.391;
          double container_len = 12000.810;


          pc_status_t pc_status;
          pc_status = pc_ready;

          LocationData location_data;
          location_data.x     = (int)(x * 1000);
          location_data.y     = (int)(y * 1000);
          location_data.theta = (int)(theta * 1000);

          ContainerData container_data;
          container_data.container_length = (int)(container_len * 1000);

          //          send_ret = chassis.SendMsg(pc_status, location_data,
          // container_data);
          //          send_ret = chassis.SendMsg(pc_status, location_data);
          send_ret = chassis.SendMsg(pc_status);

          if (1 == send_ret) {
            std::cout << "Send Buffer Success" << std::endl;
          }
          else {
            std::cout << "Send Buffer Failed" << std::endl;
          }
        }
      }
      else {
        if (0 == re) {
          ROS_WARN("Data transfer failure: Timeout\r\n");
        }
        else {
          ROS_WARN("Data transfer failure: Error\r\n");
        }
        chassis.disconnect();
        break;
      }

      #endif // ifndef _TCP_BUF_DEBUG

      // Disconnected, need reconnect
      if ((-1 == recv_ret) || (-1 == send_ret)) {
        chassis.disconnect();
        break;
      }

      //
      loop.sleep();
    }
  }

  chassis.disconnect();
  return 0;
}
