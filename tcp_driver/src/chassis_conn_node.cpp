#include "ros/ros.h"

#include "tcp_driver/chassis_conn.h"


int main(int argc, char* argv[]){
  // ROS related parameter
  ros::init(argc, argv, "chassis_conn");
  ros::NodeHandle n("~");
  ros::Rate loop(50);

  // PLC related parameters
  conn::ChassisConn chassis;


  std::string host;
  int port;

  n.param<std::string>("chassis_host", host, "192.168.10.30");
  n.param<int>("chassis_port", port, 4097);


  while (ros::ok()){
    ROS_INFO_STREAM("Connecting to PLC at " << host);
    chassis.connect(host, port);
    if (!chassis.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    // Send & Recv Data
    while (ros::ok()){
      u_char send_buf[100];
      int send_first = 3400;
      int send_second = -23400;
      memcpy(send_buf, (u_char *)&send_first, sizeof(send_first));
      memcpy(send_buf+sizeof(send_first), (u_char *)&send_second, sizeof(send_second));
      // char send_buf[] = {0x02, 0x11, 0x22, 0xAB, 0xCD, 0xEF};
      u_char recv_buf[100];

      // int send_ret = chassis.WriteBuf(send_buf, sizeof(send_buf));
      int send_ret = chassis.WriteBuf(send_buf, 8);
      if (send_ret > 0){
        std::cout << "Send Buffer Success" << std::endl;
      }
      else{
        std::cout << "Send Buffer Failed" << std::endl;
      }


      // int recv_ret = chassis.ReadBuf(recv_buf, sizeof(recv_buf));
      // if (recv_ret > 0){
      //   // for (int i = 0; i < recv_ret; ++i){
      //   //   std::cout << "Recv Buf " << i << " "<< "is: " << std::hex
      //   //   << (int)recv_buf[i] << std::endl;
      //   // }
      //   // int recv_data;
      //   char int_buf[4];
      //   int recv_data_first, recv_data_second, recv_data_three;
      //
      //   memcpy((char*)&recv_data_first, recv_buf, 4);
      //   memcpy((char*)&recv_data_second, recv_buf+4, 4);
      //   memcpy((char*)&recv_data_three, recv_buf+8, 4);
      //   // int *recv_data = (int *)int_buf;
      //
      //   std::cout << "recv buf int is: " << recv_data_first << std::endl;
      //   std::cout << "recv buf int is: " << recv_data_second << std::endl;
      //   std::cout << "recv buf int is: " << recv_data_three << std::endl;
      // }
      // else{
      //   std::cout << "ReadError" << std::endl;
      // }


      //
      loop.sleep();
    }
  }

  return 0;
}
