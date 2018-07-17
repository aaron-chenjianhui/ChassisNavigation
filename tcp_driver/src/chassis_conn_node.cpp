#include "ros/ros.h"


// Service headers
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include "laser_msgs/tcp_srv.h"

#include "chassis_conn.h"

#define _TCP_BUF_DEBUG 0


// State information
tcp_conn_status_t g_tcp_status = TCP_DISCONN;
msg_status_t g_msg_status = NO_MSG;
sys_status_t g_sys_status = SYS_INIT;

double pose_x, pose_y, pose_theta;
double container_len;

// status callback function
bool statusCallback(laser_msgs::tcp_srv::Request & req, laser_msgs::tcp_srv::Response& res);
void simuCallback(const geometry_msgs::Pose2D& laser_pose);
void containerCallback(const std_msgs::Float32& container_len);

int  main(int argc, char *argv[])
{
	// ROS related parameter
	ros::init(argc, argv, "chassis_conn");
	ros::NodeHandle n;
	ros::NodeHandle n_param("~");

	ros::ServiceServer tcp_server =
		n.advertiseService("tcp_status", statusCallback);
	ros::Subscriber simu_server =
		n.subscribe("laser_pose", 1000, simuCallback);
	ros::Subscriber container_server = n.subscribe("container_len",
						       1000,
						       containerCallback);

	int sys_freq;

	// Socket parameter
	std::string host;
	int port;

	// calibration parameter
	double container_width = 1000;
	double x_off = 350;
	double y_off = 0;
	double theta_off = 0;


	// Init parameter from parameter service
	n_param.param<std::string>("chassis_host", host, "127.0.0.1");
	n_param.param<int>("chassis_port", port, 2000);
	// n_param.param<std::string>("chassis_host", host, "192.168.10.30");
	// n_param.param<int>(        "chassis_port", port, 4097);

	// n_param.param<int>(        "sys_freq",     sys_freq, 10);
	ros::param::param<int>("/state_machine/sys_freq", sys_freq, 1);

	//  ros::Rate loop(20);
	ros::Rate loop_rate(sys_freq);

	conn::ChassisConn chassis;
	data_convert::PcPlcConvert converter;

	while (ros::ok()) {
		// Connecting
		ROS_INFO_STREAM("Connecting to PLC at " << host);
		std::cout << "Connecting to PLC at " << host << std::endl;

		chassis.connect(host, port);


		// sleep(0.5);

		if (!chassis.isConnected()) {
			ROS_WARN("Unable to connect, retrying.");
			std::cout << "Unable to connect, retrying." << std::endl;

			g_tcp_status = TCP_DISCONN;
			g_msg_status = NO_MSG;

			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		ROS_INFO("Connected");
		std::cout << "Connected" << std::endl;

		//
		g_tcp_status = TCP_CONN;


		// Send & Recv Data
		while (ros::ok()) {
			//
			std::cout <<
				"--------------------------- Loop Start ---------------------------" <<
				std::endl;

			// Communication status
			u_char send_buf[100];
			u_char recv_buf[100];

			int recv_ret = 1;
			int send_ret = 1;

			// char send_buf[] = {0x02, 0x11, 0x22, 0xAB, 0xCD, 0xEF};
			bool sendOK = false;
			bool recvOK = false;
			int re = chassis.WRSelect(recvOK, sendOK);

#if _TCP_BUF_DEBUG

			if (re > 0) {
				// If receive from socket is OK
				if (recvOK) {
					recv_ret = chassis.ReadBuf(recv_buf, sizeof(recv_buf));

					if (1 == recv_ret) {
						char int_buf[4];
						int recv_data_first, recv_data_second, recv_data_three;

						memcpy((char *)&recv_data_first, recv_buf, 4);
						memcpy((char *)&recv_data_second, recv_buf + 4, 4);
						memcpy((char *)&recv_data_three, recv_buf + 8, 4);

						std::cout << "recv buf int is: " << recv_data_first <<
							std::endl;
						std::cout << "recv buf int is: " << recv_data_second <<
							std::endl;
						std::cout << "recv buf int is: " << recv_data_three <<
							std::endl;
					} else {
						std::cout << "ReadError" << std::endl;
					}
				}

				// If send to socket is OK
				if (sendOK) {
					int send_first = 3400;
					int send_second = -23400;
					memcpy(send_buf, (u_char *)&send_first,
					       sizeof(send_first));
					memcpy(send_buf + sizeof(send_first), (u_char *)&send_second,
					       sizeof(send_second));

					send_ret = chassis.WriteBuf(send_buf, 8);

					if (1 == send_ret) {
						std::cout << "Send Buffer Success" << std::endl;
					} else {
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
							g_msg_status = MSG_READY;

							std::cout << "Recv ready msg" << std::endl;
						} else if (chassis_status == global_cmd) {
							g_msg_status = MSG_GLOBAL;

							std::cout << "Recv global msg" << std::endl;
						} else if (chassis_status == track_cmd) { // track_cmd
							g_msg_status = MSG_TRACK;

							std::cout << "Recv track msg" << std::endl;
						} else {
							g_msg_status = NO_MSG;

							std::cout << "Recv other msg" << std::endl;
						}
					} else {
						g_msg_status = NO_MSG;

						std::cout << "Recv buffer number is: 0" << std::endl;
					}
				}

				// If send to socket is OK
				if (sendOK) {
					double x, y, theta;
					double len;

					converter.PcToPlc(pose_x, pose_y, pose_theta, x, y, theta);
					len = container_len;

					pc_status_t pc_status;

					// send int data type
					LocationData location_data;
					location_data.x = (int)(x * 1000);
					location_data.y = (int)(y * 1000);
					location_data.theta = (int)(theta * 1000);

					ContainerData container_data;
					container_data.container_length = (int)(len * 1000);


					if (g_sys_status == SYS_READY) {
						pc_status = pc_ready;
						send_ret = chassis.SendMsg(pc_status);

						std::cout << "Send PC ready buffer success" << std::endl;
					} else if (g_sys_status == SYS_GLOBAL_ING) {
						pc_status = global_cal;
						send_ret = chassis.SendMsg(pc_status);

						std::cout << "Send PC global cal buffer success" << std::endl;
					} else if (g_sys_status == SYS_GLOBAL_OK) {
						pc_status = global_ready;
						send_ret = chassis.SendMsg(pc_status, location_data, container_data);

						std::cout << "Send PC global ready success" << std::endl;
						std::cout	<< "X is: " << location_data.x << "    "
								<< "Y is: " << location_data.y << "    "
								<< "Theta is: " << location_data.theta << std::endl;
						std::cout << "Container length is: " <<
							container_data.container_length << std::endl;
					} else if (g_sys_status == SYS_TRACK) {
						pc_status = track_ready;
						send_ret = chassis.SendMsg(pc_status, location_data);

						std::cout << "Send PC track ready success" << std::endl;
						std::cout	<< "X is: " << location_data.x << "    "
								<< "Y is: " << location_data.y << "    "
								<< "Theta is: " << location_data.theta << std::endl;
					} else { // sys_status == sys_init
						send_ret = 1;
						std::cout << "System status is sys_init" << std::endl;
					}

					// return check
					if (1 == send_ret) {
						//            std::cout << "Send Buffer Success" << std::endl;
					} else {
						std::cout << "Send Buffer Failed" << std::endl;
					}
				}
			} else {
				if (0 == re) {
					g_tcp_status = TCP_CONN;
					g_msg_status = NO_MSG;

					ROS_WARN("Data transfer failure: Timeout\r\n");
					std::cout << "Data transfer failure: Timeout" << std::endl;
				} else {
					g_tcp_status = TCP_CONN;
					g_msg_status = NO_MSG;

					ROS_WARN("Data transfer failure: Error\r\n");
					std::cout << "Data transfer failure: Error" << std::endl;
				}
			}
#endif       // ifndef _TCP_BUF_DEBUG

			// Disconnected, need reconnect
			if ((-1 == recv_ret) || (-1 == send_ret)) {
				g_tcp_status = TCP_DISCONN;
				g_msg_status = NO_MSG;

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

bool statusCallback(laser_msgs::tcp_srv::Request &	req,
		    laser_msgs::tcp_srv::Response&	res)
{
	std::cout << "in" << std::endl;
	uint16_t sys_status_now = req.sys_status;

	g_sys_status = (sys_status_t)sys_status_now;

	bool tcp_conn_now = g_tcp_status;
	uint8_t msg_status_now = g_msg_status;

	res.tcp_conn = tcp_conn_now;
	res.msg_status = msg_status_now;

	return true;
}

void simuCallback(const geometry_msgs::Pose2D& laser_pose)
{
	pose_x = laser_pose.x;
	pose_y = laser_pose.y;
	pose_theta = laser_pose.theta;
}

void containerCallback(const std_msgs::Float32& container_length)
{
	container_len = container_length.data;
}
