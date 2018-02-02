#ifndef _CHASSIS_CONN_H
#define _CHASSIS_CONN_H

#include <string.h>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <errno.h>

#include "tcp_buffer.h"


#include "console_bridge/console.h"


#define TEST_BUFF_SIZE 500


typedef enum {
  chassis_ready = 0x10,
  global_cmd    = 0x11,
  track_cmd     = 0x12
} chassis_status_t;

typedef enum {
  pc_ready     = 0x20,
  global_cal   = 0x21,
  global_ready = 0x22,
  track_ready  = 0x32
} pc_status_t;

struct LocationData {
  int32_t x;
  int32_t y;
  int32_t theta;
};

struct ContainerData {
  int32_t container_length;
};


struct OdomData {
  int32_t right_pos;
  int32_t left_pos;
  int32_t right_vel;
  int32_t left_vel;
};


namespace conn {
class ChassisConn {
public:

  ChassisConn();
  ~ChassisConn();

  void connect(std::string host,
               int         port);
  void disconnect();
  bool isConnected();

  int  WRSelect(bool& readOK,
                bool& writeOK);

  int  WriteBuf(u_char buf[],
                int    buf_size);
  int  ReadBuf(u_char buf[],
               int    buf_size);


  // Overload function for sending message
  int  SendMsg(const pc_status_t  & pc_status,
               const LocationData & location_data,
               const ContainerData& container_data);
  int  SendMsg(const pc_status_t & pc_status,
               const LocationData& location_data);
  int  SendMsg(const pc_status_t& pc_status);

  // Overload function for receiving message
  int  RecvMsg(chassis_status_t& chassis_status,
               OdomData        & odom_data);

  void parseTCPData(const u_char     *buffer_data,
                    chassis_status_t& chassis_status,
                    OdomData        & odom_data);

private:

  int m_socket_fd;
  bool m_connected;

  u_char buf_test[TEST_BUFF_SIZE];

  TCPBuffer m_buffer;
};
}

#endif // ifndef _CHASSIS_CONN_H
