#ifndef _CHASSIS_CONN_H
#define _CHASSIS_CONN_H

#include <string.h>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>


#include "console_bridge/console.h"


#define TEST_BUFF_SIZE 500


typedef enum{
  stop = 0,
  moving = 1,
  ready = 2
} status_t;

struct LocatData{
  double x;
  double y;
  double theta;
};

struct OdomData{
  double right_pos;
  double left_pos;
  double right_vel;
  double left_vel;
};


namespace conn{
  class ChassisConn{
  public:
    ChassisConn();
    ~ChassisConn();

    void connect(std::string host, int port);
    void disconnect();
    bool isConnected();

    int WriteBuf(u_char buf[], int buf_size);
    int ReadBuf(u_char buf[], int buf_size);

    void SendMsg();
    void RecvMsg();



  private:
    int m_socket_fd;
    bool m_connected;

    u_char buf_test[TEST_BUFF_SIZE];
  };
}

#endif
