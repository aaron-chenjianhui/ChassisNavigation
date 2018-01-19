#include "tcp_driver/chassis_conn.h"

#include <iostream>


namespace conn{

  ChassisConn::ChassisConn(){
    m_connected = false;
  }

  ChassisConn::~ChassisConn(){
    disconnect();
  }

  void ChassisConn::connect(std::string host, int port){
  if (!m_connected)
  {
    logDebug("Creating non-blocking socket.");
    m_socket_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_socket_fd)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      logDebug("Connecting socket to laser.");
      int ret = ::connect(m_socket_fd, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        m_connected = true;
        logDebug("Connected succeeded.");
      }
    }
  }
  }

  void ChassisConn::disconnect(){
  if (m_connected)
  {
    close(m_connected);
    m_connected = false;
  }
  }

  bool ChassisConn::isConnected(){
    return m_connected;
  }


  int ChassisConn::WriteBuf(u_char buf[], int size){
    int ret = write(m_socket_fd, buf, size);

    if (ret == size){
      logDebug("Write %d bytes into fd.", ret);
    }
    else if (ret < size && ret >= 0){
      logWarn("Some buf missed, write %d bytes into fd.", ret);
    }
    else{
      logWarn("Buffer write error.");
    }
    return ret;
  }

  int ChassisConn::ReadBuf(u_char buf[], int size){
    u_char read_buf[100];
    int ret = read(m_socket_fd, read_buf, sizeof(read_buf));

    if (ret > 0)
    {
      logDebug("Read %d bytes from fd.", ret);
      // std::cout << "Read " << ret << "bytes from fd." << std::endl;
      // std::cout << std::hex << (int)read_buf[0] << std::endl;

      memcpy(buf, read_buf, ret);
      return ret;
    }
    else
    {
      logWarn("Buffer read() returned error.");
      return 0;
    }
  }


  void ChassisConn::SendMsg(){

  }

  void ChassisConn::RecvMsg(){

  }
}
