#include "tcp_driver/chassis_conn.h"

#include <iostream>

#include <signal.h>


namespace conn {
ChassisConn::ChassisConn() {
  m_connected = false;

  signal(SIGPIPE, SIG_IGN);

  sigset_t signal_mask;
  sigemptyset(&signal_mask);
  sigaddset(&signal_mask, SIGPIPE);
  int rc = pthread_sigmask(SIG_BLOCK, &signal_mask, NULL);

  //    if (rc != 0)
  //    {
  //       printf("block sigpipe error\n");
  //    }
}

ChassisConn::~ChassisConn() {
  disconnect();
}

void ChassisConn::connect(std::string host, int port) {
  if (!m_connected)
  {
    logDebug("Creating non-blocking socket.");
    m_socket_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (m_socket_fd)
    {
      int flags = fcntl(m_socket_fd, F_GETFL, 0);
      fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK);

      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port   = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      logDebug("Connecting socket to laser.");
      int ret =
        ::connect(m_socket_fd, (struct sockaddr *)&stSockAddr,
                  sizeof(stSockAddr));

      if (ret == 0)
      {
        m_connected = true;
        logDebug("Connected succeeded.");
      }
    }
  }
}

void ChassisConn::disconnect() {
  if (m_connected)
  {
    close(m_connected);
    m_connected = false;
  }
}

bool ChassisConn::isConnected() {
  return m_connected;
}

int ChassisConn::WRSelect(bool& readOK, bool& writeOK) {
  //
  bool readable  = false;
  bool writeable = false;

  // 0(timeout) / -1(error) / >0(default)
  int re_status;

  fd_set readset, writeset;
  struct timeval timeout;

  timeout.tv_usec = 10000; // us
  timeout.tv_sec  = 0;     // s

  //
  FD_ZERO(&readset);
  FD_ZERO(&writeset);
  FD_SET(m_socket_fd, &readset);
  FD_SET(m_socket_fd, &writeset);

  //
  int ret = select(m_socket_fd + 1, &readset, &writeset, NULL, &timeout);

  switch (ret) {
  case 0:
    re_status = 0;
    break;

  case -1:
    re_status = -1;

  default:
    re_status = ret;

    if (FD_ISSET(m_socket_fd, &readset) > 0) {
      readable = true;
    }

    if (FD_ISSET(m_socket_fd, &writeset) > 0) {
      writeable = true;
    }
  }
  readOK  = readable;
  writeOK = writeable;
  return re_status;
}

int ChassisConn::WriteBuf(u_char buf[], int size) {
  int ret = write(m_socket_fd, buf, size);

  if (ret == size) {
    logDebug("Write %d bytes into fd.", ret);

    return 1;
  }
  else if ((ret < size) && (ret >= 0)) {
    logWarn("Some buf missed, write %d bytes into fd.", ret);

    return 1;
  }
  else {
    logWarn("Buffer write error.");

    return -1;
  }
}

int ChassisConn::ReadBuf(u_char buf[], int size) {
  u_char read_buf[100];
  int    ret = read(m_socket_fd, read_buf, sizeof(read_buf));

  if (ret > 0)
  {
    logDebug("Read %d bytes from fd.", ret);

    // std::cout << "Read " << ret << "bytes from fd." << std::endl;
    // std::cout << std::hex << (int)read_buf[0] << std::endl;

    memcpy(buf, read_buf, ret);
    return 1;
  }
  else if (0 == ret) // Disconnected
  {
    logWarn("Socket Disconnected.");
    return -1;
  }
  else {
    if (errno == EINTR) {
      logWarn("Socket connection is OK, but somehow, socket occurs error.");
      return 0;
    }
    else {
      logWarn("Socket Disconnected.");
      return -1;
    }
  }
}

int ChassisConn::SendMsg(const pc_status_t  & pc_status,
                         const LocationData & location_data,
                         const ContainerData& container_data) {
  u_char pc_status_buf[2];

  pc_status_buf[0] = 0x02;
  pc_status_buf[1] = (u_char)pc_status;

  u_char location_buf[13];
  location_buf[0] = 0x04;
  memcpy(location_buf + 1, &location_data, sizeof(location_buf) - 1);

  u_char container_buf[5];
  container_buf[0] = 0x05;
  memcpy(container_buf + 1, &container_data, sizeof(container_buf) - 1);

  u_char count_buf = sizeof(pc_status_buf) + sizeof(location_buf) +
                     sizeof(container_buf) + 3;

  u_char target_buf = 0x01;
  u_char head_buf   = TCP_BUFFER_HEAD;
  u_char tail_buf   = TCP_BUFFER_TAIL;

  u_char  send_buf[100];
  u_char *send_buf_ptr = send_buf;
  uint8_t send_length  = count_buf + 1;
  memcpy(send_buf_ptr, &head_buf,     sizeof(head_buf));      // head
  send_buf_ptr += sizeof(head_buf);
  memcpy(send_buf_ptr, &count_buf,    sizeof(count_buf));     // number
  send_buf_ptr += sizeof(count_buf);
  memcpy(send_buf_ptr, &target_buf,   sizeof(target_buf));    // send target
  send_buf_ptr += sizeof(target_buf);
  memcpy(send_buf_ptr, pc_status_buf, sizeof(pc_status_buf)); // pc status
  send_buf_ptr += sizeof(pc_status_buf);
  memcpy(send_buf_ptr, location_buf,  sizeof(location_buf));  // location data
  send_buf_ptr += sizeof(location_buf);
  memcpy(send_buf_ptr, container_buf, sizeof(container_buf)); // container data
  send_buf_ptr += sizeof(container_buf);
  memcpy(send_buf_ptr, &tail_buf,     sizeof(tail_buf));      // tail
  send_buf_ptr += sizeof(tail_buf);

  int ret = write(m_socket_fd, send_buf, send_length);

  if (ret == send_length) {
    logDebug("Write %d bytes into fd.", ret);

    return 1;
  }
  else if ((ret < send_length) && (ret >= 0)) {
    logWarn("Some buf missed, write %d bytes into fd.", ret);

    return 1;
  }
  else {
    logWarn("Buffer write error.");

    return -1;
  }
}

int ChassisConn::SendMsg(const pc_status_t & pc_status,
                         const LocationData& location_data) {
  u_char pc_status_buf[2];

  pc_status_buf[0] = 0x02;
  pc_status_buf[1] = (u_char)pc_status;

  u_char location_buf[13];
  location_buf[0] = 0x04;
  memcpy(location_buf + 1, &location_data, sizeof(location_buf) - 1);

  u_char count_buf = sizeof(pc_status_buf) + sizeof(location_buf) + 3;

  u_char target_buf = 0x01;
  u_char head_buf   = TCP_BUFFER_HEAD;
  u_char tail_buf   = TCP_BUFFER_TAIL;

  u_char  send_buf[100];
  u_char *send_buf_ptr = send_buf;
  uint8_t send_length  = count_buf + 1;
  memcpy(send_buf_ptr, &head_buf,     sizeof(head_buf));      // head
  send_buf_ptr += sizeof(head_buf);
  memcpy(send_buf_ptr, &count_buf,    sizeof(count_buf));     // number
  send_buf_ptr += sizeof(count_buf);
  memcpy(send_buf_ptr, &target_buf,   sizeof(target_buf));    // send target
  send_buf_ptr += sizeof(target_buf);
  memcpy(send_buf_ptr, pc_status_buf, sizeof(pc_status_buf)); // pc status
  send_buf_ptr += sizeof(pc_status_buf);
  memcpy(send_buf_ptr, location_buf,  sizeof(location_buf));  // location data
  send_buf_ptr += sizeof(location_buf);
  memcpy(send_buf_ptr, &tail_buf,     sizeof(tail_buf));      // tail
  send_buf_ptr += sizeof(tail_buf);

  int ret = write(m_socket_fd, send_buf, send_length);

  if (ret == send_length) {
    logDebug("Write %d bytes into fd.", ret);

    return 1;
  }
  else if ((ret < send_length) && (ret >= 0)) {
    logWarn("Some buf missed, write %d bytes into fd.", ret);

    return 1;
  }
  else {
    logWarn("Buffer write error.");

    return -1;
  }
}

int ChassisConn::SendMsg(const pc_status_t& pc_status) {
  u_char pc_status_buf[2];

  pc_status_buf[0] = 0x02;
  pc_status_buf[1] = (u_char)pc_status;

  u_char count_buf = sizeof(pc_status_buf) + 3;

  u_char target_buf = 0x01;
  u_char head_buf   = TCP_BUFFER_HEAD;
  u_char tail_buf   = TCP_BUFFER_TAIL;

  u_char  send_buf[100];
  u_char *send_buf_ptr = send_buf;
  uint8_t send_length  = count_buf + 1;
  memcpy(send_buf_ptr, &head_buf,     sizeof(head_buf));      // head
  send_buf_ptr += sizeof(head_buf);
  memcpy(send_buf_ptr, &count_buf,    sizeof(count_buf));     // number
  send_buf_ptr += sizeof(count_buf);
  memcpy(send_buf_ptr, &target_buf,   sizeof(target_buf));    // send target
  send_buf_ptr += sizeof(target_buf);
  memcpy(send_buf_ptr, pc_status_buf, sizeof(pc_status_buf)); // pc status
  send_buf_ptr += sizeof(pc_status_buf);
  memcpy(send_buf_ptr, &tail_buf,     sizeof(tail_buf));      // tail
  send_buf_ptr += sizeof(tail_buf);

  int ret = write(m_socket_fd, send_buf, send_length);

  if (ret == send_length) {
    logDebug("Write %d bytes into fd.", ret);

    return 1;
  }
  else if ((ret < send_length) && (ret >= 0)) {
    logWarn("Some buf missed, write %d bytes into fd.", ret);

    return 1;
  }
  else {
    logWarn("Buffer write error.");

    return -1;
  }
}

int ChassisConn::RecvMsg(chassis_status_t& chassis_status,
                         OdomData        & odom_data) {
  int ret = m_buffer.readFrom(m_socket_fd);

  if (ret > 0)
  {
    // Will return pointer if a complete message exists in the buffer,
    // otherwise will return null.
    u_char buffer_data[100];
    int    buf_ret = m_buffer.getNextBuffer(buffer_data, sizeof(buffer_data));

    if (buf_ret)
    {
      parseTCPData(buffer_data, chassis_status, odom_data);
      m_buffer.popLastBuffer();
      return 1;
    }

    return 0;
  }
  else if (0 == ret) // Disconnected
  {
    logWarn("Socket Disconnected.");
    return -1;
  }
  else {
    if (errno == EINTR) {
      logWarn("Socket connection is OK, but somehow, socket occurs error.");
      return 0;
    }
    else {
      logWarn("Socket Disconnected.");
      return -1;
    }
  }
}

void ChassisConn::parseTCPData(const u_char     *buffer_data,
                               chassis_status_t& chassis_status,
                               OdomData        & odom_data) {
  u_char pack_head = buffer_data[0];
  u_char pack_num  = buffer_data[1];
  u_char pack_tail = buffer_data[pack_num];

  if (0x05 == pack_num) {
    u_char pack_cmd      = buffer_data[3];
    u_char pack_cmd_data = buffer_data[4];

    if (0x11 == pack_cmd_data) {
      chassis_status = global_cmd;
    }
    else if (0x12 == pack_cmd_data) {
      chassis_status = track_cmd;
    }
    else if (0x10 == pack_cmd_data) {
      chassis_status = chassis_ready;
    }
  }
}
}
