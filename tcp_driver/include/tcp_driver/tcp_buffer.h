#ifndef _TCP_BUFFER_H
#define _TCP_BUFFER_H

#include <string.h>
#include <unistd.h>

#include "console_bridge/console.h"


#define TCP_BUFFER_SIZE 50000
#define TCP_BUFFER_HEAD 0xFA
#define TCP_BUFFER_TAIL 0xED

class TCPBuffer {
public:

  TCPBuffer() : m_total_length(0) {}

  int readFrom(int fd) {
    int ret = read(fd,
                   m_buffer + m_total_length,
                   sizeof(m_buffer) - m_total_length);

    if (ret > 0) {
      m_total_length += ret;

      logDebug("Read %d bytes from fd, total length is %d.", ret, m_total_length);
    }
    else {
      logWarn("Buffer read() returned error.");
    }

    return ret;
  }

  int getNextBuffer(u_char *buffer_return, uint16_t buffer_size) {
    if (0 == m_total_length) {
      logDebug("Empty buffer, nothing to return.");

      buffer_return = NULL;
      return 0;
    }

    for (int i = 0; i < m_total_length; ++i) {
      // Single message parameter
      u_char *msg_start = (u_char *)memchr(m_buffer,
                                           TCP_BUFFER_HEAD,
                                           m_total_length);

      // No start buffer, ignore all buffers
      if (NULL == msg_start) {
        logWarn("No package head found, dropping %d bytes from buffer.",
                m_total_length);
        m_total_length = 0;

        buffer_return = NULL;
        return 0;
      }

      // Find start buffer and shift
      shiftBuffer(msg_start);

      u_char *msg_end =
        (u_char *)memchr(m_buffer, TCP_BUFFER_TAIL, m_total_length);

      // No end buffer, keep the buffers and wait for another "end"
      if (NULL == msg_end) {
        logDebug("No package tail found, nothing to return.");

        buffer_return = NULL;
        return 0;
      }

      // Find "end" buffer, checkout package
      if ((uint16_t)(*(m_buffer + 1)) == (msg_end - m_buffer)) {
        m_end_of_first_msg = msg_end;

        uint16_t return_size = (msg_end - m_buffer) + 1;
        memcpy(buffer_return, m_buffer, return_size);

        return 1;
      }
      else {
        shiftBuffer(m_buffer + 1);
        continue;
      }
    }
  }

  // pop the last buffer after reading a complete message package
  void popLastBuffer() {
    if (m_end_of_first_msg) {
        u_char* new_start = m_end_of_first_msg + 1;
      shiftBuffer(m_end_of_first_msg + 1);
      m_end_of_first_msg = NULL;
    }
  }

private:

  // Shift Buffer to a new start
  void shiftBuffer(u_char *new_start) {
    uint16_t remaining_length = m_total_length - (new_start - m_buffer);

    if (remaining_length > 0) {
      memmove(m_buffer, new_start, remaining_length);
    }
    m_total_length = remaining_length;
  }

  // Buffer length
  uint16_t m_total_length;

  // Buffer
  u_char m_buffer[TCP_BUFFER_SIZE];
  u_char *m_end_of_first_msg;
};


#endif // ifndef _TCP_BUFFER_H
