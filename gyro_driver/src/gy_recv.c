
/**************************************************

   file: demo_rx.c
   purpose: simple demo that receives characters from
   the serial port and print them on the screen,
   exit the program by pressing Ctrl-C

   compile with the command: gcc demo_rx.c rs232.c -Wall -Wextra -o2 -o test_rx

**************************************************/

#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
# include <Windows.h>
#else  /* ifdef _WIN32 */
# include <unistd.h>
#endif /* ifdef _WIN32 */

#define BUFLENG 100
#define DATALENG 9
#define PORT 0
#define BAUDRATE 115200

// #define _RECV_FILE

#include "rs232.h"
#include "gyro.h"

typedef unsigned char uchar;

float ang_ref = 0;


int main()
{
  int  cport_nr = PORT;     /* /dev/ttyS0 (COM1 on windows) */
  int  bdrate   = BAUDRATE; /* 115200 baud */
  char mode[]   = { '8', 'N', '1', 0 };

  uchar buf[BUFLENG];
  uchar data_buf[DATALENG];
  int   recv_num;

  float ang_vel_raw, accel_raw, ang_raw;
  float ang_vel_real, accel_real, ang_real;

#ifdef _RECV_FILE
  FILE *file;
  file = fopen("gy_data.txt", "w");
#endif /* ifdef _RECV_FILE */

  if (RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return 0;
  }

  int err_count = 0;

  while (1)
  {
    recv_num = RS232_PollComport(cport_nr, buf, sizeof(buf));
    printf("recv number: %d\n", recv_num);

#ifdef _RECV_FILE
    int i;

    for (i = 0; i < recv_num; ++i) {
      char buf_char[2];
      sprintf(buf_char, "%02X", buf[i]);
      fwrite(buf_char, sizeof(buf_char[0]), sizeof(buf_char), file);
    }
    fputc('\n', file);
#endif /* ifdef _RECV_FILE */

    int pack_status = 0;
    pack_status = PackHandle(buf, recv_num, data_buf, sizeof(data_buf));

    if (1 == pack_status) {
      int data_status = DataTranslate(data_buf, sizeof(data_buf),
                                      &ang_vel_raw, &accel_raw, &ang_raw);

      if (1 == data_status) {
        ang_vel_real = ang_vel_raw;
        accel_real   = accel_raw;
        ang_real     = ang_raw - ang_ref;

        printf(
          "Real Angular Velocity: %f\nReal Acceleration: %f\nReal Angular: %f\n",
          ang_vel_real,
          accel_real,
          ang_real);
      }
      else {
        printf("Data Translation Wrong\n");
      }
      err_count = 0;
    }
    else {
      err_count++;
      printf("Data Package Broken Count: %d\n", err_count);
    }


    usleep(40 * 1000);
  }

  return 0;
}
