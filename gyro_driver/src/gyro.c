#include "gyro.h"

int PackHandle(uchar buf[], size_t recv_num, uchar data_buf[], size_t data_buf_size){
  static uchar pack_length, addr, cmd_word, data_buf_sum, check_sum;
  static size_t data_reading_count;
  static int step_flag = 0;
  uchar recv_ch;


  size_t i;
  for (i = 0; i < recv_num; ++i){
    recv_ch = buf[i];

    // printf("Buf Number: %d\n", i);
    // printf("data_buf_size: %d\n", data_buf_size);

    // Head
    if (0x68 == recv_ch){
      step_flag = 1;
      // printf("Head\n");
      continue;
    }
    // Reading Length
    if (1 == step_flag){
      pack_length = recv_ch;
      step_flag = 2;
      // printf("Length\n");
      continue;
    }
    // Reading Address
    if (2 == step_flag){
      addr = recv_ch;
      step_flag = 3;
      // printf("Address\n");
      continue;
    }
    // Reading Command Words
    if (3 == step_flag){
      cmd_word = recv_ch;
      step_flag = 4;
      data_reading_count = 0;
      data_buf_sum = 0x00;
      // printf("Command Words\n");
      continue;
    }
    // Reading Data
    if ((4 == step_flag) && (data_reading_count < data_buf_size)){
      // printf("Data Reading Count: %d\n", data_reading_count);

      data_buf[data_reading_count] = recv_ch;
      data_buf_sum += recv_ch;
      data_reading_count++;

      if (data_buf_size == data_reading_count){
        data_reading_count = 0;
        step_flag = 5;
      }
      continue;
    }
    // Check Sum
    if (5 == step_flag){
      uchar data_sum = pack_length + addr + cmd_word + data_buf_sum;
      check_sum = recv_ch;
      // printf("Recv Sum: %02X\nCal Sum: %02X\n", check_sum, data_sum);
      step_flag = 0;
      return (data_sum == check_sum);
    }
  }

  return 0;
}



int DataTranslate(uchar buf[], size_t buf_size, float* ang_vel_out, float* accel_ahead_out, float* ang_out){
    if (9 == buf_size){
        uchar vel_symbol = buf[0] >> 4;
        uchar accel_symbol = buf[3] >> 4;
        uchar ang_symbol = buf[6] >> 4;
        int vel_int, accel_int, ang_int;
        float vel, accel, ang;

        // Positive
        if (0x00 == vel_symbol){
            vel_int = (buf[0] & 0x0f)*10000 + (buf[1] >> 4)*1000 + (buf[1] & 0x0f)*100 +
                    (buf[2] >> 4)*10  + (buf[2] & 0x0f);
            vel = (float)vel_int/(float)100.0;
        }
        // Negative
        else{
            vel_int = (buf[0] & 0x0f)*10000 + (buf[1] >> 4)*1000 + (buf[1] & 0x0f)*100 +
                    (buf[2] >> 4)*10  + (buf[2] & 0x0f);
            vel = -(float)vel_int/(float)100.0;
        }

        //
        if (0x00 == accel_symbol){
            accel_int = (buf[3]&0x0f)*10000 + (buf[4]>>4)*1000 + (buf[4]&0x0f)*100 +
                    (buf[5]>>4)*10 + (buf[5]&0x0f);
            accel = (float)accel_int/(float)1000.0;
        }
        else{
            accel_int = (buf[3]&0x0f)*10000 + (buf[4]>>4)*1000 + (buf[4]&0x0f)*100 +
                    (buf[5]>>4)*10 + (buf[5]&0x0f);
            accel = -(float)accel_int/(float)1000.0;
        }

        //
        if (0x00 == ang_symbol){
            ang_int = (buf[6]&0x0f)*10000 + (buf[7]>>4)*1000 + (buf[7]&0x0f)*100 +
                    (buf[8]>>4)*10 + (buf[8]&0x0f);
            ang = (float)ang_int/(float)100.0;
        }
        else{
            ang_int = (buf[6]&0x0f)*10000 + (buf[7]>>4)*1000 + (buf[7]&0x0f)*100 +
                    (buf[8]>>4)*10 + (buf[8]&0x0f);
            ang = -(float)ang_int/(float)100.0;
        }

        *ang_vel_out = vel;
        *accel_ahead_out = accel;
        *ang_out = ang;

        return 1;
    }
    // Data Fiele is wrong
    else{
        return 0;
    }
}


// int DataClear(int comport_number){
//   uchar clear_cmd[] = {0x68, 0x04, 0x00, 0x28, 0x2C};
//   int send_re = RS232_SendBuf(comport_number, clear_cmd, sizeof(clear_cmd));
//
//   if(5 == send_re){
//     // 应答判断
//     int n;
//     uchar recv_buf[100];
//     // usleep(1000);
//     n = RS232_PollComport(comport_number, recv_buf, sizeof(recv_buf));
//     if (n > 0){
//       int i;
//       for (i = 0; i < n-4; ++i){
//         if (0x68 == recv_buf[i] && 0x05 == recv_buf [i+1] && 0x00 == recv_buf[i+2]
//         && 0x28 == recv_buf[i+3])
//         {return (0x00 == recv_buf[i+4]);}
//       }
//       return 1001;
//     }
//     else{
//       return 1002;
//     }
//   }
//   else{
//     return 1000;
//   }
// }
