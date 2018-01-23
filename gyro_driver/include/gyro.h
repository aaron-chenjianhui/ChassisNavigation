#ifndef _GYRO_H
#define _GYRO_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <unistd.h>

#include "rs232.h"

typedef unsigned char uchar;


int PackHandle(uchar buf[], size_t recv_num, uchar data_buf[], size_t data_buf_size);
int DataTranslate(uchar buf[], size_t buf_size, float* ang_vel, float* accel_ahead, float* ang);
// int DataClear(int comport_number);



#ifdef __cplusplus
} /* extern "C" */
#endif


#endif
