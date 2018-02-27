#ifndef _MAT_TYPES_H
#define _MAT_TYPES_H

#include <Eigen/Eigen>

#define DATATYPE double

typedef Eigen::Matrix<DATATYPE, 2, 2>mat2x2;
typedef Eigen::Matrix<DATATYPE, 2, 1>mat2x1;
typedef Eigen::Matrix<DATATYPE, 3, 3>mat3x3;
typedef Eigen::Matrix<DATATYPE, 3, 1>mat3x1;

#endif // ifndef _MAT_TYPES_H
