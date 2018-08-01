#ifndef _MAT_TYPES_H
#define _MAT_TYPES_H

#include <Eigen/Eigen>
#include <vector>
// #include "Eigen/Eigen"

#define DATATYPE double

typedef Eigen::Matrix<DATATYPE, 2, 2>mat2x2;
typedef Eigen::Matrix<DATATYPE, 2, 1>mat2x1;
typedef Eigen::Matrix<DATATYPE, 3, 3>mat3x3;
typedef Eigen::Matrix<DATATYPE, 3, 1>mat3x1;


class MatrixCal {
public:
typedef std::vector<double> PoseT;

public:
MatrixCal()
{
}


void PoseToMat(const PoseT& pose, mat3x3& pose_mat)
{
	double pose_x = pose[0];
	double pose_y = pose[1];
	double pose_theta = pose[2];
	double sin_theta = sin(pose_theta);
	double cos_theta = cos(pose_theta);

	pose_mat(0, 2) = pose_x;
	pose_mat(1, 2) = pose_y;
	pose_mat(0, 0) = cos_theta;
	pose_mat(0, 1) = -sin_theta;
	pose_mat(1, 0) = sin_theta;
	pose_mat(1, 1) = cos_theta;
	pose_mat(2, 0) = 0;
	pose_mat(2, 1) = 0;
	pose_mat(2, 2) = 1;
}

void MatToPose(const mat3x3& pose_mat, PoseT& pose)
{
	double pose_x, pose_y, pose_theta;

	pose_x = pose_mat(0, 2);
	pose_y = pose_mat(1, 2);
	pose_theta = atan(pose_mat(1, 0) / pose_mat(0, 0));

	pose[0] = pose_x;
	pose[1] = pose_y;
	pose[2] = pose_theta;
}
};

#endif // ifndef _MAT_TYPES_H
