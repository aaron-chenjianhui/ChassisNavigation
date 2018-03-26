#ifndef _PC_TO_PLC_HPP
#define _PC_TO_PLC_HPP

#include "MatTypes.h"

#define DEG2RAD(X) X * 3.1415926 / 180
#define RAD2DEG(X) X * 180 / 3.1415926
#define M2MM(X) X * 1000
#define MM2M(X) X / 1000

namespace data_convert {
class PcPlcConvert {
public:

  PcPlcConvert() {
    double x_off     = 572.782;
    double y_off     = 11.447;
    double theta_off = 0.00307;

    T_Laser_in_Base = EulerToTrans(x_off, y_off, theta_off);

    m_container_width = 2350;
    m_chassis_length  = 1700;
  }

  /**
   * [PcToPlc description]
   * @param width     container width
   * @param x_off     base position in laser reference
   * @param y_off     [description]
   * @param theta_off [description]
   */
  PcPlcConvert(double width, double x_off, double y_off, double theta_off) {
    //
    T_Laser_in_Base = EulerToTrans(x_off, y_off, theta_off);
  }

  PcPlcConvert(double width, mat3x3 laser_in_base) {
    T_Laser_in_Base = laser_in_base;
  }

  ~PcPlcConvert() {}

  void PcToPlc(const mat3x3 T_Laser_in_Ori,
               double     & x_plc,
               double     & y_plc,
               double     & theta_plc) {
    mat3x3 T_Base_in_Ori;

    T_Base_in_Ori = T_Laser_in_Ori * T_Laser_in_Base.inverse();

    //
    double x_base_in_ori, y_base_in_ori, theta_base_in_ori;
    TransToEuler(T_Base_in_Ori, x_base_in_ori, y_base_in_ori, theta_base_in_ori);

    //
    theta_plc = -RAD2DEG(theta_base_in_ori);
    double y_plc_tmp = -y_base_in_ori;
    double x_plc_tmp = -x_base_in_ori / cos(abs(theta_base_in_ori));

    y_plc = y_plc_tmp - m_container_width / 2;
    x_plc = x_plc_tmp - m_chassis_length / 2;
  }

  /**
   * [PcToPlc description]
   * @param x         laser position in original coordinate
   * @param y         [description]
   * @param theta     [description]
   * @param x_plc     [description]
   * @param y_plc     [description]
   * @param theta_plc [description]
   */
  void PcToPlc(const double& x,
               const double& y,
               const double& theta,
               double      & x_plc,
               double      & y_plc,
               double      & theta_plc) {
    mat3x3 T_Laser_in_Ori;

    T_Laser_in_Ori << cos(theta), -sin(theta), x,
      sin(theta), cos(theta), y,
      0, 0, 1;

    mat3x3 T_Base_in_Ori;

    T_Base_in_Ori = T_Laser_in_Ori * T_Laser_in_Base.inverse();

    //
    double x_base_in_ori, y_base_in_ori, theta_base_in_ori;
    TransToEuler(T_Base_in_Ori, x_base_in_ori, y_base_in_ori, theta_base_in_ori);

    //
    theta_plc = -RAD2DEG(theta_base_in_ori);
    double y_plc_tmp = -y_base_in_ori;
    double x_plc_tmp = -x_base_in_ori / cos(abs(theta_base_in_ori));

    y_plc = y_plc_tmp - m_container_width / 2;
    x_plc = x_plc_tmp - m_chassis_length / 2;
  }

  // euler representation to transform matrix
  mat3x3 EulerToTrans(double x, double y, double theta) {
    mat3x3 trans_mat;

    trans_mat << cos(theta), -sin(theta), x,
      sin(theta), cos(theta), y,
      0, 0, 1;

    return trans_mat;
  }

  void TransToEuler(const mat3x3 trans_mat, double& x, double& y, double& theta) {
    x = trans_mat(0, 2);
    y = trans_mat(1, 2);

    theta = atan2(trans_mat(1, 0), trans_mat(0, 0));
  }

private:

  double m_container_width;
  double m_chassis_length;

  mat3x3 T_Laser_in_Base;
};
}


#endif // ifndef _PC_TO_PLC_HPP
