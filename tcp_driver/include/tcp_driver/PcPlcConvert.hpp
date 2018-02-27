#ifndef _PC_TO_PLC_HPP
#define _PC_TO_PLC_HPP

#include "MatTypes.h"

namespace data_convert {
class PcPlcConvert {
public:

  /**
   * [PcToPlc description]
   * @param width     container width
   * @param x_off     base position in laser reference
   * @param y_off     [description]
   * @param theta_off [description]
   */
  PcPlcConvert(double width, double x_off, double y_off, double theta_off) {
    // container width
    m_width = width;

    // reference coordinate in original coordinate
    T_Ref_in_Ori << 1, 0, 0,
      0, 1, -m_width / 2.0,
      0, 0, 1;

    //
    T_Base_in_Laser = EulerToTrans(x_off, y_off, theta_off);
  }

  PcPlcConvert(double width, mat3x3 base_in_laser) {
    m_width = width;

    // reference coordinate in original coordinate
    T_Ref_in_Ori << 1, 0, 0,
      0, 1, -m_width / 2.0,
      0, 0, 1;
    T_Base_in_Laser = base_in_laser;
  }

  ~PcPlcConvert() {}

  void PcToPlc(const mat3x3 T_Laser_in_Ori,
               double     & x_plc,
               double     & y_plc,
               double     & theta_plc) {
    // resolve transform matrix base in ref
    mat3x3 T_Base_in_Ref;

    T_Base_in_Ref = (T_Ref_in_Ori.inverse()) * T_Laser_in_Ori * T_Base_in_Laser;

    //
    double x_base_in_ref, y_base_in_ref, theta_base_in_ref;
    TransToEuler(T_Base_in_Ref, x_base_in_ref, y_base_in_ref, theta_base_in_ref);

    //
    x_plc     = -y_base_in_ref;
    y_plc     = -x_base_in_ref;
    theta_plc = -theta_base_in_ref;
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

    // resolve transform matrix base in ref
    mat3x3 T_Base_in_Ref;

    T_Base_in_Ref = (T_Ref_in_Ori.inverse()) * T_Laser_in_Ori * T_Base_in_Laser;

    //
    double x_base_in_ref, y_base_in_ref, theta_base_in_ref;
    TransToEuler(T_Base_in_Ref, x_base_in_ref, y_base_in_ref, theta_base_in_ref);

    //
    x_plc     = -y_base_in_ref;
    y_plc     = -x_base_in_ref;
    theta_plc = -theta_base_in_ref;
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

  double m_width;

  mat3x3 T_Base_in_Laser;
  mat3x3 T_Ref_in_Ori;
};
}


#endif // ifndef _PC_TO_PLC_HPP
