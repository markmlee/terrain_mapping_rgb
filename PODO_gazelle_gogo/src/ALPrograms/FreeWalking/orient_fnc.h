//////////////////////////////////////
// orient_fnc.h
// Inhyeok Kim
// Rainbow Inc. 2014. 12. 15
//////////////////////////////////////

#ifndef ORIENT_FNC_H
#define ORIENT_FNC_H

#define STANCE_ROLL_MAX      30*D2R
#define STANCE_PITCH_MAX      30*D2R

int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[]);
int convert_euler_FT(double _pre_state,double pRF_3x1[], double pLF_3x1[],double qRF_4x1[], double qLF_4x1[], double control_result_RF[],double control_result_LF[]);
int convert_euler_imu(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z);
int convert_euler_imu_vel(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z);

int complete_form(const double Q_34x1[], double imu_euler_x, double imu_euler_y, double imu_euler_z,
                  double *des_pCOM_3x1, double *des_qPEL_4x1,
                  double *des_pRF_3x1, double *des_qRF_4x1,
                  double *des_pLF_3x1, double *des_qLF_4x1);

int complete_form_delta(const double Q_current_34x1[],
                      double euler_x_err, double euler_y_err, double euler_z_err,
                      double *des_pCOM_3x1, double *des_qPEL_4x1,
                      double *des_pRF_3x1, double *des_qRF_4x1,
                      double *des_pLF_3x1, double *des_qLF_4x1);
int QT2EULER(const double q_4x1[], double &roll, double &pitch, double &yaw);
//int QT2YPR(const double qt_4x1[], double &yaw, double &pit, double &rol);
//int QT2YRP(const double qt_4x1[], double &yaw, double &rol, double &pit);
#endif // ORIENT_FNC_H
