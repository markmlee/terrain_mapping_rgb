#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/kine_drc_hubo2.h"
#include "BasicMatrix.h"
#include "ManualCAN.h"



void Enc_request(int on);

int zero_gain();
int gravity_compensation();
int gain_tuning(int update, int cmd[]);
int go_pos(int n, int hand);
void save_calib(int n, int hand);

int ZeroGainLeftArm();
int ZeroGainRightArm();

int ForceToPositionControl(void);
int ForceToPositionControl_LeftArm();
int ForceToPositionControl_RightArm();

int GravityCompensation_LeftArm();
int GravityCompensation_RightArm();

void ForceControl_LeftArm(double Fx, double Fy, double Fz);
void ForceControl_RightArm(double Fx, double Fy, double Fz);
void ForceTorqueControl_LeftArm(double Fx, double Fy, double Fz, double Mx, double My, double Mz);
void ForceTorqueControl_RightArm(double Fx, double Fy, double Fz, double Mx, double My, double Mz);

int ZForceXYPositionControl_LeftArm(double x_ref, double y_ref, double Fz_ref, double LElb_ref);
int ZForceXYPositionControl_RightArm(double x_ref, double y_ref, double Fz_ref, double RElb_ref);

int XYZPositionControl_RightArm(double x_ref, double y_ref, double z_ref, double RElb_ref);
int XYZPositionControl_LeftArm(double x_ref, double y_ref, double z_ref, double LElb_ref);

int PositionControlByForce_RightArm(double x_ref, double y_ref, double z_ref, double *qRH_4x1_ref, double RElb_ref);
int PositionControlByForce_LeftArm(double x_ref, double y_ref, double z_ref, double *qLH_4x1_ref, double LElb_ref);

void JointForceControl_LeftArm(double LSP_ang, double LSR_ang, double LSY_ang, double LEB_ang, double LWY_ang, double LWP_ang, double LF1_ang);
void JointForceControl_RightArm(double RSP_ang, double RSR_ang, double RSY_ang, double REB_ang, double RWY_ang, double RWP_ang, double RF1_ang);

int FK_Pos_RightArm(double _pRH_3x1[]);
int FK_Pos_Ori_Elb_RightArm(double _pRH_3x1[], double _qRH_4x1[], double &_RElb);
int FK_Pos_LeftArm(double _pLH_3x1[]);
int FK_Pos_Ori_Elb_LeftArm(double _pLH_3x1[], double _qLH_4x1[], double &_LElb);

int torque_OLcurrent_map_LSP(double torque_Nm);
int torque_OLcurrent_map_LSR(double torque_Nm);
int torque_OLcurrent_map_LSY(double torque_Nm);
int torque_OLcurrent_map_LEB(double torque_Nm);
int torque_OLcurrent_map_LWY(double torque_Nm);
int torque_OLcurrent_map_LWP(double torque_Nm);
int torque_OLcurrent_map_LWY2(double torque_Nm);

int torque_OLcurrent_map_RSP(double torque_Nm);
int torque_OLcurrent_map_RSR(double torque_Nm);
int torque_OLcurrent_map_RSY(double torque_Nm);
int torque_OLcurrent_map_REB(double torque_Nm);
int torque_OLcurrent_map_RWY(double torque_Nm);
int torque_OLcurrent_map_RWP(double torque_Nm);
int torque_OLcurrent_map_RWY2(double torque_Nm);







#endif // FUNCTIONS_H
