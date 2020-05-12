#ifndef HB_FUNCTIONS_H
#define HB_FUNCTIONS_H

#include <iostream>
#include <stack>
#include <stdlib.h>
#include "BasicMatrix.h"
#include <iostream>
//#include "cmatrix"
#include "ManualCAN.h"
#include <unistd.h>
#include "BP_RBDL.h"
//#include "HB_dynamics_walk.h"
#include "../../share/Headers/UserSharedMemory.h"

//typedef techsoft::matrix<double> Matrix;
//typedef std::valarray<double> Vector;

#define     FootLength        0.22
#define     FootWidth         0.15
#define     freq              500.0

#define     PI                3.141592
#define D2R			1.745329251994330e-2
#define R2D			5.729577951308232e1


using namespace std;

extern pUSER_SHM                userData;

// Variables---------------------------------------------
struct Point
{
    double x, y;
};

// A globle point needed for  sorting points with reference
// to  the first point Used in compare function of qsort()
//Point pp0;

//*****************----FUNCTIONS---********************************

// for Convex Hull---------------------------
Point nextToTop(stack<Point> &S);
int swap(Point &p1, Point &p2);
double distSq(Point p1, Point p2);
int orientation(Point p, Point q, Point r);
int compare(const void *vp1, const void *vp2);
stack<Point> convexHull(Point points[], int n, int &n_final);
//------------------------------------------------

// ZMP in/out Checking---------------------------
bool zmpInOutCheck(Point zmp, stack<Point> SP);
int left_or_right(Point p, Point q, Point r);
int compare_double(const void *vp1, const void *vp2);
vec3 zmpProjectionToSP(vec3 _zmp, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF);
vec3 zmpProjectionToSP_large(vec3 _zmp, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF, double _offset);
vec3 zmpProjectionToSP_offset(vec3 _zmp, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF, double _Xoffset, double _Yoffset);
vec3 zmpProjectionTo_StanceFoot(int _swingFoot, vec3 _Vector_stanceFootFrame, double _Xoffset, double _Yoffset);
//-----------------------------------------------

// ZMP Calculation--------------------------------------
vec3 ZMP_calc_global(vec3 _pRF, quat _qRF, vec3 _F_RF, vec3 _M_RF, vec3 _pLF, quat _qLF, vec3 _F_LF,vec3 _M_LF);
vec3 Calc_local(vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF, vec3 _pGlobal);
vec3 local2global_vec(quat _qRF, quat _qLF, vec3 _VecLocal);
vec3 local2global_point(quat _qRF, quat _qLF, vec3 _pRF, vec3 _pLF, vec3 _pLocal);
vec3 global2local_vec(quat _qRF, quat _qLF, vec3 _VecGlobal);
quat global2local_quat(quat _qRF, quat _qLF, quat _QuatGlobal);
vec3 global2local_point(quat _qRF, quat _qLF, vec3 _pRF, vec3 _pLF, vec3 _pGlobal);
double get_Pel_yaw_from_qRF_qLF(quat _qRF, quat _qLF);

// COM measurement
vec3 COM_measurment(vec3 _COM, double _zc, vec3 _ZMP_global, vec3 _IMUangle, quat _qPel);
vec3 COM_measurment_by_quat(vec3 _COM, vec3 _ZMP_global, quat _IMUquat, quat _qPel);
vec3 dCOM_measurement_by_quat(vec3 _uCOM, vec3 _udCOM, vec3 _ZMP_global, quat _IMUquat, vec3 _IMUomega);
quat Global_qPel_measurment(vec3 _IMUangle, quat _qPel);

// Foot trajectory Generation
vec3 FootZ_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _Z_dZ_ddZ_Foot_old, double _MaxFootUp, double _minFootDown);
vec3 FootZ_Landing_trajectory(double _landing_time, double _t_foot_now, vec3 _Z_dZ_ddZ_Foot_old, double _minFootDown);
vec3 FootXY_Landing_trajectory(double _landing_time, double _t_foot_now, vec3 _p_dp_ddp_Foot_old, double _goal);
double LandingController(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _F_Foot, vec3 _X_dX_ddX_Foot, double _del_z_old);
vec3 FootZ_recovary(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _Z_dZ_ddZ_Foot_old, double _last_landing_height);
vec3 FootX_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _X_dX_ddX_Foot_old, double _X_footStep);
vec3 FootY_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _y_dy_ddy_Foot_old, double _Y_footStep);
vec3 Foot_yaw_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _yaw_d_dd_old, double _des_Yaw);
VectorNd calc_5th(double t_0, double t_e, vec3 h_0, vec3 h_e);
vec3 calc_5th_GG(double t_now, double t_e, vec3 p_init, vec3 p_end);
vec3 calc_3rd(double t_now, double t_e, vec3 p_init, vec3 p_end);
vec3 calc_3rd_t0_to_tend(double t_0, double t_e, double t_now, vec3 p_init, vec3 p_end);
double calc_trap(double _del_t, double _te, double _he, double _t_now);
vec3 FootZ_recovary_trapezoidal(double _t_step, double _t_now, double _dsp_ratio, double _del_t, double _max_speed, vec3 _Z_dZ_ddZ_old);

double FootY_pos_limiter(char _swingFoot, vec3 _pPel, vec3 _pFoot_des, vec3 _pRF_current, vec3 _pLF_current);
double FootX_pos_limiter(vec3 _pPel, vec3 _pFoot_des);
//quat FootOri_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, quat _qFoot_old, double _del_x);

// Foot FT Distributor
VectorNd AnkleFT_Calculator(vec3 _ZMP_ref, quat _qPel, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF);
vec4 zmpFeedBackController(int _on_off, vec3 _ZMP_ref, vec3 _dZMP_ref, vec3 _ZMP_m, vec3 _dZMP_m, quat _qRF, quat _qLF, vec3 _F_RF, vec3 _F_LF);
vec4 AnkleJointAngleContoller(int _ctrl_mode, vec3 _F_RF, vec3 _F_LF, vec4 _R_Ank_angVel, vec4 _L_Ank_angVel, vec4 _R_Ank_ref, vec4 _L_Ank_ref);

// COM damping Controller
vec3 ComDampingController(vec3 _COM_con_old, vec3 _COM_input, vec3 _dCOM_input, vec3 _COM_m, vec3 _dCOM_m, vec3 _F_RF, vec3 _F_LF);

// Ankle Torque Control
void AnkleToruqeControl_Init(void);
void AnkleTorqueControl_Stop(void);
void AnkleTorqueControl(double _RAR_T_ref, double _RAP_T_ref, double _LAR_T_ref, double _LAP_T_ref,
                        vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF);

int torque_OLcurrent_map_RAR(double torque_Nm);
int torque_OLcurrent_map_RAP(double torque_Nm);
int torque_OLcurrent_map_LAR(double torque_Nm);
int torque_OLcurrent_map_LAP(double torque_Nm);

//torso orientation controller
vec3 HipRoll_Compensation(char _swingFoot, double _real_t_step, double _t_foot_now, double _dsp_ratio);

//step pos & time control
double StepTimeLimiter(double _step_time_gap, double _step_time_modi_old, double _dT_original, double _min_step_time);
vec3 CP_eos_Limitor(vec3 _CP_eos_gap, char swingFoot, double Foot_ori_x, double Foot_ori_y, vec3 _pRF_old, vec3 _pLF_old, double _dT);

// Heel To Toe Pattern generation
vec4 Last_toe_Next_heel(char _last_RL, vec3 _last_Fpos, quat _last_Fquat, vec3 _next_Fpos, quat _next_Fquat);


//general
vec3 Fifth_trajectory(double _t_now, double _T, vec3 _x_dx_ddx_old, vec3 _x_dx_ddx);
void calc_5th_param(double t_now, double t_e, vec3 p_init, vec3 p_end, double &A, double &B, double &C, double &D, double &E, double &F);
void calc_3rd_param(double t_now, double t_e, vec3 p_init, vec3 p_end, double &A, double &B, double &C, double &D);
void calc_1st_param(double t_now, double t_e, double p_init, double p_end, double &A, double &B);
double HB_sign(double _a);
mat3 inverse_HB(mat3 _M);
quat inverse_HB(quat _Q);

#endif // HB_FUNCTIONS_H








