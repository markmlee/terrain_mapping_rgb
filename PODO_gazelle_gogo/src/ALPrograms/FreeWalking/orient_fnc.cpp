//////////////////////////////////////
// orient_fnc.cpp
// Inhyeok Kim
// Rainbow Inc. 2014. 12. 15
//////////////////////////////////////

#include <stdio.h>

#include "orient_fnc.h"
#include "../../../share/Headers/ik_math2.h"
#include "../../../share/Headers/kine_drc_hubo2.h"
//#include "lib/kine_drc_hubo2.h"
//#include "lib/ik_math2.h"
#include <math.h>
#include <string.h>
#include "WalkingFSM.h"
enum{
    HUBO_RIGHT = 0,
    HUBO_LEFT
};
enum{
    RIGHT = 0,
    LEFT
};
enum{
    Xdir = 0,
    Ydir,
    Zdir
};
enum FTNAME{
    RAFT = 0,
    LAFT
};
extern CKINE_DRC_HUBO2 _hubo2_complete_form;
extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

extern double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero);

extern double WBIKTorqInitConRAR(double _ref, double _torque, int _zero, double _gain);
extern double WBIKTorqInitConLAR(double _ref, double _torque, int _zero, double _gain);
extern double WBIKTorqInitConRAP(double _ref, double _torque, int _zero, double _gain);
extern double WBIKTorqInitConLAP(double _ref, double _torque, int _zero, double _gain);


int QT2EULER(const double q_4x1[], double &roll, double &pitch, double &yaw)
{
    double temp_3x3[9];

    QT2DC(q_4x1, temp_3x3);

//    roll = atan2(temp_3x3[3*2+1], temp_3x3[3*2+2]);
//   yaw = atan2(temp_3x3[3*1+0], temp_3x3[3*0+0]);
//    pitch = atan2(-temp_3x3[3*2+0], temp_3x3[3*0+0]*cos(yaw)+temp_3x3[3*1+0]*sin(yaw));

    pitch = atan2(-temp_3x3[3*2+0], temp_3x3[3*2+2]);
    yaw = atan2(temp_3x3[3*1+0]*cos(pitch)+temp_3x3[3*1+2]*sin(pitch), temp_3x3[3*0+0]*cos(pitch)+temp_3x3[3*0+2]*sin(pitch));
    roll = atan2(temp_3x3[3*2+1], temp_3x3[3*2+2]*cos(pitch)-temp_3x3[3*2+0]*sin(pitch));

    roll = roll*R2D;
    pitch = pitch*R2D;
    yaw = yaw*R2D;
    return 0;
}
//---------------------------------------------------------------
//int QT2YPR(const double qt_4x1[], double &yaw, double &pit, double &rol)
//{

//    double dc[9];
//    QT2DC(qt_4x1, dc);

//    rol = atan2(dc[2*3+1], dc[2*3+2]);
//    pit = atan2(-dc[2*3+0], dc[2*3+2]*cos(rol)+dc[2*3+1]*sin(rol));
//    yaw = atan2(-(dc[0*3+1]*cos(rol)-dc[0*3+2]*sin(rol)), dc[1*3+1]*cos(rol)-dc[1*3+2]*sin(rol));

//    rol = rol*R2D;
//    pit = pit*R2D;
//    yaw = yaw*R2D;
//    return 0;
//}


////---------------------------------------------------------------
//int QT2YRP(const double qt_4x1[], double &yaw, double &rol, double &pit)
//{
//    double dc[9];
//    QT2DC(qt_4x1, dc);

//    pit = atan2(-dc[2*3+0], dc[2*3+2]);
//    yaw = atan2(dc[1*3+0]*cos(pit) + dc[1*3+2]*sin(pit), dc[0*3+0]*cos(pit) + dc[0*3+2]*sin(pit));
//    rol = atan2(dc[2*3+1], dc[2*3+2]*cos(pit) - dc[2*3+0]*sin(pit));

//    rol = rol*R2D;
//    pit = pit*R2D;
//    yaw = yaw*R2D;
//    return 0;
//}
int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[])
{
    double prf_3x1[3] = {pRF_3x1[0], pRF_3x1[1], pRF_3x1[2]};
    double plf_3x1[3] = {pLF_3x1[0], pLF_3x1[1], pLF_3x1[2]};
    double stance_x_3x1[3], stance_y_3x1[3];
    double stance_z_3x1[3] = {0, 0, 1};
    double stance_frame_3x3[9], pel_global_3x3[9];
    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];
    double frame_meas_3x3[9];

    prf_3x1[2] = 0.;
    plf_3x1[2] = 0.;

    diff_vv(plf_3x1,3, prf_3x1, stance_y_3x1);
    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);

    temp = norm_v(stance_y_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_y_3x1[0] /= temp;
    stance_y_3x1[1] /= temp;
    stance_y_3x1[2] /= temp;

    temp = norm_v(stance_x_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_x_3x1[0] /= temp;
    stance_x_3x1[1] /= temp;
    stance_x_3x1[2] /= temp;

    stance_frame_3x3[0] = stance_x_3x1[0];
    stance_frame_3x3[3] = stance_x_3x1[1];
    stance_frame_3x3[6] = stance_x_3x1[2];

    stance_frame_3x3[1] = stance_y_3x1[0];
    stance_frame_3x3[4] = stance_y_3x1[1];
    stance_frame_3x3[7] = stance_y_3x1[2];

    stance_frame_3x3[2] = stance_z_3x1[0];
    stance_frame_3x3[5] = stance_z_3x1[1];
    stance_frame_3x3[8] = stance_z_3x1[2];

//    qtRZ(euler_global_z, temp2_4x1);
//    qtRY(euler_global_y, temp3_4x1);
//    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
//    qtRX(euler_global_x, temp2_4x1);
//    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    qtRZ(euler_global_z, temp2_4x1);
    qtRX(euler_global_x, temp3_4x1);
    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
    qtRY(euler_global_y, temp2_4x1);
    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    QT2DC(temp3_4x1, pel_global_3x3);

    mult_mm(pel_global_3x3,3,3,stance_frame_3x3,3, frame_meas_3x3);

    transpose(1., stance_frame_3x3, 3,3);
    mult_mm(stance_frame_3x3,3,3, frame_meas_3x3,3, temp5_3x3);

    euler_stance_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*2+2]);
    euler_stance_z = atan2(temp5_3x3[3*1+0]*cos(euler_stance_y)+temp5_3x3[3*1+2]*sin(euler_stance_y), temp5_3x3[3*0+0]*cos(euler_stance_y)+temp5_3x3[3*0+2]*sin(euler_stance_y));
    euler_stance_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]*cos(euler_stance_y)-temp5_3x3[3*2+0]*sin(euler_stance_y));

    double temp_integral;
    temp_integral=PitchRoll_Ori_Integral(0.0, euler_stance_y, 1);
    //printf("PitchRoll_Ori_Integral = %f\n",temp_integral*R2D);

    qPEL_comp_4x1[0] = cos(temp_integral);
    qPEL_comp_4x1[1] = stance_y_3x1[0]*sin(temp_integral);
    qPEL_comp_4x1[2] = stance_y_3x1[1]*sin(temp_integral);
    qPEL_comp_4x1[3] = stance_y_3x1[2]*sin(temp_integral);

    return 0;
 }
int convert_euler_FT(double _pre_state,double pRF_3x1[], double pLF_3x1[],double qRF_4x1[], double qLF_4x1[], double control_result_RF[],double control_result_LF[])
{
    double prf_3x1[3] = {pRF_3x1[0], pRF_3x1[1], pRF_3x1[2]};
    double plf_3x1[3] = {pLF_3x1[0], pLF_3x1[1], pLF_3x1[2]};
    double stance_x_3x1[3], stance_y_3x1[3];
    double stance_z_3x1[3] = {0, 0, 1};
    double stance_frame_3x3[9];
    double temp;

    prf_3x1[2] = 0.;
    plf_3x1[2] = 0.;

    diff_vv(plf_3x1,3, prf_3x1, stance_y_3x1);
    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);

    temp = norm_v(stance_y_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_y_3x1[0] /= temp;
    stance_y_3x1[1] /= temp;
    stance_y_3x1[2] /= temp;

    temp = norm_v(stance_x_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_x_3x1[0] /= temp;
    stance_x_3x1[1] /= temp;
    stance_x_3x1[2] /= temp;

    stance_frame_3x3[0] = stance_x_3x1[0];
    stance_frame_3x3[3] = stance_x_3x1[1];
    stance_frame_3x3[6] = stance_x_3x1[2];

    stance_frame_3x3[1] = stance_y_3x1[0];
    stance_frame_3x3[4] = stance_y_3x1[1];
    stance_frame_3x3[7] = stance_y_3x1[2];

    stance_frame_3x3[2] = stance_z_3x1[0];
    stance_frame_3x3[5] = stance_z_3x1[1];
    stance_frame_3x3[8] = stance_z_3x1[2];

    double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],M_LF_Stance[3],M_RF_Stance[3];

    M_LF[0] =  sharedSEN->FT[LAFT].Mx -0.03*sharedSEN->FT[LAFT].Fy;
    M_LF[1] =  sharedSEN->FT[LAFT].My -0.03*sharedSEN->FT[LAFT].Fx;
    M_LF[2] =  sharedSEN->FT[LAFT].Mz;

    QTtransform(qLF_4x1,M_LF,M_LF_Global);

    M_RF[0] =  sharedSEN->FT[RAFT].Mx -0.03*sharedSEN->FT[RAFT].Fy;
    M_RF[1] =  sharedSEN->FT[RAFT].My -0.03*sharedSEN->FT[RAFT].Fx;
    M_RF[2] =  sharedSEN->FT[RAFT].Mz;

    QTtransform(qRF_4x1,M_RF,M_RF_Global);

    transpose(1., stance_frame_3x3, 3,3);
    
    mult_mv(stance_frame_3x3,3,3,M_LF_Global,M_LF_Stance);
    mult_mv(stance_frame_3x3,3,3,M_RF_Global,M_RF_Stance);
        
    double control_RF[3],control_LF[3];
    if(_pre_state == HUBO_RIGHT)
    {
       control_RF[Xdir] = -WBIKTorqInitConRAR(0, M_RF_Stance[0], 1,0.0003);
       control_LF[Xdir] = -WBIKTorqInitConLAR(0, M_LF_Stance[0], 1,0.0003);
       
       control_RF[Ydir] = -WBIKTorqInitConRAP(0, M_RF_Stance[1]-M_LF_Stance[1], 1,0.0003);
       control_LF[Ydir] = 0.;

       control_RF[Zdir] = 0.;
       control_LF[Zdir] = 0.;
    }
    else
    {
        control_RF[Xdir] = -WBIKTorqInitConRAR(0, M_RF_Stance[0], 1,0.0003);
        control_LF[Xdir] = -WBIKTorqInitConLAR(0, M_LF_Stance[0], 1,0.0003);
        
        control_RF[Ydir] = 0.;
        control_LF[Ydir] =  WBIKTorqInitConLAP(0, M_RF_Stance[1]-M_LF_Stance[1], 1,0.0003);

        control_RF[Zdir] = 0.;
        control_LF[Zdir] = 0.;
    }

    transpose(1., stance_frame_3x3, 3,3);

    mult_mv(stance_frame_3x3,3,3,control_RF,control_result_RF);
    mult_mv(stance_frame_3x3,3,3,control_LF,control_result_LF);


    return 0;
 }
int convert_euler_imu(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z)
{
    double prf_3x1[3] = {pRF_3x1[0], pRF_3x1[1], pRF_3x1[2]};
    double plf_3x1[3] = {pLF_3x1[0], pLF_3x1[1], pLF_3x1[2]};
    double stance_x_3x1[3], stance_y_3x1[3];
    double stance_z_3x1[3] = {0, 0, 1};
    double stance_frame_3x3[9], pel_global_3x3[9];
    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];
    double frame_meas_3x3[9];

    prf_3x1[2] = 0.;
    plf_3x1[2] = 0.;

    diff_vv(plf_3x1,3, prf_3x1, stance_y_3x1);
    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);

    temp = norm_v(stance_y_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_y_3x1[0] /= temp;
    stance_y_3x1[1] /= temp;
    stance_y_3x1[2] /= temp;

    temp = norm_v(stance_x_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_x_3x1[0] /= temp;
    stance_x_3x1[1] /= temp;
    stance_x_3x1[2] /= temp;

    stance_frame_3x3[0] = stance_x_3x1[0];
    stance_frame_3x3[3] = stance_x_3x1[1];
    stance_frame_3x3[6] = stance_x_3x1[2];

    stance_frame_3x3[1] = stance_y_3x1[0];
    stance_frame_3x3[4] = stance_y_3x1[1];
    stance_frame_3x3[7] = stance_y_3x1[2];

    stance_frame_3x3[2] = stance_z_3x1[0];
    stance_frame_3x3[5] = stance_z_3x1[1];
    stance_frame_3x3[8] = stance_z_3x1[2];

//    qtRZ(euler_global_z, temp2_4x1);
//    qtRY(euler_global_y, temp3_4x1);
//    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
//    qtRX(euler_global_x, temp2_4x1);
//    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    qtRZ(euler_global_z, temp2_4x1);
    qtRX(euler_global_x, temp3_4x1);
    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
    qtRY(euler_global_y, temp2_4x1);
    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    QT2DC(temp3_4x1, pel_global_3x3);

    mult_mm(pel_global_3x3,3,3,stance_frame_3x3,3, frame_meas_3x3);

    transpose(1., stance_frame_3x3, 3,3);
    mult_mm(stance_frame_3x3,3,3, frame_meas_3x3,3, temp5_3x3);

    euler_stance_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*2+2]);
    euler_stance_z = atan2(temp5_3x3[3*1+0]*cos(euler_stance_y)+temp5_3x3[3*1+2]*sin(euler_stance_y), temp5_3x3[3*0+0]*cos(euler_stance_y)+temp5_3x3[3*0+2]*sin(euler_stance_y));
    euler_stance_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]*cos(euler_stance_y)-temp5_3x3[3*2+0]*sin(euler_stance_y));

    return 0;
 }
int convert_euler_imu_vel(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z)
{
    double prf_3x1[3] = {pRF_3x1[0], pRF_3x1[1], pRF_3x1[2]};
    double plf_3x1[3] = {pLF_3x1[0], pLF_3x1[1], pLF_3x1[2]};
    double stance_x_3x1[3], stance_y_3x1[3];
    double stance_z_3x1[3] = {0, 0, 1};
    double stance_frame_3x3[9], pel_global_3x3[9];
    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];
    double frame_meas_3x3[9];

    prf_3x1[2] = 0.;
    plf_3x1[2] = 0.;

    diff_vv(plf_3x1,3, prf_3x1, stance_y_3x1);
    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);

    temp = norm_v(stance_y_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_y_3x1[0] /= temp;
    stance_y_3x1[1] /= temp;
    stance_y_3x1[2] /= temp;

    temp = norm_v(stance_x_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_x_3x1[0] /= temp;
    stance_x_3x1[1] /= temp;
    stance_x_3x1[2] /= temp;

    stance_frame_3x3[0] = stance_x_3x1[0];
    stance_frame_3x3[3] = stance_x_3x1[1];
    stance_frame_3x3[6] = stance_x_3x1[2];

    stance_frame_3x3[1] = stance_y_3x1[0];
    stance_frame_3x3[4] = stance_y_3x1[1];
    stance_frame_3x3[7] = stance_y_3x1[2];

    stance_frame_3x3[2] = stance_z_3x1[0];
    stance_frame_3x3[5] = stance_z_3x1[1];
    stance_frame_3x3[8] = stance_z_3x1[2];

//    qtRZ(euler_global_z, temp2_4x1);
//    qtRY(euler_global_y, temp3_4x1);
//    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
//    qtRX(euler_global_x, temp2_4x1);
//    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    qtRZ(euler_global_z, temp2_4x1);
    qtRX(euler_global_x, temp3_4x1);
    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
    qtRY(euler_global_y, temp2_4x1);
    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    QT2DC(temp3_4x1, pel_global_3x3);

    mult_mm(pel_global_3x3,3,3,stance_frame_3x3,3, frame_meas_3x3);

    transpose(1., stance_frame_3x3, 3,3);
    mult_mm(stance_frame_3x3,3,3, frame_meas_3x3,3, temp5_3x3);

    euler_stance_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*2+2]);
    euler_stance_z = atan2(temp5_3x3[3*1+0]*cos(euler_stance_y)+temp5_3x3[3*1+2]*sin(euler_stance_y), temp5_3x3[3*0+0]*cos(euler_stance_y)+temp5_3x3[3*0+2]*sin(euler_stance_y));
    euler_stance_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]*cos(euler_stance_y)-temp5_3x3[3*2+0]*sin(euler_stance_y));

    return 0;
 }


//int complete_form(const double Q_34x1[],
//                  double imu_euler_x, double imu_euler_y, double imu_euler_z,
//                  double *des_pCOM_3x1, double *des_qPEL_4x1,
//                  double *des_pRF_3x1, double *des_qRF_4x1,
//                  double *des_pLF_3x1, double *des_qLF_4x1)
//{
//    double q_34x1[34] = {0,};
//    double pCOM_v_3x1[3], qPEL_v_4x1[4], pRF_v_3x1[3], pLF_v_3x1[3], qRF_v_4x1[4], qLF_v_4x1[4];
//    double pCOM_vl_3x1[3], pRF_vl_3x1[3], pLF_vl_3x1[3], qRF_vl_4x1[4], qLF_vl_4x1[4], height_com;
//    double pCOM_m_3x1[3], qPEL_m_4x1[4], pRF_m_3x1[3], pLF_m_3x1[3], qRF_m_4x1[4], qLF_m_4x1[4];
//    double des_euler_x, des_euler_y, des_euler_z;
//    double stance_x_3x1[3], stance_y_3x1[3], stance_z_3x1[3] = {0, 0, 1};
//    double stance_frame_3x3[9];
//    double stance_rol, stance_pit;
//    double stance_center_3x1[3];

//    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];

//    memcpy(q_34x1, Q_34x1, 34*sizeof(double));
//    memcpy(qPEL_v_4x1, &Q_34x1[idQ0], 4*sizeof(double));
//    memcpy(des_qPEL_4x1, qPEL_v_4x1, 4*sizeof(double));
//    QT2DC(des_qPEL_4x1, temp5_3x3);
//    des_euler_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]);
//    des_euler_z = atan2(temp5_3x3[3*1+0], temp5_3x3[3*0+0]);
//    des_euler_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*0+0]*cos(des_euler_z)+temp5_3x3[3*1+0]*sin(des_euler_z));

//    _hubo2_complete_form.FK_COM_Global(q_34x1, pCOM_v_3x1);
//    _hubo2_complete_form.FK_RightFoot_Global(q_34x1, pRF_v_3x1, qRF_v_4x1);
//    _hubo2_complete_form.FK_LeftFoot_Global(q_34x1, pLF_v_3x1, qLF_v_4x1);

//    temp2_4x1[0] = qPEL_v_4x1[0];
//    temp2_4x1[1] = -qPEL_v_4x1[1];
//    temp2_4x1[2] = -qPEL_v_4x1[2];
//    temp2_4x1[3] = -qPEL_v_4x1[3];

//    diff_vv(pCOM_v_3x1,3,q_34x1,temp3_4x1);
//    QTtransform(temp2_4x1,temp3_4x1,pCOM_vl_3x1);

//    diff_vv(pRF_v_3x1,3,q_34x1,temp3_4x1);
//    QTtransform(temp2_4x1,temp3_4x1,pRF_vl_3x1);
//    QTcross(temp2_4x1, qRF_v_4x1, qRF_vl_4x1);

//    diff_vv(pLF_v_3x1,3,q_34x1,temp3_4x1);
//    QTtransform(temp2_4x1,temp3_4x1,pLF_vl_3x1);
//    QTcross(temp2_4x1, qLF_v_4x1, qLF_vl_4x1);

//    height_com =  pCOM_vl_3x1[2]-0.5*(pRF_vl_3x1[2]+pLF_vl_3x1[2]);

//    qtRZ(des_euler_z, temp2_4x1);
//    qtRY(imu_euler_y, temp3_4x1);
//    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
//    qtRX(imu_euler_x, temp2_4x1);
//    QTcross(temp4_4x1,temp2_4x1, qPEL_m_4x1);

//    memcpy(&q_34x1[idQ0], qPEL_m_4x1, 4*sizeof(double));

//    _hubo2_complete_form.FK_COM_Global(q_34x1, pCOM_m_3x1);
//    _hubo2_complete_form.FK_RightFoot_Global(q_34x1, pRF_m_3x1, qRF_m_4x1);
//    _hubo2_complete_form.FK_LeftFoot_Global(q_34x1, pLF_m_3x1, qLF_m_4x1);

//    diff_vv(pLF_m_3x1,3, pRF_m_3x1, stance_y_3x1);
//    temp = norm_v(stance_y_3x1,3);
//    if(temp < 1e-6)
//        return -1;
//    stance_y_3x1[0] /= temp;
//    stance_y_3x1[1] /= temp;
//    stance_y_3x1[2] /= temp;

//    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);
//    temp = norm_v(stance_x_3x1,3);
//    if(temp < 1e-6)
//        return -1;
//    stance_x_3x1[0] /= temp;
//    stance_x_3x1[1] /= temp;
//    stance_x_3x1[2] /= temp;

//    cross(1., stance_x_3x1, stance_y_3x1, stance_z_3x1);
//    temp = norm_v(stance_z_3x1,3);
//    if(temp < 1e-6)
//        return -1;
//    stance_z_3x1[0] /= temp;
//    stance_z_3x1[1] /= temp;
//    stance_z_3x1[2] /= temp;

//    stance_frame_3x3[0] = stance_x_3x1[0];
//    stance_frame_3x3[3] = stance_x_3x1[1];
//    stance_frame_3x3[6] = stance_x_3x1[2];

//    stance_frame_3x3[1] = stance_y_3x1[0];
//    stance_frame_3x3[4] = stance_y_3x1[1];
//    stance_frame_3x3[7] = stance_y_3x1[2];

//    stance_frame_3x3[2] = stance_z_3x1[0];
//    stance_frame_3x3[5] = stance_z_3x1[1];
//    stance_frame_3x3[8] = stance_z_3x1[2];

//    sum_svsv(0.5,pRF_m_3x1,3, 0.5,pLF_m_3x1, stance_center_3x1);

//    stance_rol = -asin(stance_frame_3x3[6]-stance_frame_3x3[7]);
//    DC2QT(stance_frame_3x3, temp2_4x1);
//    temp2_4x1[1] *= -1;
//    temp2_4x1[2] *= -1;
//    temp2_4x1[3] *= -1;
//    QTcross(temp2_4x1,qRF_m_4x1,temp3_4x1);
//    QTcross(temp2_4x1,qLF_m_4x1,temp4_4x1);
//    stance_pit = 2*(atan2(temp3_4x1[2],temp3_4x1[0])+atan2(temp4_4x1[2],temp4_4x1[0]))/2.;

//    if(fabs(stance_rol) > 30*D2R || fabs(stance_pit) > 30*D2R)
//    {
//        memcpy(des_pCOM_3x1, pCOM_v_3x1, 3*sizeof(double));
//        memcpy(des_pRF_3x1, pRF_v_3x1, 3*sizeof(double));
//        memcpy(des_pLF_3x1, pLF_v_3x1, 3*sizeof(double));
//        memcpy(des_qRF_4x1, qRF_v_4x1, 4*sizeof(double));
//        memcpy(des_qLF_4x1, qLF_v_4x1, 4*sizeof(double));
//        return -1;
//    }

//    des_pCOM_3x1[0] = stance_center_3x1[0];
//    des_pCOM_3x1[1] = stance_center_3x1[1];
//    des_pCOM_3x1[2] = stance_center_3x1[2] + height_com;

//    memcpy(des_pRF_3x1, pRF_m_3x1, 3*sizeof(double));
//    memcpy(des_pLF_3x1, pLF_m_3x1, 3*sizeof(double));

//    memcpy(des_qRF_4x1, qRF_m_4x1, 4*sizeof(double));
//    memcpy(des_qLF_4x1, qLF_m_4x1, 4*sizeof(double));

//    temp2_4x1[0] = 0.5*((pRF_v_3x1[0]+pLF_v_3x1[0]) - (pRF_m_3x1[0]+pLF_m_3x1[0]));
//    temp2_4x1[1] = 0.5*((pRF_v_3x1[1]+pLF_v_3x1[1]) - (pRF_m_3x1[1]+pLF_m_3x1[1]));
//    temp2_4x1[2] = 0.5*((pRF_v_3x1[2]+pLF_v_3x1[2]) - (pRF_m_3x1[2]+pLF_m_3x1[2]));

//    sum_vv(des_pCOM_3x1,3, temp2_4x1, des_pCOM_3x1);
//    sum_vv(des_pLF_3x1,3, temp2_4x1, des_pLF_3x1);
//    sum_vv(des_pRF_3x1,3, temp2_4x1, des_pRF_3x1);

//    return 0;
//}


//int complete_form_delta(const double Q_current_34x1[],
//                      double euler_x_err, double euler_y_err, double euler_z_err,
//                      double *des_pCOM_3x1, double *des_qPEL_4x1,
//                      double *des_pRF_3x1, double *des_qRF_4x1,
//                      double *des_pLF_3x1, double *des_qLF_4x1)
//{
//    double q_v_34x1[34] = {0,};
//    double pCOM_v_3x1[3], qPEL_v_4x1[4], pRF_v_3x1[3], pLF_v_3x1[3], qRF_v_4x1[4], qLF_v_4x1[4];
////    double euler_x, euler_y, euler_z;
//    double stance_x_3x1[3], stance_y_3x1[3], stance_z_3x1[3] = {0, 0, 1};
//    double stance_frame_3x3[9];
//    double stance_rol, stance_pit;
//    double stance_center_3x1[3];

//    double qErr_4x1[4];

//    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];

//    memcpy(q_v_34x1, Q_current_34x1, 34*sizeof(double));
//    memcpy(qPEL_v_4x1, &Q_current_34x1[idQ0], 4*sizeof(double));
//    memcpy(des_qPEL_4x1, qPEL_v_4x1, 4*sizeof(double));
//    QT2DC(des_qPEL_4x1, temp5_3x3);
////    euler_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]);
////    euler_z = atan2(temp5_3x3[3*1+0], temp5_3x3[3*0+0]);
////    euler_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*0+0]*cos(euler_z)+temp5_3x3[3*1+0]*sin(euler_z));

//    _hubo2_complete_form.FK_COM_Global(q_v_34x1, pCOM_v_3x1);
//    _hubo2_complete_form.FK_RightFoot_Global(q_v_34x1, pRF_v_3x1, qRF_v_4x1);
//    _hubo2_complete_form.FK_LeftFoot_Global(q_v_34x1, pLF_v_3x1, qLF_v_4x1);

//    qtRZ(-euler_z_err, temp2_4x1);
//    qtRY(-euler_y_err, temp3_4x1);
//    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
//    qtRX(-euler_x_err, temp2_4x1);
//    QTcross(temp4_4x1,temp2_4x1, qErr_4x1);

//    sum_svsv(0.5,pRF_v_3x1,3, 0.5,pLF_v_3x1, stance_center_3x1);

//    diff_vv(pRF_v_3x1,3,stance_center_3x1, temp2_4x1);
//    QTtransform(qErr_4x1,temp2_4x1,temp3_4x1);
//    sum_vv(stance_center_3x1,3,temp3_4x1, des_pRF_3x1);

//    diff_vv(pLF_v_3x1,3,stance_center_3x1, temp2_4x1);
//    QTtransform(qErr_4x1,temp2_4x1,temp3_4x1);
//    sum_vv(stance_center_3x1,3,temp3_4x1, des_pLF_3x1);

//    QTcross(qErr_4x1,qRF_v_4x1, des_qRF_4x1);
//    QTcross(qErr_4x1,qLF_v_4x1, des_qLF_4x1);

//    diff_vv(des_pLF_3x1,3, des_pRF_3x1, stance_y_3x1);
//    temp = norm_v(stance_y_3x1,3);
//    if(temp < 1e-6)
//        return -1;
//    stance_y_3x1[0] /= temp;
//    stance_y_3x1[1] /= temp;
//    stance_y_3x1[2] /= temp;

//    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);
//    temp = norm_v(stance_x_3x1,3);
//    if(temp < 1e-6)
//        return -1;
//    stance_x_3x1[0] /= temp;
//    stance_x_3x1[1] /= temp;
//    stance_x_3x1[2] /= temp;

//    cross(1., stance_x_3x1, stance_y_3x1, stance_z_3x1);
//    temp = norm_v(stance_z_3x1,3);
//    if(temp < 1e-6)
//        return -1;
//    stance_z_3x1[0] /= temp;
//    stance_z_3x1[1] /= temp;
//    stance_z_3x1[2] /= temp;

//    stance_frame_3x3[0] = stance_x_3x1[0];
//    stance_frame_3x3[3] = stance_x_3x1[1];
//    stance_frame_3x3[6] = stance_x_3x1[2];

//    stance_frame_3x3[1] = stance_y_3x1[0];
//    stance_frame_3x3[4] = stance_y_3x1[1];
//    stance_frame_3x3[7] = stance_y_3x1[2];

//    stance_frame_3x3[2] = stance_z_3x1[0];
//    stance_frame_3x3[5] = stance_z_3x1[1];
//    stance_frame_3x3[8] = stance_z_3x1[2];

//    stance_rol = -asin(stance_frame_3x3[6]-stance_frame_3x3[7]);
//    DC2QT(stance_frame_3x3, temp2_4x1);
//    temp2_4x1[1] *= -1;
//    temp2_4x1[2] *= -1;
//    temp2_4x1[3] *= -1;
//    QTcross(temp2_4x1,des_qRF_4x1,temp3_4x1);
//    QTcross(temp2_4x1,des_qLF_4x1,temp4_4x1);
//    stance_pit = 2*(atan2(temp3_4x1[2],temp3_4x1[0])+atan2(temp4_4x1[2],temp4_4x1[0]))/2.;

//    if(fabs(stance_rol) > STANCE_ROLL_MAX || fabs(stance_pit) > STANCE_PITCH_MAX)
//    {
//        memcpy(des_pCOM_3x1, pCOM_v_3x1, 3*sizeof(double));
//        memcpy(des_pRF_3x1, pRF_v_3x1, 3*sizeof(double));
//        memcpy(des_pLF_3x1, pLF_v_3x1, 3*sizeof(double));
//        memcpy(des_qRF_4x1, qRF_v_4x1, 4*sizeof(double));
//        memcpy(des_qLF_4x1, qLF_v_4x1, 4*sizeof(double));
//        return -1;
//    }

//    des_pCOM_3x1[0] = stance_center_3x1[0];
//    des_pCOM_3x1[1] = stance_center_3x1[1];
//    des_pCOM_3x1[2] = pCOM_v_3x1[2];

//    return 0;
//}
