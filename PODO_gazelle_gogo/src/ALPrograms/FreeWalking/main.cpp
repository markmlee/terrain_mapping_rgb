/*
 *	This program is generated from drc-podoal-template.
 *
 *	This AL-Process will activated by "PODO_Daemon" with appropriate "AL-Number".
 *	The AL-Number is determined at Core_config.db file at Share folder.
 *	And the AL-Process needs this AL-Number as an input parameter.
 *	So trying to open the AL-Process without any input parameter will terminate the process.
 *
 *	Please check your PODO_AL_NAME and Core_Config.db file.
 *	Actually the PODO_AL_NAME is used for recognizing the certain process and making it unique.
 *	So the same name of file with different path is allowed if the PODO_AL_NAMEs are different.
 *	But we recommend you that gather all of your build file in one folder (check the Core_Config.db file).
 *
 *	You can change the period of real-time thread by changing "RT_TIMER_PERIOD_MS" as another value in "rt_task_set_periodic".
 *	Please do not change "RT_TIMER_PERIOD_MS" value in "typedef.h".
 *	You can also change the priority of the thread by changing 4th parameter of "rt_task_create".
 *	In this function you need to care about the name of thread (the name should be unique).
 *
 *	Please do not change the "RBInitialize" function and fore code of main().
 *	You may express your idea in while loop of main & real-time thread.
 *
 *	Each AL-Process has its own command structure in Shared Memory.
 *	So, it can have its own command set.
 *	Make sure that the command set of each AL-process start from over than 100.
 *	Under the 100 is reserved for common command set.
 *
 *	Now, you can do everything what you want..!!
 *	If you have any question about PODO, feel free to contact us.
 *	Thank you.
 *
 *
 *
 *	Jungho Lee		: jungho77@rainbow.re.kr
 *	Jeongsoo Lim	: yjs0497@kaist.ac.kr
 *	Okkee Sim		: sim2040@kaist.ac.kr
 *
 *	Copy Right 2014 @ Rainbow Co., HuboLab of KAIST
 *
 */

#include <QCoreApplication>
#include <alchemy/task.h>


#include <iostream>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <QDataStream>
#include <QString>
#include <QFile>
#include "WalkingFSM.h"


#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/kine_drc_hubo4.h"
#include "joint.h"
#include "orient_fnc.h"
#include "ManualCAN.h"

#define PODO_AL_NAME       "FREEWALK"

extern double fsmDEL_T;
using namespace std;

double temp_debug[3] = {0,};
double Gyro_Ankle_FeedBack_ONOFF = 1.0;
double ZMP_FeedBack_ONOFF =1.;
double Ankle_Moment_FeedBack_ONOFF = 1.;

double Leg_Length_FeedBack_ONOFF = 1.;
double EarlyLanding_ONOFF =1.;
double LateLanding_ONOFF =1.;
double Sagging_Comp_ONOFF = 1.;

double impONOFF = 0.0;
double torONOFF = 0.0;
double zmpONOFF = 0.0;


bool Pelvis_Orientation_FeedBack_ONOFF = false;
bool TorsoAngle_Comp_ONOFF = true;
int InitializeFLAG =0;

const double max_lengthY = 0.09;
const double max_lengthX = 0.17;
const double max_angleZ = 25;

const double complete_time = 2.0;

const int SW_MODE_COMPLEMENTARY = 0x00;
const int SW_MODE_NON_COMPLEMENTARY = 0x01;
const double COM_Offset = 0;//-0.05;


double InitialComz =0;

const double L_FOOT = 0.24;
double L_PEL2PEL = 0.26;
double L_PEL2PEL_OFFSET = 0.04;

int Mass_ONOFF_Flag = 0;
double DSPHoldTime = 0.0;

DSPTask initialPos;

const int CAN0 = 0;
const int CAN1 = 1;
const int CAN2 = 2;
const int CAN3 = 3;

const int JMC0 = 0;
const int JMC1 = 1;
const int JMC2 = 2;
const int JMC3 = 3;
const int JMC4 = 4;
const int JMC5 = 5;
const int JMC6 = 6;
const int JMC7 = 7;
const int JMC8 = 8;
const int JMC9 = 9;
const int JMC10 = 10;
const int JMC11 = 11;
const int JMC12 = 12;
const int JMC13 = 13;
const int JMC14 = 14;
const int JMC15 = 15;
const int JMC16 = 16;
const int JMC17 = 17;
const int JMC18 = 18;
const int JMC19 = 19;



int RF_FLAG = 0;
int LF_FLAG = 0;
int SSP_FLAG = 0;
// Functions ===========================================
// Signal handler
void CatchSignals(int _signal);
int HasAnyOwnership();

// Real-time thread for control
void RBTaskThread(void *);
void RBFlagThread(void *);

// Initialization
int RBInitialize(void);
// =====================================================
int RBindicator(unsigned char number);//0 off n n*100ms on/off -1 on 0x85 0,-1 on off

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM       userData;

// RT task handler for control
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

// Program variable
int isTerminated;
int PODO_NO;
JointControlClass *joint;
// =====================================================

// Command Set =========================================
enum FREEWALKCOMMAND

{
    FREEWALK_NO_ACT = 100,
    FREEWALK_CUR_DSP,
    FREEWALK_WALK,
    FREEWALK_SAVE,
    FREEWALK_CONTROL_TEST,
    FREEWALK_PRE_WALK,
    FREEWALK_MOTION_CHECK,
    FREEWALK_INITIALIZE,
    FREEWALK_TERRAIN,
    FREEWALK_ADDMASS,
    FREEWALK_WIDE,
    FREEWALK_DSP_HOLD_WALK,
    FREEWALK_STOP
};

enum FALL_DIR
{
    FALL_PLUS =0,
    FALL_MINUS
};

// =====================================================

double CONT_X,CONT_Y,CONT_X_n,CONT_Y_n;

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
enum{
    RF_ANKLE_LOCK = 0,
    RF_ANKLE_FREE
};
enum{
    LF_ANKLE_LOCK = 0,
    LF_ANKLE_FREE
};
enum{
    NOLANDING = 0,
    RSSP,
    LSSP,
    DSP,
    FINAL,
    END
};
enum{
    NOPE =0,
    OKIE
};

inline double currentRef(int jnum){
    return sharedSEN->ENCODER[joint->Joints[jnum]->GetId()][joint->Joints[jnum]->GetCh()].CurrentReference;
}

inline double currentPositIK_LowerBodyion(int jnum){
    return sharedSEN->ENCODER[joint->Joints[jnum]->GetId()][joint->Joints[jnum]->GetCh()].CurrentPosition;
}

inline double currentPosition(int jnum){
    return sharedSEN->ENCODER[joint->Joints[jnum]->GetId()][joint->Joints[jnum]->GetCh()].CurrentPosition;
}


enum FTNAME{
    RAFT = 0,
    LAFT
};
enum IMUNAME{
    CIMU = 0
};
typedef enum{
    DIR_FORWARD = 0,
    DIR_BACKWARD,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_CW,
    DIR_CCW
}FOOT_DIRECTION;
enum WalkingModeCommand{
    FORWARD_WALKING =0,
    BACKWARD_WALKING,
    RIGHTSIDE_WALKING,
    LEFTSIDE_WALKING,
    CWROT_WALKING,
    CCWROT_WALKING,
    GOTOWR_WALKING
};
enum WalkingStopModeCommand{
    COMPLETE_STOP_WALKING = 0,
    SPREAD_STOP_WALKING
};
enum WalkingGoalGoModeCommand
{
    GOALGO_NORMAL = 0,
    GOALGO_FOR_SIDE,
    GOALGO_SIDE_FOR,
    GOALGO_DIAGONAL,
};
enum TerrainMode
{
    GUI_TERRAIN =0,
    FIELD_TERRAIN,
    OCS_TERRAIN
};


const double DEL_T = 0.005;
const double    OFFSET_ELB = -20.0;
const double    OFFSET_RSR = -15.0;
const double    OFFSET_LSR = 15.0;

#define ROW_data_debug 20000
#define COL_data_debug 100

// Save & Load the DSPSchedule Data
class SaveDSPScheduleData{
public:
    SaveDSPScheduleData()	{}
    double		left[3];
    double		right[3];
    double		lyaw;
    double		lroll;
    double		lpitch;
    double		ryaw;
    double		rroll;
    double		rpitch;
    double      htime;
    double      COMz[3];
};
QDataStream &operator<<(QDataStream &out, const QVector<SaveDSPScheduleData> &vec){
    out << vec.size();
    for(int i=0; i<vec.size(); i++){
        out << vec[i].left[0] << vec[i].left[1] << vec[i].left[2] << vec[i].lyaw << vec[i].lroll << vec[i].lpitch
            << vec[i].right[0] << vec[i].right[1] << vec[i].right[2] << vec[i].ryaw << vec[i].rroll << vec[i].rpitch<< vec[i].htime << vec[i].COMz[0] << vec[i].COMz[1];
    }
    return out;
}
QDataStream &operator>>(QDataStream &in, QVector<SaveDSPScheduleData> &vec){
    int size;
    in >> size;
    for(int i=0; i<size; i++){
        SaveDSPScheduleData sdsp;
        in >> sdsp.left[0] >> sdsp.left[1] >> sdsp.left[2] >> sdsp.lyaw >> sdsp.lroll >> sdsp.lpitch
                >>sdsp.right[0] >> sdsp.right[1] >> sdsp.right[2] >> sdsp.ryaw >> sdsp.rroll >> sdsp.rpitch>> sdsp.htime >> sdsp.COMz[0] >> sdsp.COMz[1];
        vec.push_back(sdsp);
    }
    return in;
}


//Ckine_drc_hubo4 kine_drc_hubo4;
KINE_DRC_HUBO4 kine_drc_hubo4;
//kine_drc_hubo43 kine_drc_hubo4;
IK_RETURN_CODE ik_rtn;
//Ckine_drc_hubo42 kine_drc_hubo42;
double WBIK_Q[34] = {0.,};
double WBIK_Q2[34] = {0.,};
double WBIK_Q0[34] = {0.,};
double FWRefAngleCurrent[NO_OF_JOINTS] = {0.,};
double FWRefAngleCurrent2[NO_OF_JOINTS] = {0.,};
double FWRefAngleCurrent_last[NO_OF_JOINTS] = {0.,};

//WB JW Control
double BTW_YAW,BTW_ROLL,BTW_PITCH;
double BTW_FOOT_Angle=0,BTW_FOOT_Angle_roll=0,BTW_FOOT_Angle_pitch =0,BTW_FOOT_Angle_yaw=0,BTW_FOOT_qPEL_comp_4x1[4]={1,0,0,0};
double BTW_PEL_angle_roll = 0,BTW_PEL_angle_pitch = 0,BTW_PEL_angle_yaw =0;
double BTW_PEL_angle_roll_vel = 0,BTW_PEL_angle_pitch_vel = 0,BTW_PEL_angle_yaw_vel =0;
double den_a1,den_a2,den_a3,den_a4,den_a5,num_b1,num_b2,num_b3,num_b4,num_b5;
double den_a11,den_a21,den_a31,den_a41,den_a51,num_b11,num_b21,num_b31,num_b41,num_b51;
double u_i_4 = 0,u_i_3 = 0, u_i_2 = 0, u_i_1 = 0, u_i = 0;
double y_i_4 = 0,y_i_3 = 0, y_i_2 = 0, y_i_1 = 0, y_i = 0;

double u_i_41 = 0, u_i_31 = 0, u_i_21 = 0, u_i_11 = 0, u_i1 = 0;
double y_i_41 = 0, y_i_31 = 0, y_i_21 = 0, y_i_11 = 0, y_i1 = 0;

double Foot_angle =0;

void WBIK();
void WBIK_Motion_check();
int WB_INIT_FLAG=0;
double Qub[34]={0.,},des_pCOM_3x1[3]={0,},des_pCOM_3x1_hat[3]={0,},des_pCOM_3x1_LPF[3]={0,}, des_pPCz, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3], des_pRF_3x1_hat[3], des_qRF_4x1[4]={1,0,0,0}, des_qRF_4x1_hat[4]={1,0,0,0}, des_pLF_3x1[3],des_pLF_3x1_hat[3], des_qLF_4x1[4]={1,0,0,0}, des_qLF_4x1_hat[4]={1,0,0,0};
double pCOM_3x1[3], pPCz, qPEL_4x1[4], pRF_3x1[3], qRF_4x1[4], pLF_3x1[3], qLF_4x1[4];
double _last_pCOM_3x1[3] = {0,};
//JW
double AnkleControl1=0,AnkleControl2=0;
int duty,duty2 = 0;
int LandingState = FINAL;
int Pre_LandingState =-1;

int CNT_AfterFree_LF =0, CNT_AfterFree_RF =0;
double temp_Add1[3]={0.,},temp_Add2[3]={0.,},temp_Add3[3]={0.,};

double FOGRollVel_LPF=0,FOGPitchVel_LPF=0;
double FOGRollVel_NF=0,FOGPitchVel_NF=0;
double FOGRollVel_NF2=0,FOGPitchVel_NF2=0;
double AngleRoll =0.0 ,AngleVel =0.0 ,AngleRoll_last =0.0;
double LPF_AngleRoll =0.0 ,LPF_AngleVel =0.0 ,LPF_AngleRoll_last =0.0;
double AnglePitch =0.0 , AnglePitch_DSP =0.0 ;
double RF_FZ_LPF = 0,LF_FZ_LPF = 0;
double RF_MX_LPF = 0,LF_MX_LPF = 0;
double RF_MY_LPF = 0,LF_MY_LPF = 0;


double Ground_LF = 0,Ground_RF = 0;
int StepNumber = 0;
int EarlyLandingFlag[2]={0,};
int DSP_LandingFlag[2]={0,};
double Late_Landing_Comp[2]={0.,},Late_Landing_Comp_last[2]={0.,};
int StepShapeFlag[2]={0,};
int Recover_EarlyLandingFlag[2]={0,};


double EarlyLandingTime =0;
int sJW_Data_Debug_Index;
int sJW_Data_Debug_Index2;

//Kirk variables
double  X_ZMP_Local,Y_ZMP_Local,X_ZMP_Global,Y_ZMP_Global,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_REF_Global,Y_ZMP_REF_Global;
double  X_ZMP_IMU = 0., Y_ZMP_IMU = 0.;
double  X_ZMP_IMU_n = 0., Y_ZMP_IMU_n = 0.;
double  X_ZMP = 0., Y_ZMP = 0., X_ZMP_LF = 0., Y_ZMP_LF = 0., Old_X_ZMP_LF = 0., Old_Y_ZMP_LF = 0.;
double  X_ZMP_n = 0., Y_ZMP_n = 0.;
double  X_ZMP_LPF = 0., Y_ZMP_LPF = 0.;
double  final_gain_DSP_ZMP_CON = 0., final_gain_SSP_ZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON = 0., Del_PC_Y_DSP_YZMP_CON = 0., Old_Del_PC_X_DSP_XZMP_CON = 0., Old_Del_PC_Y_DSP_YZMP_CON = 0.,
        Del_PC_X_SSP_XZMP_CON = 0., Del_PC_Y_SSP_YZMP_CON = 0., Old_Del_PC_X_SSP_XZMP_CON = 0., Old_Del_PC_Y_SSP_YZMP_CON = 0.;
double  LPF_Del_PC_X_DSP_XZMP_CON = 0., LPF_Del_PC_Y_DSP_YZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON_n = 0.,Del_PC_Y_DSP_YZMP_CON_n = 0.;
double  LPF_Del_PC_X_SSP_XZMP_CON = 0., LPF_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Old_Del_PC_X_DSP_XZMP_CON2 = 0;
double  Old_Del_PC_Y_DSP_YZMP_CON2 = 0;
double  Old_Del_PC_X_SSP_XZMP_CON_2 = 0., Old_Del_PC_Y_SSP_YZMP_CON_2 = 0.;
unsigned int CNT_final_gain_DSP_ZMP_CON = 0,  CNT_final_gain_SSP_ZMP_CON = 0;
unsigned int CNT_SSP_ZMP_CON = 0;


double GLOBAL_ZMP_REF_Y=0.0f,GLOBAL_ZMP_REF_X=0.0f,GLOBAL_ZMP_REF_X_local=0.0f;
double GLOBAL_ZMP_REF_Y_n=0.0f,GLOBAL_ZMP_REF_X_n=0.0f;
double GLOBAL_Y_LIPM = 0.0f,GLOBAL_Y_LIPM_d = 0.0f;
double GLOBAL_Z_LIPM = 0.0f,GLOBAL_Z_LIPM2 = 0.0f,GLOBAL_Z_LF = 0.0f,GLOBAL_Z_RF = 0.0f,GLOBAL_Z_LIPM_last=0.0f;
double GLOBAL_Z_LF_last = 0.0f,GLOBAL_Z_RF_last=0.0f;
double GLOBAL_Z_LF_last2 = 0.0f,GLOBAL_Z_RF_last2=0.0f;

double GLOBAL_Xori_LF = 0.,GLOBAL_Yori_LF = 0.,GLOBAL_Zori_LF = 0.;
double GLOBAL_Xori_LF_n = 0.,GLOBAL_Yori_LF_n = 0.,GLOBAL_Zori_LF_n = 0.;
double sum_GLOBAL_Xori_LF_n = 0.,sum_GLOBAL_Yori_LF_n = 0.;
double ave_GLOBAL_Xori_LF_n = 0.,ave_GLOBAL_Yori_LF_n = 0.;
double GLOBAL_Xori_LF2 = 0.,GLOBAL_Yori_LF2 = 0.,GLOBAL_Zori_LF2 = 0.;
double GLOBAL_Xori_LF_last = 0.,GLOBAL_Yori_LF_last = 0.,GLOBAL_Zori_LF_last = 0.;
double GLOBAL_Xori_LF_last2 = 0.,GLOBAL_Yori_LF_last2 = 0.,GLOBAL_Zori_LF_last2 = 0.;
double GLOBAL_Xori_LF2_last = 0.,GLOBAL_Yori_LF2_last = 0.,GLOBAL_Zori_LF2_last = 0.;
double GLOBAL_Xori_RF = 0.,GLOBAL_Yori_RF = 0.,GLOBAL_Zori_RF = 0.;
double GLOBAL_Xori_RF_n = 0.,GLOBAL_Yori_RF_n = 0.,GLOBAL_Zori_RF_n = 0.;
double sum_GLOBAL_Xori_RF_n = 0.,sum_GLOBAL_Yori_RF_n = 0.;
double ave_GLOBAL_Xori_RF_n = 0.,ave_GLOBAL_Yori_RF_n = 0.;
double GLOBAL_Xori_RF2 = 0.,GLOBAL_Yori_RF2 = 0.,GLOBAL_Zori_RF2 = 0.;
double GLOBAL_Xori_RF_last = 0.,GLOBAL_Yori_RF_last = 0.,GLOBAL_Zori_RF_last = 0.;
double GLOBAL_Xori_RF_last2 = 0.,GLOBAL_Yori_RF_last2 = 0.,GLOBAL_Zori_RF_last2 = 0.;
double GLOBAL_Xori_RF2_last = 0.,GLOBAL_Yori_RF2_last = 0.,GLOBAL_Zori_RF2_last = 0.;

double LPF_GLOBAL_Xori_RF = 0.,LPF_GLOBAL_Xori_LF = 0.;
double LPF_GLOBAL_Yori_RF = 0.,LPF_GLOBAL_Yori_LF = 0.;

double LPF_GLOBAL_Xori_RF2 = 0.,LPF_GLOBAL_Xori_LF2 = 0.;
double LPF_GLOBAL_Yori_RF2 = 0.,LPF_GLOBAL_Yori_LF2 = 0.;

double GLOBAL_Z_LF_last_earlylanding = 0.0f,GLOBAL_Z_RF_last_earlylanding=0.0f;
double GLOBAL_X_LIPM = 0.0f,GLOBAL_X_LF = 0.0f,GLOBAL_X_RF = 0.0f,GLOBAL_X_LIPM_d = 0.0f;
double GLOBAL_X_LIPM_n = 0.0f,GLOBAL_X_LF_n = 0.0f,GLOBAL_X_RF_n = 0.0f,GLOBAL_X_LIPM_d_n = 0.0f;
double GLOBAL_Y_LIPM_n = 0.0f,GLOBAL_Y_LF_n = 0.0f,GLOBAL_Y_RF_n = 0.0f,GLOBAL_Y_LIPM_d_n = 0.0f;
double GLOBAL_X_LIPM_last =0.0f,GLOBAL_ZMP_REF_X_last=0.0f;
double GLOBAL_X_LF_last = 0.0f,GLOBAL_X_RF_last = 0.0f;
double GLOBAL_Y_LF = 0.0f,GLOBAL_Y_RF = 0.0f;

double Estimated_GLOBAL_X_LF = 0.0f,Estimated_GLOBAL_X_RF = 0.0f;
double Estimated_GLOBAL_Y_LF = 0.0f,Estimated_GLOBAL_Y_RF = 0.0f;
double Estimated_GLOBAL_Z_LF = 0.0f,Estimated_GLOBAL_Z_RF = 0.0f;
double Estimated_GLOBAL_Z_LF_last = 0.0f,Estimated_GLOBAL_Z_RF_last = 0.0f;


double I_ZMP_CON_X=0.f,I_ZMP_CON_Y=0.f;
double I_ZMP_CON_X_last=0.f,I_ZMP_CON_Y_last=0.f;
double Old_I_ZMP_CON_X=0.f,Old_I_ZMP_CON_Y=0.f;
//JW_InvPattern
double Y_inv=0.,Y_inv_d=0.,Y_inv_old=0.,theta_ref=0.,theta_dd=0.,theta_d=0.,theta=0.;
double Y_inv2=0.,Y_inv_d2=0.,Y_inv_old2=0.,theta2_ref=0.,theta2_dd=0.,theta2_d=0.,theta2=0.;
double U[2]={0.f,0.f},U_I[2]={0.f,0.f};

double JW_InvPattern_l,JW_InvPattern_l2;
double JW_InvPattern_Klqr[2]={0.f,0.f};
double JW_InvPattern_U[2]  = {0.f,0.f};
double JW_InvPattern_U_n[2]  = {0.f,0.f};
double JW_InvPattern_U_I[2]  = {0.f,0.f};
double JW_InvPattern_A[2][2] ={{0.f,0.f},{0.f,0.f}};
double JW_InvPattern_A_X[2][2] ={{0.f,0.f},{0.f,0.f}};
double JW_InvPattern_B[2] = {0.f,0.f};
double JW_InvPattern_B_X[2] = {0.f,0.f};

double JW_InvPattern_k=5550.f;//5160.f;//5550.f;
double JW_InvPattern_k_X=9000.f;//5550.f;
double JW_InvPattern_c=60.f;
double JW_InvPattern_c_X=80.f;
double JW_InvPattern_m=80.f;//69.f;//62.f;//68.f;//

double JW_InvPattern_X_d[2] ={0.f,0.f};
double JW_InvPattern_X[2] ={0.f,0.f};
double JW_InvPattern_X_old[2] ={0.f,0.f};

double JW_InvPattern_Y_d[2] ={0.f,0.f};
double JW_InvPattern_Y[2] ={0.f,0.f};
double JW_InvPattern_Y_old[2] ={0.f,0.f};

double U_Gain  = 0.;
double U0_Gain  = 1.;
double U3_Gain = 0.;
double U_Gain_DSP = 1.;
double U0_Gain_KI  = 0.,U0_Gain_KI_last = 0.;
double U0_Gain_Goal_KI  = 0.,U0_Gain_Goal_KI_last = 0.;

double Foot_gainLF=0,Foot_gainRF=0;

// Debugging data
double   JW_Data_Debug[COL_data_debug][ROW_data_debug];
double   JW_Data_Debug2[COL_data_debug][ROW_data_debug];
FILE *fp;
FILE *fp2;
FILE *fp3;
FILE *fp4;
// --------------------------------------------------------------------------------------------- //


// Preview Variables
int PV_LENGTH =2000;

int pv_Index=0;
double pv_Gd[301],pv_Gx[4],pv_Gi[2];
double pv_A[3][3],pv_B[3][1],pv_C[1][3];

int    pv_time_Index;

double fsm_state_timer[9] = {0.,};
//


WalkingFSM  *checkfsm;
WalkingFSM  *fsm;

bool    fsmFlag = false;
bool    Fall_Flag = false;
int    Fall_cnt = 0;
int    Fall_Roll_DIR,Fall_Pitch_DIR;
bool    Fall_once_Flag = false;
bool _motion_check = false;
bool _collision_check = false;
bool _motion_check_start = false;
bool _generating_motion_flag = false;
bool _SSP_singularFlag = false;
bool _DSP_singularFlag = false;
bool _transition_flag = false;
bool _INIT_singularFlag = false;
bool _AngleLimit_flag = false;
bool _LengthLimit_flag = false;
int _motion_section_num = 0;
int _last_motion_cnt = 0;
int _SSP_left_singular_num = 0;
int _SSP_right_singular_num = 0;
int _INIT_singular_num = 0;
double _pertubation[3] = {0.0f,};
int _generating_cnt = 0;
int _generating_loop_limint = 12;




//Ankle Free and Lock
double Dif_enc_RAP = 0.,Dif_enc_RAR = 0.,Dif_enc_LAP = 0.,Dif_enc_LAR= 0.;
double Dif_enc_RAP_last = 0.,Dif_enc_RAR_last = 0.,Dif_enc_LAP_last = 0.,Dif_enc_LAR_last= 0.;

//ladder
double Add_Joint[NO_OF_JOINTS]={0.,};
double Add_Joint_last[NO_OF_JOINTS]={0.,};
double Add_FootTask[2][3]={{0.,},};
double INSIDE_FLAG_Add_FootTask[2][3]={{0.,},},OUTSIDE_FLAG_Add_FootTask[2][3]={{0.,},};

double Add_FootOri[2][3]={{0.,},};
double Add_FootOri_last[2][3]={{0.,},};
double Add_FootTask_last[2][3]={{0.,},};
double Add_FootTask2[2][3]={{0.,},};

// Kirk Controllers
void Kirk_Control();
void Kirk_Control_ssp();
void get_zmp();
void get_zmp2();
void ZMP_intergral_control();

void kirkReadZMPBP(double ORF_x, double ORF_y, double OLF_x, double OLF_y);
void P1_Y_th_th_d_Observer(double u, double ZMP, int zero);
// DSP ZMP Controller
double  kirkZMPCon_XP2(double u, double ZMP, int zero);
double  kirkZMPCon_YP2(double u, double ZMP, int zero);
// SSP ZMP Controller
double  kirkZMPCon_XP1(double u, double ZMP, int zero);
double  kirkZMPCon_YP1(double u, double ZMP, int zero);
//Controller Global gain
double G_DSP_X = 1.,G_DSP_Y=1.,G_DSP_X_last = 1.,G_DSP_Y_last=1.;
//Inverse Model Pattern Generator
void JW_INV_MODEL(double Pattern1,double Pattern1_d,double Pattern2,double Pattern2_d);
//Preview Control Pattern Generator


//forcecontrol
double RDF,LDF,Zctrl=0.0;
double FootForceControl(double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0);
double RightDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y);
double LeftDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y);
void ReactiveControl(int state,int state2,int state3);

double ZMPControllerX(double desX,double desY,double desXdot,double desYdot,double desPx,double desPy,double x,double x_dot,double px,double y,double y_dot,double py);
double ZMPControllerY(double desX,double desY,double desXdot,double desYdot,double desPx,double desPy,double x,double x_dot,double px,double y,double y_dot,double py);
double Kaj_y = 0.0,Kaj_x = 0.0;
double est_x=0.0,est_x_dot=0.0;
double est_y=0.0,est_y_dot=0.0;
void mat3by3x3by1(double a[3][3],double b[3],double out[3]);

void Preview();
void Preliminary();
void Preliminary_Motion_check();
void ThreeDim_Preview();
void ThreeDim_Preview_Motion_check();
void Rodrigues(double joint_axis[3],double theta,double R[3][3]);
void RL_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3]);
void LL_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3]);
void RL_Foot_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3]);
void LL_Foot_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3]);
double _pelvis_position[3];
void Leg_Collision_Check(double Foot_pos[2][3],double obj_pos[2][3],double dis[1]);
double PELVIS_pos[3],RKN_pos[3],RAP_pos[3],RAR_pos[3],RHY_pos[3],RHP_pos[3],RHR_pos[3],FOOT_pos[3];
double PELVIS_pos2[3],RKN_pos2[3],RAP_pos2[3],RAR_pos2[3],RHY_pos2[3],RHP_pos2[3],RHR_pos2[3],FOOT_pos2[3],_LF_edge[4][3],_RF_edge[4][3];
double _legpos[4][3]={0,},_shin_dist[2],_foot_dist[4];
double _floor_center[9][3] = {{0,},},_floor_edge[9][12] = {{0,},},_floor_normal[9][3] = {{0,},};
int _floor_num = 0,_last_floor_num = 0;
double _last_floor_edge[2][12]={{0,},};


double AngularMomentumComp(double velx,double torso_yaw,int sign, int reset);
double Yaw_Gain = 2.15f,Yaw_Gain2 = 0.0f;
double Yaw_min = -41.0f,Yaw_max = 41.0f;
double yaw_angle = 0,yaw_angle_last = 0;
//double  yaw_theta = 0;


void WalkingTimer();
void GetGain(double H);
void WBIK_PARA_CHANGE();
void Print_WBIK_Infos();

//JW Functions
void Poly_5th(double _time,double _ti,double _tf,double _pi,double _vi,double _ai,double _pf,double *_info_y);
void Matrix2Cross3x1_3x1(double A[3], double B[3], double *C);
void JW_INIT_WB(void);
void JW_save();
void JW_save2();
void JW_save_Motion_check();
void JW_save2_Motion_check();
void ANKLE_FREE_LOCK();
void Get_RF_ENC_Diff();
void Get_LF_ENC_Diff();
void Free_RF_ANK();
void Free_LF_ANK();
void Lock_RF_ANK();
void Lock_LF_ANK();
void GlobalSlope();
double Global_qFoot_4x1[4]={1,0,0,0};
//Init Functions
double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero);
double HUBO2ZMPInitLegLength(double _ref, double _force, int _zero);
double HUBO2ZMPInitLegLength2(double _ref, double _force, int _zero);

double HUBO2_ForceInitCon_RF_X(double _ref, double _force, int _zero);
double HUBO2_ForceInitCon_RF_Y(double _ref, double _force, int _zero);
double HUBO2_ForceInitCon_LF_X(double _ref, double _force, int _zero);
double HUBO2_ForceInitCon_LF_Y(double _ref, double _force, int _zero);

double HUBO2TorqInitConRAR(double _ref, double _torque, int _zero, double _gain);
double HUBO2TorqInitConLAR(double _ref, double _torque, int _zero, double _gain);
double HUBO2TorqInitConRAP(double _ref, double _torque, int _zero, double _gain);
double HUBO2TorqInitConLAP(double _ref, double _torque, int _zero, double _gain);

double WBIKTorqInitConRAR(double _ref, double _torque, int _zero, double _gain);
double WBIKTorqInitConLAR(double _ref, double _torque, int _zero, double _gain);
double WBIKTorqInitConRAP(double _ref, double _torque, int _zero, double _gain);
double WBIKTorqInitConLAP(double _ref, double _torque, int _zero, double _gain);

double HUBO2TorsoInitConPitch(double _ref, double _angle, int _zero);
double HUBO2TorsoInitConRoll(double _ref, double _angle, int _zero);

double X_ZMP_integral_in_static(int _zero);
double Y_ZMP_integral_in_static(int _zero);
double HUBO2_ForceInitCon[2][2] = {{0.0}, };
double AnkleRollAngle[2] = {0., 0.},AnkleRollAngle_last[2] = {0., 0.};
double AnklePitchAngle[2] = {0.,},AnklePitchAngle_last = 0.;
double TorsoPitchAngle = 0.,TorsoPitchAngle_n=0,TorsoPitchAngle_last = 0.;
double TorsoRollAngle = 0.,TorsoRollAngle_n=0;
double Pel_Yaw=0;

double WBIK_Torq[3][3] ={{0.,},};
double qt_WBIK_Torq_RF[4] ={1.,0.,0.,0.},qt_WBIK_Torq_LF[4]={1.,0.,0.,0.};

double _LAST_FOOT_POS,_slope_z,_temp_com;
double _temp_debug_data[20]={0.0f,};
bool _preview_flag = false, _preliminary_flag = false;
bool _preview_flag_Motion = false, _preliminary_flag_Motion = false;
//---------------------------3D Preview
double pv_C_v[1][3];
double _virtual_zmp_error[2][300] = {{0.0f},},_virtual_state_old[2][3] = {{0.0f},};
int _preliminary_cnt = 0;
double _preliminary_state_I[2][3] = {{0.0f},}, _preliminary_state_II[2][3] = {{0.0f},}, _preliminary_state_298[2][3] = {{0.0f},};

double _virtual_zmp_error_Motion[2][300] = {{0.0f},},_virtual_state_old_Motion[2][3] = {{0.0f},};
int _preliminary_cnt_Motion = 0;
double _preliminary_state_I_Motion[2][3] = {{0.0f},}, _preliminary_state_II_Motion[2][3] = {{0.0f},}, _preliminary_state_298_Motion[2][3] = {{0.0f},};
void mat3by3x3by3(double a[3][3],double b[3][3],double out[3][3]);
void mat3by3plus3by3(double a[3][3],double b[3][3],double out[3][3]);

double zmp_offset[2]={0,},Gain_offset[2000]={0,};
double FK_pCOM_3x1[3]={0,},FK_pRFoot_3x1[3]={0,},FK_qRFoot_4x1[4]={1,0,0,0},FK_pLFoot_3x1[3]={0,},FK_qLFoot_4x1[4]={1,0,0,0};
double init_WBIK_pCOM[3] = {0,},init_WBIK_Q[3] = {0,};
void Walking_initialize();
void Walking_initialize_1st();
void fsm_clear();
void checkfsm_clear();
void get_WBIK_Q_from_RefAngleCurrent();

int FootSelector(int direction);
void getCurrentPos();
void planningFootPrint(int _walkingmode,int _walkingstopmode,int _walking_num, double _steplength, double _rotangle );
int planningFootPrint_Goal(double _GoalPosX, double _GoalPosY, double _GoalAngle , int _OneMoreMode,int _firstfoot);
int planningFootPrint_Goal_Rot_For_Rot(double _GoalPosX, double _GoalPosY, double _GoalAngle  , int _OneMoreMode,int _firstfoot);
int forward_foot_print(int _walkingstopmode,int _walking_num,double _steplength, int _firstfoot);
int backward_foot_print(int _walkingstopmode,int _walking_num,double _steplength, int _firstfoot);
int rightside_foot_print(int _walkingstopmode,int _walking_num,double _steplength,int _firstfoot);
int leftside_foot_print(int _walkingstopmode,int _walking_num,double _steplength,int _firstfoot);
int cwrot_foot_print(int _walkingstopmode,int _walking_num,double _rotangle);
int ccwrot_foot_print(int _walkingstopmode,int _walking_num,double _rotangle);


void MotionCheck();
unsigned int RBsetFrictionParameter(unsigned int _canch, unsigned int _bno, int _vel_saturation1, int _amp_compen1, int _vel_saturation2, int _amp_compen2);
void Upperbody_Gain_Override();
void Upperbody_Gain_Lock();
bool upperbody_lock_flag = false;
int ZeroGainRightArm();
int ZeroGainLeftArm();
int ZeroGainRightArm2();
int ZeroGainLeftArm2();
int ZeroGainRightLeg();
int ZeroGainLeftLeg();
void FullGainRightLeg();
void FullGainLeftLeg();
double NotchFilter_GyroRollControlInput(double _input, int _reset);
double NotchFilter_GyroPitchControlInput(double _input, int _reset);
double NotchFilter_GyroRollVel(double _input, int _reset);
double NotchFilter_GyroPitchVel(double _input, int _reset);
//Falling
void GotoFallPos();

//*swan adding
double hat_X[3];
double SwanObserver(double _u, double _ZMP, double *_hat_X);
double SwanZMPCon_X(double _u, double _ZMP, int zero);   // u [mm], ZMP [mm]
double SwanZMPCon_Y(double _u, double _ZMP, int zero);   // u [mm], ZMP [mm]

void Global2Local(double _Global[],double _Local[]);
void Global2Local2(double _Global[],double _Local[]);
void Local2Global(double _Local[],double _Global[]);


double odom_x = 0;
double odom_y = 0;
double odom_theta = 0;
double odom_x_prev = 0;
double odom_y_prev = 0;
double odom_theta_prev = 0;

int stopFlag = false ;

int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "g:p:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "AL for Gazebo";
    else                FILE_LOG(logWARNING) << "AL for Robot";
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}


int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }


    fsm = new WalkingFSM();
    checkfsm = new WalkingFSM();


    fsm_clear();
    checkfsm_clear();
//    fsm->DEL_T = 0.005;
//    fsm->DEL_T = 0.005;
    // Initialize RBCore -----------------------------------
    if(RBInitialize() == -1)
        isTerminated = -1;

    //Initialize wbik
    //kine_drc_hubo4.get_Q0(WBIK_Q);
    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = -5.*D2R;
    WBIK_Q0[idLSP] = -5.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
    WBIK_PARA_CHANGE();


    // Important!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    cout << "IK Version: " << kine_drc_hubo4.get_version() << endl;

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo4.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo4.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo4.L_PEL2PEL/2.;//0.13;//kine_drc_hubo4.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    get_WBIK_Q_from_RefAngleCurrent();

    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    Walking_initialize_1st();

    userData->G2M.StepTime = 1.0f;

    RBindicator(4);
    QString fileName;
    // User command cheking --------------------------------
    while(isTerminated == 0){

        usleep(100*1000);
        sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] = INSIDE_WALKING;
        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
        case FREEWALK_STOP:
            stopFlag = true;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_ADDMASS:
            Mass_ONOFF_Flag = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_WALK:
        if(fsmFlag == false)
        {
            ////sharedData->STATE_COMMAND = TCMD_TERRAIN_WALKING_START;
            Upperbody_Gain_Override();
            DSPHoldTime = 0.;
            L_PEL2PEL = 0.22;
            L_PEL2PEL_OFFSET =0.001;
            U0_Gain = 0.96;//1.06;
            cout << ">>> COMMAND: FREEWALK_WALK" << endl;
            userData->WalkDoneFlag = false;
            fsmDEL_T = 0.005;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
            fsm_clear();
            checkfsm_clear();

            //Mass_ONOFF_Flag = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            if(Mass_ONOFF_Flag == 1)
            {
                kine_drc_hubo4.m_RightWrist = 3.7+3.5;
                kine_drc_hubo4.m_LeftWrist = 3.7+3.5;

                Mass_ONOFF_Flag = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                printf("Mass ON !!!!!!!!!!!!!!!!!!!!!!!!!!!!m_RightWrist = %f,m_LeftWrist = %f %f\n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_Torso);
            }
            else if(Mass_ONOFF_Flag == -1)
            {
                kine_drc_hubo4.m_RightWrist = 3.7;
                kine_drc_hubo4.m_LeftWrist = 3.7;
//                kine_drc_hubo4.m_Torso = 28.6;//+1.0;
                Mass_ONOFF_Flag = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                printf("Mass OFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!m_RightWrist = %f,m_LeftWrist = %f \n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_RightWrist);
            }

            Walking_initialize();
            usleep(200*1000);

            fileName.sprintf("%s", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
            cout << fileName.toStdString().data() << endl;

            GetGain(userData->WalkReadyCOM[2] + COM_Offset);
            fsm->walking_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
            fsm->DSPScheduler.clear();
            if(fsm->walking_mode == NORMAL_WALKING){
                double StepTime = 0.9;//1.1;//userData->G2M.StepTime;
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == INSIDE_WALKING)
                {
                    fsm->TIME_DSP = StepTime*2./10.;//1.5f;
                    fsm->TIME_SSP = StepTime*8./10.;//3.0f;
                }
                else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
                {
                    fsm->TIME_DSP = 1.4;//1.5f;
                    fsm->TIME_SSP = 2.0;//3.0f;
                    fsm->walking_mode = TERRAIN_WALKING;
                    fsm->heel_pitching_mode = 0;
                    U0_Gain = 0.95;
                }
//                fsm->TIME_DSP = 0.5f*1.;  //dsp:0.28
//                fsm->TIME_SSP = 1.7f*1.;  //ssp:1.96
//                Pelvis_Orientation_FeedBack_ONOFF = 0.;
//                Leg_Length_FeedBack_ONOFF =0.;
                printf("NORMAL TASK speed set DSP: %f  SSP: %f \n",userData->terrain_variable[0],userData->terrain_variable[1]);

                if(fabs(userData->G2M.StepNum) > 30)userData->G2M.StepNum = 30;

                if(fabs(userData->G2M.StepLength) > 0.35)userData->G2M.StepLength = 0.35;

                if(fabs(userData->G2M.StepAngle) > 45) userData->G2M.StepAngle = 45;


                printf("num = %d, length = %f, angle = %f\n ",userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle);
                getCurrentPos();
                planningFootPrint(userData->G2M.WalkingModeCommand,userData->G2M.WalkingStopModeCommand,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle);

            }
            else if(fsm->walking_mode == GOAL_WALKING)
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == INSIDE_WALKING)
                {
                    fsm->TIME_DSP = 0.2f*1.;//1.5f;
                    fsm->TIME_SSP = 0.8f*0.9 + 0.1;//3.0f;

                }
                else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
                {
                    fsm->TIME_DSP = 2.2;//1.5f;
                    fsm->TIME_SSP = 1.5;//3.0f;
                    fsm->walking_mode = TERRAIN_WALKING;
                    U0_Gain = 0.95;
                }
                double GX = userData->G2M.GoalPosX;
                double GY = userData->G2M.GoalPosY;
                double GAngle = userData->G2M.GoalAngle;
                int Xnum,Ynum;
                double Xsteplength,Ysteplength;

                if(GAngle < -180) GAngle = GAngle + 360;
                if(GAngle > 180) GAngle = GAngle -360;

                printf("GoalX = %f, GoalY = %f, Goal angle = %f \n",GX,GY,GAngle);

                if(userData->G2M.WalkingGoalGoModeCommand == GOALGO_NORMAL)
                {
                    //if(      ((GY>3.*GX)&&(GY>-3.*GX))  ||   ((GY<3.*GX)&&(GY<-3.*GX))   ||   ((GY>GX/3.)&&(GY<-GX/3.))  ||   ((GY<GX/3.)&&(GY>-GX/3.))   ||((fabs(GY)<max_lengthY)&&(fabs(GX)<max_lengthX)))
                    {
                        printf("+++++++++++++++++++++++++++++++++++++GOALGO_NORMAL+++++++++++++++++++++++++++++++++++++\n");
                        printf("Combination Walking !!!!!\n");
                        getCurrentPos();
                        //if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                        int foot = 1;
                        if(GY>0)
                            foot = planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
                        else
                            foot = planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_LEFT);

                        planningFootPrint_Goal(GX,GY,GAngle,1,foot);
                    }
                }
                else if(userData->G2M.WalkingGoalGoModeCommand == GOALGO_FOR_SIDE)
                {
                    printf("+++++++++++++++++++++++++++++++++++++GOALGO_FOR_SIDE+++++++++++++++++++++++++++++++++++++\n");
                    printf("Combination Walking !!!!!\n");

                    Xnum = int(fabs(GX)/max_lengthX)+1;
                    Ynum = int(fabs(GY)/max_lengthY)+1;
                    Xsteplength = GX/double(Xnum);
                    Ysteplength = GY/double(Ynum);

                    printf("GX = %f,GY=%f,Xnum = %d,Ynum = %d,Xlength = %f,Ylength = %f \n",GX,GY,Xnum,Ynum,Xsteplength,Ysteplength);
                    getCurrentPos();
                    int foot =1;
                    if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                        foot = planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
//                    planningFootPrint_Goal(GX,GY,GAngle);

                    if(GX>0.02)
                    {
                        foot = forward_foot_print(COMPLETE_STOP_WALKING,Xnum,Xsteplength,foot*-1);
                    }
                    else if(GX<-0.02)
                    {
                        foot = backward_foot_print(COMPLETE_STOP_WALKING,Xnum,-Xsteplength,foot*-1);
                    }

                    if(GY>0.02)
                    {
                        foot = leftside_foot_print(COMPLETE_STOP_WALKING,Ynum,Ysteplength,foot*-1);
                    }
                    else if(GY<-0.02)
                    {
                        foot = rightside_foot_print(COMPLETE_STOP_WALKING,Ynum,-Ysteplength,foot*-1);
                    }
                    int _walking_num = int(fabs(GAngle)/max_angleZ)+1;
                    double _rotangle = GAngle/_walking_num;
                //    printf("\n foot = %d \n",foot);

                    DSPTask task;
                    int pos;
                    if(GAngle<0)
                    {
                        if(foot==FOOT_RIGHT)
                        {
                            pos = fsm->DSPScheduler.size()-1;
                            task.Left[0] = fsm->DSPScheduler[pos].Left[0]+0.001;
                            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                            task.LYaw = fsm->DSPScheduler[pos].RYaw;
                            task.RYaw = fsm->DSPScheduler[pos].RYaw;
                            task.HTime = DSPHoldTime;
                            fsm->DSPScheduler.push_back(task);
                            printf("onemore LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
                        }
                        foot = cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
                    }
                    else if(GAngle>0)
                    {
                        if(foot==FOOT_LEFT)
                        {
                            pos = fsm->DSPScheduler.size()-1;
                            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                            task.Right[0] = fsm->DSPScheduler[pos].Right[0]+0.001;
                            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                            task.LYaw = fsm->DSPScheduler[pos].RYaw;
                            task.RYaw = fsm->DSPScheduler[pos].RYaw;
                            task.HTime = DSPHoldTime;
                            fsm->DSPScheduler.push_back(task);
                            printf("onemore LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
                        }
                        foot = ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
                    }
                }
                else if(userData->G2M.WalkingGoalGoModeCommand == GOALGO_SIDE_FOR)
                {
                    printf("+++++++++++++++++++++++++++++++++++++GOALGO_SIDE_FOR+++++++++++++++++++++++++++++++++++++\n");
                    printf("Combination Walking !!!!!\n");

                    Xnum = int(fabs(GX)/max_lengthX)+1;
                    Ynum = int(fabs(GY)/max_lengthY)+1;
                    Xsteplength = GX/double(Xnum);
                    Ysteplength = GY/double(Ynum);

                    printf("GX = %f,GY=%f,Xnum = %d,Ynum = %d,Xlength = %f,Ylength = %f \n",GX,GY,Xnum,Ynum,Xsteplength,Ysteplength);
                    getCurrentPos();
                    int foot =1;
                    if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                        foot = planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
//                    planningFootPrint_Goal(GX,GY,GAngle);

                    if(GY>0.02)
                    {
                        foot = leftside_foot_print(COMPLETE_STOP_WALKING,Ynum,Ysteplength,foot*-1);
                    }
                    else if(GY<-0.02)
                    {
                        foot = rightside_foot_print(COMPLETE_STOP_WALKING,Ynum,-Ysteplength,foot*-1);
                    }
                    if(GX>0.02)
                    {
                        foot = forward_foot_print(COMPLETE_STOP_WALKING,Xnum,Xsteplength,foot*-1);
                    }
                    else if(GX<-0.02)
                    {
                        foot = backward_foot_print(COMPLETE_STOP_WALKING,Xnum,-Xsteplength,foot*-1);
                    }


                    int _walking_num = int(fabs(GAngle)/max_angleZ)+1;
                    double _rotangle = GAngle/_walking_num;
                //    printf("\n foot = %d \n",foot);

                    DSPTask task;
                    int pos;
                    if(GAngle<0)
                    {
                        if(foot==FOOT_RIGHT)
                        {
                            pos = fsm->DSPScheduler.size()-1;
                            task.Left[0] = fsm->DSPScheduler[pos].Left[0]+0.001;
                            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                            task.LYaw = fsm->DSPScheduler[pos].RYaw;
                            task.RYaw = fsm->DSPScheduler[pos].RYaw;
                            task.HTime = DSPHoldTime;
                            fsm->DSPScheduler.push_back(task);
                            printf("onemore LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
                        }
                        foot = cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
                    }
                    else if(GAngle>0)
                    {
                        if(foot==FOOT_LEFT)
                        {
                            pos = fsm->DSPScheduler.size()-1;
                            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                            task.Right[0] = fsm->DSPScheduler[pos].Right[0]+0.001;
                            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                            task.LYaw = fsm->DSPScheduler[pos].RYaw;
                            task.RYaw = fsm->DSPScheduler[pos].RYaw;
                            task.HTime = DSPHoldTime;
                            fsm->DSPScheduler.push_back(task);
                            printf("onemore LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
                        }
                        foot = ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
                    }

                }
                else if(userData->G2M.WalkingGoalGoModeCommand == GOALGO_DIAGONAL)
                {
                    printf("+++++++++++++++++++++++++++++++++++++GOALGO_DIAGONAL+++++++++++++++++++++++++++++++++++++\n");
                    printf("Combination Walking!!!!!\n");
                    getCurrentPos();

                    int foot =1;
                    if(GY>0)
                        foot = planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
                    else
                        foot = planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_LEFT);
                        planningFootPrint_Goal_Rot_For_Rot(GX,GY,GAngle,1,foot);
                }
            }

            pv_Index = 0;

            _preview_flag = false;
            //usleep(100*1000);
            if(joint->Joints[LKN]->RefAngleCurrent<135 && joint->Joints[RKN]->RefAngleCurrent<135)
            fsmFlag = true;
            joint->SetAllMotionOwner();
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;

            break;
        case FREEWALK_WIDE:
        if(fsmFlag == false)
        {
            //sharedData->STATE_COMMAND = TCMD_TERRAIN_WALKING_START;
            DSPHoldTime = 0.;
            L_PEL2PEL = 0.4;
            L_PEL2PEL_OFFSET = 0.001;
            U0_Gain = 1.1;
            cout << ">>> COMMAND: FREEWALK_WIDE" << endl;
            userData->WalkDoneFlag = false;
            fsmDEL_T = 0.005;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1; //fog zero
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
            fsm_clear();
            checkfsm_clear();




            //Mass_ONOFF_Flag = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            if(Mass_ONOFF_Flag == 1)
            {
                kine_drc_hubo4.m_RightWrist = 3.7+3.5;
                kine_drc_hubo4.m_LeftWrist = 3.7+3.5;
                Mass_ONOFF_Flag = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                printf("Mass ON !!!!!!!!!!!!!!!!!!!!!!!!!!!!m_RightWrist = %f,m_LeftWrist = %f %f\n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_Torso);
            }
            else if(Mass_ONOFF_Flag == -1)
            {
                kine_drc_hubo4.m_RightWrist = 3.7;
                kine_drc_hubo4.m_LeftWrist = 3.7;
//                kine_drc_hubo4.m_Torso = 28.6;//+1.0;
                Mass_ONOFF_Flag = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                printf("Mass OFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!m_RightWrist = %f,m_LeftWrist = %f \n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_RightWrist);
            }

            Walking_initialize();

            usleep(10*1000);


            fileName.sprintf("%s", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
            cout << fileName.toStdString().data() << endl;

//            joint->RefreshToCurrentReference();


            GetGain(userData->WalkReadyCOM[2] + COM_Offset);
            fsm->walking_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
            fsm->DSPScheduler.clear();
            if(fsm->walking_mode == NORMAL_WALKING){
                double StepTime = 1.0f;//userData->G2M.StepTime;
                fsm->TIME_DSP = StepTime*2/10.;//1.5f;
                fsm->TIME_SSP = StepTime*8/10.;//3.0f;
//                Pelvis_Orientation_FeedBack_ONOFF = 0.;
//                Leg_Length_FeedBack_ONOFF =0.;
                printf("NORMAL TASK speed set DSP: %f  SSP: %f \n",userData->terrain_variable[0],userData->terrain_variable[1]);

                printf("num = %d, length = %f, angle = %f ",userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle);
                getCurrentPos();
                planningFootPrint(userData->G2M.WalkingModeCommand,userData->G2M.WalkingStopModeCommand,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle);

            }
            else if(fsm->walking_mode == GOAL_WALKING)
            {
                fsm->TIME_DSP = 0.2f;//1.5f;
                fsm->TIME_SSP = 0.8f;//3.0f;
                double GX = userData->G2M.GoalPosX;
                double GY = userData->G2M.GoalPosY;
                double GAngle = userData->G2M.GoalAngle;

                if(GAngle < -180) GAngle = GAngle + 360;
                if(GAngle > 180) GAngle = GAngle -360;

                printf("GoalX = %f, GoalY = %f, Goal angle = %f \n",GX,GY,GAngle);

                if(      ((GY>3.*GX)&&(GY>-3.*GX))  ||   ((GY<3.*GX)&&(GY<-3.*GX))   ||   ((GY>GX/3.)&&(GY<-GX/3.))  ||   ((GY<GX/3.)&&(GY>-GX/3.))   ||((fabs(GY)<max_lengthY)&&(fabs(GX)<max_lengthX)))
                {
                    printf("Combination Walking!!!!!\n");
                    getCurrentPos();
//                    if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                        planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
                    planningFootPrint_Goal(GX,GY,GAngle,1,FOOT_LEFT);
                }
                else
                {
                    printf("Rot For Rot Walking!!!!!\n");
                    getCurrentPos();
//                    if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                        planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
                    planningFootPrint_Goal_Rot_For_Rot(GX,GY,GAngle,1,0);
                }
            }

            pv_Index = 0;

            _preview_flag = false;
            //usleep(100*1000);
            if(joint->Joints[LKN]->RefAngleCurrent<135 && joint->Joints[RKN]->RefAngleCurrent<135)
            fsmFlag = true;
            joint->SetAllMotionOwner();
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;

            break;
        case FREEWALK_DSP_HOLD_WALK:
            if(fsmFlag == false)
            {
                //sharedData->STATE_COMMAND = TCMD_TERRAIN_WALKING_START;
                 Upperbody_Gain_Override();
                DSPHoldTime = 1.1;
                L_PEL2PEL = 0.26;
                L_PEL2PEL_OFFSET = 0.04;
                U0_Gain = 1.0;
                cout << ">>> COMMAND: FREEWALK_WALK" << endl;
                userData->WalkDoneFlag = false;
                fsmDEL_T = 0.005;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1; //fog zero
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
                fsm_clear();
                checkfsm_clear();

                //Mass_ONOFF_Flag = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                if(Mass_ONOFF_Flag == 1)
                {
                    kine_drc_hubo4.m_RightWrist = 3.7+3.5;
                    kine_drc_hubo4.m_LeftWrist = 3.7+3.5;

                    Mass_ONOFF_Flag = 0;
                    sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                    printf("Mass ON !!!!!!!!!!!!!!!!!!!!!!!!!!!!m_RightWrist = %f,m_LeftWrist = %f %f\n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_Torso);
                }
                else if(Mass_ONOFF_Flag == -1)
                {
                    kine_drc_hubo4.m_RightWrist = 3.7;
                    kine_drc_hubo4.m_LeftWrist = 3.7;
    //                kine_drc_hubo4.m_Torso = 28.6;//+1.0;
                    Mass_ONOFF_Flag = 0;
                    sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                    printf("Mass OFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!m_RightWrist = %f,m_LeftWrist = %f \n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_RightWrist);
                }

                Walking_initialize();

                usleep(10*1000);


                fileName.sprintf("%s", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                cout << fileName.toStdString().data() << endl;

    //            joint->RefreshToCurrentReference();


                GetGain(userData->WalkReadyCOM[2]);
                fsm->walking_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
                fsm->DSPScheduler.clear();
                if(fsm->walking_mode == TERRAIN_WALKING){
                    fsm->TIME_DSP = 0.2f*1.4;  //dsp
                    fsm->TIME_SSP = 1.4f*1.4;  //ssp
                    printf("BLOCK CLIMB TASK speed set DSP: %f  SSP: %f \n",userData->terrain_variable[0],userData->terrain_variable[1]);
                    for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++){
                        DSPTask task;
                        task.Left[0]    = userData->G2M.walkingDSP[i*15];
                        task.Left[1]    = userData->G2M.walkingDSP[i*15+1];
                        task.Left[2]    = userData->G2M.walkingDSP[i*15+2];
                        task.LYaw       = userData->G2M.walkingDSP[i*15+3];
                        task.LRoll      = userData->G2M.walkingDSP[i*15+4];
                        task.LPitch     = userData->G2M.walkingDSP[i*15+5];

                        task.Right[0]   = userData->G2M.walkingDSP[i*15+6];
                        task.Right[1]   = userData->G2M.walkingDSP[i*15+7];
                        task.Right[2]   = userData->G2M.walkingDSP[i*15+8];
                        task.RYaw       = userData->G2M.walkingDSP[i*15+9];
                        task.RRoll      = userData->G2M.walkingDSP[i*15+10];
                        task.RPitch     = userData->G2M.walkingDSP[i*15+11];

                        task.COMz[0]    = userData->G2M.walkingDSP[i*15+12];// + _pertubation[i];
                        task.COMz[1]    = userData->G2M.walkingDSP[i*15+13];//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i*14+13];
                        task.COMz[2]    = userData->G2M.walkingDSP[i*15+14];

                        fsm->DSPScheduler.push_back(task);
                    }

                    printf(">>>>>>>>>>>>>>>>save past edge after motion generation<<<<<<<<<<<<<<<\n");

                    if(_floor_num >=2)
                    {
                        _floor_num = 2;
                    }

                    _last_floor_num = _floor_num;





                }else if(fsm->walking_mode == LADDER_WALKING){
                    fsm->TIME_DSP = 0.5f;
                    fsm->TIME_SSP = 3.5f;
                    printf("LADDER CLIMB TASK speed set DSP: %f  SSP: %f \n",userData->terrain_variable[0],userData->terrain_variable[1]);
                }else if(fsm->walking_mode == NORMAL_WALKING){
                    double StepTime = userData->G2M.StepTime;
                    fsm->TIME_DSP = StepTime*1.5/10.;//1.5f;
                    fsm->TIME_SSP = StepTime*8.5/10.;//3.0f;
    //                Pelvis_Orientation_FeedBack_ONOFF = 0.;
    //                Leg_Length_FeedBack_ONOFF =0.;
                    printf("NORMAL TASK speed set DSP: %f  SSP: %f \n",userData->terrain_variable[0],userData->terrain_variable[1]);

                    printf("num = %d, length = %f, angle = %f ",userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle);
                    getCurrentPos();
                    planningFootPrint(userData->G2M.WalkingModeCommand,userData->G2M.WalkingStopModeCommand,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle);

                }
                else if(fsm->walking_mode == GOAL_WALKING)
                {
                    fsm->TIME_DSP = 0.2f;//1.5f;
                    fsm->TIME_SSP = 0.8f;//3.0f;
                    double GX = userData->G2M.GoalPosX;
                    double GY = userData->G2M.GoalPosY;
                    double GAngle = userData->G2M.GoalAngle;


                    if(GAngle < -180) GAngle = GAngle + 360;
                    if(GAngle > 180) GAngle = GAngle -360;

                    printf("GoalX = %f, GoalY = %f, Goal angle = %f \n",GX,GY,GAngle);

                    if(      ((GY>3.*GX)&&(GY>-3.*GX))  ||   ((GY<3.*GX)&&(GY<-3.*GX))   ||   ((GY>GX/3.)&&(GY<-GX/3.))  ||   ((GY<GX/3.)&&(GY>-GX/3.))   ||(( sqrt(GY*GY+GX*GX)<0.3)&&(fabs(GY)<0.15)))
                    {
                        printf("Combination Walking!!!!!\n");
                        getCurrentPos();
//                        if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                            planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_RIGHT);
                        planningFootPrint_Goal(GX,GY,GAngle,1,FOOT_LEFT);
                    }
                    else
                    {
                        printf("Rot For Rot Walking!!!!!\n");
                        getCurrentPos();
//                        if((fabs(fsm->LeftFoot->Pos[0])>0.03)||(fabs(fsm->LeftFoot->Pos[1]-0.1)>0.03)||(fabs(fsm->RightFoot->Pos[0])>0.03)||(fabs(fsm->RightFoot->Pos[1]+0.1)>0.03))
                            planningFootPrint_Goal(-(fsm->LeftFoot->Pos[0]+fsm->RightFoot->Pos[0])/2.,-(fsm->LeftFoot->Pos[1]+fsm->RightFoot->Pos[1])/2., 0,0,FOOT_LEFT);
                        planningFootPrint_Goal_Rot_For_Rot(GX,GY,GAngle,1,0);
                    }
                }

                pv_Index = 0;

                _preview_flag = false;
                //usleep(100*1000);
                if(joint->Joints[LKN]->RefAngleCurrent<135 && joint->Joints[RKN]->RefAngleCurrent<135)
                fsmFlag = true;
                joint->SetAllMotionOwner();
            }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_SAVE:
            cout << ">>> COMMAND: FREEWALK_SAVE" << endl;

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0){
//                MCsetFrictionParameter(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 800, 180, 0);
//                MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, SW_MODE_NON_COMPLEMENTARY);
//                MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, ENABLE);
//                MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 100, 1);
//                MCsetFrictionParameter(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 800, 150, 0);
//                MCBoardSetSwitchingMode(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, SW_MODE_NON_COMPLEMENTARY);
//                MCenableFrictionCompensation(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, ENABLE);
//                MCJointGainOverride(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 100, 1);

//                MCsetFrictionParameter(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 800, 180, 0);
//                MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, SW_MODE_NON_COMPLEMENTARY);
//                MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, ENABLE);
//                MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 100, 1);
////                MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 3, 1, 0, 0);

//                MCsetFrictionParameter(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 800, 210, 0);
//                MCBoardSetSwitchingMode(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, SW_MODE_NON_COMPLEMENTARY);
//                MCenableFrictionCompensation(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, ENABLE);
//                MCJointGainOverride(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 100, 1);
                unsigned int i,j;

                fp = fopen("data.txt","w");
                for(i=0;i<ROW_data_debug;i++)
                {
                    for(j=0;j<COL_data_debug;j++)fprintf(fp,"%g\t", JW_Data_Debug[j][i]);
                    fprintf(fp,"\n");
                }
                fclose(fp);

                fp2 = fopen("data2.txt","w");
                for(i=0;i<ROW_data_debug;i++)
                {
                    for(j=0;j<COL_data_debug;j++)fprintf(fp2,"%g\t", JW_Data_Debug2[j][i]);
                    fprintf(fp2,"\n");
                }
                fclose(fp2);
                cout << ">>> Data txt save~!!" << endl;
            }else{

//                //MCenableFrictionCompensation(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, DISABLE);
//                usleep(2000);
//                MCBoardSetSwitchingMode(JOINT_INFO[RAR].canch,JOINT_INFO[RAR].bno, SW_MODE_COMPLEMENTARY);

//                MCJointGainOverride(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 0,800);
//                usleep(200);

//                //MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, DISABLE);
//                MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno, SW_MODE_COMPLEMENTARY);

//                MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 0,800);
//                usleep(200);

//                //MCenableFrictionCompensation(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, DISABLE);
//                usleep(2000);
//                MCBoardSetSwitchingMode(JOINT_INFO[LAR].canch,JOINT_INFO[LAR].bno, SW_MODE_COMPLEMENTARY);

//                MCJointGainOverride(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 0,800);
//                usleep(200);
//                //MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 0, 0, 0, 0);
//                //MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, DISABLE);
//                MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch,JOINT_INFO[LAP].bno, SW_MODE_COMPLEMENTARY);

//                MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 0,800);
//                usleep(200);


                sJW_Data_Debug_Index=0;
                sJW_Data_Debug_Index2=0;
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_INITIALIZE:
            if(fsmFlag == false)
            {
                //sharedData->STATE_COMMAND = TCMD_TERRAIN_WALKING_START;
                InitializeFLAG =1;
                get_WBIK_Q_from_RefAngleCurrent();
                L_PEL2PEL = 0.26;
                L_PEL2PEL_OFFSET = 0.05;
                U0_Gain = 1;
                userData->WalkDoneFlag = false;
                fsmDEL_T = 0.005;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1; //fog zero
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] = INSIDE_WALKING;

                fsm_clear();
                checkfsm_clear();

                Walking_initialize();
                usleep(200*1000);
                cout << ">>> COMMAND: FREEWALK INITIALIZE" << endl;

                fileName.sprintf("%s", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                cout << fileName.toStdString().data() << endl;
                GetGain(userData->WalkReadyCOM[2] + COM_Offset);

                fsm->walking_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
                fsm->DSPScheduler.clear();
//                if(fsm->walking_mode == TERRAIN_WALKING)
                {
                    fsm->TIME_DSP = 0.1f;  //dsp
                    fsm->TIME_SSP = 0.1f;  //ssp

                    printf("BLOCK CLIMB TASK speed set DSP: %f  SSP: %f \n",userData->terrain_variable[0],userData->terrain_variable[1]);
                    for(int i=0; i<2; i++){
                        DSPTask task;
                        task.Left[0]    = fsm->LeftFoot->Pos[0];
                        task.Left[1]    = fsm->LeftFoot->Pos[1];
                        task.Left[2]    = fsm->LeftFoot->Pos[2];
                        task.LYaw       = fsm->LeftFoot->Yaw;
                        task.LRoll      = fsm->LeftFoot->Roll;
                        task.LPitch     = fsm->LeftFoot->Pitch;

                        task.Right[0]   = fsm->RightFoot->Pos[0];
                        task.Right[1]   = fsm->RightFoot->Pos[1];
                        task.Right[2]   = fsm->RightFoot->Pos[2];
                        task.RYaw       = fsm->RightFoot->Yaw;
                        task.RRoll      = fsm->RightFoot->Roll;
                        task.RPitch     = fsm->RightFoot->Pitch;

                        task.COMz[0]    = fsm->CompData->AddCOM[2];
                        task.COMz[1]    = fsm->CompData->AddCOM[2];
                        task.COMz[2]    = fsm->CompData->AddCOM[2];


                        task.HTime = 0.0;

                        fsm->DSPScheduler.push_back(task);
                    }
                }

                for(int i=0; i<fsm->DSPScheduler.size(); i++)
                {
                    printf("index(%d)  LF: (%.4f,%.4f,%.4f)(%.4f,%.4f,%.4f) RF: (%.4f,%.4f,%.4f)(%.4f,%.4f,%.4f) COMz3:%f COMz:%f COMz2:%f Htime:%f\n", i, fsm->DSPScheduler[i].Left[0], fsm->DSPScheduler[i].Left[1], fsm->DSPScheduler[i].Left[2], fsm->DSPScheduler[i].LYaw, fsm->DSPScheduler[i].LRoll, fsm->DSPScheduler[i].LPitch, fsm->DSPScheduler[i].Right[0], fsm->DSPScheduler[i].Right[1], fsm->DSPScheduler[i].Right[2], fsm->DSPScheduler[i].RYaw , fsm->DSPScheduler[i].RRoll , fsm->DSPScheduler[i].RPitch ,fsm->DSPScheduler[i].COMz[2],fsm->DSPScheduler[i].COMz[0],fsm->DSPScheduler[i].COMz[1],fsm->DSPScheduler[i].HTime);
                }

                pv_Index = 0;

                _preview_flag = false;
                if(joint->Joints[LKN]->RefAngleCurrent<135 && joint->Joints[RKN]->RefAngleCurrent<135)
                fsmFlag = true;
                joint->SetAllMotionOwner();
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_PRE_WALK:
            /*fsmDEL_T = 0.005;
            LandingState = FINAL;
            fsmFlag =false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            Print_WBIK_Infos();*/
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;

        case FREEWALK_TERRAIN:
            if(fsmFlag == false)
            {
                //sharedData->STATE_COMMAND = TCMD_TERRAIN_WALKING_START;
//                Gyro_Ankle_FeedBack_ONOFF = 1.;
//                ZMP_FeedBack_ONOFF =1.;
//                Ankle_Moment_FeedBack_ONOFF = 1.;
//                Pelvis_Orientation_FeedBack_ONOFF = 1.;
//                Leg_Length_FeedBack_ONOFF = 1.;
//                Sagging_Comp_ONOFF = 1.;
//                EarlyLanding_ONOFF =1.;
                FILE_LOG(logWARNING)<<">>>>>>>>> FREEWALK TERRAIN COMMAND <<<<<<<<<";
                DSPHoldTime = 0.;
                Upperbody_Gain_Override();
                L_PEL2PEL = 0.26;
                L_PEL2PEL_OFFSET = 0.04;
                U0_Gain = 0.98;//1.0;//1.08;
                cout << ">>> COMMAND: FREEWALK_TERRAIN" << endl;
                userData->WalkDoneFlag = false;
                fsmDEL_T = 0.005;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1; //fog zero
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;

                fsm_clear();
                checkfsm_clear();
                Walking_initialize();
                usleep(200*1000);

                fileName.sprintf("%s", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                cout << fileName.toStdString().data() << endl;

                GetGain(userData->WalkReadyCOM[2] + COM_Offset);
                fsm->walking_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
                fsm->heel_pitching_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
                fsm->DSPScheduler.clear();

                if(fsm->walking_mode == TERRAIN_WALKING ){
                    fsm->TIME_DSP = 0.9f;  //dsp
                    fsm->TIME_SSP = 1.9f;  //ssp
                    printf("BLOCK CLIMB TASK speed set DSP: %f  SSP: %f \n",fsm->TIME_DSP,fsm->TIME_SSP);


                    int gab =15;

                    if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3] == FIELD_TERRAIN)
                    {
                        for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++){
                            DSPTask task;
                            task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab];
                            task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+1];
                            task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+2];
                            task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+3];
                            task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+4];
                            task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+5];

                            task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+6];
                            task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+7];
                            task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+8];
                            task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+9];
                            task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+10];
                            task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+11];

                            task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+12];
                            task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+13];
                            task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+14];

    //                        task.HTime      =  1.0f;

                            task.HTime      =  0.3f;

                            fsm->DSPScheduler.push_back(task);
                        }
                    }
                    else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3] == GUI_TERRAIN)
                    {
                        for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++){
                            DSPTask task;
                            task.Left[0]    = userData->G2M.walkingDSP[i*gab];
                            task.Left[1]    = userData->G2M.walkingDSP[i*gab+1];
                            task.Left[2]    = userData->G2M.walkingDSP[i*gab+2];
                            task.LYaw       = userData->G2M.walkingDSP[i*gab+3];
                            task.LRoll      = userData->G2M.walkingDSP[i*gab+4];
                            task.LPitch     = userData->G2M.walkingDSP[i*gab+5];

                            task.Right[0]   = userData->G2M.walkingDSP[i*gab+6];
                            task.Right[1]   = userData->G2M.walkingDSP[i*gab+7];
                            task.Right[2]   = userData->G2M.walkingDSP[i*gab+8];
                            task.RYaw       = userData->G2M.walkingDSP[i*gab+9];
                            task.RRoll      = userData->G2M.walkingDSP[i*gab+10];
                            task.RPitch     = userData->G2M.walkingDSP[i*gab+11];

                            task.HTime      =  0.0f;

                            task.COMz[0]    = userData->G2M.walkingDSP[i*gab+12];// + _pertubation[i];
                            task.COMz[1]    = userData->G2M.walkingDSP[i*gab+13];//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i*14+13];
                            task.COMz[2]    = userData->G2M.walkingDSP[i*gab+14];

                            fsm->DSPScheduler.push_back(task);
                        }
                    }

                    printf(">>>>>>>>>>>>>>>>save past edge after motion generation<<<<<<<<<<<<<<<\n");
                }
                else if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP ){
                    fsm->TIME_DSP = 0.6f;  //dsp
                    fsm->TIME_SSP = 2.0f;  //ssp
                    printf("BLOCK CLIMB TASK speed set DSP: %f  SSP: %f \n",fsm->TIME_DSP,fsm->TIME_SSP);
//                    U0_Gain = 0.92;

                    int gab =15;

                    if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3] == FIELD_TERRAIN)
                    {
                        for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++){
                            DSPTask task;
                            task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab];
                            task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+1];
                            task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+2];
                            task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+3];
                            task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+4];
                            task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+5];

                            task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+6];
                            task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+7];
                            task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+8];
                            task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+9];
                            task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+10];
                            task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+11];

                            task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+12];
                            task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+13];
                            task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+14];

    //                        task.HTime      =  1.0f;

                            task.HTime      =  0.5f;

                            fsm->DSPScheduler.push_back(task);
                        }
                    }
                    else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3] == GUI_TERRAIN)
                    {

                        for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++){
                            DSPTask task;
                            task.Left[0]    = userData->G2M.walkingDSP[i*gab];
                            task.Left[1]    = userData->G2M.walkingDSP[i*gab+1];
                            task.Left[2]    = userData->G2M.walkingDSP[i*gab+2];
                            task.LYaw       = userData->G2M.walkingDSP[i*gab+3];
                            task.LRoll      = userData->G2M.walkingDSP[i*gab+4];
                            task.LPitch     = userData->G2M.walkingDSP[i*gab+5];

                            task.Right[0]   = userData->G2M.walkingDSP[i*gab+6];
                            task.Right[1]   = userData->G2M.walkingDSP[i*gab+7];
                            task.Right[2]   = userData->G2M.walkingDSP[i*gab+8];
                            task.RYaw       = userData->G2M.walkingDSP[i*gab+9];
                            task.RRoll      = userData->G2M.walkingDSP[i*gab+10];
                            task.RPitch     = userData->G2M.walkingDSP[i*gab+11];

                            task.HTime      =  0.0f;

                            task.COMz[0]    = userData->G2M.walkingDSP[i*gab+12];// + _pertubation[i];
                            task.COMz[1]    = userData->G2M.walkingDSP[i*gab+13];//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i*14+13];
                            task.COMz[2]    = userData->G2M.walkingDSP[i*gab+14];

                            fsm->DSPScheduler.push_back(task);
                        }
                    }


                    printf(">>>>>>>>>>>>>>>>save past edge after motion generation<<<<<<<<<<<<<<<\n");
                }
                else if(fsm->walking_mode == LADDER_WALKING){
                    fsm->TIME_DSP = 1.3f;  //dsp
                    fsm->TIME_SSP = 2.9f;  //ssp
                    printf("BLOCK CLIMB TASK speed set DSP: %f  SSP: %f \n",fsm->TIME_DSP,fsm->TIME_SSP);
                    U0_Gain = 1.0;
                    double tempRP[3],tempLP[3];

                    int gab =15;

                    for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++){
                        DSPTask task;

                        if(i < 3)
                        {
                            task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab];
                            task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+1];
                            task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+2];
                            task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+3];
                            task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+4];
                            task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+5];

                            task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+6];
                            task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+7];
                            task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+8];
                            task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+9];
                            task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+10];
                            task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+11];

                            task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+12];
                            task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+13];
                            task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[i*gab+14];

                            task.HTime      =  1.f;
                        }else if(i == 3)
                        {
                            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[150]> 50)
                            {
                                printf("STAIR Left Foot FIRST");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13]+0.03f;
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13]+0.03f;
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13]+0.03f;

                                task.HTime      =  1.f;
                            }else
                            {
                                printf("STAIR Right Foot Left");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6]+ sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7]+ sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8]+ sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13]+0.03f;
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13]+0.03f;
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13]+0.03f;

                                task.HTime      =  1.f;
                            }
                        }else if(i == 4)
                        {
                            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[150]> 50)
                            {
                                printf("STAIR Left Foot FIRST");
                                task.Left[0]    = tempLP[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = tempLP[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = tempLP[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = tempRP[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = tempRP[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = tempRP[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];


                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 0.03f;


                                task.HTime      =  1.f;
                            }else
                            {
                                printf("STAIR Left Foot Left");
                                task.Left[0]    = tempLP[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = tempLP[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = tempLP[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = tempRP[0] =sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = tempRP[1] =sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = tempRP[2] =sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 0.03f;

                                task.HTime      =  1.f;
                            }

                        }else if(i == 5)
                        {
                            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[150]> 50)
                            {
                                printf("STAIR Left Foot FIRST");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];


                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;


                                task.HTime      =  1.f;
                            }else
                            {
                                printf("STAIR Right Foot Left");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;

                                task.HTime      =  1.f;
                            }

                        }else if(i == 6)
                        {
                            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[150]> 50)
                            {
                                printf("STAIR Left Foot FIRST");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];


                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;


                                task.HTime      =  1.f;
                            }else
                            {
                                printf("STAIR Right Foot Left");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;

                                task.HTime      =  1.f;
                            }

                        }else if(i == 7)
                        {
                            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[150]> 50)
                            {
                                printf("STAIR Left Foot FIRST");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152] ;
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];


                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;


                                task.HTime      =  1.f;
                            }else
                            {
                                printf("STAIR Right Foot Left");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151] +sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;

                                task.HTime      =  1.f;
                            }

                        }else if(i == 8)
                        {
                            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[150]> 50)
                            {
                                printf("STAIR Left Foot FIRST");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152] ;
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];


                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;


                                task.HTime      =  1.f;
                            }else
                            {
                                printf("STAIR Right Foot Left");
                                task.Left[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151]+ sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Left[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+1]  + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Left[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+2]  + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.LYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+3];
                                task.LRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+4];
                                task.LPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+5];

                                task.Right[0]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+6] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151] +sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[151];
                                task.Right[1]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+7] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[152];
                                task.Right[2]   = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+8] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.RYaw       = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+9];
                                task.RRoll      = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+10];
                                task.RPitch     = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+11];

                                task.COMz[0]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.COMz[1]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[158];
                                task.COMz[2]    = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2*gab+13] + 2.0f*sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[157] + 0.03f;

                                task.HTime      =  1.f;
                            }

                        }


                        fsm->DSPScheduler.push_back(task);
                    }
                }


                for(int i=0; i<fsm->DSPScheduler.size(); i++)
                {
                    printf("index(%d)  LF: (%.4f,%.4f,%.4f)(%.4f,%.4f,%.4f) RF: (%.4f,%.4f,%.4f)(%.4f,%.4f,%.4f) COMz3:%f COMz:%f COMz2:%f Htime:%f\n", i, fsm->DSPScheduler[i].Left[0], fsm->DSPScheduler[i].Left[1], fsm->DSPScheduler[i].Left[2], fsm->DSPScheduler[i].LYaw, fsm->DSPScheduler[i].LRoll, fsm->DSPScheduler[i].LPitch, fsm->DSPScheduler[i].Right[0], fsm->DSPScheduler[i].Right[1], fsm->DSPScheduler[i].Right[2], fsm->DSPScheduler[i].RYaw , fsm->DSPScheduler[i].RRoll , fsm->DSPScheduler[i].RPitch ,fsm->DSPScheduler[i].COMz[2],fsm->DSPScheduler[i].COMz[0],fsm->DSPScheduler[i].COMz[1],fsm->DSPScheduler[i].HTime);
                }

                printf(">>>>>>>>>>>>> FREEWALK_TERRAIN <<<<<<<<<<<<<< \n");

                pv_Index = 0;

                _preview_flag = false;
                //usleep(100*1000);
                if(joint->Joints[LKN]->RefAngleCurrent<135 && joint->Joints[RKN]->RefAngleCurrent<135)
                fsmFlag = true;
                joint->SetAllMotionOwner();
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_MOTION_CHECK:
//            MotionCheck();
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        case FREEWALK_CONTROL_TEST:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;

        default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = FREEWALK_NO_ACT;
            break;
        }


    }
    FILE_LOG(logERROR) << "Process FreeWalking is terminated" << endl;

    return 0;
}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void RBTaskThread(void *){
    while(isTerminated == 0)
    {


        odom_x = (fsm->RightInfos[0][0] + fsm->LeftInfos[0][0])/2.0;
        odom_y = (fsm->RightInfos[0][1] + fsm->LeftInfos[0][1])/2.0;
        odom_theta = (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.0;

        double vx = (odom_x-odom_x_prev)/0.005;
        double vy = (odom_y-odom_y_prev)/0.005;
        double vth = (odom_theta-odom_theta_prev)/0.005;

        odom_x_prev = odom_x;
        odom_y_prev = odom_y;
        odom_theta_prev = odom_theta;

        userData->odom_data[0] = odom_x;
        userData->odom_data[1] = odom_y;
        userData->odom_data[2] = odom_theta;
        userData->odom_data[3] = vx;
        userData->odom_data[4] = vy;
        userData->odom_data[5] = vth;

        if(fsmFlag == true)
        {
            if(fabs(sharedSEN->FOG.Roll)>10||fabs(sharedSEN->FOG.Pitch)>10)
            {
//                Fall_cnt++;
//                if(Fall_cnt>10)
//                {
//                    Fall_Flag = true;
//                    if(sharedSEN->FOG.Roll>0)  Fall_Roll_DIR = FALL_PLUS;
//                    else Fall_Roll_DIR = FALL_MINUS;

//                    if(sharedSEN->FOG.Pitch>0)  Fall_Pitch_DIR = FALL_PLUS;
//                    else Fall_Roll_DIR = FALL_MINUS;
//                }

            }
            else
                Fall_cnt=0;



            if(Fall_Flag == false)
            {
                fsm->Update();

                LF_FZ_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LF_FZ_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*sharedSEN->FT[LAFT].Fz;
                RF_FZ_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*RF_FZ_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*sharedSEN->FT[RAFT].Fz;

                LF_MX_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LF_MX_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*sharedSEN->FT[LAFT].Mx;
                RF_MX_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*RF_MX_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*sharedSEN->FT[RAFT].Mx;

                LF_MY_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LF_MY_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*sharedSEN->FT[LAFT].My;
                RF_MY_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*RF_MY_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*sharedSEN->FT[RAFT].My;
                get_zmp();
                get_zmp2();
                JW_save();
                JW_save2();
                //printf("fsm->StateInfos[0][0] = %d\n ",fsm->StateInfos[0][0]);
                if(_preview_flag == false)
                {
                    //LandingState = FINAL;
                    Preliminary();
                    _preview_flag = true;
                }else
                {
                    ThreeDim_Preview();
                }

                WalkingTimer();
                JW_INV_MODEL(GLOBAL_Y_LIPM_n - (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0,GLOBAL_Y_LIPM_d_n,GLOBAL_X_LIPM,GLOBAL_X_LIPM_d);
//                JW_INV_MODEL(GLOBAL_Y_LIPM_n ,GLOBAL_Y_LIPM_d_n,GLOBAL_X_LIPM_n,GLOBAL_X_LIPM_d_n);
                WBIK();
            }
            else
            {
//                if(Fall_once_Flag == false)
//                GotoFallPos();
            }
            joint->MoveAllJoint();

            if(fabs(sharedSEN->FOG.Roll)>10||fabs(sharedSEN->FOG.Pitch)>10||stopFlag==true)
            {
                Upperbody_Gain_Lock();
                fsmFlag = false;
                Fall_cnt =0;
                Fall_Flag = false;
                fsmFlag = false;
                stopFlag = false;
                Fall_once_Flag = false;
                for(int j=0;j<=STATE_FINISHED;j++)
                {
                    fsm_state_timer[j]=0;
                }
    //            WBIK_PARA_CHANGE();
                y_i_1 = 0;
                y_i_11= 0;
                u_i_1 = 0;
                u_i_11 = 0;
                NotchFilter_GyroRollControlInput(0,0);
                NotchFilter_GyroPitchControlInput(0,0);
                NotchFilter_GyroRollVel(0,0);
                NotchFilter_GyroPitchVel(0,0);
                GLOBAL_Xori_RF_last = 0;
                GLOBAL_Xori_LF_last = 0;
                GLOBAL_Yori_RF_last = 0;
                GLOBAL_Yori_LF_last = 0;

                GLOBAL_Xori_RF2_last = 0;
                GLOBAL_Xori_LF2_last = 0;
                GLOBAL_Yori_RF2_last = 0;

                GLOBAL_Yori_LF2_last = 0;

                U_Gain = 0.;
                GLOBAL_Xori_RF = 0.;
                GLOBAL_Xori_LF = 0.;
                GLOBAL_Yori_RF = 0.;
                GLOBAL_Yori_LF = 0.;
            }
        }
        else
        {
            InitializeFLAG =0;
            if(LandingState == FINAL)
            {
                printf("=============================== Walking Complete !!!!!!!!! ===============================\n");
                fsm_clear();
                checkfsm_clear();
                Walking_initialize();

                userData->M2G.curFootL[0] = fsm->LeftFoot->Pos[0];
                userData->M2G.curFootL[1] = fsm->LeftFoot->Pos[1];
                userData->M2G.curFootL[2] = fsm->LeftFoot->Pos[2];
                userData->M2G.curFootL[3] = fsm->LeftFoot->Yaw;
                userData->M2G.curFootL[4] = fsm->LeftFoot->Roll;
                userData->M2G.curFootL[5] = fsm->LeftFoot->Pitch;

                userData->M2G.curFootR[0] = fsm->RightFoot->Pos[0];
                userData->M2G.curFootR[1] = fsm->RightFoot->Pos[1];
                userData->M2G.curFootR[2] = fsm->RightFoot->Pos[2];
                userData->M2G.curFootR[3] = fsm->RightFoot->Yaw;
                userData->M2G.curFootR[4] = fsm->RightFoot->Roll;
                userData->M2G.curFootR[5] = fsm->RightFoot->Pitch;

                //----------------comz
                userData->M2G.curZMP[0] = X_ZMP;
                userData->M2G.curZMP[1] = Y_ZMP;
                userData->M2G.curZMP[2] = fsm->CompData->AddCOM[2];

                userData->M2G.curPEL[0] = WBIK_Q[0];
                userData->M2G.curPEL[1] = WBIK_Q[1];
                userData->M2G.curPEL[2] = WBIK_Q[2];

                userData->M2G._qPEL[0] = WBIK_Q[3];
                userData->M2G._qPEL[1] = WBIK_Q[4];
                userData->M2G._qPEL[2] = WBIK_Q[5];
                userData->M2G._qPEL[3] = WBIK_Q[6];

                userData->M2G._INIT_COM[0] = init_WBIK_pCOM[0];
                userData->M2G._INIT_COM[1] = init_WBIK_pCOM[1];
                userData->M2G._INIT_COM[2] = init_WBIK_pCOM[2];
                userData->M2G._INIT_PEL[0] = init_WBIK_Q[0];
                userData->M2G._INIT_PEL[1] = init_WBIK_Q[1];
                userData->M2G._INIT_PEL[2] = init_WBIK_Q[2];
                userData->M2G._ADDCOM = fsm->CompData->AddCOM[2];

                LandingState = END;

                userData->WalkDoneFlag = true;// Drill use only

            }
        }

        rt_task_suspend(&rtTaskCon);
    }
}
// --------------------------------------------------------------------------------------------- //
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(isTerminated == 0)
    {
        rt_task_wait_period(NULL);

        usleep(300);
        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
		upperbody_lock_flag = false;
            }
        }
        else
        {
	    if(upperbody_lock_flag == false)
	{
            Upperbody_Gain_Lock();
		upperbody_lock_flag = true;
	}
            Fall_cnt =0;
            Fall_Flag = false;
            fsmFlag = false;
            stopFlag = false;
            Fall_once_Flag = false;
            for(int j=0;j<=STATE_FINISHED;j++)
            {
                fsm_state_timer[j]=0;
            }
//            WBIK_PARA_CHANGE();
            y_i_1 = 0;
            y_i_11= 0;
            u_i_1 = 0;
            u_i_11 = 0;
            NotchFilter_GyroRollControlInput(0,0);
            NotchFilter_GyroPitchControlInput(0,0);
            NotchFilter_GyroRollVel(0,0);
            NotchFilter_GyroPitchVel(0,0);
            GLOBAL_Xori_RF_last = 0;
            GLOBAL_Xori_LF_last = 0;
            GLOBAL_Yori_RF_last = 0;
            GLOBAL_Yori_LF_last = 0;

            GLOBAL_Xori_RF2_last = 0;
            GLOBAL_Xori_LF2_last = 0;
            GLOBAL_Yori_RF2_last = 0;

            GLOBAL_Yori_LF2_last = 0;

            U_Gain = 0.;
            GLOBAL_Xori_RF = 0.;
            GLOBAL_Xori_LF = 0.;
            GLOBAL_Yori_RF = 0.;
            GLOBAL_Yori_LF = 0.;
        }
            //printf("//");
//            LandingState = FINAL;
//            fsmFlag =false;
//            fsm_clear();
//            Walking_initialize();

//            get_zmp2();

//            fsm_clear();
//            //checkfsm_clear();

//            Walking_initialize();

//            LandingState = FINAL;
//            fsmFlag =false;

//            get_zmp2();
//        }
    }
}
// --------------------------------------------------------------------------------------------- //


// --------------------------------------------------------------------------------------------- //
int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
// --------------------------------------------------------------------------------------------- //
int FootSelector(int direction){
    double errTh = 0.0001;
    int pos = fsm->DSPScheduler.size()-1;
    double theta = (fsm->DSPScheduler[pos].LYaw+fsm->DSPScheduler[pos].RYaw)/2.;

    double lx = fsm->DSPScheduler[pos].Left[0];
    double ly = fsm->DSPScheduler[pos].Left[1];
    double rx = fsm->DSPScheduler[pos].Right[0];
    double ry = fsm->DSPScheduler[pos].Right[1];
    double lth = fsm->DSPScheduler[pos].LYaw;
    double rth = fsm->DSPScheduler[pos].RYaw;


    double lx_n,lx_nn,ly_n,ly_nn;
    double rx_n,rx_nn,ry_n,ry_nn;

    lx_n = lx-(rx+lx)/2.;
    ly_n = ly-(ry+ly)/2.;
    rx_n = rx-(rx+lx)/2.;
    ry_n = ry-(ry+ly)/2.;

    lx_nn =  lx_n*cos(theta*D2R) + ly_n*sin(theta*D2R);
    ly_nn = -lx_n*sin(theta*D2R) + ly_n*cos(theta*D2R);

    rx_nn =  rx_n*cos(theta*D2R) + ry_n*sin(theta*D2R);
    ry_nn = -rx_n*sin(theta*D2R) + ry_n*cos(theta*D2R);

    if(direction == DIR_FORWARD){
        if(fabs(lx_nn-rx_nn) < errTh){printf("RIGHT FOOT\n");
            return FOOT_RIGHT;}
        if(lx_nn > rx_nn){printf("RIGHT FOOT\n");
            return FOOT_RIGHT;}
        else{printf("LEFT FOOT\n");
            return FOOT_LEFT;}
    }else if(direction == DIR_BACKWARD){
        if(fabs(lx_nn-rx_nn) < errTh) return FOOT_RIGHT;

        if(lx_nn > rx_nn) return FOOT_LEFT;
        else return FOOT_RIGHT;
    }else if(direction == DIR_RIGHT){
        if((fabs(ly_nn-ry_nn)-L_PEL2PEL) > 0.02){
            return FOOT_LEFT;
        }else{
            return FOOT_RIGHT;
        }
    }else if(direction == DIR_LEFT){
        if((fabs(ly_nn-ry_nn)-L_PEL2PEL) > 0.02){
            return FOOT_RIGHT;
        }else{
            return FOOT_LEFT;
        }
    }else if(direction == DIR_CW){
        if(fabs(lth-rth)>0.01)
        {
            printf("GUI foot selector : FOOT_LEFT %f =!%f\n",lth,rth);
            return FOOT_LEFT;
        }
        else
            return FOOT_RIGHT;
    }else if(direction == DIR_CCW){
        if(fabs(lth-rth)>0.01)
        {
            printf("GUI foot selector : FOOT_RIGHT\n");
            return FOOT_RIGHT;
        }
        else
            return FOOT_LEFT;
    }
    else
    {
        return FOOT_RIGHT;
    }
}

// --------------------------------------------------------------------------------------------- //
int planningFootPrint_Goal(double _GoalPosX, double _GoalPosY, double _GoalAngle , int _OneMoreMode,int _firstfoot)
{
//    forward_foot_print(0,1,0.1,0);
    int number ;
    double lengthX;// = _GoalPosX/num;
    double lengthY;// = _GoalPosY/num;
//    double angleZ = 0.;
    int foot = 0,pos;

    if(_GoalAngle <-180)_GoalAngle = _GoalAngle + 360;
    if(_GoalAngle > 180)_GoalAngle = _GoalAngle - 360;
    DSPTask task;
    double _GoalPosPX = _GoalPosX - L_FOOT/2.*cos(_GoalAngle*D2R) - (-0.03)*sin(_GoalAngle*D2R);
    double _GoalPosPY = _GoalPosY - L_FOOT/2.*sin(_GoalAngle*D2R) ;//+ (-0.03)*cos(_GoalAngle*D2R);

//    printf("%f,%f , %\n",(fabs(_GoalPosX))/max_lengthX , fabs(_GoalPosY)/max_lengthY);
    if((fabs(_GoalPosX))/max_lengthX > fabs(_GoalPosY)/max_lengthY)
    {
        number = int(fabs(_GoalPosX)/max_lengthX)+1;
        lengthX = _GoalPosX/number;
        lengthY = _GoalPosY/(number- int(number/2));
//        angleZ = _GoalAngle/(number- int(number/2));
//        printf("XXXXXXX");
    }
    else
    {
        int N;
        N = int(fabs(_GoalPosY)/max_lengthY)+1;
        number = 2*N-1;
        lengthX = _GoalPosX/number;
        lengthY = _GoalPosY/N;
//        angleZ = _GoalAngle/N;
//        printf("YYYYYYY,N = %d, lengthY = %f",N,lengthY);

    }
    _GoalPosX = _GoalPosPX - ( - L_FOOT/2.);
    _GoalPosY = _GoalPosPY - (0);


//    printf("num = %d, lX = %f , lY = %f\n", number,lengthX,lengthY);

    if(_OneMoreMode == 1)
    {
        if(_firstfoot==FOOT_RIGHT && _GoalPosY<=0)
        {
            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0]+0.001;
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];
            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RYaw;
            task.RPitch = fsm->DSPScheduler[pos].RYaw;

            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            task.HTime = DSPHoldTime;
            fsm->DSPScheduler.push_back(task);
            printf("one more %dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",1,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
        }

        if(_firstfoot==FOOT_LEFT && _GoalPosY>0)
        {
            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];
            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;

            task.Right[0] = fsm->DSPScheduler[pos].Right[0]+0.001;
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RYaw;
            task.RPitch = fsm->DSPScheduler[pos].RYaw;

            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            task.HTime = DSPHoldTime;
            fsm->DSPScheduler.push_back(task);
            printf("one more %dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",1,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
        }
    }

    for(int i=0; i<number; i++)
    {
        if(_GoalPosY<=0)
        {
            if(i==0)foot = _firstfoot;
            pos = fsm->DSPScheduler.size()-1;
            switch(foot)
            {
            case FOOT_RIGHT:
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];
                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;

                task.Right[0] = fsm->DSPScheduler[pos].Left[0] - (lengthY - (L_PEL2PEL-L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[1] = fsm->DSPScheduler[pos].Left[1] + (lengthY - (L_PEL2PEL-L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[2] = fsm->DSPScheduler[pos].Left[2];
                task.RYaw = fsm->DSPScheduler[pos].LYaw;
                task.RRoll = 0;
                task.RPitch = 0;

                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                task.HTime = DSPHoldTime;
                fsm->DSPScheduler.push_back(task);
                printf("1 %dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
                foot = FOOT_LEFT;
                break;
            case FOOT_LEFT:
                task.Left[0] = fsm->DSPScheduler[pos].Right[0] - ((L_PEL2PEL-L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[1] = fsm->DSPScheduler[pos].Right[1] + ((L_PEL2PEL-L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[2] = fsm->DSPScheduler[pos].Right[2];
                task.LYaw = fsm->DSPScheduler[pos].RYaw;
                task.LRoll = 0;
                task.LPitch = 0;

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;

                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                task.HTime = DSPHoldTime;

                fsm->DSPScheduler.push_back(task);
                printf("2 %dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
                foot = FOOT_RIGHT;
                break;
            default:
                break;
            }
        }
        else
        {
            if(i==0)foot = FOOT_LEFT;
            pos = fsm->DSPScheduler.size()-1;
            switch(foot)
            {
            case FOOT_RIGHT:
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];
                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;

                task.Right[0] = fsm->DSPScheduler[pos].Left[0] - (-(L_PEL2PEL-L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[1] = fsm->DSPScheduler[pos].Left[1] + (-(L_PEL2PEL-L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[2] = fsm->DSPScheduler[pos].Left[2];
                task.RYaw = 0;
                task.RRoll = 0;
                task.RPitch = 0;

                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                task.HTime = DSPHoldTime;
                fsm->DSPScheduler.push_back(task);
                printf("3 %dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
                foot = FOOT_LEFT;
                break;
            case FOOT_LEFT:
                task.Left[0] = fsm->DSPScheduler[pos].Right[0] - ((L_PEL2PEL-L_PEL2PEL_OFFSET) + lengthY)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[1] = fsm->DSPScheduler[pos].Right[1] + ((L_PEL2PEL-L_PEL2PEL_OFFSET) + lengthY)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + lengthX*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[2] = fsm->DSPScheduler[pos].Right[2];
                task.LYaw = 0;
                task.LRoll = 0;
                task.LPitch = 0;

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                task.HTime = DSPHoldTime;

                fsm->DSPScheduler.push_back(task);
                printf("4 %dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
                foot = FOOT_RIGHT;
                break;
            default:
                break;
            }
        }
    }
    pos = fsm->DSPScheduler.size()-1;
    if(foot == FOOT_RIGHT)
    {
        task.Left[0] = fsm->DSPScheduler[pos].Left[0];
        task.Left[1] = fsm->DSPScheduler[pos].Left[1];
        task.Left[2] = fsm->DSPScheduler[pos].Left[2];

        task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
        task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
        task.Right[2] = fsm->DSPScheduler[pos].Left[2];

        task.LYaw = fsm->DSPScheduler[pos].LYaw;
        task.LRoll = 0;
        task.LPitch = 0;
        task.RYaw = fsm->DSPScheduler[pos].LYaw;
        task.RRoll = 0;
        task.RPitch = 0;
        task.HTime = DSPHoldTime;
        fsm->DSPScheduler.push_back(task);
        printf("5 last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

    }else
    {
        task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
        task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
        task.Left[2] = fsm->DSPScheduler[pos].Right[2];

        task.Right[0] = fsm->DSPScheduler[pos].Right[0];
        task.Right[1] = fsm->DSPScheduler[pos].Right[1];
        task.Right[2] = fsm->DSPScheduler[pos].Right[2];

        task.LYaw = fsm->DSPScheduler[pos].RYaw;
        task.LRoll = 0;
        task.LPitch = 0;
        task.RYaw = fsm->DSPScheduler[pos].RYaw;
        task.RRoll = 0;
        task.RPitch = 0;
        task.HTime = DSPHoldTime;
        fsm->DSPScheduler.push_back(task);
        printf("6 last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

    }

//    if(_OneMoreMode == 1)
//    {
//        printf("one more\n");
//        if(foot==FOOT_RIGHT)
//        foot = forward_foot_print(COMPLETE_STOP_WALKING,1,0.001,FOOT_LEFT);
//        else if(foot==FOOT_LEFT)
//        foot = forward_foot_print(COMPLETE_STOP_WALKING,1,0.001,FOOT_RIGHT);
//    }
    int _walking_num = int(fabs(_GoalAngle)/max_angleZ)+1;
    double _rotangle = _GoalAngle/_walking_num;
//    printf("\n foot = %d \n",foot);

    if(_GoalAngle<0)
    {
        if(foot==FOOT_RIGHT)
        {
            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0]+0.001;
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].RYaw;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.HTime = DSPHoldTime;
            fsm->DSPScheduler.push_back(task);
            printf("onemore LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
        }
        foot = cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
    }
    else if(_GoalAngle>0)
    {
        if(foot==FOOT_LEFT)
        {
            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0]+0.001;
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].RYaw;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.HTime = DSPHoldTime;
            fsm->DSPScheduler.push_back(task);
            printf("onemore LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
        }
        foot = ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
    }

    return foot;
}

// --------------------------------------------------------------------------------------------- //
void getCurrentPos()
{
    DSPTask task;
    task.Left[0] = fsm->LeftFoot->Pos[0];
    task.Left[1] = fsm->LeftFoot->Pos[1];
    task.Left[2] = fsm->LeftFoot->Pos[2];
    task.LYaw = fsm->LeftFoot->Yaw;
    task.Right[0] = fsm->RightFoot->Pos[0];
    task.Right[1] = fsm->RightFoot->Pos[1];
    task.Right[2] = fsm->RightFoot->Pos[2];
    task.RYaw = fsm->RightFoot->Yaw;
    task.COMz[0] = InitialComz = fsm->CompData->AddCOM[2];
    task.COMz[1] = InitialComz;
    task.COMz[2] = InitialComz;
    task.HTime = 0.0;

    task.RRoll = fsm->RightFoot->Roll;
    task.RPitch = fsm->RightFoot->Pitch;

    task.LRoll = fsm->LeftFoot->Roll;
    task.LPitch = fsm->LeftFoot->Pitch;

    fsm->DSPScheduler.push_back(task);
    printf("1st LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
}
// --------------------------------------------------------------------------------------------- //
int planningFootPrint_Goal_Rot_For_Rot(double _GoalPosX, double _GoalPosY, double _GoalAngle  , int _OneMoreMode,int _firstfoot)
{

    int pos = fsm->DSPScheduler.size()-1;
    double lx = fsm->DSPScheduler[pos].Left[0];
    double ly = fsm->DSPScheduler[pos].Left[1];
    double rx = fsm->DSPScheduler[pos].Right[0];
    double ry = fsm->DSPScheduler[pos].Right[1];
    double P0[2],P0_rot[2],P_goal_rot[2],P_goal[2],th_path,th_goal,th_final;

    th_goal = _GoalAngle;
    P_goal[0] = _GoalPosX;
    P_goal[1] = _GoalPosY;
    P_goal_rot[0] = P_goal[0] - L_FOOT/2.*cos(th_goal*D2R);
    P_goal_rot[1] = P_goal[1] - L_FOOT/2.*sin(th_goal*D2R);
    P0[0] = (lx+rx)/2;
    P0[1] = (ly+ry)/2;
    P0_rot[0] = P0[0] -L_FOOT/2.;
    P0_rot[1] = P0[1];

    th_path = atan2(P_goal_rot[1]-P0_rot[1],P_goal_rot[0]-P0_rot[0])*R2D;
    printf("P goal rot = (%f,%f) , P0 rot = (%f,%f)\n",P_goal_rot[0],P_goal_rot[1],P0_rot[0],P0_rot[1]);
    printf("theta path = %f\n",th_path);
    int _walking_num;// = int(fabs(th_path)/max_angleZ)+1;
    double _rotangle;// = th_path/_walking_num;
    int foot;
    DSPTask task;
    if(fabs(th_path)<90)
    {
        _walking_num = int(fabs(th_path)/max_angleZ)+1;
        _rotangle = th_path/_walking_num;
        if(th_path > 0)
        {
            if(_firstfoot == FOOT_LEFT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0] + 0.001;
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            foot = ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
        }
        else
        {
            if(_firstfoot == FOOT_RIGHT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0] + 0.001;
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            foot =cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
        }

        double _forw_dist= sqrt((P_goal_rot[0]-P0_rot[0])*(P_goal_rot[0]-P0_rot[0]) + (P_goal_rot[1]-P0_rot[1])*(P_goal_rot[1]-P0_rot[1]));
        _walking_num = (int)(fabs(_forw_dist)/max_lengthX)+1;
        double _steplength = _forw_dist/_walking_num;
        printf("_forw_dist = %f,_walking_num = %d, _steplength = %f\n",_forw_dist,_walking_num,_steplength);
        foot = forward_foot_print(COMPLETE_STOP_WALKING,_walking_num,_steplength,foot*-1);

        th_final = th_goal - th_path;
        if(th_final < -180) th_final = th_final + 360;
        if(th_final > 180) th_final = th_final -360;
        printf("theat final = %f\n",th_final);
        _walking_num = int(fabs(th_final)/max_angleZ)+1;
        _rotangle = th_final/_walking_num;



        if(th_final > 0)
        {
            if(foot == FOOT_LEFT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0] + 0.001;
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
        }
        else
        {
            if(foot == FOOT_RIGHT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0] + 0.001;
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
        }
    }
    else
    {
        if(th_path > 0) th_path = th_path -180;
                else th_path = th_path +180;

        _walking_num = int(fabs(th_path)/max_angleZ)+1;
        _rotangle = th_path/_walking_num;
        if(th_path > 0)
        {
            if(_firstfoot == FOOT_LEFT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0] + 0.001;
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            foot = ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
        }
        else
        {
            if(_firstfoot == FOOT_RIGHT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0] + 0.001;
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            foot = cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
        }

        double _forw_dist= sqrt((P_goal_rot[0]-P0_rot[0])*(P_goal_rot[0]-P0_rot[0]) + (P_goal_rot[1]-P0_rot[1])*(P_goal_rot[1]-P0_rot[1]));
        _walking_num = int(fabs(_forw_dist)/max_lengthX)+1;
        double _steplength = _forw_dist/_walking_num;
        foot = backward_foot_print(COMPLETE_STOP_WALKING,_walking_num,_steplength,foot*-1);

        th_final = th_goal - th_path;
        if(th_final < -180) th_final = th_final + 360;
        if(th_final > 180) th_final = th_final -360;
        printf("theat final = %f\n",th_final);
        _walking_num = int(fabs(th_final)/max_angleZ)+1;
        _rotangle = th_final/_walking_num;



        if(th_final > 0)
        {
            if(foot == FOOT_LEFT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0] + 0.001;
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            ccwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,_rotangle);
        }
        else
        {
            if(foot == FOOT_RIGHT)
            {
                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0] + 0.001;
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);
            }
            cwrot_foot_print(COMPLETE_STOP_WALKING,_walking_num,-_rotangle);
        }
    }

    pos = fsm->DSPScheduler.size()-1;
    lx = fsm->DSPScheduler[pos].Left[0];
    ly = fsm->DSPScheduler[pos].Left[1];
    rx = fsm->DSPScheduler[pos].Right[0];
    ry = fsm->DSPScheduler[pos].Right[1];
    printf("goal x = %f, goal y = %f, goal angle = %f\n",(lx+rx)/2.,(ly+ry)/2.,fsm->DSPScheduler[pos].LYaw);


}
// --------------------------------------------------------------------------------------------- //
void planningFootPrint(int _walkingmode,int _walkingstopmode,int _walking_num, double _steplength, double _rotangle )
{
    switch(_walkingmode)
    {
        case FORWARD_WALKING:
            forward_foot_print(_walkingstopmode,_walking_num,_steplength,FOOT_RIGHT);
            break;
        case BACKWARD_WALKING:
            backward_foot_print(_walkingstopmode,_walking_num,_steplength,FOOT_RIGHT);
            break;
        case RIGHTSIDE_WALKING:
            rightside_foot_print(_walkingstopmode,_walking_num,_steplength,FOOT_RIGHT);
            break;
        case LEFTSIDE_WALKING:
            leftside_foot_print(_walkingstopmode,_walking_num,_steplength,FOOT_LEFT);
            break;
        case CWROT_WALKING:
            cwrot_foot_print(_walkingstopmode,_walking_num,_rotangle);
            break;
        case CCWROT_WALKING:
            ccwrot_foot_print(_walkingstopmode,_walking_num,_rotangle);
            break;
    }

}
// --------------------------------------------------------------------------------------------- //
int forward_foot_print(int _walkingstopmode,int _walking_num,double _steplength,int _firstfoot)
{
    printf("+++++++++forward_foot_print+++++++++++\n");
    DSPTask task;
    int num = _walking_num;
    double length = _steplength;
    int foot = _firstfoot;
        for(int i=0; i<num; i++)
        {
//            int foot = FootSelector(DIR_FORWARD);
            int pos = fsm->DSPScheduler.size()-1;
//            if(fsm->DSPScheduler[pos].Left[0] - fsm->DSPScheduler[pos].Right[0] >= 0.05) foot = FOOT_RIGHT;
//            else if(fsm->DSPScheduler[pos].Left[0] - fsm->DSPScheduler[pos].Right[0] < -0.05) foot = FOOT_LEFT;

            if(foot == FOOT_RIGHT)
            {
                printf("FOOT_RIGHT!!\n");
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Left[0] + L_PEL2PEL*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + length*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[1] = fsm->DSPScheduler[pos].Left[1] - L_PEL2PEL*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + length*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[2] = fsm->DSPScheduler[pos].Left[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = 0;
                task.RPitch = 0;
                task.HTime = DSPHoldTime;
                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                fsm->DSPScheduler.push_back(task);
                printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
                foot = FOOT_LEFT;
            }else
            { printf("FOOT_LEFT!!\n");
                task.Left[0] = fsm->DSPScheduler[pos].Right[0] - L_PEL2PEL*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + length*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[1] = fsm->DSPScheduler[pos].Right[1] + L_PEL2PEL*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) + length*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[2] = fsm->DSPScheduler[pos].Right[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = 0;
                task.LPitch = 0;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = DSPHoldTime;
                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;

                fsm->DSPScheduler.push_back(task);
                printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
                foot = FOOT_RIGHT;
            }
        }

        if(_walkingstopmode == COMPLETE_STOP_WALKING)
        {
            int foot = FootSelector(DIR_FORWARD);
            int pos = fsm->DSPScheduler.size()-1;
            if(foot == FOOT_RIGHT)
            {
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[2] = fsm->DSPScheduler[pos].Left[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = 0;
                task.RPitch = 0;
                task.HTime = DSPHoldTime;
                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                fsm->DSPScheduler.push_back(task);

                printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
            }else
            {
                task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (L_PEL2PEL)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (L_PEL2PEL)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[2] = fsm->DSPScheduler[pos].Right[2];

                task.Right[0] = fsm->DSPScheduler[pos].Right[0];
                task.Right[1] = fsm->DSPScheduler[pos].Right[1];
                task.Right[2] = fsm->DSPScheduler[pos].Right[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = 0;
                task.LPitch = 0;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = fsm->DSPScheduler[pos].RRoll;
                task.RPitch = fsm->DSPScheduler[pos].RPitch;
                task.HTime = DSPHoldTime;
                task.COMz[0] = InitialComz;
                task.COMz[1] = InitialComz;
                task.COMz[2] = InitialComz;
                fsm->DSPScheduler.push_back(task);
                printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

                pos = fsm->DSPScheduler.size()-1;
                task.Left[0] = fsm->DSPScheduler[pos].Left[0];
                task.Left[1] = fsm->DSPScheduler[pos].Left[1];
                task.Left[2] = fsm->DSPScheduler[pos].Left[2];

                task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[2] = fsm->DSPScheduler[pos].Left[2];

                task.LYaw = fsm->DSPScheduler[pos].LYaw;
                task.LRoll = fsm->DSPScheduler[pos].LRoll;
                task.LPitch = fsm->DSPScheduler[pos].LPitch;
                task.RYaw = fsm->DSPScheduler[pos].RYaw;
                task.RRoll = 0;
                task.RPitch = 0;
                task.HTime = 0.0;
                task.COMz[0] = InitialComz;
                fsm->DSPScheduler.push_back(task);

                printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
            }
        }
        foot = FOOT_RIGHT;
        return foot;
}
// --------------------------------------------------------------------------------------------- //
int backward_foot_print(int _walkingstopmode,int _walking_num,double _steplength,int _firstfoot)
{
    printf("+++++++++backward_foot_print+++++++++++\n");
    DSPTask task;
    int num = _walking_num;
    double length = _steplength;
    int foot = _firstfoot;
    for(int i=0; i<num; i++)
    {
//        foot = FootSelector(DIR_BACKWARD);
        printf("foot = %d, num = %d", foot, num);
        int pos = fsm->DSPScheduler.size()-1;
        if(foot == FOOT_RIGHT)
        {
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + L_PEL2PEL*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) - length*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - L_PEL2PEL*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) - length*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = 0;
            task.RPitch = 0;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
            foot = FOOT_LEFT;
        }else
        {
            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - L_PEL2PEL*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) - length*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + L_PEL2PEL*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R) - length*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = 0;
            task.LPitch = 0;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;

            fsm->DSPScheduler.push_back(task);
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
            foot = FOOT_RIGHT;
        }
    }

    if(_walkingstopmode == COMPLETE_STOP_WALKING)
    {
        int foot = FootSelector(DIR_BACKWARD);
        int pos = fsm->DSPScheduler.size()-1;
        if(foot == FOOT_RIGHT)
        {
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = 0;
            task.RPitch = 0;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
//            foot = FOOT_RIGHT;
        }else
        {
            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (L_PEL2PEL)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (L_PEL2PEL)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = 0;
            task.LPitch = 0;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = 0;
            task.RPitch = 0;
            task.HTime = 0.0;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);

        }
    }
    foot = FOOT_RIGHT;
    return foot;

}
// --------------------------------------------------------------------------------------------- //
int rightside_foot_print(int _walkingstopmode,int _walking_num,double _steplength,int _firstfoot)
{
    printf("+++++++++rightside_foot_print+++++++++++\n");
    DSPTask task;
//    task.Left[0] = fsm->LeftFoot->Pos[0];
//    task.Left[1] = fsm->LeftFoot->Pos[1];
//    task.Left[2] = fsm->LeftFoot->Pos[2];
//    task.LYaw = fsm->LeftFoot->Yaw;
//    task.Right[0] = fsm->RightFoot->Pos[0];
//    task.Right[1] = fsm->RightFoot->Pos[1];
//    task.Right[2] = fsm->RightFoot->Pos[2];
//    task.RYaw = fsm->RightFoot->Yaw;
//    task.COMz[0] = fsm->CompData->AddCOM[2];
//    task.HTime = 0.0;
//    task.RRoll = 0.;
//    task.RPitch = 0.;

//    task.LRoll = 0.;
//    task.LPitch = 0.;
//    fsm->DSPScheduler.push_back(task);

    int num = _walking_num;
    double length = _steplength;
    int pos;

    int foot = _firstfoot;//FootSelector(DIR_RIGHT);
    if(foot == FOOT_LEFT)
    {
        pos = fsm->DSPScheduler.size()-1;
        task.Left[0] = fsm->DSPScheduler[pos].Left[0] + 0.001;
        task.Left[1] = fsm->DSPScheduler[pos].Left[1];
        task.Left[2] = fsm->DSPScheduler[pos].Left[2];

        task.Right[0] = fsm->DSPScheduler[pos].Right[0];
        task.Right[1] = fsm->DSPScheduler[pos].Right[1];
        task.Right[2] = fsm->DSPScheduler[pos].Right[2];

        task.LYaw = fsm->DSPScheduler[pos].LYaw;
        task.LRoll = fsm->DSPScheduler[pos].LRoll;
        task.LPitch = fsm->DSPScheduler[pos].LPitch;
        task.RYaw = fsm->DSPScheduler[pos].RYaw;
        task.RRoll = fsm->DSPScheduler[pos].RRoll;
        task.RPitch = fsm->DSPScheduler[pos].RPitch;
        task.HTime = DSPHoldTime;
        task.COMz[0] = InitialComz;
        task.COMz[1] = InitialComz;
        task.COMz[2] = InitialComz;

        fsm->DSPScheduler.push_back(task);
    }


    for(int i=0; i<num; i++){
        foot = FootSelector(DIR_RIGHT);
        pos = fsm->DSPScheduler.size()-1;
        if(foot == FOOT_RIGHT){
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (length+(L_PEL2PEL - L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (length+(L_PEL2PEL - L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = 0;
            task.RPitch = 0;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;

            fsm->DSPScheduler.push_back(task);
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);

            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (L_PEL2PEL - L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (L_PEL2PEL - L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = 0;
            task.LPitch = 0;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_LEFT;
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
        }else{
            // already stretched
            cout << "stretched" << endl;
            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - ((L_PEL2PEL - L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + ((L_PEL2PEL - L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = 0;
            task.LPitch = 0;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.HTime = 0.0;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_RIGHT;
            printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
        }
    }

    if(_walkingstopmode == SPREAD_STOP_WALKING){
        fsm->DSPScheduler.pop_back();
    }
    foot = FOOT_LEFT;
    return foot;
}
// --------------------------------------------------------------------------------------------- //
int leftside_foot_print(int _walkingstopmode,int _walking_num,double _steplength,int _firstfoot)
{
    printf("+++++++++leftside_foot_print+++++++++++\n");
    DSPTask task;
//    task.Left[0] = fsm->LeftFoot->Pos[0];
//    task.Left[1] = fsm->LeftFoot->Pos[1];
//    task.Left[2] = fsm->LeftFoot->Pos[2];
//    task.LYaw = fsm->LeftFoot->Yaw;
//    task.Right[0] = fsm->RightFoot->Pos[0];
//    task.Right[1] = fsm->RightFoot->Pos[1];
//    task.Right[2] = fsm->RightFoot->Pos[2];
//    task.RYaw = fsm->RightFoot->Yaw;
//    task.COMz[0] = fsm->CompData->AddCOM[2];
//    task.HTime = 0.0;
//    task.RRoll = 0.;
//    task.RPitch = 0.;

//    task.LRoll = 0.;
//    task.LPitch = 0.;
//    fsm->DSPScheduler.push_back(task);
    int num = _walking_num;
    double length = _steplength;
    int pos;

    int foot = _firstfoot;//FootSelector(DIR_RIGHT);
    if(foot == FOOT_RIGHT)
    {
        pos = fsm->DSPScheduler.size()-1;
        task.Left[0] = fsm->DSPScheduler[pos].Left[0];
        task.Left[1] = fsm->DSPScheduler[pos].Left[1];
        task.Left[2] = fsm->DSPScheduler[pos].Left[2];

        task.Right[0] = fsm->DSPScheduler[pos].Right[0] + 0.001;
        task.Right[1] = fsm->DSPScheduler[pos].Right[1];
        task.Right[2] = fsm->DSPScheduler[pos].Right[2];

        task.LYaw = fsm->DSPScheduler[pos].LYaw;
        task.LRoll = fsm->DSPScheduler[pos].LRoll;
        task.LPitch = fsm->DSPScheduler[pos].LPitch;
        task.RYaw = fsm->DSPScheduler[pos].RYaw;
        task.RRoll = fsm->DSPScheduler[pos].RRoll;
        task.RPitch = fsm->DSPScheduler[pos].RPitch;
        task.HTime = DSPHoldTime;
        task.COMz[0] = InitialComz;
        task.COMz[1] = InitialComz;
        task.COMz[2] = InitialComz;

        fsm->DSPScheduler.push_back(task);
    }

    for(int i=0; i<num; i++){
        foot = FootSelector(DIR_LEFT);
        pos = fsm->DSPScheduler.size()-1;
        if(foot == FOOT_LEFT){
            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (length+(L_PEL2PEL - L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (length+(L_PEL2PEL - L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = 0;
            task.LPitch = 0;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);

            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL - L_PEL2PEL_OFFSET)*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL - L_PEL2PEL_OFFSET)*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = 0;
            task.RPitch = 0;
            task.HTime = DSPHoldTime;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_LEFT;
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+3,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);
        }else{
            // already stretched
            cout << "stretched" << endl;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + ((L_PEL2PEL - L_PEL2PEL_OFFSET))*sin((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - ((L_PEL2PEL - L_PEL2PEL_OFFSET))*cos((fsm->DSPScheduler[pos].LYaw + fsm->DSPScheduler[pos].RYaw)/2.*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = 0;
            task.RPitch = 0;
            task.HTime = 0.0;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_RIGHT;
            printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
        }
    }

    if(_walkingstopmode == SPREAD_STOP_WALKING){
        fsm->DSPScheduler.pop_back();
    }

    foot = FOOT_RIGHT;
    return foot;

}
// --------------------------------------------------------------------------------------------- //
int cwrot_foot_print(int _walkingstopmode,int _walking_num,double _rotangle)
{
    printf("+++++++++cwrot_foot_print+++++++++++\n");
    DSPTask task;
    int num = _walking_num;
    double angle = _rotangle;
    int foot = FOOT_RIGHT;//FootSelector(DIR_CW);
    for(int i=0; i<num; i++){
        foot = FOOT_RIGHT;
        int pos = fsm->DSPScheduler.size()-1;
        double th_l = (fsm->DSPScheduler[pos].LYaw+fsm->DSPScheduler[pos].RYaw)/2.*D2R;
        double lx = fsm->DSPScheduler[pos].Left[0];
        double ly = fsm->DSPScheduler[pos].Left[1];
//        double rx = fsm->DSPScheduler[pos].Right[0];
//        double ry = fsm->DSPScheduler[pos].Right[1];
        if(i ==0)
        {
//            ry = fsm->DSPScheduler[pos].Left[1] - L_PEL2PEL*cos(th_l);
//            rx = fsm->DSPScheduler[pos].Left[0] + L_PEL2PEL*sin(th_l);
        }
        double px = lx + L_PEL2PEL/2.*sin(th_l) - L_FOOT/2.*cos(th_l);
        double py = ly - L_PEL2PEL/2.*cos(th_l) - L_FOOT/2.*sin(th_l);
        double X1 = cos(-angle*D2R)*(L_FOOT/2.)-sin(-angle*D2R)*(-L_PEL2PEL/2.);
        double Y1 = sin(-angle*D2R)*(L_FOOT/2.)+cos(-angle*D2R)*(-L_PEL2PEL/2.);

        double X = cos(th_l)*X1 - sin(th_l)*Y1 + px;
        double Y = sin(th_l)*X1 + cos(th_l)*Y1 + py;

        if(foot == FOOT_RIGHT){
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];
            task.Right[0] = X;
            task.Right[1] = Y;
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];
            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.RYaw = fsm->DSPScheduler[pos].RYaw - angle;
            task.HTime = DSPHoldTime;
            task.RRoll = 0.;
            task.RPitch = 0.;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);

            pos = fsm->DSPScheduler.size()-1;

            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - L_PEL2PEL*sin(fsm->DSPScheduler[pos].RYaw*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + L_PEL2PEL*cos(fsm->DSPScheduler[pos].RYaw*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];
            if(i == num-1)
            {
                task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (L_PEL2PEL -L_PEL2PEL_OFFSET)*sin(fsm->DSPScheduler[pos].RYaw*D2R);
                task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (L_PEL2PEL -L_PEL2PEL_OFFSET)*cos(fsm->DSPScheduler[pos].RYaw*D2R);
            }

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];// + X_LF*cos(angle*D2R) - Y_LF*sin(angle*D2R)- 0.12;
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];// + X_LF*sin(angle*D2R) + Y_LF*cos(angle*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];
            task.LYaw = fsm->DSPScheduler[pos].RYaw;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.LRoll = 0.;
            task.LPitch = 0.;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_LEFT;
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

        }else{
            // already stretched
            cout << "stretched" << endl;
            task.Left[0] = fsm->DSPScheduler[pos].Right[0] - (L_PEL2PEL)*sin(fsm->DSPScheduler[pos].RYaw*D2R);
            task.Left[1] = fsm->DSPScheduler[pos].Right[1] + (L_PEL2PEL)*cos(fsm->DSPScheduler[pos].RYaw*D2R);
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];
            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];
            task.LYaw = fsm->DSPScheduler[pos].RYaw;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.LRoll = 0.;
            task.LPitch = 0.;
            task.HTime = 0.0;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_LEFT;
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

        }
    }
    if(_walkingstopmode == SPREAD_STOP_WALKING)
    {
            fsm->DSPScheduler.pop_back();
            printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
    }

    foot = FOOT_LEFT;
    return foot;
}
// --------------------------------------------------------------------------------------------- //
int ccwrot_foot_print(int _walkingstopmode,int _walking_num,double _rotangle)
{
    printf("+++++++++ccwrot_foot_print+++++++++++\n");
    DSPTask task;
    int num = _walking_num;
    double angle = _rotangle;
    int foot = FOOT_LEFT;//FootSelector(DIR_CCW);
    for(int i=0; i<num; i++){
        foot = FOOT_LEFT;
        int pos = fsm->DSPScheduler.size()-1;

        double th_l = (fsm->DSPScheduler[pos].LYaw+fsm->DSPScheduler[pos].RYaw)/2.*D2R;
//        double lx = fsm->DSPScheduler[pos].Left[0];
//        double ly = fsm->DSPScheduler[pos].Left[1];
        double rx = fsm->DSPScheduler[pos].Right[0];
        double ry = fsm->DSPScheduler[pos].Right[1];
        if(i ==0)
        {
//            lx = fsm->DSPScheduler[pos].Right[0] - L_PEL2PEL*sin(th_l);
//            ly = fsm->DSPScheduler[pos].Right[1] + L_PEL2PEL*cos(th_l);
        }
        double px = rx - L_PEL2PEL/2.*sin(th_l) - L_FOOT/2.*cos(th_l);
        double py = ry + L_PEL2PEL/2.*cos(th_l) - L_FOOT/2.*sin(th_l);
        double X1 = cos(angle*D2R)*(L_FOOT/2.)-sin(angle*D2R)*(L_PEL2PEL/2.);
        double Y1 = sin(angle*D2R)*(L_FOOT/2.)+cos(angle*D2R)*(L_PEL2PEL/2.);

        double X = cos(th_l)*X1 - sin(th_l)*Y1 + px;
        double Y = sin(th_l)*X1 + cos(th_l)*Y1 + py;

        if(foot == FOOT_LEFT){
            task.Left[0] = X;
            task.Left[1] = Y;
            task.Left[2] = fsm->DSPScheduler[pos].Right[2];

            task.Right[0] = fsm->DSPScheduler[pos].Right[0];
            task.Right[1] = fsm->DSPScheduler[pos].Right[1];
            task.Right[2] = fsm->DSPScheduler[pos].Right[2];

            task.LYaw = fsm->DSPScheduler[pos].LYaw + angle;
            task.RYaw = fsm->DSPScheduler[pos].RYaw;
            task.HTime = DSPHoldTime;
            task.RRoll = fsm->DSPScheduler[pos].RRoll;
            task.RPitch = fsm->DSPScheduler[pos].RPitch;
            task.LRoll = 0.;
            task.LPitch = 0.;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch);

            pos = fsm->DSPScheduler.size()-1;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0] ;
            task.Left[1] = fsm->DSPScheduler[pos].Left[1]  ;
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];

            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + L_PEL2PEL*sin(fsm->DSPScheduler[pos].LYaw*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - L_PEL2PEL*cos(fsm->DSPScheduler[pos].LYaw*D2R);
            if(i == num-1)
            {
                task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL-L_PEL2PEL_OFFSET)*sin(fsm->DSPScheduler[pos].LYaw*D2R);
                task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL-L_PEL2PEL_OFFSET)*cos(fsm->DSPScheduler[pos].LYaw*D2R);
            }
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];
            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.RYaw = fsm->DSPScheduler[pos].LYaw;
            task.RRoll = 0.;
            task.RPitch = 0.;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_RIGHT;
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);

        }else{
            // already stretched
            cout << "stretched" << endl;
            task.Left[0] = fsm->DSPScheduler[pos].Left[0];
            task.Left[1] = fsm->DSPScheduler[pos].Left[1];
            task.Left[2] = fsm->DSPScheduler[pos].Left[2];
            task.Right[0] = fsm->DSPScheduler[pos].Left[0] + (L_PEL2PEL)*sin(fsm->DSPScheduler[pos].LYaw*D2R);
            task.Right[1] = fsm->DSPScheduler[pos].Left[1] - (L_PEL2PEL)*cos(fsm->DSPScheduler[pos].LYaw*D2R);
            task.Right[2] = fsm->DSPScheduler[pos].Left[2];
            task.LYaw = fsm->DSPScheduler[pos].LYaw;
            task.RYaw = fsm->DSPScheduler[pos].LYaw;
            task.RRoll = 0.;
            task.RPitch = 0.;
            task.LRoll = fsm->DSPScheduler[pos].LRoll;
            task.LPitch = fsm->DSPScheduler[pos].LPitch;
            task.HTime = 0.0;
            task.COMz[0] = InitialComz;
            task.COMz[1] = InitialComz;
            task.COMz[2] = InitialComz;
            fsm->DSPScheduler.push_back(task);
            foot = FOOT_RIGHT;
            printf("%dth LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",i+2,task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
        }
    }
    if(_walkingstopmode == SPREAD_STOP_WALKING){
        fsm->DSPScheduler.pop_back();
        printf("last LF=(%f,%f,%f),(%f,%f,%f),RF=(%f,%f,%f),(%f,%f,%f)\n,X=%f,Y=%f\n",task.Left[0],task.Left[1],task.Left[2],task.LYaw,task.LRoll,task.LPitch,task.Right[0],task.Right[1],task.Right[2],task.RYaw,task.RRoll,task.RPitch,(task.Right[0]+task.Left[0])/2,(task.Right[1]+task.Left[1])/2.);
    }

    foot = FOOT_RIGHT;
    return foot;

}
// --------------------------------------------------------------------------------------------- //
void get_WBIK_Q_from_RefAngleCurrent()
{
    joint->RefreshToCurrentReference();
    for(int i=RHY; i<=LAR; i++) {
       WBIK_Q[i+7] = joint->Joints[i]->RefAngleCurrent*D2R;
    }
//    WBIK_Q[idWST] = joint->GetJointRefAngle(WST)*D2R;
    WBIK_Q[idWST] = joint->GetJointRefAngle(WST)*D2R - yaw_angle_last*D2R;
    WBIK_Q[idRSP] = joint->GetJointRefAngle(RSP)*D2R;
    WBIK_Q[idRSR] = (joint->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
    WBIK_Q[idRSY] = joint->GetJointRefAngle(RSY)*D2R;
    WBIK_Q[idREB] = (joint->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
    WBIK_Q[idRWY] = joint->GetJointRefAngle(RWY)*D2R;
    WBIK_Q[idRWP] = joint->GetJointRefAngle(RWP)*D2R;
    WBIK_Q[idRWY2] = joint->GetJointRefAngle(RWY2)*D2R;

    WBIK_Q[idLSP] = joint->GetJointRefAngle(LSP)*D2R;
    WBIK_Q[idLSR] = (joint->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
    WBIK_Q[idLSY] = joint->GetJointRefAngle(LSY)*D2R;
    WBIK_Q[idLEB] = (joint->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
    WBIK_Q[idLWY] = joint->GetJointRefAngle(LWY)*D2R;
    WBIK_Q[idLWP] = joint->GetJointRefAngle(LWP)*D2R;
    WBIK_Q[idLWY2] = joint->GetJointRefAngle(LWY2)*D2R;
//    printf("RHY = %f,RHR = %f,RHP = %f,RKN = %f,RAP = %f, RAR = %f\n",WBIK_Q[idRHY]*R2D,WBIK_Q[idRHR]*R2D,WBIK_Q[idRHP]*R2D,WBIK_Q[idRKN]*R2D,WBIK_Q[idRAP]*R2D,WBIK_Q[idRAR]*R2D);

//    printf("RSP = %f,RSR = %f,RSY = %f,REB = %f,RWY = %f, RWP = %f\n",WBIK_Q[idRSP]*R2D,WBIK_Q[idRSR]*R2D,WBIK_Q[idRSY]*R2D,WBIK_Q[idREB]*R2D,WBIK_Q[idRWY]*R2D,WBIK_Q[idRWP]*R2D);
    // PELVIS Orientation Reset
    Qub[idRSR] = WBIK_Q[idRSR];// + OFFSET_RSR*D2R;//= 10.*D2R;//joint->Joints[RSR]->RefAngleCurrent*D2R;
    Qub[idRSP] = WBIK_Q[idRSP];//= 40.*D2R;//joint->Joints[RSP]->RefAngleCurrent*D2R;
    Qub[idRSY] = WBIK_Q[idRSY];//= 0;//joint->Joints[RSY]->RefAngleCurrent*D2R;
    Qub[idREB] = WBIK_Q[idREB];// + OFFSET_ELB*D2R;//= -10*D2R;//-130.*D2R;//joint->Joints[REB]->RefAngleCurrent*D2R;
    Qub[idRWY] = WBIK_Q[idRWY];//= 0;//joint->Joints[RWY]->RefAngleCurrent*D2R;
    Qub[idRWP] = WBIK_Q[idRWP];//= 20.*D2R;//joint->Joints[RWP]->RefAngleCurrent*D2R;
    Qub[idRWY] = WBIK_Q[idRWY];
    Qub[idRWY2] = WBIK_Q[idRWY2];

    Qub[idLSR] = WBIK_Q[idLSR];// + OFFSET_LSR*D2R;//= -10.*D2R;//joint->Joints[LSR]->RefAngleCurrent*D2R;
    Qub[idLSP] = WBIK_Q[idLSP];//= 40.*D2R;//joint->Joints[LSP]->RefAngleCurrent*D2R;
    Qub[idLSY] = WBIK_Q[idLSY];//= 0;//joint->Joints[LSY]->RefAngleCurrent*D2R;
    Qub[idLEB] = WBIK_Q[idLEB];// + OFFSET_ELB*D2R;//= -10*D2R;//-130.*D2R;//joint->Joints[LEB]->RefAngleCurrent*D2R;
    Qub[idLWY] = WBIK_Q[idLWY];//= 0;//joint->Joints[LWY]->RefAngleCurrent*D2R;
    Qub[idLWP] = WBIK_Q[idLWP];//= 20.*D2R;//joint->Joints[LWP]->RefAngleCurrent*D2R;
    Qub[idLWY] = WBIK_Q[idLWY];
    Qub[idLWY2] = WBIK_Q[idLWY2];

    Qub[idWST] = WBIK_Q[idWST];//= 0;//-180*D2R;//joint->Joints[WST]->RefAngleCurrent*D2R;

}

// --------------------------------------------------------------------------------------------- //
void Walking_initialize()
{

    des_pRF_3x1[0] =0;
    des_pRF_3x1[1] =0;
    des_pRF_3x1[2] =0;

    des_pLF_3x1[0] =0;
    des_pLF_3x1[1] =0;
    des_pLF_3x1[2] =0;


    X_ZMP = 0;
    Y_ZMP = 0;
    X_ZMP_Local = 0;
    Y_ZMP_Local = 0;
    X_ZMP_Global = 0;
    Y_ZMP_Global = 0;
    X_ZMP_n = 0;
    Y_ZMP_n = 0;
    X_ZMP_LPF = 0;
    Y_ZMP_LPF = 0;
    CNT_final_gain_DSP_ZMP_CON = 0;
    CNT_final_gain_SSP_ZMP_CON = 0;
    HUBO2TorsoInitConRoll(0.0, 0.0, 0);
    HUBO2TorsoInitConPitch(0.0, 0.0, 0);
    get_WBIK_Q_from_RefAngleCurrent();

    double PelYaw,PelRoll,PelPitch,temp1_qPel_4x1[4],temp2_qPel_4x1[4],temp3_qPel_4x1[4],temp4_qPel_4x1[4],temp5_qPel_4x1[4];
    PelYaw = -yaw_angle_last*D2R;
    PelRoll = 0*D2R;
    PelPitch = 0*D2R;

    qtRZ(PelYaw, temp1_qPel_4x1);
    qtRX(PelRoll, temp2_qPel_4x1);
    qtRY(PelPitch, temp3_qPel_4x1);

    QTcross(temp1_qPel_4x1,temp2_qPel_4x1,temp4_qPel_4x1);
    QTcross(temp4_qPel_4x1,temp3_qPel_4x1,temp5_qPel_4x1);

    WBIK_Q[idQ0] = temp5_qPel_4x1[0];//1;
    WBIK_Q[idQ1] = temp5_qPel_4x1[1];//0;
    WBIK_Q[idQ2] = temp5_qPel_4x1[2];//0;
    WBIK_Q[idQ3] = temp5_qPel_4x1[3];//0;

//    WBIK_Q[idQ0] = 1;
//    WBIK_Q[idQ1] = 0;
//    WBIK_Q[idQ2] = 0;
//    WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    printf("FK3 com = %f,%f,%f,PEL = %f,%f,%f,RF = %f,%f,%f,LF = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);
    init_WBIK_pCOM[0] = FK_pCOM_3x1[0];
    init_WBIK_pCOM[1] = FK_pCOM_3x1[1];
    init_WBIK_pCOM[2] = FK_pCOM_3x1[2];

    init_WBIK_Q[0] = WBIK_Q[idX];
    init_WBIK_Q[1] = WBIK_Q[idY];
    init_WBIK_Q[2] = WBIK_Q[idZ];
    printf("init_WBIK_pCOM : (%f,%f,%f),init_WBIK_Q : (%f,%f,%f)\n",init_WBIK_pCOM[0],init_WBIK_pCOM[1],init_WBIK_pCOM[2],init_WBIK_Q[0],init_WBIK_Q[1],init_WBIK_Q[2]);
    //printf("com z = %f,pel z = %f\n",FK_pCOM_3x1[2],WBIK_Q[idZ]);
    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2] + fsm->AddComInfos[0][2];//0;
    printf("WalkReadyCOM[2] = %f,AddComInfos[0][2] = %f\n",userData->WalkReadyCOM[2],fsm->AddComInfos[0][2]);

    printf("========================\n");
    printf("PEL : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],WBIK_Q[idQ0],WBIK_Q[idQ1],WBIK_Q[idQ2],WBIK_Q[idQ3]);
    printf("RLEG : %f,%f,%f,%f,%f,%f\n",WBIK_Q[idRHY],WBIK_Q[idRHR],WBIK_Q[idRHP],WBIK_Q[idRKN],WBIK_Q[idRAP],WBIK_Q[idRAR]);
    printf("LLEG : %f,%f,%f,%f,%f,%f\n",WBIK_Q[idLHY],WBIK_Q[idLHR],WBIK_Q[idLHP],WBIK_Q[idLKN],WBIK_Q[idLAP],WBIK_Q[idLAR]);

    printf("RARM : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idRSP],WBIK_Q[idRSR],WBIK_Q[idRSY],WBIK_Q[idREB],WBIK_Q[idRWP],WBIK_Q[idRWY],WBIK_Q[idRWY2]);
    printf("LARM : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idLSP],WBIK_Q[idLSR],WBIK_Q[idLSY],WBIK_Q[idLEB],WBIK_Q[idLWP],WBIK_Q[idLWY],WBIK_Q[idLWY2]);
    printf("========================\n");
    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,PEL = %f,%f,%f,\nRF = %f,%f,%f,LF = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);
    //ine_drc_hubo.IK_LowerBody_Global(Qub,des_pCOM_3x1, temp_4x1, FK_pRFoot_3x1, FK_qRFoot_4x1, FK_pLFoot_3x1, FK_qLFoot_4x1,WBIK_Q);

    initialPos.Left[0] = FK_pLFoot_3x1[0];
    initialPos.Left[1] = FK_pLFoot_3x1[1];
    initialPos.Left[2] = FK_pLFoot_3x1[2];

    double temp_yaw,temp_pitch,temp_roll;
//    QT2YPR(FK_qLFoot_4x1,initialPos.LYaw,initialPos.LPitch,initialPos.LRoll);
    QT2YPR(FK_qLFoot_4x1,temp_yaw,temp_pitch,temp_roll);
    initialPos.LYaw = temp_yaw*R2D;
    initialPos.LPitch = temp_pitch*R2D;
    initialPos.LRoll = temp_roll*R2D;

    initialPos.Right[0] = FK_pRFoot_3x1[0];
    initialPos.Right[1] = FK_pRFoot_3x1[1];
    initialPos.Right[2] = FK_pRFoot_3x1[2];

//    QT2YPR(FK_qRFoot_4x1,initialPos.RYaw,initialPos.RPitch,initialPos.RRoll);
    QT2YPR(FK_qRFoot_4x1,temp_yaw,temp_pitch,temp_roll);
    initialPos.RYaw = temp_yaw*R2D;
    initialPos.RPitch = temp_pitch*R2D;
    initialPos.RRoll = temp_roll*R2D;

    printf("qRF =%f,%f,%f, qLF = %f,%f,%f\n",initialPos.RYaw,initialPos.RRoll,initialPos.RPitch,initialPos.LYaw,initialPos.LRoll,initialPos.LPitch);

    double Global_yaw,Global_pitch,Global_roll,tempQT1[4],tempQT2[4],tempQT3[4];
    Global_yaw      = (initialPos.RYaw   + initialPos.LYaw)/2.;
    Global_pitch    = (initialPos.RPitch + initialPos.LPitch)/2.;
    Global_roll     = (initialPos.RRoll  + initialPos.LRoll)/2.;

    qtRZ(Global_yaw*D2R, tempQT1);
    qtRY(Global_pitch*D2R, tempQT2);
    QTcross(tempQT1,tempQT2,tempQT3);
    qtRX(Global_roll*D2R, tempQT1);
    QTcross(tempQT1,tempQT3,Global_qFoot_4x1);

    double LeftYaw,LeftRoll,LeftPitch;
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];

    LeftYaw = initialPos.RYaw*D2R;
    LeftRoll = initialPos.RRoll*D2R;
    LeftPitch = initialPos.RPitch*D2R;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

    printf("QT = %f, %f, %f, %f\n",FK_qRFoot_4x1[0],FK_qRFoot_4x1[1],FK_qRFoot_4x1[2],FK_qRFoot_4x1[3]);
    printf("QT = %f, %f, %f, %f\n",temp5des_qLF_4x1[0],temp5des_qLF_4x1[1],temp5des_qLF_4x1[2],temp5des_qLF_4x1[3]);

    initialPos.HTime = 0.0;
    //----comz
    initialPos.ZMP[0] = FK_pCOM_3x1[0];
    initialPos.ZMP[1] = FK_pCOM_3x1[1];
//    printf("")
    printf("FK_pRFoot_3x1[0] = %f, %f, %f,FK_pLFoot_3x1[0] = %f, %f, %f\n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    //initialPos.COMz[0] = FK_pCOM_3x1[2] - userData->WalkReadyCOM[2];

    fsm->SetInitialFootPlace(initialPos);
    checkfsm->SetInitialFootPlace(initialPos);

    GLOBAL_Z_RF = FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    GLOBAL_Z_LF = FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];

    GLOBAL_Z_RF_last = GLOBAL_Z_RF;
    GLOBAL_Z_LF_last = GLOBAL_Z_LF;

//    AngularMomentumComp(0,0,0,0,0);
//    yaw_angle = 0;

}
// --------------------------------------------------------------------------------------------- //
void Walking_initialize_1st()
{
    X_ZMP = 0;
    Y_ZMP = 0;
    X_ZMP_LPF = 0;
    Y_ZMP_LPF = 0;
    X_ZMP_n = 0;
    Y_ZMP_n = 0;
    CNT_final_gain_DSP_ZMP_CON = 0;
    yaw_angle_last = 0;
    get_WBIK_Q_from_RefAngleCurrent();

//    WBIK_Q[idQ0] = des_qPEL_4x1[0];//1;
//    WBIK_Q[idQ1] = des_qPEL_4x1[1];//0;
//    WBIK_Q[idQ2] = des_qPEL_4x1[2];//0;
//    WBIK_Q[idQ3] = des_qPEL_4x1[3];//0;
    WBIK_Q[idQ0] = 1;
    WBIK_Q[idQ1] = 0;
    WBIK_Q[idQ2] = 0;
    WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);
    init_WBIK_pCOM[0] = FK_pCOM_3x1[0];
    init_WBIK_pCOM[1] = FK_pCOM_3x1[1];
    init_WBIK_pCOM[2] = FK_pCOM_3x1[2];

    init_WBIK_Q[0] = WBIK_Q[idX];
    init_WBIK_Q[1] = WBIK_Q[idY];
    init_WBIK_Q[2] = WBIK_Q[idZ];
    //printf("com z = %f,pel z = %f\n",FK_pCOM_3x1[2],WBIK_Q[idZ]);
    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    //printf("WBIK_Q[idZ] = %f,FK_pCOM_3x1[2] = %f,AddComInfos[0][2] = %f\n",WBIK_Q[idZ],FK_pCOM_3x1[2],fsm->AddComInfos[0][2]);
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;
    //printf("WBIK_Q[idZ] = %f,FK_pCOM_3x1[2] = %f,AddComInfos[0][2] = %f\n",WBIK_Q[idZ],FK_pCOM_3x1[2],fsm->AddComInfos[0][2]);

    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);
    //ine_drc_hubo.IK_LowerBody_Global(Qub,des_pCOM_3x1, temp_4x1, FK_pRFoot_3x1, FK_qRFoot_4x1, FK_pLFoot_3x1, FK_qLFoot_4x1,WBIK_Q);

    initialPos.Left[0] = 0;//FK_pLFoot_3x1[0];
    initialPos.Left[1] = 0.105;//FK_pLFoot_3x1[1];
    initialPos.Left[2] = 0;//FK_pLFoot_3x1[2];
//    QT2EULER(FK_qLFoot_4x1,initialPos.LRoll,initialPos.LPitch,initialPos.LYaw);
    initialPos.LRoll = 0;//
    initialPos.LPitch = 0;//
    initialPos.LYaw = 0;

    initialPos.Right[0] = 0;//FK_pRFoot_3x1[0];
    initialPos.Right[1] = -0.105;//FK_pRFoot_3x1[1];
    initialPos.Right[2] = 0;//FK_pRFoot_3x1[2];
//    QT2EULER(FK_qRFoot_4x1,initialPos.RRoll,initialPos.RPitch,initialPos.RYaw);
    initialPos.RRoll = 0;//
    initialPos.RPitch = 0;//
    initialPos.RYaw = 0;

    double LeftYaw,LeftRoll,LeftPitch;
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];

    LeftYaw = initialPos.LYaw*D2R;
    LeftRoll = initialPos.LRoll*D2R;
    LeftPitch = initialPos.LPitch*D2R;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp2des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp4des_qLF_4x1,temp5des_qLF_4x1);

    printf("QT = %f, %f, %f, %f\n",FK_qLFoot_4x1[0],FK_qLFoot_4x1[1],FK_qLFoot_4x1[2],FK_qLFoot_4x1[3]);
    printf("QT = %f, %f, %f, %f\n",temp5des_qLF_4x1[0],temp5des_qLF_4x1[1],temp5des_qLF_4x1[2],temp5des_qLF_4x1[3]);

    initialPos.HTime = 0.0;
    //----comz
    initialPos.ZMP[0] = 0;//FK_pCOM_3x1[0];
    initialPos.ZMP[1] = 0;//FK_pCOM_3x1[1];
//    printf("")
    printf("FK_pRFoot_3x1[0] = %f, FK_pRFoot_3x1[1] = %f, FK_pRFoot_3x1[2] = %f,FK_pLFoot_3x1[0] = %f, FK_pLFoot_3x1[1] = %f, FK_pLFoot_3x1[2] = %f\n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    //initialPos.COMz[0] = FK_pCOM_3x1[2] - userData->WalkReadyCOM[2];

    fsm->SetInitialFootPlace(initialPos);
    checkfsm->SetInitialFootPlace(initialPos);

    GLOBAL_Z_RF = 0;//FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    GLOBAL_Z_LF = 0;//FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];

    GLOBAL_Z_RF_last = GLOBAL_Z_RF;
    GLOBAL_Z_LF_last = GLOBAL_Z_LF;

    AngularMomentumComp(0,0,0,0);
    yaw_angle = 0;

}
// --------------------------------------------------------------------------------------------- //
void fsm_clear()
{
    fsm->CompData->JointHandler[JRAR]->StopAndEraseAll();
    fsm->CompData->JointHandler[JLAR]->StopAndEraseAll();
    fsm->CompData->JointHandler[JRAP]->StopAndEraseAll();
    fsm->CompData->JointHandler[JLAP]->StopAndEraseAll();
    fsm->CompData->JointHandler[JRHR]->StopAndEraseAll();
    fsm->CompData->JointHandler[JLHR]->StopAndEraseAll();
    fsm->LeftFoot->PosHandler[0]->StopAndEraseAll();
    fsm->LeftFoot->PosHandler[1]->StopAndEraseAll();
    fsm->LeftFoot->PosHandler[2]->StopAndEraseAll();
    fsm->LeftFoot->YawHandler->StopAndEraseAll();
    fsm->LeftFoot->PitchHandler->StopAndEraseAll();
    fsm->LeftFoot->RollHandler->StopAndEraseAll();

    fsm->RightFoot->PosHandler[0]->StopAndEraseAll();
    fsm->RightFoot->PosHandler[1]->StopAndEraseAll();
    fsm->RightFoot->PosHandler[2]->StopAndEraseAll();
    fsm->RightFoot->YawHandler->StopAndEraseAll();
    fsm->RightFoot->PitchHandler->StopAndEraseAll();
    fsm->RightFoot->RollHandler->StopAndEraseAll();

    fsm->ZMPPos->ZMPHandler[0]->StopAndEraseAll();
    fsm->ZMPPos->ZMPHandler[1]->StopAndEraseAll();

    fsm->LeftInfos.clear();
    fsm->RightInfos.clear();

    fsm->AddJointInfos.clear();
    fsm->AddRightFootInfos.clear();
    fsm->AddLeftFootInfos.clear();

    //-------------com
    fsm->AddComInfos.clear();

    //-------------virtual com
    fsm->VirtualComInfos.clear();
    fsm->VirtualComVelInfos.clear();
    fsm->VirtualComAccInfos.clear();

    //-------------foot vel
    fsm->LeftInfosVel.clear();
    fsm->RightInfosVel.clear();

    fsm->FootUpInfos.clear();
    fsm->StateInfos.clear();
    fsm->STimeInfos.clear();
    fsm->ITimeInfos.clear();

    fsm->ClearAll();

    fsm->ZMPInfos.clear();

    doubles tempR = fsm->RightFoot->Pos;

    tempR.push_back(fsm->RightFoot->Yaw);
    tempR.push_back(fsm->RightFoot->Roll);
    tempR.push_back(fsm->RightFoot->Pitch);
    fsm->RightInfos.push_back(tempR);

    doubles tempL = fsm->LeftFoot->Pos;
    tempL.push_back(fsm->LeftFoot->Yaw);
    tempL.push_back(fsm->LeftFoot->Roll);
    tempL.push_back(fsm->LeftFoot->Pitch);

    fsm->LeftInfos.push_back(tempL);

    fsm->ZMPInfos.push_back(fsm->ZMPPos->ZMP);

    fsm->AddJointInfos.push_back(fsm->CompData->AddJoint);
    fsm->AddRightFootInfos.push_back(fsm->CompData->AddRightFoot);
    fsm->AddLeftFootInfos.push_back(fsm->CompData->AddLeftFoot);

    //-------------add com
    fsm->AddComInfos.push_back(fsm->CompData->AddCOM);
    fsm->VirtualComInfos.push_back(fsm->CompData->AddCOM);
    fsm->VirtualComVelInfos.push_back(fsm->CompData->AddCOMVel);
    fsm->VirtualComAccInfos.push_back(fsm->CompData->AddCOMAcc);

    //---foot vel trajectory
    fsm->LeftInfosVel.push_back(fsm->LeftFoot->Vel);
    fsm->RightInfosVel.push_back(fsm->RightFoot->Vel);

    ints tempState;
    tempState.clear();
    tempState.push_back(fsm->CurrentState);
    tempState.push_back(fsm->IsUpward);
    fsm->StateInfos.push_back(tempState);
}
// --------------------------------------------------------------------------------------------- //
void checkfsm_clear()
{
    checkfsm->LeftFoot->PosHandler[0]->StopAndEraseAll();
    checkfsm->LeftFoot->PosHandler[1]->StopAndEraseAll();
    checkfsm->LeftFoot->PosHandler[2]->StopAndEraseAll();
    checkfsm->LeftFoot->YawHandler->StopAndEraseAll();
    checkfsm->LeftFoot->PitchHandler->StopAndEraseAll();
    checkfsm->LeftFoot->RollHandler->StopAndEraseAll();

    checkfsm->RightFoot->PosHandler[0]->StopAndEraseAll();
    checkfsm->RightFoot->PosHandler[1]->StopAndEraseAll();
    checkfsm->RightFoot->PosHandler[2]->StopAndEraseAll();
    checkfsm->RightFoot->YawHandler->StopAndEraseAll();
    checkfsm->RightFoot->PitchHandler->StopAndEraseAll();
    checkfsm->RightFoot->RollHandler->StopAndEraseAll();

    checkfsm->CompData->COMHandler[0]->StopAndEraseAll();
    checkfsm->CompData->COMHandler[1]->StopAndEraseAll();
    checkfsm->CompData->COMHandler[2]->StopAndEraseAll();

    checkfsm->ZMPPos->ZMPHandler[0]->StopAndEraseAll();
    checkfsm->ZMPPos->ZMPHandler[1]->StopAndEraseAll();

    checkfsm->LeftInfos.clear();
    checkfsm->RightInfos.clear();


    checkfsm->AddJointInfos.clear();
    checkfsm->AddRightFootInfos.clear();
    checkfsm->AddLeftFootInfos.clear();

    //-------------com
    checkfsm->AddComInfos.clear();

    //-------------virtual com
    checkfsm->VirtualComInfos.clear();
    checkfsm->VirtualComVelInfos.clear();
    checkfsm->VirtualComAccInfos.clear();

    //-------------foot vel
    checkfsm->LeftInfosVel.clear();
    checkfsm->RightInfosVel.clear();

    checkfsm->FootUpInfos.clear();
    checkfsm->StateInfos.clear();
    checkfsm->STimeInfos.clear();
    checkfsm->ITimeInfos.clear();

    checkfsm->ClearAll();

    checkfsm->ZMPInfos.clear();

    doubles tempR = checkfsm->RightFoot->Pos;

    tempR.push_back(checkfsm->RightFoot->Yaw);
    tempR.push_back(checkfsm->RightFoot->Roll);
    tempR.push_back(checkfsm->RightFoot->Pitch);
    checkfsm->RightInfos.push_back(tempR);

    doubles tempL = checkfsm->LeftFoot->Pos;
    tempL.push_back(checkfsm->LeftFoot->Yaw);
    tempL.push_back(checkfsm->LeftFoot->Roll);
    tempL.push_back(checkfsm->LeftFoot->Pitch);

    checkfsm->LeftInfos.push_back(tempL);

    checkfsm->ZMPInfos.push_back(checkfsm->ZMPPos->ZMP);

    checkfsm->AddJointInfos.push_back(checkfsm->CompData->AddJoint);
    checkfsm->AddRightFootInfos.push_back(checkfsm->CompData->AddRightFoot);
    checkfsm->AddLeftFootInfos.push_back(checkfsm->CompData->AddLeftFoot);

    //-------------add com
    checkfsm->AddComInfos.push_back(checkfsm->CompData->AddCOM);
    checkfsm->VirtualComInfos.push_back(checkfsm->CompData->AddCOM);
    checkfsm->VirtualComVelInfos.push_back(checkfsm->CompData->AddCOMVel);
    checkfsm->VirtualComAccInfos.push_back(checkfsm->CompData->AddCOMAcc);

    //---foot vel trajectory
    checkfsm->LeftInfosVel.push_back(checkfsm->LeftFoot->Vel);
    checkfsm->RightInfosVel.push_back(checkfsm->RightFoot->Vel);

    ints tempState;
    tempState.clear();
    tempState.push_back(checkfsm->CurrentState);
    tempState.push_back(checkfsm->IsUpward);
    checkfsm->StateInfos.push_back(tempState);
}


// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal){
    switch(_signal)
    {
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
int RBInitialize(void){
    isTerminated = 0;

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME_REFERENCE, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Reference]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_REFERENCE)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Reference]";
            return false;
        }else{
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedREF == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Reference]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Reference]";
    // =========================================================================

    // Core Shared Memory Creation [Sensor]=====================================
    shmFD = shm_open(RBCORE_SHM_NAME_SENSOR, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Sensor]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_SENSOR)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Sensor]";
            return false;
        }else{
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ, MAP_SHARED, shmFD, 0);
            if(sharedSEN == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Sensor]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Sensor]";
    // =========================================================================

    // Core Shared Memory Creation [Command]====================================
    shmFD = shm_open(RBCORE_SHM_NAME_COMMAND, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Command]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_COMMAND)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Command]";
            return false;
        }else{
            sharedCMD = (pRBCORE_SHM_COMMAND)mmap(0, sizeof(RBCORE_SHM_COMMAND), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedCMD == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Command]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Command]";
    // =========================================================================

    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return -1;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedREF, sharedSEN, sharedCMD, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, "FreeWalking_FLAG", 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(3, &aCPU);
        if(rt_task_set_affinity(&rtFlagCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Flag real-time thread set affinity CPU failed..";
        }
                if(rt_task_start(&rtFlagCon, &RBFlagThread, NULL) == 0 ){
                        FILE_LOG(logSUCCESS) << "Flag real-time thread start = OK";
                }else{
                        FILE_LOG(logERROR) << "Flag real-time thread start = FAIL";
                        return -1;
                }
        }else{
                FILE_LOG(logERROR) <<  "Fail to create Flag real-time thread";
                return -1;
        }

    if(rt_task_create(&rtTaskCon, "FreeWalking_TASK", 0, 90, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(2, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Task real-time thread set affinity CPU failed..";
        }
                if(rt_task_start(&rtTaskCon, &RBTaskThread, NULL) == 0 ){
                        FILE_LOG(logSUCCESS) << "Task real-time thread start = OK";
                }else{
                        FILE_LOG(logERROR) << "Task real-time thread start = FAIL";
                        return -1;
                }
        }else{
                FILE_LOG(logERROR) << "Fail to create Task real-time thread";
                return -1;
        }
    // =========================================================================

    return 0;
}
// --------------------------------------------------------------------------------------------- //
void Preview()
{
    static double pv_state[2][3] = {{0.0}}, pv_state_old[2][3] = {{0.0}};
    static double pv_ZMP[2] = {0.f,};
    static double pv_Err[2] = {0.f,};
    static double pv_U[2] = {0.f,};
    double temp_sum[2]={0.,0,};

    if(fsm->StateInfos[0][0] != STATE_FINISHED)
    {
//        printf("%d =========== \n ",pv_Index);
//        if(pv_Index == 0)
//        {
//            temp_X = GLOBAL_X_LIPM_n - 0.001*Del_PC_X_DSP_XZMP_CON - I_ZMP_CON_X;
//            temp_Y = U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0- 0.001*Del_PC_Y_DSP_YZMP_CON - I_ZMP_CON_Y;
//            pv_state_old[0][0] = temp_X*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) - temp_Y*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            pv_state_old[1][0] = temp_X*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + temp_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            Del_PC_X_DSP_XZMP_CON = 0;
//            Del_PC_Y_DSP_YZMP_CON = 0;
//            kirkZMPCon_XP2(0,0,0);
//            kirkZMPCon_YP2(0,0,0);
//            I_ZMP_CON_X =0;
//            I_ZMP_CON_Y =0;
//            printf("reset =========== \n ");
//         }
        for(int i=0;i<=1;i++)
        {
            //=========================
            pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];
            pv_Err[i] = pv_Err[i] + pv_ZMP[i] - fsm->ZMPInfos[0][i];

            temp_sum[i] = 0.0f;
            for(pv_time_Index=0;pv_time_Index<299;pv_time_Index++)
            {
                temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(fsm->ZMPInfos[pv_time_Index][i]);
            }

            pv_U[i] = -pv_Gi[1]*pv_Err[i] - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

            //temp_sum[i] =0;
            //pv_Err[i] =0;

            pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
            pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
            pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];
        }

        GLOBAL_Y_LIPM = pv_state[1][0];
        GLOBAL_X_LIPM = pv_state[0][0];

        GLOBAL_Y_LIPM_d = pv_state[1][1];
        GLOBAL_X_LIPM_d = pv_state[0][1];

        GLOBAL_X_RF = fsm->RightInfos[0][0];
        GLOBAL_X_LF = fsm->LeftInfos[0][0];

        GLOBAL_Y_RF = fsm->RightInfos[0][1];
        GLOBAL_Y_LF = fsm->LeftInfos[0][1];

        GLOBAL_ZMP_REF_X = fsm->ZMPInfos[0][0];
        GLOBAL_ZMP_REF_Y = fsm->ZMPInfos[0][1];

        GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_ZMP_REF_Y_n = -GLOBAL_ZMP_REF_X*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_LIPM_n =  GLOBAL_X_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LIPM_n = -GLOBAL_X_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_LIPM_d_n =  GLOBAL_X_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LIPM_d_n = -GLOBAL_X_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_RF_n =  GLOBAL_X_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_RF_n = -GLOBAL_X_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_LF_n =  GLOBAL_X_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LF_n = -GLOBAL_X_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_Z_RF = fsm->RightInfos[0][2];
//        GLOBAL_Z_LF = fsm->LeftInfos[0][2];

        //GLOBAL_Z_LIPM = min(GLOBAL_Z_RF,GLOBAL_Z_LF);

        pv_Index++ ;
    }
}
//-----------------------------------------------------------------------------------------------//
void WalkingTimer()
{

    static double pre_state = -1.;

    static double temp_Z_RF[3]={0.,},temp_Z_LF[3]={0.,};
    static double temp_G_DSP_X[3]={1.,};
    static double temp_distance_foot = 0.2;

//    static double IMU_NULL_FLAG = 0;
    static double EarlyLanding_FT = 200.;
    if(fsm->walking_mode==TERRAIN_WALKING_ONE_STEP||fsm->walking_mode==LADDER_WALKING)
        EarlyLanding_FT = 90.;
    double FTcnt=0,FTcnt2=0,mean_FT=0,sum_FT=0,mean_FT2=0,sum_FT2=0;
//    static double AnkleRollIMU_Pgain = 1.3;//1.3;//1.9;
//    static double AnkleRollIMU_Dgain = 0;
    static double AnkleRollIMU_Igain = 0;//0.005;
    static double err_sum_roll = 0,err_sum_pitch = 0;
//    static double GainGyro =0;
//    static int CNT_IMU_STABLE =0;
    double info_footupheight,info_footuptime;
//    double added_mass = 3.;

//    static int add_mass_FLAG = 0;

    info_footupheight = fsm->FootUpInfos[0][0];
    info_footuptime = fsm->FootUpInfos[0][1];

    if(fsm_state_timer[fsm->StateInfos[0][0]]==0)
    {
        for(int j=0;j<=STATE_FINISHED;j++)
        {
            if(j!=fsm->StateInfos[0][0]) fsm_state_timer[j]=0;
        }
    }
    AngleRoll_last = AngleRoll;
    LPF_AngleRoll_last = LPF_AngleRoll;

    double RotZ[9],TorsoOri[3],TorsoOri_n[3];
    TorsoOri[0] = sharedSEN->FOG.Roll*D2R;
    TorsoOri[1] = sharedSEN->FOG.Pitch*D2R;
    TorsoOri[2] = 0.;
    RZ(-yaw_angle*D2R,RotZ);
    mult_mv(RotZ,3,3,TorsoOri,TorsoOri_n);
    AngleRoll = TorsoOri_n[0]; // Robot Frame
    AnglePitch = TorsoOri_n[1]; // Robot Frame

    AngleVel = (AngleRoll - AngleRoll_last)/DEL_T;

    LPF_AngleRoll =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_AngleRoll + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*AngleRoll;
    LPF_AngleVel = (LPF_AngleRoll - LPF_AngleRoll_last)/DEL_T;

    err_sum_roll += (0 - AngleRoll);
    if(err_sum_roll > 7.0/AnkleRollIMU_Igain) err_sum_roll = 7./AnkleRollIMU_Igain;
    else if(err_sum_roll < -7.0/AnkleRollIMU_Igain) err_sum_roll = -7./AnkleRollIMU_Igain;

    err_sum_pitch += (0 - AnglePitch);
    if(err_sum_pitch > 7.0/AnkleRollIMU_Igain) err_sum_pitch = 7./AnkleRollIMU_Igain;
    else if(err_sum_pitch < -7.0/AnkleRollIMU_Igain) err_sum_pitch = -7./AnkleRollIMU_Igain;

    double tempAngle = AngleRoll*D2R;
    Estimated_GLOBAL_Y_LF = (GLOBAL_Y_LF - GLOBAL_Y_RF)*cos(tempAngle)-(GLOBAL_Z_LF - GLOBAL_Z_RF)*sin(tempAngle) + GLOBAL_Y_RF;
    Estimated_GLOBAL_Z_LF = (GLOBAL_Y_LF - GLOBAL_Y_RF)*sin(tempAngle)+(GLOBAL_Z_LF - GLOBAL_Z_RF)*cos(tempAngle) + GLOBAL_Z_RF;

    Estimated_GLOBAL_Y_RF = (GLOBAL_Y_RF - GLOBAL_Y_LF)*cos(tempAngle)-(GLOBAL_Z_RF - GLOBAL_Z_LF)*sin(tempAngle) + GLOBAL_Y_LF;
    Estimated_GLOBAL_Z_RF = (GLOBAL_Y_RF - GLOBAL_Y_LF)*sin(tempAngle)+(GLOBAL_Z_RF - GLOBAL_Z_LF)*cos(tempAngle) + GLOBAL_Z_LF;

    if((fsm->StateInfos[0][0]==STATE_SSP_RF)||(fsm->StateInfos[0][0]==STATE_SSP_LF)) SSP_FLAG = 1;
    else SSP_FLAG = 0;


//    printf("t = %f, dsptime = %f , G_DSP_Y = %f\n",fsm_state_timer[fsm->StateInfos[0][0]],fsm->STimeInfos[0],temp_G_DSP_X[0]);
    switch(fsm->StateInfos[0][0])
    {

    case STATE_DSP_INIT_RF:
    {
        FTcnt=FTcnt2=mean_FT=sum_FT=mean_FT2=sum_FT2=0;

        if(fsm_state_timer[fsm->StateInfos[0][0]]==0)
        {
            //printf(">> STATE_DSP_INIT_RF \n");
        }
        if((fsm_state_timer[fsm->StateInfos[0][0]]>fsm->TIME_DSP)&&(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->TIME_DSP + 2.0f))
                    Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],fsm->TIME_DSP,fsm->TIME_DSP + 2.0f,G_DSP_X_last,0.,0.,1.,temp_G_DSP_X);
        //printf("t = %f, dsptime = %f,fsm->dsptime = %f, fsm->ssptime = %f, G_DSP_Y = %f\n",fsm_state_timer[fsm->StateInfos[0][0]],fsm->TIME_DSP,fsm->TIME_DSP,fsm->TIME_SSP,temp_G_DSP_X[0]);
        G_DSP_X = G_DSP_Y = temp_G_DSP_X[0];
        GLOBAL_Z_LIPM_last = GLOBAL_Z_LIPM;
        GLOBAL_Z_RF_last2 = GLOBAL_Z_RF = fsm->RightInfos[0][2];
        GLOBAL_Z_LF_last2 = GLOBAL_Z_LF;

        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->TIME_DSP)
        {
//            GainGyro = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->TIME_DSP));
        }
        Kirk_Control();
        AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
        AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);
        AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
        double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];
//            des_pRF_3x1_n[0] =  des_pRF_3x1[0]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pRF_3x1[1]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pRF_3x1_n[1] = -des_pRF_3x1[0]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pRF_3x1[1]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            des_pLF_3x1_n[0] =  des_pLF_3x1[0]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pLF_3x1[1]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pLF_3x1_n[1] = -des_pLF_3x1[0]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pLF_3x1[1]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R, sharedSEN->FOG.Pitch*D2R, 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(yaw_angle*D2R) - sharedSEN->FOG.Pitch*D2R*sin(yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(yaw_angle*D2R) + sharedSEN->FOG.Pitch*D2R*cos(yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMUAccX[CIMU]*D2R, sharedData->IMUAccY[CIMU]*D2R, 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//                printf("BTW_FOOT_Angle_roll = %f, pitch = %f, yaw = %f\n",BTW_FOOT_Angle_roll*R2D,BTW_FOOT_Angle_pitch*R2D,BTW_FOOT_Angle_yaw*R2D);
        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
//        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
//        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
        LandingState = DSP;
        /*
        double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];
        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
        */
        /*
        HUBO2_ForceInitCon[RIGHT][Xdir] = -HUBO2_ForceInitCon_RF_X(0 , (sharedSEN->FT[RAFT].Fx-sharedSEN->FT[LAFT].Fx), 1);
        HUBO2_ForceInitCon[LEFT][Xdir]=-HUBO2_ForceInitCon[RIGHT][Xdir];
        HUBO2_ForceInitCon[RIGHT][Ydir] = -HUBO2_ForceInitCon_RF_Y(0 , sharedSEN->FT[RAFT].Fy, 1);
        //HUBO2_ForceInitCon[LEFT][Xdir]  = -HUBO2_ForceInitCon_LF_X(0 , sharedSEN->FT[LAFT].Fx, 1);
        HUBO2_ForceInitCon[LEFT][Ydir]  = -HUBO2_ForceInitCon_LF_Y(0 , sharedSEN->FT[LAFT].Fy, 1);

        //if(fabs(sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03)>30)
//            AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
//        else
        AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0002);
        AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0002);
        AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0011);
        */

        if((fsm_state_timer[fsm->StateInfos[0][0]]>=1.8)&&(fsm_state_timer[fsm->StateInfos[0][0]]<=2.7))
        {
            U_Gain = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]]-1.8)/(0.9-DEL_T)));
        }
        if(fsm_state_timer[fsm->StateInfos[0][0]]<=2.7)
        {
            U_Gain_DSP = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(2.7-DEL_T)));
        }
//        printf("U_Gain_DSP = %f\n",U_Gain_DSP);
//        printf("time = %f\n",fsm_state_timer[fsm->StateInfos[0][0]]);
        U3_Gain = U_Gain;

        ReactiveControl(1,1,1);
    }
        break;
    case STATE_DSP_INIT_LF:
    {
        FTcnt=FTcnt2=mean_FT=sum_FT=mean_FT2=sum_FT2=0;

        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->TIME_DSP)
        {
//            GainGyro = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->TIME_DSP));
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]]==0)
        {
            //printf(">> STATE_DSP_INIT_LF \n");
        }
        if((fsm_state_timer[fsm->StateInfos[0][0]]>fsm->TIME_DSP)&&(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->TIME_DSP + 2.0f))
                    Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],fsm->TIME_DSP,fsm->TIME_DSP + 2.0f,G_DSP_X_last,0.,0.,1.,temp_G_DSP_X);
        G_DSP_X = G_DSP_Y = temp_G_DSP_X[0];
        GLOBAL_Z_LIPM_last = GLOBAL_Z_LIPM;
        GLOBAL_Z_RF_last2 = GLOBAL_Z_RF;
        GLOBAL_Z_LF_last2 = GLOBAL_Z_LF = fsm->LeftInfos[0][2];
        Kirk_Control();
        AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
        AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);
        AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
        double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];
//            des_pRF_3x1_n[0] =  des_pRF_3x1[0]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pRF_3x1[1]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pRF_3x1_n[1] = -des_pRF_3x1[0]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pRF_3x1[1]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            des_pLF_3x1_n[0] =  des_pLF_3x1[0]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pLF_3x1[1]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pLF_3x1_n[1] = -des_pLF_3x1[0]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pLF_3x1[1]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R, sharedSEN->FOG.Pitch*D2R, 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(yaw_angle*D2R) - sharedSEN->FOG.Pitch*D2R*sin(yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(yaw_angle*D2R) + sharedSEN->FOG.Pitch*D2R*cos(yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMUAccX[CIMU]*D2R, sharedData->IMUAccY[CIMU]*D2R, 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//                printf("BTW_FOOT_Angle_roll = %f, pitch = %f, yaw = %f\n",BTW_FOOT_Angle_roll*R2D,BTW_FOOT_Angle_pitch*R2D,BTW_FOOT_Angle_yaw*R2D);
        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
//        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
//        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
//        if(add_mass_FLAG == 0)
//        {
//            if((fsm_state_timer[fsm->StateInfos[0][0]]>0.5)&&(fsm_state_timer[fsm->StateInfos[0][0]]<(3.-DEL_T)))
//            {

//                kine_drc_hubo4.m_RightWrist += added_mass/((3-0.5)/DEL_T);
//    //            kine_drc_hubo4.m_LeftWrist += 1.5/((3-0.5)/DEL_T);
//                printf("timer = %f,wrist = %f\n",fsm_state_timer[fsm->StateInfos[0][0]],kine_drc_hubo4.m_RightWrist);
//            }
//            add_mass_FLAG=1;
//        }
//        if(fsm_state_timer[fsm->StateInfos[0][0]]<=0.2)
//        {
//            U_Gain = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(0.2)));
//            U3_Gain = U_Gain;
//        }
        if((fsm_state_timer[fsm->StateInfos[0][0]]>=1.8)&&(fsm_state_timer[fsm->StateInfos[0][0]]<=2.7))
        {
            U_Gain = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]]-1.8)/(0.9-DEL_T)));
        }
//        printf("time = %f\n",fsm_state_timer[fsm->StateInfos[0][0]]);
        U3_Gain = U_Gain;

        LandingState = DSP;
        /*
        double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];
        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
        */
        /*
        HUBO2_ForceInitCon[RIGHT][Xdir] = -HUBO2_ForceInitCon_RF_X(0 , (sharedSEN->FT[RAFT].Fx-sharedSEN->FT[LAFT].Fx), 1);
        HUBO2_ForceInitCon[LEFT][Xdir]=-HUBO2_ForceInitCon[RIGHT][Xdir];
        HUBO2_ForceInitCon[RIGHT][Ydir] = -HUBO2_ForceInitCon_RF_Y(0 , sharedSEN->FT[RAFT].Fy, 1);
//        HUBO2_ForceInitCon[LEFT][Xdir]  = -HUBO2_ForceInitCon_LF_X(0 , sharedSEN->FT[LAFT].Fx, 1);
        HUBO2_ForceInitCon[LEFT][Ydir]  = -HUBO2_ForceInitCon_LF_Y(0 , sharedSEN->FT[LAFT].Fy, 1);

        AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0002);
        AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0002);
        AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0011);
        */
        if(fsm_state_timer[fsm->StateInfos[0][0]]<=2.7)
        {
            U_Gain_DSP = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(2.7-DEL_T)));
        }
//        printf("U_Gain_DSP = %f\n",U_Gain_DSP);

                ReactiveControl(1,1,1);

    }
        break;
    case STATE_DSP_RF:
    {

        if(fsm->walking_mode==LADDER_WALKING)
        {
            /*AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
            AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);
            AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);*/
//            double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

//            kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
//            kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

//            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
//            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

//            Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
//            Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
        }
        if(EarlyLandingFlag[LEFT] == 0)
        {
            if(fsm_state_timer[fsm->StateInfos[0][0]] < 0.1)
            {
                if(LF_FZ_LPF >EarlyLanding_FT/3.) // When Fz is higher than EarlyLanding_FT
                {
                    DSP_LandingFlag[LEFT] = 1;// Turn on EarlyLandingFlag
                }

                if(DSP_LandingFlag[LEFT] == 0)
                {
                    Late_Landing_Comp[LEFT] += 0.0006;
                    Late_Landing_Comp_last[LEFT] = Late_Landing_Comp[LEFT];
                }
            }
            else if(fsm_state_timer[fsm->StateInfos[0][0]] >= 0.1 && fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->TIME_DSP)
            {
                Late_Landing_Comp[LEFT] = Late_Landing_Comp_last[LEFT]*(1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] - 0.1)/(fsm->TIME_DSP-DEL_T-0.1))));
                DSP_LandingFlag[LEFT]  = 0;
            }
        }
//        I_ZMP_CON_X = X_ZMP_integral_in_static(1);
//        I_ZMP_CON_Y = Y_ZMP_integral_in_static(1);
        Ground_LF = Estimated_GLOBAL_Z_LF_last;

        GLOBAL_Z_RF_last2 = GLOBAL_Z_RF;
        GLOBAL_Z_LF_last2 = GLOBAL_Z_LF;

//        AnkleRollAngle_last[HUBO_RIGHT] = AnkleRollAngle[HUBO_RIGHT];
//        AnkleRollAngle_last[HUBO_LEFT] = AnkleRollAngle[HUBO_LEFT];

        //AnklePitchAngle_last = AnklePitchAngle;
//        TorsoPitchAngle_last = TorsoPitchAngle;
//        I_ZMP_CON_X_last = I_ZMP_CON_X;
//        I_ZMP_CON_Y_last = I_ZMP_CON_Y;
        Kirk_Control();

        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->TIME_DSP)
        {
            Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON*(1-0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->TIME_DSP)));
            Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON*(1-0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->TIME_DSP)));
        }
        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
        if(LandingState == RSSP) Pre_LandingState = RSSP;
        if(LandingState == LSSP) Pre_LandingState = LSSP;
        LandingState = DSP;


        yaw_angle_last = yaw_angle;


       FTcnt=FTcnt2=mean_FT=sum_FT=mean_FT2=sum_FT2=0;

              ReactiveControl(1,1,1);

    }
        break;
    case STATE_SSP_RF:
    {
//        if(fsm_state_timer[fsm->StateInfos[0][0]] >0.1 && fsm_state_timer[fsm->StateInfos[0][0]] <0.1+DEL_T)
//            ZeroGainRightLeg();
//        else if(fsm_state_timer[fsm->StateInfos[0][0]] >0.3&&fsm_state_timer[fsm->StateInfos[0][0]] <0.3+DEL_T)
//            FullGainRightLeg();

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*3./5.)
        {
            if(sharedSEN->FOG.Roll <-2.0)
            {
                U0_Gain_Goal_KI += 0.02*(sharedSEN->FOG.Roll);
                printf("ugain_KI = %f\n",U0_Gain_KI);
            }

            if(U0_Gain_Goal_KI<-0.3)U0_Gain_Goal_KI=-0.3;
            U0_Gain_KI_last = U0_Gain_KI;
            U0_Gain_KI = U0_Gain_Goal_KI*0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0]*3./5.)));
        }
        else
        {
            U0_Gain_Goal_KI=0.;
            U0_Gain_KI = U0_Gain_KI_last*(1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]]-fsm->STimeInfos[0]*3./5.)/(fsm->STimeInfos[0]*2./5.))));
        }

        if(fsm->walking_mode !=TERRAIN_WALKING_ONE_STEP || fsm->walking_mode !=TERRAIN_WALKING)
        {
        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
        }
        if(fsm_state_timer[STATE_SSP_RF]==0.)
        {
            CNT_final_gain_SSP_ZMP_CON = 0;
            kirkZMPCon_XP1(0,0,0);
            kirkZMPCon_YP1(0,0,0);
            StepNumber++; printf(">> Step No.= %d\n",StepNumber);
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0])Poly_5th(fsm_state_timer[STATE_SSP_RF], 0.0, fsm->STimeInfos[0], 1., 0., 0., 0., temp_Add2);

        //AnkleRollAngle[HUBO_RIGHT] = AnkleRollAngle_last[HUBO_RIGHT]*temp_Add2[0];

//        HUBO2TorqInitConRAR(0.0, 0.0, 0);
//        HUBO2TorqInitConRAP(0.0, 0.0, 0);

        //Recover EarlyLanding
        if(fsm_state_timer[fsm->StateInfos[0][0]] < fsm->STimeInfos[0]/2.) //Recovery after early landing during SSP
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]/2.,GLOBAL_Z_LF_last,0.,0.,fsm->LeftInfos[0][2],temp_Z_LF);
        else
        {
            GLOBAL_Z_LF_last = GLOBAL_Z_LF;
            temp_Z_LF[0]=fsm->LeftInfos[0][2];
        }

        GLOBAL_Z_LF = temp_Z_LF[0];
        if((fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*4./5.)&&(EarlyLandingFlag[RIGHT] == 0))
        {
            //Kirk_Control_ssp();
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]] < fsm->STimeInfos[0]*4./5.)
        {
            FTcnt++;
            sum_FT += RF_FZ_LPF;

            mean_FT = sum_FT/FTcnt;
        }
        else
        {
            FTcnt2++;
            sum_FT2 += RF_FZ_LPF;

            mean_FT2 = sum_FT2/FTcnt2;
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= info_footuptime)
        {
            EarlyLandingFlag[RIGHT] = 0;

            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,info_footuptime,GLOBAL_Z_RF_last2,0.,0.,info_footupheight,temp_Z_RF);
            GLOBAL_Z_RF = temp_Z_RF[0];

        }
        else
        {
//            printf("RSSP mean_FT2 - mean_FT = %f\n",mean_FT2 - mean_FT);
//            if(RF_FZ_LPF >EarlyLanding_FT && fsm_state_timer[fsm->StateInfos[0][0]] >= fsm->STimeInfos[0]*4./5.) // When Fz is higher than EarlyLanding_FT
            if((RF_FZ_LPF - mean_FT) >EarlyLanding_FT && fsm_state_timer[fsm->StateInfos[0][0]] >= fsm->STimeInfos[0]*4./5.) // When Fz is higher than EarlyLanding_FT
            {
                if(EarlyLandingFlag[RIGHT] == 0)
                {
                    EarlyLandingTime = fsm_state_timer[fsm->StateInfos[0][0]];
                    GLOBAL_Z_RF_last_earlylanding = GLOBAL_Z_RF;
                }
                EarlyLandingFlag[RIGHT] = 1;// Turn on EarlyLandingFlag

            }

             if(EarlyLandingFlag[RIGHT] == 0) //Normal walking
             {
                 GLOBAL_Z_RF = fsm->RightInfos[0][2];
                 Estimated_GLOBAL_Z_RF_last = Estimated_GLOBAL_Z_RF;
             }
             else //Early landing walking
             {
                 Ground_RF = Estimated_GLOBAL_Z_RF_last;
                 GLOBAL_Z_RF = GLOBAL_Z_RF_last_earlylanding;


             }
             GLOBAL_Z_RF_last = GLOBAL_Z_RF;
        }

        pre_state = HUBO_RIGHT;
        temp_distance_foot = (sqrt(pow(fsm->LeftInfos[0][0] - fsm->RightInfos[0][0],2) + pow((fsm->LeftInfos[0][1] - fsm->RightInfos[0][1]),2)));

        Kirk_Control();

//        yaw_angle =  Yaw_Gain*(AngularMomentumComp(fsm->RightInfosVel[0][0],fsm->RightInfosVel[0][1],(fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.,1,1));

//        if(yaw_angle > Yaw_max){ yaw_angle = Yaw_max;}
//        else if(yaw_angle < Yaw_min){ yaw_angle = Yaw_min;}
//        yaw_angle_last = yaw_angle;

        if((fsm->walking_mode == TERRAIN_WALKING)||(fsm->walking_mode == LADDER_WALKING)||(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP))
        {
//            printf("!!!!!!!!yaw\n");
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
            {
                if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0])
                {
                    Yaw_Gain2 = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0])));
                    yaw_angle = yaw_angle_last*Yaw_Gain2;
                    AngularMomentumComp(0,0,0,0);
                }
            }
            else
            {
                yaw_angle =  Yaw_Gain*(AngularMomentumComp(fsm->RightInfosVel[0][0],(fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.,1,1));

                if(yaw_angle > Yaw_max){ yaw_angle = Yaw_max;}
                else if(yaw_angle < Yaw_min){ yaw_angle = Yaw_min;}
                yaw_angle_last = yaw_angle;
            }
        }
        else
        {
            if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0])
            {
                Yaw_Gain2 = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0])));
                yaw_angle = yaw_angle_last*Yaw_Gain2;
                AngularMomentumComp(0,0,0,0);
            }
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*3./5.)
        {
            sum_GLOBAL_Xori_LF_n += GLOBAL_Xori_LF_n;
            sum_GLOBAL_Yori_LF_n += GLOBAL_Yori_LF_n;

            ave_GLOBAL_Xori_LF_n = sum_GLOBAL_Xori_LF_n/(fsm_state_timer[fsm->StateInfos[0][0]]/DEL_T);
            ave_GLOBAL_Yori_LF_n = sum_GLOBAL_Yori_LF_n/(fsm_state_timer[fsm->StateInfos[0][0]]/DEL_T);
            if((ave_GLOBAL_Xori_LF_n>5)||(Y_ZMP_n/1000 - GLOBAL_ZMP_REF_Y_n >0.4)) INSIDE_FLAG_Add_FootTask[LEFT][Xdir] = 1;
        }
        else
        {
            if(INSIDE_FLAG_Add_FootTask[LEFT][Xdir] == 1)
                Add_FootTask[LEFT][Xdir] = 0.02*0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]]-fsm->STimeInfos[0]*3./5.)/(fsm->STimeInfos[0]*2./5.)));
        }

        LandingState = RSSP;
        Late_Landing_Comp_last[RIGHT] = 0;
        Late_Landing_Comp_last[LEFT] = 0;


        //reset control RX_TC while RF swing
        if((fsm_state_timer[fsm->StateInfos[0][0]]>=fsm->STimeInfos[0]*2.0/5.0) && (fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0]*3.0/5.0) )
        {
            ReactiveControl(0,0,1);
//            U_gain_recovery = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] -fsm->STimeInfos[0]*2.0/5.0)/ (fsm->STimeInfos[0]*1.0/5.0f)));
//            RDRoll = RDRoll*U_gain_recovery;
//            RDPitch = RDPitch*U_gain_recovery;

        }else
        {
            ReactiveControl(0,1,1);
        }

    }
        break;
    case STATE_DSP_LF:
    {

        if(fsm->walking_mode==LADDER_WALKING)
        {
            /*
            AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
            AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);
            AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
            */
//            double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

//            kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
//            kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

//            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
//            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

//            Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
//            Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
        }
        if(EarlyLandingFlag[RIGHT] == 0)
        {
            if(fsm_state_timer[fsm->StateInfos[0][0]] < 0.1)
            {
                if(RF_FZ_LPF >EarlyLanding_FT/3.) // When Fz is higher than EarlyLanding_FT
                {
                    DSP_LandingFlag[RIGHT] = 1;// Turn on EarlyLandingFlag
                }

                if(DSP_LandingFlag[RIGHT] == 0)
                {
                    Late_Landing_Comp[RIGHT] += 0.0006;
                    Late_Landing_Comp_last[RIGHT] = Late_Landing_Comp[RIGHT];
                }
            }
            else if(fsm_state_timer[fsm->StateInfos[0][0]] >= 0.1 && fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->TIME_DSP)
            {
                Late_Landing_Comp[RIGHT] = Late_Landing_Comp_last[RIGHT]*(1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] - 0.1)/(fsm->TIME_DSP-DEL_T-0.1))));
                DSP_LandingFlag[RIGHT]  = 0;
            }
        }

        Ground_RF = Estimated_GLOBAL_Z_RF_last;

        GLOBAL_Z_RF_last2 = GLOBAL_Z_RF;
        GLOBAL_Z_LF_last2 = GLOBAL_Z_LF;

//        AnkleRollAngle_last[HUBO_RIGHT] = AnkleRollAngle[HUBO_RIGHT];
//        AnkleRollAngle_last[HUBO_LEFT] = AnkleRollAngle[HUBO_LEFT];

        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);

//        AnklePitchAngle_last = AnklePitchAngle;
//        TorsoPitchAngle_last = TorsoPitchAngle;
        Kirk_Control();
        //ZMP_intergral_control();
        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->TIME_DSP)
        {
            Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON*(1-0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->TIME_DSP)));
            Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON*(1-0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->TIME_DSP)));
        }
        if(LandingState == RSSP) Pre_LandingState = RSSP;
        if(LandingState == LSSP) Pre_LandingState = LSSP;
        LandingState = DSP;

        yaw_angle_last = yaw_angle;

        FTcnt=FTcnt2=mean_FT=sum_FT=mean_FT2=sum_FT2=0;

            ReactiveControl(1,1,1);
    }
        break;
    case STATE_SSP_LF:
    {
//        if(fsm_state_timer[fsm->StateInfos[0][0]] >0.1 && fsm_state_timer[fsm->StateInfos[0][0]] <0.1+DEL_T)
//            ZeroGainLeftLeg();
//        else if(fsm_state_timer[fsm->StateInfos[0][0]] >0.3&&fsm_state_timer[fsm->StateInfos[0][0]] <0.3+DEL_T)
//            FullGainLeftLeg();
        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*3./5.)
        {
            if(sharedSEN->FOG.Roll >2.0)
            {
                U0_Gain_Goal_KI -= 0.02*(sharedSEN->FOG.Roll);
                printf("ugain_KI = %f\n",U0_Gain_KI);
            }

            if(U0_Gain_Goal_KI<-0.3)U0_Gain_Goal_KI=-0.3;
            U0_Gain_KI_last = U0_Gain_KI;
            U0_Gain_KI = U0_Gain_Goal_KI*0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0]*3./5.)));
        }
        else
        {
            U0_Gain_Goal_KI=0.;
            U0_Gain_KI = U0_Gain_KI_last*(1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]]-fsm->STimeInfos[0]*3./5.)/(fsm->STimeInfos[0]*2./5.))));
        }
        if(fsm->walking_mode !=TERRAIN_WALKING_ONE_STEP || fsm->walking_mode !=TERRAIN_WALKING)
        {
        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
        }
        if(fsm_state_timer[fsm->StateInfos[0][0]]==0)
        {
            CNT_final_gain_SSP_ZMP_CON = 0;
            kirkZMPCon_XP1(0,0,0);
            kirkZMPCon_YP1(0,0,0);
            StepNumber++;        printf(">> Step No.= %d \n",StepNumber);
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]) Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0],1.,0.,0.,0.,temp_Add2);
//        AnkleRollAngle[HUBO_LEFT] = AnkleRollAngle_last[HUBO_LEFT]*temp_Add2[0];
//        Add_FootTask[LEFT][Zdir] = Add_FootTask_last[LEFT][Zdir]*temp_Add2[0];
//        AnklePitchAngle = AnklePitchAngle_last*temp_Add2[0];
//        HUBO2TorqInitConLAR(0.0, 0.0, 0);
//        HUBO2TorqInitConRAP(0.0, 0.0, 0);

        //Recover EarlyLanding
        if(fsm_state_timer[fsm->StateInfos[0][0]]<fsm->STimeInfos[0]/2.) // Recovery after early landingduring for DSP
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]/2.,GLOBAL_Z_RF_last,0.,0.,fsm->RightInfos[0][2],temp_Z_RF);
        else
        {
            GLOBAL_Z_RF_last = GLOBAL_Z_RF;
            temp_Z_RF[0]=fsm->RightInfos[0][2];
        }

        GLOBAL_Z_RF = temp_Z_RF[0];

        if((fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*4./5.)&&(EarlyLandingFlag[LEFT] == 0))
        {
            //Kirk_Control_ssp();
        }
        if(fsm_state_timer[fsm->StateInfos[0][0]] < fsm->STimeInfos[0]*4./5.)
        {
            FTcnt++;
            sum_FT += LF_FZ_LPF;

            mean_FT = sum_FT/FTcnt;
        }
        else
        {
            FTcnt2++;
            sum_FT2 += LF_FZ_LPF;

            mean_FT2 = sum_FT2/FTcnt2;
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= info_footuptime)
        {
            EarlyLandingFlag[LEFT] = 0;

            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,info_footuptime,GLOBAL_Z_LF_last2,0.,0.,info_footupheight,temp_Z_LF);
            GLOBAL_Z_LF = temp_Z_LF[0];
        }
        else
        {
//            printf("LSSP mean_FT2 - mean_FT = %f\n",mean_FT2 - mean_FT);
//             if(LF_FZ_LPF >EarlyLanding_FT && fsm_state_timer[fsm->StateInfos[0][0]] >= fsm->STimeInfos[0]*4./5.) // When Fz is higher than EarlyLanding_FT
            if((LF_FZ_LPF - mean_FT) >EarlyLanding_FT && fsm_state_timer[fsm->StateInfos[0][0]] >= fsm->STimeInfos[0]*4./5.) // When Fz is higher than EarlyLanding_FT
             {
                 if(EarlyLandingFlag[LEFT] == 0) //
                 {
                     EarlyLandingTime = fsm_state_timer[fsm->StateInfos[0][0]];
                     GLOBAL_Z_LF_last_earlylanding = GLOBAL_Z_LF;
                 }
                 EarlyLandingFlag[LEFT] = 1; // Turn on EarlyLandingFlag
             }

             if(EarlyLandingFlag[LEFT] == 0) //Normal walking
             {
                 GLOBAL_Z_LF = fsm->LeftInfos[0][2];
                 Estimated_GLOBAL_Z_LF_last=Estimated_GLOBAL_Z_LF;
                 Add_FootTask2[LEFT][Zdir] = 0;
             }
             else //Early landing walking
             {
                 Ground_LF = Estimated_GLOBAL_Z_LF_last;
                 GLOBAL_Z_LF = GLOBAL_Z_LF_last_earlylanding;
             }
             GLOBAL_Z_LF_last = GLOBAL_Z_LF;
        }

        temp_distance_foot = (sqrt(pow(fsm->LeftInfos[0][0] - fsm->RightInfos[0][0],2) + pow((fsm->LeftInfos[0][1] - fsm->RightInfos[0][1]),2)));

        Kirk_Control();

//        yaw_angle = Yaw_Gain*(AngularMomentumComp(fsm->LeftInfosVel[0][0],fsm->RightInfosVel[0][1],(fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.,0,1));

//        if(yaw_angle > Yaw_max){ yaw_angle = Yaw_max;}
//        else if(yaw_angle < Yaw_min){ yaw_angle = Yaw_min;}

//        yaw_angle_last = yaw_angle;

        if((fsm->walking_mode == TERRAIN_WALKING)||(fsm->walking_mode == LADDER_WALKING)||(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP))
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
            {
                if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0])
                {
                    Yaw_Gain2 = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0])));
                    yaw_angle = yaw_angle_last*Yaw_Gain2;
                    AngularMomentumComp(0,0,0,0);
                }
            }
            else
            {
                yaw_angle = Yaw_Gain*(AngularMomentumComp(fsm->LeftInfosVel[0][0],(fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.,0,1));

                if(yaw_angle > Yaw_max){ yaw_angle = Yaw_max;}
                else if(yaw_angle < Yaw_min){ yaw_angle = Yaw_min;}

                yaw_angle_last = yaw_angle;
            }
        }
        else
        {
            if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0])
            {
                Yaw_Gain2 = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0])));
                yaw_angle = yaw_angle_last*Yaw_Gain2;
                AngularMomentumComp(0,0,0,0);
            }
        }

        pre_state = HUBO_LEFT;
        LandingState = LSSP;
        Late_Landing_Comp_last[RIGHT] = 0;
        Late_Landing_Comp_last[LEFT] = 0;

        if((fsm_state_timer[fsm->StateInfos[0][0]]>=fsm->STimeInfos[0]*2.0/5.0) && (fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0]*3.0/5.0) )
        {
            ReactiveControl(0,1,0);
//            U_gain_recovery = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] -fsm->STimeInfos[0]*2.0/5.0)/ (fsm->STimeInfos[0]*1.0/5.0f)));
//            LDRoll = LDRoll*U_gain_recovery;
//            LDPitch = LDPitch*U_gain_recovery;

        }else
        {
            ReactiveControl(0,1,1);
        }

    }
        break;
    case STATE_DSP_FINAL_RFLF:
    {
        if(pre_state == HUBO_RIGHT)
        {
            if(EarlyLandingFlag[RIGHT] == 0)
            {
                if(fsm_state_timer[fsm->StateInfos[0][0]] < 0.1)
                {
                    if(RF_FZ_LPF >EarlyLanding_FT/3.) // When Fz is higher than EarlyLanding_FT
                    {
                        DSP_LandingFlag[RIGHT] = 1;// Turn on EarlyLandingFlag
                    }

                    if(DSP_LandingFlag[RIGHT] == 0)
                    {
                        Late_Landing_Comp[RIGHT] += 0.0006;
                        Late_Landing_Comp_last[RIGHT] = Late_Landing_Comp[RIGHT];
                    }
                }
                else if(fsm_state_timer[fsm->StateInfos[0][0]] >= 0.1 && fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->TIME_DSP)
                {
                    //Late_Landing_Comp[RIGHT] = Late_Landing_Comp_last[RIGHT]*(1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] - fsm->TIME_DSP/2.)/(fsm->TIME_DSP/2.-DEL_T))));
                    DSP_LandingFlag[RIGHT]  = 0;
                }
            }
        }
        else if(pre_state == HUBO_LEFT)
        {
            if(EarlyLandingFlag[LEFT] == 0)
            {
                if(fsm_state_timer[fsm->StateInfos[0][0]] < 0.1)
                {
                    if(LF_FZ_LPF >EarlyLanding_FT/3.) // When Fz is higher than EarlyLanding_FT
                    {
                        DSP_LandingFlag[LEFT] = 1;// Turn on EarlyLandingFlag
                    }

                    if(DSP_LandingFlag[LEFT] == 0)
                    {
                        Late_Landing_Comp[LEFT] += 0.0006;
                        Late_Landing_Comp_last[LEFT] = Late_Landing_Comp[LEFT];
                    }
                }
                else if(fsm_state_timer[fsm->StateInfos[0][0]] >= 0.1 && fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->TIME_DSP)
                {
                    //Late_Landing_Comp[LEFT] = Late_Landing_Comp_last[LEFT]*(1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] - fsm->TIME_DSP/2.)/(fsm->TIME_DSP/2.-DEL_T))));
                    DSP_LandingFlag[LEFT]  = 0;
                }
            }
        }
//        I_ZMP_CON_X = X_ZMP_integral_in_static(1);
//        I_ZMP_CON_Y = Y_ZMP_integral_in_static(1);
        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0]+1.3)
        {
//            Yaw_Gain2 = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0]+1.3)));
//            yaw_angle = yaw_angle_last*Yaw_Gain2;
//            AngularMomentumComp(0,0,0,0,0);
        }
        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0]+0.5)
        {
//            GainGyro = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(fsm->STimeInfos[0]+0.5)));
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]]==0)
        {
            Upperbody_Gain_Lock();
            //printf(">> STATE_DSP_FINAL_RFLF \n");
        }
        StepNumber=0;
        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]+0.5)
        {
            if(temp_distance_foot <= kine_drc_hubo4.L_PEL2PEL+0.03*2+0.05)
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]+0.5,1.,0.,0.,1.,temp_G_DSP_X);
            else if(temp_distance_foot <= kine_drc_hubo4.L_PEL2PEL+0.03*2+0.1)
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]+0.5,1.,0.,0.,0.6,temp_G_DSP_X);
            else
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]+0.5,1.,0.,0.,0.4,temp_G_DSP_X);
            //printf("temp_G_DSP_X  = %f\n",temp_G_DSP_X[0]);
            G_DSP_X = G_DSP_Y = temp_G_DSP_X[0];

//            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]+0.5,GLOBAL_Z_RF_last,0.,0.,fsm->RightInfos[0][2],temp_Z_RF);
//            GLOBAL_Z_RF = temp_Z_RF[0];

//            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]+0.5,GLOBAL_Z_LF_last,0.,0.,fsm->LeftInfos[0][2],temp_Z_LF);
//            GLOBAL_Z_LF = temp_Z_LF[0];

            EarlyLandingFlag[RIGHT] = 0;
            EarlyLandingFlag[LEFT] = 0;
        }

//        if(pre_state == HUBO_RIGHT)
        {
            HUBO2_ForceInitCon[RIGHT][Xdir] = -HUBO2_ForceInitCon_RF_X(0 , (sharedSEN->FT[RAFT].Fx-sharedSEN->FT[LAFT].Fx), 1);
            HUBO2_ForceInitCon[LEFT][Xdir]=-HUBO2_ForceInitCon[RIGHT][Xdir];
            HUBO2_ForceInitCon[RIGHT][Ydir] = -HUBO2_ForceInitCon_RF_Y(0 , sharedSEN->FT[RAFT].Fy, 1);
            HUBO2_ForceInitCon[LEFT][Ydir]  = -HUBO2_ForceInitCon_LF_Y(0 , sharedSEN->FT[LAFT].Fy, 1);
        }

        if(fsm_state_timer[fsm->StateInfos[0][0]]>fsm->TIME_DSP*0./5.)
        {
            double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3];
            double temp1[4],temp2[4],temp3[4],temp4[4];

            M_LF[0] =  sharedSEN->FT[LAFT].Mx;
            M_LF[1] =  sharedSEN->FT[LAFT].My;
            M_LF[2] =  sharedSEN->FT[LAFT].Mz;

//            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedSEN->FT[RAFT].Mx;
            M_RF[1] =  sharedSEN->FT[RAFT].My;
            M_RF[2] =  sharedSEN->FT[RAFT].Mz;

//            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);
            if(pre_state == HUBO_RIGHT)
            {

                if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
                {
                    double result_RF[3],result_LF[3];
                    convert_euler_FT( pre_state,des_pRF_3x1, des_pLF_3x1,des_qRF_4x1, des_qLF_4x1,result_RF,result_LF);
                    WBIK_Torq[RIGHT][Xdir] = result_RF[0];
                    WBIK_Torq[LEFT][Xdir]  = result_LF[0];
                    WBIK_Torq[RIGHT][Ydir] = result_RF[1];
                    WBIK_Torq[LEFT][Ydir]  = result_LF[1];
                }
                else
                {
                    WBIK_Torq[RIGHT][Xdir] = -WBIKTorqInitConRAR(0, M_RF[0] - sharedSEN->FT[RAFT].Fy*0.00, 1,0.0003);
                    WBIK_Torq[LEFT][Xdir]  = 0.;
                    WBIK_Torq[RIGHT][Ydir] = -WBIKTorqInitConRAP(0, M_RF[1] - sharedSEN->FT[RAFT].Fx*0.00, 1,0.0003);
                    WBIK_Torq[LEFT][Ydir]  = 0.;
                }

                qtRZ(0., temp1);
                qtRY(WBIK_Torq[RIGHT][Ydir]*D2R, temp2);
                qtRX(WBIK_Torq[RIGHT][Xdir]*D2R, temp3);

                QTcross(temp1,temp2,temp1);
                QTcross(temp1,temp3,qt_WBIK_Torq_RF);

                qtRZ(0., temp1);
                qtRY(WBIK_Torq[LEFT][Ydir]*D2R, temp2);
                qtRX(WBIK_Torq[LEFT][Xdir]*D2R, temp3);

                QTcross(temp1,temp2,temp1);
                QTcross(temp1,temp3,qt_WBIK_Torq_LF);
            }
            else if(pre_state == HUBO_LEFT)
            {
                if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
                {
                    double result_RF[3],result_LF[3];
                    convert_euler_FT( pre_state,des_pRF_3x1, des_pLF_3x1,des_qRF_4x1, des_qLF_4x1,result_RF,result_LF);
                    WBIK_Torq[RIGHT][Xdir] = result_RF[0];
                    WBIK_Torq[LEFT][Xdir]  = result_LF[0];
                    WBIK_Torq[RIGHT][Ydir] = result_RF[1];
                    WBIK_Torq[LEFT][Ydir]  = result_LF[1];
                }
                else
                {
                    WBIK_Torq[LEFT][Xdir]   = -WBIKTorqInitConLAR(0, M_LF[0] - sharedSEN->FT[LAFT].Fy*0.00, 1,0.0003);
                    WBIK_Torq[RIGHT][Xdir]  = 0.;
                    WBIK_Torq[LEFT][Ydir]   = -WBIKTorqInitConLAP(0, M_LF[1] - sharedSEN->FT[LAFT].Fx*0.00, 1,0.0003);
                    WBIK_Torq[RIGHT][Ydir]  = 0.;
                }
                qtRZ(0., temp1);
                qtRY(WBIK_Torq[RIGHT][Ydir]*D2R, temp2);
                qtRX(WBIK_Torq[RIGHT][Xdir]*D2R, temp3);

                QTcross(temp1,temp2,temp1);
                QTcross(temp1,temp3,qt_WBIK_Torq_RF);

                qtRZ(0., temp1);
                qtRY(WBIK_Torq[LEFT][Ydir]*D2R, temp2);
                qtRX(WBIK_Torq[LEFT][Xdir]*D2R, temp3);

                QTcross(temp1,temp2,temp1);
                QTcross(temp1,temp3,qt_WBIK_Torq_LF);
            }
//        printf("WBIK_Torq Roll = %f,%f, Pitch  = %f,%f\n",WBIK_Torq[RIGHT][Xdir],WBIK_Torq[LEFT][Xdir],WBIK_Torq[RIGHT][Ydir],WBIK_Torq[LEFT][Ydir]);



            if(fsm_state_timer[fsm->StateInfos[0][0]]>fsm->TIME_DSP*.0/5.)
            {
                /*
                AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
                AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);

                    AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);*/
//                    if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
//                    AnklePitchAngle[HUBO_LEFT] = HUBO2TorqInitConLAP(0.0, 1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);

//                if(fsm->walking_mode != TERRAIN_WALKING_ONE_STEP)
                {
                double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

                kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
                kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

                des_pLF_3x1_n[2] =  des_pLF_3x1[2];
                des_pRF_3x1_n[2] =  des_pRF_3x1[2];

                convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

                Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
                Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
                }
            }
            else
            {
                /*
                AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
                AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);

                AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
                */
//                    if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
//                    AnklePitchAngle[HUBO_LEFT] = HUBO2TorqInitConLAP(0.0, 1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
                TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
                TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
                {
//                double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

//                kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
//                kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

//                des_pLF_3x1_n[2] =  des_pLF_3x1[2];
//                des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//                convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

//                Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
//                Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
                }
            }
            //printf("BTW_FOOT_Angle_roll = %f\n",BTW_FOOT_Angle_roll*R2D);
        }
//        TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
//        TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);

        Add_Joint_last[RHR] = Add_Joint[RHR];
        Add_Joint_last[LHR] = Add_Joint[LHR];
        Kirk_Control();
        if(fsm_state_timer[fsm->StateInfos[0][0]]<=fsm->STimeInfos[0])
        {
            Del_PC_X_SSP_XZMP_CON = Old_Del_PC_X_SSP_XZMP_CON*(1-0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->STimeInfos[0])));
            Del_PC_Y_SSP_YZMP_CON = Old_Del_PC_Y_SSP_YZMP_CON*(1-0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/fsm->STimeInfos[0])));
        }


        if(LandingState == RSSP) Pre_LandingState = RSSP;
        if(LandingState == LSSP) Pre_LandingState = LSSP;
        LandingState = FINAL;

        ReactiveControl(1,1,1);

    }
        break;
    case STATE_FINISHED:
    {
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3];
        double temp1[4],temp2[4],temp3[4],temp4[4];

        M_LF[0] =  sharedSEN->FT[LAFT].Mx;
        M_LF[1] =  sharedSEN->FT[LAFT].My;
        M_LF[2] =  sharedSEN->FT[LAFT].Mz;

//            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

        M_RF[0] =  sharedSEN->FT[RAFT].Mx;
        M_RF[1] =  sharedSEN->FT[RAFT].My;
        M_RF[2] =  sharedSEN->FT[RAFT].Mz;

//            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

//        AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0002);
        //                AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0002);
        //                AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0011);
        if(pre_state == HUBO_RIGHT)
        {

            if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
            {
                double result_RF[3],result_LF[3];
                convert_euler_FT( pre_state,des_pRF_3x1, des_pLF_3x1,des_qRF_4x1, des_qLF_4x1,result_RF,result_LF);
                WBIK_Torq[RIGHT][Xdir] = result_RF[0];
                WBIK_Torq[LEFT][Xdir]  = result_LF[0];
                WBIK_Torq[RIGHT][Ydir] = result_RF[1];
                WBIK_Torq[LEFT][Ydir]  = result_LF[1];
//                WBIK_Torq[RIGHT][Xdir] = -WBIKTorqInitConRAR(0, M_RF[0] - sharedSEN->FT[RAFT].Fy*0.003, 1,0.0003);
//                WBIK_Torq[LEFT][Xdir]  = -WBIKTorqInitConLAR(0, M_LF[0] - sharedSEN->FT[LAFT].Fy*0.003, 1,0.0003);
//                WBIK_Torq[RIGHT][Ydir] = -WBIKTorqInitConRAP(0, M_RF[1] - M_LF[1] - sharedSEN->FT[RAFT].Fx*0.003 - sharedSEN->FT[LAFT].Fx*0.003, 1,0.0003);
//                WBIK_Torq[LEFT][Ydir]  = 0.;
            }
            else
            {
                WBIK_Torq[RIGHT][Xdir] = -WBIKTorqInitConRAR(0, M_RF[0] - sharedSEN->FT[RAFT].Fy*0.00, 1,0.0003);
                WBIK_Torq[LEFT][Xdir]  = 0.;
                WBIK_Torq[RIGHT][Ydir] = -WBIKTorqInitConRAP(0, M_RF[1] - sharedSEN->FT[RAFT].Fx*0.00, 1,0.0003);
                WBIK_Torq[LEFT][Ydir]  = 0.;
            }

            qtRZ(0., temp1);
            qtRY(WBIK_Torq[RIGHT][Ydir]*D2R, temp2);
            qtRX(WBIK_Torq[RIGHT][Xdir]*D2R, temp3);

            QTcross(temp1,temp2,temp1);
            QTcross(temp1,temp3,qt_WBIK_Torq_RF);

            qtRZ(0., temp1);
            qtRY(WBIK_Torq[LEFT][Ydir]*D2R, temp2);
            qtRX(WBIK_Torq[LEFT][Xdir]*D2R, temp3);

            QTcross(temp1,temp2,temp1);
            QTcross(temp1,temp3,qt_WBIK_Torq_LF);
        }
        else if(pre_state == HUBO_LEFT)
        {
            if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
            {
                double result_RF[3],result_LF[3];
                convert_euler_FT( pre_state,des_pRF_3x1, des_pLF_3x1,des_qRF_4x1, des_qLF_4x1,result_RF,result_LF);
                WBIK_Torq[RIGHT][Xdir] = result_RF[0];
                WBIK_Torq[LEFT][Xdir]  = result_LF[0];
                WBIK_Torq[RIGHT][Ydir] = result_RF[1];
                WBIK_Torq[LEFT][Ydir]  = result_LF[1];
//                WBIK_Torq[LEFT][Xdir]   = -WBIKTorqInitConLAR(0, M_LF[0] - sharedSEN->FT[LAFT].Fy*0.003, 1,0.0003);
//                WBIK_Torq[RIGHT][Xdir]  = -WBIKTorqInitConRAR(0, M_RF[0] - sharedSEN->FT[RAFT].Fy*0.003, 1,0.0003);
//                WBIK_Torq[LEFT][Ydir]   = WBIKTorqInitConLAP(0, M_RF[1] - M_LF[1] - sharedSEN->FT[RAFT].Fx*0.003 - sharedSEN->FT[LAFT].Fx*0.003, 1,0.0003);
//                WBIK_Torq[RIGHT][Ydir]  = 0.;
            }
            else
            {
                WBIK_Torq[LEFT][Xdir]   = -WBIKTorqInitConLAR(0, M_LF[0] - sharedSEN->FT[LAFT].Fy*0.00, 1,0.0003);
                WBIK_Torq[RIGHT][Xdir]  = 0.;
                WBIK_Torq[LEFT][Ydir]   = -WBIKTorqInitConLAP(0, M_LF[1] - sharedSEN->FT[LAFT].Fx*0.00, 1,0.0003);
                WBIK_Torq[RIGHT][Ydir]  = 0.;
            }
            qtRZ(0., temp1);
            qtRY(WBIK_Torq[RIGHT][Ydir]*D2R, temp2);
            qtRX(WBIK_Torq[RIGHT][Xdir]*D2R, temp3);

            QTcross(temp1,temp2,temp1);
            QTcross(temp1,temp3,qt_WBIK_Torq_RF);

            qtRZ(0., temp1);
            qtRY(WBIK_Torq[LEFT][Ydir]*D2R, temp2);
            qtRX(WBIK_Torq[LEFT][Xdir]*D2R, temp3);

            QTcross(temp1,temp2,temp1);
            QTcross(temp1,temp3,qt_WBIK_Torq_LF);
        }

//    printf("WBIK_Torq Roll = %f,%f, Pitch  = %f,%f\n",WBIK_Torq[RIGHT][Xdir],WBIK_Torq[LEFT][Xdir],WBIK_Torq[RIGHT][Ydir],WBIK_Torq[LEFT][Ydir]);
//        WBIK_Torq[RIGHT][Ydir] = WBIKTorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);

        Late_Landing_Comp_last[RIGHT] = 0;
        Late_Landing_Comp_last[LEFT] = 0;
//        I_ZMP_CON_X = X_ZMP_integral_in_static(1);
//        I_ZMP_CON_Y = Y_ZMP_integral_in_static(1);

        if(fsm_state_timer[fsm->StateInfos[0][0]]<complete_time)
        {
        //if(fsm_state_timer[fsm->StateInfos[0][0]]<10)
            if(fsm_state_timer[fsm->StateInfos[0][0]]<complete_time/5.)
             U_Gain = 1 - 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]])/(complete_time/5.)));
            else
            {
                if(fabs(sharedSEN->FOG.Pitch)<0.1 && fabs(sharedSEN->FOG.Roll)<0.1)
                {
                    fsmFlag =false;
                    printf("Reached Complete Form Ealier!!!!!\n");
                }
            }

            if(fsm_state_timer[fsm->StateInfos[0][0]]==0)
            {
                //printf(">> STATE_FINISHED \n");
            }

            G_DSP_X_last = G_DSP_X;
            G_DSP_Y_last = G_DSP_Y;

//            GLOBAL_Z_RF = temp_Z_RF[0];
//            GLOBAL_Z_LF = temp_Z_LF[0];
            GLOBAL_Z_RF_last = GLOBAL_Z_RF;
            GLOBAL_Z_LF_last = GLOBAL_Z_LF;
            // Ankle roll torque correction
//            if(pre_state == HUBO_RIGHT)
            //if(fsm_state_timer[fsm->StateInfos[0][0]]<10)
            {

//                AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx + sharedSEN->FT[RAFT].Mx*sin(fsm->RightInfos[0][4]*D2R)*0.03, 1);
//                AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx + sharedSEN->FT[LAFT].Mx*sin(fsm->LeftInfos[0][4]*D2R)*0.03, 1);
//                AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My + sharedSEN->FT[RAFT].Mx*sin(fsm->RightInfos[0][5]*D2R)*0.03 + sharedSEN->FT[LAFT].Mx*sin(fsm->LeftInfos[0][5]*D2R)*0.03), 1);
//                HUBO2_ForceInitCon[RIGHT][Xdir] = -HUBO2_ForceInitCon_RF_X(0 , sharedSEN->FT[RAFT].Fx, 1);
                HUBO2_ForceInitCon[RIGHT][Xdir] = -HUBO2_ForceInitCon_RF_X(0 , (sharedSEN->FT[RAFT].Fx-sharedSEN->FT[LAFT].Fx), 1);
                HUBO2_ForceInitCon[LEFT][Xdir]  = -HUBO2_ForceInitCon[RIGHT][Xdir];
                HUBO2_ForceInitCon[RIGHT][Ydir] = -HUBO2_ForceInitCon_RF_Y(0 , sharedSEN->FT[RAFT].Fy, 1);
//                HUBO2_ForceInitCon[LEFT][Xdir]  = -HUBO2_ForceInitCon_LF_X(0 , sharedSEN->FT[LAFT].Fx, 1);
                HUBO2_ForceInitCon[LEFT][Ydir]  = -HUBO2_ForceInitCon_LF_Y(0 , sharedSEN->FT[LAFT].Fy, 1);

//                if(InitializeFLAG == 1)
//                {
//                AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0002);
//                AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0002);
//                AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0011);
//                }
//                else
                {
/*
                    AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx - sharedSEN->FT[RAFT].Fy*0.03, 1,0.0003);
                    AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx - sharedSEN->FT[LAFT].Fy*0.03, 1,0.0003);
//                    if(pre_state == HUBO_RIGHT)
                        AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConRAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
//                    else if(pre_state == HUBO_LEFT)
                        if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
                        AnklePitchAngle[HUBO_LEFT] = HUBO2TorqInitConLAP(0.0, 1.0f*(sharedSEN->FT[RAFT].My -sharedSEN->FT[LAFT].My - sharedSEN->FT[RAFT].Fx*0.03 - sharedSEN->FT[LAFT].Fx*0.03     ), 1,0.0015);
                        */
                }
            }
            Kirk_Control();

//            else if(pre_state == HUBO_LEFT)
//            {
//                AnkleRollAngle[HUBO_LEFT]   = -HUBO2TorqInitConLAR(0, sharedSEN->FT[LAFT].Mx, 1);
//                AnkleRollAngle[HUBO_RIGHT]  = -HUBO2TorqInitConRAR(0, sharedSEN->FT[RAFT].Mx, 1);
//                AnklePitchAngle[HUBO_RIGHT] = HUBO2TorqInitConLAP(0.0, -1.0f*(sharedSEN->FT[RAFT].My-sharedSEN->FT[LAFT].My), 1);
//            }



            //printf("Roll = %f, Pitch = %f, Yaw = %f\n",BTW_FOOT_Angle_roll*R2D,BTW_FOOT_Angle_pitch*R2D,BTW_FOOT_Angle_yaw*R2D);

            double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];
//            des_pRF_3x1_n[0] =  des_pRF_3x1[0]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pRF_3x1[1]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pRF_3x1_n[1] = -des_pRF_3x1[0]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pRF_3x1[1]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            des_pLF_3x1_n[0] =  des_pLF_3x1[0]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pLF_3x1[1]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pLF_3x1_n[1] = -des_pLF_3x1[0]*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + des_pLF_3x1[1]*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
            kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
            kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R, sharedSEN->FOG.Pitch*D2R, 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(yaw_angle*D2R) - sharedSEN->FOG.Pitch*D2R*sin(yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(yaw_angle*D2R) + sharedSEN->FOG.Pitch*D2R*cos(yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedSEN->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedSEN->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedSEN->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedSEN->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMUAccX[CIMU]*D2R, sharedData->IMUAccY[CIMU]*D2R, 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
//                printf("BTW_FOOT_Angle_roll = %f, pitch = %f, yaw = %f\n",BTW_FOOT_Angle_roll*R2D,BTW_FOOT_Angle_pitch*R2D,BTW_FOOT_Angle_yaw*R2D);
            Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
            Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
            //printf("BTW_FOOT_Angle_roll = %f\n",BTW_FOOT_Angle_roll*R2D);
//            TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
//            TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);

            //if(fsm_state_timer[fsm->StateInfos[0][0]]<3)
            SwanObserver(-Del_PC_X_DSP_XZMP_CON, X_ZMP_n - (GLOBAL_ZMP_REF_X_n)*1000,hat_X);
            LandingState = FINAL;

            ReactiveControl(1,1,1);


        }
        else
        {
            fsmFlag =false;
        }
        break;
    }

    default:
        break;
    }

    if(Add_FootTask[RIGHT][Zdir] > 0.05)Add_FootTask[RIGHT][Zdir] =  0.05;
    if(Add_FootTask[RIGHT][Zdir] <-0.05)Add_FootTask[RIGHT][Zdir] = -0.05;

    if(Add_FootTask[LEFT][Zdir] > 0.05)Add_FootTask[LEFT][Zdir] =  0.05;
    if(Add_FootTask[LEFT][Zdir] <-0.05)Add_FootTask[LEFT][Zdir] = -0.05;


    static int CNT_AnkleControl1=0,CNT_AnkleControl2=0,CNT_AnkleControl3=0,CNT_AnkleControl4=0;

    double RF_PEL[3] ={0,},LF_PEL[3] ={0,};
    RF_PEL[0] =  kine_drc_hubo4.L_PEL2PEL*sin(yaw_angle*D2R);
    RF_PEL[1] = -kine_drc_hubo4.L_PEL2PEL*cos(yaw_angle*D2R);
    RF_PEL[2] = 0;

    LF_PEL[0] = -kine_drc_hubo4.L_PEL2PEL*sin(yaw_angle*D2R);
    LF_PEL[1] =  kine_drc_hubo4.L_PEL2PEL*cos(yaw_angle*D2R);
    LF_PEL[2] = 0;

    convert_euler_imu(RF_PEL, LF_PEL, sharedSEN->FOG.Roll*D2R, sharedSEN->FOG.Pitch*D2R, 0,BTW_PEL_angle_roll, BTW_PEL_angle_pitch, BTW_PEL_angle_yaw);
    convert_euler_imu(RF_PEL, LF_PEL, sharedSEN->FOG.RollVel*D2R, sharedSEN->FOG.PitchVel*D2R, 0,BTW_PEL_angle_roll_vel, BTW_PEL_angle_pitch_vel, BTW_PEL_angle_yaw_vel);

    FOGRollVel_NF2 = NotchFilter_GyroRollVel(sharedSEN->FOG.RollVel,1);
    FOGPitchVel_NF2 = NotchFilter_GyroPitchVel(sharedSEN->FOG.PitchVel,1);
//    FOGRollVel_NF2 = NotchFilter_GyroRollVel(BTW_PEL_angle_roll_vel*R2D,1);
//    FOGPitchVel_NF2 = NotchFilter_GyroPitchVel(BTW_PEL_angle_pitch_vel*R2D,1);

    FOGRollVel_LPF  = (1.f-2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f)*FOGRollVel_LPF  + 2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f*FOGRollVel_NF2;
    FOGPitchVel_LPF = (1.f-2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f)*FOGPitchVel_LPF + 2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f*FOGPitchVel_NF2;

//    static float pgain=0,dgain=0;
//    static float pgain_2=0,dgain_2=0;
//    pgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
//    dgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];

//    pgain_2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
//    dgain_2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
        if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
        {
            //only 1st order butterworth filter pitch pgain = 2.1, dgain = 10.
            //only 1st order butterworth filter roll  pgain = 2. , dgain = 7.
            AnkleControl1 = (sharedSEN->FOG.Roll*2.  + sharedSEN->FOG.RollVel*6.);
            AnkleControl2 = (sharedSEN->FOG.Pitch*2. + sharedSEN->FOG.PitchVel*4.);
//       AnkleControl1 = (sharedSEN->FOG.Roll*2.1 + FOGRollVel_LPF*19.4);
//       AnkleControl2 = (sharedSEN->FOG.Pitch*1.8 + FOGPitchVel_LPF*5.1);
        }
        else
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == LADDER_WALKING)
            {
                AnkleControl1 = (sharedSEN->FOG.Roll*2.3 + FOGRollVel_LPF*6.);
                AnkleControl2 = (sharedSEN->FOG.Pitch*2.3 + FOGPitchVel_LPF*4.);
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == GOAL_WALKING)
            {
                AnkleControl1 = (sharedSEN->FOG.Roll*3.7f + sharedSEN->FOG.RollVel*6.);
                AnkleControl2 = (sharedSEN->FOG.Pitch*3.7f + sharedSEN->FOG.PitchVel*4.);
            }else
            {
                AnkleControl1 = (sharedSEN->FOG.Roll*2.5f + sharedSEN->FOG.RollVel*6.);
                AnkleControl2 = (sharedSEN->FOG.Pitch*2.3f + sharedSEN->FOG.PitchVel*4.);
            }

        }
//    AnkleControl1 = (BTW_PEL_angle_roll*R2D*2.1 + FOGRollVel_LPF*2.1);
//    AnkleControl2 = (BTW_PEL_angle_pitch*R2D*1.9 + FOGPitchVel_LPF*2.1);

       den_a1 = 1;
       den_a2 = -0.801151070558751;//-0.854080685463467;//-0.909929988177738;
       num_b1 = 0.099424464720624;//0.072959657268267;//0.045035005911131;
       num_b2 = 0.099424464720624;//0.072959657268267;//0.045035005911131;

       u_i = AnkleControl1;
       y_i = -den_a2*y_i_1 + num_b1*u_i + num_b2*u_i_1; //1st order
       y_i_1 = y_i;
       u_i_1 = u_i;

       u_i1 = AnkleControl2;
       y_i1 = -den_a2*y_i_11 + num_b1*u_i1 + num_b2*u_i_11; //1st order
       y_i_11 = y_i1;
       u_i_11 = u_i1;


       if(y_i>20) y_i = 20;
       if(y_i<-20) y_i = -20;
       if(y_i1>20) y_i1 = 20;
       if(y_i1<-20) y_i1 = -20;

       if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
       {
           FOGRollVel_NF = y_i;//NotchFilter_GyroRollControlInput(y_i,1);
           FOGPitchVel_NF = y_i1;//NotchFilter_GyroPitchControlInput(y_i1,1);
       }
       else
       {
           FOGRollVel_NF = NotchFilter_GyroRollControlInput(y_i,1);
           FOGPitchVel_NF = NotchFilter_GyroPitchControlInput(y_i1,1);
       }

       if(pv_Index == 1)
       {
           GLOBAL_Xori_RF_last2=0;
           GLOBAL_Yori_RF_last2=0;
           GLOBAL_Xori_LF_last2=0;
           GLOBAL_Yori_LF_last2=0;
       }

       //       int Pre_Pre_LandingState = -1;
              if(LandingState == LSSP) // Left foot is swing foot
              {
                  Foot_gainRF = 0.5*(1 - cos(PI*CNT_AnkleControl1/20));
                  if(CNT_AnkleControl1<20)CNT_AnkleControl1++;

                  if(EarlyLandingFlag[LEFT] == 0)// If the left foot is not landed earlyer,
                  {
                  //supporting foot
                  GLOBAL_Xori_RF = Foot_gainRF*(FOGRollVel_NF) + GLOBAL_Xori_RF_last*(1-Foot_gainRF);
                  GLOBAL_Yori_RF = Foot_gainRF*(FOGPitchVel_NF) + GLOBAL_Yori_RF_last*(1-Foot_gainRF);
                  GLOBAL_Xori_RF_last2 = GLOBAL_Xori_RF;
                  GLOBAL_Yori_RF_last2 = GLOBAL_Yori_RF;
                  }

                  //swing foot
                  GLOBAL_Xori_LF = GLOBAL_Xori_LF_last*(1-Foot_gainRF);
                  GLOBAL_Yori_LF = GLOBAL_Yori_LF_last*(1-Foot_gainRF);

                  GLOBAL_Xori_LF2 = Foot_gainRF*(-AngleRoll);// +  GLOBAL_Xori_LF2_last*(1-Foot_gainRF) + ;
                  GLOBAL_Yori_LF2 = Foot_gainRF*(-AnglePitch);// +  GLOBAL_Yori_LF2_last*(1-Foot_gainRF) + ;

                  GLOBAL_Xori_LF2_last = GLOBAL_Xori_LF2;
                  GLOBAL_Yori_LF2_last = GLOBAL_Yori_LF2;
                  CNT_AnkleControl4 = 0;



              }
              else if(LandingState == RSSP) // Right foot is swing foot
              {
                  Foot_gainLF = 0.5*(1 - cos(PI*CNT_AnkleControl2/20));
                  if(CNT_AnkleControl2<20)CNT_AnkleControl2++;

                  if(EarlyLandingFlag[RIGHT] == 0)// If the right foot is not landed earlyer,
                  {
                  //supporting foot
                  GLOBAL_Xori_LF = Foot_gainLF*(FOGRollVel_NF) + GLOBAL_Xori_LF_last*(1-Foot_gainLF);
                  GLOBAL_Yori_LF = Foot_gainLF*(FOGPitchVel_NF) + GLOBAL_Yori_LF_last*(1-Foot_gainLF);
                  GLOBAL_Xori_LF_last2 = GLOBAL_Xori_LF;
                  GLOBAL_Yori_LF_last2 = GLOBAL_Yori_LF;
                  }

                  //swing foot
                  GLOBAL_Xori_RF = GLOBAL_Xori_RF_last*(1-Foot_gainLF);
                  GLOBAL_Yori_RF = GLOBAL_Yori_RF_last*(1-Foot_gainLF);

                  GLOBAL_Xori_RF2 = Foot_gainLF*(-AngleRoll);// + GLOBAL_Xori_RF2_last*(1-Foot_gainLF);
                  GLOBAL_Yori_RF2 = Foot_gainLF*(-AnglePitch);// + GLOBAL_Yori_RF2_last*(1-Foot_gainLF);

                  GLOBAL_Xori_RF2_last = GLOBAL_Xori_RF2;
                  GLOBAL_Yori_RF2_last = GLOBAL_Yori_RF2;
                  CNT_AnkleControl4 = 0;
              }
              else if(LandingState == DSP)
              {
                  Foot_gainLF = 1 - 0.5*(1 - cos(PI*CNT_AnkleControl4/50));
                  if(CNT_AnkleControl4<50)CNT_AnkleControl4++;

                  if(Pre_LandingState == RSSP)
                  {
       //               Pre_Pre_LandingState = RSSP;
                      GLOBAL_Xori_LF = Foot_gainLF*(GLOBAL_Xori_LF_last2);// when dsp, set Xori to zero
                      GLOBAL_Yori_LF = Foot_gainLF*(GLOBAL_Yori_LF_last2);

                      GLOBAL_Xori_RF2 = Foot_gainLF*(GLOBAL_Xori_RF2_last);
                      GLOBAL_Yori_RF2 = Foot_gainLF*(GLOBAL_Yori_RF2_last);
                  }
                  if(Pre_LandingState == LSSP)
                  {
       //               Pre_Pre_LandingState = LSSP;
                      GLOBAL_Xori_RF = Foot_gainLF*(GLOBAL_Xori_RF_last2);// when dsp, set Xori to zero
                      GLOBAL_Yori_RF = Foot_gainLF*(GLOBAL_Yori_RF_last2);

                      GLOBAL_Xori_LF2 = Foot_gainLF*(GLOBAL_Xori_LF2_last);
                      GLOBAL_Yori_LF2 = Foot_gainLF*(GLOBAL_Yori_LF2_last);
                  }

                  CNT_AnkleControl1 = CNT_AnkleControl2 = CNT_AnkleControl3 = 0;
                  GLOBAL_Xori_LF_last = GLOBAL_Xori_LF;
                  GLOBAL_Yori_LF_last = GLOBAL_Yori_LF;

                  GLOBAL_Xori_RF_last = GLOBAL_Xori_RF;
                  GLOBAL_Yori_RF_last = GLOBAL_Yori_RF;
              }
              else if(LandingState == FINAL)
              {
                  Foot_gainLF = 1 - 0.5*(1 - cos(PI*CNT_AnkleControl3/50));
                  if(CNT_AnkleControl3<50)CNT_AnkleControl3++;

          //        if((GLOBAL_Xori_LF >0.001)||(GLOBAL_Xori_RF >0.001)||(GLOBAL_Yori_LF >0.001)||(GLOBAL_Yori_RF >0.001))
          //        {
                  if(Pre_LandingState == RSSP)
                  {
                  GLOBAL_Xori_LF = Foot_gainLF*(GLOBAL_Xori_LF_last2);
                  GLOBAL_Yori_LF = Foot_gainLF*(GLOBAL_Yori_LF_last2);
                  GLOBAL_Xori_RF2 = Foot_gainLF*(GLOBAL_Xori_RF2_last);
                  GLOBAL_Yori_RF2 = Foot_gainLF*(GLOBAL_Yori_RF2_last);
                  }

                  if(Pre_LandingState == LSSP)
                  {
                  GLOBAL_Xori_RF = Foot_gainLF*(GLOBAL_Xori_RF_last2);
                  GLOBAL_Yori_RF = Foot_gainLF*(GLOBAL_Yori_RF_last2);
                  GLOBAL_Xori_LF2 = Foot_gainLF*(GLOBAL_Xori_LF2_last);
                  GLOBAL_Yori_LF2 = Foot_gainLF*(GLOBAL_Yori_LF2_last);
                  }
          //        }

                  CNT_AnkleControl1 = CNT_AnkleControl2 = 0;

                  GLOBAL_Xori_LF_last = GLOBAL_Xori_LF;
                  GLOBAL_Yori_LF_last = GLOBAL_Yori_LF;

                  GLOBAL_Xori_RF_last = GLOBAL_Xori_RF;
                  GLOBAL_Yori_RF_last = GLOBAL_Yori_RF;

                  GLOBAL_Xori_LF2_last = GLOBAL_Xori_LF2;
                  GLOBAL_Yori_LF2_last = GLOBAL_Yori_LF2;

                  GLOBAL_Xori_RF2_last = GLOBAL_Xori_RF2;
                  GLOBAL_Yori_RF2_last = GLOBAL_Yori_RF2;


              }

    if(EarlyLanding_ONOFF == 0){
    GLOBAL_Z_RF = fsm->RightInfos[0][2];
    GLOBAL_Z_LF = fsm->LeftInfos[0][2];
    }
    fsm_state_timer[fsm->StateInfos[0][0]] += DEL_T;
}
//-----------------------------------------------------------------------------------------------//
void GetGain(double H)
{
    int nCount=0;
    double temp_Gd_gain,temp_Gx_gain,temp_Gi_gain;

    pv_A[0][0] = 1.0f;
    pv_A[0][1] = 0.005f;
    pv_A[0][2] = 0.005f*0.005f;

    pv_A[1][0] = 0.0f;
    pv_A[1][1] = 1.0f;
    pv_A[1][2] = 0.005f;

    pv_A[2][0] = 0.0f;
    pv_A[2][1] = 0.0f;
    pv_A[2][2] = 1.0f;

    pv_B[0][0] = 0.005f*0.005f*0.005f/6.0f;
    pv_B[1][0] = 0.005f*0.005f/2.0f;
    pv_B[2][0] = 0.005f;


    pv_C[0][0] = 1.0f;
    pv_C[0][1] = 0.0f;
    pv_C[0][2] = -H/9.81f;

    pv_Index= 0;

    if(H >= 0.49 && H < 0.51)
        {   //printf("H =0.50 \n");
            fp2 = fopen("../share/Gain/Gd50.txt","r");
            if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
            while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
            fclose(fp2);
            fp3 = fopen("../share/Gain/Gx50.txt","r");nCount = 0;
            if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
            while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
            fclose(fp3);
            fp4 = fopen("../share/Gain/Gi50.txt","r");nCount = 0;
            if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
            while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
            fclose(fp4);
        }
    else if(H >= 0.51 && H < 0.53)
    {   //printf("H =0.52 \n");
        fp2 = fopen("../share/Gain/Gd52.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx52.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi52.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.53 && H < 0.55)
    {   //printf("H =0.54 \n");
        fp2 = fopen("v/Gd54.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx54.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi54.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.55 && H < 0.56)
    {   //printf("H =0.55 \n");
        fp2 = fopen("../share/Gain/Gd55.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx55.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi55.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.56 && H < 0.57)
    {   //printf("H =0.56 \n");
        fp2 = fopen("../share/Gain/Gd56.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx56.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi56.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.57 && H < 0.58)
    {   //printf("H =0.57 \n");
        fp2 = fopen("../share/Gain/Gd57.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx57.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi57.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.58 && H < 0.59)
    {   //printf("H =0.58 \n");
        fp2 = fopen("../share/Gain/Gd58.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx58.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi58.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.59 && H < 0.60)
    {   //printf("H =0.59 \n");
        fp2 = fopen("../share/Gain/Gd59.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx59.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi59.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.60 && H < 0.61)
    {   //printf("H =0.60 \n");
        fp2 = fopen("../share/Gain/Gd60.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx60.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi60.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.61 && H < 0.62)
    {   //printf("H =0.61 \n");
        fp2 = fopen("../share/Gain/Gd61.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx61.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi61.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.62 && H < 0.63)
    {   //printf("H =0.62 \n");
        fp2 = fopen("../share/Gain/Gd62.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx62.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi62.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.63 && H < 0.65)
        {   //printf("H =0.63 \n");
            fp2 = fopen("../share/Gain/Gd63.txt","r");
            if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
            while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
            fclose(fp2);
            fp3 = fopen("../share/Gain/Gx63.txt","r");nCount = 0;
            if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
            while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
            fclose(fp3);
            fp4 = fopen("../share/Gain/Gi63.txt","r");nCount = 0;
            if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
            while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
            fclose(fp4);
        }
    else if(H >= 0.65 && H < 0.66)
    {   //printf("H =0.65 \n");
        fp2 = fopen("../share/Gain/Gd65.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx65.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi65.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.66 && H < 0.67)
    {   //printf("H =0.66 \n");
        fp2 = fopen("../share/Gain/Gd66.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx66.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi66.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.67 && H < 0.68)
    {   //printf("H =0.67 \n");
        fp2 = fopen("../share/Gain/Gd67.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx67.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi67.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.68 && H < 0.69)
    {//printf("H =0.68 \n");
        fp2 = fopen("../share/Gain/Gd68.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx68.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi68.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.69 && H < 0.70)
    {//printf("H =0.69 \n");
        fp2 = fopen("../share/Gain/Gd69.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx69.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi69.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.70 && H < 0.71)
    {//printf("H =0.70 \n");
        fp2 = fopen("../share/Gain/Gd70.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx70.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi70.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.71 && H < 0.72)
    {//printf("H =0.71 \n");
        fp2 = fopen("../share/Gain/Gd71.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx71.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi71.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.72 && H < 0.73)
    {//printf("H =0.72 \n");
        fp2 = fopen("../share/Gain/Gd72.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx72.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi72.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.73 && H < 0.74)
    {//printf("H =0.73 \n");
        fp2 = fopen("../share/Gain/Gd73.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx73.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi73.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.74 && H < 0.75)
    {//printf("H =0.74 \n");
        fp2 = fopen("../share/Gain/Gd74.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx74.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi74.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.75 && H < 0.76)
    {//printf("H =0.75 \n");
        fp2 = fopen("../share/Gain/Gd75.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx75.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi75.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.76 && H < 0.77)
    {//printf("H =0.76 \n");
        fp2 = fopen("../share/Gain/Gd76.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx76.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi76.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.77 && H < 0.78)
    {//printf("H =0.77 \n");
        fp2 = fopen("../share/Gain/Gd77.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx77.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi77.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.78 && H < 0.79)
    {//printf("H =0.78 \n");
        fp2 = fopen("../share/Gain/Gd78.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx78.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi78.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.79 && H < 0.80)
    {//printf("H =0.79 \n");
        fp2 = fopen("../share/Gain/Gd79.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx79.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi79.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.80 && H < 0.81)
    {//printf("H =0.80 \n");
        fp2 = fopen("../share/Gain/Gd80.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx80.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi80.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.81 && H < 0.82)
    {//printf("H =0.81 \n");
        fp2 = fopen("../share/Gain/Gd81.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx81.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi81.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.82 && H < 0.83)
    {//printf("H =0.82 \n");
        fp2 = fopen("../share/Gain/Gd82.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx82.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi82.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.83 && H < 0.84)
    {//printf("H =0.83 \n");
        fp2 = fopen("../share/Gain/Gd83.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx83.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi83.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.84 && H < 0.85)
    {//printf("H =0.84 \n");
        fp2 = fopen("../share/Gain/Gd84.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx84.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi84.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    //else if(H >= 0.85 && H < 0.86)
    else if(H >= 0.85)
    {//printf("H =0.85 \n");
        fp2 = fopen("../share/Gain/Gd85.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx85.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi85.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }

}
//-----------------------------------------------------------------------------------------------//
void JW_INV_MODEL(double Pattern1,double Pattern1_d,double Pattern2,double Pattern2_d)
{
    if(_preview_flag == true && pv_Index == 1){
        JW_InvPattern_U[0]=Pattern1;
        JW_InvPattern_U[1]=Pattern2;

        JW_InvPattern_U_I[0] = 0.;
        JW_InvPattern_U_I[1] = 0.;

        JW_InvPattern_Y_old[0] = Pattern1;
        JW_InvPattern_Y_old[1] = Pattern1_d;

        JW_InvPattern_X_old[0] = Pattern2;
        JW_InvPattern_X_old[1] = Pattern2_d;
        printf("Invmodel reset!!!\n");
    }
        JW_InvPattern_Klqr[0]=30.6386;//20.6386f;//2.3166;// 30.6386;//315.2293f;
        JW_InvPattern_Klqr[1]=0.7508;//0.7508f;//0.1868;//0.7508;//2.4780f;

        JW_InvPattern_U_I[0] +=0.1*(Pattern1-JW_InvPattern_Y_old[0]);
        JW_InvPattern_U[0]   = JW_InvPattern_Klqr[0]*(Pattern1-JW_InvPattern_Y_old[0])+JW_InvPattern_Klqr[1]*(Pattern1_d-JW_InvPattern_Y_old[1])+JW_InvPattern_U_I[0];

        JW_InvPattern_U_I[1] +=0.1*(Pattern2-JW_InvPattern_X_old[0]);
        JW_InvPattern_U[1]   = JW_InvPattern_Klqr[0]*(Pattern2-JW_InvPattern_X_old[0])+JW_InvPattern_Klqr[1]*(Pattern2_d-JW_InvPattern_X_old[1])+JW_InvPattern_U_I[1];


        JW_InvPattern_A[0][0] =0.f;
        JW_InvPattern_A[0][1] =1.0f;
        JW_InvPattern_A[1][0] =-JW_InvPattern_k/JW_InvPattern_m;
        JW_InvPattern_A[1][1] =-JW_InvPattern_c/JW_InvPattern_m;

        JW_InvPattern_A_X[0][0] =0.f;
        JW_InvPattern_A_X[0][1] =1.0f;
        JW_InvPattern_A_X[1][0] =-JW_InvPattern_k_X/JW_InvPattern_m;
        JW_InvPattern_A_X[1][1] =-JW_InvPattern_c_X/JW_InvPattern_m;
    //    if((GLOBAL_Z_LF>0.01)||(GLOBAL_Z_RF>0.01))
    //    {
    //        JW_InvPattern_A[1][0] =-JW_InvPattern_k/JW_InvPattern_m/2;
    //        JW_InvPattern_A[1][1] =-JW_InvPattern_c/JW_InvPattern_m/2;
    //    }

        JW_InvPattern_l = sqrt((userData->WalkReadyCOM[2]+COM_Offset)*(userData->WalkReadyCOM[2]+COM_Offset) + Pattern1*Pattern1);

        if(_preview_flag == true && pv_Index == 1){

            Y_inv = Pattern1;
            Y_inv_d = Pattern1_d;
            theta = atan2(Pattern1,(userData->WalkReadyCOM[2])+COM_Offset);
            theta_d = 0;
            U_I[0] = 0.0f;
            Y_inv_old = Pattern1;
            //printf("Invmodel reset!!!\n");
        }
        U_I[0] +=.1*(Pattern1-Y_inv);
        U[0]   = JW_InvPattern_Klqr[0]*(Pattern1 - Y_inv) + JW_InvPattern_Klqr[1]*(Pattern1_d - Y_inv_d) + U_I[0];

        theta_ref = atan2(U[0],(userData->WalkReadyCOM[2]+COM_Offset));
        if(_preview_flag == true && pv_Index == 1){
            theta_ref =-(((9.81/JW_InvPattern_l*sin(theta))/(1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l))-JW_InvPattern_c*(theta_d))/JW_InvPattern_k - theta) ;//*(*(theta-theta_ref)+);
            //theta_ref = theta;
        }
        theta_dd = 9.81/JW_InvPattern_l*sin(theta)-1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l)*(JW_InvPattern_k*(theta-theta_ref)+JW_InvPattern_c*(theta_d));
        theta_d = theta_d + theta_dd*DEL_T;
        theta   = theta + theta_d*DEL_T;

        //Y_inv_d=Hubo2->WalkReadyCOM[2]/cos(theta)/cos(theta)*theta_d;
        Y_inv_old = Y_inv;
        Y_inv=(userData->WalkReadyCOM[2]+COM_Offset)*tan(theta);
        Y_inv_d = (Y_inv - Y_inv_old)/DEL_T;
        JW_InvPattern_B[0] = 0.f;
        JW_InvPattern_B[1] = JW_InvPattern_k/JW_InvPattern_m;

        JW_InvPattern_B_X[0] = 0.f;
        JW_InvPattern_B_X[1] = JW_InvPattern_k_X/JW_InvPattern_m;

        JW_InvPattern_Y_d[0] = JW_InvPattern_A[0][0]*JW_InvPattern_Y_old[0] + JW_InvPattern_A[0][1]*JW_InvPattern_Y_old[1] + JW_InvPattern_B[0]*JW_InvPattern_U[0];
        JW_InvPattern_Y_d[1] = JW_InvPattern_A[1][0]*JW_InvPattern_Y_old[0] + JW_InvPattern_A[1][1]*JW_InvPattern_Y_old[1] + JW_InvPattern_B[1]*JW_InvPattern_U[0];

        JW_InvPattern_X_d[0] = JW_InvPattern_A_X[0][0]*JW_InvPattern_X_old[0] + JW_InvPattern_A_X[0][1]*JW_InvPattern_X_old[1] + JW_InvPattern_B_X[0]*JW_InvPattern_U[1];
        JW_InvPattern_X_d[1] = JW_InvPattern_A_X[1][0]*JW_InvPattern_X_old[0] + JW_InvPattern_A_X[1][1]*JW_InvPattern_X_old[1] + JW_InvPattern_B_X[1]*JW_InvPattern_U[1];

        JW_InvPattern_Y[0] = JW_InvPattern_Y_old[0] + JW_InvPattern_Y_d[0]*DEL_T;
        JW_InvPattern_Y[1] = JW_InvPattern_Y_old[1] + JW_InvPattern_Y_d[1]*DEL_T;

        JW_InvPattern_X[0] = JW_InvPattern_X_old[0] + JW_InvPattern_X_d[0]*DEL_T;
        JW_InvPattern_X[1] = JW_InvPattern_X_old[1] + JW_InvPattern_X_d[1]*DEL_T;

        JW_InvPattern_Y_old[0] = JW_InvPattern_Y[0];
        JW_InvPattern_Y_old[1] = JW_InvPattern_Y[1];

        JW_InvPattern_X_old[0] = JW_InvPattern_X[0];
        JW_InvPattern_X_old[1] = JW_InvPattern_X[1];

        double Global[3],Local[3];
        Global[0] = JW_InvPattern_U[1];
        Global[1] = 0;
        Global[2] = 0;
        Global2Local2(Global,Local);
        JW_InvPattern_U_n[1] = Local[0];
}
//-----------------------------------------------------------------------------------------------//
void WBIK()
{
    double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4];
    double temp1des_qRF_4x1[4],temp2des_qRF_4x1[4],temp3des_qRF_4x1[4],temp4des_qRF_4x1[4],temp5des_qRF_4x1[4];
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
    double RightYaw,RightRoll,RightPitch,LeftYaw,LeftRoll,LeftPitch;
    static double RightPitch_Torso=0.,RightRoll_Torso=0.;
    static double LeftPitch_Torso=0.,LeftRoll_Torso=0.;
    static double RightPitch_Torso_last=0.,RightRoll_Torso_last=0.;
    static double LeftPitch_Torso_last=0.,LeftRoll_Torso_last=0.;
    static double gain_left=0.,gain_right=0.;
//    double temp_4x1[4];
//    double RightYaw_hat,RightRoll_hat,RightPitch_hat,LeftYaw_hat,LeftRoll_hat,LeftPitch_hat;
    if((fsm->walking_mode == TERRAIN_WALKING)||(fsm->walking_mode == LADDER_WALKING)||(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP))
        LateLanding_ONOFF=1.;
    else LateLanding_ONOFF=1.;

    if(pv_Index == 1){
        LeftPitch_Torso_last = 0.;
        LeftRoll_Torso_last = 0.;
        RightPitch_Torso_last = 0.;
        RightRoll_Torso_last  = 0.;

        Late_Landing_Comp[RIGHT] = 0;
        Late_Landing_Comp[LEFT] = 0;
        Dif_enc_LAP = 0;
        Dif_enc_LAR = 0;
        Dif_enc_RAP = 0;
        Dif_enc_RAR = 0;
        Dif_enc_RAR_last = 0;
        Dif_enc_RAP_last = 0;
        Dif_enc_LAR_last = 0;
        Dif_enc_LAP_last = 0;

        CNT_final_gain_DSP_ZMP_CON = 0;
        CNT_final_gain_SSP_ZMP_CON = 0;
        des_pCOM_3x1_LPF[Zdir] = userData->WalkReadyCOM[Zdir];
        HUBO2ZMPInitLegLength(0.,0.,0);
        Add_FootTask[RIGHT][Zdir] = 0.;
        Add_FootTask[LEFT][Zdir] = 0.;

        HUBO2TorsoInitConRoll(0.0, 0.0, 0);
        HUBO2TorsoInitConPitch(0.0, 0.0, 0);
        TorsoRollAngle=0;
        TorsoPitchAngle=0;

        PitchRoll_Ori_Integral(0.0, 0.0, 0);
        BTW_FOOT_qPEL_comp_4x1[0] = 1;
        BTW_FOOT_qPEL_comp_4x1[1] = 0;
        BTW_FOOT_qPEL_comp_4x1[2] = 0;
        BTW_FOOT_qPEL_comp_4x1[3] = 0;

        kirkZMPCon_XP2(0,0,0);
        kirkZMPCon_YP2(0,0,0);
        Del_PC_X_DSP_XZMP_CON = 0;
        Del_PC_Y_DSP_YZMP_CON = 0;

        kirkZMPCon_XP1(0,0,0);
        kirkZMPCon_YP1(0,0,0);
        Del_PC_X_SSP_XZMP_CON = 0;
        Del_PC_Y_SSP_YZMP_CON = 0;

        I_ZMP_CON_X = 0.0f;
        I_ZMP_CON_Y = 0.0f;

        Dif_enc_RAP = 0;
        Dif_enc_RAR = 0;
        Dif_enc_LAP = 0;
        Dif_enc_LAR = 0;
        HUBO2TorqInitConRAR(0.0, 0.0, 0.0,0.);
        HUBO2TorqInitConLAR(0.0, 0.0, 0.0,0.);
        HUBO2TorqInitConRAP(0.0, 0.0, 0.0,0.);
        HUBO2TorqInitConLAP(0.0, 0.0, 0.0,0.);

        WBIKTorqInitConRAR(0.0, 0.0, 0.0,0.);
        WBIKTorqInitConLAR(0.0, 0.0, 0.0,0.);
        WBIKTorqInitConRAP(0.0, 0.0, 0.0,0.);
        WBIKTorqInitConLAP(0.0, 0.0, 0.0,0.);

        HUBO2_ForceInitCon_RF_X(0.0, 0.0, 0.0);
        HUBO2_ForceInitCon_RF_Y(0.0, 0.0, 0.0);
        HUBO2_ForceInitCon_LF_X(0.0, 0.0, 0.0);
        HUBO2_ForceInitCon_LF_Y(0.0, 0.0, 0.0);
        AnkleRollAngle[HUBO_RIGHT] = 0;
        AnkleRollAngle[HUBO_LEFT] = 0;
        AnklePitchAngle[HUBO_RIGHT] = 0;
        AnklePitchAngle[HUBO_LEFT] = 0;

        WBIK_Torq[RIGHT][Xdir] = 0.;
        WBIK_Torq[RIGHT][Ydir] = 0.;
        WBIK_Torq[LEFT][Xdir] = 0.;
        WBIK_Torq[LEFT][Ydir] = 0.;

        qt_WBIK_Torq_RF[0] = 1.;
        qt_WBIK_Torq_RF[1] = 0.;
        qt_WBIK_Torq_RF[2] = 0.;
        qt_WBIK_Torq_RF[3] = 0.;

        qt_WBIK_Torq_LF[0] = 1.;
        qt_WBIK_Torq_LF[1] = 0.;
        qt_WBIK_Torq_LF[2] = 0.;
        qt_WBIK_Torq_LF[3] = 0.;

        printf("controllers reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    }

    // 1 . PELVIS Orientation
    Pel_Yaw = U3_Gain*(fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.- 1*yaw_angle*D2R;
    qtRZ(Pel_Yaw, temp1des_qPEL_4x1);


    QT2YRP(BTW_FOOT_qPEL_comp_4x1,BTW_YAW,BTW_ROLL,BTW_PITCH);

//    qtRZ(BTW_YAW*D2R,temp1des_qPEL_4x1);
//    qtRX(BTW_ROLL*D2R, temp2des_qPEL_4x1);
//    qtRY(BTW_PITCH*D2R, temp4des_qPEL_4x1);

//    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
//    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);

//    printf("BTW = %f,%f,%f,%f, PEL= %f,%f,%f,%f",BTW_FOOT_qPEL_comp_4x1[0],BTW_FOOT_qPEL_comp_4x1[1],BTW_FOOT_qPEL_comp_4x1[2],BTW_FOOT_qPEL_comp_4x1[3],des_qPEL_4x1[0],des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3]);
//    printf("BTW_YAW = %f,BTW_ROLL = %f,BTW_PITCH = %f,TorsoRollAngle = %f,TorsoPitchAngle = %f \n",BTW_YAW,BTW_ROLL,BTW_PITCH,TorsoRollAngle,TorsoPitchAngle);

    if(TorsoAngle_Comp_ONOFF == false)
    {
        TorsoRollAngle = 0.;
        TorsoPitchAngle = 0.;
    }
    if(Pelvis_Orientation_FeedBack_ONOFF == false)
    {
        BTW_ROLL = 0;
        BTW_PITCH = 0;
    }

    //For Global Frame
    TorsoRollAngle_n  =  (TorsoRollAngle )*cos(Pel_Yaw) + (TorsoPitchAngle )*sin(Pel_Yaw) + BTW_ROLL;
    TorsoPitchAngle_n = -(TorsoRollAngle )*sin(Pel_Yaw) + (TorsoPitchAngle )*cos(Pel_Yaw) + BTW_PITCH;

//    printf("Theta = %f, %f, %f,TorsoRollAngle_n = %f,TorsoPitchAngle_n = %f \n",BTW_YAW,BTW_ROLL,BTW_PITCH,TorsoRollAngle_n,TorsoPitchAngle_n);

    //qtRZ(0, temp1des_qPEL_4x1);

    qtRX(1.0*TorsoRollAngle_n*D2R, temp2des_qPEL_4x1);
    qtRY(1.0*TorsoPitchAngle_n*D2R, temp4des_qPEL_4x1);


    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);
//    printf("qPEL = %f,%f,%f,%f ,",des_qPEL_4x1[0],des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3]);


    Kaj_y = zmpONOFF*ZMPControllerY(GLOBAL_X_LIPM,GLOBAL_Y_LIPM,GLOBAL_X_LIPM_d,GLOBAL_Y_LIPM_d,fsm->ZMPInfos[0][0],fsm->ZMPInfos[0][1],est_x/1000.0,est_x_dot/1000.0,X_ZMP_n/1000.0,est_y/1000.0,est_y_dot/1000.0,Y_ZMP_n/1000.0);
    Kaj_x = zmpONOFF*ZMPControllerX(GLOBAL_X_LIPM,GLOBAL_Y_LIPM,GLOBAL_X_LIPM_d,GLOBAL_Y_LIPM_d,fsm->ZMPInfos[0][0],fsm->ZMPInfos[0][1],est_x/1000.0,est_x_dot/1000.0,X_ZMP_n/1000.0,est_y/1000.0,est_y_dot/1000.0,Y_ZMP_n/1000.0);


    // 2 . COM Position
    if(fsm->walking_mode == TERRAIN_WALKING)
        CONT_X = GLOBAL_X_LIPM_n      +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.0/0.4) + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
    else if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
    {
//        CONT_X = JW_InvPattern_U[1]*U_Gain + (GLOBAL_X_LIPM_n)*(1-U_Gain)      +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.0/0.4) + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
        CONT_X = JW_InvPattern_U_n[1]*U_Gain + (GLOBAL_X_LIPM_n)*(1-U_Gain)      +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.2/0.43*U_Gain_DSP) + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
    }
    else if(fsm->walking_mode == LADDER_WALKING)
        CONT_X = GLOBAL_X_LIPM_n      +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.0/0.4) + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
    else
        CONT_X = zmpONOFF*Kaj_x + GLOBAL_X_LIPM_n      +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.0/0.4) + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
//    CONT_X = GLOBAL_X_LIPM_n   +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4)- 0.001*Del_PC_X_SSP_XZMP_CON*0 + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
    //CONT_Y = ((GLOBAL_Y_LIPM_n-(GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*(1-U_Gain) + U[0]*U_Gain + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0) - 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f + I_ZMP_CON_Y;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];


    if((fsm->walking_mode == TERRAIN_WALKING))
        CONT_Y = (JW_InvPattern_U[0] *U0_Gain       + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
    else if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
        CONT_Y = (JW_InvPattern_U[0] *U0_Gain       + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
    else if(fsm->walking_mode == LADDER_WALKING)
        CONT_Y = (U[0]*U0_Gain                      + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
    else
    {
        if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
        CONT_Y = (JW_InvPattern_U[0] *U0_Gain + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
        else
        CONT_Y = zmpONOFF*Kaj_y+(U[0]*(U0_Gain + U0_Gain_KI*0) + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
    }

    //CONT_Y = (JW_InvPattern_U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
//    CONT_X = GLOBAL_X_LIPM_n;//
//    CONT_Y = GLOBAL_Y_LIPM_n;//

    double Global[3],Local[3] ;
    Local[0]=CONT_X;
    Local[1]=CONT_Y;
    Local[2]=0;
    Local2Global(Local,Global);
    CONT_X_n = Global[0];//CONT_X*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) - CONT_Y*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
    CONT_Y_n = Global[1];//CONT_X*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + CONT_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

    des_pCOM_3x1_hat[Xdir] = CONT_X_n;
    des_pCOM_3x1_hat[Ydir] = CONT_Y_n;
    des_pCOM_3x1_hat[Zdir] = userData->WalkReadyCOM[Zdir] + fsm->AddComInfos[0][2];// + GLOBAL_Z_LIPM;// - (fsm->AddRightFootInfos[0][2] + fsm->AddLeftFootInfos[0][2])*0.7;
    
    double RotX[9],RotY[9],RotZ[9],RotYX[9],TorsoOri[3],TorsoOri_n[3];
    TorsoOri[0] = TorsoRollAngle_n*D2R;
    TorsoOri[1] = TorsoPitchAngle_n*D2R;
    TorsoOri[2] = 0.;

    RZ(U3_Gain*(fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,RotZ);
    mult_mv(RotZ,3,3,TorsoOri,TorsoOri_n);

    RX(TorsoOri_n[0],RotX);
    RY(TorsoOri_n[1],RotY);

    mult_mm(RotY,3,3,RotX,3,RotYX);

    mult_mv(RotYX,3,3,des_pCOM_3x1_hat,des_pCOM_3x1);
//    des_pCOM_3x1[Xdir] = (des_pCOM_3x1_hat[Ydir]*sin(TorsoRollAngle_n*D2R) + des_pCOM_3x1_hat[Zdir]*cos(TorsoRollAngle_n*D2R))*sin(TorsoPitchAngle_n*D2R) + des_pCOM_3x1_hat[Xdir]*cos(TorsoPitchAngle_n*D2R);
//    des_pCOM_3x1[Ydir] =  des_pCOM_3x1_hat[Ydir]*cos(TorsoRollAngle_n*D2R) - des_pCOM_3x1_hat[Zdir]*sin(TorsoRollAngle_n*D2R);
//    des_pCOM_3x1[Zdir] = (des_pCOM_3x1_hat[Ydir]*sin(TorsoRollAngle_n*D2R) + des_pCOM_3x1_hat[Zdir]*cos(TorsoRollAngle_n*D2R))*cos(TorsoPitchAngle_n*D2R) - des_pCOM_3x1_hat[Xdir]*sin(TorsoPitchAngle_n*D2R);

    _last_pCOM_3x1[Xdir] = CONT_X_n;
    _last_pCOM_3x1[Ydir] = CONT_Y_n;
    _last_pCOM_3x1[Zdir] = userData->WalkReadyCOM[Zdir] + fsm->AddComInfos[0][2];// + GLOBAL_Z_LIPM;// - (fsm->AddRightFootInfos[0][2] + fsm->AddLeftFootInfos[0][2])*0.7;


    //================= Right Foot ======================
    // 3 . RF Position
    if(InitializeFLAG == 1)
    {
        des_pRF_3x1_hat[Xdir] = GLOBAL_X_RF;// + HUBO2_ForceInitCon[RIGHT][Xdir];
        des_pRF_3x1_hat[Ydir] = GLOBAL_Y_RF;// + HUBO2_ForceInitCon[RIGHT][Ydir];
    }
    else
    {
        des_pRF_3x1_hat[Xdir] = GLOBAL_X_RF;// + HUBO2_ForceInitCon[RIGHT][Xdir];
        des_pRF_3x1_hat[Ydir] = GLOBAL_Y_RF;// + HUBO2_ForceInitCon[RIGHT][Ydir];
    }
    if((fsm->walking_mode == TERRAIN_WALKING)||(fsm->walking_mode == LADDER_WALKING))
    {
        if(InitializeFLAG == 1)
        des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF + Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + fsm->AddRightFootInfos[0][2]*1  - fsm->AddJointInfos[0][JRAR]/25. -Late_Landing_Comp[RIGHT]*0*LateLanding_ONOFF;//+ fsm->AddRightFootInfos[0][2]/2.;
        else
            des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF + Add_FootTask[RIGHT][Zdir]*0 + fsm->AddRightFootInfos[0][2]*1  - fsm->AddJointInfos[0][JRAR]/25. -Late_Landing_Comp[RIGHT]*0*LateLanding_ONOFF;//+ fsm->AddRightFootInfos[0][2]/2.;
    }
    else if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
    {
        des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF + Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + fsm->AddRightFootInfos[0][2]*1  - fsm->AddJointInfos[0][JRAR]/25. -Late_Landing_Comp[RIGHT]*LateLanding_ONOFF;//+ fsm->AddRightFootInfos[0][2]/2.;
    }
    else
    {
        if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
            des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF + Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + fsm->AddRightFootInfos[0][2]  - fsm->AddJointInfos[0][JRAR]/25. -Late_Landing_Comp[RIGHT]*LateLanding_ONOFF;//+ fsm->AddRightFootInfos[0][2]/2.;
        else
            des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF+ Zctrl*0.5*impONOFF + Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF  -Late_Landing_Comp[RIGHT]*LateLanding_ONOFF;//+ fsm->AddRightFootInfos[0][2]*0.5;//  - fsm->AddJointInfos[0][JRAR]/20.;// -Late_Landing_Comp[RIGHT];//+ fsm->AddRightFootInfos[0][2]/2.;
    }

    mult_mv(RotYX,3,3,des_pRF_3x1_hat,des_pRF_3x1);

//    mat3by3x3by3(double a[3][3],double b[3][3],double out[3][3])
//    printf("IH %f,%f,%f \n", des_pRF_3x1[Xdir],des_pRF_3x1[Ydir],des_pRF_3x1[Zdir]);
//    des_pRF_3x1[Xdir] = (des_pRF_3x1_hat[Ydir]*sin(TorsoRollAngle_n*D2R) + des_pRF_3x1_hat[Zdir]*cos(TorsoRollAngle_n*D2R))*sin(TorsoPitchAngle_n*D2R) + des_pRF_3x1_hat[Xdir]*cos(TorsoPitchAngle_n*D2R);
//    des_pRF_3x1[Ydir] =  des_pRF_3x1_hat[Ydir]*cos(TorsoRollAngle_n*D2R) - des_pRF_3x1_hat[Zdir]*sin(TorsoRollAngle_n*D2R);
//    des_pRF_3x1[Zdir] = (des_pRF_3x1_hat[Ydir]*sin(TorsoRollAngle_n*D2R) + des_pRF_3x1_hat[Zdir]*cos(TorsoRollAngle_n*D2R))*cos(TorsoPitchAngle_n*D2R) - des_pRF_3x1_hat[Xdir]*sin(TorsoPitchAngle_n*D2R);
//    printf("JW %f,%f,%f \n", des_pRF_3x1[Xdir],des_pRF_3x1[Ydir],des_pRF_3x1[Zdir]);
    // 4 . RF Orietation
    double RY,RP,RR;
    double temp_QT[4]={1,0,0,0};
    QTcross(temp_QT,temp2des_qPEL_4x1,temp1des_qPEL_4x1);
    QTcross(temp1des_qPEL_4x1,temp4des_qPEL_4x1,temp3des_qPEL_4x1);
    QT2YPR(temp3des_qPEL_4x1,RY,RP,RR);
//    RP = TorsoPitchAngle_n;
//    RR = TorsoRollAngle_n;


    if(LandingState == LSSP) // Left foot is swing foot
    {
        RightPitch_Torso = RightPitch_Torso_last;
        RightRoll_Torso  = RightRoll_Torso_last;

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->TIME_SSP)
        gain_left = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] - DEL_T)/(fsm->TIME_SSP)));

        LeftPitch_Torso = RP*gain_left + LeftPitch_Torso_last*(1-gain_left);
        LeftRoll_Torso  = RR*gain_left + LeftRoll_Torso_last*(1-gain_left);

    }
    else if(LandingState == RSSP) // Right foot is swing foot
    {
        LeftPitch_Torso = LeftPitch_Torso_last;
        LeftRoll_Torso  = LeftRoll_Torso_last;

        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->TIME_SSP)
        gain_right = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]] - DEL_T)/(fsm->TIME_SSP)));

        RightPitch_Torso = RP*gain_right + RightPitch_Torso_last*(1-gain_right);
        RightRoll_Torso  = RR*gain_right + RightRoll_Torso_last*(1-gain_right);

    }
    else if(LandingState == DSP)
    {
        LeftPitch_Torso_last = LeftPitch_Torso;
        LeftRoll_Torso_last = LeftRoll_Torso;
        RightPitch_Torso_last = RightPitch_Torso;
        RightRoll_Torso_last  = RightRoll_Torso;
    }
//    printf("TorsoPitchAngle_n = %f,TorsoRollAngle_n = %f, YPR = %f,%f,%f\n",TorsoPitchAngle_n,TorsoRollAngle_n,RY,RP,RR);
    RightYaw = fsm->RightInfos[0][3]*D2R;// + fsm->AddJointInfos[0][JRHY]*D2R;
    RightPitch = fsm->RightInfos[0][5]*D2R + RightPitch_Torso*D2R;// + WBIK_Torq[RIGHT][Ydir]*D2R;// + GLOBAL_Yori_RF*D2R*Gyro_Ankle_FeedBack_ONOFF;// + Rori[1]*D2R;//- TorsoPitchAngle_n*D2R;
    RightRoll  = fsm->RightInfos[0][4]*D2R + RightRoll_Torso*D2R;// + WBIK_Torq[RIGHT][Xdir]*D2R;// + GLOBAL_Xori_RF*D2R*Gyro_Ankle_FeedBack_ONOFF;//+ Rori[0]*D2R;//- TorsoRollAngle_n*D2R;

    qtRZ(RightYaw, temp1des_qRF_4x1);
    qtRY(RightPitch, temp4des_qRF_4x1);
    qtRX(RightRoll, temp2des_qRF_4x1);

    QTcross(temp1des_qRF_4x1,temp4des_qRF_4x1,temp3des_qRF_4x1);
    QTcross(temp3des_qRF_4x1,temp2des_qRF_4x1,temp5des_qRF_4x1);
    QTcross(temp5des_qRF_4x1,qt_WBIK_Torq_RF,temp5des_qRF_4x1);

    des_qRF_4x1[0] = temp5des_qRF_4x1[0];
    des_qRF_4x1[1] = temp5des_qRF_4x1[1];
    des_qRF_4x1[2] = temp5des_qRF_4x1[2];
    des_qRF_4x1[3] = temp5des_qRF_4x1[3];
    //================= Left Foot =======================
    // 5 . LF Position
    if(InitializeFLAG == 1)
    {
    des_pLF_3x1_hat[Xdir] = GLOBAL_X_LF ;//+ HUBO2_ForceInitCon[LEFT][Xdir];
    des_pLF_3x1_hat[Ydir] = GLOBAL_Y_LF ;//+ HUBO2_ForceInitCon[LEFT][Ydir];
    }
    else
    {
        des_pLF_3x1_hat[Xdir] = GLOBAL_X_LF;// + HUBO2_ForceInitCon[LEFT][Xdir];
        des_pLF_3x1_hat[Ydir] = GLOBAL_Y_LF;// + HUBO2_ForceInitCon[LEFT][Ydir];
    }
    if((fsm->walking_mode == TERRAIN_WALKING)||(fsm->walking_mode == LADDER_WALKING))
    {
        des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF + Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + fsm->AddLeftFootInfos[0][2]*1  - fsm->AddJointInfos[0][JLAR]/25. -Late_Landing_Comp[LEFT]*0*LateLanding_ONOFF;//  + fsm->AddLeftFootInfos[0][2]/2.;
    }
    else if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)
    {
        des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF + Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + fsm->AddLeftFootInfos[0][2]*1  - fsm->AddJointInfos[0][JLAR]/25. -Late_Landing_Comp[LEFT]*1*LateLanding_ONOFF;//  + fsm->AddLeftFootInfos[0][2]/2.;
    }
    else
    {
        if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
            des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF + Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + fsm->AddLeftFootInfos[0][2]*1  - fsm->AddJointInfos[0][JLAR]/25. -Late_Landing_Comp[LEFT]*LateLanding_ONOFF;//  + fsm->AddLeftFootInfos[0][2]/2.;
        else
            des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF- Zctrl*0.5*impONOFF + Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF -Late_Landing_Comp[LEFT]*LateLanding_ONOFF;//+ fsm->AddLeftFootInfos[0][2]*1.0;//  - fsm->AddJointInfos[0][JLAR]/20.;// -Late_Landing_Comp[LEFT];//  + fsm->AddLeftFootInfos[0][2]/2.;
    }

    mult_mv(RotYX,3,3,des_pLF_3x1_hat,des_pLF_3x1);
//    des_pLF_3x1[Xdir] = (des_pLF_3x1_hat[Ydir]*sin(TorsoRollAngle_n*D2R) + des_pLF_3x1_hat[Zdir]*cos(TorsoRollAngle_n*D2R))*sin(TorsoPitchAngle_n*D2R) + des_pLF_3x1_hat[Xdir]*cos(TorsoPitchAngle_n*D2R);
//    des_pLF_3x1[Ydir] =  des_pLF_3x1_hat[Ydir]*cos(TorsoRollAngle_n*D2R) - des_pLF_3x1_hat[Zdir]*sin(TorsoRollAngle_n*D2R);
//    des_pLF_3x1[Zdir] = (des_pLF_3x1_hat[Ydir]*sin(TorsoRollAngle_n*D2R) + des_pLF_3x1_hat[Zdir]*cos(TorsoRollAngle_n*D2R))*cos(TorsoPitchAngle_n*D2R) - des_pLF_3x1_hat[Xdir]*sin(TorsoPitchAngle_n*D2R);

    // 6 . LF Orietation
    LeftYaw = 1*fsm->LeftInfos[0][3]*D2R ;//+ fsm->AddJointInfos[0][JLHY]*D2R;
    LeftPitch = fsm->LeftInfos[0][5]*D2R + LeftPitch_Torso*D2R;// + WBIK_Torq[LEFT][Ydir]*D2R;// + GLOBAL_Yori_LF*D2R*Gyro_Ankle_FeedBack_ONOFF;//+ Lori[1]*D2R;//-TorsoPitchAngle_n*D2R;
    LeftRoll = fsm->LeftInfos[0][4]*D2R  + LeftRoll_Torso*D2R;// + WBIK_Torq[LEFT][Xdir]*D2R;// + GLOBAL_Xori_LF*D2R*Gyro_Ankle_FeedBack_ONOFF;// + Lori[0]*D2R;//-TorsoRollAngle_n*D2R;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);
    QTcross(temp5des_qLF_4x1,qt_WBIK_Torq_LF,temp5des_qLF_4x1);

    des_qLF_4x1[0] = temp5des_qLF_4x1[0];
    des_qLF_4x1[1] = temp5des_qLF_4x1[1];
    des_qLF_4x1[2] = temp5des_qLF_4x1[2];
    des_qLF_4x1[3] = temp5des_qLF_4x1[3];

    memcpy(WBIK_Q0,WBIK_Q,34*sizeof(double));


//    QTcross(des_qPEL_4x1,BTW_FOOT_qPEL_comp_4x1,temp_4x1);
//    if(Pelvis_Orientation_FeedBack_ONOFF == true)
//        kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, temp_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);
//    else
    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] == OUTSIDE_WALKING)
    {
//        for(int i=0; i<=LAR; i++) {
//            FWRefAngleCurrent2[i] = WBIK_Q[i+7];
//            FWRefAngleCurrent_last[i] = WBIK_Q[i+7];
//        }
//        ANKLE_FREE_LOCK();
    }
    if((fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)||(fsm->walking_mode == TERRAIN_WALKING))
    {
//            for(int i=0; i<=LAR; i++) {
//                FWRefAngleCurrent2[i] = WBIK_Q[i+7];
//                FWRefAngleCurrent_last[i] = WBIK_Q[i+7];
//            }
//            ANKLE_FREE_LOCK();
    }

//    if((ik_rtn.err_code.err_bit.joint_limit == 1)||(ik_rtn.err_code.err_bit.workspace_limit == 1)||(ik_rtn.err_code.err_bit.iteration_limit == 1)||(ik_rtn.err_code.err_bit.ill_quaternion == 1)||(ik_rtn.err_code.err_bit.second_mode == 1))
//    if((ik_rtn.err_code.err_bit.workspace_limit == 1)||(ik_rtn.err_code.err_bit.iteration_limit == 1)||(ik_rtn.err_code.err_bit.ill_quaternion == 1)||(ik_rtn.err_code.err_bit.second_mode == 1))
//    {
//        memcpy(WBIK_Q,WBIK_Q0,34*sizeof(double));
//        fsmFlag = false;
////            if(((ik_rtn.limited_joint_id>>2)&0x01)==0x01){//if(WBIK_Q2[idRHP] < kine_drc_hubo4.lower_limit[idRHP]) _AngleLimit_flag = true;
////                FILE_LOG(logWARNING)<<"Joint limit RHP!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Joint limit RHP!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Joint limit RHP!!!"<<endl;
////            }
////            if(((ik_rtn.limited_joint_id>>3)&0x01)==0x01){//if(WBIK_Q2[idRKN] < kine_drc_hubo4.lower_limit[idRKN]) _LengthLimit_flag = true;
////                FILE_LOG(logWARNING)<<"Leg Length very Long RKN!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Leg Length very Long RKN!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Leg Length very Long RKN!!!"<<endl;
////            }
////            if(((ik_rtn.limited_joint_id>>8)&0x01)==0x01){//if(WBIK_Q2[idLHP] < kine_drc_hubo4.lower_limit[idLHP]) _AngleLimit_flag = true;
////                FILE_LOG(logWARNING)<<"Joint limit LHP!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Joint limit LHP!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Joint limit LHP!!!"<<endl;
////            }
////            if(((ik_rtn.limited_joint_id>>9)&0x01)==0x01){//if(WBIK_Q2[idLKN] < kine_drc_hubo4.lower_limit[idLKN]) _LengthLimit_flag = true;
////                FILE_LOG(logWARNING)<<"Leg Length very Long LKN!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Leg Length very Long LKN!!!"<<endl;
////                FILE_LOG(logWARNING)<<"Leg Length very Long LKN!!!"<<endl;
////            }
//    }

//    double temp[3];
//    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
//    QT2EULER(FK_qRFoot_4x1,temp[0],temp[1],temp[2]);
//    printf("roll = %f,%f,pitch = %f,%f,yaw = %f,%f\n",RightRoll*R2D,temp[0],RightPitch*R2D,temp[1],RightYaw*R2D,temp[2]);

    GLOBAL_Xori_RF_n = GLOBAL_Xori_RF*cos(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*sin(WBIK_Q[idRHY]);
    GLOBAL_Yori_RF_n =-GLOBAL_Xori_RF*sin(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*cos(WBIK_Q[idRHY]);

    GLOBAL_Xori_LF_n = GLOBAL_Xori_LF*cos(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*sin(WBIK_Q[idLHY]);
    GLOBAL_Yori_LF_n =-GLOBAL_Xori_LF*sin(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*cos(WBIK_Q[idLHY]);
    // Sagging Compensation
    for(int i=0; i<=LAR; i++)
    {
        FWRefAngleCurrent[i] = WBIK_Q[i+7]*R2D;

        ///*

        if((fsm->walking_mode == TERRAIN_WALKING)||(fsm->walking_mode == LADDER_WALKING)||(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP))
        {
            if(fabs(fsm->FootUpInfos[0][4])>0.06)
            {
                //0.07,0.07
                printf("side = %f/n",fsm->FootUpInfos[0][4]);
                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0./5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0./5.*Sagging_Comp_ONOFF) - fsm->AddJointInfos[0][JRAR]*(17. +0.0)*Sagging_Comp_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF+ GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF;
                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0./5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0./5.*Sagging_Comp_ONOFF) + fsm->AddJointInfos[0][JLAR]*(17. +0.0)*Sagging_Comp_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF+ GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF;;
            }
            else
            {
//                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*1/5. + fsm->AddJointInfos[0][JLAR]*1/5.) - fsm->AddJointInfos[0][JRAR]*(19. + ((-0.13-GLOBAL_Y_RF)/0.04))+ GLOBAL_Xori_RF*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;
//                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*1/5. + fsm->AddJointInfos[0][JLAR]*1/5.) + fsm->AddJointInfos[0][JLAR]*(19. + (-( 0.13-GLOBAL_Y_LF)/0.04))+ GLOBAL_Xori_LF*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;
                //0.07,0.07
                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0./5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0./5.*Sagging_Comp_ONOFF) - fsm->AddJointInfos[0][JRAR]*(13. +0.0)*Sagging_Comp_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF+ GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF;//
                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0./5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0./5.*Sagging_Comp_ONOFF) + fsm->AddJointInfos[0][JLAR]*(13. +0.0)*Sagging_Comp_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF+ GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF;//
            }

            FWRefAngleCurrent[RHR] = WBIK_Q[RHR+7]*R2D*(1 + fsm->AddJointInfos[0][JRHR]*0.0/5.*Sagging_Comp_ONOFF);//+ fsm->AddJointInfos[0][JRHR];//=-0.2
            FWRefAngleCurrent[LHR] = WBIK_Q[LHR+7]*R2D*(1 + fsm->AddJointInfos[0][JLHR]*0.0/5.*Sagging_Comp_ONOFF);//- fsm->AddJointInfos[0][JLHR];// =-0.2
            //        printf("LAR = %f,RAR = %fLHR = %f,RHR = %f, FOG roll = %f\n",fsm->AddJointInfos[0][JLAR],fsm->AddJointInfos[0][JRAR],fsm->AddJointInfos[0][JLHR],fsm->AddJointInfos[0][JRHR],sharedSEN->FOG.Roll);

            // fsm->AddJointInfos[0][JRAP] =-1.5,fsm->AddJointInfos[0][JLAP] =-1.5
            FWRefAngleCurrent[RAP] = WBIK_Q[RAP+7]*R2D - fsm->AddJointInfos[0][JRAP]*0/5.*Sagging_Comp_ONOFF  + GLOBAL_Yori_RF_n*Gyro_Ankle_FeedBack_ONOFF - AnklePitchAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;//+ AnklePitchAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;// + fsm->AddJointInfos[0][JRAP]/3;// ;// + fsm->AddJointInfos[0][JRAP]*2.5/5. ;//// + TorsoPitchAngle_n;// + AnklePitchAngle;
            FWRefAngleCurrent[LAP] = WBIK_Q[LAP+7]*R2D - fsm->AddJointInfos[0][JLAP]*0/5.*Sagging_Comp_ONOFF  + GLOBAL_Yori_LF_n*Gyro_Ankle_FeedBack_ONOFF - AnklePitchAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;//+ fsm->AddJointInfos[0][JLAP]/3;//
        }
        else
        {
//            printf("fsm->FootUpInfos[0][4] = %f\n",fsm->FootUpInfos[0][4]);
            if(fabs(fsm->FootUpInfos[0][4])>0.05)
            {
                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0/5.*Sagging_Comp_ONOFF) - fsm->AddJointInfos[0][JRAR]*20.*Sagging_Comp_ONOFF + GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;
                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0/5.*Sagging_Comp_ONOFF) + fsm->AddJointInfos[0][JLAR]*20.*Sagging_Comp_ONOFF + GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;
            }
            else
            {
                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0/5.*Sagging_Comp_ONOFF) - fsm->AddJointInfos[0][JRAR]*15.*Sagging_Comp_ONOFF+ GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;
                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*0/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*0/5.*Sagging_Comp_ONOFF) + fsm->AddJointInfos[0][JLAR]*15.*Sagging_Comp_ONOFF+ GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;
            }

            FWRefAngleCurrent[RHR] = WBIK_Q[RHR+7]*R2D*(1 + fsm->AddJointInfos[0][JRHR]*0./5.*Sagging_Comp_ONOFF+ fsm->AddJointInfos[0][JLHR]*0./5.*Sagging_Comp_ONOFF);//+ fsm->AddJointInfos[0][JRHR];//=-0.2
            FWRefAngleCurrent[LHR] = WBIK_Q[LHR+7]*R2D*(1 + fsm->AddJointInfos[0][JRHR]*0./5.*Sagging_Comp_ONOFF+ fsm->AddJointInfos[0][JLHR]*0./5.*Sagging_Comp_ONOFF);//- fsm->AddJointInfos[0][JLHR];// =-0.2
            //        printf("LAR = %f,RAR = %fLHR = %f,RHR = %f, FOG roll = %f\n",fsm->AddJointInfos[0][JLAR],fsm->AddJointInfos[0][JRAR],fsm->AddJointInfos[0][JLHR],fsm->AddJointInfos[0][JRHR],sharedSEN->FOG.Roll);

            // fsm->AddJointInfos[0][JRAP] =-1.5,fsm->AddJointInfos[0][JLAP] =-1.5
            FWRefAngleCurrent[RAP] = WBIK_Q[RAP+7]*R2D + fsm->AddJointInfos[0][JRAP]*0./5.*Sagging_Comp_ONOFF  + GLOBAL_Yori_RF_n*Gyro_Ankle_FeedBack_ONOFF;// - AnklePitchAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;// + fsm->AddJointInfos[0][JRAP]/3;// ;// + fsm->AddJointInfos[0][JRAP]*2.5/5. ;//// + TorsoPitchAngle_n;// + AnklePitchAngle;
            FWRefAngleCurrent[LAP] = WBIK_Q[LAP+7]*R2D + fsm->AddJointInfos[0][JLAP]*0./5.*Sagging_Comp_ONOFF  + GLOBAL_Yori_LF_n*Gyro_Ankle_FeedBack_ONOFF - AnklePitchAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;//+ fsm->AddJointInfos[0][JLAP]/3;//
//            if(fabs(fsm->SideStepLength)>0.05)
//            {
//                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*1/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*1/5.*Sagging_Comp_ONOFF) - fsm->AddJointInfos[0][JRAR]*20.*Sagging_Comp_ONOFF + GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;
//                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*1/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*1/5.*Sagging_Comp_ONOFF) + fsm->AddJointInfos[0][JLAR]*20.*Sagging_Comp_ONOFF + GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;
//            }
//            else
//            {
//                FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*1/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*1/5.*Sagging_Comp_ONOFF) - fsm->AddJointInfos[0][JRAR]*20.*Sagging_Comp_ONOFF+ GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;
//                FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D*(1 + fsm->AddJointInfos[0][JRAR]*1/5.*Sagging_Comp_ONOFF + fsm->AddJointInfos[0][JLAR]*1/5.*Sagging_Comp_ONOFF) + fsm->AddJointInfos[0][JLAR]*20.*Sagging_Comp_ONOFF+ GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF;
//            }

//            FWRefAngleCurrent[RHR] = WBIK_Q[RHR+7]*R2D*(1 + fsm->AddJointInfos[0][JRHR]*1./5.*Sagging_Comp_ONOFF+ fsm->AddJointInfos[0][JLHR]*1./5.*Sagging_Comp_ONOFF);//+ fsm->AddJointInfos[0][JRHR];//=-0.2
//            FWRefAngleCurrent[LHR] = WBIK_Q[LHR+7]*R2D*(1 + fsm->AddJointInfos[0][JRHR]*1./5.*Sagging_Comp_ONOFF+ fsm->AddJointInfos[0][JLHR]*1./5.*Sagging_Comp_ONOFF);//- fsm->AddJointInfos[0][JLHR];// =-0.2
//            //        printf("LAR = %f,RAR = %fLHR = %f,RHR = %f, FOG roll = %f\n",fsm->AddJointInfos[0][JLAR],fsm->AddJointInfos[0][JRAR],fsm->AddJointInfos[0][JLHR],fsm->AddJointInfos[0][JRHR],sharedSEN->FOG.Roll);

//            // fsm->AddJointInfos[0][JRAP] =-1.5,fsm->AddJointInfos[0][JLAP] =-1.5
//            FWRefAngleCurrent[RAP] = WBIK_Q[RAP+7]*R2D + fsm->AddJointInfos[0][JRAP]*0./5.*Sagging_Comp_ONOFF  + GLOBAL_Yori_RF_n*Gyro_Ankle_FeedBack_ONOFF ;//+ AnklePitchAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;// + fsm->AddJointInfos[0][JRAP]/3;// ;// + fsm->AddJointInfos[0][JRAP]*2.5/5. ;//// + TorsoPitchAngle_n;// + AnklePitchAngle;
//            FWRefAngleCurrent[LAP] = WBIK_Q[LAP+7]*R2D + fsm->AddJointInfos[0][JLAP]*0./5.*Sagging_Comp_ONOFF  + GLOBAL_Yori_LF_n*Gyro_Ankle_FeedBack_ONOFF - AnklePitchAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF;//+ fsm->AddJointInfos[0][JLAP]/3;//
        }
        //*/
    }
    FWRefAngleCurrent[WST] = WBIK_Q[idWST]*R2D;
    //printf("%f\n",fsm->AddJointInfos[0][JLHR]);
    for(int i=0; i<=LAR; i++)
    {
        joint->Joints[i]->RefAngleCurrent = FWRefAngleCurrent[i];
    }

    joint->Joints[WST]->RefAngleCurrent = FWRefAngleCurrent[WST] + yaw_angle*1.0f;

//    if(joint->Joints[RKN]->RefAngleCurrent < 5.0) joint->Joints[RKN]->RefAngleCurrent = 5.0f;
//    if(joint->Joints[LKN]->RefAngleCurrent < 5.0) joint->Joints[LKN]->RefAngleCurrent = 5.0f;
//    if(fsm->walking_mode == TERRAIN_WALKING)
//    {joint->Joints[WST]->RefAngleCurrent = FWRefAngleCurrent[WST] + yaw_angle*1.0f;    }
//    else if(fsm->walking_mode == NORMAL_WALKING)
//    {
//        if((userData->G2M.WalkingModeCommand == FORWARD_WALKING)||(userData->G2M.WalkingModeCommand == BACKWARD_WALKING))
//            joint->Joints[WST]->RefAngleCurrent = FWRefAngleCurrent[WST] + yaw_angle*1.0f;
//        else
//            joint->Joints[WST]->RefAngleCurrent = FWRefAngleCurrent[WST] + yaw_angle*0.0f;
//    }
//    else
//        joint->Joints[WST]->RefAngleCurrent = FWRefAngleCurrent[WST] + yaw_angle*0.0f;

}
//-----------------------------------------------------------------------------------------------//
void ANKLE_FREE_LOCK()
{
    static double pre_state = -1.;
    static double temp_Dif_enc_RAP[3] = {0.,},temp_Dif_enc_LAP[3] = {0.,},temp_Dif_enc_RAR[3] = {0.,},temp_Dif_enc_LAR[3] = {0.,};
    static int once_flag[2] ={0};
//    static double EarlyLanding_FT = 100.;

    switch(fsm->StateInfos[0][0])
    {
    case STATE_DSP_INIT_RF:
        RF_FLAG = RF_ANKLE_LOCK;
        LF_FLAG = LF_ANKLE_LOCK;
        break;
    case STATE_DSP_INIT_LF:
        RF_FLAG = RF_ANKLE_LOCK;
        LF_FLAG = LF_ANKLE_LOCK;
        break;
    case STATE_DSP_RF:
        if((LF_FLAG==LF_ANKLE_LOCK)&&(once_flag[LEFT]==0))
        {
            if(fsm->walking_mode==LADDER_WALKING||fsm->walking_mode==TERRAIN_WALKING_ONE_STEP||fsm->walking_mode==TERRAIN_WALKING)
            {
                if((fabs(sharedSEN->FT[LAFT].Mx) >4.)||(fabs(sharedSEN->FT[LAFT].My) >4.)||(sharedSEN->FT[LAFT].Fz >20.))
                {
                    Free_LF_ANK();
                }
            }
            else
            {
                Free_LF_ANK();
            }
        }
        break;
    case STATE_SSP_RF:
        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]/2.0)
        {
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]/2.0,Dif_enc_RAP_last,0.,0.,0.,temp_Dif_enc_RAP);
            Dif_enc_RAP = temp_Dif_enc_RAP[0];

            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]/2.0,Dif_enc_RAR_last,0.,0.,0.,temp_Dif_enc_RAR);
            Dif_enc_RAR = temp_Dif_enc_RAR[0];
            once_flag[RIGHT] = 0;
         }
        else if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*3.95/4.0)
        {
            Dif_enc_RAR_last = 0;
            Dif_enc_RAP_last = 0;
        }
        else
        {
            if((fabs(sharedSEN->FT[RAFT].Mx) >4.)||(fabs(sharedSEN->FT[RAFT].My) >4.)||(sharedSEN->FT[RAFT].Fz >20.))
            {
                    if((RF_FLAG==RF_ANKLE_LOCK)&&(once_flag[RIGHT]==0))Free_RF_ANK();
            }
        }
        pre_state = HUBO_RIGHT;
        break;
    case STATE_DSP_LF:
        if((RF_FLAG==RF_ANKLE_LOCK)&&(once_flag[RIGHT]==0))
        {
            if(fsm->walking_mode==LADDER_WALKING||fsm->walking_mode==TERRAIN_WALKING_ONE_STEP||fsm->walking_mode==TERRAIN_WALKING)
            {
                if((fabs(sharedSEN->FT[RAFT].Mx) >4.)||(fabs(sharedSEN->FT[RAFT].My) >4.)||(sharedSEN->FT[RAFT].Fz >20.))
                {
                    Free_RF_ANK();
                }
            }
            else
            {
                Free_RF_ANK();
            }
        }
        break;
    case STATE_SSP_LF:
        if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]/2.0)
        {
            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]/2.0,Dif_enc_LAP_last,0.,0.,0.,temp_Dif_enc_LAP);
            Dif_enc_LAP = temp_Dif_enc_LAP[0];

            Poly_5th(fsm_state_timer[fsm->StateInfos[0][0]],0.0,fsm->STimeInfos[0]/2.0,Dif_enc_LAR_last,0.,0.,0.,temp_Dif_enc_LAR);
            Dif_enc_LAR = temp_Dif_enc_LAR[0];
            once_flag[LEFT] = 0;
        }
        else if(fsm_state_timer[fsm->StateInfos[0][0]] <= fsm->STimeInfos[0]*3.95/4.0)
        {
            Dif_enc_LAR_last = 0;            Dif_enc_LAP_last = 0;
        }
        else
        {
            if((fabs(sharedSEN->FT[LAFT].Mx) >4.)||(fabs(sharedSEN->FT[LAFT].My) >4.0)||(sharedSEN->FT[LAFT].Fz >20.))
            {
                    if((LF_FLAG==LF_ANKLE_LOCK)&&(once_flag[LEFT]==0))Free_LF_ANK();
            }
        }
        pre_state = HUBO_LEFT;
        break;
    case STATE_DSP_FINAL_RFLF:
        if((LF_FLAG==LF_ANKLE_LOCK)&&(once_flag[LEFT]==0)&&(pre_state == HUBO_LEFT))
        {
            if((fsm->walking_mode==LADDER_WALKING||fsm->walking_mode==TERRAIN_WALKING_ONE_STEP||fsm->walking_mode==TERRAIN_WALKING)&&((fabs(sharedSEN->FT[LAFT].Mx) >4.)||(fabs(sharedSEN->FT[LAFT].My) >4.0)||(sharedSEN->FT[LAFT].Fz >20.)))
            Free_LF_ANK();
        }
        if((RF_FLAG==RF_ANKLE_LOCK)&&(once_flag[RIGHT]==0)&&(pre_state == HUBO_RIGHT))
        {
            if((fsm->walking_mode==LADDER_WALKING||fsm->walking_mode==TERRAIN_WALKING_ONE_STEP||fsm->walking_mode==TERRAIN_WALKING)&&((fabs(sharedSEN->FT[RAFT].Mx) >4.)||(fabs(sharedSEN->FT[RAFT].My) >4.0)||(sharedSEN->FT[RAFT].Fz >20.)))
            Free_RF_ANK();
        }
    case STATE_FINISHED:
        break;
    } // End of Switch

    if((fsm->StateInfos[0][0] == STATE_DSP_RF || fsm->StateInfos[0][0] == STATE_DSP_FINAL_RFLF) &&(LF_FLAG == LF_ANKLE_FREE))
    {
        duty = 0.1*(0-sharedSEN->FT[LAFT].Mx);
        //printf("duty = %d\n",duty);
        //RBJointPWMCommand2ch(CAN1, JMC11,duty,0,0x04);
//        MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 3, duty, 0, 0);
        Get_LF_ENC_Diff();

        if((fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)||(fsm->walking_mode == LADDER_WALKING))
        {
            if(CNT_AfterFree_LF>=20)
            {
                Lock_LF_ANK();
                once_flag[LEFT]=1;
                duty=0;
            }
            if(sharedSEN->FT[LAFT].Fz>7)
            CNT_AfterFree_LF++;
        }
        else
        {
            if(CNT_AfterFree_LF>=30)
            {
                Lock_LF_ANK();
                once_flag[LEFT]=1;
                duty=0;
            }
            CNT_AfterFree_LF++;
        }


    }

    if((fsm->StateInfos[0][0] == STATE_DSP_LF ||fsm->StateInfos[0][0] == STATE_DSP_FINAL_RFLF)&&(RF_FLAG == RF_ANKLE_FREE))
    {
        duty2 = 10*(0-sharedSEN->FT[RAFT].Mx);
        //printf("duty = %d\n",duty);
        //RBJointPWMCommand2ch(CAN0, JMC5,duty2,0,0x04);
        Get_RF_ENC_Diff();

        if((fsm->walking_mode == TERRAIN_WALKING_ONE_STEP)||(fsm->walking_mode == LADDER_WALKING))
        {
            if(CNT_AfterFree_RF>=20)
            {
                Lock_RF_ANK();
                once_flag[RIGHT]=1;
                duty2=0;
            }
            if(sharedSEN->FT[RAFT].Fz>7)
            CNT_AfterFree_RF++;
        }
        else
        {
            if(CNT_AfterFree_RF>=30)
            {
                Lock_RF_ANK();
                once_flag[RIGHT]=1;
                duty2=0;
            }
            CNT_AfterFree_RF++;
        }

    }
    WBIK_Q[LAP+7] = FWRefAngleCurrent2[LAP] + Dif_enc_LAP*D2R;
    WBIK_Q[LAR+7] = FWRefAngleCurrent2[LAR] + Dif_enc_LAR*D2R;
    WBIK_Q[RAP+7] = FWRefAngleCurrent2[RAP] + Dif_enc_RAP*D2R;
    WBIK_Q[RAR+7] = FWRefAngleCurrent2[RAR] + Dif_enc_RAR*D2R;
//    for(int i=0; i<=LAR; i++) {
//        WBIK_Q[i+7] = currentPosition(i)*D2R;
//    }
}
// ----------------------------------------------------------------------------------- //
// ----------------------------------------------------------------------------------- //
void Get_RF_ENC_Diff()
{
    Dif_enc_RAP = currentPosition(RAP) - (userData->ZMPInitAnlge[RAP]  + FWRefAngleCurrent_last[RAP]*R2D + GLOBAL_Yori_RF_n*Gyro_Ankle_FeedBack_ONOFF);
    Dif_enc_RAP_last = Dif_enc_RAP;

    //Dif_enc_RAR = (currentPosition(RAR) - sharedData->ZMPInitAnlge[RAR]+ fsm->AddJointInfos[0][JRAR]*45/3. - GLOBAL_Xori_RF)/(1 + fsm->AddJointInfos[0][JRAR]*1/5. + fsm->AddJointInfos[0][JLAR]*1/5.) - FWRefAngleCurrent_last[RAR]*R2D;// ;
    Dif_enc_RAR = currentPosition(RAR) - (userData->ZMPInitAnlge[RAR]  +  FWRefAngleCurrent_last[RAR]*R2D - fsm->AddJointInfos[0][JRAR]*(10. +0.0)*Sagging_Comp_ONOFF+ GLOBAL_Xori_RF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF);// ;
    Dif_enc_RAR_last = Dif_enc_RAR;
}
void Get_LF_ENC_Diff()
{
    Dif_enc_LAP = currentPosition(LAP)  - (userData->ZMPInitAnlge[LAP]+ FWRefAngleCurrent_last[LAP]*R2D + GLOBAL_Yori_LF_n*Gyro_Ankle_FeedBack_ONOFF - AnklePitchAngle[HUBO_RIGHT]*Ankle_Moment_FeedBack_ONOFF) ;
    Dif_enc_LAP_last = Dif_enc_LAP;

//    Dif_enc_LAR = (currentPosition(LAR)- sharedData->ZMPInitAnlge[LAR] -fsm->AddJointInfos[0][JLAR]*45/3. - GLOBAL_Xori_LF)/(1 + fsm->AddJointInfos[0][JRAR]*1/5. + fsm->AddJointInfos[0][JLAR]*1/5.) - FWRefAngleCurrent_last[LAR]*R2D;// ;
    Dif_enc_LAR = currentPosition(LAR)- (userData->ZMPInitAnlge[LAR]+ FWRefAngleCurrent_last[LAR]*R2D + fsm->AddJointInfos[0][JLAR]*(10. +0.0)*Sagging_Comp_ONOFF+ GLOBAL_Xori_LF_n*Gyro_Ankle_FeedBack_ONOFF + AnkleRollAngle[HUBO_LEFT]*Ankle_Moment_FeedBack_ONOFF);// ;
    Dif_enc_LAR_last = Dif_enc_LAR;
}
void Lock_LF_ANK()
{
//    RBJointPWMCommand2ch(CAN1, JMC11,0,0,0x00);
//    RBenableFrictionCompensation(CAN1,JMC10,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN1,JMC11,DISABLE,DISABLE);

//    RBBoardSetSwitchingMode(CAN1,JMC10, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN1,JMC11, SW_MODE_COMPLEMENTARY);

//    RBJointGainOverride(CAN1,JMC10,1000,1000,1); // Lock Ankle
//    RBJointGainOverride(CAN1,JMC11,1000,1000,10); // Lock Ankle
    MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch,JOINT_INFO[LAP].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 0,200);

    MCenableFrictionCompensation(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LAR].canch,JOINT_INFO[LAR].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 0,200);


    printf(">> Left Ankle Locked!!! at %f,state =  %d  \n",fsm_state_timer[fsm->StateInfos[0][0]],fsm->StateInfos[0][0]);
    LF_FLAG = LF_ANKLE_LOCK;
}
void Lock_RF_ANK()
{
//    RBenableFrictionCompensation(CAN0,JMC4,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN0,JMC5,DISABLE,DISABLE);

//    RBBoardSetSwitchingMode(CAN0,JMC4, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN0,JMC5, SW_MODE_COMPLEMENTARY);

//    RBJointGainOverride(CAN0,JMC4,1000,1000,1); // Lock Ankle
//    RBJointGainOverride(CAN0,JMC5,1000,1000,1); // Lock Ankle
    MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 0,200);

    MCenableFrictionCompensation(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RAR].canch,JOINT_INFO[RAR].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 0,200);
    printf(">> Right Ankle Locked!!! at %f,state =  %d \n",fsm_state_timer[fsm->StateInfos[0][0]],fsm->StateInfos[0][0]);
    RF_FLAG = RF_ANKLE_LOCK;
}
void Free_LF_ANK()
{
//    RBBoardSetSwitchingMode(CAN1,JMC10, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN1,JMC11, SW_MODE_NON_COMPLEMENTARY);

//    RBenableFrictionCompensation(CAN1,JMC10,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN1,JMC11,ENABLE,ENABLE);

//    RBJointGainOverride(CAN1,JMC10,0,1000,1);   // Free Ankle
//    RBJointGainOverride(CAN1,JMC11,0,0,10);      // Free Ankle
    int vel_satu,add_amp;
    if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP||fsm->walking_mode == TERRAIN_WALKING||fsm->walking_mode == LADDER_WALKING) {vel_satu = 700;add_amp=20;}
    else  {vel_satu = 700;add_amp=0;}
    MCsetFrictionParameter(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, vel_satu, 150+add_amp, 0);//150
    MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 100, 1);
    MCsetFrictionParameter(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, vel_satu, 140+add_amp, 0);//140
    MCBoardSetSwitchingMode(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, JOINT_INFO[LAR].mch, 100, 1);


    printf(">> Left Ankle Free!!! at %f,state =  %d  \n",fsm_state_timer[fsm->StateInfos[0][0]],fsm->StateInfos[0][0]);
    LF_FLAG = LF_ANKLE_FREE;
    CNT_AfterFree_LF=0;

}
void Free_RF_ANK()
{
//    RBBoardSetSwitchingMode(CAN0,JMC4, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN0,JMC5, SW_MODE_NON_COMPLEMENTARY);

//    RBenableFrictionCompensation(CAN0,JMC4,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN0,JMC5,ENABLE,ENABLE);

//    RBJointGainOverride(CAN0,JMC4,0,1000,1);    // Free Ankle
//    RBJointGainOverride(CAN0,JMC5,0,0,10);       // Free Ankle
    int vel_satu,add_amp;
    if(fsm->walking_mode == TERRAIN_WALKING_ONE_STEP||fsm->walking_mode == TERRAIN_WALKING||fsm->walking_mode == LADDER_WALKING) {vel_satu = 700;add_amp=20;}
    else  {vel_satu = 700;add_amp=0;}
    MCsetFrictionParameter(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 1000, 150+add_amp, 0);//150
    MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 100, 1);
    MCsetFrictionParameter(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, vel_satu, 90+add_amp, 0);//90
    MCBoardSetSwitchingMode(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, JOINT_INFO[RAR].mch, 100, 1);
    printf(">> Right Ankle Free!!! at %f,state =  %d \n",fsm_state_timer[fsm->StateInfos[0][0]],fsm->StateInfos[0][0]);
    RF_FLAG = RF_ANKLE_FREE;
    CNT_AfterFree_RF=0;

}
double X_ZMP_integral_in_static(int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.001;
//    double totalMx;
    double totalMy;
    double  X_ZMP_static = 0.;//, Y_ZMP_static = 0.;

    if(sharedSEN->FT[RAFT].Fz + sharedSEN->FT[LAFT].Fz > 50.)
    {
//        totalMx = sharedSEN->FT[RAFT].Mx+sharedSEN->FT[LAFT].Mx+(0.5f*kine_drc_hubo4.L_PEL2PEL+0.03)*sharedSEN->FT[LAFT].Fz+(-0.5f*kine_drc_hubo4.L_PEL2PEL+0.03)*sharedSEN->FT[RAFT].Fz;
        totalMy = sharedSEN->FT[RAFT].My+sharedSEN->FT[LAFT].My;

        X_ZMP_static = -totalMy/(sharedSEN->FT[LAFT].Fz+sharedSEN->FT[RAFT].Fz);
//        Y_ZMP_static = totalMx/(sharedSEN->FT[LAFT].Fz+sharedSEN->FT[RAFT].Fz);
    }

    y = 0.0*(0 - X_ZMP_static) + KI*sume;

    sume += 0 - X_ZMP_static;

    if(sume > 7.0/KI) sume = 7./KI;
    else if(sume < -7.0/KI) sume = -7./KI;

    if(y > 20.0) y = 20.0;
    else if(y < -20.0) y = -20.0;

    if(_zero == 0) sume = 0.;

    return y;
}
double Y_ZMP_integral_in_static(int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.001;
    double totalMx;//, totalMy;
    double  Y_ZMP_static = 0.;

    if(sharedSEN->FT[RAFT].Fz + sharedSEN->FT[LAFT].Fz > 50.)
    {
        totalMx = sharedSEN->FT[RAFT].Mx+sharedSEN->FT[LAFT].Mx+(0.5f*kine_drc_hubo4.L_PEL2PEL+0.03)*sharedSEN->FT[LAFT].Fz+(-0.5f*kine_drc_hubo4.L_PEL2PEL+0.03)*sharedSEN->FT[RAFT].Fz;
//        totalMy = sharedSEN->FT[RAFT].My+sharedSEN->FT[LAFT].My;
        Y_ZMP_static = totalMx/(sharedSEN->FT[LAFT].Fz+sharedSEN->FT[RAFT].Fz);
    }

    y = 0.0*(0 - Y_ZMP_static) + KI*sume;

    sume += 0 - Y_ZMP_static;

    if(sume > 7.0/KI) sume = 7./KI;
    else if(sume < -7.0/KI) sume = -7./KI;

    if(y > 20.0) y = 20.0;
    else if(y < -20.0) y = -20.0;

    if(_zero == 0) sume = 0.;

    return y;
}
void get_zmp()
{
    // ZMP 읽기 //
    double r_LF[3],r_RF[3],M_LF[3],M_RF[3],F_LF[3],F_RF[3];
    double rxF_LF[3],rxF_RF[3];
    double _FK_pLFoot_3x1[3],_FK_pRFoot_3x1[3],_FK_qLFoot_4x1[3],_FK_qRFoot_4x1[3];

    kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,_FK_pRFoot_3x1,  _FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,_FK_pLFoot_3x1,  _FK_qLFoot_4x1);
    Foot_angle = atan2((_FK_pRFoot_3x1[0]-_FK_pLFoot_3x1[0]),(fabs(_FK_pLFoot_3x1[1]) + fabs(_FK_pRFoot_3x1[1])));
    //printf(" Foot angle = %f\n",atan2((_FK_pRFoot_3x1[0]-_FK_pLFoot_3x1[0]),(fabs(_FK_pLFoot_3x1[1]) + fabs(_FK_pRFoot_3x1[1])))*R2D);

    double RotX[9],RotY[9],RotZ[9],RotYX[9],TorsoOri[3],TorsoOri_n[3];
    TorsoOri[0] = TorsoRollAngle_n*D2R;
    TorsoOri[1] = TorsoPitchAngle_n*D2R;
    TorsoOri[2] = 0.;

    RZ(U3_Gain*(fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,RotZ);
    mult_mv(RotZ,3,3,TorsoOri,TorsoOri_n);

    RX(TorsoOri_n[0],RotX);
    RY(TorsoOri_n[1],RotY);

    mult_mm(RotY,3,3,RotX,3,RotYX);

    mult_mv(RotYX,3,3,des_pCOM_3x1_hat,des_pCOM_3x1);

    if(sharedSEN->FT[RAFT].Fz + sharedSEN->FT[LAFT].Fz > 50.)
    {
        r_LF[0] =  GLOBAL_X_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        r_LF[1] = -GLOBAL_X_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        r_LF[2] = 0;

        r_RF[0] =  GLOBAL_X_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        r_RF[1] = -GLOBAL_X_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        r_RF[2] = 0;

        M_LF[0] =  sharedSEN->FT[LAFT].Mx*cos((fsm->LeftInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R) - sharedSEN->FT[LAFT].My*sin((fsm->LeftInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R);
        M_LF[1] =  sharedSEN->FT[LAFT].Mx*sin((fsm->LeftInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R) + sharedSEN->FT[LAFT].My*cos((fsm->LeftInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R);
        M_LF[2] = 0;

        M_RF[0] =  sharedSEN->FT[RAFT].Mx*cos((fsm->RightInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R) - sharedSEN->FT[RAFT].My*sin((fsm->RightInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R);
        M_RF[1] =  sharedSEN->FT[RAFT].Mx*sin((fsm->RightInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R) + sharedSEN->FT[RAFT].My*cos((fsm->RightInfos[0][3] - (fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])/2.)*D2R);
        M_RF[2] = 0;

        F_LF[0] = 0;
        F_LF[1] = 0;
        F_LF[2] = sharedSEN->FT[LAFT].Fz;

        F_RF[0] = 0;
        F_RF[1] = 0;
        F_RF[2] = sharedSEN->FT[RAFT].Fz;

        Matrix2Cross3x1_3x1(r_LF,F_LF,rxF_LF);
        Matrix2Cross3x1_3x1(r_RF,F_RF,rxF_RF);

        X_ZMP=(double)1000.*(-rxF_LF[1]-rxF_RF[1]-M_LF[1]-M_RF[1])/(F_LF[2]+F_RF[2]);
        Y_ZMP=(double)1000.*(rxF_LF[0]+rxF_RF[0]+M_LF[0]+M_RF[0])/(F_LF[2]+F_RF[2]);


//        X_ZMP_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*X_ZMP_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*X_ZMP;
//        Y_ZMP_LPF =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_LPF + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP;
        X_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*X_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*X_ZMP;
        Y_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP;


//        X_ZMP_n = X_ZMP_LPF*cos(Foot_angle) + Y_ZMP_LPF*sin(Foot_angle);
//        Y_ZMP_n =-X_ZMP_LPF*sin(Foot_angle) + Y_ZMP_LPF*cos(Foot_angle);

//        Y_ZMP = 1000.*(Hubo2->FTSensor[LAFT].Mx + Hubo2->FTSensor[RAFT].Mx + Hubo2->FTSensor[LAFT].Fz*PELVIS_WIDTH/2.0  - Hubo2->FTSensor[RAFT].Fz*PELVIS_WIDTH/2.0 )/(Hubo2->FTSensor[RAFT].Fz + Hubo2->FTSensor[LAFT].Fz );
//        X_ZMP = 1000.*(-1.0f*Hubo2->FTSensor[LAFT].My -1.0f*Hubo2->FTSensor[RAFT].My + Hubo2->FTSensor[LAFT].Fz*0 + Hubo2->FTSensor[RAFT].Fz*0 )/(Hubo2->FTSensor[RAFT].Fz + Hubo2->FTSensor[LAFT].Fz );

    }


//    Hubo2->ZMP[0]=X_ZMP;
//    Hubo2->ZMP[1]=Y_ZMP;
//    Y_ZMP_IMU = (double)1000.*(des_pCOM_3x1[1] + sharedData->WalkReadyCOM[Zdir]*sharedData->IMUAccY[CIMU]/1000.);
//    Y_ZMP_IMU_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_IMU_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP_IMU;

}
void get_zmp2()
{
        // ZMP 읽기 //
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
        double pCenter[3],qCenter[4],qCenter_bar[4];
        double zmp[3],zmp_local[3],zmp_ref_local[3];

        // Foot Center in Global Coord.
        pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
        pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
        pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
        qtRZ((fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,qCenter);


        if(sharedSEN->FT[RAFT].Fz + sharedSEN->FT[LAFT].Fz > 50.)
        {
            M_LF[0] =  sharedSEN->FT[LAFT].Mx;
            M_LF[1] =  sharedSEN->FT[LAFT].My;
            M_LF[2] =  sharedSEN->FT[LAFT].Mz;

            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedSEN->FT[RAFT].Mx;
            M_RF[1] =  sharedSEN->FT[RAFT].My;
            M_RF[2] =  sharedSEN->FT[RAFT].Mz;

            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = sharedSEN->FT[LAFT].Fx;
            F_LF[1] = sharedSEN->FT[LAFT].Fy;
            F_LF[2] = sharedSEN->FT[LAFT].Fz;

            QTtransform(des_qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = sharedSEN->FT[RAFT].Fx;
            F_RF[1] = sharedSEN->FT[RAFT].Fy;
            F_RF[2] = sharedSEN->FT[RAFT].Fz;

            QTtransform(des_qRF_4x1,F_RF,F_RF_Global);

             double temp1[3],temp2[3],temp3[3],temp4[3];

             diff_vv(des_pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
             diff_vv(des_pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

             cross(1,temp1,F_RF_Global,temp3);// (despRF - pCenter)x(F_RF_Global)
             cross(1,temp2,F_LF_Global,temp4);// (despLF - pCenter)x(F_LF_Global)

             sum_vv(temp3,3,temp4,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
             sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
             sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global

             zmp[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
             zmp[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
             zmp[2] = 0.;

             diff_vv(zmp,3,pCenter,temp1); // zmp - pCenter

             qCenter_bar[0] = qCenter[0];
             qCenter_bar[1] = -qCenter[1];
             qCenter_bar[2] = -qCenter[2];
             qCenter_bar[3] = -qCenter[3];

             QTtransform(qCenter_bar, temp1, zmp_local); // qCenter_bar*(zmp-pCenter)

             temp2[0] = GLOBAL_ZMP_REF_X;
             temp2[1] = GLOBAL_ZMP_REF_Y;
             temp2[2] = 0;
             diff_vv(temp2,3,pCenter,temp1);
             QTtransform(qCenter_bar, temp1, zmp_ref_local);


            X_ZMP_Local = 1000.*zmp_local[0];
            Y_ZMP_Local = 1000.*zmp_local[1];

            X_ZMP_Global = 1000.*zmp[0];
            Y_ZMP_Global = 1000.*zmp[1];

            X_ZMP_REF_Local = zmp_ref_local[0];
            Y_ZMP_REF_Local = zmp_ref_local[1];

            X_ZMP_REF_Global = GLOBAL_ZMP_REF_X;
            Y_ZMP_REF_Global = GLOBAL_ZMP_REF_Y;


//            X_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*X_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*X_ZMP;
//            Y_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP;
        }
}
// ----------------------------------------------------------------------------------- //
void Kirk_Control()
{
    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

//    Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_n - (GLOBAL_ZMP_REF_X_n)*1000 , 1);
//    Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_n - (GLOBAL_ZMP_REF_Y_n)*1000 , 1);
    Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);
//    Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*SwanZMPCon_X(-Del_PC_X_DSP_XZMP_CON, X_ZMP_n - (GLOBAL_ZMP_REF_X_n)*1000 , 1);
//    Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*SwanZMPCon_Y(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_n - (GLOBAL_ZMP_REF_Y_n)*1000 , 1);
//    SwanZMPCon_X

    //printf("final_gain_DSP_ZMP_CON  = %f,zmp error = %f\n",final_gain_DSP_ZMP_CON,Y_ZMP_n - (GLOBAL_ZMP_REF_Y_n)*1000);
    //Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_n, 1);

//            if(_preview_flag == true && pv_Index == 1){

//                Del_PC_X_DSP_XZMP_CON = 0.0f;
//                Del_PC_Y_DSP_YZMP_CON = 0.0f;

//                I_ZMP_CON_X = 0.0f;
//                I_ZMP_CON_Y = 0.0f;

//                Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_n - GLOBAL_ZMP_REF_X_n*1000 , 0);
//                Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_n - GLOBAL_ZMP_REF_Y_n*1000 , 0);
//            }

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

    if(CNT_final_gain_DSP_ZMP_CON < 40) CNT_final_gain_DSP_ZMP_CON++;

    Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
    Old_Del_PC_X_DSP_XZMP_CON2 = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON2 = Del_PC_Y_DSP_YZMP_CON;
    Old_I_ZMP_CON_X = I_ZMP_CON_X;
    Old_I_ZMP_CON_Y = I_ZMP_CON_Y;
}
void Kirk_Control_ssp()
{
    final_gain_SSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_SSP_ZMP_CON/40));

    Del_PC_X_SSP_XZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_XP1(-Del_PC_X_SSP_XZMP_CON, X_ZMP_n - (GLOBAL_ZMP_REF_X_n)*1000 , 1);
    Del_PC_Y_SSP_YZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_YP1(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP_n - (GLOBAL_ZMP_REF_Y_n)*1000 , 1);
    //printf(" Del_PC_Y_SSP_YZMP_CON = %f\n",Del_PC_Y_SSP_YZMP_CON);
    LPF_Del_PC_X_SSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_SSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_SSP_XZMP_CON;
    LPF_Del_PC_Y_SSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_SSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_SSP_YZMP_CON;

    if(CNT_final_gain_SSP_ZMP_CON < 40) CNT_final_gain_SSP_ZMP_CON++;

    Old_Del_PC_X_SSP_XZMP_CON = Del_PC_X_SSP_XZMP_CON;
    Old_Del_PC_Y_SSP_YZMP_CON = Del_PC_Y_SSP_YZMP_CON;
}
void ZMP_intergral_control()
{
//    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_n - (GLOBAL_ZMP_REF_X_n ));//-X_ZMP_n_OFFSET_BP
//    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_n - (GLOBAL_ZMP_REF_Y_n ));

    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;
//    if(_preview_flag == true && pv_Index == 1){
//        I_ZMP_CON_X = 0.0f;
//        I_ZMP_CON_Y = 0.0f;
//    }
}
//-----------------------------------------------------------------------------------------------//

double SwanObserver(double _u, double _ZMP, double *_hat_X)
{
    //*swan's new parameter
    const double A[3][3] = {{0.0, 1.0, 0.0}, {-90.8819, -1.0039, 1.0}, {0.0, 0.0, 0.0}};
    const double B[3] = {0,132.6395,0};
    const double C[3] = {6.4163,0.0623,0};
    const double D = -8.2261;
    //const double Kg[2] = {-0.561351369682899,0.0541756604962248};       // controller pole: ( -5.0 + 0.1i, -5.0 - 0.1i )
    const double Og[3] = {3.4427,14.5665,79.887};          // observer pole: ( -8.0 + 0.1i, -8.0 - 0.1i , -8.0)

    static double hat_x_old[3], hat_x_new[3], TTemp_1[3], TTemp_2[3], TTemp_3[3];
//    double y;

    _u=0.0;

    TTemp_1[0] = A[0][0]*hat_x_old[0] + A[0][1]*hat_x_old[1] + A[0][2]*hat_x_old[2];
    TTemp_1[1] = A[1][0]*hat_x_old[0] + A[1][1]*hat_x_old[1] + A[1][2]*hat_x_old[2];
    TTemp_1[2] = A[2][0]*hat_x_old[0] + A[2][1]*hat_x_old[1] + A[2][2]*hat_x_old[2];

    TTemp_2[0] = B[0]*_u;
    TTemp_2[1] = B[1]*_u;
    TTemp_2[2] = B[2]*_u;

    TTemp_3[0] = Og[0]*(_ZMP - C[0]*hat_x_old[0] - C[1]*hat_x_old[1] - C[2]*hat_x_old[2] -D*_u);
    TTemp_3[1] = Og[1]*(_ZMP - C[0]*hat_x_old[0] - C[1]*hat_x_old[1] - C[2]*hat_x_old[2] -D*_u);
    TTemp_3[2] = Og[2]*(_ZMP - C[0]*hat_x_old[0] - C[1]*hat_x_old[1] - C[2]*hat_x_old[2] -D*_u);


    hat_x_new[0] = hat_x_old[0] + DEL_T*(TTemp_1[0] + TTemp_2[0] + TTemp_3[0]);
    hat_x_new[1] = hat_x_old[1] + DEL_T*(TTemp_1[1] + TTemp_2[1] + TTemp_3[1]);
    hat_x_new[2] = hat_x_old[2] + DEL_T*(TTemp_1[2] + TTemp_2[1] + TTemp_3[2]);

    //y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    hat_x_old[0] = hat_x_new[0];    //*[swan] old state
    hat_x_old[1] = hat_x_new[1];
    hat_x_old[2] = hat_x_new[2];

    _hat_X[0]=hat_x_new[0];
    _hat_X[1]=hat_x_new[1];
    _hat_X[2]=hat_x_new[2];

    return 1.0;
}

//* Disturbance + State Observation
double SwanZMPCon_X(double _u, double _ZMP, int zero)   // u [mm], ZMP [mm]
{
    int i;

    const double A[3][3] = {{0.0, 1.0, 0.0}, {-90.8819, -1.0039, 0.0173}, {0.0, 0.0, 0.0}};
    const double B[3] = {0,132.6395,0};
    const double C[3] = {6.4163,0.0623,0};
    const double D = -8.2261;
    const double Kg[2] = {-0.4966,0.0678};       // controller pole: ( -5.0 + 0.1i, -5.0 - 0.1i )
    const double Og[3] = {3.4427,14.5665,4606.6};          // observer pole: ( -8.0 + 0.1i, -8.0 - 0.1i , -8.0)


    static double hat_x_old[3], hat_x_new[3], TTemp_1[3], TTemp_2[3], TTemp_3[3];
    double y;

    TTemp_1[0] = A[0][0]*hat_x_old[0] + A[0][1]*hat_x_old[1] + A[0][2]*hat_x_old[2];
    TTemp_1[1] = A[1][0]*hat_x_old[0] + A[1][1]*hat_x_old[1] + A[1][2]*hat_x_old[2];
    TTemp_1[2] = A[2][0]*hat_x_old[0] + A[2][1]*hat_x_old[1] + A[2][2]*hat_x_old[2];

    TTemp_2[0] = B[0]*_u;
    TTemp_2[1] = B[1]*_u;
    TTemp_2[2] = B[2]*_u;

    TTemp_3[0] = Og[0]*(_ZMP - C[0]*hat_x_old[0] - C[1]*hat_x_old[1] - C[2]*hat_x_old[2] -D*_u);
    TTemp_3[1] = Og[1]*(_ZMP - C[0]*hat_x_old[0] - C[1]*hat_x_old[1] - C[2]*hat_x_old[2] -D*_u);
    TTemp_3[2] = Og[2]*(_ZMP - C[0]*hat_x_old[0] - C[1]*hat_x_old[1] - C[2]*hat_x_old[2] -D*_u);


    //hat_x_new[0] = hat_x_old[0] + DEL_T*(TTemp_1[0] + TTemp_2[0] + TTemp_3[0]);
    //hat_x_new[1] = hat_x_old[1] + DEL_T*(TTemp_1[1] + TTemp_2[1] + TTemp_3[1]);
    //hat_x_new[2] = hat_x_old[2] + DEL_T*(TTemp_1[2] + TTemp_2[1] + TTemp_3[2]);

    if(zero == 0)
    {
        for(i=0; i<3; i++)
        {
            hat_x_old[i] = 0.;
            hat_x_new[i] = 0.;
            TTemp_1[i] = 0.;
            TTemp_2[i] = 0.;
            TTemp_3[i] = 0.;
        }
    }

    hat_x_new[0] = hat_x_old[0] + DEL_T*(TTemp_1[0] + TTemp_2[0] + TTemp_3[0]);
    hat_x_new[1] = hat_x_old[1] + DEL_T*(TTemp_1[1] + TTemp_2[1] + TTemp_3[1]);
    hat_x_new[2] = hat_x_old[2] + DEL_T*(TTemp_1[2] + TTemp_2[2] + TTemp_3[2]);



    y = Kg[0]*hat_x_new[0] + Kg[1]*hat_x_new[1] + 0.78*0.78/4658.0*hat_x_new[2]*1.0;

    hat_x_old[0] = hat_x_new[0];    //*[swan] old state
    hat_x_old[1] = hat_x_new[1];
    hat_x_old[2] = hat_x_new[2];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}

double SwanZMPCon_Y(double _u, double _ZMP, int zero)   // u [mm], ZMP [mm]
{
    int i;

    const double A[3][3] = {{0.0, 1.0, 0.0}, {-135.6157, -0.9594, 0.0173}, {0.0, 0.0, 0.0}};
    const double B[3] = {0,189.9905,0};
    const double C[3] = {9.1907,0.0595,0};
    const double D = -11.7829;
    const double Kg[2] = {-0.5822,0.0476};       // controller pole: ( -5.0 + 0.1i, -5.0 - 0.1i )
    const double Og[3] = {2.4709,5.5741,3216.0};          // observer pole: ( -8.0 + 0.1i, -8.0 - 0.1i , -8.0)


    static double hat_x_old2[3], hat_x_new2[3], TTemp1[3][3];
    double y;

    TTemp1[0][0] = A[0][0]*hat_x_old2[0] + A[0][1]*hat_x_old2[1] + A[0][2]*hat_x_old2[2];
    TTemp1[0][1] = A[1][0]*hat_x_old2[0] + A[1][1]*hat_x_old2[1] + A[1][2]*hat_x_old2[2];
    TTemp1[0][2] = A[2][0]*hat_x_old2[0] + A[2][1]*hat_x_old2[1] + A[2][2]*hat_x_old2[2];

    TTemp1[1][0] = B[0]*_u;
    TTemp1[1][1] = B[1]*_u;
    TTemp1[1][2] = B[2]*_u;

    TTemp1[2][0] = Og[0]*(_ZMP - C[0]*hat_x_old2[0] - C[1]*hat_x_old2[1] - C[2]*hat_x_old2[2] -D*_u);
    TTemp1[2][1] = Og[1]*(_ZMP - C[0]*hat_x_old2[0] - C[1]*hat_x_old2[1] - C[2]*hat_x_old2[2] -D*_u);
    TTemp1[2][2] = Og[2]*(_ZMP - C[0]*hat_x_old2[0] - C[1]*hat_x_old2[1] - C[2]*hat_x_old2[2] -D*_u);


    if(zero == 0)
    {
        for(i=0; i<3; i++)
        {
            hat_x_old2[i] = 0.;
            hat_x_new2[i] = 0.;
            TTemp1[0][i] = 0.;
            TTemp1[1][i] = 0.;
            TTemp1[2][i] = 0.;
        }
    }

    hat_x_new2[0] = hat_x_old2[0] + DEL_T*(TTemp1[0][0] + TTemp1[1][0] + TTemp1[2][0]);
    hat_x_new2[1] = hat_x_old2[1] + DEL_T*(TTemp1[0][1] + TTemp1[1][1] + TTemp1[2][1]);
    hat_x_new2[2] = hat_x_old2[2] + DEL_T*(TTemp1[0][2] + TTemp1[1][2] + TTemp1[2][2]);

    y = Kg[0]*hat_x_new2[0] + Kg[1]*hat_x_new2[1]  + 0.78*0.78/6671.9*hat_x_new2[2]*1.0;

    hat_x_old2[0] = hat_x_new2[0];    //*[swan] old state
    hat_x_old2[1] = hat_x_new2[1];
    hat_x_old2[2] = hat_x_new2[2];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
//-----------------------------------------------------------------------------------------------//
double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.0204}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0510};
    const double D = -7.6675;
    const double Kg[2] = {-0.1917,0.0976};
    const double Og[2] = {3.5220,1.4988};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-0.8218}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0510};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.2165,0.1117};
//    const double Og[2] = {2.9066,1.3227};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-121.509441687773,-0.686900801711364}};
//    const double B[2] = {0,171.905595852174};
//    const double C[2] = {8.31581491568207,0.0426004533905397};
//    const double D = -10.6613011739514;
//    const double Kg[2] = {-0.561351369682899,0.0541756604962248};
//    const double Og[2] = {1.87687421562180,-6.91634420265651};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    est_x = x_old[0] = x_new[0];
    est_x_dot =  x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_YP2(double u, double ZMP, int zero)
{
    int i;
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.2327}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0765};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.0915,0.1234};
//    const double Og[2] = {2.8458,0.7336};
//    const double Kg[2] = {-0.4152,0.0792};
//

//    const double Og[2] = {2.8405,1.1907};
//

    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.5306}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0765};
    const double D = -7.6675;

    const double Kg[2] = {-0.0809742090339354,	0.107288107510564};
    const double Og[2] = {3.43082537995055,0.723663240519607};

//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-119.405421865731,-0.441579086814448}};
//    const double B[2] = {0,169.208134541865};
//    const double C[2] = {8.18532708084719,0.0273860057510612};
//    const double D = -10.4940090780092;
//    const double Kg[2] = {-0.552014961447503,0.0564891335695250};
//    const double Og[2] = {3.56807911031747,12.8724994839785};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

   // y = Kg[0]*(atan2(GLOBAL_Y_LIPM_n,des_pCOM_3x1[Zdir])*1000-x_new[0]) + Kg[1]*(atan2(GLOBAL_Y_LIPM_d_n,des_pCOM_3x1[Zdir])*1000-x_new[1]);
    y = Kg[0]*(x_new[0]) + Kg[1]*(x_new[1]);

    est_y = x_old[0] = x_new[0];
    est_y_dot = x_old[1] = x_new[1];

    if(y > 80.0) y = 80.0;
    else if(y < -80.0) y = -80.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_YP1(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0,1},{-14.9504066667389	-0.616370808678501}};
    const double B[2] = {0,35.2914483893102};
    const double C[2] = {1.70719953272619,0.0382262996941896};
    const double D = -2.18871734964897;
//    const double Kg[2] = {1.87183004235299,0.492573413240444};
    const double Kg[2] = {0.596733608123605,0.322560555343192};
//    const double Og[2] = {10.2903310784662,47.5060572719318};
    const double Og[2] = {8.40760843975646,26.9490900208963};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_XP1(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0,1},{-42.8368246575059,-0.719099276791585}};
    const double B[2] = {0,71.0432663261910};
    const double C[2] = {3.43666912554807,0.0445973496432212};
    const double D = -4.40598605839496;
    const double Kg[2] = {0.537322920475309,	0.243244738267863};
    const double Og[2] = {5.40216194635022,	16.0426024573112};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
void P1_Y_th_th_d_Observer(double u, double ZMP, int zero)
{
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-26.742818746707, -1.002436477550}};
    const double B[2] = {0.000000000000, 64.361732983099};
    const double C[2] = {1.801767677929, 0.043173232596};
    const double D = -2.771950273737;
    //const double Kg[2] = {-0.275518043483, 0.077648057173};
    const double Og[2] = {11.302299056258, 60.997202161162};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0) {x_old[0] = 0.; x_old[1] = 0.; x_new[0] = 0.; x_new[1] = 0.; Temp_1[0] = 0.; Temp_1[1] = 0.; Temp_2[0] = 0.; Temp_2[1] = 0.; Temp_3[0] = 0.; Temp_3[1] = 0.;}

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

//    Body_Y_th = x_new[0]*0.001;
//    Body_Y_th_d = x_new[1]*0.001;

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];
}
// --------------------------------------------------------------------------------------------- //

void Matrix2Cross3x1_3x1(double A[3], double B[3], double *C)
{
    //double C[3][3];

    C[0]=A[1]*B[2]-A[2]*B[1];
    C[1]=A[2]*B[0]-A[0]*B[2];
    C[2]=A[0]*B[1]-A[1]*B[0];

    //return C;
}
//-----------------------------------------------------------------------------------------------//
void Poly_5th(double _time,double _ti,double _tf,double _pi,double _vi,double _ai,double _pf,double *_info_y)
{
    double coe[6], pow5, pow4, pow3, pow2;
    double _y,_y_d,_y_dd;
    _time=_time-_ti;
    _tf=_tf-_ti;

    pow5 = _tf*_tf*_tf*_tf*_tf;//pow(_tf, 5);
    pow4 = _tf*_tf*_tf*_tf;//pow(_tf, 4);
    pow3 = _tf*_tf*_tf;//pow(_tf, 3);
    pow2 = _tf*_tf;//pow(_tf, 2);

    coe[0]= (double)(-6./pow5*_pi -3./pow4*_vi -1./2./pow3*_ai +6./pow5*_pf);
    coe[1] = (double)(15./pow4*_pi +8./pow3*_vi +3./2./pow2*_ai -15./pow4*_pf);
    coe[2] = (double)(-10./pow3*_pi -6./pow2*_vi -3./2./_tf*_ai +10./pow3*_pf);
    coe[3] = (double)(1./2.*_ai);
    coe[4] = (double)(1.*_vi);
    coe[5] = (double)(_pi);

    pow5 = _time*_time*_time*_time*_time;//pow(_time, 5);
    pow4 = _time*_time*_time*_time;//pow(_time, 4);
    pow3 = _time*_time*_time;//pow(_time, 3);
    pow2 = _time*_time;//pow(_time, 2);

    _y      = coe[0]*pow5+coe[1]*pow4+coe[2]*pow3+coe[3]*pow2+coe[4]*(_time)+coe[5];
    _y_d    = 5.*coe[0]*pow4+4.*coe[1]*pow3+3.*coe[2]*pow2+2.*coe[3]*_time+coe[4];
    _y_dd   = 4.*5.*coe[0]*pow3+3.*4.*coe[1]*pow2+2.*3.*coe[2]*_time+2.*coe[3];

    _info_y[0] = _y;
    _info_y[1] = _y_d;
    _info_y[2] = _y_dd;
    //return _y;
}
//-----------------------------------------------------------------------------------------------//
double HUBO2ZMPInitLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.000007;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

    if(sume > 0.08/KI) sume = .08/KI;
    else if(sume < -0.08/KI) sume = -.08/KI;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;
//    printf("Angle = %f,sume = %f,y = %f\n", _force,sume,y);

    return y;
}
//-----------------------------------------------------------------------------------------------//
double HUBO2ZMPInitLegLength2(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.000018;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

    if(sume > 0.05/KI) sume = .05/KI;
    else if(sume < -0.05/KI) sume = -.05/KI;

    if(y > 0.05) y = 0.05;
    else if(y < -0.05) y = -0.05;

    if(_zero == 0) sume = 0.;

    return y;
}
double HUBO2_ForceInitCon_RF_Y(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.0000009;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;


    if(sume > 0.03/KI) sume = .03/KI;
    else if(sume < -0.03/KI) sume = -.03/KI;

    if(y > 0.03) y = 0.03;
    else if(y < -0.03) y = -0.03;

    if(_zero == 0) sume = 0.;

    return y;
}
double HUBO2_ForceInitCon_RF_X(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.0000009;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;


    if(sume > 0.03/KI) sume = .03/KI;
    else if(sume < -0.03/KI) sume = -.03/KI;

    if(y > 0.03) y = 0.03;
    else if(y < -0.03) y = -0.03;

    if(_zero == 0) sume = 0.;

    return y;
}
double HUBO2_ForceInitCon_LF_Y(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.0000009;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;


    if(sume > 0.03/KI) sume = .03/KI;
    else if(sume < -0.03/KI) sume = -.03/KI;

    if(y > 0.03) y = 0.03;
    else if(y < -0.03) y = -0.03;

    if(_zero == 0) sume = 0.;

    return y;
}
double HUBO2_ForceInitCon_LF_X(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.0000009;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;


    if(sume > 0.03/KI) sume = .03/KI;
    else if(sume < -0.03/KI) sume = -.03/KI;

    if(y > 0.03) y = 0.03;
    else if(y < -0.03) y = -0.03;

    if(_zero == 0) sume = 0.;

    return y;
}
double HUBO2TorqInitConRAR(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
//    const double KI = 0.00019;
    static double KI=0;
    KI = _gain*1.0;


//    KI = fabs(_ref-_torque)/70.*0.00019*3.;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;


    if(y > 30.0) y = 30.0;
    else if(y < -30.0) y = -30.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double HUBO2TorqInitConLAR(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
    //    const double KI = 0.00019;
        static double KI=0;
        KI = _gain*1.0;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;

    if(y > 30.0) y = 30.0;
    else if(y < -30.0) y = -30.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double HUBO2TorqInitConRAP(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
    //    const double KI = 0.0011;
        static double KI=0;
        KI = _gain;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;
    if(y > 10.0) y = 10.0;
    else if(y < -10.0) y = -10.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double HUBO2TorqInitConLAP(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
    //    const double KI = 0.0011;
        static double KI=0;
        KI = _gain;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;
    if(y > 10.0) y = 10.0;
    else if(y < -10.0) y = -10.0;

    if(_zero == 0) sume = 0.;

    return y;
}
double WBIKTorqInitConRAR(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
//    const double KI = 0.00019;
    static double KI=0;
    KI = _gain*1.0;


//    KI = fabs(_ref-_torque)/70.*0.00019*3.;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;


    if(y > 30.0) y = 30.0;
    else if(y < -30.0) y = -30.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double WBIKTorqInitConLAR(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
    //    const double KI = 0.00019;
        static double KI=0;
        KI = _gain*1.0;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;

    if(y > 30.0) y = 30.0;
    else if(y < -30.0) y = -30.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double WBIKTorqInitConRAP(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
    //    const double KI = 0.0011;
        static double KI=0;
        KI = _gain;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;
    if(y > 10.0) y = 10.0;
    else if(y < -10.0) y = -10.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double WBIKTorqInitConLAP(double _ref, double _torque, int _zero, double _gain)
{
    static double y;
    static double sume = 0.;
    //    const double KI = 0.0011;
        static double KI=0;
        KI = _gain;

    y = 0.0*(_ref - _torque) + KI*sume;

    sume += _ref - _torque;
    if(y > 10.0) y = 10.0;
    else if(y < -10.0) y = -10.0;

    if(_zero == 0) sume = 0.;

    return y;
}
// ----------------------------------------------------------------------------------- //
double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.11;
    const double KD =  0.8;
    static double angle_last =0.;

    y = 4.0*(_ref - _angle) + KI*sume + KD*(_angle - angle_last);

    sume += _ref - _angle;

//    if(sume >0.005*D2R/KI) sume = 0.005*D2R/KI;
//    else if(sume < -0.005*D2R/KI) sume = -0.005*D2R/KI;

    if(y > 100.0*D2R) y = 100.0*D2R;
    else if(y < -100.0*D2R) y = -100.0*D2R;

    if(_zero == 0)
    {
        sume = 0.;
        y=0.;
    }

    angle_last = y;

    return y;
}
// ----------------------------------------------------------------------------------- //
double HUBO2TorsoInitConPitch(double _ref, double _angle, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.03f;
    const double KD =  0.6;
    static double angle_last =0.;

    y = 3.7*(_ref - _angle) + KI*sume + KD*(_angle - angle_last);

//    y = 0.0*(_ref - _angle) + KI*sume;

    sume += _ref - _angle;

    if(sume > 15.0/KI) sume = 15./KI;
    else if(sume < -15.0/KI) sume = -15./KI;

    if(y > 15.0) y = 15.0;
    else if(y < -15.0) y = -15.0;

    if(_zero == 0) {
        sume = 0.;
        y=0;
    }

    angle_last = y;

    return y;
}
// ----------------------------------------------------------------------------------- //
double HUBO2TorsoInitConRoll(double _ref, double _angle, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.01f;
    const double KD =  0.6;
    static double angle_last =0.;

    y = 3.1*(_ref - _angle) + KI*sume + KD*(_angle - angle_last);

//    y = 0.0*(_ref - _angle) + KI*sume;

    sume += _ref - _angle;

    if(sume > 15.0/KI) sume = 15./KI;
    else if(sume < -15.0/KI) sume = -15./KI;

    if(y > 15.0) y = 15.0;
    else if(y < -15.0) y = -15.0;

    if(_zero == 0) {
        sume = 0.;
        y=0.;
    }

    angle_last = y;
    return y;
}

//int	PushCANMessage(MANUAL_CAN MCData){
//    for(int i=0; i<MAX_MANUAL_CAN; i++){
//        if(sharedData->ManualCAN[i].status == MANUALCAN_EMPTY){
//            sharedData->ManualCAN[i].status = MANUALCAN_WRITING;
//            sharedData->ManualCAN[i].channel = MCData.channel;
//            sharedData->ManualCAN[i].id = MCData.id;
//            sharedData->ManualCAN[i].dlc = MCData.dlc;
//            for(int j=0; j<MCData.dlc; j++){
//                sharedData->ManualCAN[i].data[j] = MCData.data[j];
//            }
//            sharedData->ManualCAN[i].status = MANUALCAN_NEW;
//            return RB_SUCCESS;
//        }
//    }
//    cout << "Fail to send Manual CAN..!!" << endl;
//    return RB_FAIL;
//}

//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration){
//    // MsgID                Byte0	Byte1	Byte2	Byte3	Byte4		Byte5
//    // CANID_SEND_CMD		BNO		0x6F	OVER1	OVER2	DURATION	DURATION
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 6;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x6F;
//    MCData.data[2] = _override1;
//    MCData.data[3] = _override2;
//    MCData.data[4] = (_duration & 0xFF);
//    MCData.data[5] = ((_duration>>8) & (0xFF));

//    return PushCANMessage(MCData);
//}

//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode){
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0x13	_mode
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x13;
//    MCData.data[2] = _mode;

//    return PushCANMessage(MCData);
//}

//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2){
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0xB1	ENABLE
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0xB1;
//    MCData.data[2] = (_enable1&0x01) | ((_enable2&0x01)<<1);

//    return PushCANMessage(MCData);
//}
//int RBJointPWMCommand2ch(unsigned int _canch, unsigned int _bno,int _duty1, int _duty2, int _zeroduty)
//{
//    // MsgID                Byte0	Byte1	Byte2		Byte3	Byte4	Byte5	Byte6
//    // CANID_SEND_CMD		BNO		0x0D	_zeroduty	DIR0	DUTY0	DIR1	DUTY1

//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.data[0] = _bno;							// board no.
//    MCData.data[1] = 0x0D;							// command
//    MCData.data[2] = _zeroduty;					// Zero Duty
//    // _zeroduty = 0x00 : Enforce zero duty to stop motor. Back EMF will break the motor
//    // _zeroduty = 0x01 : Pulse out to run motor in specified PWM duty and direction
//    if(_duty1 >= 0)  MCData.data[3] = 0x00;		// 1st motor direction : CCW
//    else MCData.data[3] = 0x01;					// 1st motor direction : CW
//    MCData.data[4] = (unsigned char)(abs(_duty1));	// 1st motor duty
//    if(_duty2 >= 0)  MCData.data[5] = 0x00;		// 2nd motor direction : CCW
//    else MCData.data[5] = 0x01;					// 2nd motor direction : CW
//    MCData.data[6] = (unsigned char)(abs(_duty2));	// 2nd motor duty

//    MCData.dlc = 7;
//    MCData.id = COMMAND_CANID;

//    return PushCANMessage(MCData);
//}
//int RBJointSetControlMode(unsigned int _canch, unsigned int _bno,int _mode)
//{
//    // MsgID                Byte0	Byte1	Byte2
//    // CANID_SEND_CMD		BNO		0x10	_mode

//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;

//    MCData.data[0] = _bno;		// board no.
//    MCData.data[1] = 0x10;		// command
//    MCData.data[2] = _mode;	// control mode
//    // _mode = 0x00 : position control mode
//    // _mode = 0x01 : current control mode

//    MCData.dlc = 3;
//    MCData.id = COMMAND_CANID;

//    return PushCANMessage(MCData);
//}
// ----------------------------------------------------------------------------------- //

void JW_save()
{
//    if(sJW_Data_Debug_Index < ROW_data_debug)
//    {

//        if(fsm->LeftInfos.size() > 1){
//            JW_Data_Debug[0][sJW_Data_Debug_Index] = Late_Landing_Comp[RIGHT];//BTW_PEL_angle_roll;//fsm->StateInfos[0][0];//GLOBAL_Z_LIPM;//des_pCOM_3x1[0];//fsm->AddJointInfos[0][JRHR];//U_I[0];//Del_PC_Y_DSP_YZMP_CON;//LPF_GLOBAL_Xori_LF;//FWRefAngleCurrent2[LAR];//fw_RF_3x1[0];//fsm->LeftInfos[0][0];
//            JW_Data_Debug[1][sJW_Data_Debug_Index] = Late_Landing_Comp[LEFT];//BTW_PEL_angle_pitch;//GLOBAL_Z_LIPM2;// des_pCOM_3x1[1];//fsm->AddJointInfos[0][JRAR];//U[0];//GLOBAL_Xori_LF;//Dif_enc_LAR_last;//Del_PC_X_DSP_XZMP_CON;//LPF_GLOBAL_Xori_LF2;//FWRefAngleCurrent_last[LAR];//Recover_EarlyLandingFlag[LEFT];//fw_RF_3x1[1];//fsm->LeftInfos[0][1];
//            JW_Data_Debug[2][sJW_Data_Debug_Index] = yaw_angle;//des_pCOM_3x1[2];//fsm->AddJointInfos[0][JRAP];//theta_ref;//GLOBAL_Yori_RF;//FWRefAngleCurrent2[LAR];//I_ZMP_CON_X;//LPF_GLOBAL_Yori_LF;//FWRefAngleCurrent[LAR];//w_RF_3x1[2];//fsm->LeftInfos[0][2];

//            JW_Data_Debug[3][sJW_Data_Debug_Index] = (double)((unsigned char)(sharedData->TemperatureVoltage[23]))/4.0;//pv_Index;//Pel_Yaw;//HUBO2_ForceInitCon[RIGHT][Xdir];//des_pCOM_3x1_LPF[Zdir];//des_qPEL_4x1[0];//fsm->AddJointInfos[0][JLHR];//theta_dd;//GLOBAL_Yori_LF;//Dif_enc_LAR*R2D;//fw_LF_3x1[0];//fsm->RightInfos[0][0];
//            JW_Data_Debug[4][sJW_Data_Debug_Index] = _preview_flag;//yaw_angle_last;//HUBO2_ForceInitCon[RIGHT][Ydir];//GLOBAL_X_LIPM;//des_qPEL_4x1[1];//fsm->AddJointInfos[0][JLAR];//theta_d;//GLOBAL_ZMP_REF_X;//currentPosition(LAR);//fw_LF_3x1[1];//fsm->RightInfos[0][1];
//            JW_Data_Debug[5][sJW_Data_Debug_Index] = HUBO2_ForceInitCon[LEFT][Xdir];//GLOBAL_X_LIPM_n;//des_qPEL_4x1[2];//fsm->AddJointInfos[0][JLAP];//theta;//GLOBAL_ZMP_REF_Y;//userData->ZMPInitAnlge[LAR];// fsm->RightInfos[0][2];
//            JW_Data_Debug[6][sJW_Data_Debug_Index] = GLOBAL_Y_LIPM_d_n;//HUBO2_ForceInitCon[LEFT][Ydir];//CONT_X_n;//des_qPEL_4x1[3];//Y_inv;//RF_FLAG;//fsm->AddJointInfos[0][JLAR];//fsm->ZMPInfos[0][0];

//            JW_Data_Debug[7][sJW_Data_Debug_Index] = ((GLOBAL_Y_LIPM_n-(GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0) + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0) - 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f + I_ZMP_CON_Y;

//            JW_Data_Debug[8][sJW_Data_Debug_Index] = GLOBAL_X_RF;
//            JW_Data_Debug[9][sJW_Data_Debug_Index] = GLOBAL_Y_RF;
//            JW_Data_Debug[10][sJW_Data_Debug_Index] = GLOBAL_Z_RF;

//            JW_Data_Debug[11][sJW_Data_Debug_Index] = GLOBAL_X_LF;
//            JW_Data_Debug[12][sJW_Data_Debug_Index] = GLOBAL_Y_LF;
//            JW_Data_Debug[13][sJW_Data_Debug_Index] = GLOBAL_Z_LF;

//            JW_Data_Debug[14][sJW_Data_Debug_Index] = CONT_Y;
//            JW_Data_Debug[15][sJW_Data_Debug_Index] = GLOBAL_Y_LIPM;
//            JW_Data_Debug[16][sJW_Data_Debug_Index] = (U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0);

//            JW_Data_Debug[17][sJW_Data_Debug_Index] = X_ZMP/1000;
//            JW_Data_Debug[18][sJW_Data_Debug_Index] = Y_ZMP/1000;

//            JW_Data_Debug[19][sJW_Data_Debug_Index] = GLOBAL_Y_LIPM_n;//Add_COMTask_FootPrint[0][Ydir];//temp_IK_return>>4;//fsm->ITimeInfos[0];
//            JW_Data_Debug[20][sJW_Data_Debug_Index] = U_Gain;//Add_FootTask_FootPrint[RIGHT][Ydir];//I_ZMP_CON_X;//fsm->STimeInfos[0];
//            JW_Data_Debug[21][sJW_Data_Debug_Index] = U[0];//Add_FootTask_FootPrint[LEFT][Ydir];//I_ZMP_CON_Y;//fsm->StateInfos[0][0];


//            JW_Data_Debug[22][sJW_Data_Debug_Index] = GLOBAL_X_LIPM;//Add_COMTask_FootPrint[0][Ydir];//temp_IK_return>>4;//fsm->ITimeInfos[0];
//            JW_Data_Debug[23][sJW_Data_Debug_Index] = GLOBAL_X_LIPM_n;//Add_FootTask_FootPrint[RIGHT][Ydir];//I_ZMP_CON_X;//fsm->STimeInfos[0];
//            JW_Data_Debug[24][sJW_Data_Debug_Index] = CONT_Y_n;//

//            //JW_Data_Debug[22][sJW_Data_Debug_Index] = thread_time;

//            JW_Data_Debug[25][sJW_Data_Debug_Index] = GLOBAL_Y_LIPM_n - (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.;//LandingState;//currentPosition(LAP);
//            JW_Data_Debug[26][sJW_Data_Debug_Index] = JW_InvPattern_U[1];//des_pCOM_3x1_hat[Xdir];//FWRefAngleCurrent[LAP];
//            JW_Data_Debug[27][sJW_Data_Debug_Index] = JW_InvPattern_U_n[1];//des_pCOM_3x1_hat[Ydir];//Dif_enc_LAP_last;
//            JW_Data_Debug[28][sJW_Data_Debug_Index] = JW_InvPattern_U[1]*U_Gain + GLOBAL_X_LIPM_n*(1-U_Gain);//RF_FLAG;//des_pCOM_3x1_hat[Zdir];//Dif_enc_LAP;
//            JW_Data_Debug[29][sJW_Data_Debug_Index] = (- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.0/0.4) + I_ZMP_CON_X*1.);//LF_FLAG;//joint->Joints[LAP]->RefAngleCurrent;

//            JW_Data_Debug[30][sJW_Data_Debug_Index] = sharedData->IMUPitch[CIMU];
//            JW_Data_Debug[31][sJW_Data_Debug_Index] = sharedData->IMURoll[CIMU];

//            JW_Data_Debug[32][sJW_Data_Debug_Index] = sharedData->IMUPitchVel[CIMU];
//            JW_Data_Debug[33][sJW_Data_Debug_Index] = sharedData->IMURollVel[CIMU];

//            JW_Data_Debug[34][sJW_Data_Debug_Index] = sharedData->IMUAccX[CIMU]*9.81/1000.;
//            JW_Data_Debug[35][sJW_Data_Debug_Index] = sharedData->IMUAccY[CIMU]*9.81/1000.;

//            JW_Data_Debug[36][sJW_Data_Debug_Index] = des_pCOM_3x1[0] + userData->WalkReadyCOM[Zdir]*sharedData->IMUAccX[CIMU]/1000.;
//            JW_Data_Debug[37][sJW_Data_Debug_Index] = Y_ZMP_IMU;

//            JW_Data_Debug[38][sJW_Data_Debug_Index] = sharedData->IMUAccY[CIMU]*9.81/1000.*DEL_T;
//            JW_Data_Debug[39][sJW_Data_Debug_Index] = EarlyLandingFlag[RIGHT];
//            JW_Data_Debug[40][sJW_Data_Debug_Index] = EarlyLandingFlag[LEFT];

//            JW_Data_Debug[41][sJW_Data_Debug_Index] = TorsoRollAngle;//Recover_EarlyLandingFlag[RIGHT];
//            JW_Data_Debug[42][sJW_Data_Debug_Index] = TorsoPitchAngle;//Recover_EarlyLandingFlag[LEFT];

//            JW_Data_Debug[43][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_Y_n;//Ground_RF;
//            JW_Data_Debug[44][sJW_Data_Debug_Index] = U3_Gain;//Ground_LF;

//            JW_Data_Debug[45][sJW_Data_Debug_Index] = BTW_PEL_angle_roll*R2D;
//            JW_Data_Debug[46][sJW_Data_Debug_Index] = BTW_PEL_angle_pitch*R2D;

//            JW_Data_Debug[47][sJW_Data_Debug_Index] = sharedData->FOGYaw;//Estimated_GLOBAL_Z_RF;
//            JW_Data_Debug[48][sJW_Data_Debug_Index] = sharedData->FOGYawVel;//Estimated_GLOBAL_Z_LF;

//            JW_Data_Debug[49][sJW_Data_Debug_Index] = sharedSEN->FOG.Roll;//COMPEN_RHR;
//            JW_Data_Debug[50][sJW_Data_Debug_Index] = sharedSEN->FOG.Pitch;//COMPEN_LHR;

//            JW_Data_Debug[51][sJW_Data_Debug_Index] = sharedSEN->FOG.RollVel;
//            JW_Data_Debug[52][sJW_Data_Debug_Index] = sharedSEN->FOG.PitchVel;

//            JW_Data_Debug[53][sJW_Data_Debug_Index] = SSP_FLAG;

//            JW_Data_Debug[54][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_X_n;
//            JW_Data_Debug[55][sJW_Data_Debug_Index] = X_ZMP_n;
//            JW_Data_Debug[56][sJW_Data_Debug_Index] = CONT_X;//LF_FZ_LPF;

//            JW_Data_Debug[57][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_Y_n;
//            JW_Data_Debug[58][sJW_Data_Debug_Index] = Y_ZMP_n;
//            JW_Data_Debug[59][sJW_Data_Debug_Index] = CONT_Y;//LF_FZ_LPF;
//            JW_Data_Debug[60][sJW_Data_Debug_Index] = currentPosition(WST);//(U[0]*0.98 + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain);//currentPosition(RHP);


//            JW_Data_Debug[61][sJW_Data_Debug_Index] = currentPosition(RHY);//_temp_debug_data[5];
//            JW_Data_Debug[62][sJW_Data_Debug_Index] = currentPosition(RHR);//_temp_debug_data[6];
//            JW_Data_Debug[63][sJW_Data_Debug_Index] = currentPosition(RHP);//_temp_debug_data[7];
//            JW_Data_Debug[64][sJW_Data_Debug_Index] = currentPosition(RKN);
//            JW_Data_Debug[65][sJW_Data_Debug_Index] = currentPosition(RAP);
//            JW_Data_Debug[66][sJW_Data_Debug_Index] = currentPosition(RAR);

//            JW_Data_Debug[67][sJW_Data_Debug_Index] = currentPosition(LHY);
//            JW_Data_Debug[68][sJW_Data_Debug_Index] = currentPosition(LHR);
//            JW_Data_Debug[69][sJW_Data_Debug_Index] = currentPosition(LHP);
//            JW_Data_Debug[70][sJW_Data_Debug_Index] = currentPosition(LKN);
//            JW_Data_Debug[71][sJW_Data_Debug_Index] = currentPosition(LAP);
//            JW_Data_Debug[72][sJW_Data_Debug_Index] = currentPosition(LAR);

//            JW_Data_Debug[73][sJW_Data_Debug_Index] = G_DSP_X;//fsm->AddComInfos[0][2];
//            JW_Data_Debug[74][sJW_Data_Debug_Index] = G_DSP_Y;//fsm->VirtualComInfos[0][1];

//            JW_Data_Debug[75][sJW_Data_Debug_Index] = 0.001*Del_PC_X_SSP_XZMP_CON; //_temp_debug_data[5];
//            JW_Data_Debug[76][sJW_Data_Debug_Index] = 0.001*Del_PC_Y_SSP_YZMP_CON;//_temp_debug_data[6];

//            JW_Data_Debug[77][sJW_Data_Debug_Index] = I_ZMP_CON_X; //_temp_debug_data[5];
//            JW_Data_Debug[78][sJW_Data_Debug_Index] = I_ZMP_CON_Y;//_temp_debug_data[6];


//            JW_Data_Debug[79][sJW_Data_Debug_Index] = 0.001*Del_PC_X_DSP_XZMP_CON;//_temp_debug_data[1];
//            JW_Data_Debug[80][sJW_Data_Debug_Index] = 0.001*Del_PC_Y_DSP_YZMP_CON;//_temp_debug_data[3];


//            JW_Data_Debug[81][sJW_Data_Debug_Index] = GLOBAL_Xori_LF;//_temp_debug_data[6];
//            JW_Data_Debug[82][sJW_Data_Debug_Index] = GLOBAL_Yori_LF;//_temp_debug_data[8];


//            JW_Data_Debug[83][sJW_Data_Debug_Index2] = GLOBAL_Xori_RF;//FWRefAngleCurrent[RHY];
//            JW_Data_Debug[84][sJW_Data_Debug_Index2] = GLOBAL_Yori_RF;//FWRefAngleCurrent[RHR];
//            JW_Data_Debug[85][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHP];
//            JW_Data_Debug[86][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RKN];
//            JW_Data_Debug[87][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RAP];
//            JW_Data_Debug[88][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RAR];

//            JW_Data_Debug[89][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHY];
//            JW_Data_Debug[90][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHR];
//            JW_Data_Debug[91][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHP];
//            JW_Data_Debug[92][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LKN];
//            JW_Data_Debug[93][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LAP];
//            JW_Data_Debug[94][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LAR];

//            JW_Data_Debug[95][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JRAR];//FWRefAngleCurrent[LAP];
//            JW_Data_Debug[96][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLAR];//FWRefAngleCurrent[LAR];

//            JW_Data_Debug[97][sJW_Data_Debug_Index2] = AnkleRollAngle[HUBO_RIGHT];//_temp_debug_data[4];//FWRefAngleCurrent[LAR];
//            JW_Data_Debug[98][sJW_Data_Debug_Index2] = AnkleRollAngle[HUBO_LEFT];//_temp_debug_data[9];//FWRefAngleCurrent[LAR];
//            JW_Data_Debug[99][sJW_Data_Debug_Index2] = AnklePitchAngle[HUBO_RIGHT];//_temp_debug_data[9];//FWRefAngleCurrent[LAR];

//            //JW_Data_Debug[29][sJW_Data_Debug_Index] =
//            //JW_Data_Debug[30][sJW_Data_Debug_Index] =




////            JW_Data_Debug[18][sJW_Data_Debug_Index] = fsm_state_timer[0];
////            JW_Data_Debug[19][sJW_Data_Debug_Index] = fsm_state_timer[1];
////            JW_Data_Debug[20][sJW_Data_Debug_Index] = fsm_state_timer[2];
////            JW_Data_Debug[21][sJW_Data_Debug_Index] = fsm_state_timer[3];
////            JW_Data_Debug[22][sJW_Data_Debug_Index] = fsm_state_timer[4];
////            JW_Data_Debug[23][sJW_Data_Debug_Index] = fsm_state_timer[5];
////            JW_Data_Debug[24][sJW_Data_Debug_Index] = fsm_state_timer[6];
////            JW_Data_Debug[25][sJW_Data_Debug_Index] = fsm_state_timer[7];
////            JW_Data_Debug[26][sJW_Data_Debug_Index] = fsm_state_timer[8];


////            JW_Data_Debug[15][sJW_Data_Debug_Index] = Y_ZMP;
////            JW_Data_Debug[16][sJW_Data_Debug_Index] = Y_ZMP;
////            JW_Data_Debug[17][sJW_Data_Debug_Index] = Y_ZMP;
////            JW_Data_Debug[18][sJW_Data_Debug_Index] = Y_ZMP;

////            printf("%f, %f\n",fsm->LeftFoot->Pos[0], fsm->LeftFoot->Pos[1]);
////            printf("%f, %f\n",fsm->LeftInfos[0][0], fsm->LeftFoot->Pos[0]);

////            JW_Data_Debug[0][sJW_Data_Debug_Index] = fsm->LeftFoot->Pos[0];
////            JW_Data_Debug[1][sJW_Data_Debug_Index] = fsm->LeftFoot->Pos[1];
////            JW_Data_Debug[2][sJW_Data_Debug_Index] = fsm->RightFoot->Pos[0];
////            JW_Data_Debug[3][sJW_Data_Debug_Index] = fsm->RightFoot->Pos[1];
////            JW_Data_Debug[4][sJW_Data_Debug_Index] = fsm->ZMPPos->ZMP[0];
////            JW_Data_Debug[5][sJW_Data_Debug_Index] = fsm->ZMPPos->ZMP[1];
////            JW_Data_Debug[6][sJW_Data_Debug_Index] = fsm->LeftFoot->Pos[2];
////            JW_Data_Debug[7][sJW_Data_Debug_Index] = fsm->RightFoot->Pos[2];
////            JW_Data_Debug[8][sJW_Data_Debug_Index] = fsm->LeftInfos[0][2];
////            JW_Data_Debug[9][sJW_Data_Debug_Index] = fsm->RightInfos[0][2];
//            //printf("%f, %f\n",fsm->LeftFoot->Pos[0], fsm->LeftFoot->Pos[1]);
//            sJW_Data_Debug_Index++;
//            if(sJW_Data_Debug_Index >= ROW_data_debug) sJW_Data_Debug_Index = 0;
//        }
//    }
}

// --------------------------------------------------------------------------------------------- //
void JW_save2()
{
//    if(sJW_Data_Debug_Index2 < ROW_data_debug)
//    {
//        JW_Data_Debug2[0][sJW_Data_Debug_Index2] = fsm->LeftInfos[0][0];
//        JW_Data_Debug2[1][sJW_Data_Debug_Index2] = fsm->LeftInfos[0][1];
//        JW_Data_Debug2[2][sJW_Data_Debug_Index2] = fsm->LeftInfos[0][2];

//        JW_Data_Debug2[3][sJW_Data_Debug_Index2] = fsm->RightInfos[0][0];
//        JW_Data_Debug2[4][sJW_Data_Debug_Index2] = fsm->RightInfos[0][1];
//        JW_Data_Debug2[5][sJW_Data_Debug_Index2] = fsm->RightInfos[0][2];

//        JW_Data_Debug2[6][sJW_Data_Debug_Index2] = fsm->LeftInfos[0][3];
//        JW_Data_Debug2[7][sJW_Data_Debug_Index2] = fsm->LeftInfos[0][4];
//        JW_Data_Debug2[8][sJW_Data_Debug_Index2] = fsm->LeftInfos[0][5];

//        JW_Data_Debug2[9][sJW_Data_Debug_Index2] = fsm->RightInfos[0][3];
//        JW_Data_Debug2[10][sJW_Data_Debug_Index2] = fsm->RightInfos[0][4];
//        JW_Data_Debug2[11][sJW_Data_Debug_Index2] = fsm->RightInfos[0][5];

////        JW_Data_Debug2[0][sJW_Data_Debug_Index2] = TorsoRollAngle;//fsm->AddJointInfos[0][JRAR];
////        JW_Data_Debug2[1][sJW_Data_Debug_Index2] = TorsoPitchAngle;//fsm->AddJointInfos[0][JLAR];
////        JW_Data_Debug2[2][sJW_Data_Debug_Index2] = (U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0);//fsm->AddJointInfos[0][JRAP];
////        JW_Data_Debug2[3][sJW_Data_Debug_Index2] = Del_PC_Y_DSP_YZMP_CON*G_DSP_Y;//fsm->AddJointInfos[0][JLAP];
////        JW_Data_Debug2[4][sJW_Data_Debug_Index2] = I_ZMP_CON_Y;//fsm->AddJointInfos[0][JRHR];
////        JW_Data_Debug2[5][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLHR];
////        JW_Data_Debug2[6][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JRHP];
////        JW_Data_Debug2[7][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLHP];

////        JW_Data_Debug2[4][sJW_Data_Debug_Index2] = WBIK_Q[idX];
////        JW_Data_Debug2[5][sJW_Data_Debug_Index2] = WBIK_Q[idY];
////        JW_Data_Debug2[6][sJW_Data_Debug_Index2] = WBIK_Q[idZ];
////        JW_Data_Debug2[7][sJW_Data_Debug_Index2] = 1000;//fsm->AddLeftFootInfos[0][2];

////        JW_Data_Debug2[8][sJW_Data_Debug_Index2] = WBIK_Q[RHY+7];
////        JW_Data_Debug2[9][sJW_Data_Debug_Index2] = WBIK_Q[RHR+7];
////        JW_Data_Debug2[10][sJW_Data_Debug_Index2] = WBIK_Q[RHP+7];


////        JW_Data_Debug2[11][sJW_Data_Debug_Index2] = WBIK_Q[RKN+7];
//        JW_Data_Debug2[12][sJW_Data_Debug_Index2] = fsm->ZMPInfos[0][0];
//        JW_Data_Debug2[13][sJW_Data_Debug_Index2] = fsm->ZMPInfos[0][1];

//        JW_Data_Debug2[14][sJW_Data_Debug_Index2] = fsm->LeftInfosVel[0][0];
//        JW_Data_Debug2[15][sJW_Data_Debug_Index2] = fsm->LeftInfosVel[0][1];
//        JW_Data_Debug2[16][sJW_Data_Debug_Index2] = WBIK_Q[LHP+7];
//        JW_Data_Debug2[17][sJW_Data_Debug_Index2] = WBIK_Q[LKN+7];
//        JW_Data_Debug2[18][sJW_Data_Debug_Index2] = WBIK_Q[LAR+7];
//        JW_Data_Debug2[19][sJW_Data_Debug_Index2] = WBIK_Q[LAP+7];

//        JW_Data_Debug2[21][sJW_Data_Debug_Index2] = des_pCOM_3x1[0];
//        JW_Data_Debug2[22][sJW_Data_Debug_Index2] = des_pCOM_3x1[1];
//        JW_Data_Debug2[23][sJW_Data_Debug_Index2] = des_pCOM_3x1[2];

//        //JW_Data_Debug2[24][sJW_Data_Debug_Index2] = des_pPCz;

//        JW_Data_Debug2[24][sJW_Data_Debug_Index2] = des_qPEL_4x1[0];
//        JW_Data_Debug2[25][sJW_Data_Debug_Index2] = des_qPEL_4x1[1];
//        JW_Data_Debug2[26][sJW_Data_Debug_Index2] = des_qPEL_4x1[2];
//        JW_Data_Debug2[27][sJW_Data_Debug_Index2] = des_qPEL_4x1[3];

//        JW_Data_Debug2[28][sJW_Data_Debug_Index2] = des_pRF_3x1[0];
//        JW_Data_Debug2[29][sJW_Data_Debug_Index2] = des_pRF_3x1[1];
//        JW_Data_Debug2[30][sJW_Data_Debug_Index2] = des_pRF_3x1[2];

//        JW_Data_Debug2[31][sJW_Data_Debug_Index2] = des_qRF_4x1[0];
//        JW_Data_Debug2[32][sJW_Data_Debug_Index2] = des_qRF_4x1[1];
//        JW_Data_Debug2[33][sJW_Data_Debug_Index2] = des_qRF_4x1[2];
//        JW_Data_Debug2[34][sJW_Data_Debug_Index2] = des_qRF_4x1[3];

//        JW_Data_Debug2[35][sJW_Data_Debug_Index2] = des_pLF_3x1[0];
//        JW_Data_Debug2[36][sJW_Data_Debug_Index2] = des_pLF_3x1[1];
//        JW_Data_Debug2[37][sJW_Data_Debug_Index2] = des_pLF_3x1[2];

//        JW_Data_Debug2[38][sJW_Data_Debug_Index2] = des_qLF_4x1[0];
//        JW_Data_Debug2[39][sJW_Data_Debug_Index2] = des_qLF_4x1[1];
//        JW_Data_Debug2[40][sJW_Data_Debug_Index2] = des_qLF_4x1[2];
//        JW_Data_Debug2[41][sJW_Data_Debug_Index2] = des_qLF_4x1[3];

////        JW_Data_Debug2[12][sJW_Data_Debug_Index2] = sharedData->Joint[LAR].CurrentPosition;//sharedData->ZMP[2];
////        JW_Data_Debug2[13][sJW_Data_Debug_Index2] = sharedData->ZMP[3];
////        JW_Data_Debug2[14][sJW_Data_Debug_Index2] = sharedData->ZMP[4];
////        JW_Data_Debug2[15][sJW_Data_Debug_Index2] = sharedData->ZMP[5];

//        JW_Data_Debug2[42][sJW_Data_Debug_Index2] = sharedData->IMUPitch[CIMU];
//        JW_Data_Debug2[43][sJW_Data_Debug_Index2] = sharedData->IMURoll[CIMU];

//        JW_Data_Debug2[44][sJW_Data_Debug_Index2] = sharedData->IMUPitchVel[CIMU];
//        JW_Data_Debug2[45][sJW_Data_Debug_Index2] = sharedData->IMURollVel[CIMU];

//        JW_Data_Debug2[46][sJW_Data_Debug_Index2] = sharedData->IMUAccX[CIMU];
//        JW_Data_Debug2[47][sJW_Data_Debug_Index2] = sharedData->IMUAccY[CIMU];
//        JW_Data_Debug2[48][sJW_Data_Debug_Index2] = joint->Joints[WST]->RefAngleCurrent ;//sharedData->IMUAccZ[CIMU]*9.81/1000.;

//        JW_Data_Debug2[49][sJW_Data_Debug_Index2] = joint->Joints[RHY]->RefAngleCurrent ;
//        JW_Data_Debug2[50][sJW_Data_Debug_Index2] = joint->Joints[RHR]->RefAngleCurrent ;
//        JW_Data_Debug2[51][sJW_Data_Debug_Index2] = joint->Joints[RHP]->RefAngleCurrent ;
//        JW_Data_Debug2[52][sJW_Data_Debug_Index2] = joint->Joints[RKN]->RefAngleCurrent ;
//        JW_Data_Debug2[53][sJW_Data_Debug_Index2] = joint->Joints[RAP]->RefAngleCurrent ;
//        JW_Data_Debug2[54][sJW_Data_Debug_Index2] = joint->Joints[RAR]->RefAngleCurrent ;

//        JW_Data_Debug2[55][sJW_Data_Debug_Index2] = joint->Joints[LHY]->RefAngleCurrent ;
//        JW_Data_Debug2[56][sJW_Data_Debug_Index2] = joint->Joints[LHR]->RefAngleCurrent ;
//        JW_Data_Debug2[57][sJW_Data_Debug_Index2] = joint->Joints[LHP]->RefAngleCurrent ;
//        JW_Data_Debug2[58][sJW_Data_Debug_Index2] = joint->Joints[LKN]->RefAngleCurrent ;
//        JW_Data_Debug2[59][sJW_Data_Debug_Index2] = joint->Joints[LAP]->RefAngleCurrent ;
//        JW_Data_Debug2[60][sJW_Data_Debug_Index2] = joint->Joints[LAR]->RefAngleCurrent ;

//        JW_Data_Debug2[61][sJW_Data_Debug_Index2] = sharedSEN->FT[RAFT].Fz;
//        JW_Data_Debug2[62][sJW_Data_Debug_Index2] = sharedSEN->FT[RAFT].Mx;
//        JW_Data_Debug2[63][sJW_Data_Debug_Index2] = sharedSEN->FT[RAFT].My;

//        JW_Data_Debug2[64][sJW_Data_Debug_Index2] = sharedSEN->FT[LAFT].Fz;
//        JW_Data_Debug2[65][sJW_Data_Debug_Index2] = sharedSEN->FT[LAFT].Mx;
//        JW_Data_Debug2[66][sJW_Data_Debug_Index2] = sharedSEN->FT[LAFT].My;

//        JW_Data_Debug2[67][sJW_Data_Debug_Index2] = X_ZMP;
//        JW_Data_Debug2[68][sJW_Data_Debug_Index2] = Y_ZMP;

//        JW_Data_Debug2[69][sJW_Data_Debug_Index2] = X_ZMP_Global;
//        JW_Data_Debug2[70][sJW_Data_Debug_Index2] = Y_ZMP_Global;

//        JW_Data_Debug2[71][sJW_Data_Debug_Index2] = X_ZMP_Local;
//        JW_Data_Debug2[72][sJW_Data_Debug_Index2] = Y_ZMP_Local;

//        JW_Data_Debug2[73][sJW_Data_Debug_Index2] = X_ZMP_REF_Global;
//        JW_Data_Debug2[74][sJW_Data_Debug_Index2] = Y_ZMP_REF_Global;

//        JW_Data_Debug2[75][sJW_Data_Debug_Index2] = X_ZMP_REF_Local;
//        JW_Data_Debug2[76][sJW_Data_Debug_Index2] = Y_ZMP_REF_Local;

////        JW_Data_Debug2[69][sJW_Data_Debug_Index2] = GLOBAL_ZMP_REF_X_n;//GLOBAL_X_LIPM_n;
////        JW_Data_Debug2[70][sJW_Data_Debug_Index2] = GLOBAL_ZMP_REF_Y_n;//(U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0);
////        JW_Data_Debug2[71][sJW_Data_Debug_Index2] = CONT_X_n;
////        JW_Data_Debug2[72][sJW_Data_Debug_Index2] = CONT_Y_n;
////        JW_Data_Debug2[73][sJW_Data_Debug_Index2] = userData->WalkReadyCOM[Zdir] + GLOBAL_Z_LIPM + fsm->AddComInfos[0][2];;
////        JW_Data_Debug2[74][sJW_Data_Debug_Index2] = FOGRollVel_NF2;//GLOBAL_X_RF;
////        JW_Data_Debug2[75][sJW_Data_Debug_Index2] = FOGRollVel_LPF;//GLOBAL_Y_RF;
////        JW_Data_Debug2[76][sJW_Data_Debug_Index2] = sharedSEN->FOG.RollVel;//GLOBAL_Z_RF;
////        JW_Data_Debug2[77][sJW_Data_Debug_Index2] = FOGPitchVel_NF2;//GLOBAL_X_LF;
////        JW_Data_Debug2[78][sJW_Data_Debug_Index2] = FOGPitchVel_LPF;//GLOBAL_Y_LF;
////        JW_Data_Debug2[79][sJW_Data_Debug_Index2] = sharedSEN->FOG.PitchVel;//GLOBAL_Z_LF;


////        JW_Data_Debug2[80][sJW_Data_Debug_Index2] = BTW_YAW;//temp_debug[0];//_legpos[0][0];
////        JW_Data_Debug2[81][sJW_Data_Debug_Index2] = BTW_ROLL;//temp_debug[1];//_legpos[0][1];
////        JW_Data_Debug2[82][sJW_Data_Debug_Index2] = BTW_PITCH;//fsm->VirtualComInfos[0][1];
////        JW_Data_Debug2[83][sJW_Data_Debug_Index2] = fsm->AddComInfos[0][2];
////        JW_Data_Debug2[84][sJW_Data_Debug_Index2] = des_pCOM_3x1[2];

////        JW_Data_Debug2[69][sJW_Data_Debug_Index2] = _temp_debug_data[0];//GLOBAL_X_LIPM_n;
////        JW_Data_Debug2[70][sJW_Data_Debug_Index2] = _temp_debug_data[1];//(U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0);
////        JW_Data_Debug2[71][sJW_Data_Debug_Index2] = _temp_debug_data[2];
////        JW_Data_Debug2[72][sJW_Data_Debug_Index2] = _temp_debug_data[3];
////        JW_Data_Debug2[73][sJW_Data_Debug_Index2] = _temp_debug_data[4];
////        JW_Data_Debug2[74][sJW_Data_Debug_Index2] = _temp_debug_data[5];//FOGRollVel_NF2;//GLOBAL_X_RF;
////        JW_Data_Debug2[75][sJW_Data_Debug_Index2] = _temp_debug_data[6];//FOGRollVel_LPF;//GLOBAL_Y_RF;
////        JW_Data_Debug2[76][sJW_Data_Debug_Index2] = _temp_debug_data[7];//sharedSEN->FOG.RollVel;//GLOBAL_Z_RF;
//        JW_Data_Debug2[77][sJW_Data_Debug_Index2] = RDF;//FOGPitchVel_NF2;//GLOBAL_X_LF;
//        JW_Data_Debug2[78][sJW_Data_Debug_Index2] = LDF;//FOGPitchVel_LPF;//GLOBAL_Y_LF;
//        JW_Data_Debug2[79][sJW_Data_Debug_Index2] = _temp_debug_data[10];//sharedSEN->FOG.PitchVel;//GLOBAL_Z_LF;


//        JW_Data_Debug2[80][sJW_Data_Debug_Index2] = _temp_debug_data[11];//BTW_YAW;//temp_debug[0];//_legpos[0][0];
//        JW_Data_Debug2[81][sJW_Data_Debug_Index2] = _temp_debug_data[12];//BTW_ROLL;//temp_debug[1];//_legpos[0][1];
//        JW_Data_Debug2[82][sJW_Data_Debug_Index2] = _temp_debug_data[13];//BTW_PITCH;//fsm->VirtualComInfos[0][1];
//        JW_Data_Debug2[83][sJW_Data_Debug_Index2] = _temp_debug_data[14];//fsm->AddComInfos[0][2];
//        JW_Data_Debug2[84][sJW_Data_Debug_Index2] = des_pCOM_3x1[2];

//        JW_Data_Debug2[85][sJW_Data_Debug_Index2] = 1 + fsm->AddJointInfos[0][JRAR]*1./5. + fsm->AddJointInfos[0][JLAR]*1./5.;

//        JW_Data_Debug2[86][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JRAR];//_legpos[2][0];
//        JW_Data_Debug2[87][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLAR];//_legpos[2][1];
//        JW_Data_Debug2[88][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JRHR];//_legpos[2][2];
//        JW_Data_Debug2[89][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLHR];//_legpos[3][0];
//        JW_Data_Debug2[90][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JRAP];//_legpos[3][1];
//        JW_Data_Debug2[91][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLAP];//_legpos[3][2];

//        JW_Data_Debug2[92][sJW_Data_Debug_Index2] = fsm->AddRightFootInfos[0][2];//_shin_dist[0];
//        JW_Data_Debug2[93][sJW_Data_Debug_Index2] = fsm->AddLeftFootInfos[0][2];//_shin_dist[1];

//        JW_Data_Debug2[94][sJW_Data_Debug_Index2] = _temp_debug_data[1];//_foot_dist[0];
//        JW_Data_Debug2[95][sJW_Data_Debug_Index2] = _temp_debug_data[6];//_foot_dist[1];
//        JW_Data_Debug2[96][sJW_Data_Debug_Index2] = _foot_dist[2];
//        JW_Data_Debug2[97][sJW_Data_Debug_Index2] = _foot_dist[3];

////        JW_Data_Debug2[85][sJW_Data_Debug_Index2] =temp_debug[3];
////        JW_Data_Debug2[86][sJW_Data_Debug_Index2] = temp_debug[8];
////        JW_Data_Debug2[87][sJW_Data_Debug_Index2] = Qub[idRSP];
////        JW_Data_Debug2[88][sJW_Data_Debug_Index2] = Qub[idREB];
////        JW_Data_Debug2[89][sJW_Data_Debug_Index2] = Qub[idRWY];
////        JW_Data_Debug2[90][sJW_Data_Debug_Index2] = Qub[idRWP];
////        JW_Data_Debug2[91][sJW_Data_Debug_Index2] = Qub[idRWY2];

////        JW_Data_Debug2[92][sJW_Data_Debug_Index2] = Qub[idLSY];
////        JW_Data_Debug2[93][sJW_Data_Debug_Index2] = Qub[idLSR];
////        JW_Data_Debug2[94][sJW_Data_Debug_Index2] = Qub[idLSP];
////        JW_Data_Debug2[95][sJW_Data_Debug_Index2] = Qub[idLEB];
////        JW_Data_Debug2[96][sJW_Data_Debug_Index2] = Qub[idLWY];
////        JW_Data_Debug2[97][sJW_Data_Debug_Index2] = Qub[idLWP];
////        JW_Data_Debug2[98][sJW_Data_Debug_Index2] = Qub[idLWY2];

//        JW_Data_Debug2[98][sJW_Data_Debug_Index2] =fsm->AddComInfos[0][2];




////        JW_Data_Debug2[22][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].AccPitch;
////        JW_Data_Debug2[23][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].AccRoll;
////        JW_Data_Debug2[24][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].VelPitch;
////        JW_Data_Debug2[46][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].VelRoll;
////        JW_Data_Debug2[46][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].Pitch;
////        JW_Data_Debug2[47][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].Roll;

////        JW_Data_Debug2[28][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].dAccPitch;
////        JW_Data_Debug2[29][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].dAccRoll;
////        JW_Data_Debug2[30][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].AccPitch;
////        JW_Data_Debug2[31][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].AccRoll;
////        JW_Data_Debug2[32][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].VelPitch;
////        JW_Data_Debug2[33][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].VelRoll;
////        JW_Data_Debug2[34][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].Pitch;
////        JW_Data_Debug2[35][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].Roll;


//        sJW_Data_Debug_Index2++;
//        if(sJW_Data_Debug_Index2 >= ROW_data_debug) sJW_Data_Debug_Index2 = 0;
//    }
}


void Preliminary()
{
    //   long t,dt;
    //   t = myclock();
    //   sleep(5);
        _preliminary_flag = false;
        double pv_state[2][3] = {{0.0f}}, pv_state_old[2][3] = {{0.0f}};
        double vir_pv_state[2][3] = {{0.0f}},vir_pv_state_old[2][3] = {{0.0f}},modification_state[2][3]= {{0.0f}},modification_state_old[2][3]= {{0.0f}};
        static double pv_ZMP[2] = {0.f,};
        static double pv_Err[2] = {0.f,};
        static double vir_pv_Err[2]= {0.f,};
        static double pv_U[2] = {0.f,},vir_pv_U[2] = {0.f,};
        static double zmp_z_v = 0.0f,zmp_x_v = 0.0f,pv_z_ref = 0.0f,pv_z_acc_ref = 0.0f;
        //static double temp_buffer[2][300] = {{0.0f}};
        double temp_sum[2]={0.0,0.0,};

        //if(fsm->StateInfos[0][0] != STATE_FINISHED)
        {
       //      printf("Preliminary_preview test1 \n");

            if(pv_Index == 0){
                //memcpy(temp_Q_34x1,WBIK_Q,34*sizeof(double));
//                get_WBIK_Q_from_RefAngleCurrent();

////                WBIK_Q[idQ0] = des_qPEL_4x1[0];//1;
////                WBIK_Q[idQ1] = des_qPEL_4x1[1];//0;
////                WBIK_Q[idQ2] = des_qPEL_4x1[2];//0;
////                WBIK_Q[idQ3] = des_qPEL_4x1[3];//0;
//                WBIK_Q[idQ0] = 1;
//                WBIK_Q[idQ1] = 0;
//                WBIK_Q[idQ2] = 0;
//                WBIK_Q[idQ3] = 0;

//                kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
//                kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
//                kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

//                printf("Est RF = (%f,%f,%f) ,(%f,%f,%f,%f), LF = (%f,%f,%f) ,(%f,%f,%f,%f), \n COM = (%f,%f,%f), qPEL = (%f,%f,%f,%f)\n"
//                        ,FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2]
//                        ,FK_qRFoot_4x1[0],FK_qRFoot_4x1[1],FK_qRFoot_4x1[2],FK_qRFoot_4x1[3]
//                        ,FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]
//                        ,FK_qLFoot_4x1[0],FK_qLFoot_4x1[1],FK_qLFoot_4x1[2],FK_qLFoot_4x1[3]
//                        ,FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2]
//                        ,des_qPEL_4x1[0],des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3]);
//                printf("Des RF = (%f,%f,%f) ,(%f,%f,%f,%f), LF = (%f,%f,%f) ,(%f,%f,%f,%f), \n COM = (%f,%f,%f)\n"
//                        ,des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]
//                        ,des_qRF_4x1[0],des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3]
//                        ,des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]
//                        ,des_qLF_4x1[0],des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3]
//                        ,des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);

                zmp_offset[0] = FK_pCOM_3x1[0] -pv_state_old[0][0];
                zmp_offset[1] = FK_pCOM_3x1[1] -pv_state_old[1][0];

                pv_state_old[0][0] = FK_pCOM_3x1[0];//CONT_X_n;
    //            pv_state_old[0][0] = CONT_X_n;
                //pv_state_old[0][0] = pv_state_old[0][0];
                pv_state_old[0][1] = pv_state_old[0][1];
                pv_state_old[0][2] = pv_state_old[0][2];

                pv_state_old[1][0] = FK_pCOM_3x1[1];//CONT_Y_n;
    //            pv_state_old[1][0] = CONT_Y_n;
                //pv_state_old[1][0] = pv_state_old[1][0];
                pv_state_old[1][1] = pv_state_old[1][1];
                pv_state_old[1][2] = pv_state_old[1][2];

//                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n FK_pCOM_3x1[0] = %f, FK_pCOM_3x1[1] = %f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1]);
                for(pv_time_Index=0;pv_time_Index<2000;pv_time_Index++)
                {
                        if(pv_time_Index < 600)
                            Gain_offset[pv_time_Index] = 1;
                        else if(pv_time_Index < 660)
                            Gain_offset[pv_time_Index] = 1 - 0.5*(1 - cos(PI*(pv_time_Index-600)/60));
                        else
                            Gain_offset[pv_time_Index] = 0.;
                }

                modification_state_old[0][0] = 0;
                modification_state_old[0][1] = 0;
                modification_state_old[0][2] = 0;

                modification_state_old[1][0] = 0;
                modification_state_old[1][1] = 0;
                modification_state_old[1][2] = 0;


            }




            //-------------------------------------------------COM 1----------------------------------------------------//
            for(int i=0;i<=1;i++)
            {
                //=========================
                pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];
                pv_Err[i] = pv_Err[i] + pv_ZMP[i] - fsm->ZMPInfos[0][i];

                if(pv_Index == 0)
                {
                    _preliminary_state_I[i][0] = pv_state_old[i][0];
                    _preliminary_state_I[i][1] = pv_state_old[i][1];
                    _preliminary_state_I[i][2] = pv_state_old[i][2];

                    printf("preliminary_state_I: %f    pv_state_old: %f \n",_preliminary_state_I[i][0],pv_state_old[i][0]);

                }

                temp_sum[i] = 0.0f;
                for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
                {
                    temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(fsm->ZMPInfos[pv_time_Index][i]);
                }

                //pv_U[i] = -pv_Gi[1]*pv_Err[i] - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];
//                if(pv_Index == 0)
//                {
//                    pv_state_old[i][0] = _preliminary_state_I[i][0];
//                    pv_state_old[i][1] = _preliminary_state_I[i][1];
//                    pv_state_old[i][2] = _preliminary_state_I[i][2];

//                    printf("preliminary_state_I: %f    pv_state_old: %f \n",_preliminary_state_I[i][0],pv_state_old[i][0]);

//                }

                pv_U[i] =  - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

                pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
                pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
                pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

                pv_state_old[i][0] = pv_state[i][0];
                pv_state_old[i][1] = pv_state[i][1];
                pv_state_old[i][2] = pv_state[i][2];

                _preliminary_state_I[i][0] = pv_state[i][0];
                _preliminary_state_I[i][1] = pv_state[i][1];
                _preliminary_state_I[i][2] = pv_state[i][2];
            }

     //       printf("preliminary pv_state %f \n",pv_state_old[0][0]);

            _temp_com = pv_state[0][0];


            //-----------------------------------------------------COM 2-----------------------------------------------------------//
            for(int k = 0;k < 2;k++)
            {
                    //      printf("Preliminary_preview test3 \n");

                for(int i=0;i<=299;i++)
                {

                    pv_z_ref = fsm->VirtualComInfos[i][1] + userData->WalkReadyCOM[Zdir]+COM_Offset; //0.05f*sin(2.0f*PI*0.002f*((double)(pv_Index + i))) + 0.71f; //+ 0.71f;
                    pv_z_acc_ref = fsm->VirtualComAccInfos[i][1]; //-0.05f*2.0f*PI*2.0f*PI*sin(2.0f*PI*0.002f*((double)(pv_Index + i)));

                    pv_C_v[0][0] = 1.0f;
                    pv_C_v[0][1] = 0.0f;
                    pv_C_v[0][2] = - pv_z_ref/(pv_z_acc_ref + 9.81f);

                    //--------------------------OUTPUT ZMP
                    zmp_x_v = pv_C_v[0][0]*vir_pv_state_old[k][0] + pv_C_v[0][1]*vir_pv_state_old[k][1]+ pv_C_v[0][2]*vir_pv_state_old[k][2];

                    //Virtual Plane O.K
                    zmp_z_v = pv_z_ref - (userData->WalkReadyCOM[Zdir]+COM_Offset) - (userData->WalkReadyCOM[Zdir]+COM_Offset)*pv_z_acc_ref/9.81f;

                    //ZMP Error from Virtual plane-
                    //-------------------------------------------------------------------------------------------------------------------------------
                    _virtual_zmp_error[k][i] = (zmp_x_v - fsm->ZMPInfos[i+1][k])*(1.0f - zmp_z_v)/pv_z_ref;

                    //-------------------------------------------------------------------------------------------------------------------------------

                    //pr_Err_z[k][i] = (zmp_x_v - fsm->ZMPInfos[i+1][k])*(1.0f - zmp_z_v)/pv_z_ref;

                    temp_sum[k] = 0.0f;

                    for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
                    {
                        temp_sum[k] = temp_sum[k] + pv_Gd[1 + pv_time_Index]*(fsm->ZMPInfos[pv_time_Index + i][k]);
                    }

                    pv_U[k] =  - (pv_Gx[1]*vir_pv_state_old[k][0] + pv_Gx[2]*vir_pv_state_old[k][1] + pv_Gx[3]*vir_pv_state_old[k][2]) - temp_sum[k];


                    vir_pv_state[k][0] = (pv_A[0][0]*vir_pv_state_old[k][0] + pv_A[0][1]*vir_pv_state_old[k][1] + pv_A[0][2]*vir_pv_state_old[k][2]) + (pv_B[0][0])*pv_U[k];
                    vir_pv_state[k][1] = (pv_A[1][0]*vir_pv_state_old[k][0] + pv_A[1][1]*vir_pv_state_old[k][1] + pv_A[1][2]*vir_pv_state_old[k][2]) + (pv_B[1][0])*pv_U[k];
                    vir_pv_state[k][2] = (pv_A[2][0]*vir_pv_state_old[k][0] + pv_A[2][1]*vir_pv_state_old[k][1] + pv_A[2][2]*vir_pv_state_old[k][2]) + (pv_B[2][0])*pv_U[k];

                    vir_pv_state_old[k][0] = vir_pv_state[k][0];
                    vir_pv_state_old[k][1] = vir_pv_state[k][1];
                    vir_pv_state_old[k][2] = vir_pv_state[k][2];

                    //printf("Preliminary_preview test4 \n");
                }

               //-------------------------------sum of virtual zmp error

                _preliminary_state_298[k][0] = vir_pv_state_old[k][0];
                _preliminary_state_298[k][1] = vir_pv_state_old[k][1];
                _preliminary_state_298[k][2] = vir_pv_state_old[k][2];

               vir_pv_Err[k] = 0.0f;

               for(int i = 0;i<=299;i++)
               {
           //        printf("Preliminary_preview test5 \n");
                   vir_pv_Err[k] = vir_pv_Err[k] + pv_Gd[i+1]*_virtual_zmp_error[k][i];
               }
//               if(k == 0)
//               {
//                   _temp_debug_data4 = vir_pv_Err[k];
//               }

               //-------------------------------Control input2
               vir_pv_U[k] = -pv_Gx[1]*modification_state_old[k][0] - pv_Gx[2]*modification_state_old[k][1] - pv_Gx[3]*modification_state_old[k][2] + vir_pv_Err[k];

               modification_state[k][0] = (pv_A[0][0]*modification_state_old[k][0] + pv_A[0][1]*modification_state_old[k][1] + pv_A[0][2]*modification_state_old[k][2]) + (pv_B[0][0])*vir_pv_U[k];
               modification_state[k][1] = (pv_A[1][0]*modification_state_old[k][0] + pv_A[1][1]*modification_state_old[k][1] + pv_A[1][2]*modification_state_old[k][2]) + (pv_B[1][0])*vir_pv_U[k];
               modification_state[k][2] = (pv_A[2][0]*modification_state_old[k][0] + pv_A[2][1]*modification_state_old[k][1] + pv_A[2][2]*modification_state_old[k][2]) + (pv_B[2][0])*vir_pv_U[k];

                modification_state_old[k][0] = modification_state[k][0];
                modification_state_old[k][1] = modification_state[k][1];
                modification_state_old[k][2] = modification_state[k][2];

                pv_state[k][0] =   modification_state_old[k][0] + pv_state_old[k][0];
                pv_state[k][1] =   modification_state_old[k][1] + pv_state_old[k][1];
                pv_state[k][2] =   modification_state_old[k][2] + pv_state_old[k][2];

                _preliminary_state_II[k][0] = modification_state[k][0];
                _preliminary_state_II[k][1] = modification_state[k][1];
                _preliminary_state_II[k][2] = modification_state[k][2];
            }

//            if(fsm->walking_mode == TERRAIN_WALKING)
            {
               GLOBAL_Y_LIPM = pv_state[1][0]*3.004/2.99;//*15.0011/15.114;
               GLOBAL_X_LIPM = pv_state[0][0]*3.004/2.99;//*3.005/3.027;//*15.0011/15.114;
            }
//            else
//            {
//                GLOBAL_Y_LIPM = pv_state[1][0];//*15.0011/15.114;
//                GLOBAL_X_LIPM = pv_state[0][0];//*15.0011/15.114;
//            }

           GLOBAL_Y_LIPM_d = pv_state[1][1];
           GLOBAL_X_LIPM_d = pv_state[0][1];

           GLOBAL_X_RF = fsm->RightInfos[0][0];
           GLOBAL_X_LF = fsm->LeftInfos[0][0];

           GLOBAL_Y_RF = fsm->RightInfos[0][1];
           GLOBAL_Y_LF = fsm->LeftInfos[0][1];

           GLOBAL_ZMP_REF_X = fsm->ZMPInfos[0][0];
           GLOBAL_ZMP_REF_Y = fsm->ZMPInfos[0][1];


//           GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((fsm->Right-Infos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

           GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_ZMP_REF_Y_n = -GLOBAL_ZMP_REF_X*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);


//           step_length = GLOBAL_X_RF - GLOBAL_X_LF;
//           theta_step_length = atan2(0.2,step_length);
//           GLOBAL_ZMP_REF_X_nn =  GLOBAL_ZMP_REF_X_n*cos(theta_step_length) + GLOBAL_ZMP_REF_Y_n*sin(theta_step_length);
//           GLOBAL_ZMP_REF_Y_nn = -GLOBAL_ZMP_REF_X_n*sin(theta_step_length) + GLOBAL_ZMP_REF_Y_n*cos(theta_step_length);

           //Initial value setting
            des_pRF_3x1[0] = fsm->RightInfos[0][0];
            des_pRF_3x1[1] = fsm->RightInfos[0][1];
            des_pRF_3x1[2] = FK_pRFoot_3x1[2];

            des_pLF_3x1[0] = fsm->LeftInfos[0][0];
            des_pLF_3x1[1] = fsm->LeftInfos[0][1];
            des_pLF_3x1[2] = FK_pLFoot_3x1[2];

           double Global[3],Local[3];
           Global[0] = GLOBAL_X_LIPM;
           Global[1] = GLOBAL_Y_LIPM;
           Global[2] = 0;
           Global2Local(Global,Local);
           GLOBAL_X_LIPM_n =  Local[0];//GLOBAL_X_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_LIPM_n =  Local[1];//-GLOBAL_X_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

           Global[0] = GLOBAL_X_LIPM_d;
           Global[1] = GLOBAL_Y_LIPM_d;
           Global[2] = 0;
           Global2Local(Global,Local);
           GLOBAL_X_LIPM_d_n =  Local[0];//GLOBAL_X_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_LIPM_d_n =  Local[1];//-GLOBAL_X_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

           Global[0] = GLOBAL_X_RF;
           Global[1] = GLOBAL_Y_RF;
           Global[2] = 0;
           Global2Local(Global,Local);
           GLOBAL_X_RF_n = Local[0];// GLOBAL_X_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_RF_n = Local[1];//-GLOBAL_X_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

           Global[0] = GLOBAL_X_LF;
           Global[1] = GLOBAL_Y_LF;
           Global[2] = 0;
           Global2Local(Global,Local);
           GLOBAL_X_LF_n = Local[0];// GLOBAL_X_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_LF_n = Local[1];//-GLOBAL_X_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

   //        GLOBAL_Z_RF = fsm->RightInfos[0][2];
   //        GLOBAL_Z_LF = fsm->LeftInfos[0][2];
           //------------------------------------------------------ GLOBAL Z REF ---------------------------------------------------------//
    //       GLOBAL_Z_LIPM =  0.05f*sin(2.0f*PI*0.002f*((double)(pv_Index )));
           pv_Index++;
       }
}
void Global2Local(double _Global[],double _Local[])
{
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double temp1[3],temp2[3],temp3[3],temp4[3];

    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
    qtRZ((fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,qCenter);

    diff_vv(_Global,3,pCenter,temp1); // _Global - pCenter

    qCenter_bar[0] = qCenter[0];
    qCenter_bar[1] = -qCenter[1];
    qCenter_bar[2] = -qCenter[2];
    qCenter_bar[3] = -qCenter[3];

    QTtransform(qCenter_bar, temp1, _Local);
}
void Global2Local2(double _Global[],double _Local[])
{
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double temp1[3],temp2[3],temp3[3],temp4[3];

    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
    qtRZ((fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,qCenter);

    diff_vv(_Global,3,pCenter,temp1); // _Global - pCenter

    qCenter_bar[0] = qCenter[0];
    qCenter_bar[1] = -qCenter[1];
    qCenter_bar[2] = -qCenter[2];
    qCenter_bar[3] = -qCenter[3];

    QTtransform(qCenter_bar, temp1, _Local);
}
void Local2Global(double _Local[],double _Global[])
{
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double temp1[3],temp2[3],temp3[3],temp4[3];

    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
    qtRZ((fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,qCenter);

    QTtransform(qCenter, _Local, temp1);

    sum_vv(temp1,3,pCenter,_Global); // zmp - pCenter


}

void ThreeDim_Preview()
{
//    printf("ThreeDim!!!!!!!!!!!!!!!!!!!!!!!\n");
    static double pv_state[2][3] = {{0.0f}},pv_state_old[2][3] = {{0.0f}};
    static double vir_pv_state[2][3] = {{0.0f}},vir_pv_state_old[2][3] = {{0.0f}},modification_state[2][3]= {{0.0f}},modification_state_old[2][3]= {{0.0f}};
    static double pv_ZMP[2] = {0.f,};
    static double temp_virtual_zmp_error = 0.0f;
    static double vir_pv_Err[2]= {0.f,};
    static double pv_U[2] = {0.f,},vir_pv_U[2] = {0.f,};
    static double zmp_z_v = 0.0f,zmp_x_v = 0.0f,pv_z_ref = 0.0f,pv_z_acc_ref = 0.0f;
    double temp_sum[2]={0.,0,};

    //if(fsm->StateInfos[0][0] != STATE_FINISHED)
    {

        if(pv_Index == 1 && _preliminary_flag == false )
        {
            pv_state_old[0][0] = _preliminary_state_I[0][0];
            pv_state_old[0][1] = _preliminary_state_I[0][1];
            pv_state_old[0][2] = _preliminary_state_I[0][2];

            pv_state_old[1][0] = _preliminary_state_I[1][0];
            pv_state_old[1][1] = _preliminary_state_I[1][1];
            pv_state_old[1][2] = _preliminary_state_I[1][2];
        }

//        static double temp_Q_34x1[34] = {0,};

        for(int i=0;i<=1;i++)
        {
            pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];

            temp_sum[i] = 0.0f;

            for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
            {
                temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(fsm->ZMPInfos[pv_time_Index][i]);
            }

            pv_U[i] =  - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

            pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
            pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
            pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];

            if(i==1)
            {
                _temp_debug_data[0] = pv_state_old[i][0];
                _temp_debug_data[4] = pv_ZMP[i];
            }else
            {
                _temp_debug_data[5] = pv_state_old[i][0];
                _temp_debug_data[9] = pv_ZMP[i];
            }
        }

        _temp_com = pv_state[0][0];

        if(pv_Index == 1 && _preliminary_flag == false)
        {
            vir_pv_state_old[0][0] = _preliminary_state_298[0][0];
            vir_pv_state_old[0][1] = _preliminary_state_298[0][1];
            vir_pv_state_old[0][2] = _preliminary_state_298[0][2];

            vir_pv_state_old[1][0] = _preliminary_state_298[1][0];
            vir_pv_state_old[1][1] = _preliminary_state_298[1][1];
            vir_pv_state_old[1][2] = _preliminary_state_298[1][2];

            modification_state_old[0][0] = 0.0f;
            modification_state_old[0][1] = 0.0f;
            modification_state_old[0][2] = 0.0f;

            modification_state_old[1][0] = 0.0f;
            modification_state_old[1][1] = 0.0f;
            modification_state_old[1][2] = 0.0f;

            _preliminary_flag = true;
        }


        if(pv_Index == 0 && _preliminary_flag == true)
        {
            modification_state_old[0][0] = 0.0f;
            modification_state_old[0][1] = 0.0f;
            modification_state_old[0][2] = 0.0f;

            modification_state_old[1][0] = 0.0f;
            modification_state_old[1][1] = 0.0f;
            modification_state_old[1][2] = 0.0f;
        }




        for(int k = 0;k < 2;k++)
        {
            int i=299;
            pv_z_ref = fsm->VirtualComInfos[i][1] + (userData->WalkReadyCOM[Zdir]+COM_Offset);
            pv_z_acc_ref = fsm->VirtualComAccInfos[i][1];

            pv_C_v[0][0] = 1.0f; pv_C_v[0][1] = 0.0f; pv_C_v[0][2] = - pv_z_ref/(pv_z_acc_ref + 9.81f);


            zmp_x_v = pv_C_v[0][0]*vir_pv_state_old[k][0] + pv_C_v[0][1]*vir_pv_state_old[k][1]+ pv_C_v[0][2]*vir_pv_state_old[k][2];

            zmp_z_v = pv_z_ref - (userData->WalkReadyCOM[Zdir]+COM_Offset) - (userData->WalkReadyCOM[Zdir]+COM_Offset)*pv_z_acc_ref/9.81f;

             temp_virtual_zmp_error = (zmp_x_v - fsm->ZMPInfos[i][k])*(1.0f - zmp_z_v)/pv_z_ref;

            for(int j=0;j<=198;j++)
            {
                _virtual_zmp_error[k][j] = _virtual_zmp_error[k][j+1];
            }

            _virtual_zmp_error[k][i] = temp_virtual_zmp_error;

            temp_sum[k] = 0.0f;

            for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
            {
                temp_sum[k] = temp_sum[k] + pv_Gd[1 + pv_time_Index]*(fsm->ZMPInfos[pv_time_Index + i][k]);
            }

             pv_U[k] =  - (pv_Gx[1]*vir_pv_state_old[k][0] + pv_Gx[2]*vir_pv_state_old[k][1] + pv_Gx[3]*vir_pv_state_old[k][2]) - temp_sum[k];

             vir_pv_state[k][0] = (pv_A[0][0]*vir_pv_state_old[k][0] + pv_A[0][1]*vir_pv_state_old[k][1] + pv_A[0][2]*vir_pv_state_old[k][2]) + (pv_B[0][0])*pv_U[k];
             vir_pv_state[k][1] = (pv_A[1][0]*vir_pv_state_old[k][0] + pv_A[1][1]*vir_pv_state_old[k][1] + pv_A[1][2]*vir_pv_state_old[k][2]) + (pv_B[1][0])*pv_U[k];
             vir_pv_state[k][2] = (pv_A[2][0]*vir_pv_state_old[k][0] + pv_A[2][1]*vir_pv_state_old[k][1] + pv_A[2][2]*vir_pv_state_old[k][2]) + (pv_B[2][0])*pv_U[k];

             vir_pv_state_old[k][0] = vir_pv_state[k][0];
             vir_pv_state_old[k][1] = vir_pv_state[k][1];
             vir_pv_state_old[k][2] = vir_pv_state[k][2];

            vir_pv_Err[k] = 0.0f;

            for(int i = 0;i<=299;i++)
            {
                 vir_pv_Err[k] = vir_pv_Err[k] + pv_Gd[i+1]*_virtual_zmp_error[k][i];
            }

            vir_pv_U[k] = -pv_Gx[1]*modification_state_old[k][0] - pv_Gx[2]*modification_state_old[k][1] - pv_Gx[3]*modification_state_old[k][2] + vir_pv_Err[k];

            modification_state[k][0] = (pv_A[0][0]*modification_state_old[k][0] + pv_A[0][1]*modification_state_old[k][1] + pv_A[0][2]*modification_state_old[k][2]) + (pv_B[0][0])*vir_pv_U[k];
            modification_state[k][1] = (pv_A[1][0]*modification_state_old[k][0] + pv_A[1][1]*modification_state_old[k][1] + pv_A[1][2]*modification_state_old[k][2]) + (pv_B[1][0])*vir_pv_U[k];
            modification_state[k][2] = (pv_A[2][0]*modification_state_old[k][0] + pv_A[2][1]*modification_state_old[k][1] + pv_A[2][2]*modification_state_old[k][2]) + (pv_B[2][0])*vir_pv_U[k];

            modification_state_old[k][0] = modification_state[k][0];
            modification_state_old[k][1] = modification_state[k][1];
            modification_state_old[k][2] = modification_state[k][2];

            if(k == 0)
            {
                pv_state[k][0] =   pv_state_old[k][0];
                pv_state[k][1] =   pv_state_old[k][1];
                pv_state[k][2] =   pv_state_old[k][2];
            }else
            {

                pv_state[k][0] =   modification_state_old[k][0] + pv_state_old[k][0];
                pv_state[k][1] =   modification_state_old[k][1] + pv_state_old[k][1];
                pv_state[k][2] =   modification_state_old[k][2] + pv_state_old[k][2];

//                pv_state[k][0] =   pv_state_old[k][0];
//                pv_state[k][1] =   pv_state_old[k][1];
//                pv_state[k][2] =   pv_state_old[k][2];
            }
//            zmp_x_v = pv_C_v[0][0]*pv_state[k][0] + pv_C_v[0][1]*pv_state[k][1]+ pv_C_v[0][2]*pv_state[k][2];


           // temp_debug[k] =modification_state_old[k][0];

            if(k == 1)
            {
                _temp_debug_data[1] = modification_state_old[k][0];
                _temp_debug_data[2] = pv_state[k][0];
                _temp_debug_data[3] = zmp_x_v;//pv_C_v[0][0]*pv_state[k][0] + pv_C_v[0][1]*pv_state[k][1]+ pv_C_v[0][2]*pv_state[k][2];
                _temp_debug_data[4] = zmp_z_v;
                _temp_debug_data[5] = fsm->VirtualComInfos[299][1];
                _temp_debug_data[10] = temp_virtual_zmp_error;
                _temp_debug_data[14] = fsm->VirtualComAccInfos[299][1];

            }else
            {
                _temp_debug_data[6] = modification_state_old[k][0];
                _temp_debug_data[7] = pv_state[k][0];
                _temp_debug_data[8] = zmp_x_v;//pv_C_v[0][0]*pv_state[k][0] + pv_C_v[0][1]*pv_state[k][1]+ pv_C_v[0][2]*pv_state[k][2];
                _temp_debug_data[9] = zmp_z_v;
                _temp_debug_data[11] = temp_virtual_zmp_error;
                _temp_debug_data[12] = fsm->VirtualComInfos[299][1];
                _temp_debug_data[13] = fsm->VirtualComAccInfos[299][1];
            }

        }

//        if(fsm->walking_mode == TERRAIN_WALKING)
        {
           GLOBAL_Y_LIPM = pv_state[1][0]*3.004/2.99;//*3.009/2.923;//*15.0011/15.114;
           GLOBAL_X_LIPM = pv_state[0][0]*3.004/2.99;//*3.005/3.027;//*15.0011/15.114;
        }
//        else
//        {
//            GLOBAL_Y_LIPM = pv_state[1][0];//*15.0011/15.114;
//            GLOBAL_X_LIPM = pv_state[0][0];//*15.0011/15.114;
//        }

        GLOBAL_Y_LIPM_d = pv_state[1][1];
        GLOBAL_X_LIPM_d = pv_state[0][1];

        GLOBAL_X_RF = fsm->RightInfos[0][0];
        GLOBAL_X_LF = fsm->LeftInfos[0][0];

        GLOBAL_Y_RF = fsm->RightInfos[0][1];
        GLOBAL_Y_LF = fsm->LeftInfos[0][1];

        GLOBAL_ZMP_REF_X = fsm->ZMPInfos[0][0];
        GLOBAL_ZMP_REF_Y = fsm->ZMPInfos[0][1];

        GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_ZMP_REF_Y_n = -GLOBAL_ZMP_REF_X*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_X_LIPM_n =  GLOBAL_X_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//        GLOBAL_Y_LIPM_n = -GLOBAL_X_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_X_LIPM_d_n =  GLOBAL_X_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//        GLOBAL_Y_LIPM_d_n = -GLOBAL_X_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_X_RF_n =  GLOBAL_X_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//        GLOBAL_Y_RF_n = -GLOBAL_X_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_X_LF_n =  GLOBAL_X_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
//        GLOBAL_Y_LF_n = -GLOBAL_X_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        double Global[3],Local[3];
        Global[0] = GLOBAL_X_LIPM;
        Global[1] = GLOBAL_Y_LIPM;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LIPM_n =  Local[0];//GLOBAL_X_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LIPM_n =  Local[1];//-GLOBAL_X_LIPM*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        Global[0] = GLOBAL_X_LIPM_d;
        Global[1] = GLOBAL_Y_LIPM_d;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LIPM_d_n =  Local[0];//GLOBAL_X_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LIPM_d_n =  Local[1];//-GLOBAL_X_LIPM_d*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        Global[0] = GLOBAL_X_RF;
        Global[1] = GLOBAL_Y_RF;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_RF_n = Local[0];// GLOBAL_X_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_RF_n = Local[1];//-GLOBAL_X_RF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

        Global[0] = GLOBAL_X_LF;
        Global[1] = GLOBAL_Y_LF;
        Global[2] = 0;
        Global2Local(Global,Local);
        GLOBAL_X_LF_n = Local[0];// GLOBAL_X_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LF_n = Local[1];//-GLOBAL_X_LF*sin((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_Z_RF = fsm->RightInfos[0][2];
//        GLOBAL_Z_LF = fsm->LeftInfos[0][2];

        pv_Index++;
    }

}
//-----------------------------------------------------------------------------------------------//
double AngularMomentumComp(double velx,double torso_yaw,int sign, int reset)
{
    double  M_torso = 35.0f,M_leg = 5.0f;
    double vel = velx*cos(torso_yaw) + 0*sin(torso_yaw);
    static double yaw_theta =0;


    if(sign > 0)
    {
        yaw_theta = -(vel*M_leg*0.26f/((M_torso)*0.2f*0.2f))*0.005f*R2D + yaw_theta;
        //printf("r yaw_theta: %f \n",yaw_theta);
    }else
    {
        yaw_theta = (vel*M_leg*0.26f/((M_torso)*0.2f*0.2f))*0.005f*R2D + yaw_theta ;
        //printf("l yaw_theta: %f \n",yaw_theta);
    }
    if(reset == 0)yaw_theta = 0;
    return yaw_theta;

}

//-----------------------------------------------------------------------------------------------//
// --------------------------------------------------------------------------------------------- //

void WBIK_PARA_CHANGE(){
//    kine_drc_hubo4.L_PEL2PEL = 0.2;
   /* kine_drc_hubo4.m_Torso = 28.6+1.0;
    kine_drc_hubo4.C_Torso[1] =0;
    kine_drc_hubo4.C_Torso[0] = -0.035 - 0.01;
    kine_drc_hubo4.L_HAND=0.160;
    kine_drc_hubo4.m_LeftLowerArm = 0;
    kine_drc_hubo4.m_LeftUpperArm = 0;
    kine_drc_hubo4.m_RightUpperArm = 0;
    kine_drc_hubo4.m_RightLowerArm = 0;
    kine_drc_hubo4.m_RightHand = 0;
    kine_drc_hubo4.m_LeftHand = 0;*/
//    kine_drc_hubo43.C_Torso[0] = -0.035 -0.01;
//    kine_drc_hubo43.C_Torso[1] =  0;
//    kine_drc_hubo43.m_Torso = 28.6+1;
    kine_drc_hubo4.C_Torso[0] = 0.000941-0.065;
//    kine_drc_hubo43.C_Torso[1] =  0;
    kine_drc_hubo4.m_Torso = 24.98723;
    kine_drc_hubo4.m_RightWrist = 4.5;
    kine_drc_hubo4.m_LeftWrist = 4.5;
    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo4.C_Torso[1] =0;
    kine_drc_hubo4.L_FOOT = 0.113;


    kine_drc_hubo4.iter_limit = 100;
    kine_drc_hubo4.converge_criterium = 1e-6;

//    Print_WBIK_Infos();


//    printf("!!!!!!!!!!!HY limit = %f,%f\n",kine_drc_hubo4.upper_limit[idLHY]*R2D,kine_drc_hubo4.lower_limit[idLHY]*R2D);
//    printf("!!!!!!!!!!!HR limit = %f,%f\n",kine_drc_hubo4.upper_limit[idLHR]*R2D,kine_drc_hubo4.lower_limit[idLHR]*R2D);
//    printf("!!!!!!!!!!!HP limit = %f,%f\n",kine_drc_hubo4.upper_limit[idLHP]*R2D,kine_drc_hubo4.lower_limit[idLHP]*R2D);
//    printf("!!!!!!!!!!!KN limit = %f,%f\n",kine_drc_hubo4.upper_limit[idLKN]*R2D,kine_drc_hubo4.lower_limit[idLKN]*R2D);
//    printf("!!!!!!!!!!!AP limit = %f,%f\n",kine_drc_hubo4.upper_limit[idLAP]*R2D,kine_drc_hubo4.lower_limit[idLAP]*R2D);
//    printf("!!!!!!!!!!!AR limit = %f,%f\n",kine_drc_hubo4.upper_limit[idLAR]*R2D,kine_drc_hubo4.lower_limit[idLAR]*R2D);

//    printf("!!!!!!!!!!!HY limit = %f,%f\n",kine_drc_hubo4.upper_limit[idRHY]*R2D,kine_drc_hubo4.lower_limit[idRHY]*R2D);
//    printf("!!!!!!!!!!!HR limit = %f,%f\n",kine_drc_hubo4.upper_limit[idRHR]*R2D,kine_drc_hubo4.lower_limit[idRHR]*R2D);
//    printf("!!!!!!!!!!!HP limit = %f,%f\n",kine_drc_hubo4.upper_limit[idRHP]*R2D,kine_drc_hubo4.lower_limit[idRHP]*R2D);
//    printf("!!!!!!!!!!!KN limit = %f,%f\n",kine_drc_hubo4.upper_limit[idRKN]*R2D,kine_drc_hubo4.lower_limit[idRKN]*R2D);
//    printf("!!!!!!!!!!!AP limit = %f,%f\n",kine_drc_hubo4.upper_limit[idRAP]*R2D,kine_drc_hubo4.lower_limit[idRAP]*R2D);
//    printf("!!!!!!!!!!!AR limit = %f,%f\n",kine_drc_hubo4.upper_limit[idRAR]*R2D,kine_drc_hubo4.lower_limit[idRAR]*R2D);
//    kine_drc_hubo4.m_LeftWrist = 0;
//    kine_drc_hubo4.m_RightWrist = 0;
//    kine_drc_hubo4.m_Torso = 41;
//    kine_drc_hubo4.C_Torso[0] = 0.008;

//    kine_drc_hubo4.m_Pelvis = 7.699878+0.3;
//    kine_drc_hubo4.C_Pelvis[0] = -0.013095;
//    kine_drc_hubo4.C_Pelvis[1] = 0.000028;
//    kine_drc_hubo4.C_Pelvis[2] = 0.1425;

//    kine_drc_hubo4.m_LeftUpperLeg = 8.609714;
//    kine_drc_hubo4.C_LeftUpperLeg[0] = 0.00488;
//    kine_drc_hubo4.C_LeftUpperLeg[1] = 0.01602;
//    kine_drc_hubo4.C_LeftUpperLeg[2] = 0.21943-0.4;
//    kine_drc_hubo4.m_RightUpperLeg = 8.60971;
//    kine_drc_hubo4.C_RightUpperLeg[0] = 0.00488;
//    kine_drc_hubo4.C_RightUpperLeg[1] = -0.01602;
//    kine_drc_hubo4.C_RightUpperLeg[2] = 0.21943-0.4;

//    kine_drc_hubo4.m_LeftLowerLeg = 3.7246 + 1.841;
//    kine_drc_hubo4.C_LeftLowerLeg[0] = 0.01833;
//    kine_drc_hubo4.C_LeftLowerLeg[1] = 0.01362;
//    kine_drc_hubo4.C_LeftLowerLeg[2] = 0.1771-0.38 -0.01;
//    kine_drc_hubo4.m_RightLowerLeg = 3.7246 + 1.841;
//    kine_drc_hubo4.C_RightLowerLeg[0] = 0.01833;
//    kine_drc_hubo4.C_RightLowerLeg[1] = -0.01362;
//    kine_drc_hubo4.C_RightLowerLeg[2] = 0.1771-0.38 -0.01;

//    kine_drc_hubo4.m_LeftFoot = 1.33;
//    kine_drc_hubo4.m_RightFoot = 1.33;

//        printf("m_TORSO = %f,C_TORSO = %f, %f, %f\n",kine_drc_hubo4.m_Torso,kine_drc_hubo4.C_Torso[0],kine_drc_hubo4.C_Torso[1],kine_drc_hubo4.C_Torso[2]);

//        printf("m_Pelvis = %f,C_TORSO = %f, %f, %f\n",kine_drc_hubo4.m_Pelvis,kine_drc_hubo4.C_Pelvis[0],kine_drc_hubo4.C_Pelvis[1],kine_drc_hubo4.C_Pelvis[2]);

//        printf("m_LEFT_UPPER_LEG = %f,C_LEFT_UPPER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_LeftUpperLeg,kine_drc_hubo4.C_LEFT_UPPER_LEG[0],kine_drc_hubo4.C_LEFT_UPPER_LEG[1],kine_drc_hubo4.C_LEFT_UPPER_LEG[2]);
//        printf("m_RIGHT_UPPER_LEG = %f,C_RIGHT_UPPER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_RIGHT_UPPER_LEG,kine_drc_hubo4.C_RIGHT_UPPER_LEG[0],kine_drc_hubo4.C_RIGHT_UPPER_LEG[1],kine_drc_hubo4.C_RIGHT_UPPER_LEG[2]);

//        printf("m_LEFT_LOWER_LEG = %f,C_LEFT_LOWER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_LEFT_LOWER_LEG,kine_drc_hubo4.C_LEFT_LOWER_LEG[0],kine_drc_hubo4.C_LEFT_LOWER_LEG[1],kine_drc_hubo4.C_LEFT_LOWER_LEG[2]);
//        printf("m_RIGHT_LOWER_LEG = %f,C_RIGHT_LOWER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_RIGHT_LOWER_LEG,kine_drc_hubo4.C_RIGHT_LOWER_LEG[0],kine_drc_hubo4.C_RIGHT_LOWER_LEG[1],kine_drc_hubo4.C_RIGHT_LOWER_LEG[2]);

//        printf("m_LEFT_FOOT = %f,C_LEFT_FOOT = %f, %f, %f\n",kine_drc_hubo4.m_LEFT_FOOT,kine_drc_hubo4.C_LEFT_FOOT[0],kine_drc_hubo4.C_LEFT_FOOT[1],kine_drc_hubo4.C_LEFT_FOOT[2]);
//        printf("m_RIGHT_FOOT = %f,C_RIGHT_FOOT = %f, %f, %f\n",kine_drc_hubo4.m_RIGHT_FOOT,kine_drc_hubo4.C_RIGHT_FOOT[0],kine_drc_hubo4.C_RIGHT_FOOT[1],kine_drc_hubo4.C_RIGHT_FOOT[2]);

//        printf("m_LEFT_LOWER_ARM = %f,C_LEFT_LOWER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_LEFT_LOWER_ARM,kine_drc_hubo4.C_LEFT_LOWER_ARM[0],kine_drc_hubo4.C_LEFT_LOWER_ARM[1],kine_drc_hubo4.C_LEFT_LOWER_ARM[2]);
//        printf("m_RIGHT_LOWER_ARM = %f,C_RIGHT_LOWER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_RIGHT_LOWER_ARM,kine_drc_hubo4.C_RIGHT_LOWER_ARM[0],kine_drc_hubo4.C_RIGHT_LOWER_ARM[1],kine_drc_hubo4.C_RIGHT_LOWER_ARM[2]);

//        printf("m_LEFT_UPPER_ARM = %f,C_LEFT_UPPER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_LEFT_UPPER_ARM,kine_drc_hubo4.C_LEFT_UPPER_ARM[0],kine_drc_hubo4.C_LEFT_UPPER_ARM[1],kine_drc_hubo4.C_LEFT_UPPER_ARM[2]);
//        printf("m_RIGHT_UPPER_ARM = %f,C_RIGHT_UPPER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_RIGHT_UPPER_ARM,kine_drc_hubo4.C_RIGHT_UPPER_ARM[0],kine_drc_hubo4.C_RIGHT_UPPER_ARM[1],kine_drc_hubo4.C_RIGHT_UPPER_ARM[2]);
//*/
//        double total_mass;

//        total_mass = kine_drc_hubo4.m_PELVIS+kine_drc_hubo4.m_TORSO + kine_drc_hubo4.m_LEFT_UPPER_LEG+kine_drc_hubo4.m_RIGHT_UPPER_LEG+kine_drc_hubo4.m_LEFT_LOWER_LEG+kine_drc_hubo4.m_RIGHT_LOWER_LEG+kine_drc_hubo4.m_LEFT_FOOT+kine_drc_hubo4.m_RIGHT_FOOT+kine_drc_hubo4.m_LEFT_LOWER_ARM+kine_drc_hubo4.m_RIGHT_LOWER_ARM+kine_drc_hubo4.m_LEFT_UPPER_ARM+kine_drc_hubo4.m_RIGHT_UPPER_ARM;
//        //printf("total mass : %f\n",total_mass);


//    //    printf("L_PEL2PEL = %f, L_UPPER_LEG = %f, L_LOWER_LEG = %f, L_FOOT = %f, L_PC2WST = %f\n",kine_drc_hubo4.L_PEL2PEL,kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_LOWER_LEG,kine_drc_hubo4.L_FOOT,kine_drc_hubo4.L_PC2WST);


}



void JW_save_Motion_check()
{
//    if(sJW_Data_Debug_Index < ROW_data_debug)
//    {

//        if(checkfsm->LeftInfos.size() > 1){

//            JW_Data_Debug[0][sJW_Data_Debug_Index] = PELVIS_pos[0]; //fsm->LeftInfos[0][0];
//            JW_Data_Debug[1][sJW_Data_Debug_Index] = PELVIS_pos[1];
//            JW_Data_Debug[2][sJW_Data_Debug_Index] = PELVIS_pos[2];//fsm->ZMPInfos[0][1];//RKN_pos[2];

//            JW_Data_Debug[3][sJW_Data_Debug_Index] = RHY_pos[0];//GLOBAL_Y_LIPM;//RAP_pos[0];
//            JW_Data_Debug[4][sJW_Data_Debug_Index] = RHY_pos[1];//CONT_X;//[1];
//           JW_Data_Debug[5][sJW_Data_Debug_Index] = RHY_pos[2];//RAP_pos[2];

//            JW_Data_Debug[6][sJW_Data_Debug_Index] = RHR_pos[0];//RAR_pos[0];//RF_FLAG;//fsm->AddJointInfos[0][JLAR];//fsm->ZMPInfos[0][0];
//            JW_Data_Debug[7][sJW_Data_Debug_Index] = RHR_pos[1];//RAR_pos[1];
//            JW_Data_Debug[8][sJW_Data_Debug_Index] = RHR_pos[2];//RAR_pos[2];

//            JW_Data_Debug[9][sJW_Data_Debug_Index] = RHP_pos[0];//Qub[9];//GLOBAL_Y_RF;
//            JW_Data_Debug[10][sJW_Data_Debug_Index] = RHP_pos[1];//Qub[10];
//            JW_Data_Debug[11][sJW_Data_Debug_Index] = RHP_pos[2];//Qub[11];

//            JW_Data_Debug[12][sJW_Data_Debug_Index] = RKN_pos[0];
//            JW_Data_Debug[13][sJW_Data_Debug_Index] = RKN_pos[1];

//            JW_Data_Debug[14][sJW_Data_Debug_Index] = RKN_pos[2];
//            JW_Data_Debug[15][sJW_Data_Debug_Index] = RAP_pos[0];
//            JW_Data_Debug[16][sJW_Data_Debug_Index] = RAP_pos[1];

//            JW_Data_Debug[17][sJW_Data_Debug_Index] = RAP_pos[2];
//            JW_Data_Debug[18][sJW_Data_Debug_Index] = RAR_pos[0];

//            JW_Data_Debug[19][sJW_Data_Debug_Index] = RAR_pos[1];//Add_COMTask_FootPrint[0][Ydir];//temp_IK_return>>4;//fsm->ITimeInfos[0];
//            JW_Data_Debug[20][sJW_Data_Debug_Index] = RAR_pos[2];//Add_FootTask_FootPrint[RIGHT][Ydir];//I_ZMP_CON_X;//fsm->STimeInfos[0];
//            JW_Data_Debug[21][sJW_Data_Debug_Index] = FOOT_pos[0];//Add_FootTask_FootPrint[LEFT][Ydir];//I_ZMP_CON_Y;//fsm->StateInfos[0][0];


//            JW_Data_Debug[22][sJW_Data_Debug_Index] = FOOT_pos[1];//Add_COMTask_FootPrint[0][Ydir];//temp_IK_return>>4;//fsm->ITimeInfos[0];
//            JW_Data_Debug[23][sJW_Data_Debug_Index] = FOOT_pos[2];//Add_FootTask_FootPrint[RIGHT][Ydir];//I_ZMP_CON_X;//fsm->STimeInfos[0];
//            JW_Data_Debug[24][sJW_Data_Debug_Index] = WBIK_Q2[0];//I_ZMP_CON_Y;//fsm->StateInfos[0][0];

//            //JW_Data_Debug[22][sJW_Data_Debug_Index] = thread_time;

//            JW_Data_Debug[25][sJW_Data_Debug_Index] = WBIK_Q2[1];//currentPosition(LAP);
//            JW_Data_Debug[26][sJW_Data_Debug_Index] = WBIK_Q2[2];
//            JW_Data_Debug[27][sJW_Data_Debug_Index] = FOOT_pos2[0];
//            JW_Data_Debug[28][sJW_Data_Debug_Index] = FOOT_pos2[1];
//            JW_Data_Debug[29][sJW_Data_Debug_Index] = FOOT_pos2[2];

//            JW_Data_Debug[30][sJW_Data_Debug_Index] = sharedData->IMUPitch[CIMU];
//            JW_Data_Debug[31][sJW_Data_Debug_Index] = sharedData->IMURoll[CIMU];

//            JW_Data_Debug[32][sJW_Data_Debug_Index] = sharedData->IMUPitchVel[CIMU];
//            JW_Data_Debug[33][sJW_Data_Debug_Index] = sharedData->IMURollVel[CIMU];

//            JW_Data_Debug[34][sJW_Data_Debug_Index] = sharedData->IMUAccX[CIMU]*9.81/1000.;
//            JW_Data_Debug[35][sJW_Data_Debug_Index] = sharedData->IMUAccY[CIMU]*9.81/1000.;

//            JW_Data_Debug[36][sJW_Data_Debug_Index] = des_pCOM_3x1[0] + userData->WalkReadyCOM[Zdir]*sharedData->IMUAccX[CIMU]/1000.;
//            JW_Data_Debug[37][sJW_Data_Debug_Index] = Y_ZMP_IMU;

//            JW_Data_Debug[38][sJW_Data_Debug_Index] = sharedData->IMUAccY[CIMU]*9.81/1000.*DEL_T;
//            JW_Data_Debug[39][sJW_Data_Debug_Index] = EarlyLandingFlag[RIGHT];
//            JW_Data_Debug[40][sJW_Data_Debug_Index] = EarlyLandingFlag[LEFT];

//            JW_Data_Debug[41][sJW_Data_Debug_Index] = 0;//Recover_EarlyLandingFlag[RIGHT];
//            JW_Data_Debug[42][sJW_Data_Debug_Index] = 0;//Recover_EarlyLandingFlag[LEFT];

//            JW_Data_Debug[43][sJW_Data_Debug_Index] = Estimated_GLOBAL_Z_RF;
//            JW_Data_Debug[44][sJW_Data_Debug_Index] = Estimated_GLOBAL_Z_LF;

//            JW_Data_Debug[45][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_Y_n;//Ground_RF;
//            JW_Data_Debug[46][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_X_n;

//            JW_Data_Debug[47][sJW_Data_Debug_Index] = 0;
//            JW_Data_Debug[48][sJW_Data_Debug_Index] = 0;
//            JW_Data_Debug[49][sJW_Data_Debug_Index] = GLOBAL_Y_LIPM_n;//COMPEN_RHR;
//            JW_Data_Debug[50][sJW_Data_Debug_Index] = AngleRoll;//COMPEN_LHR;

//            JW_Data_Debug[51][sJW_Data_Debug_Index] = sharedData->IMUAccX[CIMU];
//            JW_Data_Debug[52][sJW_Data_Debug_Index] = sharedSEN->FOG.Roll;

//            JW_Data_Debug[53][sJW_Data_Debug_Index] = SSP_FLAG;

////            JW_Data_Debug[54][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_Y_n;
////            JW_Data_Debug[55][sJW_Data_Debug_Index] = Y_ZMP_n;

////            JW_Data_Debug[56][sJW_Data_Debug_Index] = LF_FZ_LPF;
////            JW_Data_Debug[57][sJW_Data_Debug_Index] = RF_FZ_LPF;


//            JW_Data_Debug[54][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_X_n;
//            JW_Data_Debug[55][sJW_Data_Debug_Index] = X_ZMP_n;
//            JW_Data_Debug[56][sJW_Data_Debug_Index] = CONT_X;//LF_FZ_LPF;

//            JW_Data_Debug[57][sJW_Data_Debug_Index] = GLOBAL_ZMP_REF_Y_n;
//            JW_Data_Debug[58][sJW_Data_Debug_Index] = Y_ZMP_n;
//            JW_Data_Debug[59][sJW_Data_Debug_Index] = CONT_Y;//LF_FZ_LPF;




////            JW_Data_Debug[58][sJW_Data_Debug_Index] = yaw_angle;//currentPosition(RHY);
////            JW_Data_Debug[59][sJW_Data_Debug_Index] = currentPosition(RHR);
//            JW_Data_Debug[60][sJW_Data_Debug_Index] = currentPosition(RHP);
//            JW_Data_Debug[61][sJW_Data_Debug_Index] = currentPosition(RKN);
//            JW_Data_Debug[62][sJW_Data_Debug_Index] = currentPosition(RAP);
//            JW_Data_Debug[63][sJW_Data_Debug_Index] = currentPosition(RAR);

//            JW_Data_Debug[64][sJW_Data_Debug_Index] = currentPosition(LHY);
//            JW_Data_Debug[65][sJW_Data_Debug_Index] = currentPosition(LHR);
//            JW_Data_Debug[66][sJW_Data_Debug_Index] = currentPosition(LHP);
//            JW_Data_Debug[67][sJW_Data_Debug_Index] = currentPosition(LKN);
//            JW_Data_Debug[68][sJW_Data_Debug_Index] = currentPosition(LAP);
//            JW_Data_Debug[69][sJW_Data_Debug_Index] = currentPosition(LAR);

//            JW_Data_Debug[70][sJW_Data_Debug_Index] = currentPosition(WST);//_temp_debug_data[5];
//            JW_Data_Debug[71][sJW_Data_Debug_Index] = _temp_debug_data[6];
//            JW_Data_Debug[72][sJW_Data_Debug_Index] = _temp_debug_data[7];


//            JW_Data_Debug[73][sJW_Data_Debug_Index] = checkfsm->AddComInfos[0][2];
//            JW_Data_Debug[74][sJW_Data_Debug_Index] = checkfsm->VirtualComInfos[0][1];

//            JW_Data_Debug[75][sJW_Data_Debug_Index] = 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X; //_temp_debug_data[5];
//            JW_Data_Debug[76][sJW_Data_Debug_Index] = 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y;//_temp_debug_data[6];

//            JW_Data_Debug[77][sJW_Data_Debug_Index] = I_ZMP_CON_X; //_temp_debug_data[5];
//            JW_Data_Debug[78][sJW_Data_Debug_Index] = I_ZMP_CON_Y;//_temp_debug_data[6];


//            JW_Data_Debug[79][sJW_Data_Debug_Index] = _temp_debug_data[1];
//            JW_Data_Debug[80][sJW_Data_Debug_Index] = _temp_debug_data[3];


//            JW_Data_Debug[81][sJW_Data_Debug_Index] = _temp_debug_data[6];
//            JW_Data_Debug[82][sJW_Data_Debug_Index] = _temp_debug_data[8];


//            JW_Data_Debug[83][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHY];
//            JW_Data_Debug[84][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHR];
//            JW_Data_Debug[85][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHP];
//            JW_Data_Debug[86][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RKN];
//            JW_Data_Debug[87][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RAP];
//            JW_Data_Debug[88][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RAR];

//            JW_Data_Debug[89][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHY];
//            JW_Data_Debug[90][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHR];
//            JW_Data_Debug[91][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHP];
//            JW_Data_Debug[92][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LKN];
//            JW_Data_Debug[93][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LAP];
//            JW_Data_Debug[94][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LAR];

//            //JW_Data_Debug[29][sJW_Data_Debug_Index] =
//            //JW_Data_Debug[30][sJW_Data_Debug_Index] =


////            JW_Data_Debug[18][sJW_Data_Debug_Index] = fsm_state_timer[0];
////            JW_Data_Debug[19][sJW_Data_Debug_Index] = fsm_state_timer[1];
////            JW_Data_Debug[20][sJW_Data_Debug_Index] = fsm_state_timer[2];
////            JW_Data_Debug[21][sJW_Data_Debug_Index] = fsm_state_timer[3];
////            JW_Data_Debug[22][sJW_Data_Debug_Index] = fsm_state_timer[4];
////            JW_Data_Debug[23][sJW_Data_Debug_Index] = fsm_state_timer[5];
////            JW_Data_Debug[24][sJW_Data_Debug_Index] = fsm_state_timer[6];
////            JW_Data_Debug[25][sJW_Data_Debug_Index] = fsm_state_timer[7];
////            JW_Data_Debug[26][sJW_Data_Debug_Index] = fsm_state_timer[8];


////            JW_Data_Debug[15][sJW_Data_Debug_Index] = Y_ZMP;
////            JW_Data_Debug[16][sJW_Data_Debug_Index] = Y_ZMP;
////            JW_Data_Debug[17][sJW_Data_Debug_Index] = Y_ZMP;
////            JW_Data_Debug[18][sJW_Data_Debug_Index] = Y_ZMP;

////            printf("%f, %f\n",fsm->LeftFoot->Pos[0], fsm->LeftFoot->Pos[1]);
////            printf("%f, %f\n",fsm->LeftInfos[0][0], fsm->LeftFoot->Pos[0]);

////            JW_Data_Debug[0][sJW_Data_Debug_Index] = fsm->LeftFoot->Pos[0];
////            JW_Data_Debug[1][sJW_Data_Debug_Index] = fsm->LeftFoot->Pos[1];
////            JW_Data_Debug[2][sJW_Data_Debug_Index] = fsm->RightFoot->Pos[0];
////            JW_Data_Debug[3][sJW_Data_Debug_Index] = fsm->RightFoot->Pos[1];
////            JW_Data_Debug[4][sJW_Data_Debug_Index] = fsm->ZMPPos->ZMP[0];
////            JW_Data_Debug[5][sJW_Data_Debug_Index] = fsm->ZMPPos->ZMP[1];
////            JW_Data_Debug[6][sJW_Data_Debug_Index] = fsm->LeftFoot->Pos[2];
////            JW_Data_Debug[7][sJW_Data_Debug_Index] = fsm->RightFoot->Pos[2];
////            JW_Data_Debug[8][sJW_Data_Debug_Index] = fsm->LeftInfos[0][2];
////            JW_Data_Debug[9][sJW_Data_Debug_Index] = fsm->RightInfos[0][2];
//            //printf("%f, %f\n",fsm->LeftFoot->Pos[0], fsm->LeftFoot->Pos[1]);
//            sJW_Data_Debug_Index++;
//            if(sJW_Data_Debug_Index >= ROW_data_debug) sJW_Data_Debug_Index = 0;
//        }
//    }
}

void JW_save2_Motion_check()
{
//    if(sJW_Data_Debug_Index2 < ROW_data_debug)
//    {
//        JW_Data_Debug2[0][sJW_Data_Debug_Index2] = checkfsm->LeftInfos[0][0];
//        JW_Data_Debug2[1][sJW_Data_Debug_Index2] = checkfsm->LeftInfos[0][1];
//        JW_Data_Debug2[2][sJW_Data_Debug_Index2] = checkfsm->LeftInfos[0][2];

//        JW_Data_Debug2[3][sJW_Data_Debug_Index2] = checkfsm->RightInfos[0][0];
//        JW_Data_Debug2[4][sJW_Data_Debug_Index2] = checkfsm->RightInfos[0][1];
//        JW_Data_Debug2[5][sJW_Data_Debug_Index2] = checkfsm->RightInfos[0][2];

//        JW_Data_Debug2[6][sJW_Data_Debug_Index2] = checkfsm->LeftInfos[0][3];
//        JW_Data_Debug2[7][sJW_Data_Debug_Index2] = checkfsm->LeftInfos[0][4];
//        JW_Data_Debug2[8][sJW_Data_Debug_Index2] = checkfsm->LeftInfos[0][5];

//        JW_Data_Debug2[9][sJW_Data_Debug_Index2] = checkfsm->RightInfos[0][3];
//        JW_Data_Debug2[10][sJW_Data_Debug_Index2] = checkfsm->RightInfos[0][4];
//        JW_Data_Debug2[11][sJW_Data_Debug_Index2] = checkfsm->RightInfos[0][5];

////        JW_Data_Debug2[0][sJW_Data_Debug_Index2] = TorsoRollAngle;//fsm->AddJointInfos[0][JRAR];
////        JW_Data_Debug2[1][sJW_Data_Debug_Index2] = TorsoPitchAngle;//fsm->AddJointInfos[0][JLAR];
////        JW_Data_Debug2[2][sJW_Data_Debug_Index2] = (U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0);//fsm->AddJointInfos[0][JRAP];
////        JW_Data_Debug2[3][sJW_Data_Debug_Index2] = Del_PC_Y_DSP_YZMP_CON*G_DSP_Y;//fsm->AddJointInfos[0][JLAP];
////        JW_Data_Debug2[4][sJW_Data_Debug_Index2] = I_ZMP_CON_Y;//fsm->AddJointInfos[0][JRHR];
////        JW_Data_Debug2[5][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLHR];
////        JW_Data_Debug2[6][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JRHP];
////        JW_Data_Debug2[7][sJW_Data_Debug_Index2] = fsm->AddJointInfos[0][JLHP];

////        JW_Data_Debug2[4][sJW_Data_Debug_Index2] = WBIK_Q[idX];
////        JW_Data_Debug2[5][sJW_Data_Debug_Index2] = WBIK_Q[idY];
////        JW_Data_Debug2[6][sJW_Data_Debug_Index2] = WBIK_Q[idZ];
////        JW_Data_Debug2[7][sJW_Data_Debug_Index2] = 1000;//fsm->AddLeftFootInfos[0][2];

////        JW_Data_Debug2[8][sJW_Data_Debug_Index2] = WBIK_Q[RHY+7];
////        JW_Data_Debug2[9][sJW_Data_Debug_Index2] = WBIK_Q[RHR+7];
////        JW_Data_Debug2[10][sJW_Data_Debug_Index2] = WBIK_Q[RHP+7];
////        JW_Data_Debug2[11][sJW_Data_Debug_Index2] = WBIK_Q[RKN+7];
//        JW_Data_Debug2[12][sJW_Data_Debug_Index2] = WBIK_Q[RAR+7];
//        JW_Data_Debug2[13][sJW_Data_Debug_Index2] = WBIK_Q[RAP+7];

//        JW_Data_Debug2[14][sJW_Data_Debug_Index2] = WBIK_Q[LHY+7];
//        JW_Data_Debug2[15][sJW_Data_Debug_Index2] = WBIK_Q[LHR+7];
//        JW_Data_Debug2[16][sJW_Data_Debug_Index2] = WBIK_Q[LHP+7];
//        JW_Data_Debug2[17][sJW_Data_Debug_Index2] = WBIK_Q[LKN+7];
//        JW_Data_Debug2[18][sJW_Data_Debug_Index2] = WBIK_Q[LAR+7];
//        JW_Data_Debug2[19][sJW_Data_Debug_Index2] = WBIK_Q[LAP+7];

//        JW_Data_Debug2[21][sJW_Data_Debug_Index2] = des_pCOM_3x1[0];
//        JW_Data_Debug2[22][sJW_Data_Debug_Index2] = des_pCOM_3x1[1];
//        JW_Data_Debug2[23][sJW_Data_Debug_Index2] = des_pCOM_3x1[2];

//        //JW_Data_Debug2[24][sJW_Data_Debug_Index2] = des_pPCz;

//        JW_Data_Debug2[24][sJW_Data_Debug_Index2] = des_qPEL_4x1[0];
//        JW_Data_Debug2[25][sJW_Data_Debug_Index2] = des_qPEL_4x1[1];
//        JW_Data_Debug2[26][sJW_Data_Debug_Index2] = des_qPEL_4x1[2];
//        JW_Data_Debug2[27][sJW_Data_Debug_Index2] = des_qPEL_4x1[3];

//        JW_Data_Debug2[28][sJW_Data_Debug_Index2] = des_pRF_3x1[0];
//        JW_Data_Debug2[29][sJW_Data_Debug_Index2] = des_pRF_3x1[1];
//        JW_Data_Debug2[30][sJW_Data_Debug_Index2] = des_pRF_3x1[2];

//        JW_Data_Debug2[31][sJW_Data_Debug_Index2] = des_qRF_4x1[0];
//        JW_Data_Debug2[32][sJW_Data_Debug_Index2] = des_qRF_4x1[1];
//        JW_Data_Debug2[33][sJW_Data_Debug_Index2] = des_qRF_4x1[2];
//        JW_Data_Debug2[34][sJW_Data_Debug_Index2] = des_qRF_4x1[3];

//        JW_Data_Debug2[35][sJW_Data_Debug_Index2] = des_pLF_3x1[0];
//        JW_Data_Debug2[36][sJW_Data_Debug_Index2] = des_pLF_3x1[1];
//        JW_Data_Debug2[37][sJW_Data_Debug_Index2] = des_pLF_3x1[2];

//        JW_Data_Debug2[38][sJW_Data_Debug_Index2] = des_qLF_4x1[0];
//        JW_Data_Debug2[39][sJW_Data_Debug_Index2] = des_qLF_4x1[1];
//        JW_Data_Debug2[40][sJW_Data_Debug_Index2] = des_qLF_4x1[2];
//        JW_Data_Debug2[41][sJW_Data_Debug_Index2] = des_qLF_4x1[3];

////        JW_Data_Debug2[12][sJW_Data_Debug_Index2] = sharedData->Joint[LAR].CurrentPosition;//sharedData->ZMP[2];
////        JW_Data_Debug2[13][sJW_Data_Debug_Index2] = sharedData->ZMP[3];
////        JW_Data_Debug2[14][sJW_Data_Debug_Index2] = sharedData->ZMP[4];
////        JW_Data_Debug2[15][sJW_Data_Debug_Index2] = sharedData->ZMP[5];

//        JW_Data_Debug2[42][sJW_Data_Debug_Index2] = sharedData->IMUPitch[CIMU];
//        JW_Data_Debug2[43][sJW_Data_Debug_Index2] = sharedData->IMURoll[CIMU];

//        JW_Data_Debug2[44][sJW_Data_Debug_Index2] = sharedData->IMUPitchVel[CIMU];
//        JW_Data_Debug2[45][sJW_Data_Debug_Index2] = sharedData->IMURollVel[CIMU];

//        JW_Data_Debug2[46][sJW_Data_Debug_Index2] = sharedData->IMUAccX[CIMU];
//        JW_Data_Debug2[47][sJW_Data_Debug_Index2] = sharedData->IMUAccY[CIMU];
//        JW_Data_Debug2[48][sJW_Data_Debug_Index2] = sharedData->IMUAccZ[CIMU]*9.81/1000.;

//        JW_Data_Debug2[49][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHY];
//        JW_Data_Debug2[50][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHR];
//        JW_Data_Debug2[51][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RHP];
//        JW_Data_Debug2[52][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RKN];
//        JW_Data_Debug2[53][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RAP];
//        JW_Data_Debug2[54][sJW_Data_Debug_Index2] = FWRefAngleCurrent[RAR];

//        JW_Data_Debug2[55][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHY];
//        JW_Data_Debug2[56][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHR];
//        JW_Data_Debug2[57][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LHP];
//        JW_Data_Debug2[58][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LKN];
//        JW_Data_Debug2[59][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LAP];
//        JW_Data_Debug2[60][sJW_Data_Debug_Index2] = FWRefAngleCurrent[LAR];

//        JW_Data_Debug2[61][sJW_Data_Debug_Index2] = sharedSEN->FT[RAFT].Fz;
//        JW_Data_Debug2[62][sJW_Data_Debug_Index2] = sharedSEN->FT[RAFT].Mx;
//        JW_Data_Debug2[63][sJW_Data_Debug_Index2] = sharedSEN->FT[RAFT].My;

//        JW_Data_Debug2[64][sJW_Data_Debug_Index2] = sharedSEN->FT[LAFT].Fz;
//        JW_Data_Debug2[65][sJW_Data_Debug_Index2] = sharedSEN->FT[LAFT].Mx;
//        JW_Data_Debug2[66][sJW_Data_Debug_Index2] = sharedSEN->FT[LAFT].My;

//        JW_Data_Debug2[67][sJW_Data_Debug_Index2] = checkfsm->ZMPInfos[0][0];
//        JW_Data_Debug2[68][sJW_Data_Debug_Index2] = checkfsm->ZMPInfos[0][1];

//        JW_Data_Debug2[69][sJW_Data_Debug_Index2] = GLOBAL_X_LIPM_n;
//        JW_Data_Debug2[70][sJW_Data_Debug_Index2] = (U[0] + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0);
//        JW_Data_Debug2[71][sJW_Data_Debug_Index2] = CONT_X_n;
//        JW_Data_Debug2[72][sJW_Data_Debug_Index2] = CONT_Y_n;
//        JW_Data_Debug2[73][sJW_Data_Debug_Index2] = userData->WalkReadyCOM[Zdir] + GLOBAL_Z_LIPM + checkfsm->AddComInfos[0][2];;
//        JW_Data_Debug2[74][sJW_Data_Debug_Index2] = GLOBAL_X_RF;
//        JW_Data_Debug2[75][sJW_Data_Debug_Index2] = GLOBAL_Y_RF;
//        JW_Data_Debug2[76][sJW_Data_Debug_Index2] = GLOBAL_Z_RF;
//        JW_Data_Debug2[77][sJW_Data_Debug_Index2] = GLOBAL_X_LF;
//        JW_Data_Debug2[78][sJW_Data_Debug_Index2] = GLOBAL_Y_LF;
//        JW_Data_Debug2[79][sJW_Data_Debug_Index2] = GLOBAL_Z_LF;

//        JW_Data_Debug2[80][sJW_Data_Debug_Index2] = _legpos[0][0];
//        JW_Data_Debug2[81][sJW_Data_Debug_Index2] = _legpos[0][1];
//        JW_Data_Debug2[82][sJW_Data_Debug_Index2] = _legpos[0][2];
//        JW_Data_Debug2[83][sJW_Data_Debug_Index2] = _legpos[1][0];
//        JW_Data_Debug2[84][sJW_Data_Debug_Index2] = _legpos[1][1];
//        JW_Data_Debug2[85][sJW_Data_Debug_Index2] = _legpos[1][2];

//        JW_Data_Debug2[86][sJW_Data_Debug_Index2] = _legpos[2][0];
//        JW_Data_Debug2[87][sJW_Data_Debug_Index2] = _legpos[2][1];
//        JW_Data_Debug2[88][sJW_Data_Debug_Index2] = _legpos[2][2];
//        JW_Data_Debug2[89][sJW_Data_Debug_Index2] = _legpos[3][0];
//        JW_Data_Debug2[90][sJW_Data_Debug_Index2] = _legpos[3][1];
//        JW_Data_Debug2[91][sJW_Data_Debug_Index2] = _legpos[3][2];


//        JW_Data_Debug2[92][sJW_Data_Debug_Index2] = _shin_dist[0];
//        JW_Data_Debug2[93][sJW_Data_Debug_Index2] = _shin_dist[1];

//        JW_Data_Debug2[94][sJW_Data_Debug_Index2] = _foot_dist[0];
//        JW_Data_Debug2[95][sJW_Data_Debug_Index2] = _foot_dist[1];
//        JW_Data_Debug2[96][sJW_Data_Debug_Index2] = _foot_dist[2];
//        JW_Data_Debug2[97][sJW_Data_Debug_Index2] = _foot_dist[3];
////        JW_Data_Debug2[22][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].AccPitch;
////        JW_Data_Debug2[23][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].AccRoll;
////        JW_Data_Debug2[24][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].VelPitch;
////        JW_Data_Debug2[46][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].VelRoll;
////        JW_Data_Debug2[46][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].Pitch;
////        JW_Data_Debug2[47][sJW_Data_Debug_Index2] = sharedData->FTSensor[RAFT].Roll;

////        JW_Data_Debug2[28][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].dAccPitch;
////        JW_Data_Debug2[29][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].dAccRoll;
////        JW_Data_Debug2[30][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].AccPitch;
////        JW_Data_Debug2[31][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].AccRoll;
////        JW_Data_Debug2[32][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].VelPitch;
////        JW_Data_Debug2[33][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].VelRoll;
////        JW_Data_Debug2[34][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].Pitch;
////        JW_Data_Debug2[35][sJW_Data_Debug_Index2] = sharedData->FTSensor[LAFT].Roll;


//        sJW_Data_Debug_Index2++;
//        if(sJW_Data_Debug_Index2 >= ROW_data_debug) sJW_Data_Debug_Index2 = 0;
//    }
}



//-----------------------------------For motion check
//-----------------------------------For motion check
//-----------------------------------For motion check

void Preliminary_Motion_check()
{
    //   long t,dt;
    //   t = myclock();
    //   sleep(5);
        _preliminary_flag_Motion = false;
        double pv_state[2][3] = {{0.0f}}, pv_state_old[2][3] = {{0.0f}};
        double vir_pv_state[2][3] = {{0.0f}},vir_pv_state_old[2][3] = {{0.0f}},modification_state[2][3]= {{0.0f}},modification_state_old[2][3]= {{0.0f}};
        //static double pv_ZMP[2] = {0.f,};
        static double vir_pv_Err[2]= {0.f,};
        static double pv_U[2] = {0.f,},vir_pv_U[2] = {0.f,};
        static double zmp_z_v = 0.0f,zmp_x_v = 0.0f,pv_z_ref = 0.0f,pv_z_acc_ref = 0.0f;
        //static double temp_buffer[2][300] = {{0.0f}};
        double temp_sum[2]={0.0,0.0,};

        //if(checkfsm->StateInfos[0][0] != STATE_FINISHED)
        {
       //      printf("Preliminary_preview test1 \n");

            if(pv_Index == 0){
                //memcpy(temp_Q_34x1,WBIK_Q,34*sizeof(double));
//                get_WBIK_Q_from_RefAngleCurrent();

////                WBIK_Q[idQ0] = des_qPEL_4x1[0];//1;
////                WBIK_Q[idQ1] = des_qPEL_4x1[1];//0;
////                WBIK_Q[idQ2] = des_qPEL_4x1[2];//0;
////                WBIK_Q[idQ3] = des_qPEL_4x1[3];//0;
//                WBIK_Q[idQ0] = 1;
//                WBIK_Q[idQ1] = 0;
//                WBIK_Q[idQ2] = 0;
//                WBIK_Q[idQ3] = 0;

//                kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
//                kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
//                kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

//                printf("Est RF = (%f,%f,%f) ,(%f,%f,%f,%f), LF = (%f,%f,%f) ,(%f,%f,%f,%f), COM = (%f,%f,%f), qPEL = (%f,%f,%f,%f)\n"
//                        ,FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2]
//                        ,FK_qRFoot_4x1[0],FK_qRFoot_4x1[1],FK_qRFoot_4x1[2],FK_qRFoot_4x1[3]
//                        ,FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]
//                        ,FK_qLFoot_4x1[0],FK_qLFoot_4x1[1],FK_qLFoot_4x1[2],FK_qLFoot_4x1[3]
//                        ,FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2]
//                        ,des_qPEL_4x1[0],des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3]);
//                printf("Des RF = (%f,%f,%f) ,(%f,%f,%f,%f), LF = (%f,%f,%f) ,(%f,%f,%f,%f), COM = (%f,%f,%f)\n"
//                        ,des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]
//                        ,des_qRF_4x1[0],des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3]
//                        ,des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]
//                        ,des_qLF_4x1[0],des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3]
//                        ,des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);

                zmp_offset[0] = FK_pCOM_3x1[0] -pv_state_old[0][0];
                zmp_offset[1] = FK_pCOM_3x1[1] -pv_state_old[1][0];

                pv_state_old[0][0] = FK_pCOM_3x1[0];//CONT_X_n;
    //            pv_state_old[0][0] = CONT_X_n;
                //pv_state_old[0][0] = pv_state_old[0][0];
                pv_state_old[0][1] = pv_state_old[0][1];
                pv_state_old[0][2] = pv_state_old[0][2];

                pv_state_old[1][0] = FK_pCOM_3x1[1];//CONT_Y_n;
    //            pv_state_old[1][0] = CONT_Y_n;
                //pv_state_old[1][0] = pv_state_old[1][0];
                pv_state_old[1][1] = pv_state_old[1][1];
                pv_state_old[1][2] = pv_state_old[1][2];

//                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n FK_pCOM_3x1[0] = %f, FK_pCOM_3x1[1] = %f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1]);
                for(pv_time_Index=0;pv_time_Index<2000;pv_time_Index++)
                {
                        if(pv_time_Index < 600)
                            Gain_offset[pv_time_Index] = 1;
                        else if(pv_time_Index < 660)
                            Gain_offset[pv_time_Index] = 1 - 0.5*(1 - cos(PI*(pv_time_Index-600)/60));
                        else
                            Gain_offset[pv_time_Index] = 0.;
                }
            }




            //-------------------------------------------------COM 1----------------------------------------------------//
            for(int i=0;i<=1;i++)
            {
                //=========================
                //pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];
                //pv_Err[i] = pv_Err[i] + pv_ZMP[i] - fsm->ZMPInfos[0][i];

                if(pv_Index == 0)
                {
                    _preliminary_state_I_Motion[i][0] = pv_state_old[i][0];
                    _preliminary_state_I_Motion[i][1] = pv_state_old[i][1];
                    _preliminary_state_I_Motion[i][2] = pv_state_old[i][2];

                    printf("preliminary_state_I: %f    pv_state_old: %f \n",_preliminary_state_I_Motion[i][0],pv_state_old[i][0]);

                }

                temp_sum[i] = 0.0f;
                for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
                {
                    temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(checkfsm->ZMPInfos[pv_time_Index][i]);
                }

                //pv_U[i] = -pv_Gi[1]*pv_Err[i] - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];
//                if(pv_Index == 0)
//                {
//                    pv_state_old[i][0] = _preliminary_state_I[i][0];
//                    pv_state_old[i][1] = _preliminary_state_I[i][1];
//                    pv_state_old[i][2] = _preliminary_state_I[i][2];

//                    printf("preliminary_state_I: %f    pv_state_old: %f \n",_preliminary_state_I[i][0],pv_state_old[i][0]);

//                }

                pv_U[i] =  - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

                pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
                pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
                pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

                pv_state_old[i][0] = pv_state[i][0];
                pv_state_old[i][1] = pv_state[i][1];
                pv_state_old[i][2] = pv_state[i][2];

                _preliminary_state_I_Motion[i][0] = pv_state[i][0];
                _preliminary_state_I_Motion[i][1] = pv_state[i][1];
                _preliminary_state_I_Motion[i][2] = pv_state[i][2];
            }

     //       printf("preliminary pv_state %f \n",pv_state_old[0][0]);

            _temp_com = pv_state[0][0];


            //-----------------------------------------------------COM 2-----------------------------------------------------------//
            for(int k = 0;k < 2;k++)
            {
                    //      printf("Preliminary_preview test3 \n");

                for(int i=0;i<=299;i++)
                {

                    pv_z_ref = checkfsm->VirtualComInfos[i][1] + (userData->WalkReadyCOM[Zdir]+COM_Offset); //0.05f*sin(2.0f*PI*0.002f*((double)(pv_Index + i))) + 0.71f; //+ 0.71f;
                    pv_z_acc_ref = checkfsm->VirtualComAccInfos[i][1]; //-0.05f*2.0f*PI*2.0f*PI*sin(2.0f*PI*0.002f*((double)(pv_Index + i)));

                    pv_C_v[0][0] = 1.0f;
                    pv_C_v[0][1] = 0.0f;
                    pv_C_v[0][2] = - pv_z_ref/(pv_z_acc_ref + 9.81f);

                    //--------------------------OUTPUT ZMP
                    zmp_x_v = pv_C_v[0][0]*vir_pv_state_old[k][0] + pv_C_v[0][1]*vir_pv_state_old[k][1]+ pv_C_v[0][2]*vir_pv_state_old[k][2];

                    //Virtual Plane O.K
                    zmp_z_v = pv_z_ref - (userData->WalkReadyCOM[Zdir]+COM_Offset) - (userData->WalkReadyCOM[Zdir]+COM_Offset)*pv_z_acc_ref/9.81f;

                    //ZMP Error from Virtual plane-
                    //-------------------------------------------------------------------------------------------------------------------------------
                    _virtual_zmp_error_Motion[k][i] = (zmp_x_v - checkfsm->ZMPInfos[i+1][k])*(1.0f - zmp_z_v)/pv_z_ref;

                    //-------------------------------------------------------------------------------------------------------------------------------

                    //pr_Err_z[k][i] = (zmp_x_v - checkfsm->ZMPInfos[i+1][k])*(1.0f - zmp_z_v)/pv_z_ref;

                    temp_sum[k] = 0.0f;

                    for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
                    {
                        temp_sum[k] = temp_sum[k] + pv_Gd[1 + pv_time_Index]*(checkfsm->ZMPInfos[pv_time_Index + i][k]);
                    }

                    pv_U[k] =  - (pv_Gx[1]*vir_pv_state_old[k][0] + pv_Gx[2]*vir_pv_state_old[k][1] + pv_Gx[3]*vir_pv_state_old[k][2]) - temp_sum[k];


                    vir_pv_state[k][0] = (pv_A[0][0]*vir_pv_state_old[k][0] + pv_A[0][1]*vir_pv_state_old[k][1] + pv_A[0][2]*vir_pv_state_old[k][2]) + (pv_B[0][0])*pv_U[k];
                    vir_pv_state[k][1] = (pv_A[1][0]*vir_pv_state_old[k][0] + pv_A[1][1]*vir_pv_state_old[k][1] + pv_A[1][2]*vir_pv_state_old[k][2]) + (pv_B[1][0])*pv_U[k];
                    vir_pv_state[k][2] = (pv_A[2][0]*vir_pv_state_old[k][0] + pv_A[2][1]*vir_pv_state_old[k][1] + pv_A[2][2]*vir_pv_state_old[k][2]) + (pv_B[2][0])*pv_U[k];

                    vir_pv_state_old[k][0] = vir_pv_state[k][0];
                    vir_pv_state_old[k][1] = vir_pv_state[k][1];
                    vir_pv_state_old[k][2] = vir_pv_state[k][2];

                    //printf("Preliminary_preview test4 \n");
                }

               //-------------------------------sum of virtual zmp error

                _preliminary_state_298_Motion[k][0] = vir_pv_state_old[k][0];
                _preliminary_state_298_Motion[k][1] = vir_pv_state_old[k][1];
                _preliminary_state_298_Motion[k][2] = vir_pv_state_old[k][2];

               vir_pv_Err[k] = 0.0f;

               for(int i = 0;i<=299;i++)
               {
           //        printf("Preliminary_preview test5 \n");
                   vir_pv_Err[k] = vir_pv_Err[k] + pv_Gd[i+1]*_virtual_zmp_error_Motion[k][i];
               }
//               if(k == 0)
//               {
//                   _temp_debug_data4 = vir_pv_Err[k];
//               }

               //-------------------------------Control input2
               vir_pv_U[k] = -pv_Gx[1]*modification_state_old[k][0] - pv_Gx[2]*modification_state_old[k][1] - pv_Gx[3]*modification_state_old[k][2] + vir_pv_Err[k];

               modification_state[k][0] = (pv_A[0][0]*modification_state_old[k][0] + pv_A[0][1]*modification_state_old[k][1] + pv_A[0][2]*modification_state_old[k][2]) + (pv_B[0][0])*vir_pv_U[k];
               modification_state[k][1] = (pv_A[1][0]*modification_state_old[k][0] + pv_A[1][1]*modification_state_old[k][1] + pv_A[1][2]*modification_state_old[k][2]) + (pv_B[1][0])*vir_pv_U[k];
               modification_state[k][2] = (pv_A[2][0]*modification_state_old[k][0] + pv_A[2][1]*modification_state_old[k][1] + pv_A[2][2]*modification_state_old[k][2]) + (pv_B[2][0])*vir_pv_U[k];

                modification_state_old[k][0] = modification_state[k][0];
                modification_state_old[k][1] = modification_state[k][1];
                modification_state_old[k][2] = modification_state[k][2];

                pv_state[k][0] =   modification_state_old[k][0] + pv_state_old[k][0];
                pv_state[k][1] =   modification_state_old[k][1] + pv_state_old[k][1];
                pv_state[k][2] =   modification_state_old[k][2] + pv_state_old[k][2];

                _preliminary_state_II_Motion[k][0] = modification_state[k][0];
                _preliminary_state_II_Motion[k][1] = modification_state[k][1];
                _preliminary_state_II_Motion[k][2] = modification_state[k][2];
            }

           GLOBAL_Y_LIPM = pv_state[1][0];
           GLOBAL_X_LIPM = pv_state[0][0];

           GLOBAL_Y_LIPM_d = pv_state[1][1];
           GLOBAL_X_LIPM_d = pv_state[0][1];

           GLOBAL_X_RF = checkfsm->RightInfos[0][0];
           GLOBAL_X_LF = checkfsm->LeftInfos[0][0];

           GLOBAL_Y_RF = checkfsm->RightInfos[0][1];
           GLOBAL_Y_LF = checkfsm->LeftInfos[0][1];

           GLOBAL_ZMP_REF_X = checkfsm->ZMPInfos[0][0];
           GLOBAL_ZMP_REF_Y = checkfsm->ZMPInfos[0][1];


//           GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((checkfsm->Right-Infos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((fsm->RightInfos[0][3] + fsm->LeftInfos[0][3])*D2R/2.);

           GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_ZMP_REF_Y_n = -GLOBAL_ZMP_REF_X*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);


//           step_length = GLOBAL_X_RF - GLOBAL_X_LF;
//           theta_step_length = atan2(0.2,step_length);
//           GLOBAL_ZMP_REF_X_nn =  GLOBAL_ZMP_REF_X_n*cos(theta_step_length) + GLOBAL_ZMP_REF_Y_n*sin(theta_step_length);
//           GLOBAL_ZMP_REF_Y_nn = -GLOBAL_ZMP_REF_X_n*sin(theta_step_length) + GLOBAL_ZMP_REF_Y_n*cos(theta_step_length);

           GLOBAL_X_LIPM_n =  GLOBAL_X_LIPM*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_LIPM_n = -GLOBAL_X_LIPM*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

           GLOBAL_X_LIPM_d_n =  GLOBAL_X_LIPM_d*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_LIPM_d_n = -GLOBAL_X_LIPM_d*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

           GLOBAL_X_RF_n =  GLOBAL_X_RF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_RF_n = -GLOBAL_X_RF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

           GLOBAL_X_LF_n =  GLOBAL_X_LF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
           GLOBAL_Y_LF_n = -GLOBAL_X_LF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

//           GLOBAL_Z_RF = checkfsm->RightInfos[0][2];
//           GLOBAL_Z_LF = checkfsm->LeftInfos[0][2];
           //------------------------------------------------------ GLOBAL Z REF ---------------------------------------------------------//
    //       GLOBAL_Z_LIPM =  0.05f*sin(2.0f*PI*0.002f*((double)(pv_Index )));
           pv_Index++;
       }
}


void ThreeDim_Preview_Motion_check()
{
    static double pv_state[2][3] = {{0.0f}},pv_state_old[2][3] = {{0.0f}};
    static double vir_pv_state[2][3] = {{0.0f}},vir_pv_state_old[2][3] = {{0.0f}},modification_state[2][3]= {{0.0f}},modification_state_old[2][3]= {{0.0f}};
    static double pv_ZMP[2] = {0.f,};
    static double temp_virtual_zmp_error = 0.0f;
    static double vir_pv_Err[2]= {0.f,};
    static double pv_U[2] = {0.f,},vir_pv_U[2] = {0.f,};
    static double zmp_z_v = 0.0f,zmp_x_v = 0.0f,pv_z_ref = 0.0f,pv_z_acc_ref = 0.0f;
    double temp_sum[2]={0.,0,};

    //if(checkfsm->StateInfos[0][0] != STATE_FINISHED)
    {

        if(pv_Index == 1 && _preliminary_flag_Motion == false )
        {
            pv_state_old[0][0] = _preliminary_state_I_Motion[0][0];
            pv_state_old[0][1] = _preliminary_state_I_Motion[0][1];
            pv_state_old[0][2] = _preliminary_state_I_Motion[0][2];

            pv_state_old[1][0] = _preliminary_state_I_Motion[1][0];
            pv_state_old[1][1] = _preliminary_state_I_Motion[1][1];
            pv_state_old[1][2] = _preliminary_state_I_Motion[1][2];
        }

//        static double temp_Q_34x1[34] = {0,};

        for(int i=0;i<=1;i++)
        {
            //printf("ZMPInfos[pv_time_Index][i] = %f\n",checkfsm->ZMPInfos[pv_time_Index][0]);
            pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];

            temp_sum[i] = 0.0f;

            for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
            {
                temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(checkfsm->ZMPInfos[pv_time_Index][i]);
            }

            pv_U[i] =  - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

            pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
            pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
            pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];

            if(i==1)
            {
                _temp_debug_data[0] = pv_state_old[i][0];
                _temp_debug_data[4] = pv_ZMP[i];
            }else
            {
                _temp_debug_data[5] = pv_state_old[i][0];
                _temp_debug_data[9] = pv_ZMP[i];
            }
        }

        _temp_com = pv_state[0][0];

        if(pv_Index == 1 && _preliminary_flag_Motion == false)
        {
            vir_pv_state_old[0][0] = _preliminary_state_298_Motion[0][0];
            vir_pv_state_old[0][1] = _preliminary_state_298_Motion[0][1];
            vir_pv_state_old[0][2] = _preliminary_state_298_Motion[0][2];

            vir_pv_state_old[1][0] = _preliminary_state_298_Motion[1][0];
            vir_pv_state_old[1][1] = _preliminary_state_298_Motion[1][1];
            vir_pv_state_old[1][2] = _preliminary_state_298_Motion[1][2];

            modification_state_old[0][0] = 0.0f;
            modification_state_old[0][1] = 0.0f;
            modification_state_old[0][2] = 0.0f;

            modification_state_old[1][0] = 0.0f;
            modification_state_old[1][1] = 0.0f;
            modification_state_old[1][2] = 0.0f;

            _preliminary_flag_Motion = true;
        }


        if(pv_Index == 0 && _preliminary_flag_Motion == true)
        {
            modification_state_old[0][0] = 0.0f;
            modification_state_old[0][1] = 0.0f;
            modification_state_old[0][2] = 0.0f;

            modification_state_old[1][0] = 0.0f;
            modification_state_old[1][1] = 0.0f;
            modification_state_old[1][2] = 0.0f;
        }




        for(int k = 0;k < 2;k++)
        {
            int i=299;
            pv_z_ref = checkfsm->VirtualComInfos[i][1] + (userData->WalkReadyCOM[Zdir]+COM_Offset);
            pv_z_acc_ref = checkfsm->VirtualComAccInfos[i][1];

            pv_C_v[0][0] = 1.0f; pv_C_v[0][1] = 0.0f; pv_C_v[0][2] = - pv_z_ref/(pv_z_acc_ref + 9.81f);


            zmp_x_v = pv_C_v[0][0]*vir_pv_state_old[k][0] + pv_C_v[0][1]*vir_pv_state_old[k][1]+ pv_C_v[0][2]*vir_pv_state_old[k][2];

            zmp_z_v = pv_z_ref - (userData->WalkReadyCOM[Zdir]+COM_Offset) - (userData->WalkReadyCOM[Zdir]+COM_Offset)*pv_z_acc_ref/9.81f;

            temp_virtual_zmp_error = (zmp_x_v - checkfsm->ZMPInfos[i][k])*(1.0f - zmp_z_v)/pv_z_ref;

            for(int j=0;j<=298;j++)
            {
                _virtual_zmp_error_Motion[k][j] = _virtual_zmp_error_Motion[k][j+1];
            }

            _virtual_zmp_error_Motion[k][i] = temp_virtual_zmp_error;

            temp_sum[k] = 0.0f;

            for(pv_time_Index=0;pv_time_Index<=299;pv_time_Index++)
            {
                temp_sum[k] = temp_sum[k] + pv_Gd[1 + pv_time_Index]*(checkfsm->ZMPInfos[pv_time_Index + i][k]);
            }

             pv_U[k] =  - (pv_Gx[1]*vir_pv_state_old[k][0] + pv_Gx[2]*vir_pv_state_old[k][1] + pv_Gx[3]*vir_pv_state_old[k][2]) - temp_sum[k];

             vir_pv_state[k][0] = (pv_A[0][0]*vir_pv_state_old[k][0] + pv_A[0][1]*vir_pv_state_old[k][1] + pv_A[0][2]*vir_pv_state_old[k][2]) + (pv_B[0][0])*pv_U[k];
             vir_pv_state[k][1] = (pv_A[1][0]*vir_pv_state_old[k][0] + pv_A[1][1]*vir_pv_state_old[k][1] + pv_A[1][2]*vir_pv_state_old[k][2]) + (pv_B[1][0])*pv_U[k];
             vir_pv_state[k][2] = (pv_A[2][0]*vir_pv_state_old[k][0] + pv_A[2][1]*vir_pv_state_old[k][1] + pv_A[2][2]*vir_pv_state_old[k][2]) + (pv_B[2][0])*pv_U[k];

             vir_pv_state_old[k][0] = vir_pv_state[k][0];
             vir_pv_state_old[k][1] = vir_pv_state[k][1];
             vir_pv_state_old[k][2] = vir_pv_state[k][2];

            vir_pv_Err[k] = 0.0f;

            for(int i = 0;i<=299;i++)
            {
                 vir_pv_Err[k] = vir_pv_Err[k] + pv_Gd[i+1]*_virtual_zmp_error[k][i];
            }

            vir_pv_U[k] = -pv_Gx[1]*modification_state_old[k][0] - pv_Gx[2]*modification_state_old[k][1] - pv_Gx[3]*modification_state_old[k][2] + vir_pv_Err[k];

            modification_state[k][0] = (pv_A[0][0]*modification_state_old[k][0] + pv_A[0][1]*modification_state_old[k][1] + pv_A[0][2]*modification_state_old[k][2]) + (pv_B[0][0])*vir_pv_U[k];
            modification_state[k][1] = (pv_A[1][0]*modification_state_old[k][0] + pv_A[1][1]*modification_state_old[k][1] + pv_A[1][2]*modification_state_old[k][2]) + (pv_B[1][0])*vir_pv_U[k];
            modification_state[k][2] = (pv_A[2][0]*modification_state_old[k][0] + pv_A[2][1]*modification_state_old[k][1] + pv_A[2][2]*modification_state_old[k][2]) + (pv_B[2][0])*vir_pv_U[k];

            modification_state_old[k][0] = modification_state[k][0];
            modification_state_old[k][1] = modification_state[k][1];
            modification_state_old[k][2] = modification_state[k][2];

            pv_state[k][0] =   modification_state_old[k][0] + pv_state_old[k][0];
            pv_state[k][1] =   modification_state_old[k][1] + pv_state_old[k][1];
            pv_state[k][2] =   modification_state_old[k][2] + pv_state_old[k][2];

            if(k == 1)
            {
                _temp_debug_data[1] = modification_state_old[k][0];
                _temp_debug_data[2] = pv_state[k][0];
                _temp_debug_data[3] = zmp_x_v;
            }else
            {
                _temp_debug_data[6] = modification_state_old[k][0];
                _temp_debug_data[7] = pv_state[k][0];
                _temp_debug_data[8] = zmp_x_v;
            }

        }


        GLOBAL_Y_LIPM = pv_state[1][0];
        GLOBAL_X_LIPM = pv_state[0][0];

        GLOBAL_Y_LIPM_d = pv_state[1][1];
        GLOBAL_X_LIPM_d = pv_state[0][1];

        GLOBAL_X_RF = checkfsm->RightInfos[0][0];
        GLOBAL_X_LF = checkfsm->LeftInfos[0][0];

        GLOBAL_Y_RF = checkfsm->RightInfos[0][1];
        GLOBAL_Y_LF = checkfsm->LeftInfos[0][1];

        GLOBAL_ZMP_REF_X = checkfsm->ZMPInfos[0][0];
        GLOBAL_ZMP_REF_Y = checkfsm->ZMPInfos[0][1];

        GLOBAL_ZMP_REF_X_n =  GLOBAL_ZMP_REF_X*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_ZMP_REF_Y_n = -GLOBAL_ZMP_REF_X*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_ZMP_REF_Y*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_LIPM_n =  GLOBAL_X_LIPM*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LIPM_n = -GLOBAL_X_LIPM*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_LIPM_d_n =  GLOBAL_X_LIPM_d*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LIPM_d_n = -GLOBAL_X_LIPM_d*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LIPM_d*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_RF_n =  GLOBAL_X_RF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_RF_n = -GLOBAL_X_RF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_RF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

        GLOBAL_X_LF_n =  GLOBAL_X_LF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
        GLOBAL_Y_LF_n = -GLOBAL_X_LF*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + GLOBAL_Y_LF*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

//        GLOBAL_Z_RF = checkfsm->RightInfos[0][2];
//        GLOBAL_Z_LF = checkfsm->LeftInfos[0][2];

        pv_Index++;
    }
}


//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------




void WBIK_Motion_check()
{
    //double tempPEL[4];
    double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4];
    double temp1des_qRF_4x1[4],temp2des_qRF_4x1[4],temp3des_qRF_4x1[4],temp4des_qRF_4x1[4],temp5des_qRF_4x1[4];
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
    double RightYaw,RightRoll,RightPitch,LeftYaw,LeftRoll,LeftPitch;
    double  last_theta[12],theta_diff[12];
//    double Rangle[6],Langle[6],leg[2][3],obj[2][3],dis[1],Lpos[6][3],Rpos[6][3],vision[2][3];

    if(pv_Index == 1){
        HUBO2ZMPInitLegLength(0.,0.,0);
        Add_FootTask[RIGHT][Zdir] = 0.;
        Add_FootTask[LEFT][Zdir] = 0.;

        HUBO2TorsoInitConRoll(0.0, 0.0, 0);
        HUBO2TorsoInitConPitch(0.0, 0.0, 0);
        TorsoRollAngle = 0;
        TorsoPitchAngle = 0;

        PitchRoll_Ori_Integral(0.0, 0.0, 0);
        BTW_FOOT_qPEL_comp_4x1[0] = 1;
        BTW_FOOT_qPEL_comp_4x1[1] = 0;
        BTW_FOOT_qPEL_comp_4x1[2] = 0;
        BTW_FOOT_qPEL_comp_4x1[3] = 0;

        kirkZMPCon_XP2(0,0,0);
        kirkZMPCon_YP2(0,0,0);
        Del_PC_X_DSP_XZMP_CON = 0;
        Del_PC_Y_DSP_YZMP_CON = 0;

        I_ZMP_CON_X = 0.0f;
        I_ZMP_CON_Y = 0.0f;

        Dif_enc_RAP = 0;
        Dif_enc_RAR = 0;
        Dif_enc_LAP = 0;
        Dif_enc_LAR = 0;
        HUBO2TorqInitConRAR(0.0, 0.0, 0.0,0.);
        HUBO2TorqInitConLAR(0.0, 0.0, 0.0,0.);
        HUBO2TorqInitConRAP(0.0, 0.0, 0.0,0.);
        HUBO2TorqInitConLAP(0.0, 0.0, 0.0,0.);

        WBIKTorqInitConRAR(0.0, 0.0, 0.0,0.);
        WBIKTorqInitConLAR(0.0, 0.0, 0.0,0.);
        WBIKTorqInitConRAP(0.0, 0.0, 0.0,0.);
        WBIKTorqInitConLAP(0.0, 0.0, 0.0,0.);

        AnkleRollAngle[HUBO_RIGHT] = 0;
        AnkleRollAngle[HUBO_LEFT] = 0;
        AnklePitchAngle[HUBO_RIGHT] = 0;
        AnklePitchAngle[HUBO_LEFT] = 0;

        WBIK_Torq[RIGHT][Xdir] = 0.;
        WBIK_Torq[RIGHT][Ydir] = 0.;
        WBIK_Torq[LEFT][Xdir] = 0.;
        WBIK_Torq[LEFT][Ydir] = 0.;

        qt_WBIK_Torq_RF[0] = 1.;
        qt_WBIK_Torq_RF[1] = 0.;
        qt_WBIK_Torq_RF[2] = 0.;
        qt_WBIK_Torq_RF[3] = 0.;

        qt_WBIK_Torq_LF[0] = 1.;
        qt_WBIK_Torq_LF[1] = 0.;
        qt_WBIK_Torq_LF[2] = 0.;
        qt_WBIK_Torq_LF[3] = 0.;
    }

    // 1 . PELVIS Orientation
    qtRZ(1.0*(checkfsm->RightInfos[0][3]*D2R + checkfsm->LeftInfos[0][3]*D2R)/2., temp1des_qPEL_4x1);
    qtRX(1*TorsoRollAngle*D2R, temp2des_qPEL_4x1);
    qtRY(1*TorsoPitchAngle*D2R, temp4des_qPEL_4x1);

    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);

    // 2 . COM Position
    CONT_X = GLOBAL_X_LIPM_n;// +(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X + I_ZMP_CON_X)*ZMP_FeedBack_ONOFF;
    //CONT_Y = ((GLOBAL_Y_LIPM_n-(GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*(1-U_Gain) + U[0]*U_Gain + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0) - 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f + I_ZMP_CON_Y;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
    //CONT_Y = (U[0]*0.92 + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain);//+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f+ I_ZMP_CON_Y)*ZMP_FeedBack_ONOFF;// ;//;// + (GLOBAL_Y_LIPM_n)*(1-U_Gain) ;//- 0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*1.0f;// + Add_COMTask_FootPrint[0][Ydir]+ Add_COMTask_FootPrint[1][Ydir];
    CONT_Y = GLOBAL_Y_LIPM_n;//

    CONT_X_n = CONT_X*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) - CONT_Y*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);
    CONT_Y_n = CONT_X*sin((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.) + CONT_Y*cos((checkfsm->RightInfos[0][3] + checkfsm->LeftInfos[0][3])*D2R/2.);

    des_pCOM_3x1[Xdir] = CONT_X_n;
    des_pCOM_3x1[Ydir] = CONT_Y_n;
    des_pCOM_3x1[Zdir] = userData->WalkReadyCOM[Zdir] + checkfsm->AddComInfos[0][2];// + GLOBAL_Z_LIPM;// - (checkfsm->AddRightFootInfos[0][2] + checkfsm->AddLeftFootInfos[0][2])*0.7;

    //================= Right Foot ======================
    // 3 . RF Position
    des_pRF_3x1[Xdir] = GLOBAL_X_RF;
    des_pRF_3x1[Ydir] = GLOBAL_Y_RF;
    des_pRF_3x1[Zdir] = checkfsm->RightInfos[0][2];//GLOBAL_Z_RF;// + Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF;

    // 4 . RF Orietation
    RightYaw = checkfsm->RightInfos[0][3]*D2R;
    RightRoll = checkfsm->RightInfos[0][4]*D2R;
    RightPitch = checkfsm->RightInfos[0][5]*D2R;

    qtRZ(RightYaw, temp1des_qRF_4x1);
    qtRX(RightRoll, temp2des_qRF_4x1);
    qtRY(RightPitch, temp4des_qRF_4x1);

    QTcross(temp1des_qRF_4x1,temp2des_qRF_4x1,temp3des_qRF_4x1);
    QTcross(temp3des_qRF_4x1,temp4des_qRF_4x1,temp5des_qRF_4x1);
    des_qRF_4x1[0] = temp5des_qRF_4x1[0];
    des_qRF_4x1[1] = temp5des_qRF_4x1[1];
    des_qRF_4x1[2] = temp5des_qRF_4x1[2];
    des_qRF_4x1[3] = temp5des_qRF_4x1[3];
    //================= Left Foot =======================
    // 5 . LF Position
    des_pLF_3x1[Xdir] = 0.  + GLOBAL_X_LF ;
    des_pLF_3x1[Ydir] = GLOBAL_Y_LF;
    des_pLF_3x1[Zdir] = checkfsm->LeftInfos[0][2];//GLOBAL_Z_LF;// + Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF;

    // 6 . LF Orietation
    LeftYaw = 1*checkfsm->LeftInfos[0][3]*D2R;
    LeftRoll = checkfsm->LeftInfos[0][4]*D2R;
    LeftPitch = checkfsm->LeftInfos[0][5]*D2R;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp2des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp4des_qLF_4x1,temp5des_qLF_4x1);

    des_qLF_4x1[0] = temp5des_qLF_4x1[0];
    des_qLF_4x1[1] = temp5des_qLF_4x1[1];
    des_qLF_4x1[2] = temp5des_qLF_4x1[2];
    des_qLF_4x1[3] = temp5des_qLF_4x1[3];

//    double temp_4x1[4];
    //QTcross(des_qPEL_4x1,BTW_FOOT_qPEL_comp_4x1,temp_4x1);
    /* //Ankle Free Strategy at a "stepping on" timing
    temp_IK_return = kine_drc_hubo4.IK_LowerBody(des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1);

    for(int i=0; i<=LAR; i++) {
        FWRefAngleCurrent2[i] = WBIK_Q[i+7];
        FWRefAngleCurrent_last[i] = WBIK_Q[i+7];
    }
    ANKLE_FREE_LOCK();

    kine_drc_hubo4.FK_RightFoot(des_pRF_3x1, des_qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot(des_pLF_3x1, des_qLF_4x1);
    */


    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q2);


    // Sagging Compensation
    for(int i=0; i<=LAR; i++)
    {
        FWRefAngleCurrent[i] = WBIK_Q2[i+7]*R2D;
        last_theta[i] = WBIK_Q2[i+7]*R2D;
        theta_diff[i] = last_theta[i] - FWRefAngleCurrent[i];
        if(fabs(theta_diff[i]) > 5.0)
        {
            _AngleLimit_flag = true;
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");
            printf("Big transition !!!\n");

        }
    }

    if(FWRefAngleCurrent[RKN]<15.0f || FWRefAngleCurrent[LKN]<15.0f)
    {
//        if(_motion_check  == false)
//        {
            printf("Lengthlimit !!!\n");
            printf("Lengthlimit !!!\n");
            printf("Lengthlimit !!!\n");
            printf("Lengthlimit !!!\n");
            printf("Lengthlimit !!!\n");
//            _motion_check = true;
//        }
            _LengthLimit_flag = true;
    }


    if(FWRefAngleCurrent[RKN]>130.0f || FWRefAngleCurrent[LKN]>130.0f)
    {
//        if(_motion_check  == false)
//        {
            printf("Leg Length very short !!!\n");
            printf("Leg Length very short !!!\n");
            printf("Leg Length very short !!!\n");
            printf("Leg Length very short !!!\n");
            printf("Leg Length very short !!!\n");
//            _motion_check = true;
//        }
            _AngleLimit_flag = true;
    }

    if(FWRefAngleCurrent[RHP]<-99.0f || FWRefAngleCurrent[LHP]<-99.0f)
    {
//        if(_motion_check  == false)
//        {
            printf("Anglelimit !!!\n");
            printf("Anglelimit !!!\n");
            printf("Anglelimit !!!\n");
            printf("Anglelimit !!!\n");
            printf("Anglelimit !!!\n");

//            _motion_check = true;
//        }
            _AngleLimit_flag = true;
    }





//    if(FWRefAngleCurrent[RHP]<-99.0f || FWRefAngleCurrent[LHP]<-99.0f)
//    {
////        if(_motion_check  == false)
////        {
//            printf("Anglelimit !!!\n");
//            printf("Anglelimit !!!\n");
//            printf("Anglelimit !!!\n");
//            printf("Anglelimit !!!\n");
//            printf("Anglelimit !!!\n");

////            _motion_check = true;
////        }
//            _AngleLimit_flag = true;

//    }


    if(checkfsm->StateInfos[0][0] == STATE_DSP_INIT_RF || checkfsm->StateInfos[0][0] == STATE_DSP_INIT_LF){
        _motion_section_num = 0;
    }else if(checkfsm->StateInfos[0][0] == STATE_DSP_RF || checkfsm->StateInfos[0][0] == STATE_DSP_LF){
        _transition_flag = true;
    }else if(checkfsm->StateInfos[0][0] == STATE_SSP_RF){
        if(_motion_section_num == 0){
            _motion_section_num = 1;
        }
        else if(_motion_section_num == 1 && _transition_flag == true){
            _motion_section_num = 2;
        }
    }else if(checkfsm->StateInfos[0][0] == STATE_SSP_LF){
        if(_motion_section_num == 0){
            _motion_section_num = 1;
        }else if(_motion_section_num == 1 && _transition_flag == true){
            _motion_section_num = 2;
        }

    }else if(checkfsm->StateInfos[0][0] == STATE_DSP_FINAL_RFLF){
    }


//    //-----------------------------------Collision Check -----------------------------------------//
//    _pelvis_position[0] = WBIK_Q2[0];
//    _pelvis_position[1] = WBIK_Q2[1];
//    _pelvis_position[2] = WBIK_Q2[2];


////    obj[0][0] = vision[0][0];
////    obj[0][1] = vision[0][1];
////    obj[0][2] = vision[0][2];

////    obj[1][0] = vision[1][0];
////    obj[1][1] = vision[1][1];
////    obj[1][2] = vision[1][2];


//    obj[0][0] = 0.33;
//    obj[0][1] = -0.2;
//    obj[0][2] = 0.15;

//    obj[1][0] = 0.33;
//    obj[1][1] = 0.2;
//    obj[1][2] = 0.15;

//    Langle[0] = FWRefAngleCurrent[LHY];
//    Langle[1] = FWRefAngleCurrent[LHR];
//    Langle[2] = FWRefAngleCurrent[LHP];
//    Langle[3] = FWRefAngleCurrent[LKN];
//    Langle[4] = FWRefAngleCurrent[LAP];
//    Langle[5] = FWRefAngleCurrent[LAR];

//    LL_Configurtion(Langle,_pelvis_position,Lpos);

//    leg[0][0] =  _legpos[2][0] = Lpos[4][0];
//    leg[0][1] =  _legpos[2][1] = Lpos[4][1];
//    leg[0][2] =  _legpos[2][2] = Lpos[4][2];
//    leg[1][0] =  _legpos[3][0] = Lpos[3][0];
//    leg[1][1] =  _legpos[3][1] = Lpos[3][1];
//    leg[1][2] =  _legpos[3][2] = Lpos[3][2];

//    Leg_Collision_Check(leg,obj,dis);
//    _collision_dist[1] = dis[0];


//    Rangle[0] = FWRefAngleCurrent[RHY];
//    Rangle[1] = FWRefAngleCurrent[RHR];
//    Rangle[2] = FWRefAngleCurrent[RHP];
//    Rangle[3] = FWRefAngleCurrent[RKN];
//    Rangle[4] = FWRefAngleCurrent[RAP];
//    Rangle[5] = FWRefAngleCurrent[RAR];

//    RL_Configurtion(Rangle,_pelvis_position,Rpos);

//    leg[0][0] = _legpos[0][0] = Rpos[4][0];
//    leg[0][1] = _legpos[0][1] = Rpos[4][1];
//    leg[0][2] = _legpos[0][2] = Rpos[4][2];
//    leg[1][0] = _legpos[1][0] = Rpos[3][0];
//    leg[1][1] = _legpos[1][1] = Rpos[3][1];
//    leg[1][2] = _legpos[1][2] = Rpos[3][2];

//    Leg_Collision_Check(leg,obj,dis);

//    _collision_dist[0] = dis[0];

//    if(_collision_check == false)
//    {
//        if(_collision_dist[0] < 0.1f || _collision_dist[1] < 0.1f)
//        {
//            printf("collision!!!  collision!!!  collision!!! \n");
//            printf("collision!!!  collision!!!  collision!!! \n");
//            printf("collision!!!  collision!!!  collision!!! \n");
//            printf("collision!!!  collision!!!  collision!!! \n");
//            printf("collision!!!  collision!!!  collision!!! \n");
//            _collision_check = true;
//        }
//    }






    if(checkfsm->StateInfos[0][0] == STATE_FINISHED){

        _motion_check_start = false;
    }

}
//-----------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------//

void Rodrigues(double joint_axis[3],double theta,double R[3][3])
{
    double E[3][3], a_hat[3][3],aa_hat[3][3];

    E[0][0] = 1.0f;
    E[1][0] = 0.0f;
    E[2][0] = 0.0f;

    E[0][1] = 0.0f;
    E[1][1] = 1.0f;
    E[2][1] = 0.0f;

    E[0][2] = 0.0f;
    E[1][2] = 0.0f;
    E[2][2] = 1.0f;

    a_hat[0][0] = 0.0f;
    a_hat[1][0] = joint_axis[2];
    a_hat[2][0] = -joint_axis[1];

    a_hat[0][1] = -joint_axis[2];
    a_hat[1][1] = 0.0f;
    a_hat[2][1] = joint_axis[0];

    a_hat[0][2] = joint_axis[1];
    a_hat[1][2] = -joint_axis[0];
    a_hat[2][2] = 0.0f;

//    printf("111-----------------------------------------\n");
//    printf("%f   %f   %f \n",a_hat[0][0],a_hat[0][1],a_hat[0][2]);
//    printf("%f   %f   %f \n",a_hat[1][0],a_hat[1][1],a_hat[1][2]);
//    printf("%f   %f   %f \n",a_hat[2][0],a_hat[2][1],a_hat[2][2]);

    double sum = 0.0f;

    for(int i=0;i<=2;i++)
    {
        for(int k = 0;k<=2;k++)
        {
            for(int j=0;j<=2;j++)
            {
                sum = sum + a_hat[i][j]*a_hat[j][k];
           //     printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            aa_hat[i][k] = sum ;
            sum = 0.0f;
        }

    }



//    printf("-----------------------------------------\n");
//    printf("%f   %f   %f \n",aa_hat[0][0],aa_hat[0][1],aa_hat[0][2]);
//    printf("%f   %f   %f \n",aa_hat[1][0],aa_hat[1][1],aa_hat[1][2]);
//    printf("%f   %f   %f \n",aa_hat[2][0],aa_hat[2][1],aa_hat[2][2]);

    for(int i=0;i<=2;i++)
    {
        for(int k = 0;k<=2;k++)
        {
            R[i][k] = E[i][k] + a_hat[i][k]*sin(theta*D2R) + aa_hat[i][k]*(1.0f -cos(theta*D2R));

        }

    }

//    printf("333-----------------------------------------\n");
//    printf("%f   %f   %f \n",R[0][0],R[0][1],R[0][2]);
//    printf("%f   %f   %f \n",R[1][0],R[1][1],R[1][2]);
//    printf("%f   %f   %f \n",R[2][0],R[2][1],R[2][2]);


}


void mat3by3x3by3(double a[3][3],double b[3][3],double out[3][3])
{

    double sum = 0.0f;

    for(int i=0;i<=2;i++)
    {
        for(int k = 0;k<=2;k++)
        {
            for(int j=0;j<=2;j++)
            {
                sum = sum + a[i][j]*b[j][k];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat3by3plus3by3(double a[3][3],double b[3][3],double out[3][3])
{

    for(int i=0;i<=2;i++)
    {
        for(int k = 0;k<=2;k++)
        {
            out[i][k] = a[i][k] + b[i][k];

        }

    }

}


void Leg_Collision_Check(double Foot_pos[2][3],double obj_pos[2][3], double dis[1])
{
//    double  knee_pos[3],ankle_pos[3],leg_middle_pos[3];
//    double  end_diff, knee_diff,ankle_diff;
//    double  object_line_eq,leg_line_eg;
    double a,b,c,d,e,u[3],v[3],Wo[3],Sc,Tc,foot_cp[3],obj_cp[3];

    v[0] = Foot_pos[1][0] - Foot_pos[0][0];
    v[1] = Foot_pos[1][1] - Foot_pos[0][1];
    v[2] = Foot_pos[1][2] - Foot_pos[0][2];

//    v_m = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);

//    v[0] = v[0]/v_m;
//    v[1] = v[1]/v_m;
//    v[2] = v[2]/v_m;


    u[0] = obj_pos[1][0] - obj_pos[0][0];
    u[1] = obj_pos[1][1] - obj_pos[0][1];
    u[2] = obj_pos[1][2] - obj_pos[0][2];

//    u_m = sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);

//    u[0] = u[0]/u_m;
//    u[1] = u[1]/u_m;
//    u[2] = u[2]/u_m;

    Wo[0] = obj_pos[0][0] - Foot_pos[0][0];
    Wo[1] = obj_pos[0][1] - Foot_pos[0][1];
    Wo[2] = obj_pos[0][2] - Foot_pos[0][2];

//    Wo_m = sqrt(Wo[0]*Wo[0] + Wo[1]*Wo[1] + Wo[2]*Wo[2]);

//    Wo[0] = Wo[0]/Wo_m;
//    Wo[1] = Wo[1]/Wo_m;
//    Wo[2] = Wo[2]/Wo_m;


    a = u[0]*u[0] + u[1]*u[1] + u[2]*u[2];
    b = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
    c = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    d = u[0]*Wo[0] + u[1]*Wo[1] + u[2]*Wo[2];
    e = v[0]*Wo[0] + v[1]*Wo[1] + v[2]*Wo[2];

    if(fabs(a*c - b*b) > 0.001)
    {

        Sc = (b*e - c*d)/(a*c - b*b);
        Tc = (a*e - b*d)/(a*c - b*b);
//        printf("===================Sc and Tc===================\n");
//        printf("a:%f   b:%f   c:%f   d:%f   e:%f \n",a,b,c,d,e);
//        printf("%f   %f \n",Sc,Tc);
//        printf("===================Sc and Tc===================\n");

    }else
    {
        Sc = 0.0f;
        Tc = 0.0f;

//        printf("obejct and leg is parallel \n");
    }


//    end_diff = sqrt((ankle_pos[0][0]-object[0][])*(ankle_pos[0][]-object[0][]) + (ankle_pos[1][]-object[1][])*(ankle_pos[1]-object[1]) + (ankle_pos[2]-object[2])*(ankle_pos[2]-object[2]));
//    knee_diff = sqrt((knee_pos[0][0]-object[0])*(knee_pos[0][]-object[0][]) + (knee_pos[1][]-object[1][])*(knee_pos[1]-object[1]) + (knee_pos[2]-object[2])*(knee_pos[2]-object[2]));
//    ankle_diff = sqrt((leg_middle_pos[0][]-object[0][])*(leg_middle_pos[0][]-object[0][]) + (leg_middle_pos[1][]-object[1])*(leg_middle_pos[1]-object[1]) + (leg_middle_pos[2]-object[2])*(leg_middle_pos[2]-object[2]));

    if(Sc < 0)
    {
        Sc = 0.0f;


    }else if(Sc > 1)
    {
        Sc = 1.0f;
    }

    if(Tc < 0)
    {
        Tc = 0.0f;
    }else if(Tc > 1)
    {
        Tc = 1.0f;
    }

    obj_cp[0] = obj_pos[0][0] + Sc*u[0];
    obj_cp[1] = obj_pos[0][1] + Sc*u[1];
    obj_cp[2] = obj_pos[0][2] + Sc*u[2];

    foot_cp[0] = Foot_pos[0][0] + Tc*v[0];
    foot_cp[1] = Foot_pos[0][1] + Tc*v[1];
    foot_cp[2] = Foot_pos[0][2] + Tc*v[2];

    dis[0] = sqrt((obj_cp[0] - foot_cp[0])*(obj_cp[0] - foot_cp[0]) + (obj_cp[1] - foot_cp[1])*(obj_cp[1] - foot_cp[1]) + (obj_cp[2] - foot_cp[2])*(obj_cp[2] - foot_cp[2]));

//    printf("===================distance===================\n");
//    printf("obj:%f   %f   %f \n",obj_cp[0],obj_cp[1],obj_cp[2]);
//    printf("fcp:%f   %f   %f \n",foot_cp[0] ,foot_cp[1],foot_cp[2]);
//    printf("distance : %f \n",dis[0]);
//    printf("===================distance===================\n");
}


void RL_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3])
{
        double axis0[3],axis1[3],axis2[3],axis3[3],axis4[3],axis5[3],axis6[3],axis7[3],axiswheel[3];
        double R_RHY[3][3],R_RHR[3][3],R_RHP[3][3],R_RKN[3][3],R_RAP[3][3],R_RAR[3][3],R_FOOT[3][3],R_WHEEL[3][3],P_RHY[3],P_RHR[3],P_RHP[3],P_RKN[3],P_RAP[3],P_RAR[3],P_FOOT[3],P_WHEEL[3];
        double R_PEL[3][3],P_PEL[3],temp_RHY[3][3],temp_RHR[3][3],temp_RHP[3][3],temp_RKN[3][3],temp_RAP[3][3],temp_RAR[3][3],temp_FOOT[3][3],temp_WHEEL[3][3];
        double offset0[3],offset1[3],offset2[3],offset3[3],offset4[3],offset5[3],offset6[3],offset7[3],offsetwheel[3];

//        double p[6][3],aRAR[3],sigh1[2][3],sigh2[2][3],sigh3[2][3],sigh4[2][3];
//        double R_LHY[3][3],R_LHR[3][3],R_LHP[3][3],R_LKN[3][3],R_LAP[3][3],R_LAR[3][3],P_LHY[3],P_LHR[3],P_LHP[3],P_LKN[3],P_LAP[3],P_LAR[3];
    //    ----------------initial joint rotation axis

//        aPEL[0] = 0.0f;
//        aPEL[1] = 0.0f;
//        aPEL[2] = 0.1f;

//        Rodrigues(aPEL,0.0f,R_PEL);

//        printf("leg angle %f  %f  %f  %f  %f  %f\n",angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);

        axis0[0] = 0.0f;
        axis0[1] = 0.0f;
        axis0[2] = 1.0f;

        axis1[0] = 0.0f;
        axis1[1] = 0.0f;
        axis1[2] = 1.0f;

        axis2[0] = 1.0f;
        axis2[1] = 0.0f;
        axis2[2] = 0.0f;

        axis3[0] = 0.0f;
        axis3[1] = 1.0f;
        axis3[2] = 0.0f;

        axis4[0] = 0.0f;
        axis4[1] = 1.0f;
        axis4[2] = 0.0f;

        axis5[0] = 0.0f;
        axis5[1] = 1.0f;
        axis5[2] = 0.0f;

        axis6[0] = 1.0f;
        axis6[1] = 0.0f;
        axis6[2] = 0.0f;

        axis7[0] = 0.0f;
        axis7[1] = 0.0f;
        axis7[2] = 1.0f;

        axiswheel[0] = 1.0f;
        axiswheel[1] = 0.0f;
        axiswheel[2] = 0.0f;

        //----------------offset for all joint
//        offset0[0] = P_PEL[0] = WBIK_Q2[0];
//        offset0[1] = P_PEL[1] = WBIK_Q2[1];
//        offset0[2] = P_PEL[2] = WBIK_Q2[2];
        offset0[0] = PELVISpos[0];
        offset0[1] = PELVISpos[1];
        offset0[2] = PELVISpos[2];


        P_PEL[0] = PELVISpos[0];
        P_PEL[1] = PELVISpos[1];
        P_PEL[2] = PELVISpos[2];

        PELVIS_pos[0] = offset0[0];
        PELVIS_pos[1] = offset0[1];
        PELVIS_pos[2] = offset0[2];

//        printf("====================PELVIS POS===================\n");
//        printf("%f   %f   %f\n",P_PEL[0],P_PEL[1],P_PEL[2]);
//        printf("=================================================\n");


        offset1[0] =  0.0f;
        offset1[1] = - 0.1f;
        offset1[2] = 0.0f;

        offset2[0] = 0.0f;
        offset2[1] = 0.0f;
        offset2[2] = 0.0f;

        offset3[0] = 0.0f;
        offset3[1] = 0.0f;
        offset3[2] = 0.0f;

        offset4[0] = 0.0;
        offset4[1] = 0.0f;
        offset4[2] = -kine_drc_hubo4.L_UPPER_LEG; //-0.4f;


        offsetwheel[0] = 0.04858f;
        offsetwheel[1] = 0.0f;
        offsetwheel[2] = -0.12458f;

        offset5[0] = -offsetwheel[0];
        offset5[1] = 0.0f;
        offset5[2] = -kine_drc_hubo4.L_LOWER_LEG - offsetwheel[2];//-0.38f;

        offset6[0] = 0.0f;
        offset6[1] = 0.0f;
        offset6[2] = -kine_drc_hubo4.L_ANKLE;//-0.03f;

        offset7[0] = 0.0f;
        offset7[1] = 0.0f;
        offset7[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;



//        printf("LU: %f  %f  %f  %f \n",kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_LOWER_LEG,kine_drc_hubo4.L_ANKLE,kine_drc_hubo4.L_FOOT);

        Rodrigues(axis0,0.0f,R_PEL);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[0][0],R_PEL[0][1],R_PEL[0][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[1][0],R_PEL[1][1],R_PEL[1][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[2][0],R_PEL[2][1],R_PEL[2][2]);

        //----------------RHY pos  &  RHY R mat
        P_RHY[0] = P_PEL[0] + (R_PEL[0][0]*offset1[0] + R_PEL[0][1]*offset1[1] + R_PEL[0][2]*offset1[2]);
        P_RHY[1] = P_PEL[1] + (R_PEL[1][0]*offset1[0] + R_PEL[1][1]*offset1[1] + R_PEL[1][2]*offset1[2]);
        P_RHY[2] = P_PEL[2] + (R_PEL[2][0]*offset1[0] + R_PEL[2][1]*offset1[1] + R_PEL[2][2]*offset1[2]);

        RHY_pos[0] = P_RHY[0];
        RHY_pos[1] = P_RHY[1];
        RHY_pos[2] = P_RHY[2];



        Rodrigues(axis1,angle[0],temp_RHY);

        mat3by3x3by3(R_PEL,temp_RHY,R_RHY);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[0][0],R_RHY[0][1],R_RHY[0][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[1][0],R_RHY[1][1],R_RHY[1][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[2][0],R_RHY[2][1],R_RHY[2][2]);

        //----------------RHR pos  &  RHR R mat
        P_RHR[0] = P_RHY[0] + (R_RHY[0][0]*offset2[0] + R_RHY[0][1]*offset2[1] + R_RHY[0][2]*offset2[2]);
        P_RHR[1] = P_RHY[1] + (R_RHY[1][0]*offset2[0] + R_RHY[1][1]*offset2[1] + R_RHY[1][2]*offset2[2]);
        P_RHR[2] = P_RHY[2] + (R_RHY[2][0]*offset2[0] + R_RHY[2][1]*offset2[1] + R_RHY[2][2]*offset2[2]);

        Rodrigues(axis2,angle[1],temp_RHR);

        mat3by3x3by3(R_RHY,temp_RHR,R_RHR);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[0][0],R_RHR[0][1],R_RHR[0][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[1][0],R_RHR[1][1],R_RHR[1][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[2][0],R_RHR[2][1],R_RHR[2][2]);

        //----------------RHP pos  &  RHP R mat
        P_RHP[0] = P_RHR[0] + (R_RHR[0][0]*offset3[0] + R_RHR[0][1]*offset3[1] + R_RHR[0][2]*offset3[2]);
        P_RHP[1] = P_RHR[1] + (R_RHR[1][0]*offset3[0] + R_RHR[1][1]*offset3[1] + R_RHR[1][2]*offset3[2]);
        P_RHP[2] = P_RHR[2] + (R_RHR[2][0]*offset3[0] + R_RHR[2][1]*offset3[1] + R_RHR[2][2]*offset3[2]);


        Rodrigues(axis3,angle[2],temp_RHP);

        mat3by3x3by3(R_RHR,temp_RHP,R_RHP);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[0][0],R_RHP[0][1],R_RHP[0][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[1][0],R_RHP[1][1],R_RHP[1][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[2][0],R_RHP[2][1],R_RHP[2][2]);


        //----------------RKN pos  &  RKN R mat
        P_RKN[0] = P_RHP[0] + (R_RHP[0][0]*offset4[0] + R_RHP[0][1]*offset4[1] + R_RHP[0][2]*offset4[2]);
        P_RKN[1] = P_RHP[1] + (R_RHP[1][0]*offset4[0] + R_RHP[1][1]*offset4[1] + R_RHP[1][2]*offset4[2]);
        P_RKN[2] = P_RHP[2] + (R_RHP[2][0]*offset4[0] + R_RHP[2][1]*offset4[1] + R_RHP[2][2]*offset4[2]);

   //     printf("Pelvis pos  %f \n",PELVISpos[2]);
//        printf("P_RKN: %f \n",P_RKN[2]);


        Rodrigues(axis4,angle[3],temp_RKN);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[0][0],temp_RKN[0][1],temp_RKN[0][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[1][0],temp_RKN[1][1],temp_RKN[1][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[2][0],temp_RKN[2][1],temp_RKN[2][2]);

        mat3by3x3by3(R_RHP,temp_RKN,R_RKN);

//        printf("R_RKN: %f   %f   %f\n",R_RKN[0][0],R_RKN[0][1],R_RKN[0][2]);

//        printf("R_RKN: %f   %f   %f\n",R_RKN[1][0],R_RKN[1][1],R_RKN[1][2]);

//        printf("R_RKN: %f   %f   %f\n",R_RKN[2][0],R_RKN[2][1],R_RKN[2][2]);



        //----------------WHEEL pos  &  WHEEL R mat

        P_WHEEL[0] = P_RKN[0] + (R_RKN[0][0]*offsetwheel[0] + R_RKN[0][1]*offsetwheel[1] + R_RKN[0][2]*offsetwheel[2]);
        P_WHEEL[1] = P_RKN[1] + (R_RKN[1][0]*offsetwheel[0] + R_RKN[1][1]*offsetwheel[1] + R_RKN[1][2]*offsetwheel[2]);
        P_WHEEL[2] = P_RKN[2] + (R_RKN[2][0]*offsetwheel[0] + R_RKN[2][1]*offsetwheel[1] + R_RKN[2][2]*offsetwheel[2]);

        Rodrigues(axiswheel,0.0f,temp_WHEEL);

        mat3by3x3by3(R_RKN,temp_WHEEL,R_WHEEL);




        //----------------RAP pos  &  RAP R mat
//        P_RAP[0] = P_RKN[0] + (R_RKN[0][0]*offset5[0] + R_RKN[0][1]*offset5[1] + R_RKN[0][2]*offset5[2]);
//        P_RAP[1] = P_RKN[1] + (R_RKN[1][0]*offset5[0] + R_RKN[1][1]*offset5[1] + R_RKN[1][2]*offset5[2]);
//        P_RAP[2] = P_RKN[2] + (R_RKN[2][0]*offset5[0] + R_RKN[2][1]*offset5[1] + R_RKN[2][2]*offset5[2]);

//        Rodrigues(axis5,angle[4],temp_RAP);

//        mat3by3x3by3(R_RKN,temp_RAP,R_RAP);

        P_RAP[0] = P_WHEEL[0] + (R_WHEEL[0][0]*offset5[0] + R_WHEEL[0][1]*offset5[1] + R_WHEEL[0][2]*offset5[2]);
        P_RAP[1] = P_WHEEL[1] + (R_WHEEL[1][0]*offset5[0] + R_WHEEL[1][1]*offset5[1] + R_WHEEL[1][2]*offset5[2]);
        P_RAP[2] = P_WHEEL[2] + (R_WHEEL[2][0]*offset5[0] + R_WHEEL[2][1]*offset5[1] + R_WHEEL[2][2]*offset5[2]);

        Rodrigues(axis5,angle[4],temp_RAP);

        mat3by3x3by3(R_WHEEL,temp_RAP,R_RAP);

//        printf("R_RAP: %f   %f   %f\n",R_RAP[0][0],R_RAP[0][1],R_RAP[0][2]);

//        printf("R_RAP: %f   %f   %f\n",R_RAP[1][0],R_RAP[1][1],R_RAP[1][2]);

//        printf("R_RAP: %f   %f   %f\n",R_RAP[2][0],R_RAP[2][1],R_RAP[2][2]);

        //----------------RAR pos  &  RAR R mat
        P_RAR[0] = P_RAP[0] + (R_RAP[0][0]*offset6[0] + R_RAP[0][1]*offset6[1] + R_RAP[0][2]*offset6[2]);
        P_RAR[1] = P_RAP[1] + (R_RAP[1][0]*offset6[0] + R_RAP[1][1]*offset6[1] + R_RAP[1][2]*offset6[2]);
        P_RAR[2] = P_RAP[2] + (R_RAP[2][0]*offset6[0] + R_RAP[2][1]*offset6[1] + R_RAP[2][2]*offset6[2]);

        Rodrigues(axis6,angle[5],temp_RAR);

        mat3by3x3by3(R_RAP,temp_RAR,R_RAR);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[0][0],R_RAR[0][1],R_RAR[0][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[1][0],R_RAR[1][1],R_RAR[1][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[2][0],R_RAR[2][1],R_RAR[2][2]);
        //----------------Foot Pos & ori
        P_FOOT[0] = P_RAR[0] + (R_RAR[0][0]*offset7[0] + R_RAR[0][1]*offset7[1] + R_RAR[0][2]*offset7[2]);
        P_FOOT[1] = P_RAR[1] + (R_RAR[1][0]*offset7[0] + R_RAR[1][1]*offset7[1] + R_RAR[1][2]*offset7[2]);
        P_FOOT[2] = P_RAR[2] + (R_RAR[2][0]*offset7[0] + R_RAR[2][1]*offset7[1] + R_RAR[2][2]*offset7[2]);

        Rodrigues(axis7,0.0,temp_FOOT);

        mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);

        FOOT_pos[0] = P_FOOT[0];
        FOOT_pos[1] = P_FOOT[1];
        FOOT_pos[2] = P_FOOT[2];

        RHY_pos[0] = P_RHY[0];
        RHY_pos[1] = P_RHY[1];
        RHY_pos[2] = P_RHY[2];

        RHR_pos[0] = P_RHR[0];
        RHR_pos[1] = P_RHR[1];
        RHR_pos[2] = P_RHR[2];

        RHP_pos[0] = P_RHP[0];
        RHP_pos[1] = P_RHP[1];
        RHP_pos[2] = P_RHP[2];

        RKN_pos[0] = P_RKN[0];
        RKN_pos[1] = P_RKN[1];
        RKN_pos[2] = P_RKN[2];

        RAP_pos[0] = P_RAP[0];
        RAP_pos[1] = P_RAP[1];
        RAP_pos[2] = P_RAP[2];

        RAR_pos[0] = P_RAR[0];
        RAR_pos[1] = P_RAR[1];
        RAR_pos[2] = P_RAR[2];


//        printf("================= leg collision check================\n");
//        printf("%f   %f   %f \n",WBIK_Q2[0],WBIK_Q2[1],WBIK_Q2[2]);
//        printf("================= leg collision check================\n");
//        printf("RHY pos: %f    %f    %f \n",P_RHY[0],P_RHY[1],P_RHY[2]);
//        printf("RHR pos: %f    %f    %f \n",P_RHR[0],P_RHR[1],P_RHR[2]);
//        printf("RHP pos: %f    %f    %f \n",P_RHP[0],P_RHP[1],P_RHP[2]);
//        printf("RKN pos: %f    %f    %f \n",P_RKN[0],P_RKN[1],P_RKN[2]);
//        printf("RAP pos: %f    %f    %f \n",P_RAP[0],P_RAP[1],P_RAP[2]);
//        printf("RAR pos: %f    %f    %f \n",P_RAR[0],P_RAR[1],P_RAR[2]);

        pos[3][0] = P_WHEEL[0];
        pos[3][1] = P_WHEEL[1];
        pos[3][2] = P_WHEEL[2];

        pos[4][0] = P_RAP[0];
        pos[4][1] = P_RAP[1];
        pos[4][2] = P_RAP[2];

}


void LL_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3])
{
        double axis0[3],axis1[3],axis2[3],axis3[3],axis4[3],axis5[3],axis6[3],axis7[3],axiswheel[3];
        double R_RHY[3][3],R_RHR[3][3],R_RHP[3][3],R_RKN[3][3],R_RAP[3][3],R_RAR[3][3],R_FOOT[3][3],R_WHEEL[3][3],P_RHY[3],P_RHR[3],P_RHP[3],P_RKN[3],P_RAP[3],P_RAR[3],P_FOOT[3],P_WHEEL[3];
        double R_PEL[3][3],P_PEL[3],temp_RHY[3][3],temp_RHR[3][3],temp_RHP[3][3],temp_RKN[3][3],temp_RAP[3][3],temp_RAR[3][3],temp_FOOT[3][3],temp_WHEEL[3][3];
        double offset0[3],offset1[3],offset2[3],offset3[3],offset4[3],offset5[3],offset6[3],offset7[3],offsetwheel[3];

//        double p[6][3],aRAR[3],sigh1[2][3],sigh2[2][3],sigh3[2][3],sigh4[2][3];
//        double R_LHY[3][3],R_LHR[3][3],R_LHP[3][3],R_LKN[3][3],R_LAP[3][3],R_LAR[3][3],P_LHY[3],P_LHR[3],P_LHP[3],P_LKN[3],P_LAP[3],P_LAR[3];
    //    ----------------initial joint rotation axis

//        aPEL[0] = 0.0f;
//        aPEL[1] = 0.0f;
//        aPEL[2] = 0.1f;

//        Rodrigues(aPEL,0.0f,R_PEL);

//        printf("leg angle %f  %f  %f  %f  %f  %f\n",angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);

        axis0[0] = 0.0f;
        axis0[1] = 0.0f;
        axis0[2] = 1.0f;

        axis1[0] = 0.0f;
        axis1[1] = 0.0f;
        axis1[2] = 1.0f;

        axis2[0] = 1.0f;
        axis2[1] = 0.0f;
        axis2[2] = 0.0f;

        axis3[0] = 0.0f;
        axis3[1] = 1.0f;
        axis3[2] = 0.0f;

        axis4[0] = 0.0f;
        axis4[1] = 1.0f;
        axis4[2] = 0.0f;

        axis5[0] = 0.0f;
        axis5[1] = 1.0f;
        axis5[2] = 0.0f;

        axis6[0] = 1.0f;
        axis6[1] = 0.0f;
        axis6[2] = 0.0f;

        axis7[0] = 0.0f;
        axis7[1] = 0.0f;
        axis7[2] = 1.0f;

        axiswheel[0] = 0.0f;
        axiswheel[1] = 1.0f;
        axiswheel[2] = 0.0f;

        //----------------offset for all joint
//        offset0[0] = P_PEL[0] = WBIK_Q2[0];
//        offset0[1] = P_PEL[1] = WBIK_Q2[1];
//        offset0[2] = P_PEL[2] = WBIK_Q2[2];
        offset0[0] = PELVISpos[0];
        offset0[1] = PELVISpos[1];
        offset0[2] = PELVISpos[2];


        P_PEL[0] = PELVISpos[0];
        P_PEL[1] = PELVISpos[1];
        P_PEL[2] = PELVISpos[2];

        PELVIS_pos[0] = offset0[0];
        PELVIS_pos[1] = offset0[1];
        PELVIS_pos[2] = offset0[2];

//        printf("====================PELVIS POS===================\n");
//        printf("%f   %f   %f\n",P_PEL[0],P_PEL[1],P_PEL[2]);
//        printf("=================================================\n");

        offset1[0] =  0.0f;
        offset1[1] = 0.1f;
        offset1[2] = 0.0f;

        offset2[0] = 0.0f;
        offset2[1] = 0.0f;
        offset2[2] = 0.0f;

        offset3[0] = 0.0f;
        offset3[1] = 0.0f;
        offset3[2] = 0.0f;

        offset4[0] = 0.0;
        offset4[1] = 0.0f;
        offset4[2] = -kine_drc_hubo4.L_UPPER_LEG; //-0.4f;


        offsetwheel[0] = 0.04858f;
        offsetwheel[1] = 0.0f;
        offsetwheel[2] = -0.12458f;

        offset5[0] = -offsetwheel[0];
        offset5[1] = 0.0f;
        offset5[2] = -kine_drc_hubo4.L_LOWER_LEG - offsetwheel[2];//-0.38f;

        offset6[0] = 0.0f;
        offset6[1] = 0.0f;
        offset6[2] = -kine_drc_hubo4.L_ANKLE;//-0.03f;

        offset7[0] = 0.0f;
        offset7[1] = 0.0f;
        offset7[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;



//        offset1[0] =  0.0f;
//        offset1[1] =  0.1f;
//        offset1[2] = 0.0f;

//        offset2[0] = 0.0f;
//        offset2[1] = 0.0f;
//        offset2[2] = 0.0f;

//        offset3[0] = 0.0f;
//        offset3[1] = 0.0f;
//        offset3[2] = 0.0f;

//        offset4[0] = 0.0;
//        offset4[1] = 0.0f;
//        offset4[2] = -kine_drc_hubo4.L_UPPER_LEG; //-0.4f;


////        offsetwheel[0] = 0.0f;
////        offsetwheel[1] = 0.0f;
////        offsetwheel[2] = 0.0;//-0.1f;

//        offset5[0] = 0.0f;
//        offset5[1] = 0.0f;
//        offset5[2] = -kine_drc_hubo4.L_LOWER_LEG;//-0.38f;

//        offset6[0] = 0.0f;
//        offset6[1] = 0.0f;
//        offset6[2] = -kine_drc_hubo4.L_ANKLE;//-0.03f;

//        offset7[0] = 0.0f;
//        offset7[1] = 0.0f;
//        offset7[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

//        printf("LU: %f  %f  %f  %f \n",kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_LOWER_LEG,kine_drc_hubo4.L_ANKLE,kine_drc_hubo4.L_FOOT);

        Rodrigues(axis0,0.0f,R_PEL);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[0][0],R_PEL[0][1],R_PEL[0][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[1][0],R_PEL[1][1],R_PEL[1][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[2][0],R_PEL[2][1],R_PEL[2][2]);

        //----------------RHY pos  &  RHY R mat
        P_RHY[0] = P_PEL[0] + (R_PEL[0][0]*offset1[0] + R_PEL[0][1]*offset1[1] + R_PEL[0][2]*offset1[2]);
        P_RHY[1] = P_PEL[1] + (R_PEL[1][0]*offset1[0] + R_PEL[1][1]*offset1[1] + R_PEL[1][2]*offset1[2]);
        P_RHY[2] = P_PEL[2] + (R_PEL[2][0]*offset1[0] + R_PEL[2][1]*offset1[1] + R_PEL[2][2]*offset1[2]);

        Rodrigues(axis1,angle[0],temp_RHY);

        mat3by3x3by3(R_PEL,temp_RHY,R_RHY);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[0][0],R_RHY[0][1],R_RHY[0][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[1][0],R_RHY[1][1],R_RHY[1][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[2][0],R_RHY[2][1],R_RHY[2][2]);

        //----------------RHR pos  &  RHR R mat
        P_RHR[0] = P_RHY[0] + (R_RHY[0][0]*offset2[0] + R_RHY[0][1]*offset2[1] + R_RHY[0][2]*offset2[2]);
        P_RHR[1] = P_RHY[1] + (R_RHY[1][0]*offset2[0] + R_RHY[1][1]*offset2[1] + R_RHY[1][2]*offset2[2]);
        P_RHR[2] = P_RHY[2] + (R_RHY[2][0]*offset2[0] + R_RHY[2][1]*offset2[1] + R_RHY[2][2]*offset2[2]);

        Rodrigues(axis2,angle[1],temp_RHR);

        mat3by3x3by3(R_RHY,temp_RHR,R_RHR);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[0][0],R_RHR[0][1],R_RHR[0][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[1][0],R_RHR[1][1],R_RHR[1][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[2][0],R_RHR[2][1],R_RHR[2][2]);

        //----------------RHP pos  &  RHP R mat
        P_RHP[0] = P_RHR[0] + (R_RHR[0][0]*offset3[0] + R_RHR[0][1]*offset3[1] + R_RHR[0][2]*offset3[2]);
        P_RHP[1] = P_RHR[1] + (R_RHR[1][0]*offset3[0] + R_RHR[1][1]*offset3[1] + R_RHR[1][2]*offset3[2]);
        P_RHP[2] = P_RHR[2] + (R_RHR[2][0]*offset3[0] + R_RHR[2][1]*offset3[1] + R_RHR[2][2]*offset3[2]);


        Rodrigues(axis3,angle[2],temp_RHP);

        mat3by3x3by3(R_RHR,temp_RHP,R_RHP);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[0][0],R_RHP[0][1],R_RHP[0][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[1][0],R_RHP[1][1],R_RHP[1][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[2][0],R_RHP[2][1],R_RHP[2][2]);



        //----------------RKN pos  &  RKN R mat
        P_RKN[0] = P_RHP[0] + (R_RHP[0][0]*offset4[0] + R_RHP[0][1]*offset4[1] + R_RHP[0][2]*offset4[2]);
        P_RKN[1] = P_RHP[1] + (R_RHP[1][0]*offset4[0] + R_RHP[1][1]*offset4[1] + R_RHP[1][2]*offset4[2]);
        P_RKN[2] = P_RHP[2] + (R_RHP[2][0]*offset4[0] + R_RHP[2][1]*offset4[1] + R_RHP[2][2]*offset4[2]);

   //     printf("Pelvis pos  %f \n",PELVISpos[2]);
//        printf("P_RKN: %f \n",P_RKN[2]);


        Rodrigues(axis4,angle[3],temp_RKN);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[0][0],temp_RKN[0][1],temp_RKN[0][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[1][0],temp_RKN[1][1],temp_RKN[1][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[2][0],temp_RKN[2][1],temp_RKN[2][2]);

        mat3by3x3by3(R_RHP,temp_RKN,R_RKN);





        //----------------WHEEL pos  &  WHEEL R mat

        P_WHEEL[0] = P_RKN[0] + (R_RKN[0][0]*offsetwheel[0] + R_RKN[0][1]*offsetwheel[1] + R_RKN[0][2]*offsetwheel[2]);
        P_WHEEL[1] = P_RKN[1] + (R_RKN[1][0]*offsetwheel[0] + R_RKN[1][1]*offsetwheel[1] + R_RKN[1][2]*offsetwheel[2]);
        P_WHEEL[2] = P_RKN[2] + (R_RKN[2][0]*offsetwheel[0] + R_RKN[2][1]*offsetwheel[1] + R_RKN[2][2]*offsetwheel[2]);

        Rodrigues(axiswheel,0.0f,temp_WHEEL);

        mat3by3x3by3(R_RKN,temp_WHEEL,R_WHEEL);




        //----------------RAP pos  &  RAP R mat
//        P_RAP[0] = P_RKN[0] + (R_RKN[0][0]*offset5[0] + R_RKN[0][1]*offset5[1] + R_RKN[0][2]*offset5[2]);
//        P_RAP[1] = P_RKN[1] + (R_RKN[1][0]*offset5[0] + R_RKN[1][1]*offset5[1] + R_RKN[1][2]*offset5[2]);
//        P_RAP[2] = P_RKN[2] + (R_RKN[2][0]*offset5[0] + R_RKN[2][1]*offset5[1] + R_RKN[2][2]*offset5[2]);

//        Rodrigues(axis5,angle[4],temp_RAP);

//        mat3by3x3by3(R_RKN,temp_RAP,R_RAP);

        P_RAP[0] = P_WHEEL[0] + (R_WHEEL[0][0]*offset5[0] + R_WHEEL[0][1]*offset5[1] + R_WHEEL[0][2]*offset5[2]);
        P_RAP[1] = P_WHEEL[1] + (R_WHEEL[1][0]*offset5[0] + R_WHEEL[1][1]*offset5[1] + R_WHEEL[1][2]*offset5[2]);
        P_RAP[2] = P_WHEEL[2] + (R_WHEEL[2][0]*offset5[0] + R_WHEEL[2][1]*offset5[1] + R_WHEEL[2][2]*offset5[2]);

        Rodrigues(axis5,angle[4],temp_RAP);

        mat3by3x3by3(R_WHEEL,temp_RAP,R_RAP);

//        //----------------RKN pos  &  RKN R mat
//        P_RKN[0] = P_RHP[0] + (R_RHP[0][0]*offset4[0] + R_RHP[0][1]*offset4[1] + R_RHP[0][2]*offset4[2]);
//        P_RKN[1] = P_RHP[1] + (R_RHP[1][0]*offset4[0] + R_RHP[1][1]*offset4[1] + R_RHP[1][2]*offset4[2]);
//        P_RKN[2] = P_RHP[2] + (R_RHP[2][0]*offset4[0] + R_RHP[2][1]*offset4[1] + R_RHP[2][2]*offset4[2]);

//   //     printf("Pelvis pos  %f \n",PELVISpos[2]);
////        printf("P_RKN: %f \n",P_RKN[2]);


//        Rodrigues(axis4,angle[3],temp_RKN);

////        printf("temp_RKN: %f   %f   %f\n",temp_RKN[0][0],temp_RKN[0][1],temp_RKN[0][2]);

////        printf("temp_RKN: %f   %f   %f\n",temp_RKN[1][0],temp_RKN[1][1],temp_RKN[1][2]);

////        printf("temp_RKN: %f   %f   %f\n",temp_RKN[2][0],temp_RKN[2][1],temp_RKN[2][2]);

//        mat3by3x3by3(R_RHP,temp_RKN,R_RKN);

////        printf("R_RKN: %f   %f   %f\n",R_RKN[0][0],R_RKN[0][1],R_RKN[0][2]);

////        printf("R_RKN: %f   %f   %f\n",R_RKN[1][0],R_RKN[1][1],R_RKN[1][2]);

////        printf("R_RKN: %f   %f   %f\n",R_RKN[2][0],R_RKN[2][1],R_RKN[2][2]);

//        //----------------RAP pos  &  RAP R mat
//        P_RAP[0] = P_RKN[0] + (R_RKN[0][0]*offset5[0] + R_RKN[0][1]*offset5[1] + R_RKN[0][2]*offset5[2]);
//        P_RAP[1] = P_RKN[1] + (R_RKN[1][0]*offset5[0] + R_RKN[1][1]*offset5[1] + R_RKN[1][2]*offset5[2]);
//        P_RAP[2] = P_RKN[2] + (R_RKN[2][0]*offset5[0] + R_RKN[2][1]*offset5[1] + R_RKN[2][2]*offset5[2]);

//        Rodrigues(axis5,angle[4],temp_RAP);

//        mat3by3x3by3(R_RKN,temp_RAP,R_RAP);

////        printf("R_RAP: %f   %f   %f\n",R_RAP[0][0],R_RAP[0][1],R_RAP[0][2]);

////        printf("R_RAP: %f   %f   %f\n",R_RAP[1][0],R_RAP[1][1],R_RAP[1][2]);

////        printf("R_RAP: %f   %f   %f\n",R_RAP[2][0],R_RAP[2][1],R_RAP[2][2]);

        //----------------RAR pos  &  RAR R mat
        P_RAR[0] = P_RAP[0] + (R_RAP[0][0]*offset6[0] + R_RAP[0][1]*offset6[1] + R_RAP[0][2]*offset6[2]);
        P_RAR[1] = P_RAP[1] + (R_RAP[1][0]*offset6[0] + R_RAP[1][1]*offset6[1] + R_RAP[1][2]*offset6[2]);
        P_RAR[2] = P_RAP[2] + (R_RAP[2][0]*offset6[0] + R_RAP[2][1]*offset6[1] + R_RAP[2][2]*offset6[2]);

        Rodrigues(axis6,angle[5],temp_RAR);

        mat3by3x3by3(R_RAP,temp_RAR,R_RAR);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[0][0],R_RAR[0][1],R_RAR[0][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[1][0],R_RAR[1][1],R_RAR[1][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[2][0],R_RAR[2][1],R_RAR[2][2]);


        //----------------Foot Pos & ori
        P_FOOT[0] = P_RAR[0] + (R_RAR[0][0]*offset7[0] + R_RAR[0][1]*offset7[1] + R_RAR[0][2]*offset7[2]);
        P_FOOT[1] = P_RAR[1] + (R_RAR[1][0]*offset7[0] + R_RAR[1][1]*offset7[1] + R_RAR[1][2]*offset7[2]);
        P_FOOT[2] = P_RAR[2] + (R_RAR[2][0]*offset7[0] + R_RAR[2][1]*offset7[1] + R_RAR[2][2]*offset7[2]);

        Rodrigues(axis7,0.0,temp_FOOT);

        mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);

        FOOT_pos2[0] = P_FOOT[0];
        FOOT_pos2[1] = P_FOOT[1];
        FOOT_pos2[2] = P_FOOT[2];

        RHY_pos2[0] = P_RHY[0];
        RHY_pos2[1] = P_RHY[1];
        RHY_pos2[2] = P_RHY[2];

        RHR_pos2[0] = P_RHR[0];
        RHR_pos2[1] = P_RHR[1];
        RHR_pos2[2] = P_RHR[2];

        RHP_pos2[0] = P_RHP[0];
        RHP_pos2[1] = P_RHP[1];
        RHP_pos2[2] = P_RHP[2];

        RKN_pos2[0] = P_RKN[0];
        RKN_pos2[1] = P_RKN[1];
        RKN_pos2[2] = P_RKN[2];

        RAP_pos2[0] = P_RAP[0];
        RAP_pos2[1] = P_RAP[1];
        RAP_pos2[2] = P_RAP[2];

        RAR_pos2[0] = P_RAR[0];
        RAR_pos2[1] = P_RAR[1];
        RAR_pos2[2] = P_RAR[2];


//        printf("================= leg collision check================\n");
//        printf("%f   %f   %f \n",WBIK_Q2[0],WBIK_Q2[1],WBIK_Q2[2]);
//        printf("================= leg collision check================\n");
//        printf("RHY pos: %f    %f    %f \n",P_RHY[0],P_RHY[1],P_RHY[2]);
//        printf("RHR pos: %f    %f    %f \n",P_RHR[0],P_RHR[1],P_RHR[2]);
//        printf("RHP pos: %f    %f    %f \n",P_RHP[0],P_RHP[1],P_RHP[2]);
//        printf("RKN pos: %f    %f    %f \n",P_RKN[0],P_RKN[1],P_RKN[2]);
//        printf("RAP pos: %f    %f    %f \n",P_RAP[0],P_RAP[1],P_RAP[2]);
//        printf("RAR pos: %f    %f    %f \n",P_RAR[0],P_RAR[1],P_RAR[2]);

        pos[3][0] = P_WHEEL[0];
        pos[3][1] = P_WHEEL[1];
        pos[3][2] = P_WHEEL[2];

        pos[4][0] = P_RAP[0];
        pos[4][1] = P_RAP[1];
        pos[4][2] = P_RAP[2];

}


void RL_Foot_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3])
{
    double axis0[3],axis1[3],axis2[3],axis3[3],axis4[3],axis5[3],axis6[3],axis7[3],axiswheel[3];
    double R_RHY[3][3],R_RHR[3][3],R_RHP[3][3],R_RKN[3][3],R_RAP[3][3],R_RAR[3][3],R_FOOT[3][3],R_WHEEL[3][3],P_RHY[3],P_RHR[3],P_RHP[3],P_RKN[3],P_RAP[3],P_RAR[3],P_FOOT[3],P_EDGE[4][3],P_WHEEL[3];
    double R_PEL[3][3],P_PEL[3],temp_RHY[3][3],temp_RHR[3][3],temp_RHP[3][3],temp_RKN[3][3],temp_RAP[3][3],temp_RAR[3][3],temp_FOOT[3][3],temp_WHEEL[3][3];
    double offset0[3],offset1[3],offset2[3],offset3[3],offset4[3],offset5[3],offset6[3],offset7[3],offset8[3],offset9[3],offset10[3],offset11[3],offsetwheel[3];

    axis0[0] = 0.0f;
    axis0[1] = 0.0f;
    axis0[2] = 1.0f;

    axis1[0] = 0.0f;
    axis1[1] = 0.0f;
    axis1[2] = 1.0f;

    axis2[0] = 1.0f;
    axis2[1] = 0.0f;
    axis2[2] = 0.0f;

    axis3[0] = 0.0f;
    axis3[1] = 1.0f;
    axis3[2] = 0.0f;

    axis4[0] = 0.0f;
    axis4[1] = 1.0f;
    axis4[2] = 0.0f;

    axis5[0] = 0.0f;
    axis5[1] = 1.0f;
    axis5[2] = 0.0f;

    axis6[0] = 1.0f;
    axis6[1] = 0.0f;
    axis6[2] = 0.0f;

    axis7[0] = 0.0f;
    axis7[1] = 0.0f;
    axis7[2] = 1.0f;

    axiswheel[0] = 1.0f;
    axiswheel[1] = 0.0f;
    axiswheel[2] = 0.0f;

    offset0[0] = PELVISpos[0];
    offset0[1] = PELVISpos[1];
    offset0[2] = PELVISpos[2];


    P_PEL[0] = PELVISpos[0];
    P_PEL[1] = PELVISpos[1];
    P_PEL[2] = PELVISpos[2];

    PELVIS_pos[0] = offset0[0];
    PELVIS_pos[1] = offset0[1];
    PELVIS_pos[2] = offset0[2];

//        printf("====================PELVIS POS===================\n");
//        printf("%f   %f   %f\n",P_PEL[0],P_PEL[1],P_PEL[2]);
//        printf("=================================================\n");


    offset1[0] =  0.0f;
    offset1[1] = - 0.1f;
    offset1[2] = 0.0f;

    offset2[0] = 0.0f;
    offset2[1] = 0.0f;
    offset2[2] = 0.0f;

    offset3[0] = 0.0f;
    offset3[1] = 0.0f;
    offset3[2] = 0.0f;

    offset4[0] = 0.0;
    offset4[1] = 0.0f;
    offset4[2] = -kine_drc_hubo4.L_UPPER_LEG; //-0.4f;


    offsetwheel[0] = 0.04858f;
    offsetwheel[1] = 0.0f;
    offsetwheel[2] = -0.12458f;

    offset5[0] = -offsetwheel[0];
    offset5[1] = 0.0f;
    offset5[2] = -kine_drc_hubo4.L_LOWER_LEG - offsetwheel[2];//-0.38f;

    offset6[0] = 0.0f;
    offset6[1] = 0.0f;
    offset6[2] = -kine_drc_hubo4.L_ANKLE;//-0.03f;

    offset7[0] = 0.0f;
    offset7[1] = 0.0f;
    offset7[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    //----------------for foot edge configuration
    offset8[0] = 0.12f;
    offset8[1] = -0.07f;
    offset8[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    offset9[0] = 0.12f;
    offset9[1] = 0.09f;
    offset9[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    offset10[0] = -0.12f;
    offset10[1] = -0.07f;
    offset10[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    offset11[0] = -0.12f;
    offset11[1] = 0.09f;
    offset11[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;


//        printf("LU: %f  %f  %f  %f \n",kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_LOWER_LEG,kine_drc_hubo4.L_ANKLE,kine_drc_hubo4.L_FOOT);

    Rodrigues(axis0,0.0f,R_PEL);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[0][0],R_PEL[0][1],R_PEL[0][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[1][0],R_PEL[1][1],R_PEL[1][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[2][0],R_PEL[2][1],R_PEL[2][2]);

    //----------------RHY pos  &  RHY R mat
    P_RHY[0] = P_PEL[0] + (R_PEL[0][0]*offset1[0] + R_PEL[0][1]*offset1[1] + R_PEL[0][2]*offset1[2]);
    P_RHY[1] = P_PEL[1] + (R_PEL[1][0]*offset1[0] + R_PEL[1][1]*offset1[1] + R_PEL[1][2]*offset1[2]);
    P_RHY[2] = P_PEL[2] + (R_PEL[2][0]*offset1[0] + R_PEL[2][1]*offset1[1] + R_PEL[2][2]*offset1[2]);

    RHY_pos[0] = P_RHY[0];
    RHY_pos[1] = P_RHY[1];
    RHY_pos[2] = P_RHY[2];



    Rodrigues(axis1,angle[0],temp_RHY);

    mat3by3x3by3(R_PEL,temp_RHY,R_RHY);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[0][0],R_RHY[0][1],R_RHY[0][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[1][0],R_RHY[1][1],R_RHY[1][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[2][0],R_RHY[2][1],R_RHY[2][2]);

    //----------------RHR pos  &  RHR R mat
    P_RHR[0] = P_RHY[0] + (R_RHY[0][0]*offset2[0] + R_RHY[0][1]*offset2[1] + R_RHY[0][2]*offset2[2]);
    P_RHR[1] = P_RHY[1] + (R_RHY[1][0]*offset2[0] + R_RHY[1][1]*offset2[1] + R_RHY[1][2]*offset2[2]);
    P_RHR[2] = P_RHY[2] + (R_RHY[2][0]*offset2[0] + R_RHY[2][1]*offset2[1] + R_RHY[2][2]*offset2[2]);

    Rodrigues(axis2,angle[1],temp_RHR);

    mat3by3x3by3(R_RHY,temp_RHR,R_RHR);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[0][0],R_RHR[0][1],R_RHR[0][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[1][0],R_RHR[1][1],R_RHR[1][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[2][0],R_RHR[2][1],R_RHR[2][2]);

    //----------------RHP pos  &  RHP R mat
    P_RHP[0] = P_RHR[0] + (R_RHR[0][0]*offset3[0] + R_RHR[0][1]*offset3[1] + R_RHR[0][2]*offset3[2]);
    P_RHP[1] = P_RHR[1] + (R_RHR[1][0]*offset3[0] + R_RHR[1][1]*offset3[1] + R_RHR[1][2]*offset3[2]);
    P_RHP[2] = P_RHR[2] + (R_RHR[2][0]*offset3[0] + R_RHR[2][1]*offset3[1] + R_RHR[2][2]*offset3[2]);


    Rodrigues(axis3,angle[2],temp_RHP);

    mat3by3x3by3(R_RHR,temp_RHP,R_RHP);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[0][0],R_RHP[0][1],R_RHP[0][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[1][0],R_RHP[1][1],R_RHP[1][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[2][0],R_RHP[2][1],R_RHP[2][2]);


    //----------------RKN pos  &  RKN R mat
    P_RKN[0] = P_RHP[0] + (R_RHP[0][0]*offset4[0] + R_RHP[0][1]*offset4[1] + R_RHP[0][2]*offset4[2]);
    P_RKN[1] = P_RHP[1] + (R_RHP[1][0]*offset4[0] + R_RHP[1][1]*offset4[1] + R_RHP[1][2]*offset4[2]);
    P_RKN[2] = P_RHP[2] + (R_RHP[2][0]*offset4[0] + R_RHP[2][1]*offset4[1] + R_RHP[2][2]*offset4[2]);

//     printf("Pelvis pos  %f \n",PELVISpos[2]);
//        printf("P_RKN: %f \n",P_RKN[2]);


    Rodrigues(axis4,angle[3],temp_RKN);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[0][0],temp_RKN[0][1],temp_RKN[0][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[1][0],temp_RKN[1][1],temp_RKN[1][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[2][0],temp_RKN[2][1],temp_RKN[2][2]);

    mat3by3x3by3(R_RHP,temp_RKN,R_RKN);

//        printf("R_RKN: %f   %f   %f\n",R_RKN[0][0],R_RKN[0][1],R_RKN[0][2]);

//        printf("R_RKN: %f   %f   %f\n",R_RKN[1][0],R_RKN[1][1],R_RKN[1][2]);

//        printf("R_RKN: %f   %f   %f\n",R_RKN[2][0],R_RKN[2][1],R_RKN[2][2]);



    //----------------WHEEL pos  &  WHEEL R mat

    P_WHEEL[0] = P_RKN[0] + (R_RKN[0][0]*offsetwheel[0] + R_RKN[0][1]*offsetwheel[1] + R_RKN[0][2]*offsetwheel[2]);
    P_WHEEL[1] = P_RKN[1] + (R_RKN[1][0]*offsetwheel[0] + R_RKN[1][1]*offsetwheel[1] + R_RKN[1][2]*offsetwheel[2]);
    P_WHEEL[2] = P_RKN[2] + (R_RKN[2][0]*offsetwheel[0] + R_RKN[2][1]*offsetwheel[1] + R_RKN[2][2]*offsetwheel[2]);

    Rodrigues(axiswheel,0.0f,temp_WHEEL);

    mat3by3x3by3(R_RKN,temp_WHEEL,R_WHEEL);




    //----------------RAP pos  &  RAP R mat
//        P_RAP[0] = P_RKN[0] + (R_RKN[0][0]*offset5[0] + R_RKN[0][1]*offset5[1] + R_RKN[0][2]*offset5[2]);
//        P_RAP[1] = P_RKN[1] + (R_RKN[1][0]*offset5[0] + R_RKN[1][1]*offset5[1] + R_RKN[1][2]*offset5[2]);
//        P_RAP[2] = P_RKN[2] + (R_RKN[2][0]*offset5[0] + R_RKN[2][1]*offset5[1] + R_RKN[2][2]*offset5[2]);

//        Rodrigues(axis5,angle[4],temp_RAP);

//        mat3by3x3by3(R_RKN,temp_RAP,R_RAP);

    P_RAP[0] = P_WHEEL[0] + (R_WHEEL[0][0]*offset5[0] + R_WHEEL[0][1]*offset5[1] + R_WHEEL[0][2]*offset5[2]);
    P_RAP[1] = P_WHEEL[1] + (R_WHEEL[1][0]*offset5[0] + R_WHEEL[1][1]*offset5[1] + R_WHEEL[1][2]*offset5[2]);
    P_RAP[2] = P_WHEEL[2] + (R_WHEEL[2][0]*offset5[0] + R_WHEEL[2][1]*offset5[1] + R_WHEEL[2][2]*offset5[2]);

    Rodrigues(axis5,angle[4],temp_RAP);

    mat3by3x3by3(R_WHEEL,temp_RAP,R_RAP);

//        printf("R_RAP: %f   %f   %f\n",R_RAP[0][0],R_RAP[0][1],R_RAP[0][2]);

//        printf("R_RAP: %f   %f   %f\n",R_RAP[1][0],R_RAP[1][1],R_RAP[1][2]);

//        printf("R_RAP: %f   %f   %f\n",R_RAP[2][0],R_RAP[2][1],R_RAP[2][2]);

    //----------------RAR pos  &  RAR R mat
    P_RAR[0] = P_RAP[0] + (R_RAP[0][0]*offset6[0] + R_RAP[0][1]*offset6[1] + R_RAP[0][2]*offset6[2]);
    P_RAR[1] = P_RAP[1] + (R_RAP[1][0]*offset6[0] + R_RAP[1][1]*offset6[1] + R_RAP[1][2]*offset6[2]);
    P_RAR[2] = P_RAP[2] + (R_RAP[2][0]*offset6[0] + R_RAP[2][1]*offset6[1] + R_RAP[2][2]*offset6[2]);

    Rodrigues(axis6,angle[5],temp_RAR);

    mat3by3x3by3(R_RAP,temp_RAR,R_RAR);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[0][0],R_RAR[0][1],R_RAR[0][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[1][0],R_RAR[1][1],R_RAR[1][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[2][0],R_RAR[2][1],R_RAR[2][2]);
    //----------------Foot Pos & ori
    P_FOOT[0] = P_RAR[0] + (R_RAR[0][0]*offset7[0] + R_RAR[0][1]*offset7[1] + R_RAR[0][2]*offset7[2]);
    P_FOOT[1] = P_RAR[1] + (R_RAR[1][0]*offset7[0] + R_RAR[1][1]*offset7[1] + R_RAR[1][2]*offset7[2]);
    P_FOOT[2] = P_RAR[2] + (R_RAR[2][0]*offset7[0] + R_RAR[2][1]*offset7[1] + R_RAR[2][2]*offset7[2]);

    //------------------------Foot edge configuration
    //------------------------------------Foot edge pos & ori

    //----------------Foot Pos & ori
    P_EDGE[0][0] = P_RAR[0] + (R_RAR[0][0]*offset8[0] + R_RAR[0][1]*offset8[1] + R_RAR[0][2]*offset8[2]);
    P_EDGE[0][1] = P_RAR[1] + (R_RAR[1][0]*offset8[0] + R_RAR[1][1]*offset8[1] + R_RAR[1][2]*offset8[2]);
    P_EDGE[0][2] = P_RAR[2] + (R_RAR[2][0]*offset8[0] + R_RAR[2][1]*offset8[1] + R_RAR[2][2]*offset8[2]);

    //----------------Foot Pos & ori
    P_EDGE[1][0] = P_RAR[0] + (R_RAR[0][0]*offset9[0] + R_RAR[0][1]*offset9[1] + R_RAR[0][2]*offset9[2]);
    P_EDGE[1][1] = P_RAR[1] + (R_RAR[1][0]*offset9[0] + R_RAR[1][1]*offset9[1] + R_RAR[1][2]*offset9[2]);
    P_EDGE[1][2] = P_RAR[2] + (R_RAR[2][0]*offset9[0] + R_RAR[2][1]*offset9[1] + R_RAR[2][2]*offset9[2]);

    //----------------Foot Pos & ori
    P_EDGE[2][0] = P_RAR[0] + (R_RAR[0][0]*offset10[0] + R_RAR[0][1]*offset10[1] + R_RAR[0][2]*offset10[2]);
    P_EDGE[2][1] = P_RAR[1] + (R_RAR[1][0]*offset10[0] + R_RAR[1][1]*offset10[1] + R_RAR[1][2]*offset10[2]);
    P_EDGE[2][2] = P_RAR[2] + (R_RAR[2][0]*offset10[0] + R_RAR[2][1]*offset10[1] + R_RAR[2][2]*offset10[2]);

    //----------------Foot Pos & ori
    P_EDGE[3][0] = P_RAR[0] + (R_RAR[0][0]*offset11[0] + R_RAR[0][1]*offset11[1] + R_RAR[0][2]*offset11[2]);
    P_EDGE[3][1] = P_RAR[1] + (R_RAR[1][0]*offset11[0] + R_RAR[1][1]*offset11[1] + R_RAR[1][2]*offset11[2]);
    P_EDGE[3][2] = P_RAR[2] + (R_RAR[2][0]*offset11[0] + R_RAR[2][1]*offset11[1] + R_RAR[2][2]*offset11[2]);


    Rodrigues(axis7,0.0,temp_FOOT);

    mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);

    for(int i=0; i<3; i++)
    {
        FOOT_pos[i] = P_FOOT[i];
        RHY_pos[i] = P_RHY[i];
        RHR_pos[i] = P_RHR[i];
        RHP_pos[i] = P_RHP[i];
        RKN_pos[i] = P_RKN[i];
        RAP_pos[i] = P_RAP[i];
        RAR_pos[i] = P_RAR[i];
    }

//        printf("================= leg collision check================\n");
//        printf("%f   %f   %f \n",WBIK_Q2[0],WBIK_Q2[1],WBIK_Q2[2]);
//        printf("================= leg collision check================\n");
//        printf("RHY pos: %f    %f    %f \n",P_RHY[0],P_RHY[1],P_RHY[2]);
//        printf("RHR pos: %f    %f    %f \n",P_RHR[0],P_RHR[1],P_RHR[2]);
//        printf("RHP pos: %f    %f    %f \n",P_RHP[0],P_RHP[1],P_RHP[2]);
//        printf("RKN pos: %f    %f    %f \n",P_RKN[0],P_RKN[1],P_RKN[2]);
//        printf("RAP pos: %f    %f    %f \n",P_RAP[0],P_RAP[1],P_RAP[2]);
//        printf("RAR pos: %f    %f    %f \n",P_RAR[0],P_RAR[1],P_RAR[2]);


    for(int i=0;i<4;i++)
    {
        for(int j=0 ; j<3; j++)
        {
            _RF_edge[i][j] = P_EDGE[i][j];
            pos[i][j] = P_EDGE[i][j];
        }
    }
}


void LL_Foot_Configurtion(double angle[6],double PELVISpos[3],double pos[6][3])
{
    double axis0[3],axis1[3],axis2[3],axis3[3],axis4[3],axis5[3],axis6[3],axis7[3],axiswheel[3];
    double R_RHY[3][3],R_RHR[3][3],R_RHP[3][3],R_RKN[3][3],R_RAP[3][3],R_RAR[3][3],R_FOOT[3][3],R_WHEEL[3][3],P_RHY[3],P_RHR[3],P_RHP[3],P_RKN[3],P_RAP[3],P_RAR[3],P_FOOT[3],P_EDGE[4][3],P_WHEEL[3];
    double R_PEL[3][3],P_PEL[3],temp_RHY[3][3],temp_RHR[3][3],temp_RHP[3][3],temp_RKN[3][3],temp_RAP[3][3],temp_RAR[3][3],temp_FOOT[3][3],temp_WHEEL[3][3];
    double offset0[3],offset1[3],offset2[3],offset3[3],offset4[3],offset5[3],offset6[3],offset7[3],offset8[3],offset9[3],offset10[3],offset11[3],offsetwheel[3];

//        double p[6][3],aRAR[3],sigh1[2][3],sigh2[2][3],sigh3[2][3],sigh4[2][3];
//        double R_LHY[3][3],R_LHR[3][3],R_LHP[3][3],R_LKN[3][3],R_LAP[3][3],R_LAR[3][3],P_LHY[3],P_LHR[3],P_LHP[3],P_LKN[3],P_LAP[3],P_LAR[3];
//    ----------------initial joint rotation axis

//        aPEL[0] = 0.0f;
//        aPEL[1] = 0.0f;
//        aPEL[2] = 0.1f;

//        Rodrigues(aPEL,0.0f,R_PEL);

//        printf("leg angle %f  %f  %f  %f  %f  %f\n",angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);

    axis0[0] = 0.0f;
    axis0[1] = 0.0f;
    axis0[2] = 1.0f;

    axis1[0] = 0.0f;
    axis1[1] = 0.0f;
    axis1[2] = 1.0f;

    axis2[0] = 1.0f;
    axis2[1] = 0.0f;
    axis2[2] = 0.0f;

    axis3[0] = 0.0f;
    axis3[1] = 1.0f;
    axis3[2] = 0.0f;

    axis4[0] = 0.0f;
    axis4[1] = 1.0f;
    axis4[2] = 0.0f;

    axis5[0] = 0.0f;
    axis5[1] = 1.0f;
    axis5[2] = 0.0f;

    axis6[0] = 1.0f;
    axis6[1] = 0.0f;
    axis6[2] = 0.0f;

    axis7[0] = 0.0f;
    axis7[1] = 0.0f;
    axis7[2] = 1.0f;

    axiswheel[0] = 0.0f;
    axiswheel[1] = 1.0f;
    axiswheel[2] = 0.0f;

    //----------------offset for all joint
//        offset0[0] = P_PEL[0] = WBIK_Q2[0];
//        offset0[1] = P_PEL[1] = WBIK_Q2[1];
//        offset0[2] = P_PEL[2] = WBIK_Q2[2];
    offset0[0] = PELVISpos[0];
    offset0[1] = PELVISpos[1];
    offset0[2] = PELVISpos[2];


    P_PEL[0] = PELVISpos[0];
    P_PEL[1] = PELVISpos[1];
    P_PEL[2] = PELVISpos[2];

    PELVIS_pos[0] = offset0[0];
    PELVIS_pos[1] = offset0[1];
    PELVIS_pos[2] = offset0[2];

//        printf("====================PELVIS POS===================\n");
//        printf("%f   %f   %f\n",P_PEL[0],P_PEL[1],P_PEL[2]);
//        printf("=================================================\n");

    offset1[0] =  0.0f;
    offset1[1] = 0.1f;
    offset1[2] = 0.0f;

    offset2[0] = 0.0f;
    offset2[1] = 0.0f;
    offset2[2] = 0.0f;

    offset3[0] = 0.0f;
    offset3[1] = 0.0f;
    offset3[2] = 0.0f;

    offset4[0] = 0.0;
    offset4[1] = 0.0f;
    offset4[2] = -kine_drc_hubo4.L_UPPER_LEG; //-0.4f;


    offsetwheel[0] = 0.04858f;
    offsetwheel[1] = 0.0f;
    offsetwheel[2] = -0.12458f;

    offset5[0] = -offsetwheel[0];
    offset5[1] = 0.0f;
    offset5[2] = -kine_drc_hubo4.L_LOWER_LEG - offsetwheel[2];//-0.38f;

    offset6[0] = 0.0f;
    offset6[1] = 0.0f;
    offset6[2] = -kine_drc_hubo4.L_ANKLE;//-0.03f;

    offset7[0] = 0.0f;
    offset7[1] = 0.0f;
    offset7[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    //---------------foot edge configuration

    offset8[0] = 0.12f;
    offset8[1] = 0.09f;
    offset8[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    offset9[0] = 0.12f;
    offset9[1] = -0.07f;
    offset9[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    offset10[0] = -0.12f;
    offset10[1] = 0.09f;
    offset10[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

    offset11[0] = -0.12f;
    offset11[1] = -0.07f;
    offset11[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;


//        offset1[0] =  0.0f;
//        offset1[1] =  0.1f;
//        offset1[2] = 0.0f;

//        offset2[0] = 0.0f;
//        offset2[1] = 0.0f;
//        offset2[2] = 0.0f;

//        offset3[0] = 0.0f;
//        offset3[1] = 0.0f;
//        offset3[2] = 0.0f;

//        offset4[0] = 0.0;
//        offset4[1] = 0.0f;
//        offset4[2] = -kine_drc_hubo4.L_UPPER_LEG; //-0.4f;


////        offsetwheel[0] = 0.0f;
////        offsetwheel[1] = 0.0f;
////        offsetwheel[2] = 0.0;//-0.1f;

//        offset5[0] = 0.0f;
//        offset5[1] = 0.0f;
//        offset5[2] = -kine_drc_hubo4.L_LOWER_LEG;//-0.38f;

//        offset6[0] = 0.0f;
//        offset6[1] = 0.0f;
//        offset6[2] = -kine_drc_hubo4.L_ANKLE;//-0.03f;

//        offset7[0] = 0.0f;
//        offset7[1] = 0.0f;
//        offset7[2] = -kine_drc_hubo4.L_FOOT;//-0.1f;

//        printf("LU: %f  %f  %f  %f \n",kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_LOWER_LEG,kine_drc_hubo4.L_ANKLE,kine_drc_hubo4.L_FOOT);

    Rodrigues(axis0,0.0f,R_PEL);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[0][0],R_PEL[0][1],R_PEL[0][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[1][0],R_PEL[1][1],R_PEL[1][2]);

//        printf("R_PEL: %f   %f   %f\n",R_PEL[2][0],R_PEL[2][1],R_PEL[2][2]);

    //----------------RHY pos  &  RHY R mat
    P_RHY[0] = P_PEL[0] + (R_PEL[0][0]*offset1[0] + R_PEL[0][1]*offset1[1] + R_PEL[0][2]*offset1[2]);
    P_RHY[1] = P_PEL[1] + (R_PEL[1][0]*offset1[0] + R_PEL[1][1]*offset1[1] + R_PEL[1][2]*offset1[2]);
    P_RHY[2] = P_PEL[2] + (R_PEL[2][0]*offset1[0] + R_PEL[2][1]*offset1[1] + R_PEL[2][2]*offset1[2]);

    Rodrigues(axis1,angle[0],temp_RHY);

    mat3by3x3by3(R_PEL,temp_RHY,R_RHY);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[0][0],R_RHY[0][1],R_RHY[0][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[1][0],R_RHY[1][1],R_RHY[1][2]);

//        printf("R_RHY: %f   %f   %f\n",R_RHY[2][0],R_RHY[2][1],R_RHY[2][2]);

    //----------------RHR pos  &  RHR R mat
    P_RHR[0] = P_RHY[0] + (R_RHY[0][0]*offset2[0] + R_RHY[0][1]*offset2[1] + R_RHY[0][2]*offset2[2]);
    P_RHR[1] = P_RHY[1] + (R_RHY[1][0]*offset2[0] + R_RHY[1][1]*offset2[1] + R_RHY[1][2]*offset2[2]);
    P_RHR[2] = P_RHY[2] + (R_RHY[2][0]*offset2[0] + R_RHY[2][1]*offset2[1] + R_RHY[2][2]*offset2[2]);

    Rodrigues(axis2,angle[1],temp_RHR);

    mat3by3x3by3(R_RHY,temp_RHR,R_RHR);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[0][0],R_RHR[0][1],R_RHR[0][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[1][0],R_RHR[1][1],R_RHR[1][2]);

//        printf("R_RHR: %f   %f   %f\n",R_RHR[2][0],R_RHR[2][1],R_RHR[2][2]);

    //----------------RHP pos  &  RHP R mat
    P_RHP[0] = P_RHR[0] + (R_RHR[0][0]*offset3[0] + R_RHR[0][1]*offset3[1] + R_RHR[0][2]*offset3[2]);
    P_RHP[1] = P_RHR[1] + (R_RHR[1][0]*offset3[0] + R_RHR[1][1]*offset3[1] + R_RHR[1][2]*offset3[2]);
    P_RHP[2] = P_RHR[2] + (R_RHR[2][0]*offset3[0] + R_RHR[2][1]*offset3[1] + R_RHR[2][2]*offset3[2]);


    Rodrigues(axis3,angle[2],temp_RHP);

    mat3by3x3by3(R_RHR,temp_RHP,R_RHP);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[0][0],R_RHP[0][1],R_RHP[0][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[1][0],R_RHP[1][1],R_RHP[1][2]);

//        printf("R_RHP: %f   %f   %f\n",R_RHP[2][0],R_RHP[2][1],R_RHP[2][2]);



    //----------------RKN pos  &  RKN R mat
    P_RKN[0] = P_RHP[0] + (R_RHP[0][0]*offset4[0] + R_RHP[0][1]*offset4[1] + R_RHP[0][2]*offset4[2]);
    P_RKN[1] = P_RHP[1] + (R_RHP[1][0]*offset4[0] + R_RHP[1][1]*offset4[1] + R_RHP[1][2]*offset4[2]);
    P_RKN[2] = P_RHP[2] + (R_RHP[2][0]*offset4[0] + R_RHP[2][1]*offset4[1] + R_RHP[2][2]*offset4[2]);

//     printf("Pelvis pos  %f \n",PELVISpos[2]);
//        printf("P_RKN: %f \n",P_RKN[2]);


    Rodrigues(axis4,angle[3],temp_RKN);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[0][0],temp_RKN[0][1],temp_RKN[0][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[1][0],temp_RKN[1][1],temp_RKN[1][2]);

//        printf("temp_RKN: %f   %f   %f\n",temp_RKN[2][0],temp_RKN[2][1],temp_RKN[2][2]);

    mat3by3x3by3(R_RHP,temp_RKN,R_RKN);





    //----------------WHEEL pos  &  WHEEL R mat

    P_WHEEL[0] = P_RKN[0] + (R_RKN[0][0]*offsetwheel[0] + R_RKN[0][1]*offsetwheel[1] + R_RKN[0][2]*offsetwheel[2]);
    P_WHEEL[1] = P_RKN[1] + (R_RKN[1][0]*offsetwheel[0] + R_RKN[1][1]*offsetwheel[1] + R_RKN[1][2]*offsetwheel[2]);
    P_WHEEL[2] = P_RKN[2] + (R_RKN[2][0]*offsetwheel[0] + R_RKN[2][1]*offsetwheel[1] + R_RKN[2][2]*offsetwheel[2]);

    Rodrigues(axiswheel,0.0f,temp_WHEEL);

    mat3by3x3by3(R_RKN,temp_WHEEL,R_WHEEL);




    //----------------RAP pos  &  RAP R mat
//        P_RAP[0] = P_RKN[0] + (R_RKN[0][0]*offset5[0] + R_RKN[0][1]*offset5[1] + R_RKN[0][2]*offset5[2]);
//        P_RAP[1] = P_RKN[1] + (R_RKN[1][0]*offset5[0] + R_RKN[1][1]*offset5[1] + R_RKN[1][2]*offset5[2]);
//        P_RAP[2] = P_RKN[2] + (R_RKN[2][0]*offset5[0] + R_RKN[2][1]*offset5[1] + R_RKN[2][2]*offset5[2]);

//        Rodrigues(axis5,angle[4],temp_RAP);

//        mat3by3x3by3(R_RKN,temp_RAP,R_RAP);

    P_RAP[0] = P_WHEEL[0] + (R_WHEEL[0][0]*offset5[0] + R_WHEEL[0][1]*offset5[1] + R_WHEEL[0][2]*offset5[2]);
    P_RAP[1] = P_WHEEL[1] + (R_WHEEL[1][0]*offset5[0] + R_WHEEL[1][1]*offset5[1] + R_WHEEL[1][2]*offset5[2]);
    P_RAP[2] = P_WHEEL[2] + (R_WHEEL[2][0]*offset5[0] + R_WHEEL[2][1]*offset5[1] + R_WHEEL[2][2]*offset5[2]);

    Rodrigues(axis5,angle[4],temp_RAP);

    mat3by3x3by3(R_WHEEL,temp_RAP,R_RAP);

//        //----------------RKN pos  &  RKN R mat
//        P_RKN[0] = P_RHP[0] + (R_RHP[0][0]*offset4[0] + R_RHP[0][1]*offset4[1] + R_RHP[0][2]*offset4[2]);
//        P_RKN[1] = P_RHP[1] + (R_RHP[1][0]*offset4[0] + R_RHP[1][1]*offset4[1] + R_RHP[1][2]*offset4[2]);
//        P_RKN[2] = P_RHP[2] + (R_RHP[2][0]*offset4[0] + R_RHP[2][1]*offset4[1] + R_RHP[2][2]*offset4[2]);

//   //     printf("Pelvis pos  %f \n",PELVISpos[2]);
////        printf("P_RKN: %f \n",P_RKN[2]);


//        Rodrigues(axis4,angle[3],temp_RKN);

////        printf("temp_RKN: %f   %f   %f\n",temp_RKN[0][0],temp_RKN[0][1],temp_RKN[0][2]);

////        printf("temp_RKN: %f   %f   %f\n",temp_RKN[1][0],temp_RKN[1][1],temp_RKN[1][2]);

////        printf("temp_RKN: %f   %f   %f\n",temp_RKN[2][0],temp_RKN[2][1],temp_RKN[2][2]);

//        mat3by3x3by3(R_RHP,temp_RKN,R_RKN);

////        printf("R_RKN: %f   %f   %f\n",R_RKN[0][0],R_RKN[0][1],R_RKN[0][2]);

////        printf("R_RKN: %f   %f   %f\n",R_RKN[1][0],R_RKN[1][1],R_RKN[1][2]);

////        printf("R_RKN: %f   %f   %f\n",R_RKN[2][0],R_RKN[2][1],R_RKN[2][2]);

//        //----------------RAP pos  &  RAP R mat
//        P_RAP[0] = P_RKN[0] + (R_RKN[0][0]*offset5[0] + R_RKN[0][1]*offset5[1] + R_RKN[0][2]*offset5[2]);
//        P_RAP[1] = P_RKN[1] + (R_RKN[1][0]*offset5[0] + R_RKN[1][1]*offset5[1] + R_RKN[1][2]*offset5[2]);
//        P_RAP[2] = P_RKN[2] + (R_RKN[2][0]*offset5[0] + R_RKN[2][1]*offset5[1] + R_RKN[2][2]*offset5[2]);

//        Rodrigues(axis5,angle[4],temp_RAP);

//        mat3by3x3by3(R_RKN,temp_RAP,R_RAP);

////        printf("R_RAP: %f   %f   %f\n",R_RAP[0][0],R_RAP[0][1],R_RAP[0][2]);

////        printf("R_RAP: %f   %f   %f\n",R_RAP[1][0],R_RAP[1][1],R_RAP[1][2]);

////        printf("R_RAP: %f   %f   %f\n",R_RAP[2][0],R_RAP[2][1],R_RAP[2][2]);

    //----------------RAR pos  &  RAR R mat
    P_RAR[0] = P_RAP[0] + (R_RAP[0][0]*offset6[0] + R_RAP[0][1]*offset6[1] + R_RAP[0][2]*offset6[2]);
    P_RAR[1] = P_RAP[1] + (R_RAP[1][0]*offset6[0] + R_RAP[1][1]*offset6[1] + R_RAP[1][2]*offset6[2]);
    P_RAR[2] = P_RAP[2] + (R_RAP[2][0]*offset6[0] + R_RAP[2][1]*offset6[1] + R_RAP[2][2]*offset6[2]);

    Rodrigues(axis6,angle[5],temp_RAR);

    mat3by3x3by3(R_RAP,temp_RAR,R_RAR);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[0][0],R_RAR[0][1],R_RAR[0][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[1][0],R_RAR[1][1],R_RAR[1][2]);

//        printf("R_RAR: %f   %f   %f\n",R_RAR[2][0],R_RAR[2][1],R_RAR[2][2]);


    //----------------Foot Pos & ori
    P_FOOT[0] = P_RAR[0] + (R_RAR[0][0]*offset7[0] + R_RAR[0][1]*offset7[1] + R_RAR[0][2]*offset7[2]);
    P_FOOT[1] = P_RAR[1] + (R_RAR[1][0]*offset7[0] + R_RAR[1][1]*offset7[1] + R_RAR[1][2]*offset7[2]);
    P_FOOT[2] = P_RAR[2] + (R_RAR[2][0]*offset7[0] + R_RAR[2][1]*offset7[1] + R_RAR[2][2]*offset7[2]);


    //------------------------------------Foot edge pos & ori

    //----------------Foot Pos & ori
    P_EDGE[0][0] = P_RAR[0] + (R_RAR[0][0]*offset8[0] + R_RAR[0][1]*offset8[1] + R_RAR[0][2]*offset8[2]);
    P_EDGE[0][1] = P_RAR[1] + (R_RAR[1][0]*offset8[0] + R_RAR[1][1]*offset8[1] + R_RAR[1][2]*offset8[2]);
    P_EDGE[0][2] = P_RAR[2] + (R_RAR[2][0]*offset8[0] + R_RAR[2][1]*offset8[1] + R_RAR[2][2]*offset8[2]);

    //----------------Foot Pos & ori
    P_EDGE[1][0] = P_RAR[0] + (R_RAR[0][0]*offset9[0] + R_RAR[0][1]*offset9[1] + R_RAR[0][2]*offset9[2]);
    P_EDGE[1][1] = P_RAR[1] + (R_RAR[1][0]*offset9[0] + R_RAR[1][1]*offset9[1] + R_RAR[1][2]*offset9[2]);
    P_EDGE[1][2] = P_RAR[2] + (R_RAR[2][0]*offset9[0] + R_RAR[2][1]*offset9[1] + R_RAR[2][2]*offset9[2]);

    //----------------Foot Pos & ori
    P_EDGE[2][0] = P_RAR[0] + (R_RAR[0][0]*offset10[0] + R_RAR[0][1]*offset10[1] + R_RAR[0][2]*offset10[2]);
    P_EDGE[2][1] = P_RAR[1] + (R_RAR[1][0]*offset10[0] + R_RAR[1][1]*offset10[1] + R_RAR[1][2]*offset10[2]);
    P_EDGE[2][2] = P_RAR[2] + (R_RAR[2][0]*offset10[0] + R_RAR[2][1]*offset10[1] + R_RAR[2][2]*offset10[2]);

    //----------------Foot Pos & ori
    P_EDGE[3][0] = P_RAR[0] + (R_RAR[0][0]*offset11[0] + R_RAR[0][1]*offset11[1] + R_RAR[0][2]*offset11[2]);
    P_EDGE[3][1] = P_RAR[1] + (R_RAR[1][0]*offset11[0] + R_RAR[1][1]*offset11[1] + R_RAR[1][2]*offset11[2]);
    P_EDGE[3][2] = P_RAR[2] + (R_RAR[2][0]*offset11[0] + R_RAR[2][1]*offset11[1] + R_RAR[2][2]*offset11[2]);



    Rodrigues(axis7,0.0,temp_FOOT);

    mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);

    FOOT_pos2[0] = P_FOOT[0];
    FOOT_pos2[1] = P_FOOT[1];
    FOOT_pos2[2] = P_FOOT[2];

    RHY_pos2[0] = P_RHY[0];
    RHY_pos2[1] = P_RHY[1];
    RHY_pos2[2] = P_RHY[2];

    RHR_pos2[0] = P_RHR[0];
    RHR_pos2[1] = P_RHR[1];
    RHR_pos2[2] = P_RHR[2];

    RHP_pos2[0] = P_RHP[0];
    RHP_pos2[1] = P_RHP[1];
    RHP_pos2[2] = P_RHP[2];

    RKN_pos2[0] = P_RKN[0];
    RKN_pos2[1] = P_RKN[1];
    RKN_pos2[2] = P_RKN[2];

    RAP_pos2[0] = P_RAP[0];
    RAP_pos2[1] = P_RAP[1];
    RAP_pos2[2] = P_RAP[2];

    RAR_pos2[0] = P_RAR[0];
    RAR_pos2[1] = P_RAR[1];
    RAR_pos2[2] = P_RAR[2];


//        printf("================= leg collision check================\n");
//        printf("%f   %f   %f \n",WBIK_Q2[0],WBIK_Q2[1],WBIK_Q2[2]);
//        printf("================= leg collision check================\n");
//        printf("RHY pos: %f    %f    %f \n",P_RHY[0],P_RHY[1],P_RHY[2]);
//        printf("RHR pos: %f    %f    %f \n",P_RHR[0],P_RHR[1],P_RHR[2]);
//        printf("RHP pos: %f    %f    %f \n",P_RHP[0],P_RHP[1],P_RHP[2]);
//        printf("RKN pos: %f    %f    %f \n",P_RKN[0],P_RKN[1],P_RKN[2]);
//        printf("RAP pos: %f    %f    %f \n",P_RAP[0],P_RAP[1],P_RAP[2]);
//        printf("RAR pos: %f    %f    %f \n",P_RAR[0],P_RAR[1],P_RAR[2]);

    for(int i=0; i<3; i++)
    {
        FOOT_pos[i] = P_FOOT[i];
        RHY_pos[i] = P_RHY[i];
        RHR_pos[i] = P_RHR[i];
        RHP_pos[i] = P_RHP[i];
        RKN_pos[i] = P_RKN[i];
        RAP_pos[i] = P_RAP[i];
        RAR_pos[i] = P_RAR[i];
    }


    for(int i=0;i<4;i++)
    {
        for(int j=0 ; j<3; j++)
        {
            _LF_edge[i][j] = P_EDGE[i][j];
            pos[i][j] = P_EDGE[i][j];
        }
    }




}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void Print_WBIK_Infos()
{
    printf("m_TORSO = %f,C_TORSO = %f, %f, %f\n",kine_drc_hubo4.m_Torso,kine_drc_hubo4.C_Torso[0],kine_drc_hubo4.C_Torso[1],kine_drc_hubo4.C_Torso[2]);
    printf("m_PELVIS = %f,C_PELVIS = %f, %f, %f\n",kine_drc_hubo4.m_Pelvis,kine_drc_hubo4.C_Pelvis[0],kine_drc_hubo4.C_Pelvis[1],kine_drc_hubo4.C_Pelvis[2]);

    printf("m_LEFT_UPPER_LEG = %f,C_LEFT_UPPER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_LeftUpperLeg,kine_drc_hubo4.C_LeftUpperLeg[0],kine_drc_hubo4.C_LeftUpperLeg[1],kine_drc_hubo4.C_LeftUpperLeg[2]);
    printf("m_RIGHT_UPPER_LEG = %f,C_RIGHT_UPPER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_RightUpperLeg,kine_drc_hubo4.C_RightUpperLeg[0],kine_drc_hubo4.C_RightUpperLeg[1],kine_drc_hubo4.C_RightUpperLeg[2]);

    printf("m_LEFT_LOWER_LEG = %f,C_LEFT_LOWER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_LeftLowerLeg,kine_drc_hubo4.C_LeftLowerLeg[0],kine_drc_hubo4.C_LeftLowerLeg[1],kine_drc_hubo4.C_LeftLowerLeg[2]);
    printf("m_RIGHT_LOWER_LEG = %f,C_RIGHT_LOWER_LEG = %f, %f, %f\n",kine_drc_hubo4.m_RightLowerLeg,kine_drc_hubo4.C_RightLowerLeg[0],kine_drc_hubo4.C_RightLowerLeg[1],kine_drc_hubo4.C_RightLowerLeg[2]);

    printf("m_LEFT_ANKLE = %f,C_LEFT_ANKLE = %f, %f, %f\n",kine_drc_hubo4.m_LeftFoot,kine_drc_hubo4.C_LeftFoot[0],kine_drc_hubo4.C_LeftFoot[1],kine_drc_hubo4.C_LeftFoot[2]);
    printf("m_RIGHT_ANKLE = %f,C_RIGHT_ANKLE = %f, %f, %f\n",kine_drc_hubo4.m_RightFoot,kine_drc_hubo4.C_RightFoot[0],kine_drc_hubo4.C_RightFoot[1],kine_drc_hubo4.C_RightFoot[2]);

    printf("m_LEFT_LOWER_ARM = %f,C_LEFT_LOWER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_LeftLowerArm,kine_drc_hubo4.C_LeftLowerArm[0],kine_drc_hubo4.C_LeftLowerArm[1],kine_drc_hubo4.C_LeftLowerArm[2]);
    printf("m_RIGHT_LOWER_ARM = %f,C_RIGHT_LOWER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_RightLowerArm,kine_drc_hubo4.C_RightLowerArm[0],kine_drc_hubo4.C_RightLowerArm[1],kine_drc_hubo4.C_RightLowerArm[2]);

    printf("m_LEFT_UPPER_ARM = %f,C_LEFT_UPPER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_LeftUpperArm,kine_drc_hubo4.C_LeftUpperArm[0],kine_drc_hubo4.C_LeftUpperArm[1],kine_drc_hubo4.C_LeftUpperArm[2]);
    printf("m_RIGHT_UPPER_ARM = %f,C_RIGHT_UPPER_ARM = %f, %f, %f\n",kine_drc_hubo4.m_RightUpperArm,kine_drc_hubo4.C_RightUpperArm[0],kine_drc_hubo4.C_RightUpperArm[1],kine_drc_hubo4.C_RightUpperArm[2]);

    printf("m_LEFT_HAND = %f,C_LEFT_HAND = %f, %f, %f\n",kine_drc_hubo4.m_LeftHand,kine_drc_hubo4.C_LeftHand[0],kine_drc_hubo4.C_LeftHand[1],kine_drc_hubo4.C_LeftHand[2]);
    printf("m_RIGHT_HAND = %f,C_RIGHT_HAND = %f, %f, %f\n",kine_drc_hubo4.m_RightHand,kine_drc_hubo4.C_RightHand[0],kine_drc_hubo4.C_RightHand[1],kine_drc_hubo4.C_RightHand[2]);

    printf("m_RIGHT_WRIST = %f,m_LEFT_WRIST = %f\n",kine_drc_hubo4.m_RightWrist,kine_drc_hubo4.m_LeftWrist);

     printf("L_LOWER_LEG = %f , L_UPPER_LEG = %f, L_FOOT = %f\n",kine_drc_hubo4.L_LOWER_LEG,kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_FOOT);

     printf("L_PC2WST = %f , L_PEL2PEL = %f, L_ANKLE = %f\n",kine_drc_hubo4.L_PC2WST,kine_drc_hubo4.L_PEL2PEL,kine_drc_hubo4.L_ANKLE);
     printf("L_WST2SHOULDER = %f , L_SHOULDER2SHOULDER = %f, L_UPPER_ARM = %f\n",kine_drc_hubo4.L_WST2SHOULDER,kine_drc_hubo4.L_UPPER_LEG,kine_drc_hubo4.L_UPPER_ARM);
     printf("L_LOWER_ARM = %f , L_HAND = %f, L_ELB_OFFSET = %f\n",kine_drc_hubo4.L_LOWER_ARM,kine_drc_hubo4.L_HAND,kine_drc_hubo4.L_ELB_OFFSET);

     double total_mass;

     total_mass = kine_drc_hubo4.m_Pelvis+kine_drc_hubo4.m_Torso + kine_drc_hubo4.m_LeftUpperLeg+kine_drc_hubo4.m_RightUpperLeg+kine_drc_hubo4.m_LeftLowerLeg+kine_drc_hubo4.m_RightLowerLeg+kine_drc_hubo4.m_LeftFoot+kine_drc_hubo4.m_RightFoot+kine_drc_hubo4.m_LeftLowerArm+kine_drc_hubo4.m_RightLowerArm+kine_drc_hubo4.m_LeftUpperArm+kine_drc_hubo4.m_RightUpperArm+kine_drc_hubo4.m_LeftWrist*2.;
     printf("total mass : %f\n",total_mass);
}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void MotionCheck()
{
    //-----------------Vision DATA-----------------//
//    _floor_num = userData->G2M.block_num;

//            printf("========================Vision DATA TES======================\n");
//            printf(">>>>>>>>>>>>>>>>Plane center<<<<<<<<<<<<<<<\n");
//            printf("Plane Number : %d \n",_floor_num);

//    for(int i = 0; i< _floor_num ; i++)
//    {
//        for(int j=0; j<3; j++)
//        {
//            _floor_center[i][j] = userData->G2M.block_center[i][j];
//        }

//                printf("INDEX: %d    Plane center : %f   %f   %f \n", i,_floor_center[i][0],_floor_center[i][1],_floor_center[i][2]);

//    }

//            printf(">>>>>>>>>>>>>>>>Plane center<<<<<<<<<<<<<<<\n");

//    for(int i = 0; i< _floor_num ; i++)
//    {

//        for(int j=0; j<3; j++)
//        {
//            _floor_normal[i][j] = userData->G2M.block_normal[i][j];
//        }

////                printf("INDEX: %d    Plane normal : %f   %f   %f \n", i,_floor_normal[i][0],_floor_normal[i][1],_floor_normal[i][2]);
//    }

//            printf(">>>>>>>>>>>>>>>>Plane edge<<<<<<<<<<<<<<<\n");

//    for(int i = 0; i< _floor_num ; i++)
//    {
//        for(int k = 0; k<12; k++)
//        {
//            _floor_edge[i][k] = userData->G2M.block_edge[i][k];

//        }

////                printf("INDEX: %d    Plane edge : %f   %f   %f \n", i,_floor_edge[i][0],_floor_edge[i][1],_floor_edge[i][2]);
////                printf("INDEX: %d    Plane edge : %f   %f   %f \n", i,_floor_edge[i][3],_floor_edge[i][4],_floor_edge[i][5]);
////                printf("INDEX: %d    Plane edge : %f   %f   %f \n", i,_floor_edge[i][6],_floor_edge[i][7],_floor_edge[i][8]);
////                printf("INDEX: %d    Plane edge : %f   %f   %f \n", i,_floor_edge[i][9],_floor_edge[i][10],_floor_edge[i][11]);
//    }


    //=====coordinate conversion

    _generating_motion_flag = true;
    _generating_cnt = 0;
    _pertubation[0] = 0.0f; _pertubation[1] = 0.0f; _pertubation[2] = 0.0f;


    while(_generating_motion_flag)
    {
        fsmDEL_T = 0.005;
        cout << ">>> COMMAND: MOTION CHECK" << endl;

                checkfsm_clear();

        joint->RefreshToCurrentReference();

        for(int i=0; i<=LAR; i++) {
           WBIK_Q2[i+7] = joint->Joints[i]->RefAngleCurrent*D2R;
        }

        //-------------------------------INIT MOTION PARAMETER---------------------------------//
    //    printf("RHY = %f,RHR = %f,RHP = %f,RKN = %f,RAP = %f, RAR = %f\n",WBIK_Q[idRHY]*R2D,WBIK_Q[idRHR]*R2D,WBIK_Q[idRHP]*R2D,WBIK_Q[idRKN]*R2D,WBIK_Q[idRAP]*R2D,WBIK_Q[idRAR]*R2D);
        // PELVIS Orientation Reset
        WBIK_Q2[idRSR]= 0;//10.*D2R;//joint->Joints[RSR]->RefAngleCurrent*D2R;
        WBIK_Q2[idRSP]= 0;//40.*D2R;//joint->Joints[RSP]->RefAngleCurrent*D2R;
        WBIK_Q2[idRSY]= 0;//joint->Joints[RSY]->RefAngleCurrent*D2R;
        WBIK_Q2[idREB]= -10*D2R;//-130.*D2R;//joint->Joints[REB]->RefAngleCurrent*D2R;
        WBIK_Q2[idRWY]= 0;//joint->Joints[RWY]->RefAngleCurrent*D2R;
        WBIK_Q2[idRWP]= 0;//20.*D2R;//joint->Joints[RWP]->RefAngleCurrent*D2R;

        WBIK_Q2[idLSR]= 0;//-10.*D2R;//joint->Joints[LSR]->RefAngleCurrent*D2R;
        WBIK_Q2[idLSP]= 0;//40.*D2R;//joint->Joints[LSP]->RefAngleCurrent*D2R;
        WBIK_Q2[idLSY]= 0;//joint->Joints[LSY]->RefAngleCurrent*D2R;
        WBIK_Q2[idLEB]= -10*D2R;//-130.*D2R;//joint->Joints[LEB]->RefAngleCurrent*D2R;
        WBIK_Q2[idLWY]= 0;//joint->Joints[LWY]->RefAngleCurrent*D2R;
        WBIK_Q2[idLWP]= 0;//20.*D2R;//joint->Joints[LWP]->RefAngleCurrent*D2R;

        WBIK_Q2[idWST]= -180*D2R;//joint->Joints[WST]->RefAngleCurrent*D2R;

        WBIK_Q2[idQ0] = 1;
        WBIK_Q2[idQ1] = 0;
        WBIK_Q2[idQ2] = 0;
        WBIK_Q2[idQ3] = 0;

        // PELVIS Position Reset
        //kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
        //printf("com z = %f,pel z = %f\n",FK_pCOM_3x1[2],WBIK_Q[idZ]);
        WBIK_Q2[idX] = init_WBIK_Q[0] - init_WBIK_pCOM[0];//reset to 0;
        WBIK_Q2[idY] = init_WBIK_Q[1] - init_WBIK_pCOM[1];//reset to 0;
        WBIK_Q2[idZ] = init_WBIK_Q[2] - init_WBIK_pCOM[2] + userData->WalkReadyCOM[2] + fsm->AddComInfos[0][2];//0;

        kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q2,FK_pRFoot_3x1,  FK_qRFoot_4x1);
        kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q2,FK_pLFoot_3x1,  FK_qLFoot_4x1);
        kine_drc_hubo4.FK_COM_Global(WBIK_Q2,FK_pCOM_3x1);

        initialPos.Left[0] = FK_pLFoot_3x1[0];
        initialPos.Left[1] = FK_pLFoot_3x1[1];//+0.02;
        initialPos.Left[2] = FK_pLFoot_3x1[2];
        QT2EULER(FK_qLFoot_4x1,initialPos.LRoll,initialPos.LPitch,initialPos.LYaw);

        initialPos.Right[0] = FK_pRFoot_3x1[0];
        initialPos.Right[1] = FK_pRFoot_3x1[1];
        initialPos.Right[2] = FK_pRFoot_3x1[2];
        QT2EULER(FK_qRFoot_4x1,initialPos.RRoll,initialPos.RPitch,initialPos.RYaw);
        initialPos.HTime = 0.0;
        //----comz
        initialPos.ZMP[0] = FK_pCOM_3x1[0];
        initialPos.ZMP[1] = FK_pCOM_3x1[1];
        initialPos.COMz[0] = fsm->AddComInfos[0][2];
        printf("initialPos.Left[0]= %f,%f,%f,initialPos.Right[0]= %f,%f,%f\n",initialPos.Left[0],initialPos.Left[1],initialPos.Left[2],initialPos.Right[0],initialPos.Right[1],initialPos.Right[2]);
        checkfsm->SetInitialFootPlace2(initialPos);
       //Walking_initialize();
        GetGain(userData->WalkReadyCOM[2]+COM_Offset);



//                cout<<"======================================================"<<endl;
//                cout<<"WBIK_Q2x :"<<WBIK_Q2[idX]<<","<<WBIK_Q2[idY]<<","<<WBIK_Q2[idZ]<<endl;
//                cout<<"======================================================"<<endl;
//                cout<<"======================================================"<<endl;
//                cout<<"WBIK_Q2x :"<<WBIK_Q2[idX]<<","<<WBIK_Q2[idY]<<","<<WBIK_Q2[idZ]<<endl;
//                cout<<"======================================================"<<endl;
//                cout<<"======================================================"<<endl;
//                cout<<"WBIK_Q2x :"<<WBIK_Q2[idX]<<","<<WBIK_Q2[idY]<<","<<WBIK_Q2[idZ]<<endl;
//                cout<<"initialpos left :"<<initialPos.Left[0]<<","<<initialPos.Left[1]<<","<<initialPos.Left[2]<<endl;
//                cout<<"initialpos Right :"<<initialPos.Right[0]<<","<<initialPos.Right[1]<<","<<initialPos.Right[2]<<endl;
//                cout<<"======================================================"<<endl;

        //-------------------------------INIT MOTION PARAMETER---------------------------------//



        //------------------------------------USER COMMAND-------------------------------------//
        checkfsm->walking_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];

        if(checkfsm->walking_mode == TERRAIN_WALKING){
            checkfsm->TIME_DSP = 0.2f*1.0f;  //dsp
            checkfsm->TIME_SSP = 1.4f*1.0f;  //ssp
            printf("BLOCK CLIMB TASK speed set DSP: %f  SSP: %f \n",checkfsm->TIME_DSP,checkfsm->TIME_SSP);
        }else if(checkfsm->walking_mode == LADDER_WALKING){
            checkfsm->TIME_DSP = 0.5f;
            checkfsm->TIME_SSP = 3.5f;
            printf("LADDER CLIMB TASK speed set DSP: %f  SSP: %f \n",checkfsm->TIME_DSP,checkfsm->TIME_SSP);
        }else if(checkfsm->walking_mode == NORMAL_WALKING){
            checkfsm->TIME_DSP = 0.3f*1.0;//1.5f;
            checkfsm->TIME_SSP = 0.7*1.0;//3.0f;
            printf("NORMAL TASK speed set DSP: %f  SSP: %f \n",checkfsm->TIME_DSP,checkfsm->TIME_SSP);
        }

        // Recv All Data (after 2014/10/30)
        checkfsm->DSPScheduler.clear();
        for(int i=0; i<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]; i++)
        {
            DSPTask task;

            task.Left[0]    = userData->G2M.walkingDSP[i*14];
            task.Left[1]    = userData->G2M.walkingDSP[i*14+1];
            task.Left[2]    = userData->G2M.walkingDSP[i*14+2];
            task.LYaw       = userData->G2M.walkingDSP[i*14+3];
            task.LRoll      = userData->G2M.walkingDSP[i*14+4];
            task.LPitch     = userData->G2M.walkingDSP[i*14+5];

            task.Right[0]   = userData->G2M.walkingDSP[i*14+6];
            task.Right[1]   = userData->G2M.walkingDSP[i*14+7];
            task.Right[2]   = userData->G2M.walkingDSP[i*14+8];
            task.RYaw       = userData->G2M.walkingDSP[i*14+9];
            task.RRoll      = userData->G2M.walkingDSP[i*14+10];
            task.RPitch     = userData->G2M.walkingDSP[i*14+11];

            task.COMz[0]    = userData->G2M.walkingDSP[i*14+12];
            task.COMz[1]    = 0;//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i*14+13];


//                    if(_LengthLimit_flag == true  && _AngleLimit_flag == false)
            if(_LengthLimit_flag == true){
                if(i == _motion_section_num && i != 0){
                    task.COMz[0]    = userData->G2M.walkingDSP[i*14+12] - (float)(_generating_cnt)*0.01f;
                    printf("Length limit exceed index: %d task com0 : %f  Pertubation :  %f \n",i, task.COMz[0],(float)(_generating_cnt)*0.03f);
                    printf("Length limit exceed index: %d task com0 : %f  Pertubation :  %f \n",i, task.COMz[0],(float)(_generating_cnt)*0.03f);
                    printf("Length limit exceed index: %d task com0 : %f  Pertubation :  %f \n",i, task.COMz[0],(float)(_generating_cnt)*0.03f);
                    _pertubation[i]  = task.COMz[0] ;
                }
            }

//                    if(_AngleLimit_flag == true && _LengthLimit_flag == false)
            if(_AngleLimit_flag == true){
                if(i == _motion_section_num && i != 0){
                    task.COMz[0]    =  userData->G2M.walkingDSP[i*14+12] + (float)(_generating_cnt)*0.02f;
                    printf("Angle limit exceed index: %d task com0 : %f  Pertubation :  %f \n",i, task.COMz[0],(float)(_generating_cnt)*0.02f);
                    printf("Angle limit exceed index: %d task com0 : %f  Pertubation :  %f \n",i, task.COMz[0],(float)(_generating_cnt)*0.02f);
                    printf("Angle limit exceed index: %d task com0 : %f  Pertubation :  %f \n",i, task.COMz[0],(float)(_generating_cnt)*0.02f);
                    _pertubation[i] = task.COMz[0];
                }
            }

            _pertubation[i] = task.COMz[0];

            if(_AngleLimit_flag == true && _LengthLimit_flag == true){
                printf("Angle and Length Limit exceed!!! \n");
                printf("Angle and Length Limit exceed!!! \n");
                printf("Angle and Length Limit exceed!!! \n");
            }

            checkfsm->DSPScheduler.push_back(task);
        }

        for(int i=0; i<checkfsm->DSPScheduler.size(); i++)
        {
            printf("index(%d)  LF: (%.4f, %.4f, %.4f)(%.4f,%.4f,%.4f) RF: (%.4f,%.4f,%.4f)(%.4f,%.4f,%.4f) COMz:%f \n", i, checkfsm->DSPScheduler[i].Left[0], checkfsm->DSPScheduler[i].Left[1], checkfsm->DSPScheduler[i].Left[2], checkfsm->DSPScheduler[i].LYaw, checkfsm->DSPScheduler[i].LRoll, checkfsm->DSPScheduler[i].LPitch, checkfsm->DSPScheduler[i].Right[0], checkfsm->DSPScheduler[i].Right[1], checkfsm->DSPScheduler[i].Right[2], checkfsm->DSPScheduler[i].RYaw , checkfsm->DSPScheduler[i].RRoll , checkfsm->DSPScheduler[i].RPitch ,checkfsm->DSPScheduler[i].COMz[0]);
        }
        //------------------------------------USER COMMAND-------------------------------------//


        //----------------------------------Motion Generation----------------------------------//
        pv_Index = 0;_motion_check = false;_collision_check = false;_motion_check_start = true; _LengthLimit_flag = false;
        _AngleLimit_flag = false;_transition_flag = false;

        while(_motion_check_start == true && _LengthLimit_flag == false && _AngleLimit_flag == false)
        {
            checkfsm->Update();
            if(_preview_flag_Motion == true)
            {
                ThreeDim_Preview_Motion_check();
            }else
            {
                Preliminary_Motion_check();
                _preview_flag_Motion = true;
            }
            //JW_INV_MODEL(GLOBAL_Y_LIPM_n - (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0,GLOBAL_Y_LIPM_d_n,GLOBAL_X_LIPM_n,GLOBAL_X_LIPM_d_n);
            WBIK_Motion_check();



             double Rangle[6],Langle[6],lleg[2][3],lf_up_edge[2][3],lf_down_edge[2][3],rf_up_edge[2][3],rf_down_edge[2][3],rleg[2][3],obj[2][3],dis[1],Lpos[6][3],Rpos[6][3];//,vision[2][3];

            _pelvis_position[0] = WBIK_Q2[0];
            _pelvis_position[1] = WBIK_Q2[1];
            _pelvis_position[2] = WBIK_Q2[2];

            Langle[0] = FWRefAngleCurrent[LHY];
            Langle[1] = FWRefAngleCurrent[LHR];
            Langle[2] = FWRefAngleCurrent[LHP];
            Langle[3] = FWRefAngleCurrent[LKN];
            Langle[4] = FWRefAngleCurrent[LAP];
            Langle[5] = FWRefAngleCurrent[LAR];

            Rangle[0] = FWRefAngleCurrent[RHY];
            Rangle[1] = FWRefAngleCurrent[RHR];
            Rangle[2] = FWRefAngleCurrent[RHP];
            Rangle[3] = FWRefAngleCurrent[RKN];
            Rangle[4] = FWRefAngleCurrent[RAP];
            Rangle[5] = FWRefAngleCurrent[RAR];

            //---------------------------Leg shin configuration

            LL_Configurtion(Langle,_pelvis_position,Lpos);

            lleg[0][0] =  _legpos[2][0] = Lpos[4][0];
            lleg[0][1] =  _legpos[2][1] = Lpos[4][1];
            lleg[0][2] =  _legpos[2][2] = Lpos[4][2];
            lleg[1][0] =  _legpos[3][0] = Lpos[3][0];
            lleg[1][1] =  _legpos[3][1] = Lpos[3][1];
            lleg[1][2] =  _legpos[3][2] = Lpos[3][2];

            RL_Configurtion(Rangle,_pelvis_position,Rpos);

            rleg[0][0] = _legpos[0][0] = Rpos[4][0];
            rleg[0][1] = _legpos[0][1] = Rpos[4][1];
            rleg[0][2] = _legpos[0][2] = Rpos[4][2];
            rleg[1][0] = _legpos[1][0] = Rpos[3][0];
            rleg[1][1] = _legpos[1][1] = Rpos[3][1];
            rleg[1][2] = _legpos[1][2] = Rpos[3][2];

            //---------------------------Leg Foot edge configuration

            LL_Foot_Configurtion(Langle,_pelvis_position,Lpos);
            RL_Foot_Configurtion(Rangle,_pelvis_position,Rpos);

            for(int k = 0; k<2;k++)
            {
                for(int m = 0;m<3;m++)
                {
                    lf_up_edge[k][m] = Lpos[k][m];
                    lf_down_edge[k][m] = Lpos[k+2][m];
                    rf_up_edge[k][m] = Rpos[k][m];
                    rf_down_edge[k][m] = Rpos[k+2][m];
                }
            }



            //---------------------------Leg and block edge configuration

            //-----------------------------------Past block Collision Check---------------------------------//
            for(int i = 0; i< _last_floor_num; i++)
            {
                obj[0][0] = _last_floor_edge[i][0] + _last_pCOM_3x1[0];// vision[0][0];
                obj[0][1] = _last_floor_edge[i][1] + _last_pCOM_3x1[1];
                obj[0][2] = _last_floor_edge[i][2] + _last_pCOM_3x1[2];

                obj[1][0] = _last_floor_edge[i][3] + _last_pCOM_3x1[0];
                obj[1][1] = _last_floor_edge[i][4] + _last_pCOM_3x1[1];
                obj[1][2] = _last_floor_edge[i][5] + _last_pCOM_3x1[2];

                //---shin edge collision check
                Leg_Collision_Check(lleg,obj,dis);

                _shin_dist[1] = dis[0];

                Leg_Collision_Check(rleg,obj,dis);

                _shin_dist[0] = dis[0];
                //---shin edge collision check


                //---foot edge collision check
                Leg_Collision_Check(lf_up_edge,obj,dis);

                _foot_dist[0] = dis[0];

                Leg_Collision_Check(lf_down_edge,obj,dis);

                _foot_dist[1] = dis[0];

                Leg_Collision_Check(rf_up_edge,obj,dis);

                _foot_dist[2] = dis[0];

                Leg_Collision_Check(rf_down_edge,obj,dis);

                _foot_dist[3] = dis[0];
                //---lf and rf edge collision check


                if(_collision_check == false)
                {
                    if(_shin_dist[0] < 0.1f || _shin_dist[1] < 0.1f)
                    {
                        printf("Past Block collision!!!  collision!!!  collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        _collision_check = true;
                    }

                    if(_foot_dist[0] < 0.03f || _foot_dist[1] < 0.03f || _foot_dist[2] < 0.03 || _foot_dist[3] < 0.03)
                    {
                        printf("Past Block collision!!!  collision!!!  collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        printf("Past Block !!!  Past Block collision!!!  Past Block collision!!! \n");
                        _collision_check = true;
                    }
                }

            }
            //-----------------------------------Past block  Collision Check --------------------------------//



            //-----------------------------------Current block Collision Check---------------------------------//
            for(int i = 0; i< _floor_num; i++)
            {
                obj[0][0] = _floor_edge[i][0];// vision[0][0];
                obj[0][1] = _floor_edge[i][1];
                obj[0][2] = _floor_edge[i][2];

                obj[1][0] = _floor_edge[i][3];
                obj[1][1] = _floor_edge[i][4];
                obj[1][2] = _floor_edge[i][5];

                Leg_Collision_Check(lleg,obj,dis);

                _shin_dist[1] = dis[0];

                Leg_Collision_Check(rleg,obj,dis);

                _shin_dist[0] = dis[0];

                //---foot edge collision check
                Leg_Collision_Check(lf_up_edge,obj,dis);

                _foot_dist[0] = dis[0];

                Leg_Collision_Check(lf_down_edge,obj,dis);

                _foot_dist[1] = dis[0];

                Leg_Collision_Check(rf_up_edge,obj,dis);

                _foot_dist[2] = dis[0];

                Leg_Collision_Check(rf_down_edge,obj,dis);

                _foot_dist[3] = dis[0];
                //---lf and rf edge collision check

                if(_collision_check == false)
                {
                    if(_shin_dist[0] < 0.1f || _shin_dist[1] < 0.1f)
                    {
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        _collision_check = true;
                    }

                    if(_foot_dist[0] < 0.03f || _foot_dist[1] < 0.03f || _foot_dist[2] < 0.03f || _foot_dist[3] < 0.03f)
                    {
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        printf("collision!!!  collision!!!  collision!!! \n");
                        _collision_check = true;
                    }
                }

            }
            //-----------------------------------Current block Collision Check --------------------------------//


            JW_save_Motion_check();
            JW_save2_Motion_check();
        }
        _preview_flag_Motion = false;
        //----------------------------------Motion Generation----------------------------------//



        //------------------------------------Motion Check-------------------------------------//
//                if(_INIT_singularFlag == false && _DSP_singularFlag == false && _SSP_singularFlag == false)
//                {
//                    _generating_motion_flag = false;
//                }
//                _INIT_singularFlag = false;
//                _DSP_singularFlag = false;
//                _SSP_singularFlag = false;

        if(_LengthLimit_flag == false && _AngleLimit_flag == false)
        {
            _generating_motion_flag = false;
        }

        _generating_cnt++;

        if(_generating_cnt >= _generating_loop_limint)
        {
            _generating_motion_flag = false;
            printf("Motion Generation Loop Count Exceed %d!!! \n",_generating_cnt);
            printf("Motion Generation Loop Count Exceed %d!!! \n",_generating_cnt);
            printf("Motion Generation Loop Count Exceed %d!!! \n",_generating_cnt);
        }
        //------------------------------------Motion Check-------------------------------------//

    }
    printf("Motion Check is finished : %d   Loop Count: %d  Section of Problem : %d \n",checkfsm->StateInfos[0][0],_generating_cnt, _motion_section_num);
    printf("===============DELTA COM0 : %f ===============\n" , _pertubation[0]);
    printf("===============DELTA COM1 : %f ===============\n" , _pertubation[1]);
    printf("===============DELTA COM2 : %f ===============\n" , _pertubation[2]);



}


int RBindicator(unsigned char number)
{
    MANUAL_CAN	MCData;

    MCData.channel = 1;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 3;
    MCData.data[0] = 24;
    MCData.data[1] = 0x84;
    MCData.data[2] = (unsigned char)number;

    return PushCANMessage(MCData);
}
void Upperbody_Gain_Override()
{
//    RBsetFrictionParameter(CAN2, JMC13, 60, 70, 10, 0);
//    RBsetFrictionParameter(CAN2, JMC14, 50, 70, 10, 0);
//    RBsetFrictionParameter(CAN2, JMC15, 60, 50, 70, 50);

//    RBsetFrictionParameter(CAN3, JMC17, 60, 80, 10, 0);
//    RBsetFrictionParameter(CAN3, JMC18, 50, 70, 10, 0);
//    RBsetFrictionParameter(CAN3, JMC19, 60, 55, 60, 42);

//    RBBoardSetSwitchingMode(CAN2,JMC13, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC14, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC15, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN2,JMC13,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC14,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC15,ENABLE,ENABLE);
//    RBJointGainOverride(CAN2,JMC13,1,1000,1000);
//    RBJointGainOverride(CAN2,JMC14,1,1000,1000);
//    RBJointGainOverride(CAN2,JMC15,1,1,1000);

//    RBBoardSetSwitchingMode(CAN3,JMC17, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC18, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC19, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN3,JMC17,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC18,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC19,ENABLE,ENABLE);
//    RBJointGainOverride(CAN3,JMC17,1,1000,1000);
//    RBJointGainOverride(CAN3,JMC18,1,1000,1000);
//    RBJointGainOverride(CAN3,JMC19,1,1,1000);
    ZeroGainLeftArm();
    ZeroGainRightArm();
}
int ZeroGainRightArm(){
    //-- Rsp
//    RBsetFrictionParameter(2, 13, 60, 70, 10,0);
//    RBBoardSetSwitchingMode(2, 13, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 13,ENABLE,DISABLE);
//    RBJointGainOverride(2, 13,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1000, 8, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 50, 10);
    usleep(5000);

    //-- Rsr
//    RBsetFrictionParameter(2, 14, 50, 70, 10,0);
//    RBBoardSetSwitchingMode(2, 14, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 14,ENABLE,DISABLE);
//    RBJointGainOverride(2, 14,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 50, 10);
    usleep(5000);

    //--- Rsy, Reb
//    RBsetFrictionParameter(2, 15, 60, 50, 70, 50);
//    RBBoardSetSwitchingMode(2, 15, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 15,ENABLE,ENABLE);
//    RBJointGainOverride(2, 15,0,0,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 40, 10);
    usleep(5000);

    //--- Rwy, Rwp
//    RBsetFrictionParameter(2, 16, 80, 80, 60,110);
//    RBBoardSetSwitchingMode(2, 16, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 16,ENABLE,ENABLE);
//    RBJointGainOverride(2, 16,0,0,1);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 360, 5, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 100, 10);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 350,6, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 100, 10);
//    usleep(5000);

    //---- Rwy2
    //RBsetFrictionParameter(2, 36, 6, 120, 10,0);
//    RBsetFrictionParameter(2, 36, 400, 50, 10,0);
//    RBenableFrictionCompensation(2, 36,ENABLE,DISABLE);
//    RBJointGainOverride(2, 36,0,1000,1);
//    MCsetFrictionParameter(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 280, 120, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 65, 10);
//    usleep(5000);

//    //---- RAP
//    RBsetFrictionParameter(0, 4, 60, 70, 10,0);
//    RBBoardSetSwitchingMode(0, 4, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(0, 4,ENABLE,DISABLE);
//    RBJointGainOverride(0, 4,0,1000,1);

//    RBJointOLCurrentCommand2ch(2, 13, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 14, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 15, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 16, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 36, 0, 0, 0x05);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}
int ZeroGainRightArm2(){
    //-- Rsp
//    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1000, 8, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 95, 10);
    usleep(5000);

    //-- Rsr
//    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 95, 10);
    usleep(5000);

    //--- Rsy, Reb
//    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 95, 10);
    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 95, 10);
    usleep(5000);

    //--- Rwy, Rwp
    MCsetFrictionParameter(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 360, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 95, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 350,6, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 95, 10);
    usleep(5000);

    //---- Rwy2
    MCsetFrictionParameter(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 280, 120, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 70, 10);
    usleep(5000);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}
int ZeroGainLeftLeg(){
    //-- lsp
//    RBsetFrictionParameter(3, 17, 60, 80, 10,0);
//    RBBoardSetSwitchingMode(3,17, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,17,ENABLE,DISABLE);
//    RBJointGainOverride(3,17,0,1000,1);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[LHR].canch, JOINT_INFO[LHR].bno, JOINT_INFO[LHR].mch, 1000, 10, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[LHR].canch, JOINT_INFO[LHR].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LHR].canch, JOINT_INFO[LHR].bno, JOINT_INFO[LHR].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LHR].canch, JOINT_INFO[LHR].bno, JOINT_INFO[LHR].mch, 80, 3);
//    usleep(5000);

//    MCsetFrictionParameter(JOINT_INFO[LKN].canch, JOINT_INFO[LKN].bno, JOINT_INFO[LKN].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LKN].canch, JOINT_INFO[LKN].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LKN].canch, JOINT_INFO[LKN].bno, JOINT_INFO[LKN].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LKN].canch, JOINT_INFO[LKN].bno, JOINT_INFO[LKN].mch, 90, 3);
//    usleep(5000);

    cout<<"Zero gain LeftLeg!"<<endl;
    return 0;
}
int ZeroGainRightLeg(){
    //-- lsp
//    RBsetFrictionParameter(3, 17, 60, 80, 10,0);
//    RBBoardSetSwitchingMode(3,17, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,17,ENABLE,DISABLE);
//    RBJointGainOverride(3,17,0,1000,1);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[RHR].canch, JOINT_INFO[RHR].bno, JOINT_INFO[RHR].mch, 1000, 10, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RHR].canch, JOINT_INFO[RHR].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RHR].canch, JOINT_INFO[RHR].bno, JOINT_INFO[RHR].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RHR].canch, JOINT_INFO[RHR].bno, JOINT_INFO[RHR].mch, 80, 3);
//    usleep(5000);

//    MCsetFrictionParameter(JOINT_INFO[RKN].canch, JOINT_INFO[RKN].bno, JOINT_INFO[RKN].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RKN].canch, JOINT_INFO[RKN].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RKN].canch, JOINT_INFO[RKN].bno, JOINT_INFO[RKN].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RKN].canch, JOINT_INFO[RKN].bno, JOINT_INFO[RKN].mch, 90, 3);


//    usleep(5000);

    cout<<"Zero gain RightLeg!"<<endl;
    return 0;
}
int ZeroGainLeftArm(){
    //-- lsp
//    RBsetFrictionParameter(3, 17, 60, 80, 10,0);
//    RBBoardSetSwitchingMode(3,17, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,17,ENABLE,DISABLE);
//    RBJointGainOverride(3,17,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 50, 10);
    usleep(5000);

    //-- lsr
//    RBsetFrictionParameter(3, 18, 50, 70, 10,0);
//    RBBoardSetSwitchingMode(3,18, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,18,ENABLE,DISABLE);
//    RBJointGainOverride(3,18,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 50, 10);
    usleep(5000);

    //--- lsy, leb
//    RBsetFrictionParameter(3, 19, 60, 55, 60, 42);
//    RBBoardSetSwitchingMode(3,19, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,19,ENABLE,ENABLE);
//    RBJointGainOverride(3,19,0,0,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 40, 10);
    usleep(5000);

//    //--- lwy, lwp
//    RBsetFrictionParameter(3, 20, 80, 100, 60,115);
//    RBBoardSetSwitchingMode(3,20, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,20,ENABLE,ENABLE);
//    RBJointGainOverride(3,20,0,0,1);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 330, 5, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 100, 10);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 350, 5, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 100, 10);
//    usleep(5000);

//    //---- lwy2
//    //RBsetFrictionParameter(3, 37, 6, 120, 10,0);  // 11W motor(Iptime Hubo)
////    RBsetFrictionParameter(3, 37, 400, 50, 10,0);
////    RBenableFrictionCompensation(3,37,ENABLE,DISABLE);
////    RBJointGainOverride(3,37,0,1000,1);
//    MCsetFrictionParameter(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 280, 150, 0);
//    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 100, 10);
//    usleep(5000);

//    RBJointOLCurrentCommand2ch(3, 17, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, 0, 0, 0x05);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);



    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}
int ZeroGainLeftArm2(){
    //-- lsp
//    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 95, 10);
    usleep(5000);

    //-- lsr
//    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 95, 10);
    usleep(5000);

    //--- lsy, leb
//    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 95, 10);
    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 95, 10);
    usleep(5000);

//    //--- lwy, lwp
    MCsetFrictionParameter(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 330, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 95, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 350, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 95, 10);
    usleep(5000);

//    //---- lwy2
    MCsetFrictionParameter(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 280, 150, 0);
    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 70, 10);
    usleep(5000);

    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}
void FullGainLeftLeg()
{
    MCenableFrictionCompensation(JOINT_INFO[LHR].canch, JOINT_INFO[LHR].bno, JOINT_INFO[LHR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LHR].canch,JOINT_INFO[LHR].bno, SW_MODE_COMPLEMENTARY);
    MCJointGainOverride(JOINT_INFO[LHR].canch, JOINT_INFO[LHR].bno, JOINT_INFO[LHR].mch, 0,400);
    cout<<"Full gain RightLeg!"<<endl;
}
void FullGainRightLeg()
{
    MCenableFrictionCompensation(JOINT_INFO[RHR].canch, JOINT_INFO[RHR].bno, JOINT_INFO[RHR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RHR].canch,JOINT_INFO[RHR].bno, SW_MODE_COMPLEMENTARY);
    MCJointGainOverride(JOINT_INFO[RHR].canch, JOINT_INFO[RHR].bno, JOINT_INFO[RHR].mch, 0,400);
    cout<<"Full gain RightLeg!"<<endl;
}
void Upperbody_Gain_Lock()
{
//    RBBoardSetSwitchingMode(CAN2,JMC13, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC14, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC15, SW_MODE_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN2,JMC13,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC14,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC15,DISABLE,DISABLE);
//    RBJointGainOverride(CAN2,JMC13,1000,1000,1000);
//    RBJointGainOverride(CAN2,JMC14,1000,1000,1000);
//    RBJointGainOverride(CAN2,JMC15,1000,1000,1000);

//    RBBoardSetSwitchingMode(CAN3,JMC17, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC18, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC19, SW_MODE_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN3,JMC17,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC18,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC19,DISABLE,DISABLE);
//    RBJointGainOverride(CAN3,JMC17,1000,1000,1000);
//    RBJointGainOverride(CAN3,JMC18,1000,1000,1000);
//    RBJointGainOverride(CAN3,JMC19,1000,1000,1000);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch,JOINT_INFO[LWY].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch,JOINT_INFO[LWP].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWY2].canch,JOINT_INFO[LWY2].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch,JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch,JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch,JOINT_INFO[RWY2].bno, SW_MODE_COMPLEMENTARY);

            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,500); //--LSP
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,500); //--LSR
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,500); //--LSY
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,500); //--LEB

//            MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 0,500); //---LWY
//            MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 0,500); //--LWP
//            MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0,500); //--LWY2

            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,500); //--RSP
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,500); //--RSR
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,500); //--RSY
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,500); //--REB

//            MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 0,500); //---RWY
//            MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 0,500); //--RWP
//            MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,500); //--RWY2

}

unsigned int RBsetFrictionParameter(unsigned int _canch, unsigned int _bno,
                                    int _vel_saturation1, int _amp_compen1, int _vel_saturation2, int _amp_compen2) // inhyeok
{
    // MsgID		Byte0	Byte1	Byte2				Byte3				Byte4						Byte5				Byte6				Byte7
    // CMD_TXDF		BNO		0xB0	VEL_Saturation1		VEL_Saturation1		Ampere_Compensation1(mA)	VEL_Saturation2		VEL_Saturation2		Ampere_Compensation2(mA)

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 8;
    MCData.data[0] = _bno;							// board no.
    MCData.data[1] = 0xB0;							// command
    MCData.data[2] = _vel_saturation1 & 0xFF;
    MCData.data[3] = (_vel_saturation1>>8) & 0xFF;
    MCData.data[4] = _amp_compen1 & 0xFF;
    MCData.data[5] = _vel_saturation2 & 0xFF;
    MCData.data[6] = (_vel_saturation2>>8) & 0xFF;
    MCData.data[7] = _amp_compen2 & 0xFF;

    return PushCANMessage(MCData);
}
double NotchFilter_GyroRollControlInput(double _input, int _reset)
{
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroPitchControlInput(double _input, int _reset)
{
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroRollVel(double _input, int _reset)
{
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroPitchVel(double _input, int _reset)
{
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
void GotoFallPos(){

    ZeroGainLeftArm2();
    ZeroGainRightArm2();

    ZeroGainRightLeg();
    ZeroGainLeftLeg();


    Walking_initialize();

    double postime = 900.0;
    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();
    double xcom,ycom;
    if(Fall_Pitch_DIR == FALL_PLUS) xcom =-0.1;
    else xcom =0.1;

    if(Fall_Roll_DIR == FALL_PLUS) ycom =0.08;
    else ycom =-0.08;

    des_pCOM_3x1[0] = FK_pCOM_3x1[0] + xcom ;
    des_pCOM_3x1[1] = FK_pCOM_3x1[1] + ycom;
    des_pCOM_3x1[2] = FK_pCOM_3x1[2]-0.28;

    double PelYaw,PelRoll,PelPitch,temp1_qPel_4x1[4],temp2_qPel_4x1[4],temp3_qPel_4x1[4],temp4_qPel_4x1[4],temp5_qPel_4x1[4];
    PelYaw = 0*D2R;
    PelRoll = 0*D2R;
    PelPitch = 20*D2R;

    qtRZ(PelYaw, temp1_qPel_4x1);
    qtRX(PelRoll, temp2_qPel_4x1);
    qtRY(PelPitch, temp3_qPel_4x1);

    QTcross(temp1_qPel_4x1,temp2_qPel_4x1,temp4_qPel_4x1);
    QTcross(temp4_qPel_4x1,temp3_qPel_4x1,temp5_qPel_4x1);


    des_qPEL_4x1[0] = temp5_qPel_4x1[0];
    des_qPEL_4x1[1] = temp5_qPel_4x1[1];
    des_qPEL_4x1[2] = temp5_qPel_4x1[2];
    des_qPEL_4x1[3] = temp5_qPel_4x1[3];

    des_pRF_3x1[0] = FK_pRFoot_3x1[0];
    des_pRF_3x1[1] = FK_pRFoot_3x1[1];
    des_pRF_3x1[2] = FK_pRFoot_3x1[2];

    des_qRF_4x1[0] = FK_qRFoot_4x1[0];
    des_qRF_4x1[1] = FK_qRFoot_4x1[1];
    des_qRF_4x1[2] = FK_qRFoot_4x1[2];
    des_qRF_4x1[3] = FK_qRFoot_4x1[3];

    des_pLF_3x1[0] = FK_pLFoot_3x1[0];
    des_pLF_3x1[1] = FK_pLFoot_3x1[1];
    des_pLF_3x1[2] = FK_pLFoot_3x1[2];

    des_qLF_4x1[0] = FK_qLFoot_4x1[0];
    des_qLF_4x1[1] = FK_qLFoot_4x1[1];
    des_qLF_4x1[2] = FK_qLFoot_4x1[2];
    des_qLF_4x1[3] = FK_qLFoot_4x1[3];

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

    Qub[idRSR] = 10.*D2R;
    Qub[idLSR] = -10.*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = -130.*D2R;
    Qub[idLEB] = -130.*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
//    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
//                                const double des_pRF_3x1[], const double des_qRF_4x1[],
//                                const double des_pLF_3x1[], const double des_qLF_4x1[],
//                                double Q_return_34x1[]);
    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D+10, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D+10, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);

    double total_mass;
   double total_lower_mass;
   total_mass          = kine_drc_hubo4.m_Pelvis + kine_drc_hubo4.m_Torso + kine_drc_hubo4.m_LeftUpperLeg+kine_drc_hubo4.m_RightUpperLeg+kine_drc_hubo4.m_LeftLowerLeg+kine_drc_hubo4.m_RightLowerLeg+kine_drc_hubo4.m_LeftFoot+kine_drc_hubo4.m_RightFoot+kine_drc_hubo4.m_LeftLowerArm+kine_drc_hubo4.m_RightLowerArm+kine_drc_hubo4.m_LeftUpperArm+kine_drc_hubo4.m_RightUpperArm + kine_drc_hubo4.m_LeftHand + kine_drc_hubo4.m_RightHand + kine_drc_hubo4.m_LeftWrist+kine_drc_hubo4.m_RightWrist;
   total_lower_mass    = kine_drc_hubo4.m_Pelvis + kine_drc_hubo4.m_LeftUpperLeg+kine_drc_hubo4.m_RightUpperLeg+kine_drc_hubo4.m_LeftLowerLeg+kine_drc_hubo4.m_RightLowerLeg+kine_drc_hubo4.m_LeftFoot+kine_drc_hubo4.m_RightFoot;
   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
   Fall_once_Flag = true;

}
void GlobalSlope()
{
    double ori_RF[3],ori_LF[3];
    for(int i=0; i<fsm->DSPScheduler.size();i++)
    {
        if(i==0)continue;
        else if(i == 1)
        {
            if(fsm->DSPScheduler[i].LYaw == fsm->DSPScheduler[i-1].LYaw && fsm->DSPScheduler[i].LRoll == fsm->DSPScheduler[i-1].LRoll && fsm->DSPScheduler[i].LPitch == fsm->DSPScheduler[i-1].LPitch && fsm->DSPScheduler[i].Left[0] == fsm->DSPScheduler[i-1].Left[0] && fsm->DSPScheduler[i].Left[1] == fsm->DSPScheduler[i-1].Left[1] && fsm->DSPScheduler[i].Left[2] == fsm->DSPScheduler[i-1].Left[2])
            {
                ori_RF[0] = fsm->DSPScheduler[i].RYaw*D2R;
                ori_RF[1] = fsm->DSPScheduler[i].RRoll*D2R;
                ori_RF[2] = fsm->DSPScheduler[i].RPitch*D2R;

                QTtransform(Global_qFoot_4x1,fsm->DSPScheduler[i].Right,fsm->DSPScheduler[i].Right);
                QTtransform(Global_qFoot_4x1,ori_RF,ori_RF);

                fsm->DSPScheduler[i].RYaw   = ori_RF[0]*R2D;
                fsm->DSPScheduler[i].RRoll  = ori_RF[1]*R2D;
                fsm->DSPScheduler[i].RPitch = ori_RF[2]*R2D;

            }
            else if(fsm->DSPScheduler[i].RYaw == fsm->DSPScheduler[i-1].RYaw && fsm->DSPScheduler[i].RRoll == fsm->DSPScheduler[i-1].RRoll && fsm->DSPScheduler[i].RPitch == fsm->DSPScheduler[i-1].RPitch && fsm->DSPScheduler[i].Right[0] == fsm->DSPScheduler[i-1].Right[0] && fsm->DSPScheduler[i].Right[1] == fsm->DSPScheduler[i-1].Right[1] && fsm->DSPScheduler[i].Right[2] == fsm->DSPScheduler[i-1].Right[2])
            {
                ori_LF[0] = fsm->DSPScheduler[i].LYaw*D2R;
                ori_LF[1] = fsm->DSPScheduler[i].LRoll*D2R;
                ori_LF[2] = fsm->DSPScheduler[i].LPitch*D2R;

                QTtransform(Global_qFoot_4x1,fsm->DSPScheduler[i].Left,fsm->DSPScheduler[i].Left);
                QTtransform(Global_qFoot_4x1,ori_LF,ori_LF);

                fsm->DSPScheduler[i].LYaw   = ori_LF[0]*R2D;
                fsm->DSPScheduler[i].LRoll  = ori_LF[1]*R2D;
                fsm->DSPScheduler[i].LPitch = ori_LF[2]*R2D;
            }
        }
        else if(i >= 2)
        {
            ori_LF[0] = fsm->DSPScheduler[i].LYaw*D2R;
            ori_LF[1] = fsm->DSPScheduler[i].LRoll*D2R;
            ori_LF[2] = fsm->DSPScheduler[i].LPitch*D2R;
            ori_RF[0] = fsm->DSPScheduler[i].RYaw*D2R;
            ori_RF[1] = fsm->DSPScheduler[i].RRoll*D2R;
            ori_RF[2] = fsm->DSPScheduler[i].RPitch*D2R;

            QTtransform(Global_qFoot_4x1,fsm->DSPScheduler[i].Right,fsm->DSPScheduler[i].Right);
            QTtransform(Global_qFoot_4x1,ori_RF,ori_RF);
            QTtransform(Global_qFoot_4x1,fsm->DSPScheduler[i].Left,fsm->DSPScheduler[i].Left);
            QTtransform(Global_qFoot_4x1,ori_LF,ori_LF);

            fsm->DSPScheduler[i].RYaw   = ori_RF[0]*R2D;
            fsm->DSPScheduler[i].RRoll  = ori_RF[1]*R2D;
            fsm->DSPScheduler[i].RPitch = ori_RF[2]*R2D;

            fsm->DSPScheduler[i].LYaw   = ori_LF[0]*R2D;
            fsm->DSPScheduler[i].LRoll  = ori_LF[1]*R2D;
            fsm->DSPScheduler[i].LPitch = ori_LF[2]*R2D;
        }
    }
}



void mat3by3x3by1(double a[3][3],double b[3],double out[3])
{

    double sum = 0.0f;

    for(int i=0;i<=2;i++)
    {
            for(int j=0;j<=2;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}

double ZMPControllerX(double desX,double desY,double desXdot,double desYdot,double desPx,double desPy,double x,double x_dot,double px,double y,double y_dot,double py)
{
    double B[3],A[3][3],g=9.81,Tp=0.01,z=0.83;
    double K[3]={-13.0,-3.0,-sqrt(g/z)};
    double X[3] = {x,x_dot,px},Y[3] = {y,y_dot,py};
    double AX[3],AY[3],UX,UY;
    double NX[3],NY[3],cv;
    A[0][0] = 0.0;
    A[1][0] = 1.0;
    A[2][0] = 0.0;

    A[0][1] = g/z;
    A[1][1] = 0.0;
    A[2][1] =-g/z;

    A[0][2] = 0.0;
    A[1][2] = 0.0;
    A[2][2] = -1.0/Tp;

    B[0] = 0.0;
    B[1] = 0.0;
    B[2] = 1.0/Tp;

    UX = K[0]*(desX - x) + K[1]*(desXdot - x_dot) + K[2]*(desPx - px) + desPx;

    UY = K[0]*(desY - y) + K[1]*(desYdot - y_dot) + K[2]*(desPy - py) + desPy;

    mat3by3x3by1(A,X,AX);

    cv  = NX[0] = - AX[0]*Tp;
    NX[1] = AX[1]*Tp;
    NX[2] = (AX[2] + B[2]*UX)*Tp;

    mat3by3x3by1(A,Y,AY);

    NY[0] = AY[0]*Tp;
    NY[1] = AY[1]*Tp;
    NY[2] = (AY[2] + B[2]*UY)*Tp;

    return cv;



}



double ZMPControllerY(double desX,double desY,double desXdot,double desYdot,double desPx,double desPy,double x,double x_dot,double px,double y,double y_dot,double py)
{
    double B[3],A[3][3],g=9.81,Tp=0.01,z=0.83;
    double K[3]={-13.0,-3.0,-sqrt(g/z)};
    double X[3] = {x,x_dot,px},Y[3] = {y,y_dot,py};
    double AX[3],AY[3],UX,UY;
    double NX[3],NY[3],cv;
    A[0][0] = 0.0;
    A[1][0] = 1.0;
    A[2][0] = 0.0;

    A[0][1] = g/z;
    A[1][1] = 0.0;
    A[2][1] =-g/z;

    A[0][2] = 0.0;
    A[1][2] = 0.0;
    A[2][2] = -1.0/Tp;

    B[0] = 0.0;
    B[1] = 0.0;
    B[2] = 1.0/Tp;

    UX = K[0]*(desX - x) + K[1]*(desXdot - x_dot) + K[2]*(desPx - px) + desPx;

    UY = K[0]*(desY - y) + K[1]*(desYdot - y_dot) + K[2]*(desPy - py) + desPy;

    mat3by3x3by1(A,X,AX);

    NX[0] = AX[0]*Tp;
    NX[1] = AX[1]*Tp;
    NX[2] = (AX[2] + B[2]*UX)*Tp;

    mat3by3x3by1(A,Y,AY);

    cv = NY[0] = -AY[0]*Tp;
    NY[1] = AY[1]*Tp;
    NY[2] = (AY[2] + B[2]*UY)*Tp;

    return cv;



}



double FootForceControl(double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{
//    static double Z_ctrl = 0.0,a = 0.0001,T = 1.0;

//    Z_ctrl = a*(T*((dLforce - dRforce) - (mLforce - mRforce)) - T*T) + a*T*T*exp(-((dLforce - dRforce) - (mLforce - mRforce))/T);

//    static double d = 2000, m = 1.5,dt = 0.005;
//    static double Qd[3]={0.0,},Q[3]={0.0,};

//    Q[0] = (mLforce - mRforce) - (dLforce - dRforce) + (d/dt)*(Qd)

//    static double QQ0 = 0.0, QQ1 = 0.0,d = 0.001,T = 1.0, dt = 0.005;

//    QQ1 = (d*((dLforce - dRforce) - (mLforce - mRforce)) + QQ0/dt)/(1.0/0.005 + 1.0/1.0);
//    QQ0 = QQ1;

    static double CL=0.0;
    static double d = 6000, m = 2.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};

//    if(reset == 0)
//    {
//        Q[0] = 0.0f;
//        Q[1] = 0.0f;
//        Q[2] = 0.0f;
//        Qd0 = 0.0f;
//        Qd1 = 0.0f;
//        Qd2 = 0.0f;
////        CL = 0.0;
//    }else
    {

        if(pv_Index >=2)
        {
            if(reset == 1)
            {
                //DSP
                d = 5000;
            }else
            {
                d = 25000;
            }

            CL = Q[0] = (((dLforce - dRforce) - (mLforce - mRforce)) + d*(Qd0-Qd1)/dt + ( 2.0*m/(dt*dt) + d/dt)*Q[1] - m*Q[2]/(dt*dt))/(m/(dt*dt) + d/dt);

            Q[2] = Q[1];

            Q[1] = Q[0];

            Qd1 = Qd0;


//            CL = Q[0] = (((mRforce) - (dRforce)) + d*(Qd0-Qd1)/dt + ( 2.0*m/(dt*dt) + d/dt)*Q[1] - m*Q[2]/(dt*dt))/(m/(dt*dt) + d/dt);

//            Q[2] = Q[1];

//            Q[1] = Q[0];

//            Qd1 = Qd0;

            if(CL > 0.05)
            {
                CL = 0.05;
            }else if( CL < -0.05)
            {
                CL = -0.05;
            }


        }
    }

    return CL;
}



double RightDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y)
{
    double alpha,RDF,LDF;
    double M = 0.0,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = -alpha*M*g;
    LDF = -(1.0 - alpha)*M*g;

    return RDF;
}

double LeftDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y)
{
    double alpha,RDF,LDF;
    double M = 0.0,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = -alpha*M*g;
    LDF = -(1.0 - alpha)*M*g;


    return LDF;
}

void ReactiveControl(int state,int state2,int state3)
{
//    RDesTorqueX = RX_DesTorque(state, GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1]);

//    RDRoll = RightRollTorqueControl(RDesTorqueX,sharedSEN->FT[RAFT].Mx, fsm->RightInfos[0][4]*D2R);


//    LDesTorqueX = LX_DesTorque(state, GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1]);

//    LDRoll = LeftRollTorqueControl(LDesTorqueX,sharedSEN->FT[LAFT].Mx,fsm->LeftInfos[0][4]*D2R);

//    RDRoll = RX_TC(state,state2,state3,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedSEN->FT[RAFT].Mx,fsm->RightInfos[0][4]*D2R);
//    LDRoll = LX_TC(state,state2,state3,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedSEN->FT[LAFT].Mx,fsm->LeftInfos[0][4]*D2R);

//    RDPitch = RY_TC(state,state2,state3,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedSEN->FT[RAFT].Mx,fsm->RightInfos[0][5]*D2R);
//    LDPitch = LY_TC(state,state2,state3,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedSEN->FT[LAFT].Mx,fsm->LeftInfos[0][5]*D2R);


    RDF = RightDesForce(state, GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1]);

    LDF = LeftDesForce(state, GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1]);


    Zctrl = FootForceControl(RDF,LDF,sharedSEN->FT[RAFT].Fz,sharedSEN->FT[LAFT].Fz,state,0.0);

//    LZctrl = LFootForceControl(RDF,LDF,sharedSEN->FT[RAFT].Fz,sharedSEN->FT[LAFT].Fz,state,fsm->LeftInfos[0][2]);

//    RZctrl = RFootForceControl(RDF,LDF,sharedSEN->FT[RAFT].Fz,sharedSEN->FT[LAFT].Fz,state,fsm->RightInfos[0][2]);


//    LZctrl = LFootForceControl(RDF,LDF,sharedSEN->FT[RAFT].Fz,sharedSEN->FT[LAFT].Fz,state,0.0);

//    RZctrl = RFootForceControl(RDF,LDF,sharedSEN->FT[RAFT].Fz,sharedSEN->FT[LAFT].Fz,state,0.0);
}
