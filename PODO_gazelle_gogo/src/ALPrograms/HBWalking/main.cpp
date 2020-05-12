
#include "BasicFiles/BasicSetting.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <timer.h>

#include "taskmotion.h"

#include "HB_functions.h"
#include "HB_walking.h"
#include "HB_inverse.h"
#include "HB_State_est.h"
#include "HB_Jumping.h"

#include "HB_PreviewWalk.h"
#include "GG_SingleLogWalk.h"

#include "robotstateestimator.h"                        //hyoin
#include "../../share/Headers/LANData/ROSLANData.h"

#define SAVEN       390

#define PODO_AL_NAME       "HBWalking"

using namespace std;

// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *joint;

int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

HB_WALKING      HBWalking;
DesiredStates   DS;
RobotStates     RST, RST_ini;
RobotSensor     RSEN;
REFERENCE       REF;
BP_RBDL         bp_rbdl;
HB_SE           SE;
HB_PreviewWalk  HBPW;
GG_SingleLogWalk GGSW;
HB_inverse      Hi;

HB_JUMP HBJumping;

TaskMotion      *WBmotion;
int             WB_FLAG = 0;
static const double dt = 0.002;

Gazelle_Kine GK;
RobotStateEstimator RSE;

//SYS ID
double      InputFreq;
double      InputAmp;
vec3        Initial_COM;
vec3        Initial_pRF;
vec3        Initial_pLF;
vec3        ctrl_COM;
double      z_ctrl;
unsigned int SysIDcnt;
unsigned int SysIDcnt_init;
double Fz_diff_error_old = 0;
int FLAG_SendROS = CMD_BREAK;
int input_Amp;
vec4 input_torque_filtered = vec4(0,0,0,0);

// Hip roll vib control
double RHR_con_deg = 0;
double LHR_con_deg = 0;
double RHY_con_deg = 0;
double LHY_con_deg = 0;
double LHP_con_deg = 0;
double RHP_con_deg = 0;
double LKN_con_deg = 0;
double RKN_con_deg = 0;

//Ankle angle
double RAR_ang_old = 0;
double RAP_ang_old = 0;
double LAR_ang_old = 0;
double LAP_ang_old = 0;

//torso Orientation control
double RHR_comp_Ang = 0;
double LHR_comp_Ang = 0;

//Ankle angle
double RA1_ref_deg, RA2_ref_deg, LA1_ref_deg, LA2_ref_deg;

// waste momentum compensation
double WST_ref_deg = 0;

// Ready To walk mode
vec3 COM_ini_global, pRF_ini_global, pLF_ini_global;

// State Estimator
vec3 pel_estimated;

//SAVE
int Scnt = 0;
int Sggcnt = 0;
double SAVE[SAVEN][50000];
double SAVE_GG[SAVEN][50000];

// Functions
void ResetGlobalCoord(int RF_OR_LF_OR_PC_MidFoot);
void SensorInput();
double getEnc(int in);
double getPosRef(int in);
double getEncVel(int in);
void Torque2Choreonoid(DesiredStates _Des_State);
void save_onestep(int cnt);
void save_onestep_ggsw(int cnt);
void save_all();
void save_all_gg();
void init_StateEstimator();
doubles quat2doubles(quat _quat);

enum HBWalking_COMMAND
{
    HBWalking_NO_ACT = 100,
    HBWalking_DATA_SAVE,
    HBWalking_TORQUE_TEST,
    HBWalking_ANKLE_TEST,
    HBWalking_STOP,
    HBWalking_WALK_TEST,
    HBWalking_SYSID_START,
    HBWalking_SYSID_STEP_INPUT_START,
    HBWalking_SYSID_STOP_SAVE,
    HBWalking_CONTROL_ON,
    HBWalking_OL_TORQUE_TUNING,
    HBWalking_ZERO_GAIN,
    HBWalking_INV_DYN_CONTROL,
    HBWalking_DYN_STANDING,
    HBWalking_POS_STANDING,
    HBWalking_DYN_WALKING,
    HBWalking_DYN_WALKING2,
    HBWalking_JUMP,
    HBWalking_PrevWalk,
    HBWalking_GetComHeight,
    HBWalking_Test,
    HBWalking_JoyStick_Walk_Stop,
    HBWalking_Ready_To_Walk,
    HBWalking_SingleLog,
    HBWalking_ROSWalk
};

enum SE_COMMAND
{
    SE_NO_ACT = 100,
    SE_CALC_TEST,
    SE_EST_AXIS,
    SE_EST_START,
    SE_EST_FINISH
};

enum task_thread
{
    _task_Idle = 0,
    _task_Idle_SingleLog,
    _task_CPS_walking,
    _task_Stabilization,
    _task_TorqueTest,
    _task_HB_Walking,
    _task_SysID,
    _task_SysID_step,
    _task_Control_on,
    _task_Ankle_Torque_con_test,
    _task_Inv_Dyn_con,
    _task_Dyn_StandUp,
    _task_Dyn_Walking,
    _task_Dyn_Walking2,
    _task_HB_Jump,
    _task_HB_PrevWalk,
    _task_HB_test,
    _task_Ready_To_Walk,
    _task_SingleLog_Walk,
    _task_ROS_Walk

}_task_thread;

int main(int argc, char *argv[])
{

    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "HBWalking");


    CheckArguments(argc, argv);

    if(PODO_NO == -1)
    {
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }


    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    userData->FLAG_receivedROS = ROS_RX_FALSE;
    userData->FLAG_sendROS = CMD_BREAK;
    userData->ros_walking_cmd = ROS_ROSWALK_BREAK;
    userData->ros_footstep_flag = false;

    joint->SetMotionOwner(0);

    HBPW.sharedCMD = sharedCMD;
    HBPW.sharedREF = sharedREF;
    HBPW.sharedSEN = sharedSEN;
    HBPW.userData = userData;

    GGSW.sharedCMD = sharedCMD;
    GGSW.sharedREF = sharedREF;
    GGSW.sharedSEN = sharedSEN;
    GGSW.userData = userData;


    // WBIK Initialize
    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, joint);

    if(__IS_GAZEBO == true)
    {
        HBPW.Pos_Ankle_torque_control_flag = false;
        GGSW.Pos_Ankle_torque_control_flag = false;
    }else
    {
        HBPW.Pos_Ankle_torque_control_flag = true;
        GGSW.Pos_Ankle_torque_control_flag = true;

    }

    while(__IS_WORKING)
    {
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
        case HBWalking_WALK_TEST:
        {
//            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
//            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_ZERO;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;
            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            cout<<"Walking Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            WB_FLAG = 1;
            usleep(500*1000);

//            vec3 midfoot = vec3((WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2, (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2, (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2);

//            WBmotion->addCOMInfo(0.015, 0, WBmotion->pCOM_3x1[2]- midfoot.z,1);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]- midfoot.x, WBmotion->pRF_3x1[1]- midfoot.y, WBmotion->pRF_3x1[2]- midfoot.z,1);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]- midfoot.x, WBmotion->pLF_3x1[1]- midfoot.y, WBmotion->pLF_3x1[2]- midfoot.z,1);
//            WB_FLAG = 1;
//            usleep(2000*1000);

            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;


            vec3 COM_off = vec3(WBmotion->pCOM_3x1[0],WBmotion->pCOM_3x1[1],WBmotion->pCOM_3x1[2]);
            vec3 pRF = vec3(WBmotion->pRF_3x1[0],WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2]);
            vec3 pLF = vec3(WBmotion->pLF_3x1[0],WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            HBWalking.HB_set_step(COM_off, pRF, pLF, qRF, qLF, t_step, no_of_step, step_stride, 0.22, -1); //0.177

            cout<<"foot print & timing : total "<<HBWalking.step_data_buf.size()<<"steps "<<endl;

            for(int i=HBWalking.step_data_buf.size()-1;i>=0;i--){
                cout<<HBWalking.step_data_buf[i].x<<", "<<HBWalking.step_data_buf[i].y<<" ("<<HBWalking.step_data_buf[i].t<<"s)"<<endl;
            }

            HBWalking.Choreonoid_flag = false;

            usleep(500*1000);

            _task_thread = _task_HB_Walking;
            break;
        }
        case HBWalking_DATA_SAVE:
        {

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            save_all_gg();
            break;
        }
        case HBWalking_TORQUE_TEST:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            AnkleToruqeControl_Init();
            usleep(50*1000);
             _task_thread = _task_TorqueTest;
            break;
        }
        case HBWalking_ANKLE_TEST:
        {
            sharedCMD->Choreonoid.TorqueConOnOff = 0;

            usleep(1000*1000);

             _task_thread = _task_TorqueTest;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        }
        case HBWalking_SYSID_START:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;
            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->ResetGlobalCoord(1);
            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            Initial_COM = vec3(WBmotion->pCOM_3x1[0],WBmotion->pCOM_3x1[1],WBmotion->pCOM_3x1[2]);
            cout<<"System ID Command received..!"<<endl;

            WB_FLAG = 1;

            usleep(500*1000);

            InputFreq = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            InputAmp = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            SysIDcnt = 0;
            SysIDcnt_init = 0;
            HBWalking.k = 0;
            WB_FLAG = 1;
            _task_thread = _task_SysID;
            sharedCMD->COMMAND[4].USER_COMMAND = SE_EST_START;
            break;
        }
        case HBWalking_SYSID_STEP_INPUT_START:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;
            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->ResetGlobalCoord(1);
            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            Initial_COM = vec3(WBmotion->pCOM_3x1[0],WBmotion->pCOM_3x1[1],WBmotion->pCOM_3x1[2]);
            cout<<"System ID Command received..!"<<endl;

//            vec3 midfoot = vec3((WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2, (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2, (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2);

//            WBmotion->addCOMInfo(0, 0, WBmotion->pCOM_3x1[2]- midfoot.z,0.5);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]- midfoot.x, WBmotion->pRF_3x1[1]- midfoot.y, WBmotion->pRF_3x1[2]- midfoot.z,0.5);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]- midfoot.x, WBmotion->pLF_3x1[1]- midfoot.y, WBmotion->pLF_3x1[2]- midfoot.z,0.5);
            WB_FLAG = 1;

            usleep(500*1000);

            InputFreq = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            InputAmp = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            SysIDcnt = 0;
            SysIDcnt_init = 0;
            HBWalking.k = 0;
            WB_FLAG = 1;
            _task_thread = _task_SysID_step;
            sharedCMD->COMMAND[4].USER_COMMAND = SE_EST_START;
            break;
        }
        case HBWalking_SYSID_STOP_SAVE:
        {
            sharedCMD->COMMAND[4].USER_COMMAND = SE_EST_FINISH;
            WB_FLAG = 0;
            _task_thread = _task_Idle;
            HBWalking.save_sysID(InputFreq);
            SysIDcnt = 0;
            SysIDcnt_init = 0;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        }
        case HBWalking_CONTROL_ON:
        {
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_ZERO;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            cout<<"control on"<<endl;
            _task_thread = _task_Idle;
            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->ResetGlobalCoord(1);
            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            cout<<"Walking Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

//            vec3 midfoot = vec3((WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2, (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2, (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2);

//            WBmotion->addCOMInfo(0.015, 0, WBmotion->pCOM_3x1[2]- midfoot.z,1);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]- midfoot.x, WBmotion->pRF_3x1[1]- midfoot.y, WBmotion->pRF_3x1[2]- midfoot.z,1);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]- midfoot.x, WBmotion->pLF_3x1[1]- midfoot.y, WBmotion->pLF_3x1[2]- midfoot.z,1);
//            WB_FLAG = 1;
//            usleep(2000*1000);
//            vec3 COM_ori(WBmotion->pCOM_3x1);

            ResetGlobalCoord(1);
            usleep(1000*1000);


            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            Initial_COM = vec3(WBmotion->pCOM_3x1[0],WBmotion->pCOM_3x1[1],WBmotion->pCOM_3x1[2]);
            Initial_pRF = vec3(WBmotion->pRF_3x1);
            Initial_pLF = vec3(WBmotion->pLF_3x1);
            z_ctrl = 0;
            ctrl_COM = vec3(0,0,0);//Initial_COM;

            cout<<"COM_initial value : ("<<Initial_COM.x<<", "<<Initial_COM.y<<")"<<endl;

            WB_FLAG = 1;




            _task_thread = _task_Control_on;
//            sharedCMD->COMMAND[4].USER_COMMAND = SE_EST_START;




            break;
        }
        case HBWalking_OL_TORQUE_TUNING:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            input_Amp = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            cout<<"Ampere Input : "<<input_Amp<<"mA"<<endl;

            //MCJointPWMCommand2chHR(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, 4, input_Amp, 0, 0); //RAR
           // AnkleTorqueControl(0,input_Amp,0,0,   0,-sharedSEN->FT[0].My,0,0);
            _task_thread = _task_Ankle_Torque_con_test;

            break;
        }
        case HBWalking_ZERO_GAIN:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;

            AnkleToruqeControl_Init();
            usleep(50*1000);
            AnkleToruqeControl_Init();
            usleep(50*1000);

            _task_thread = _task_Idle;
            break;
        }
        case HBWalking_INV_DYN_CONTROL:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            cout<<"Walking Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(1000*1000);

            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            WB_FLAG = 1;

            _task_thread = _task_Inv_Dyn_con;
            break;
        }
        case HBWalking_DYN_STANDING:
        {
            cout<<"dynamic standing go"<<endl;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            joint->RefreshToCurrentPosition();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            _task_thread = _task_Idle;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            cout<<"Walking Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(1000*1000);

            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ////Initialize State Estimator--------------------------------------------------------------------
            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            RST_ini.F_RF = F_RF; RST_ini.F_LF = F_LF; RST_ini.M_RF = M_RF; RST_ini.M_LF = M_LF;

            RST_ini.cRF = false; RST_ini.cLF = false;
            if(F_RF.norm() > 20) RST_ini.cRF = true;
            if(F_LF.norm() > 20) RST_ini.cLF = true;

            RST_ini.CSP.pPel = vec3(WBmotion->pPel_3x1); RST_ini.CSP.qPel = quat(WBmotion->qPEL_4x1); RST_ini.CSP.pCOM = vec3(WBmotion->pCOM_3x1);
            RST_ini.CSP.pRF = vec3(WBmotion->pRF_3x1); RST_ini.CSP.qRF = quat(WBmotion->qRF_4x1);
            RST_ini.CSP.pLF = vec3(WBmotion->pLF_3x1); RST_ini.CSP.qLF = quat(WBmotion->qLF_4x1);

            RST_ini.CSV.dpPel = vec3(); RST_ini.CSV.dqPel = vec3(); RST_ini.CSV.dpCOM  =vec3();
            RST_ini.CSV.dpRF = vec3(); RST_ini.CSV.dqRF = vec3();
            RST_ini.CSV.dpLF = vec3(); RST_ini.CSV.dqLF = vec3();
            RST_ini.JSP.pPel = RST_ini.CSP.pPel; RST_ini.JSP.qPel = RST_ini.CSP.qPel;
            RST_ini.JSV.dpPel = vec3(); RST_ini.JSV.dqPel = vec3();

            for(int i=0; i<12 ;i++){
                RST_ini.JSP.JSP_Array[i] = getEnc(i);
                RST_ini.JSV.JSV_Array[i] = 0.0;
            }
            RST_ini.Qnow = RST_ini.getQnow(bp_rbdl.Robot->q_size);
            RST_ini.dQnow = RST_ini.getdQnow(bp_rbdl.Robot->qdot_size);

            SE.Initialize_StateEst(RST_ini);
            ////---------------------------------------------------------------------------------------------------------------

            sharedCMD->Choreonoid.TorqueConOnOff = 1;
            Scnt = 0;

            _task_thread = _task_Dyn_StandUp;
            break;
        }
        case HBWalking_DYN_WALKING:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            joint->RefreshToCurrentPosition();
            //joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            _task_thread = _task_Idle;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            cout<<"Walking Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_off = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            //HBD.set_step(COM_off, pRF, pLF, qRF, qLF, t_step, no_of_step, step_stride, 0.1178*2, -1); //0.177

            //cout<<"foot print & timing : total "<<HBD.step_data_buf.size()<<"steps "<<endl;

//            for(int i=HBD.step_data_buf.size()-1;i>=0;i--){
//                cout<<HBD.step_data_buf[i].x<<", "<<HBD.step_data_buf[i].y<<" ("<<HBD.step_data_buf[i].t<<"s)"<<endl;
//            }


            ////Initialize State Estimator--------------------------------------------------------------------
            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
            RST_ini.F_RF = F_RF; RST_ini.F_LF = F_LF; RST_ini.M_RF = M_RF; RST_ini.M_LF = M_LF;

            RST_ini.cRF = false; RST_ini.cLF = false;
            if(F_RF.norm() > 20) RST_ini.cRF = true;
            if(F_LF.norm() > 20) RST_ini.cLF = true;
            RST_ini.CSP.pPel = vec3(WBmotion->pPel_3x1); RST_ini.CSP.qPel = IMUquat; RST_ini.CSP.pCOM = vec3(WBmotion->pCOM_3x1);
            RST_ini.CSP.pRF = vec3(WBmotion->pRF_3x1); RST_ini.CSP.qRF = quat(WBmotion->qRF_4x1);
            RST_ini.CSP.pLF = vec3(WBmotion->pLF_3x1); RST_ini.CSP.qLF = quat(WBmotion->qLF_4x1);

            RST_ini.CSV.dpPel = vec3(); RST_ini.CSV.dqPel = vec3(); RST_ini.CSV.dpCOM  =vec3();
            RST_ini.CSV.dpRF = vec3(); RST_ini.CSV.dqRF = vec3();
            RST_ini.CSV.dpLF = vec3(); RST_ini.CSV.dqLF = vec3();
            RST_ini.JSP.pPel = RST_ini.CSP.pPel; RST_ini.JSP.qPel = RST_ini.CSP.qPel;
            RST_ini.JSV.dpPel = vec3(); RST_ini.JSV.dqPel = vec3();

            for(int i=0; i<12 ;i++){
                RST_ini.JSP.JSP_Array[i] = getEnc(i);
                RST_ini.JSV.JSV_Array[i] = 0.0;
            }
            RST_ini.Qnow = RST_ini.getQnow(bp_rbdl.Robot->q_size);
            RST_ini.dQnow = RST_ini.getdQnow(bp_rbdl.Robot->qdot_size);

            SE.Initialize_StateEst(RST_ini);
            ////---------------------------------------------------------------------------------------------------------------

            sharedCMD->Choreonoid.TorqueConOnOff = 1;

            _task_thread = _task_Dyn_Walking;
            break;
        }
        case HBWalking_DYN_WALKING2:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            joint->RefreshToCurrentPosition();
            //joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            _task_thread = _task_Idle;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();
            cout<<"Walking Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_off = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            //HBD2.set_step(COM_off, pRF, pLF, qRF, qLF, 0.1178*2, -1); //0.177



            ////Initialize State Estimator--------------------------------------------------------------------
            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
            RST_ini.F_RF = F_RF; RST_ini.F_LF = F_LF; RST_ini.M_RF = M_RF; RST_ini.M_LF = M_LF;

            RST_ini.cRF = false; RST_ini.cLF = false;
            if(F_RF.norm() > 20) RST_ini.cRF = true;
            if(F_LF.norm() > 20) RST_ini.cLF = true;
            RST_ini.CSP.pPel = vec3(WBmotion->pPel_3x1); RST_ini.CSP.qPel = IMUquat; RST_ini.CSP.pCOM = vec3(WBmotion->pCOM_3x1);
            RST_ini.CSP.pRF = vec3(WBmotion->pRF_3x1); RST_ini.CSP.qRF = quat(WBmotion->qRF_4x1);
            RST_ini.CSP.pLF = vec3(WBmotion->pLF_3x1); RST_ini.CSP.qLF = quat(WBmotion->qLF_4x1);

            RST_ini.CSV.dpPel = vec3(); RST_ini.CSV.dqPel = vec3(); RST_ini.CSV.dpCOM  =vec3();
            RST_ini.CSV.dpRF = vec3(); RST_ini.CSV.dqRF = vec3();
            RST_ini.CSV.dpLF = vec3(); RST_ini.CSV.dqLF = vec3();
            RST_ini.JSP.pPel = RST_ini.CSP.pPel; RST_ini.JSP.qPel = RST_ini.CSP.qPel;
            RST_ini.JSV.dpPel = vec3(); RST_ini.JSV.dqPel = vec3();

            for(int i=0; i<12 ;i++){
                RST_ini.JSP.JSP_Array[i] = getEnc(i);
                RST_ini.JSV.JSV_Array[i] = 0.0;
            }
            RST_ini.Qnow = RST_ini.getQnow(bp_rbdl.Robot->q_size);
            RST_ini.dQnow = RST_ini.getdQnow(bp_rbdl.Robot->qdot_size);

            SE.Initialize_StateEst(RST_ini);
            ////---------------------------------------------------------------------------------------------------------------

            sharedCMD->Choreonoid.TorqueConOnOff = 1;

            _task_thread = _task_Dyn_Walking2;
            break;
        }
        case HBWalking_JUMP:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;

            _task_thread = _task_Idle;
            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();

            cout<<"Jump Command received..!"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            WB_FLAG = 1;
            usleep(500*1000);

            cout<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;


            vec3 COM_off = vec3(WBmotion->pCOM_3x1[0],WBmotion->pCOM_3x1[1],WBmotion->pCOM_3x1[2]);
            vec3 pRF = vec3(WBmotion->pRF_3x1[0],WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2]);
            vec3 pLF = vec3(WBmotion->pLF_3x1[0],WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            HBJumping.HB_set_jump(COM_off);

            usleep(3000*1000);
            Scnt = 0;

            _task_thread = _task_HB_Jump;

            break;

        }

        case HBWalking_STOP:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;
//            joint->RefreshToCurrentPosition();
//            //joint->RefreshToCurrentReference();
//            joint->SetAllMotionOwner();

//            sharedCMD->Choreonoid.TorqueConOnOff = 0;

            WB_FLAG = 0;
            save_all();




            break;
        }

        case HBWalking_GetComHeight:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            cout<<"COM Height Check!"<<endl;

            _task_thread = _task_Idle;
            WBmotion->MomentFlag = false;
            WB_FLAG = 0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference();

            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            double total_mass = Hi.m_pel + Hi.m_rleg + Hi.m_lleg;
            cout<<"total mass : "<<total_mass<<endl;

            break;

        }
        case HBWalking_Test:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            cout<<"Logging start!"<<endl;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB();

            cout<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);



            HBPW.Test_init(COM_ini, pRF, qRF, pLF, qLF);

            init_StateEstimator(); // State Estimator Initialization



            _task_thread = _task_HB_test;
            WB_FLAG = 1;

            break;

        }
        case HBWalking_PrevWalk:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

//            joint->RefreshToCurrentPosition();
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB();

            cout<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);

            quat qPel_ini = quat(WBmotion->qPEL_4x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            // Waste angle initialize
            WST_ref_deg = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition;

            // preview Gain load
            HBPW.PreviewGainLoad(HBPW.zc);

            HBPW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

            cout<<"foot print & timing : total "<<HBPW.SDB.size()<<"steps "<<endl;

            for(int i=HBPW.SDB.size()-1;i>=0;i--){
                cout<<"phase: "<<i<<"  "<<HBPW.SDB[i].Fpos.x<<", "<<HBPW.SDB[i].Fpos.y<<" ("<<HBPW.SDB[i].t<<"s)"<<endl;
            }

            init_StateEstimator(); // State Estimator Initialization

            WB_FLAG = 1;


            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[15] == 10){
                HBPW.Joystick_walk_flag = true;
                HBPW.Joystick_on_signal = true;

                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3] = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5] = 0;
            }

            _task_thread = _task_HB_PrevWalk;

            break;
        }
        case HBWalking_Ready_To_Walk:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            cout<<"Logging start!"<<endl;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            RSE.HSE_ESTIMATOR_ONOFF(true,0);
            cout<<"State Estimation Start"<<endl;

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB();

            cout<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);

            quat qPel_ini = quat(WBmotion->qPEL_4x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            COM_ini_global = COM_ini;
            pRF_ini_global = pRF;
            pLF_ini_global = pLF;

            int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            HBPW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

            // Waste angle initialize
            WST_ref_deg = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition;

            // preview Gain load
            HBPW.PreviewGainLoad(HBPW.zc);

            init_StateEstimator(); // State Estimator Initialization

            WB_FLAG = 1;

            _task_thread = _task_Ready_To_Walk;

            break;
        }
        case HBWalking_SingleLog:
        {
            int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            FILE_LOG(logSUCCESS) << "--------------------------------------------------";
            FILE_LOG(logSUCCESS) << "      New CMD : SingleLog Walking";
            FILE_LOG(logSUCCESS) << "   " << step_stride  << "m, " << t_step << "s, " << no_of_step << "step" << endl;
            FILE_LOG(logSUCCESS) << "--------------------------------------------------";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB();

            cout<<endl<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);

            quat qPel_ini = quat(WBmotion->qPEL_4x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            // Waist angle initialize
            WST_ref_deg = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition;

            // preview Gain load
            GGSW.PreviewGainLoad(GGSW.zc);
            GGSW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

            cout<<"foot print & timing : total "<<GGSW.SDB.size()<<"steps "<<endl;

            for(int i=GGSW.SDB.size()-1;i>=0;i--)
            {
                cout<<"phase: "<<i<<"  "<<GGSW.SDB[i].Fpos.x<<", "<<GGSW.SDB[i].Fpos.y<<" ("<<GGSW.SDB[i].t<<"s)"<<endl;
            }

            init_StateEstimator(); // State Estimator Initialization
            WB_FLAG = 1;

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10] == 1)
            {
                FILE_LOG(logSUCCESS) << "ROS Walk Start\n";
                GGSW.ROSWalk_flag = true;
                userData->M2G.ROSWalk_state = 1;

                for(int i=0;i<4;i++)
                {
                    userData->ros_footsteps[i].x = 0.;
                    userData->ros_footsteps[i].y = 0.;
                    userData->ros_footsteps[i].r = 0.;
                    userData->ros_footsteps[i].step_phase = 0;
                    userData->ros_footsteps[i].lr_state = 0;
                }
            }
            _task_thread = _task_SingleLog_Walk;
            break;
        }
        case HBWalking_ROSWalk:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10] == 0)
            {
                FILE_LOG(logSUCCESS) << "ROS Walk Stop\n";
                GGSW.ROSWalk_off_flag = true;
                userData->M2G.ROSWalk_state = 0;
            }else
            {
                int no_of_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                double t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double step_stride = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

                FILE_LOG(logSUCCESS) << "--------------------------------------------------";
                FILE_LOG(logSUCCESS) << "      New CMD : ROS Walking";
                FILE_LOG(logSUCCESS) << "   " << step_stride  << "m, " << t_step << "s, " << no_of_step << "step" << endl;
                FILE_LOG(logSUCCESS) << "--------------------------------------------------";

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
                _task_thread = _task_Idle;

                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                WBmotion->MomentFlag = false;
                WB_FLAG = 0;

                WBmotion->StopAll();
                WBmotion->RefreshToCurrentReference_HB();

                cout<<endl<<"Initial Task position: "<<endl;
                cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
                cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
                cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
                cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl<<endl;

                ResetGlobalCoord(1);
                usleep(200*1000);

                cout<<"After ResetGlobal"<<endl;
                cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
                cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
                cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
                cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl<<endl;

                vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
                vec3 pRF = vec3(WBmotion->pRF_3x1);
                vec3 pLF = vec3(WBmotion->pLF_3x1);

                quat qPel_ini = quat(WBmotion->qPEL_4x1);
                quat qRF = quat(WBmotion->qRF_4x1);
                quat qLF = quat(WBmotion->qLF_4x1);

                // Waist angle initialize
                WST_ref_deg = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition;

                // preview Gain load
                GGSW.Set_walkingmode(0);
                GGSW.PreviewGainLoad(GGSW.zc);
                GGSW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

                cout<<"foot print & timing : total "<<GGSW.SDB.size()<<"steps "<<endl;

                for(int i=GGSW.SDB.size()-1;i>=0;i--)
                {
                    cout<<"phase: "<<i<<"  "<<GGSW.SDB[i].Fpos.x<<", "<<GGSW.SDB[i].Fpos.y<<" ("<<GGSW.SDB[i].t<<"s)"<<endl;
                }

                init_StateEstimator(); // State Estimator Initialization
                WB_FLAG = 1;

                //if(userData->ros_walking_cmd == ROSWALK_START)
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10] == 1)
                {
                    FILE_LOG(logSUCCESS) << "ROS Walk Start\n";
                    GGSW.ROSWalk_flag = true;
                    userData->M2G.ROSWalk_state = 1;

                    for(int i=0;i<4;i++)
                    {
                        userData->ros_footsteps[i].x = 0.;
                        userData->ros_footsteps[i].y = 0.;
                        userData->ros_footsteps[i].r = 0.;
                        userData->ros_footsteps[i].step_phase = 0;
                        userData->ros_footsteps[i].lr_state = 0;
                    }
                }
                _task_thread = _task_ROS_Walk;
                break;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        }
        case HBWalking_JoyStick_Walk_Stop:
        {
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            HBPW.Joystick_off_signal = true;

            break;
        }

        case 999:
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            joint->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        }

        switch(userData->ros_walking_cmd)
        {
        case ROS_ROSWALK_NORMAL_START:
        {
            int no_of_step = 10;
            double t_step = 0.8;
            double step_stride = 0.;

            FILE_LOG(logSUCCESS) << "--------------------------------------------------";
            FILE_LOG(logSUCCESS) << "      New CMD : ROS Walking Start";
            FILE_LOG(logSUCCESS) << "--------------------------------------------------";

            _task_thread = _task_Idle;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB();

            cout<<endl<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);

            quat qPel_ini = quat(WBmotion->qPEL_4x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            // Waist angle initialize
            WST_ref_deg = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition;

            // preview Gain load
//            GGSW.Set_walkingmode(0);
            GGSW.PreviewGainLoad(GGSW.zc);
            GGSW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

            cout<<"foot print & timing : total "<<GGSW.SDB.size()<<"steps "<<endl;

            for(int i=GGSW.SDB.size()-1;i>=0;i--)
            {
                cout<<"phase: "<<i<<"  "<<GGSW.SDB[i].Fpos.x<<", "<<GGSW.SDB[i].Fpos.y<<" ("<<GGSW.SDB[i].t<<"s)"<<endl;
            }

            init_StateEstimator(); // State Estimator Initialization
            WB_FLAG = 1;
            userData->FLAG_sendROS = CMD_ACCEPT;
            userData->FLAG_receivedROS = ROS_RX_EMPTY;
            printf("receive empty\n");


            FILE_LOG(logSUCCESS) << "ROS Walk Start\n";
            GGSW.ROSWalk_flag =true;
            userData->M2G.ROSWalk_state = 1;

            for(int i=0;i<4;i++)
            {
                userData->ros_footsteps[i].x = 0.;
                userData->ros_footsteps[i].y = 0.;
                userData->ros_footsteps[i].r = 0.;
                userData->ros_footsteps[i].step_phase = 0;
                userData->ros_footsteps[i].lr_state = 0;
            }
//            _task_thread = _task_ROS_Walk;
            _task_thread = _task_SingleLog_Walk;

            userData->ros_walking_cmd = ROS_ROSWALK_BREAK;
            break;
        }
        case ROS_ROSWALK_STOP:
        {
            FILE_LOG(logSUCCESS) << "ROS Walk Stop\n";
            GGSW.ROSWalk_off_flag = true;
            userData->M2G.ROSWalk_state = 0;
            userData->ros_walking_cmd = ROS_ROSWALK_BREAK;
            break;
        }
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



////========================================================================================================================////
//// Task Thread
////========================================================================================================================//
////========================================================================================================================//

doubles dbs_qRF(4), dbs_qLF(4), dbs_qPel(4);
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        if(RSE.HSE_ONOFF)
        {
            RSE.READ_DATA_FROM_SENSOR();
            RSE.HSE_ESTIMATOR();

            // output
            pel_estimated.x = RSE.OUTPUT.FK_PELVIS_POSITION[0];
            pel_estimated.y = RSE.OUTPUT.FK_PELVIS_POSITION[1];
            pel_estimated.z = RSE.OUTPUT.FK_PELVIS_POSITION[2];

        }

        switch(_task_thread)
        {
        case _task_Ready_To_Walk:
        {
            SensorInput();
            RST = SE.StateEst(RSEN);
            HBPW.MeasurementInput(RST);

            HBPW.k++;
            save_onestep(HBPW.k);


            //// -----------------------COM Damping Control
            HBPW.COM_ref = COM_ini_global;

            HBPW.DampingControl3();

            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_damping_con;//HBPW.COM_ref;

//            if(HBPW.k % 10 == 0) cout<<"uCOMx: "<<HBPW.uCOM.x<<" uCOMy: "<<HBPW.uCOM.y<<endl;

//            cout<<"uCOMy: "<<HBPW.uCOM.y<<"  ZMPy : "<<HBPW.ZMP_global.y<<endl;

            HBPW.Standing_mode_flag = true;

            //// ---------------------- DSP_FZ control
            HBPW.t_now = 0;
            HBPW.RF_landing_flag = true;
            HBPW.LF_landing_flag = true;
            HBPW.step_phase = 0;

            HBPW.cZMP_dsp_global = vec3(0,0,0) + (1 + 4.0/HBPW.w)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w));

            HBPW.cZMP_dsp_global_proj = zmpProjectionToSP_offset(HBPW.cZMP_dsp_global, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, 100, 100, -0.002, -0.002);

            HBPW.cZMP_dsp_local_proj = global2local_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_dsp_global_proj);  //-> not using

            HBPW.DSP_Fz_controller(0,HBPW.RS.F_RF.z, HBPW.RS.F_LF.z);

            HBPW.pRF_ref.z = pRF_ini_global.z + 0.5*HBPW.z_ctrl;
            HBPW.pLF_ref.z = pLF_ini_global.z - 0.5*HBPW.z_ctrl;


            //// --------------------- Ankle Torque Control
            HBPW.step_phase = 0;
            HBPW.Pos_Ankle_torque_control_flag = true;
            HBPW.Standing_mode_flag = false;

//            vec3 temp_cZMP_TC_local = (1 + 10.0/w)*(HBPW.COM_e_imu_local + 1.0*HBPW.dCOM_e_imu_local/HBPW.w);

//            HBPW.cZMP_TC_local.x = temp_cZMP_TC_local.x - 0.000;
//            HBPW.cZMP_TC_local.y = temp_cZMP_TC_local.y;

//            HBPW.cZMP_TC_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref,HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_TC_local);

//            HBPW.cZMP_TC_proj = zmpProjectionToSP_offset(HBPW.cZMP_TC_global, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, +0.00);

            HBPW.cZMP_TC_local = (1 + 4/HBPW.w)*((HBPW.COM_m_filtered + 1.5*HBPW.dCOM_m_diff_filtered/HBPW.w));

            HBPW.cZMP_TC_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_TC_local);


            HBPW.cZMP_TC_proj =  zmpProjectionToSP_offset(HBPW.cZMP_TC_local, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, -0.002, +0.02);



            HBPW.AnkleTorque_ref = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_TC_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);


            HBPW.AnkleTorqueController_pos(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

            HBPW.Standing_mode_flag = false;


            //// ---------------- CP error & Walking Trigger

            vec3 CP_error = HBPW.COM_m_filtered + 1.4*HBPW.dCOM_m_diff_filtered/HBPW.w;

            if(HBPW.k % 10 == 0) cout<<"CP_error_x: "<<CP_error.x<<" CP_error_y: "<<CP_error.y<<endl;

            if(fabs(CP_error.x) > 0.06 && HBPW.k > 1000) _task_thread = _task_HB_PrevWalk;
            else _task_thread = _task_Ready_To_Walk;



            //// ------------------- Reference Trajectory

            WBmotion->addCOMInfo_xy_pelz_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pPel_3x1[2]);
            WBmotion->addRFPosInfo_HB(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z);
            WBmotion->addLFPosInfo_HB(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z);

            WBmotion->addRFOriInfo_HB(HBPW.qRF_ref);
            WBmotion->addLFOriInfo_HB(HBPW.qLF_ref);


            break;

        }
        case _task_HB_PrevWalk:
        {
//            RTIME begin, end;
//            begin = rt_timer_read();

            userData->M2G.valveMode = 1;

            //// if JoyStick on, calc del_pos from Joystick input
            if(HBPW.Joystick_walk_flag == true)
            {
                int JOY_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                int JOY_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                int JOY_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
                int JOY_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
                int JOY_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
                int JOY_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
                double des_t_step = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[15];

                //cout<<"Ljog RL : "<<JOY_LJOG_RL<<endl;

                HBPW.Calc_del_pos_from_Joystick(JOY_RJOG_RL, JOY_RJOG_UD, JOY_LJOG_RL, JOY_LJOG_UD, des_t_step);

                if(JOY_RB == 1 && JOY_LB == 1){ // Walking Emergency Stop
                    _task_thread = _task_Idle;
                    save_all();
                }
            }

            userData->M2G.valveMode = 2;


            //// Put sensor value into RSEN object
            SensorInput();

            userData->M2G.valveMode = 3;

            //// State Estimation
            RST = SE.StateEst(RSEN);
            HBPW.MeasurementInput(RST);

            userData->M2G.valveMode = 4;

            /// Calc Origival leg joint for kinematics update during Ankle Torque Control
            //HBPW.LJ_ref = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);

            //// Main Walking code
            if(HBPW.Preveiw_walking() == -1){
                _task_thread = _task_Idle;
                //WB_FLAG = 0;
                save_all();
                //HBPW.save_WD();

                cout<<"Preview Walk finished"<<endl;
                //cout<<RHR_con_deg<<", "<<LHR_con_deg<<", "<<RHY_con_deg<<", "<<LHY_con_deg<<", "<<LHP_con_deg<<", "<<RHP_con_deg<<endl;
            }

            //// disturbance command (Choreonoid)
//            if(HBPW.step_phase == 2 && (HBPW.t_now >= 0.65 && HBPW.t_now < 0.655)){
//                sharedCMD->COMMAND[MAX_AL-1].USER_COMMAND = 222;
//                sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[0] = 25;
//                sharedCMD->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[1] = 570;
//                // 330 : Ankle
//                // 400 : Ankle + hip
//                // 570 : Ankle + hip + stepping
//            }



            userData->M2G.valveMode = 5;

            vec3 COM_total = HBPW.uCOM;


            //// Leg Vibration Control
            LHY_con_deg = -HBPW.LHY_con_deg;
            RHY_con_deg = -HBPW.RHY_con_deg;

            LHR_con_deg = 1.0*HBPW.LHR_con_deg + HBPW.L_roll_compen_deg;
            RHR_con_deg = 1.0*HBPW.RHR_con_deg + HBPW.R_roll_compen_deg;

//            LHR_con_deg =  HBPW.L_roll_compen_deg;
//            RHR_con_deg =  HBPW.R_roll_compen_deg;

            LHP_con_deg =  1.0*HBPW.LHP_con_deg*0.5;
            RHP_con_deg =  1.0*HBPW.RHP_con_deg*0.5;
            LKN_con_deg = HBPW.L_knee_compen_deg;
            RKN_con_deg = HBPW.R_knee_compen_deg;

            //cout<<"LHR_con_deg :"<<HBPW.LHR_con_deg<<"  RHR_con_deg: "<<HBPW.RHR_con_deg<<endl;

            ////Foot and Pelv Orientation
            //foot Orientation

            for(int i=0;i<4;i++){
                dbs_qRF[i] = HBPW.qRF_ref[i];
                dbs_qLF[i] = HBPW.qLF_ref[i];
            }

            // Pelvis Orientation
            for(int i=0;i<4;i++){
                dbs_qPel[i] = HBPW.qPel_ref[i];
            }

            userData->M2G.valveMode = 6;

            //// Put reference Task to Trajectory Handler
//            WBmotion->addCOMInfo(COM_total.x, COM_total.y, 0.62 + HBPW.del_COMz_con, 0.005);
//            WBmotion->addRFPosInfo(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z,0.005);
//            WBmotion->addLFPosInfo(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z,0.005);
//            WBmotion->addRFOriInfo(dbs_qRF,0.005);
//            WBmotion->addLFOriInfo(dbs_qLF,0.005);
//            WBmotion->addPELOriInfo(dbs_qPel,0.005);

            //WBmotion->addCOMInfo_HB(COM_total.x, COM_total.y, 0.62 + HBPW.del_COMz_con);
            WBmotion->addCOMInfo_xy_pelz_HB(COM_total.x, COM_total.y, WBmotion->pPel_3x1[2]);
            WBmotion->addRFPosInfo_HB(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z);
            WBmotion->addLFPosInfo_HB(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z);
            WBmotion->addRFOriInfo_HB(HBPW.qRF_ref);
            WBmotion->addLFOriInfo_HB(HBPW.qLF_ref);
            WBmotion->addPELOriInfo_HB(HBPW.qPel_ref);

            userData->M2G.valveMode = 7;

            WST_ref_deg = HBPW.WST_ref_deg;

            save_onestep(HBPW.k);

//            end = rt_timer_read();
//            cout<<"time : "<<(long)(end - begin)/100000.0<<endl; // 0.1ms unit

            userData->M2G.valveMode = 8;

            break;
        }
        case _task_HB_test:
        {
            SensorInput();
            RST = SE.StateEst(RSEN);
            HBPW.MeasurementInput(RST);

            //// -----------------------system id for damping control
            HBPW.k++;
            save_onestep(HBPW.k);


            ////-----------------------------temp
//            if(HBPW.k % 500 == 0){
//                cout<<"11sec"<<endl;
//                cout<<"COM : "<<WBmotion->des_pCOM_3x1_HB[0]<<", "<<WBmotion->des_pCOM_3x1_HB[1]<<", "<<WBmotion->des_pCOM_3x1_HB[2]<<endl;
//                cout<<"pPEL : "<<WBmotion->des_pPELz_HB<<endl;
//                cout<<"pRF : "<<WBmotion->des_pRF_3x1_HB[0]<<", "<<WBmotion->des_pRF_3x1_HB[1]<<", "<<WBmotion->des_pRF_3x1_HB[2]<<endl;
//                cout<<"pLF : "<<WBmotion->des_pLF_3x1_HB[0]<<", "<<WBmotion->des_pLF_3x1_HB[1]<<", "<<WBmotion->des_pLF_3x1_HB[2]<<endl;
//                cout<<"qPEL : "<<WBmotion->des_qPEL_4x1_HB[0]<<", "<<WBmotion->des_qPEL_4x1_HB[1]<<", "<<WBmotion->des_qPEL_4x1_HB[2]<<endl;
//                cout<<"qRF : "<<WBmotion->des_qRF_4x1_HB[0]<<", "<<WBmotion->des_qRF_4x1_HB[1]<<", "<<WBmotion->des_qRF_4x1_HB[2]<<endl;
//                cout<<"qLF : "<<WBmotion->des_qLF_4x1_HB[0]<<", "<<WBmotion->des_qLF_4x1_HB[1]<<", "<<WBmotion->des_qLF_4x1_HB[2]<<endl;
//                cout<<endl;


//            }

//            WBmotion->addCOMInfo_xy_pelz_HB(0, 0, WBmotion->pPel_3x1[2]);
//            WBmotion->addRFPosInfo_HB(0, WBmotion->pRF_3x1[1], 0);
//            WBmotion->addLFPosInfo_HB(0, 0.1091, 0);
//            WBmotion->addRFOriInfo_HB(quat());
//            WBmotion->addLFOriInfo_HB(quat());
//            WBmotion->addPELOriInfo_HB(quat());

//            HBPW.k++;


            //// -----------------------COM Damping Control
////            HBPW.p_ref[0] = vec3(0, 0.0,0);
//            HBPW.COM_ref = vec3(0, 0.0,0);

//            HBPW.DampingControl3();
//            //HBPW.ZMP_Tracking_controller();



//            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_damping_con;//HBPW.COM_ref;
////            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_zmp_con;//HBPW.COM_ref;

////            cout<<"uCOMy: "<<HBPW.uCOM.y<<endl;
//            cout<<"uCOMy: "<<HBPW.uCOM.y<<"  ZMPy : "<<HBPW.ZMP_global.y<<endl;


//            //WBmotion->addCOMInfo_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pCOM_3x1[2]);
//            WBmotion->addCOMInfo_xy_pelz_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pPel_3x1[2]);
//            //WBmotion->addCOMInfo_xy_pelz_HB(WBmotion->pCOM_3x1[0], WBmotion->pCOM_3x1[1], WBmotion->pPel_3x1[2]);

            //// ----------------------Standing Test



//            // using only imu
////            HBPW.cZMP_dsp_local = (1 + 2/HBPW.w)*(HBPW.COM_e_imu_local + 1.0*HBPW.dCOM_e_imu_local/HBPW.w);
////            HBPW.cZMP_dsp_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_dsp_local);

//            // using imu + damping con input
//            double k_gain = 6;
//            HBPW.cZMP_dsp_global = (1 + 1.5/HBPW.w)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w));


//            HBPW.cZMP_dsp_global_proj = zmpProjectionToSP_offset(HBPW.cZMP_dsp_global, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, 100, 100, -0.005, -0.005);

//            HBPW.cZMP_dsp_local_proj = global2local_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_dsp_global_proj);

//            //HBPW.DSP_Fz_controller2(0);
//            HBPW.DSP_Fz_controller(0,HBPW.RS.F_RF.z, HBPW.RS.F_LF.z);

//            double RFz = HBPW.pRF_ref.z + 0.5*HBPW.z_ctrl;
//            double LFz = HBPW.pLF_ref.z - 0.5*HBPW.z_ctrl;

//            //cout<<"z ctrl: "<<HBPW.z_ctrl<<endl;
//            //cout<<"cZMP_dsp_local_proj.y : "<<HBPW.cZMP_dsp_local_proj.y<<endl;
//            //cout<<"Alpha dsp : "<<HBPW.Alpha_dsp<<"z_ctrl : "<<HBPW.z_ctrl<<endl;

//            // Ankle Torque
//            //HBPW.cZMP_TC_local = (1 + 6/HBPW.w)*(1.1*HBPW.COM_e_imu_local + 1*HBPW.dCOM_e_imu_local/HBPW.w);

//            //
//            HBPW.cZMP_TC_local = (1 + 4/HBPW.w)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w));

//            HBPW.cZMP_TC_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_TC_local);


//            HBPW.cZMP_TC_proj =  zmpProjectionToSP_offset(HBPW.cZMP_TC_local, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, -0.002, +0.02);

//            HBPW.AnkleTorque_ref = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_TC_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

//            HBPW.AnkleTorqueController_pos(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
//                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

//            HBPW.qRF_ref = quat()*HBPW.RF_quat_ctrl;
//            HBPW.qLF_ref = quat()*HBPW.LF_quat_ctrl;



//            // Damping Control
//            HBPW.DampingControl3();
//            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_damping_con;// + HBPW.COM_con;//HBPW.COM_ref;

//            //cout<<"com_con : "<<HBPW.COM_con.y<<endl;


//            WBmotion->addCOMInfo_xy_pelz_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pPel_3x1[2]);
//            WBmotion->addRFPosInfo_HB(HBPW.pRF_ref.x, HBPW.pRF_ref.y, RFz);
//            WBmotion->addLFPosInfo_HB(HBPW.pLF_ref.x, HBPW.pLF_ref.y, LFz);

//            WBmotion->addRFOriInfo_HB(HBPW.qRF_ref);
//            WBmotion->addLFOriInfo_HB(HBPW.qLF_ref);

//            save_onestep(HBPW.k);
//            HBPW.k++;


            ////------------------------- Left Swing Leg Vibration Control


//            LHR_con_deg = HBPW.LSwingLeg_Vib_Control(HBPW.ACC_LF_filtered, false);

//            if(LHR_con_deg > 0.3) LHR_con_deg = 0.3;
//            if(LHR_con_deg < -0.3) LHR_con_deg = -0.3;

////            LHR_con_deg = HBPW.LSwingLeg_Vib_Control2(HBPW.ACC_LF_filtered, HBPW.RS.IMULocalW.x, false);

//            //LHP_con_deg = 1.2*HBPW.LSwingLeg_Pitch_Vib_Control(HBPW.ACC_LF_filtered, false);

//            cout<<"ACC_LFy_bpf: "<<HBPW.ACC_LF_filtered.y<<"   con deg : "<<LHR_con_deg<<endl;
//            cout<<"torso _rollvel: "<<HBPW.RS.IMULocalW.x<<endl;

//            //RHR_con_deg = HBPW.LTorso_Roll_Vib_Control(HBPW.RS.IMULocalW, false);

//            RHY_con_deg = HBPW.LTorso_Yaw_Vib_Control(HBPW.RS.IMULocalW, false);
//            cout<<"IMU yaw vel: "<<HBPW.RS.IMULocalW.z<<"   con deg : "<<RHY_con_deg<<endl;

            //// -------------------------Right Swing Leg Vibration Control
            RHR_con_deg = HBPW.RSwingLeg_Vib_Control(HBPW.ACC_RF_filtered, false);
            //RHP_con_deg = 1*HBPW.RSwingLeg_Pitch_Vib_Control(HBPW.ACC_RF_filtered);

            //LHR_con_deg = HBPW.RTorso_Roll_Vib_Control(HBPW.RS.IMULocalW);

            LHY_con_deg = HBPW.RTorso_Yaw_Vib_Control(HBPW.RS.IMULocalW, false);

            cout<<"ACC_RFy_bpf: "<<HBPW.ACC_RF_filtered.y<<"   con deg : "<<RHR_con_deg<<endl;


            //// ----------------Ankle Torque Control (by position)
//            HBPW.R_SSP_ZMP_Control(vec3(),vec3(),RSEN.F_RF, RSEN.M_RF);
//            HBPW.L_SSP_ZMP_Control(vec3(),vec3(),RSEN.F_LF, RSEN.M_LF);
//            cout<<"angle x: "<<HBPW.LF_del_angle.x*R2D<<endl;

//            quat total_qRF = HBPW.RF_del_quat;
//            quat total_qLF = HBPW.LF_del_quat;
//            for(int i=0;i<4;i++){
//                WBmotion->des_qRF_4x1[i] = total_qRF[i];
//                WBmotion->des_qLF_4x1[i] = total_qLF[i];
//            }

            //// -------------------Arial Phase Ankle Torque Control (by pwm)
/*            LegJoints LJ_original = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);

            vec4 AnkleToque =  HBPW.Ankle_Torque_from_PositionFB(2, HBPW.RS.F_RF, HBPW.RS.F_LF,
                                          LJ_original.RAP*R2D, LJ_original.RAR*R2D, LJ_original.LAP*R2D, LJ_original.LAR*R2D,

                                          HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                          0, 0, 0, 0,
                                          HBPW.RS.JSV.dRAP*R2D, HBPW.RS.JSV.dRAR*R2D, HBPW.RS.JSV.dLAP*R2D, HBPW.RS.JSV.dLAR*R2D);

//            vec4 AnkleToque = HBPW.Ankle_Torque_from_cZMP(vec3(0.1, 0, 0), vec3(0,0,0), HBPW.qRF_ref, HBPW.qLF_ref,
//                                                        vec3(WBmotion->pRF_3x1), vec3(WBmotion->pLF_3x1), vec3(0,0,100), vec3(0,0,100));

            double Tx_ref, Ty_ref;

            Tx_ref = 5;
            Ty_ref = 0;
            HBPW.AnkleTorqueController(-AnkleToque[0], -AnkleToque[1], -AnkleToque[2], -AnkleToque[3],
                                   HBPW.RS.JSP.RA1*R2D, HBPW.RS.JSP.RA2*R2D, HBPW.RS.JSP.LA1*R2D, HBPW.RS.JSP.LA2*R2D,
                                   HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                   HBPW.RS.JSV.dRA1*R2D, HBPW.RS.JSV.dRA2*R2D, HBPW.RS.JSV.dLA1*R2D, HBPW.RS.JSV.dLA2*R2D,
                                   HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);



            HBPW.AnkleQuatCalculaltor(HBPW.RS.JSP.RAP*R2D - LJ_original.RAP*R2D, HBPW.RS.JSP.RAR*R2D - LJ_original.RAR*R2D,
                                      HBPW.RS.JSP.LAP*R2D - LJ_original.LAP*R2D, HBPW.RS.JSP.LAR*R2D - LJ_original.LAR*R2D);

            quat total_qRF = HBPW.qRF_ref*HBPW.RF_del_quat;
            quat total_qLF = HBPW.qLF_ref*HBPW.LF_del_quat;
            doubles dbs_qRF(4), dbs_qLF(4);
            for(int i=0;i<4;i++){
                dbs_qRF[i] = total_qRF[i];
                dbs_qLF[i] = total_qLF[i];

            }

            WBmotion->addRFOriInfo(dbs_qRF,0.005);
            WBmotion->addLFOriInfo(dbs_qLF,0.005);



//            cout<<"ref Tx: "<<Tx_ref<<", Tx: "<<HBPW.RS.M_RF.x<<endl;
            cout<<"T_RAR: "<<AnkleToque[0]<<" T_RAP: "<<AnkleToque[1]<<" T_LAR: "<<AnkleToque[2]<<" T_LAP: "<<AnkleToque[3]<<endl;
//            cout<<"E_RAR: "<<HBPW.RS.JSP.RAP*R2D - WBmotion->LJ.RAP*R2D<<" E_RAP: "<<HBPW.RS.JSP.RAR*R2D - WBmotion->LJ.RAR*R2D
//               <<" E_LAR: "<<HBPW.RS.JSP.LAP*R2D - WBmotion->LJ.LAP*R2D<<" E_LAP: "<<HBPW.RS.JSP.LAR*R2D - WBmotion->LJ.LAR*R2D<<endl;
//            cout<<"q_RF: "<<total_qRF.w<<", "<<total_qRF.x<<", "<<total_qRF.y<<", "<<total_qRF.z<<" ::: "
//                <<"q_LF: "<<total_qLF.w<<", "<<total_qLF.x<<", "<<total_qLF.y<<", "<<total_qLF.z<<endl;
//            cout<<"E_RAP: "<<HBPW.RS.JSP.RAP*R2D - LJ_original.RAP*R2D<<" E_RAR: "<<HBPW.RS.JSP.RAR*R2D - LJ_original.RAR*R2D
//               <<" E_LAP: "<<HBPW.RS.JSP.LAP*R2D - LJ_original.LAP*R2D<<" E_LAR: "<<HBPW.RS.JSP.LAR*R2D - LJ_original.LAR*R2D<<endl;
*/
            ////----------------------- Standing Test
/*           // original LJ ref
            LegJoints LJ_original = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);

            //cZMP generation
            double dt_gain1 = 25;
            vec3 CP_ref = HBPW.COM_ref;
            //HBPW.cZMP = 1/(1 - exp(HBPW.w*HBPW.dt*dt_gain1))*CP_ref - exp(HBPW.w*HBPW.dt*dt_gain1)/(1 - exp(HBPW.w*HBPW.dt*dt_gain1))*HBPW.CP_m;// - CP_ref + p_ref[0];

            //cZMP from DCM Tracking Controller
            double DCM_gain = 8;
            HBPW.cZMP = HBPW.COM_ref + (1 + DCM_gain/HBPW.w)*(HBPW.CP_m - (HBPW.COM_ref + HBPW.dCOM_ref/HBPW.w));

            HBPW.cZMP_proj = zmpProjectionToSP_offset(HBPW.cZMP, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, 0.00);

            vec4 cZMP_FF_torque = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

            vec4 Angle_FB_torque =  HBPW.Ankle_Torque_from_PositionFB(2, HBPW.RS.F_RF, HBPW.RS.F_LF,
                                          LJ_original.RAP*R2D, LJ_original.RAR*R2D, LJ_original.LAP*R2D, LJ_original.LAR*R2D,

                                          HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                          0, 0, 0, 0,
                                          HBPW.RS.JSV.dRAP*R2D, HBPW.RS.JSV.dRAR*R2D, HBPW.RS.JSV.dLAP*R2D, HBPW.RS.JSV.dLAR*R2D);

            vec4 AnkleToque;

            AnkleToque[0] = cZMP_FF_torque[0];// - Angle_FB_torque[0];
            AnkleToque[1] = cZMP_FF_torque[1];// - Angle_FB_torque[1];
            AnkleToque[2] = cZMP_FF_torque[2];// - Angle_FB_torque[2];
            AnkleToque[3] = cZMP_FF_torque[3];// - Angle_FB_torque[3];

            HBPW.AnkleTorqueController(AnkleToque[0], AnkleToque[1], AnkleToque[2], AnkleToque[3],
                                   HBPW.RS.JSP.RA1*R2D, HBPW.RS.JSP.RA2*R2D, HBPW.RS.JSP.LA1*R2D, HBPW.RS.JSP.LA2*R2D,
                                   HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                   HBPW.RS.JSV.dRA1*R2D, HBPW.RS.JSV.dRA2*R2D, HBPW.RS.JSV.dLA1*R2D, HBPW.RS.JSV.dLA2*R2D,
                                   HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);



            HBPW.AnkleQuatCalculaltor(HBPW.RS.JSP.RAP*R2D - LJ_original.RAP*R2D, HBPW.RS.JSP.RAR*R2D - LJ_original.RAR*R2D,
                                      HBPW.RS.JSP.LAP*R2D - LJ_original.LAP*R2D, HBPW.RS.JSP.LAR*R2D - LJ_original.LAR*R2D);

            quat total_qRF = HBPW.qRF_ref*HBPW.RF_del_quat;
            quat total_qLF = HBPW.qLF_ref*HBPW.LF_del_quat;
            doubles dbs_qRF(4), dbs_qLF(4);
            for(int i=0;i<4;i++){
                dbs_qRF[i] = total_qRF[i];
                dbs_qLF[i] = total_qLF[i];

            }

            WBmotion->addRFOriInfo(dbs_qRF,0.005);
            WBmotion->addLFOriInfo(dbs_qLF,0.005);

            //cout<<"ref RAP: "<<AnkleToque[1]<<" RAP: "<<HBPW.RS.M_RF.y<<" ref LAP: "<<AnkleToque[3]<<" LAP: "<<HBPW.RS.M_LF.y<<endl;
*/
            //// -------------------------Sensor agreement test with Choreonoid
//            double Torso_ori_deg = 0;

//            Torso_ori_deg = 10*sin(2*3.141*0.5*HBPW.k*HBPW.dt);

//            quat Torso_quat = quat(vec3(1,0,0),Torso_ori_deg*D2R);

//            doubles Torso_quat_dbs(4);
//            for(int i=0;i<4;i++){
//                Torso_quat_dbs[i] = Torso_quat[i];
//            }

//            WBmotion->addPELOriInfo(Torso_quat_dbs, 0.005);

//            if(HBPW.k > 10*200){
//                _task_thread = _task_Idle;
//                HBPW.save_all();
//            }

            ////----------------------- Kinematics calibration
//            cout<<"Solution: 0.1091    "<<"ZMP y: "<<HBPW.ZMP_global.y<<endl;

            ////---------------- DSP Fz Controller & Ankle PWM Torque Control Test
//            double dT_cZMP = 0.1;
//            HBPW.cZMP = vec3(0,0,0) + exp(HBPW.w*dT_cZMP)/(exp(HBPW.w*dT_cZMP) - 1)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w) - 0);

//            HBPW.cZMP_proj = zmpProjectionToSP_offset(HBPW.cZMP, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, 0.00);

//            // get alpha_dsp and Ankle Torque Control

//            // Calc Original leg joint for kinematics update during Ankle Torque Control
//            HBPW.LJ_ref = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);


//            vec4 cZMP_FF_torque = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

//            vec4 Angle_FB_torque = HBPW.Ankle_Torque_from_PositionFB(2, HBPW.RS.F_RF, HBPW.RS.F_LF,
//                                                                HBPW.LJ_ref.RAP*R2D, HBPW.LJ_ref.RAR*R2D, HBPW.LJ_ref.LAP*R2D, HBPW.LJ_ref.LAR*R2D,
//                                                                HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
//                                                                0*R2D, 0*R2D, 0*R2D, 0*R2D,
//                                                                HBPW.RS.JSV.dRAP*R2D, HBPW.RS.JSV.dRAR*R2D, HBPW.RS.JSV.dLAP*R2D, HBPW.RS.JSV.dLAR*R2D);

//            HBPW.AnkleTorque_ref[0] = cZMP_FF_torque[0] - Angle_FB_torque[0];
//            HBPW.AnkleTorque_ref[1] = cZMP_FF_torque[1] - Angle_FB_torque[1];
//            HBPW.AnkleTorque_ref[2] = cZMP_FF_torque[2] - Angle_FB_torque[2];
//            HBPW.AnkleTorque_ref[3] = cZMP_FF_torque[3] - Angle_FB_torque[3];

//            HBPW.AnkleTorqueController(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
//                                       HBPW.RS.JSP.RA1*R2D, HBPW.RS.JSP.RA2*R2D, HBPW.RS.JSP.LA1*R2D, HBPW.RS.JSP.LA2*R2D,
//                                       HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
//                                       HBPW.RS.JSV.dRA1*R2D, HBPW.RS.JSV.dRA2*R2D, HBPW.RS.JSV.dLA1*R2D, HBPW.RS.JSV.dLA2*R2D,
//                                       HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

//            //Ankle quaternion compensation( Calc RF_LF_del_quat )
//            HBPW.AnkleQuatCalculaltor(HBPW.RS.JSP.RAP*R2D - HBPW.LJ_ref.RAP*R2D, HBPW.RS.JSP.RAR*R2D - HBPW.LJ_ref.RAR*R2D,
//                                      HBPW.RS.JSP.LAP*R2D - HBPW.LJ_ref.LAP*R2D, HBPW.RS.JSP.LAR*R2D - HBPW.LJ_ref.LAR*R2D);


//            // get Alpha dsp
//            vec3 L = HBPW.pRF_ref - HBPW.pLF_ref;
//            L.z = 0;
//            vec3 L_G = HBPW.G_R_g_pitroll*L;

//            if(L_G.z >= 0.005) L_G.z = 0.005;
//            if(L_G.z <= -0.005) L_G.z = -0.005;

//            HBPW.Alpha_dsp = 0.5 - L_G.z/0.005*0.5;


//            // calc ref Fz differnece
//            double Fz_total = 320;//HBPW.RS.F_LF.z + HBPW.RS.F_RF.z ; //32kg

//            static double Fz_total_filtered = 0;

//            double alpha_Fz = 1/(1 + 2*PI*dt*3.0);

//            Fz_total_filtered = alpha_Fz*Fz_total_filtered + (1 - alpha_Fz)*Fz_total;

//            HBPW.RF_Fz_ref = HBPW.Alpha_dsp*Fz_total;
//            HBPW.LF_Fz_ref = (1.0 - HBPW.Alpha_dsp)*Fz_total;

//            HBPW.Fz_diff_ref = HBPW.LF_Fz_ref - HBPW.RF_Fz_ref;

//            HBPW.Fz_diff = HBPW.RS.F_LF.z - HBPW.RS.F_RF.z;

//            // Control
//            double dsp_ctrl_gain = 0.0006;

//            HBPW.dz_ctrl = dsp_ctrl_gain*(HBPW.Fz_diff_ref - HBPW.Fz_diff);// - HBPW.z_ctrl/5;


//            double alpha_dz = 1/(1 + 2*PI*dt*5);
//            HBPW.dz_ctrl_filtered = alpha_dz*HBPW.dz_ctrl_filtered + (1.0 - alpha_dz)*HBPW.dz_ctrl;


//            HBPW.z_ctrl += HBPW.dz_ctrl_filtered*HBPW.dt;

//            if(HBPW.z_ctrl > 0.03) HBPW.z_ctrl = 0.03;
//            if(HBPW.z_ctrl < -0.03) HBPW.z_ctrl = -0.03;

//            double alpha_z = 1/(1 + 2*PI*dt*10.0);
//            HBPW.z_ctrl_filtered = alpha_z*HBPW.z_ctrl_filtered + (1.0 - alpha_z)*HBPW.z_ctrl;

//            HBPW.pRF_ref.z = + 0.5*HBPW.z_ctrl;
//            HBPW.pLF_ref.z = - 0.5*HBPW.z_ctrl;


//            cout<<"z_ctrl: "<<HBPW.z_ctrl<<" cZMP: "<<HBPW.cZMP_proj.y<<"  alpha dsp : "<<HBPW.Alpha_dsp<<endl;


//            doubles dbs_qRF(4), dbs_qLF(4);
//                for(int i=0;i<4;i++){
//                    dbs_qRF[i] = HBPW.qRF_ref[i];
//                    dbs_qLF[i] = HBPW.qLF_ref[i];
//                }
//

//            // Put reference Task to Trajectory Handler
//            WBmotion->addRFPosInfo(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z,0.005);
//            WBmotion->addLFPosInfo(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z,0.005);
//            WBmotion->addRFOriInfo(dbs_qRF,0.005);
//            WBmotion->addLFOriInfo(dbs_qLF,0.005);

//            HBPW.k++;

//            save_onestep(HBPW.k);


            ////--------------- DSP Fz Controller & Ankle "angle" Torque Control Test

            //--------Ankle angle control ---------------------

//            double dT_cZMP = 0.2;
//            HBPW.cZMP = exp(HBPW.w*dT_cZMP)/(exp(HBPW.w*dT_cZMP) - 1)*HBPW.CP_e_imu;

//            HBPW.cZMP_proj = zmpProjectionToSP_offset(HBPW.cZMP, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, 0.00);

//            HBPW.AnkleTorque_ref = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

//            // calc 'XF_qaut_ctrl' according to torque reference
////            HBPW.AnkleTorqueController_pos(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
////                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);
//            HBPW.AnkleTorqueController_pos(0, 0, 0, 0,
//                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

//            HBPW.qRF_ref = HBPW.RF_quat_ctrl;
//            HBPW.qLF_ref = HBPW.LF_quat_ctrl;

//            // -----------DSP Fz control--------------------

//            // get Alpha dsp
//            vec3 L = HBPW.pRF_ref - HBPW.pLF_ref;
//            L.z = 0;
//            vec3 L_G = HBPW.G_R_g_pitroll*L;

//            if(L_G.z >= 0.005) L_G.z = 0.005;
//            if(L_G.z <= -0.005) L_G.z = -0.005;

//            HBPW.Alpha_dsp = 0.5 - L_G.z/0.005*0.5;

//            // Calc ref Fz difference
//            //robot mass
//            double Fz_total = 320; //32kg
//            HBPW.RF_Fz_ref = HBPW.Alpha_dsp*Fz_total;
//            HBPW.LF_Fz_ref = (1.0 - HBPW.Alpha_dsp)*Fz_total;

//            HBPW.Fz_diff_ref = HBPW.LF_Fz_ref - HBPW.RF_Fz_ref;

//            // get Fz difference Error
//            HBPW.Fz_diff = HBPW.RS.F_LF.z - HBPW.RS.F_RF.z;


//            // Control
//            double dsp_ctrl_gain;

//            dsp_ctrl_gain = 0.0003;

//            HBPW.dz_ctrl = dsp_ctrl_gain*(HBPW.Fz_diff_ref - HBPW.Fz_diff);


//            double alpha_dz = 1/(1 + 2*PI*dt*5);
//            HBPW.dz_ctrl_filtered = alpha_dz*HBPW.dz_ctrl_filtered + (1.0 - alpha_dz)*HBPW.dz_ctrl;

//            HBPW.z_ctrl += HBPW.dz_ctrl_filtered*HBPW.dt;

//            if(HBPW.z_ctrl > 0.05) HBPW.z_ctrl = 0.05;
//            if(HBPW.z_ctrl < -0.05) HBPW.z_ctrl = -0.05;

//            HBPW.pRF_ref.z = + 0.5*HBPW.z_ctrl;
//            HBPW.pLF_ref.z = - 0.5*HBPW.z_ctrl;

            // Put reference Task to Trajectory Handler

//            doubles dbs_qRF(4), dbs_qLF(4);
//            for(int i=0;i<4;i++){
//                dbs_qRF[i] = HBPW.qRF_ref[i];
//                dbs_qLF[i] = HBPW.qLF_ref[i];
//            }

//            WBmotion->addRFPosInfo(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z,0.005);
//            WBmotion->addLFPosInfo(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z,0.005);
//            WBmotion->addRFOriInfo(dbs_qRF,0.005);
//            WBmotion->addLFOriInfo(dbs_qLF,0.005);

//            HBPW.k++;

//            save_onestep(HBPW.k);


            //// --------------------------temp

//            quat Qu1 = quat(vec3(0,0,1),D2R*181)*quat(vec3(0,1,0),D2R*20)*quat(vec3(1,0,0),D2R*-10);
//            quat Qu2 = quat(vec3(0,0,1),D2R*-179)*quat(vec3(0,1,0),D2R*20)*quat(vec3(1,0,0),D2R*-10);
//            quat Qu3 = quat(vec3(0,0,1),D2R*-90)*quat(vec3(0,1,0),D2R*20)*quat(vec3(1,0,0),D2R*-10);

//            rpy Euler_Qu1 = rpy(Qu1);
//            rpy Euler_Qu2 = rpy(Qu2);
//            rpy Euler_Qu3 = rpy(Qu3);

//            //if(Euler_Qu.y < 0) Euler_Qu.y = Euler_Qu.y + 360*D2R;

//            cout<<"Qu1: "<<Qu1.w<<", "<<Qu1.x<<", "<<Qu1.y<<", "<<Qu1.z<<endl;
//            cout<<"Qu2: "<<Qu2.w<<", "<<Qu2.x<<", "<<Qu2.y<<", "<<Qu2.z<<endl;
//            cout<<"Qu3: "<<Qu3.w<<", "<<Qu3.x<<", "<<Qu3.y<<", "<<Qu3.z<<endl;

//            cout<<"yaw : "<<Euler_Qu1.y*R2D<<", pitch: "<<Euler_Qu1.p*R2D<<", roll: "<<Euler_Qu1.r*R2D<<endl;
//            cout<<"yaw : "<<Euler_Qu2.y*R2D<<", pitch: "<<Euler_Qu2.p*R2D<<", roll: "<<Euler_Qu2.r*R2D<<endl;
//            cout<<"yaw : "<<Euler_Qu3.y*R2D<<", pitch: "<<Euler_Qu3.p*R2D<<", roll: "<<Euler_Qu3.r*R2D<<endl<<endl;

            ////---------------------------- Logging
            HBPW.Logging();

            break;
        }
        case _task_HB_Jump:
        {
            if(HBJumping.Jump_once() == -1){

                _task_thread = _task_Idle;
                HBJumping.save_all();
                save_all();
                Scnt = 0;
                cout<<"Jump Once Finish!"<<endl;
            }

            vec3 COM_total = HBJumping.COM_ref;

            WBmotion->addCOMInfo(COM_total.x, COM_total.y, COM_total.z, 0.005);

            quat total_qPel = HBJumping.qPel_ref;
            doubles dbs_qPel(4);
            for(int i=0;i<4;i++){
                dbs_qPel[i] = total_qPel[i];

            }

            WBmotion->addPELOriInfo(dbs_qPel,0.005);

            //save
            SAVE[0][Scnt] = WBmotion->LJ.RHP;
            SAVE[1][Scnt] = WBmotion->LJ.RKN;
            SAVE[2][Scnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentPosition;
            SAVE[3][Scnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition;
            SAVE[4][Scnt] = sharedSEN->FT[0].Fz;

            Scnt++;

            break;


        }
        case _task_Dyn_Walking2:
        {
            SensorInput(); //fill in the RSEN
            RST = SE.StateEst(RSEN);
//            DS = HBD2.Walk(RST,0,0,0);
            Torque2Choreonoid(DS);
            break;
        }
        case _task_Dyn_Walking:
        {
            SensorInput(); //fill in the RSEN
            RST = SE.StateEst(RSEN);
//            DS = HBD.Walk(RST);
            Torque2Choreonoid(DS);
//            if(HBD.walk_done_flag == true){
//                _task_thread = _task_Dyn_StandUp;
//                RST_ini = RST;
//            }
            break;
        }
        case _task_Dyn_StandUp:
        {
            SensorInput(); //fill in the ROBOTSEN
            RST = SE.StateEst(RSEN);
            //DS = HBD.StandUp(RST, RST_ini);
            Torque2Choreonoid(DS);

//            if(Scnt%5 == 0){
//                cout<<"pPel("<<RST.CSP.pPel.x<<", "<<RST.CSP.pPel.y<<", "<<RST.CSP.pPel.z<<")"<<endl;//  RF_Gnd("<<RST.CSP.pRF.x<<", "<<RST.CSP.pRF.y<<", "<<RST.CSP.pRF.z<<")"<<endl;
//            }

            Scnt++;
            break;
        }

        case _task_Inv_Dyn_con:
        {
            
            SensorInput();

            REF.Qref = VectorNd::Zero(bp_rbdl.Robot->q_size);
            REF.dQref = VectorNd::Zero(bp_rbdl.Robot->dof_count);
            REF.ddQref = VectorNd::Zero(bp_rbdl.Robot->dof_count);

            for(int i=0 ;i<12;i++){
                REF.JSP.JSP_Array[i] = getPosRef(i);
            }

            REF.Qref = REF.getQref(bp_rbdl.Robot->q_size);

            //DS = HBD.InvDyn_Control(RST, REF);


            Scnt++;

            break;
        }

        case _task_HB_Walking:
        {
            sharedCMD->Choreonoid.TorqueConOnOff = 0;

//            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
//            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
//            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
//            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            vec3 F_RF = vec3(0,0,200);
            vec3 F_LF = vec3(0,0,200);
            vec3 M_RF = vec3(0,0,0);
            vec3 M_LF = vec3(0,0,0);
            vec3 pRF(WBmotion->pRF_3x1);
            vec3 pLF(WBmotion->pLF_3x1);
            quat qRF(WBmotion->qRF_4x1);
            quat qLF(WBmotion->qLF_4x1);
            vec3 pPel(WBmotion->pPel_3x1);
            quat qPel(WBmotion->qPEL_4x1);
            vec3 pCOM(WBmotion->pCOM_3x1);
            vec3 IMUangle(sharedSEN->IMU[0].Roll,sharedSEN->IMU[0].Pitch,sharedSEN->IMU[0].Yaw);
            //quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
            quat IMUquat(1, 0,0, 0);
//            vec3 IMUangle(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->FOG.Yaw);
            vec3 IMUvel(sharedSEN->FOG.RollVel,sharedSEN->FOG.PitchVel,sharedSEN->FOG.YawVel);
            vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);

            vec4 R_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentVelocity);

            vec4 L_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentVelocity);

            double RAR_ref_vel = (joint->GetJointRefAngle(RAR) - RAR_ang_old)/0.005;
            double RAP_ref_vel = (joint->GetJointRefAngle(RAP) - RAP_ang_old)/0.005;
            double LAR_ref_vel = (joint->GetJointRefAngle(LAR) - LAR_ang_old)/0.005;
            double LAP_ref_vel = (joint->GetJointRefAngle(LAP) - LAP_ang_old)/0.005;

            vec4 R_Ank_ref = vec4(joint->GetJointRefAngle(RAR), RAR_ref_vel, joint->GetJointRefAngle(RAP),RAP_ref_vel);
            vec4 L_Ank_ref = vec4(joint->GetJointRefAngle(LAR), LAR_ref_vel, joint->GetJointRefAngle(LAP),LAP_ref_vel);

            RAR_ang_old = joint->GetJointRefAngle(RAR);
            RAP_ang_old = joint->GetJointRefAngle(RAP);
            LAR_ang_old = joint->GetJointRefAngle(LAR);
            LAP_ang_old = joint->GetJointRefAngle(LAP);


            vec3 COM_est = vec3(sharedCMD->StateEstimate.X,sharedCMD->StateEstimate.Y,0);
            vec3 dCOM_est = vec3(sharedCMD->StateEstimate.dX,sharedCMD->StateEstimate.dY,0);


            HBWalking.MeasurementInput(pCOM, COM_est, dCOM_est, pPel,qPel,IMUangle,IMUquat,IMUvel,vec3(0,0,0),F_RF,F_LF,M_RF,M_LF,pRF,pLF,qRF,qLF,R_Ank_angVel,L_Ank_angVel,R_Ank_ref,L_Ank_ref);


            if(HBWalking.CPS_CPT_Walking() == -1){
                HBWalking.save_all();
                sharedCMD->Choreonoid.TorqueConOnOff = 0;

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HBWalking_STOP;
                _task_thread = _task_Idle;
                //WB_FLAG = 0;
                cout<<"Walk Finished!"<<endl;
            }

            vec3 COM_total = HBWalking.COM;

            WBmotion->addCOMInfo(COM_total.x, COM_total.y, pCOM.z, 0.005);
            WBmotion->addRFPosInfo(HBWalking.pRF.x,HBWalking.pRF.y,HBWalking.pRF.z,0.005);
            WBmotion->addLFPosInfo(HBWalking.pLF.x,HBWalking.pLF.y,HBWalking.pLF.z,0.005);

            //Ankle Torque Control for Choreonoid
            sharedCMD->Choreonoid.RX_TorqueRef = HBWalking.R_ank_torque.x;
            sharedCMD->Choreonoid.RY_TorqueRef = HBWalking.R_ank_torque.y;
            sharedCMD->Choreonoid.LX_TorqueRef = HBWalking.L_ank_torque.x;
            sharedCMD->Choreonoid.LY_TorqueRef = HBWalking.L_ank_torque.y;

            HBWalking.R_KNEE_torque = sharedCMD->Choreonoid.R_KNEE_T;
            HBWalking.R_HIP_P_torque = sharedCMD->Choreonoid.R_HIP_PITCH_T;
            HBWalking.R_HIP_R_torque = sharedCMD->Choreonoid.R_HIP_ROLL_T;

            //Torso Orientation Control

            break;
        }
        case _task_Control_on:
        {
            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz*4.0/5.0);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            vec3 pRF(WBmotion->pRF_3x1);
            vec3 pLF(WBmotion->pLF_3x1);
            quat qRF(WBmotion->qRF_4x1);
            quat qLF(WBmotion->qLF_4x1);
            vec3 pPel(WBmotion->pPel_3x1);
            quat qPel(WBmotion->qPEL_4x1);
            vec3 pCOM(WBmotion->pCOM_3x1);
            vec3 IMUangle(sharedSEN->IMU[0].Roll,sharedSEN->IMU[0].Pitch,sharedSEN->IMU[0].Yaw);
            quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
//            vec3 IMUangle(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->FOG.Yaw);
            vec3 IMUvel(sharedSEN->FOG.RollVel,sharedSEN->FOG.PitchVel,sharedSEN->FOG.YawVel);
            vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);

            vec4 R_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentVelocity);

            vec4 L_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentVelocity);


            double RAR_ref_vel = (joint->GetJointRefAngle(RAR) - RAR_ang_old)/0.005;
            double RAP_ref_vel = (joint->GetJointRefAngle(RAP) - RAP_ang_old)/0.005;
            double LAR_ref_vel = (joint->GetJointRefAngle(LAR) - LAR_ang_old)/0.005;
            double LAP_ref_vel = (joint->GetJointRefAngle(LAP) - LAP_ang_old)/0.005;

            vec4 R_Ank_ref = vec4(joint->GetJointRefAngle(RAR), RAR_ref_vel, joint->GetJointRefAngle(RAP),RAP_ref_vel);
            vec4 L_Ank_ref = vec4(joint->GetJointRefAngle(LAR), LAR_ref_vel, joint->GetJointRefAngle(LAP),LAP_ref_vel);


            RAR_ang_old = joint->GetJointRefAngle(RAR);
            RAP_ang_old = joint->GetJointRefAngle(RAP);
            LAR_ang_old = joint->GetJointRefAngle(LAR);
            LAP_ang_old = joint->GetJointRefAngle(LAP);

            vec3 COM_est = vec3(sharedCMD->StateEstimate.X,sharedCMD->StateEstimate.Y,0);
            vec3 dCOM_est = vec3(sharedCMD->StateEstimate.dX,sharedCMD->StateEstimate.dY,0);

            HBWalking.MeasurementInput(pCOM, COM_est, dCOM_est, pPel,qPel,IMUangle,IMUquat,IMUvel,vec3(0,0,0),F_RF,F_LF,M_RF,M_LF,pRF,pLF,qRF,qLF,R_Ank_angVel,L_Ank_angVel,R_Ank_ref,L_Ank_ref);

            vec3 CP_m_filtered = HBWalking.COM_m_filtered2 + HBWalking.dCOM_m_filtered2/HBWalking.w;   //for ddCOM_con generation
            vec3 CP_m = HBWalking.COM_m_filtered + HBWalking.dCOM_m_filtered/HBWalking.w;   //for ATC

            // CPT approach
            int dt_gain2 = 40;//60;  // for CP feedback -->> controller gain tuning parameter ( large value is low control gain)
            int dt_gain3 = 60;  // for Ankle Torque Control

            vec3 ZMP_ref = 1/(1 - exp(HBWalking.w*dt*dt_gain3))*Initial_COM - exp(HBWalking.w*dt*dt_gain3)/(1 - exp(HBWalking.w*dt*dt_gain3))*CP_m; //ankle torque control
            vec3 ZMP_ref_filtered = 1/(1 - exp(HBWalking.w*dt*dt_gain2))*Initial_COM - exp(HBWalking.w*dt*dt_gain2)/(1 - exp(HBWalking.w*dt*dt_gain2))*CP_m_filtered; //for ddCOM_con generation

            vec3 ZMP_ref_filtered_inSP = zmpProjectionToSP(ZMP_ref_filtered, pRF, pLF, qRF, qLF, F_RF, F_LF);
            vec3 ZMP_ref_inSP = zmpProjectionToSP_large(ZMP_ref, pRF, pLF, qRF, qLF, F_RF, F_LF, -0.01);

            double k_gain_y = 0.01;//1.0; //--> gain scheduling needed
            double k_gain_x = 0.01;//0.008;//1.0;//1.5;

            double Fz_both = 430;//F_RF.z + F_LF.z;

            vec3 ZMP_ref_filtered_local = Calc_local(pRF, qRF, pLF, qLF, ZMP_ref_filtered_inSP);
            vec3 ZMP_filtered_local = Calc_local(pRF, qRF, pLF, qLF, HBWalking.ZMP_global_filtered);

            vec3 ddCOM_con;
            ddCOM_con.y = k_gain_y*Fz_both/HBWalking.zc*(ZMP_filtered_local.y - ZMP_ref_filtered_local.y);
            ddCOM_con.x = k_gain_x*Fz_both/HBWalking.zc*(ZMP_filtered_local.x - ZMP_ref_filtered_local.x);
            ddCOM_con.z = 0;


//            HBWalking.dCOM_con  = HBWalking.dCOM_con + ddCOM_con*dt;
//            HBWalking.COM_con  = HBWalking.COM_con + HBWalking.dCOM_con*dt - HBWalking.dCOM_con*0.01;

//            if(HBWalking.COM_con.x >= 0.06) HBWalking.COM_con.x = 0.06;
//            if(HBWalking.COM_con.x <= -0.06) HBWalking.COM_con.x = -0.06;
//            if(HBWalking.COM_con.y >= 0.06) HBWalking.COM_con.y = 0.06;
//            if(HBWalking.COM_con.y <= -0.06) HBWalking.COM_con.y = -0.06;

            vec3 u;
            u.y = ddCOM_con.y;// + HBWalking.k_dsp_y*Initial_COM.y;
            u.x = ddCOM_con.x;// + HBWalking.k_dsp_x*Initial_COM.x;

            ctrl_COM.y = exp(-HBWalking.k_dsp_y/HBWalking.c_dsp_y*dt)*ctrl_COM.y - 1/HBWalking.k_dsp_y*u.y*(exp(-HBWalking.k_dsp_y/HBWalking.c_dsp_y*dt) - 1);
            ctrl_COM.x = exp(-HBWalking.k_dsp_x/HBWalking.c_dsp_x*dt)*ctrl_COM.x - 1/HBWalking.k_dsp_x*u.x*(exp(-HBWalking.k_dsp_x/HBWalking.c_dsp_x*dt) - 1);
//            u.y = ddCOM_con.y + HBWalking.k_ssp_y*Initial_COM.y;
//            u.x = ddCOM_con.x + HBWalking.k_ssp_x*Initial_COM.x;

//            ctrl_COM.y = exp(-HBWalking.k_ssp_y/HBWalking.c_ssp_y*dt)*ctrl_COM.y - 1/HBWalking.k_ssp_y*u.y*(exp(-HBWalking.k_ssp_y/HBWalking.c_ssp_y*dt) - 1);
//            ctrl_COM.x = exp(-HBWalking.k_ssp_x/HBWalking.c_ssp_x*dt)*ctrl_COM.x - 1/HBWalking.k_ssp_x*u.x*(exp(-HBWalking.k_ssp_x/HBWalking.c_ssp_x*dt) - 1);

            //ctrl_COM = Initial_COM + HBWalking.COM_con;

            //cout<<"ctrl_COM.y: "<<ctrl_COM.y<<endl;

            //DSP control
            VectorNd FF_torque = AnkleFT_Calculator(ZMP_ref_inSP, qPel, pRF, pLF, qRF, qLF, F_RF, F_LF);
            vec4 Angle_FB_torque = AnkleJointAngleContoller(2, F_RF, F_LF, R_Ank_angVel, L_Ank_angVel, R_Ank_ref, L_Ank_ref);

            double Fz_diff_ref = FF_torque[5] - FF_torque[4]; // LFz_ref - RFz_ref
            double Fz_diff = F_LF.z - F_RF.z;

            double alpha = 1/(1 + 2*PI*dt*2);
            double Fz_diff_error = alpha*Fz_diff_error_old + (1 - alpha)*(Fz_diff_ref - Fz_diff);
            Fz_diff_error_old = Fz_diff_error;

            double z_ctrl_gain = 0.0007;
            double dz_ctrl = 0;//z_ctrl_gain*Fz_diff_error;// - z_ctrl/30.0;

            z_ctrl += dz_ctrl*dt;
            //cout<<"z_ctrl : "<<z_ctrl<<endl;


            vec3 ctrl_pRF = vec3(0,0, 0.5*z_ctrl);
            vec3 ctrl_pLF = vec3(0,0, -0.5*z_ctrl);

            vec3 COM_total = Initial_COM + ctrl_COM;
            vec3 pRF_total = Initial_pRF + ctrl_pRF;
            vec3 pLF_total = Initial_pLF + ctrl_pLF;

            //ATC
            double alpha1 = 1/(1 + 2*PI*dt*2.5); // roll torque filtering
            double alpha2 = 1/(1 + 2*PI*dt*4); // pitch torque filtering
            for(int i=0 ;i<4;i++){
                if(i == 0 || i == 2)
                    input_torque_filtered[i] = alpha1*input_torque_filtered[i] + (1.0 - alpha1)*(FF_torque[i]);// + Angle_FB_torque[i]);
                if(i == 1 || i == 3)
                    input_torque_filtered[i] = alpha2*input_torque_filtered[i] + (1.0 - alpha2)*(FF_torque[i]);// + Angle_FB_torque[i]);
            }

            vec4 input_torque;
            input_torque[0] = input_torque_filtered[0] + Angle_FB_torque[0];
            input_torque[1] = input_torque_filtered[1] + Angle_FB_torque[1];
            input_torque[2] = input_torque_filtered[2] + Angle_FB_torque[2];
            input_torque[3] = input_torque_filtered[3] + Angle_FB_torque[3];

            WBmotion->addCOMInfo(COM_total.x, COM_total.y, pCOM.z, 0.005);
//            WBmotion->addCOMInfo(pCOM.x, pCOM.y, pCOM.z, 0.005);
            WBmotion->addRFPosInfo(pRF_total.x,pRF_total.y,pRF_total.z,0.005);
            WBmotion->addLFPosInfo(pLF_total.x,pLF_total.y,pLF_total.z,0.005);


            //pelvis control
//            double rpyGain = 0.01;
//            double RollError = -IMUangle[0]; double PitchError = -IMUangle[1];

//            if(RollError >= 5) RollError = 0;
//            if(RollError <= -5) RollError = -0;
//            if(PitchError >= 5) PitchError = 0;
//            if(PitchError <= -5) PitchError = -0;

//            cout<<"RollError : "<<RollError<<", PitchError: "<<PitchError<<endl;

//            rpy RP_Error = rpy(rpyGain*RollError*D2R, rpyGain*PitchError*D2R, 0);

//            //cout<<"RollError : "<<RP_Error.r<<", PitchError: "<<RP_Error.p<<endl;

//            quat Error_quat = quat(RP_Error);

//            cout<<"Error quat: "<<Error_quat[0]<<", "<<Error_quat[1]<<", "<<Error_quat[2]<<endl;
//            quat NEWqPel =  Error_quat;

//            doubles newqPel(4);
//            newqPel[0] = NEWqPel[0];
//            newqPel[1] = NEWqPel[1];
//            newqPel[2] = NEWqPel[2];
//            newqPel[3] = NEWqPel[3];
//            WBmotion->addPELOriInfo(newqPel,0.005);

            //rpy temp = rpy(NEWqPel);

            //cout<<"rpy : "<<temp.r<<", "<<temp.p<<endl;

            break;
        }

        case _task_TorqueTest:
        {
            // Ankle position control Test
            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            vec3 pRF(WBmotion->pRF_3x1);
            vec3 pLF(WBmotion->pLF_3x1);
            quat qRF(WBmotion->qRF_4x1);
            quat qLF(WBmotion->qLF_4x1);
            vec3 pPel(WBmotion->pPel_3x1);
            quat qPel(WBmotion->qPEL_4x1);
            vec3 pCOM(WBmotion->pCOM_3x1);
            vec3 IMUangle(sharedSEN->IMU[0].Roll+0.0,sharedSEN->IMU[0].Pitch,sharedSEN->IMU[0].Yaw);
            quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
            //vec3 IMUangle(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->FOG.Yaw);
            vec3 IMUvel(sharedSEN->FOG.RollVel,sharedSEN->FOG.PitchVel,sharedSEN->FOG.YawVel);
            vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);

            vec4 R_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentVelocity);

            vec4 L_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentVelocity);

            vec4 Angle_FB_torque = AnkleJointAngleContoller(2,F_RF,F_LF,R_Ank_angVel,L_Ank_angVel,vec4(0,0,0,0),vec4(0,0,0,0));



            AnkleTorqueControl(-Angle_FB_torque[0],-Angle_FB_torque[1],-Angle_FB_torque[2],-Angle_FB_torque[3], F_RF,F_LF,M_RF,M_LF);
            //cout<<Angle_FB_torque[0]<<endl;
            break;
        }
        case _task_SysID:
        {
            double w = 2*PI*InputFreq;
            vec3 COM_input = vec3(0,0,0);

            COM_input.y = InputAmp*sin(w*0.005*SysIDcnt);

            WBmotion->addCOMInfo(Initial_COM.x, Initial_COM.y + COM_input.y, WBmotion->pCOM_3x1[2], 0.005);

            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            vec3 pRF(WBmotion->pRF_3x1);
            vec3 pLF(WBmotion->pLF_3x1);
            quat qRF(WBmotion->qRF_4x1);
            quat qLF(WBmotion->qLF_4x1);
            vec3 pPel(WBmotion->pPel_3x1);
            quat qPel(WBmotion->qPEL_4x1);
            vec3 pCOM(WBmotion->pCOM_3x1);
//            vec3 IMUangle(sharedSEN->FOG.Roll,sharedSEN->FOG.Pitch,sharedSEN->FOG.Yaw);
            vec3 IMUangle(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->FOG.Yaw);
            quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
            vec3 IMUvel(sharedSEN->FOG.RollVel,sharedSEN->FOG.PitchVel,sharedSEN->FOG.YawVel);
            vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);

            vec4 R_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentVelocity);

            vec4 L_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentVelocity);


            vec4 R_Ank_ref = vec4(joint->GetJointRefAngle(RAR), 0, joint->GetJointRefAngle(RAP),0);
            vec4 L_Ank_ref = vec4(joint->GetJointRefAngle(LAR), 0, joint->GetJointRefAngle(LAP),0);

//            vec3 COM_est = vec3(sharedCMD->StateEstimate.X,sharedCMD->StateEstimate.Y,0);
//            vec3 dCOM_est = vec3(sharedCMD->StateEstimate.dX,sharedCMD->StateEstimate.dY,0);
//            vec3 ZMP_est = vec3(sharedCMD->StateEstimate.Px,sharedCMD->StateEstimate.Py,0);

//            sharedCMD->StateEstimate.uX = HBWalking.COM_m.x;
//            sharedCMD->StateEstimate.uY = HBWalking.COM_m.y;
//            sharedCMD->StateEstimate.udX = HBWalking.dCOM_m.x;
//            sharedCMD->StateEstimate.udY = HBWalking.dCOM_m.y;
//            sharedCMD->StateEstimate.uZMPx = HBWalking.ZMP_global.x;
//            sharedCMD->StateEstimate.uZMPy = HBWalking.ZMP_global.y;
            
//            vec3 pPel_choreonoid(sharedCMD->Choreonoid.pPel_3x1);
//            quat qPel_choreonoid(sharedCMD->Choreonoid.qPel_4x1);

            vec3 COM_est = vec3(sharedCMD->StateEstimate.X,sharedCMD->StateEstimate.Y,0);
            vec3 dCOM_est = vec3(sharedCMD->StateEstimate.dX,sharedCMD->StateEstimate.dY,0);

            HBWalking.MeasurementInput(pCOM, COM_est, dCOM_est, pPel,qPel,IMUangle,IMUquat,IMUvel,vec3(0,0,0),F_RF,F_LF,M_RF,M_LF,pRF,pLF,qRF,qLF,R_Ank_angVel,L_Ank_angVel,R_Ank_ref,L_Ank_ref);

            //HBWalking.Measurement_Choreonoid();


            if(SysIDcnt_init >= 400){
                HBWalking.save_onestep(SysIDcnt);
                HBWalking.k++;
                SysIDcnt++;
            }
            SysIDcnt_init++;
            break;
        }
        case _task_SysID_step:
        {
            vec3 COM_input = vec3(0,0,0);

            if(SysIDcnt_init < 100) COM_input.y = 0.00;
            else  COM_input.y = InputAmp;

            WBmotion->addCOMInfo(Initial_COM.x, Initial_COM.y + COM_input.y, WBmotion->pCOM_3x1[2], 0.050);

            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            vec3 pRF(WBmotion->pRF_3x1);
            vec3 pLF(WBmotion->pLF_3x1);
            quat qRF(WBmotion->qRF_4x1);
            quat qLF(WBmotion->qLF_4x1);
            vec3 pPel(WBmotion->pPel_3x1);
            quat qPel(WBmotion->qPEL_4x1);
            vec3 pCOM(WBmotion->pCOM_3x1);
//            vec3 IMUangle(sharedSEN->FOG.Roll,sharedSEN->FOG.Pitch,sharedSEN->FOG.Yaw);
            vec3 IMUangle(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->FOG.Yaw);
            quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
            vec3 IMUvel(sharedSEN->FOG.RollVel,sharedSEN->FOG.PitchVel,sharedSEN->FOG.YawVel);
            vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);

            vec4 R_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentVelocity);

            vec4 L_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentVelocity);


            vec4 R_Ank_ref = vec4(joint->GetJointRefAngle(RAR), 0, joint->GetJointRefAngle(RAP),0);
            vec4 L_Ank_ref = vec4(joint->GetJointRefAngle(LAR), 0, joint->GetJointRefAngle(LAP),0);

            vec3 COM_est = vec3(sharedCMD->StateEstimate.X,sharedCMD->StateEstimate.Y,0);
            vec3 dCOM_est = vec3(sharedCMD->StateEstimate.dX,sharedCMD->StateEstimate.dY,0);

            HBWalking.MeasurementInput(pCOM, COM_est, dCOM_est, pPel,qPel,IMUangle,IMUquat,IMUvel,vec3(0,0,0),F_RF,F_LF,M_RF,M_LF,pRF,pLF,qRF,qLF,R_Ank_angVel,L_Ank_angVel,R_Ank_ref,L_Ank_ref);

            sharedCMD->StateEstimate.uX = HBWalking.COM_m.x;
            sharedCMD->StateEstimate.uY = HBWalking.COM_m.y;
            sharedCMD->StateEstimate.udX = HBWalking.dCOM_m.x;
            sharedCMD->StateEstimate.udY = HBWalking.dCOM_m.y;
            sharedCMD->StateEstimate.uZMPx = HBWalking.ZMP_global.x;
            sharedCMD->StateEstimate.uZMPy = HBWalking.ZMP_global.y;


            if(SysIDcnt_init >= 0){
                HBWalking.save_onestep(SysIDcnt);
                HBWalking.k++;
                SysIDcnt++;
            }
            SysIDcnt_init++;
            break;
        }
        case _task_Ankle_Torque_con_test:
        {
            // Ankle torque Control test-------------------------------------------------------------
            vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz*4.0/5.0);
            vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
            vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
            vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
            vec3 pRF(WBmotion->pRF_3x1);
            vec3 pLF(WBmotion->pLF_3x1);
            quat qRF(WBmotion->qRF_4x1);
            quat qLF(WBmotion->qLF_4x1);
            vec3 pPel(WBmotion->pPel_3x1);
            quat qPel(WBmotion->qPEL_4x1);
            vec3 pCOM(WBmotion->pCOM_3x1);
            vec3 IMUangle(sharedSEN->IMU[0].Roll+0.0,sharedSEN->IMU[0].Pitch,sharedSEN->IMU[0].Yaw);
//            vec3 IMUangle(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->FOG.Yaw);
            vec3 IMUvel(sharedSEN->FOG.RollVel,sharedSEN->FOG.PitchVel,sharedSEN->FOG.YawVel);
            vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);

            vec4 R_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentVelocity);

            vec4 L_Ank_angVel = vec4(sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentVelocity,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition,
                                     sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentVelocity);


            double RAR_ref_vel = (joint->GetJointRefAngle(RAR) - RAR_ang_old)/0.005;
            double RAP_ref_vel = (joint->GetJointRefAngle(RAP) - RAP_ang_old)/0.005;
            double LAR_ref_vel = (joint->GetJointRefAngle(LAR) - LAR_ang_old)/0.005;
            double LAP_ref_vel = (joint->GetJointRefAngle(LAP) - LAP_ang_old)/0.005;

//            vec4 R_Ank_ref = vec4(joint->GetJointRefAngle(RAR), RAR_ref_vel, joint->GetJointRefAngle(RAP),RAP_ref_vel);
//            vec4 L_Ank_ref = vec4(joint->GetJointRefAngle(LAR), LAR_ref_vel, joint->GetJointRefAngle(LAP),LAP_ref_vel);
            vec4 R_Ank_ref = vec4(joint->GetJointRefAngle(RAR), 0, joint->GetJointRefAngle(RAP),0);
            vec4 L_Ank_ref = vec4(joint->GetJointRefAngle(LAR), 0, joint->GetJointRefAngle(LAP),0);

            RAR_ang_old = joint->GetJointRefAngle(RAR);
            RAP_ang_old = joint->GetJointRefAngle(RAP);
            LAR_ang_old = joint->GetJointRefAngle(LAR);
            LAP_ang_old = joint->GetJointRefAngle(LAP);

            cout<<"input torque : "<<input_Amp<<"  ";
            AnkleTorqueControl(0,0,0,input_Amp,   F_RF,F_LF,M_RF,M_LF);



            break;
        }
        case _task_SingleLog_Walk:
        {

            //// Put sensor value into RSEN object
            SensorInput();

            //// State Estimation
            RST = SE.StateEst(RSEN);
            GGSW.MeasurementInput(RST);

            //// Main Walking code
            if(GGSW.Preveiw_walking() == -1)
            {
                _task_thread = _task_Idle_SingleLog;

                save_all_gg();
                cout<<"Preview Walk finished"<<endl;
            }

            vec3 COM_total = GGSW.uCOM;

            //// Leg Vibration Control
            LHY_con_deg = -GGSW.LHY_con_deg;
            RHY_con_deg = -GGSW.RHY_con_deg;

            LHR_con_deg = 1.0*GGSW.LHR_con_deg + GGSW.L_roll_compen_deg;
            RHR_con_deg = 1.0*GGSW.RHR_con_deg + GGSW.R_roll_compen_deg;

            LHP_con_deg = 1.0*GGSW.LHP_con_deg*0.5;
            RHP_con_deg = 1.0*GGSW.RHP_con_deg*0.5;
            LKN_con_deg = GGSW.L_knee_compen_deg;
            RKN_con_deg = GGSW.R_knee_compen_deg;


            ////Foot and Pelv Orientation
            //foot Orientation

                for(int i=0;i<4;i++)
                {
                    dbs_qRF[i] = GGSW.qRF_ref[i];
                    dbs_qLF[i] = GGSW.qLF_ref[i];
                }


            // Pelvis Orientation
            for(int i=0;i<4;i++)
            {
                dbs_qPel[i] = GGSW.qPel_ref[i];
            }

            //// Put reference Task to Trajectory Handler
            WBmotion->addCOMInfo_xy_pelz_HB(COM_total.x, COM_total.y, WBmotion->pPel_3x1[2]);
            WBmotion->addRFPosInfo_HB(GGSW.pRF_ref.x, GGSW.pRF_ref.y, GGSW.pRF_ref.z);
            WBmotion->addLFPosInfo_HB(GGSW.pLF_ref.x, GGSW.pLF_ref.y, GGSW.pLF_ref.z);
            WBmotion->addRFOriInfo_HB(GGSW.qRF_ref);
            WBmotion->addLFOriInfo_HB(GGSW.qLF_ref);
            WBmotion->addPELOriInfo_HB(GGSW.qPel_ref);

            WST_ref_deg = GGSW.WST_ref_deg;

            save_onestep_ggsw(GGSW.k);

            //// Set shared memory variables
            if(GGSW.step_phase_change_flag == true)
            {
                //send result one step
                switch(GGSW.ROSWalk_status)
                {
                case ROSWALK_START:
                {
                    if(userData->ros_step_num < 2 && GGSW.ROSWalk_off_flag == false)
                    {
                        FILE_LOG(logERROR) << "ROS Step_phase is null";
                        GGSW.ROSWalk_off_flag = true;
                    }
                    break;
                }
                case ROSWALK_STEP_DONE:
                {
                    printf("stepping done? %d\n",userData->FLAG_receivedROS);
                    userData->FLAG_sendROS = CMD_DONE;
                    GGSW.ROSWalk_status = ROSWALK_START;
                    break;
                }
                case ROSWALK_STEP_PASS:
                {
                    printf("step pass\n");
                    GGSW.ROSWalk_status = ROSWALK_START;
                    break;
                }
                case ROSWALK_WALKING_DONE:
                {
                    printf("walking done? %d\n",userData->FLAG_receivedROS);
                    userData->FLAG_sendROS = CMD_WALKING_FINISHED;
                    GGSW.ROSWalk_status = ROSWALK_BREAK;
                    break;
                }
                case ROSWALK_FALL_DONE:
                {
                    printf("falling done? %d\n",userData->FLAG_receivedROS);
                    userData->FLAG_sendROS = CMD_ERROR;
                    GGSW.ROSWalk_status = ROSWALK_BREAK;
                    break;
                }
                }
            }

            userData->pel_pose[0] = GGSW.COM_m_filtered[0];
            userData->pel_pose[1] = GGSW.COM_m_filtered[1];
            userData->pel_pose[2] = GGSW.COM_m_filtered[2];

            userData->step_phase = GGSW.step_phase;
            userData->lr_state = GGSW.R_or_L;
            break;
        }
        case _task_ROS_Walk:
        {
            //// Put sensor value into RSEN object
            SensorInput();

            //// State Estimation
            RST = SE.StateEst(RSEN);
            GGSW.MeasurementInput(RST);

            //// Main Walking code
            if(GGSW.Preveiw_walking() == -1)
            {
                _task_thread = _task_Idle;
                userData->FLAG_receivedROS = ROS_RX_EMPTY;
                printf("receive empty walking done\n");

                save_all_gg();
                cout<<"Preview Walk finished"<<endl;
            }

            vec3 COM_total = GGSW.uCOM;

            //// Leg Vibration Control
            LHY_con_deg = -GGSW.LHY_con_deg;
            RHY_con_deg = -GGSW.RHY_con_deg;

            LHR_con_deg = 1.0*GGSW.LHR_con_deg + GGSW.L_roll_compen_deg;
            RHR_con_deg = 1.0*GGSW.RHR_con_deg + GGSW.R_roll_compen_deg;

            LHP_con_deg = 1.0*GGSW.LHP_con_deg*0.5;
            RHP_con_deg = 1.0*GGSW.RHP_con_deg*0.5;
            LKN_con_deg = GGSW.L_knee_compen_deg;
            RKN_con_deg = GGSW.R_knee_compen_deg;


            ////Foot and Pelv Orientation
            //foot Orientation
            for(int i=0;i<4;i++)
            {
                dbs_qRF[i] = GGSW.qRF_ref[i];
                dbs_qLF[i] = GGSW.qLF_ref[i];
            }


            // Pelvis Orientation
            for(int i=0;i<4;i++)
            {
                dbs_qPel[i] = GGSW.qPel_ref[i];
            }

            //// Put reference Task to Trajectory Handler
            WBmotion->addCOMInfo_xy_pelz_HB(COM_total.x, COM_total.y, WBmotion->pPel_3x1[2]);
            WBmotion->addRFPosInfo_HB(GGSW.pRF_ref.x, GGSW.pRF_ref.y, GGSW.pRF_ref.z);
            WBmotion->addLFPosInfo_HB(GGSW.pLF_ref.x, GGSW.pLF_ref.y, GGSW.pLF_ref.z);
            WBmotion->addRFOriInfo_HB(GGSW.qRF_ref);
            WBmotion->addLFOriInfo_HB(GGSW.qLF_ref);
            WBmotion->addPELOriInfo_HB(GGSW.qPel_ref);

            WST_ref_deg = GGSW.WST_ref_deg;

            save_onestep_ggsw(GGSW.k);

            //// Set shared memory variables

            if(GGSW.ROSWalk_status == ROSWALK_STEP_DONE)
            {
                printf("stepping done? %d\n",userData->FLAG_receivedROS);
                userData->FLAG_sendROS = CMD_DONE;
                GGSW.ROSWalk_status = ROSWALK_START;
            }else if(GGSW.ROSWalk_status == ROSWALK_WALKING_DONE)
            {
                printf("walking done? %d\n",userData->FLAG_receivedROS);
                userData->FLAG_sendROS = CMD_WALKING_FINISHED;
                GGSW.ROSWalk_status = ROSWALK_BREAK;
            }

            if(GGSW.ROSWalk_flag == true && userData->ros_footstep_flag == true)
            {
                if(userData->ros_step_num < 2 && userData->FLAG_receivedROS != ROS_RX_FALSE && GGSW.ROSWalk_off_flag == false)
                {
                    FILE_LOG(logERROR) << "ROS Step_phase is null";
                    GGSW.ROSWalk_off_flag = true;
                }
            }

            userData->step_phase = GGSW.step_phase;
            userData->lr_state = GGSW.R_or_L;
            break;
        }
        case _task_Idle:
        case _task_Idle_SingleLog:

            break;
        }

    if(WB_FLAG == 1)
    {
        WBmotion->updateAll_HB();
        WBmotion->WBIK_xy_pelz();

        joint->SetJointRefAngle(WST, WST_ref_deg);

        //////////////////////////////////--------------Right leg------------////////////////////////////////////////
        double RHY_ref_deg = WBmotion->LJ.RHY*R2D + RHY_con_deg;
        if(RHY_ref_deg > 30.0) RHY_ref_deg = 30.0;
        if(RHY_ref_deg < -30.0) RHY_ref_deg = -30.0;

        joint->SetJointRefAngle(RHY,RHY_ref_deg);
        joint->SetJointRefAngle(RHR,WBmotion->LJ.RHR*R2D + RHR_con_deg);
        joint->SetJointRefAngle(RHP,WBmotion->LJ.RHP*R2D + RHP_con_deg);
        joint->SetJointRefAngle(RKN,WBmotion->LJ.RKN*R2D + RKN_con_deg);

        if(_task_thread == _task_SingleLog_Walk || _task_thread == _task_Idle_SingleLog)
        {
            GK.IK_Ankle_right(WBmotion->LJ.RAP*R2D + GGSW.RF_angle_ctrl.y*R2D, WBmotion->LJ.RAR*R2D + GGSW.RF_angle_ctrl.x*R2D, RA1_ref_deg, RA2_ref_deg);
        }else
        {
            GK.IK_Ankle_right(WBmotion->LJ.RAP*R2D + HBPW.RF_angle_ctrl.y*R2D, WBmotion->LJ.RAR*R2D + HBPW.RF_angle_ctrl.x*R2D, RA1_ref_deg, RA2_ref_deg);
        }
        joint->SetJointRefAngle(RAP, RA1_ref_deg);
        joint->SetJointRefAngle(RAR, RA2_ref_deg);


        //////////////////////////////////--------------Left leg------------////////////////////////////////////////
        double LHY_ref_deg = WBmotion->LJ.LHY*R2D + LHY_con_deg;
        if(LHY_ref_deg > 30.0) LHY_ref_deg = 30.0;
        if(LHY_ref_deg < -30.0) LHY_ref_deg = -30.0;

        joint->SetJointRefAngle(LHY,LHY_ref_deg);
        joint->SetJointRefAngle(LHR,WBmotion->LJ.LHR*R2D + LHR_con_deg);
        joint->SetJointRefAngle(LHP,WBmotion->LJ.LHP*R2D + LHP_con_deg);
        joint->SetJointRefAngle(LKN,WBmotion->LJ.LKN*R2D + LKN_con_deg);

        if(_task_thread == _task_SingleLog_Walk || _task_thread == _task_Idle_SingleLog)
        {
            GK.IK_Ankle_left(WBmotion->LJ.LAP*R2D + GGSW.LF_angle_ctrl.y*R2D, WBmotion->LJ.LAR*R2D + GGSW.LF_angle_ctrl.x*R2D, LA1_ref_deg, LA2_ref_deg);
        }else
        {
            GK.IK_Ankle_left(WBmotion->LJ.LAP*R2D + HBPW.LF_angle_ctrl.y*R2D, WBmotion->LJ.LAR*R2D + HBPW.LF_angle_ctrl.x*R2D, LA1_ref_deg, LA2_ref_deg);
        }
        joint->SetJointRefAngle(LAP, LA1_ref_deg);
        joint->SetJointRefAngle(LAR, LA2_ref_deg);
    }
    joint->MoveAllJoint();
    rt_task_suspend(&rtTaskCon);
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 200*1000);        // 2 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true)
        {
            joint->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }
    }
}
//==============================//

void ResetGlobalCoord(int RF_OR_LF_OR_PC_MidFoot){
    switch(RF_OR_LF_OR_PC_MidFoot)
    {
    case 1: //MidFoot
    {
        double LowerFootz;
        if(WBmotion->pRF_3x1[2] < WBmotion->pLF_3x1[2] - 0.0001){
            LowerFootz = WBmotion->pRF_3x1[2];
        }
        else if(WBmotion->pRF_3x1[2] > WBmotion->pLF_3x1[2] + 0.0001){
            LowerFootz = WBmotion->pLF_3x1[2];
        }
        else{
            LowerFootz = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2;
        }
        vec3 midfoot = vec3((WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2, (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2, LowerFootz);
        cout<<"midFoot : ("<<midfoot.x<<", "<<midfoot.y<<", "<<midfoot.z<<")"<<endl;

        WBmotion->pCOM_3x1[0] = WBmotion->pCOM_3x1[0] - midfoot.x;
        WBmotion->pCOM_3x1[1] = WBmotion->pCOM_3x1[1] - midfoot.y;
        WBmotion->pCOM_3x1[2] = WBmotion->pCOM_3x1[2]- midfoot.z;

        WBmotion->pRF_3x1[0] = WBmotion->pRF_3x1[0] - midfoot.x;
        WBmotion->pRF_3x1[1] = WBmotion->pRF_3x1[1] - midfoot.y;
        WBmotion->pRF_3x1[2] = WBmotion->pRF_3x1[2] - midfoot.z;

        WBmotion->pLF_3x1[0] = WBmotion->pLF_3x1[0] - midfoot.x;
        WBmotion->pLF_3x1[1] = WBmotion->pLF_3x1[1] - midfoot.y;
        WBmotion->pLF_3x1[2] = WBmotion->pLF_3x1[2] - midfoot.z;

        WBmotion->pPel_3x1[0] = WBmotion->pPel_3x1[0] - midfoot.x;
        WBmotion->pPel_3x1[1] = WBmotion->pPel_3x1[1] - midfoot.y;
        WBmotion->pPel_3x1[2] = WBmotion->pPel_3x1[2] - midfoot.z;

        WBmotion->addCOMInfo_HB(WBmotion->pCOM_3x1[0], WBmotion->pCOM_3x1[1], WBmotion->pCOM_3x1[2]);
        WBmotion->addRFPosInfo_HB(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2]);
        WBmotion->addLFPosInfo_HB(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], WBmotion->pLF_3x1[2]);
        WBmotion->addPELPosInfo_HB(WBmotion->pPel_3x1[2]);
        quat qDefalt = quat();
        WBmotion->addPELOriInfo_HB(qDefalt);
        WBmotion->addRFOriInfo_HB(qDefalt);
        WBmotion->addLFOriInfo_HB(qDefalt);
        WBmotion->updateAll_HB();

        WBmotion->WBIK();

        break;
    }
    }
}

double getEnc(int in)
{

    double a = sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentPosition*M_PI/180.;

    return a;
}
double getPosRef(int in)
{

    double a = sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentReference*M_PI/180.;

    return a;
}
double getEncVel(int in)
{

    double a = sharedSEN->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentVelocity*M_PI/180.;

    return a;
}

void SensorInput()
{
    // Fill in the ROBOTSEN struct
    vec3 F_RF = vec3(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
    vec3 F_LF = vec3(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
//    vec3 F_RF = vec3(0,0,sharedSEN->FT[0].Fz);
//    vec3 F_LF = vec3(0,0,sharedSEN->FT[1].Fz);
    vec3 M_RF = vec3(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
    vec3 M_LF = vec3(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);

    vec3 IMUangle(sharedSEN->IMU[0].Roll,sharedSEN->IMU[0].Pitch,sharedSEN->IMU[0].Yaw);   // Gyro Integration
    vec3 IMUangle_comp(sharedSEN->IMU[0].Roll_Comp,sharedSEN->IMU[0].Pitch_Comp,sharedSEN->IMU[0].Yaw);   // Complementary filter
    //vec3 IMUangle_comp = IMUangle;

    quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
    vec3 IMUvel = vec3(sharedSEN->IMU[0].RollVel,sharedSEN->IMU[0].PitchVel,sharedSEN->IMU[0].YawVel);
    vec3 IMUacc(sharedSEN->IMU[0].AccX,sharedSEN->IMU[0].AccY,sharedSEN->IMU[0].AccZ);
    vec3 ACC_RF = vec3(sharedSEN->FT[0].Pitch*4.0, -sharedSEN->FT[0].Roll*4.0,0);
    vec3 ACC_LF = vec3(sharedSEN->FT[1].Pitch*4.0, -sharedSEN->FT[1].Roll*4.0,0);

    for(int i=0; i<12 ; i++)
    {
        RSEN.JSP.JSP_Array[i] = getEnc(i);
        RSEN.JSV.JSV_Array[i] = getEncVel(i);
    }
    //for gazelle
    double RAP_deg, RAR_deg;
    GK.FK_diff_Ankle_right(getEnc(RAP)*R2D, getEnc(RAR)*R2D, 0, 0, RAP_deg, RAR_deg);
    RSEN.JSP.RAP = RAP_deg*D2R;
    RSEN.JSP.RAR = RAR_deg*D2R;

    RSEN.JSP.RA1 = getEnc(RAP);
    RSEN.JSP.RA2 = getEnc(RAR);

    double LAP_deg, LAR_deg;
    GK.FK_diff_Ankle_left(getEnc(LAP)*R2D, getEnc(LAR)*R2D, 0, 0, LAP_deg, LAR_deg);
    RSEN.JSP.LAP = LAP_deg*D2R;
    RSEN.JSP.LAR = LAR_deg*D2R;

    RSEN.JSP.LA1 = getEnc(LAP);
    RSEN.JSP.LA2 = getEnc(LAR);

    double RAP_vel_deg, RAR_vel_deg;
    GK.FK_diff_Ankle_vel(getEncVel(RAP)*R2D,getEncVel(RAR)*R2D, getEnc(RAP)*R2D, getEnc(RAR)*R2D, RAP_deg, RAR_deg, RAP_vel_deg, RAR_vel_deg);
    RSEN.JSV.dRAP = RAP_vel_deg*D2R;
    RSEN.JSV.dRAR = RAR_vel_deg*D2R;

    RSEN.JSV.dRA1 = getEncVel(RAP);
    RSEN.JSV.dRA2 = getEncVel(RAR);

    double LAP_vel_deg, LAR_vel_deg;
    GK.FK_diff_Ankle_vel(getEncVel(LAP)*R2D,getEncVel(LAR)*R2D, getEnc(LAP)*R2D, getEnc(LAR)*R2D, LAP_deg, LAR_deg, LAP_vel_deg, LAR_vel_deg);
    RSEN.JSV.dLAP = LAP_vel_deg*D2R;
    RSEN.JSV.dLAR = LAR_vel_deg*D2R;

    RSEN.JSV.dLA1 = getEncVel(LAP);
    RSEN.JSV.dLA2 = getEncVel(LAR);


    RSEN.F_RF = F_RF;
    RSEN.F_LF = F_LF;
    RSEN.M_RF = M_RF;
    RSEN.M_LF = M_LF;

    RSEN.ACC_RF = ACC_RF;
    RSEN.ACC_LF = ACC_LF;

    RSEN.IMUangle = IMUangle*D2R;
    RSEN.IMUangle_comp = IMUangle_comp*D2R;

    RSEN.IMUquat = IMUquat;
    RSEN.IMULocalW = IMUvel*D2R;
    RSEN.IMUomega = mat3(RSEN.IMUquat)*RSEN.IMULocalW;

    //------------------------------------------------------------------------------
    
}

void Torque2Choreonoid(DesiredStates _Des_State)
{
    for(int i=0 ; i<12 ; i++){
        double GR = 160;
        //cout<<"_Des_State.ddQdes"<<_Des_State.ddQdes<<endl;
        //sharedSEN->ENCODER[MC_ID_CH_Pairs[0].id][MC_ID_CH_Pairs[0].ch].CurrentTorqueRef = 0.1;//_Des_State.Tdes[i] + _Des_State.ddQdes[i+6]*GR*GR*7.0e-6;
        sharedCMD->Choreonoid.LTorque[i] = _Des_State.Tdes[i] + _Des_State.ddQdes[i+6]*GR*GR*7.0e-6;
    }
}



void save_onestep(int cnt)
{
    SAVE[0][cnt] = HBPW.COM_ref.x;
    SAVE[1][cnt] = HBPW.CP_ref.x;
    SAVE[2][cnt] = HBPW.COM_ref.y;
    SAVE[3][cnt] = HBPW.CP_ref.y;
    SAVE[4][cnt] = HBPW.p_ref[0].x;
    SAVE[5][cnt] = HBPW.p_ref[0].y;
    SAVE[6][cnt] = HBPW.p_out.x;
    SAVE[7][cnt] = HBPW.p_out.y;
    SAVE[8][cnt] = HBPW.COM_LIPM.x;
    SAVE[9][cnt] = HBPW.COM_LIPM.y;

    SAVE[10][cnt] = HBPW.dCOM_ref.x;
    SAVE[11][cnt] = HBPW.dCOM_ref.y;
    SAVE[12][cnt] = HBPW.cZMP.x;
    SAVE[13][cnt] = HBPW.cZMP.y;
    SAVE[14][cnt] = HBPW.ZMP_global.x;
    SAVE[15][cnt] = HBPW.ZMP_global.y;
    SAVE[16][cnt] = HBPW.cZMP_proj.x;
    SAVE[17][cnt] = HBPW.cZMP_proj.y;

    SAVE[18][cnt] = HBPW.RS.IMUangle.x;
    SAVE[19][cnt] = HBPW.RS.IMUangle.y;
    SAVE[20][cnt] = HBPW.RS.IMUangle.z;

    SAVE[21][cnt] = HBPW.X_obs[0];
    SAVE[22][cnt] = HBPW.Y_obs[0];
    SAVE[23][cnt] = HBPW.X_obs[1];
    SAVE[24][cnt] = HBPW.Y_obs[1];
    SAVE[25][cnt] = HBPW.RS.IMULocalW.x;
    SAVE[26][cnt] = HBPW.RS.IMULocalW.y;

    SAVE[27][cnt] = HBPW.dT;//dT_est;//dT_buf[0];
    SAVE[28][cnt] = HBPW.pRF_ref.x;
    SAVE[29][cnt] = HBPW.pLF_ref.x;

    SAVE[30][cnt] = HBPW.pRF_ref.z;
    SAVE[31][cnt] = HBPW.pLF_ref.z;
    SAVE[32][cnt] = HBPW.ZMP_error_local.y;

    SAVE[33][cnt] = HBPW.RS.CSP.pCOM.x; // COM_measure x
    SAVE[34][cnt] = HBPW.RS.CSP.pCOM.y; // COM_measure y

    SAVE[35][cnt] = HBPW.uCOM.x;
    SAVE[36][cnt] = HBPW.uCOM.y;

    SAVE[37][cnt] = HBPW.CP_m_filtered.x; // temp x
    SAVE[38][cnt] = HBPW.CP_m_filtered.y; // temp y

    SAVE[39][cnt] = HBPW.COM_error_local.y;
    SAVE[40][cnt] = HBPW.SDB[HBPW.step_phase].swingFoot;

    SAVE[41][cnt] = HBPW.COM_m.x;
    SAVE[42][cnt] = HBPW.COM_m.y;
    SAVE[43][cnt] = HBPW.COM_damping_con.x;
    SAVE[44][cnt] = HBPW.COM_damping_con.y;

    SAVE[45][cnt] = HBPW.CP_m.y; // ZMP x estimation
    SAVE[46][cnt] = HBPW.CP_m.x;
    SAVE[47][cnt] = 0;
    SAVE[48][cnt] = HBPW.pRF_ref.x;
    SAVE[49][cnt] = HBPW.pRF_ref.y;

    SAVE[50][cnt] = HBPW.RS.F_RF.z;
    SAVE[51][cnt] = HBPW.RS.F_LF.z;
    SAVE[52][cnt] = HBPW.pLF_ref.x;
    SAVE[53][cnt] = HBPW.pLF_ref.y;
    SAVE[54][cnt] = HBPW.RS.CSV.dpCOM.y;

    SAVE[55][cnt] = HBPW.RS.CSV.dpCOM.x;
    SAVE[56][cnt] = HBPW.RS.CSV.dpCOM.y;
    SAVE[57][cnt] = HBPW.Fz_diff_error;

    SAVE[58][cnt] = HBPW.ACC_RF_filtered.x;
    SAVE[59][cnt] = HBPW.ACC_RF_filtered.y;
    SAVE[60][cnt] = HBPW.ACC_LF_filtered.x;
    SAVE[61][cnt] = HBPW.ACC_LF_filtered.y;

    SAVE[62][cnt] = HBPW.Ye_obs[0];
    SAVE[63][cnt] = HBPW.Ye_obs[1];
    SAVE[64][cnt] = HBPW.Xe_obs[0];
    SAVE[65][cnt] = HBPW.Xe_obs[1];
    SAVE[66][cnt] = HBPW.L_roll_obs[0];
    SAVE[67][cnt] = HBPW.LHR_con_deg;

    SAVE[68][cnt] = HBPW.dCOM_m_diff.x;
    SAVE[69][cnt] = HBPW.dCOM_m_diff.y;
    SAVE[70][cnt] = HBPW.dCOM_m_imu.x;
    SAVE[71][cnt] = HBPW.dCOM_m_imu.y;
    SAVE[72][cnt] = 0;
    SAVE[73][cnt] = 0;

    SAVE[74][cnt] = HBPW.RS.M_RF.x;
    SAVE[75][cnt] = HBPW.RS.M_RF.y;
    SAVE[76][cnt] = HBPW.RS.F_RF.z;
    SAVE[77][cnt] = HBPW.RS.M_LF.x;
    SAVE[78][cnt] = HBPW.RS.M_LF.y;
    SAVE[79][cnt] = HBPW.RS.F_LF.z;

    SAVE[80][cnt] = HBPW.RF_z_dz_ddz[0];
    SAVE[81][cnt] = HBPW.RF_z_dz_ddz[1];
    SAVE[82][cnt] = HBPW.LF_z_dz_ddz[0];
    SAVE[83][cnt] = HBPW.LF_z_dz_ddz[1];
    SAVE[84][cnt] = HBPW.RF_landing_flag;
    SAVE[85][cnt] = HBPW.LF_landing_flag;

    SAVE[86][cnt] = HBPW.pRF_landing.z;
    SAVE[87][cnt] = HBPW.pLF_landing.z;
    SAVE[88][cnt] = 0;
    SAVE[89][cnt] = 0;

    SAVE[90][cnt] = HBPW.COM_y_dy_ddy_SA[0];
    SAVE[91][cnt] = HBPW.COM_y_dy_ddy_SA[1];
    SAVE[92][cnt] = HBPW.COM_y_dy_ddy_SA[2];
    SAVE[93][cnt] = HBPW.COM_x_dx_ddx_SA[0];
    SAVE[94][cnt] = HBPW.COM_x_dx_ddx_SA[1];
    SAVE[95][cnt] = HBPW.COM_x_dx_ddx_SA[2];
    SAVE[96][cnt] = HBPW.p_out_SA.y;
    SAVE[97][cnt] = HBPW.p_out_SA.x;
    SAVE[98][cnt] = HBPW.p_ref_SA[0].y;
    SAVE[99][cnt] = HBPW.p_ref_SA[0].x;

    SAVE[100][cnt] = HBPW.COM_SA_ref.y;
    SAVE[101][cnt] = HBPW.COM_SA_ref.x;
    SAVE[102][cnt] = HBPW.dCOM_SA_ref.y;
    SAVE[103][cnt] = HBPW.dCOM_SA_ref.x;
    SAVE[104][cnt] = HBPW.t_now;
    SAVE[105][cnt] = HBPW.CP_SA_ref_local.x;
    SAVE[106][cnt] = HBPW.CP_SA_ref_local.y;
    SAVE[107][cnt] = 0;
    SAVE[108][cnt] = HBPW.Landing_delXY.x;
    SAVE[109][cnt] = HBPW.Landing_delXY.y;

    SAVE[110][cnt] = HBPW.CP_error_lf.x;
    SAVE[111][cnt] = HBPW.CP_error_lf.y;
    SAVE[112][cnt] = HBPW.cZMP_SA_lf.x;
    SAVE[113][cnt] = HBPW.cZMP_SA_lf.y;
    SAVE[114][cnt] = HBPW.RF_y_dy_ddy[0];
    SAVE[115][cnt] = HBPW.RF_y_dy_ddy[1];
    SAVE[116][cnt] = HBPW.pRF_landing.x;
    SAVE[117][cnt] = HBPW.pRF_landing.y;
    SAVE[118][cnt] = 0;
    SAVE[119][cnt] = 0;

    SAVE[120][cnt] = HBPW.CP_error_rf.x;
    SAVE[121][cnt] = HBPW.CP_error_rf.y;
    SAVE[122][cnt] = HBPW.cZMP_SA_rf.x;
    SAVE[123][cnt] = HBPW.cZMP_SA_rf.y;
    SAVE[124][cnt] = HBPW.LF_y_dy_ddy[0];
    SAVE[125][cnt] = HBPW.LF_y_dy_ddy[1];
    SAVE[126][cnt] = HBPW.pLF_landing.x;
    SAVE[127][cnt] = HBPW.pLF_landing.y;
    SAVE[128][cnt] = 0;
    SAVE[129][cnt] = 0;

    SAVE[130][cnt] = HBPW.del_u_f.x;
    SAVE[131][cnt] = HBPW.del_u_f.y;
    SAVE[132][cnt] = HBPW.del_b_f.x;
    SAVE[133][cnt] = HBPW.del_b_f.y;
    SAVE[134][cnt] = HBPW.new_T;
    SAVE[135][cnt] = HBPW.del_u_g.x;
    SAVE[136][cnt] = HBPW.del_u_g.y;
    SAVE[137][cnt] = HBPW.del_b_g.x;
    SAVE[138][cnt] = HBPW.del_b_g.y;
    SAVE[139][cnt] = HBPW.SA_Enable_flag;

    SAVE[140][cnt] = HBPW.del_u_f_filtered.x;
    SAVE[141][cnt] = HBPW.del_u_f_filtered.y;
    SAVE[142][cnt] = HBPW.del_b_f_filtered.x;
    SAVE[143][cnt] = HBPW.del_b_f_filtered.y;
    SAVE[144][cnt] = HBPW.new_T_filtered;
    SAVE[145][cnt] = HBPW.UD_flag;
    SAVE[146][cnt] = HBPW.LandingZ_des;
    SAVE[147][cnt] = 0;
    SAVE[148][cnt] = HBPW.Pelv_roll_ref;
    SAVE[149][cnt] = HBPW.Pelv_pitch_ref;

    SAVE[150][cnt] = HBPW.Omega_pitch;
    SAVE[151][cnt] = HBPW.Omega_roll;
    SAVE[152][cnt] = HBPW.Omega_pitch_filtered;
    SAVE[153][cnt] = HBPW.Omega_roll_filtered;
    SAVE[154][cnt] = HBPW.del_b0_Nf.x;
    SAVE[155][cnt] = HBPW.del_b0_Nf.y;
    SAVE[156][cnt] = HBPW.b0_Nf.x;
    SAVE[157][cnt] = HBPW.b0_Nf.y;
    SAVE[158][cnt] = HBPW.SDB[HBPW.step_phase].t;
    SAVE[159][cnt] = HBPW.T_nom;

    SAVE[160][cnt] = HBPW.del_u_Nf.x;
    SAVE[161][cnt] = HBPW.del_u_Nf.y;
    SAVE[162][cnt] = HBPW.del_b_Nf.x;
    SAVE[163][cnt] = HBPW.del_b_Nf.y;
    SAVE[164][cnt] = HBPW.new_T_Nf;
    SAVE[165][cnt] = HBPW.del_u_Nf_filtered.x;
    SAVE[166][cnt] = HBPW.del_u_Nf_filtered.y;
    SAVE[167][cnt] = HBPW.del_b_Nf_filtered.x;
    SAVE[168][cnt] = HBPW.del_b_Nf_filtered.y;
    SAVE[169][cnt] = HBPW.new_T_Nf_filtered;

    SAVE[170][cnt] = HBPW.dz_com_ctrl;
    SAVE[171][cnt] = HBPW.z_com_ctrl;
    SAVE[172][cnt] = 0;
    SAVE[173][cnt] = 0;
    SAVE[174][cnt] = 0;
    SAVE[175][cnt] = 0;
    SAVE[176][cnt] = 0;
    SAVE[177][cnt] = HBPW.G_R_g_pitroll_rpy.r;
    SAVE[178][cnt] = HBPW.G_R_g_pitroll_rpy.p;
    SAVE[179][cnt] = HBPW.G_R_g_pitroll_rpy.y;

    SAVE[180][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHY].id][MC_ID_CH_Pairs[RHY].ch].CurrentPosition;
    SAVE[181][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHR].id][MC_ID_CH_Pairs[RHR].ch].CurrentPosition;
    SAVE[182][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentPosition;
    SAVE[183][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition;
    SAVE[184][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition;
    SAVE[185][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition;
    SAVE[186][cnt] = 0;
    SAVE[187][cnt] = 0;
    SAVE[188][cnt] = 0;
    SAVE[189][cnt] = 0;

    SAVE[190][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHY].id][MC_ID_CH_Pairs[LHY].ch].CurrentPosition;
    SAVE[191][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHR].id][MC_ID_CH_Pairs[LHR].ch].CurrentPosition;
    SAVE[192][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentPosition;
    SAVE[193][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LKN].id][MC_ID_CH_Pairs[LKN].ch].CurrentPosition;
    SAVE[194][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition;
    SAVE[195][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition;
    SAVE[196][cnt] = 0;
    SAVE[197][cnt] = 0;
    SAVE[198][cnt] = 0;
    SAVE[199][cnt] = 0;

    SAVE[200][cnt] = WBmotion->LJ.RHY*R2D + RHY_con_deg;
    SAVE[201][cnt] = WBmotion->LJ.RHR*R2D + RHR_con_deg;
    SAVE[202][cnt] = WBmotion->LJ.RHP*R2D + RHP_con_deg;
    SAVE[203][cnt] = WBmotion->LJ.RKN*R2D;
    SAVE[204][cnt] = RA1_ref_deg;  //RAP board
    SAVE[205][cnt] = RA2_ref_deg;  //RAR board
    SAVE[206][cnt] = 0;
    SAVE[207][cnt] = RHY_con_deg;
    SAVE[208][cnt] = RHR_con_deg;
    SAVE[209][cnt] = RHP_con_deg;

    SAVE[210][cnt] = WBmotion->LJ.LHY*R2D + LHY_con_deg;
    SAVE[211][cnt] = WBmotion->LJ.LHR*R2D + LHR_con_deg;
    SAVE[212][cnt] = WBmotion->LJ.LHP*R2D + LHP_con_deg;
    SAVE[213][cnt] = WBmotion->LJ.LKN*R2D;
    SAVE[214][cnt] = LA1_ref_deg; //LAP board
    SAVE[215][cnt] = LA2_ref_deg; //LAR board
    SAVE[216][cnt] = 0;
    SAVE[217][cnt] = LHY_con_deg;
    SAVE[218][cnt] = LHR_con_deg;
    SAVE[219][cnt] = LHP_con_deg;

    SAVE[220][cnt] = HBPW.AnkleTorque_ref[0]; //RAR ref torque
    SAVE[221][cnt] = HBPW.AnkleTorque_ref[1]; //RAP ref torque
    SAVE[222][cnt] = HBPW.AnkleTorque_ref[2]; //LAR ref torque
    SAVE[223][cnt] = HBPW.AnkleTorque_ref[3]; //LAP ref torque
    SAVE[224][cnt] = HBPW.RF_Fz_ref;
    SAVE[225][cnt] = HBPW.LF_Fz_ref;
    SAVE[226][cnt] = 0;
    SAVE[227][cnt] = 0;
    SAVE[228][cnt] = 0;
    SAVE[229][cnt] = HBPW.Alpha_dsp;

    SAVE[230][cnt] = HBPW.DSP_time_flag;
    SAVE[231][cnt] = HBPW.DSP_force_flag;
    SAVE[232][cnt] = HBPW.RF_Fz_ref;
    SAVE[233][cnt] = HBPW.LF_Fz_ref;
    SAVE[234][cnt] = HBPW.Fz_diff_ref;
    SAVE[235][cnt] = HBPW.Fz_diff;
    SAVE[236][cnt] = HBPW.dz_ctrl;
    SAVE[237][cnt] = HBPW.dz_ctrl_filtered;
    SAVE[238][cnt] = HBPW.z_ctrl;
    SAVE[239][cnt] = HBPW.z_ctrl_filtered;

    SAVE[240][cnt] = HBPW.dRF_angle_ctrl.x;
    SAVE[241][cnt] = HBPW.dRF_angle_ctrl.y;
    SAVE[242][cnt] = HBPW.RF_angle_ctrl.x;
    SAVE[243][cnt] = HBPW.RF_angle_ctrl.y;
    SAVE[244][cnt] = HBPW.dLF_angle_ctrl.x;
    SAVE[245][cnt] = HBPW.dLF_angle_ctrl.y;
    SAVE[246][cnt] = HBPW.LF_angle_ctrl.x;
    SAVE[247][cnt] = HBPW.LF_angle_ctrl.y;
    SAVE[248][cnt] = 0;
    SAVE[249][cnt] = 0;

    SAVE[250][cnt] = HBPW.RS.IMUangle_comp.x;
    SAVE[251][cnt] = HBPW.RS.IMUangle_comp.y;
    SAVE[252][cnt] = HBPW.COM_m_comp.x;
    SAVE[253][cnt] = HBPW.COM_m_comp.y;
    SAVE[254][cnt] = HBPW.dCOM_m_comp.x;
    SAVE[255][cnt] = HBPW.dCOM_m_comp.y;
    SAVE[256][cnt] = HBPW.CP_m_comp.x;
    SAVE[257][cnt] = HBPW.CP_m_comp.y;
    SAVE[258][cnt] = HBPW.cZMP_TC_proj.x;
    SAVE[259][cnt] = HBPW.cZMP_TC_proj.y;

    SAVE[260][cnt] = HBPW.COM_e_imu_local.x;
    SAVE[261][cnt] = HBPW.COM_e_imu_local.y;
    SAVE[262][cnt] = HBPW.dCOM_e_imu_local.x;
    SAVE[263][cnt] = HBPW.dCOM_e_imu_local.y;
    SAVE[264][cnt] = HBPW.dsp_ctrl_gain;
    SAVE[265][cnt] = HBPW.dsp_tilt_gain;
    SAVE[266][cnt] = HBPW.cZMP_dsp_global.y;
    SAVE[267][cnt] = HBPW.cZMP_dsp_global.x;
    SAVE[268][cnt] = HBPW.cZMP_dsp_global_proj.y;
    SAVE[269][cnt] = HBPW.cZMP_dsp_global_proj.x;

    SAVE[270][cnt] = HBPW.COM_m_comp_filtered.x;
    SAVE[271][cnt] = HBPW.COM_m_comp_filtered.y;
    SAVE[272][cnt] = HBPW.dCOM_m_comp_filtered.x;
    SAVE[273][cnt] = HBPW.dCOM_m_comp_filtered.y;
    SAVE[274][cnt] = HBPW.Pelv_pitch_acc_ref;
    SAVE[275][cnt] = HBPW.Pelv_pitch_vel_ref;
    SAVE[276][cnt] = HBPW.Pelv_pitch_ref;
    SAVE[277][cnt] = HBPW.Pelv_roll_acc_ref;
    SAVE[278][cnt] = HBPW.Pelv_roll_vel_ref;
    SAVE[279][cnt] = HBPW.Pelv_roll_ref;

    SAVE[280][cnt] = HBPW.p_ref_con_error_filtered;
    SAVE[281][cnt] = HBPW.COM_m_filtered.x;
    SAVE[282][cnt] = HBPW.COM_m_filtered.y;
    SAVE[283][cnt] = HBPW.COM_m_filtered.z;
    SAVE[284][cnt] = HBPW.dCOM_m_filtered.x;
    SAVE[285][cnt] = HBPW.dCOM_m_filtered.y;
    SAVE[286][cnt] = HBPW.dCOM_m_filtered.z;
    SAVE[287][cnt] = HBPW.COM_SA_LIPM.x;
    SAVE[288][cnt] = HBPW.COM_SA_LIPM.y;
    SAVE[289][cnt] = HBPW.COM_SA_LIPM.z;
    SAVE[290][cnt] = HBPW.dCOM_SA_LIPM.x;
    SAVE[291][cnt] = HBPW.dCOM_SA_LIPM.y;
    SAVE[292][cnt] = HBPW.dCOM_SA_LIPM.z;
    SAVE[293][cnt] = HBPW.p_ref_offset_y;
    SAVE[294][cnt] = HBPW.R_roll_obs.x;
    SAVE[295][cnt] = HBPW.R_roll_obs.y;
    SAVE[296][cnt] = HBPW.R_roll_obs.z;
    SAVE[297][cnt] = HBPW.L_roll_obs.x;
    SAVE[298][cnt] = HBPW.L_roll_obs.y;
    SAVE[299][cnt] = HBPW.L_roll_obs.z;
    SAVE[300][cnt] = HBPW.RF_landing_angle.r;
    SAVE[301][cnt] = HBPW.RF_landing_angle.p;
    SAVE[302][cnt] = HBPW.RF_landing_angle.y;
    SAVE[303][cnt] = HBPW.LF_landing_angle.r;
    SAVE[304][cnt] = HBPW.LF_landing_angle.p;
    SAVE[305][cnt] = HBPW.LF_landing_angle.y;
    SAVE[306][cnt] = HBPW.X_RTorsoYaw.x;
    SAVE[307][cnt] = HBPW.X_RTorsoYaw.y;
    SAVE[308][cnt] = HBPW.X_RTorsoYaw.z;
    SAVE[309][cnt] = HBPW.X_LTorsoYaw.x;
    SAVE[310][cnt] = HBPW.X_LTorsoYaw.y;
    SAVE[311][cnt] = HBPW.X_LTorsoYaw.z;
    
    SAVE[312][cnt] = HBPW.R_roll_compen_deg;
    SAVE[313][cnt] = HBPW.L_roll_compen_deg;
    SAVE[314][cnt] = HBPW.R_knee_compen_deg;
    SAVE[315][cnt] = HBPW.L_knee_compen_deg;


    SAVE[316][cnt] = HBPW.Y_zmp_e_sum;
    SAVE[317][cnt] = HBPW.X_zmp_e_sum;
    SAVE[318][cnt] = HBPW.p_ref_for_COM[0].y;
    SAVE[319][cnt] = HBPW.p_ref_for_COM[0].x;

    SAVE[320][cnt] = HBPW.RS.IMULocalW.z;

    SAVE[321][cnt] = LHY_con_deg;
    SAVE[322][cnt] = RHY_con_deg;
    SAVE[323][cnt] = LHR_con_deg;
    SAVE[324][cnt] = RHR_con_deg;
    SAVE[325][cnt] = LHP_con_deg;
    SAVE[326][cnt] = RHP_con_deg;
    SAVE[327][cnt] = LKN_con_deg;
    SAVE[328][cnt] = RKN_con_deg;

    SAVE[328][cnt] = HBPW.ZMP_local.x;
    SAVE[329][cnt] = HBPW.ZMP_local.y;
    SAVE[330][cnt] = HBPW.ZMP_local.z;

}

void save_onestep_ggsw(int cnt)
{
    SAVE_GG[0][cnt] = GGSW.COM_ref.x;
    SAVE_GG[1][cnt] = GGSW.CP_ref.x;
    SAVE_GG[2][cnt] = GGSW.COM_ref.y;
    SAVE_GG[3][cnt] = GGSW.CP_ref.y;
    SAVE_GG[4][cnt] = GGSW.p_ref[0].x;
    SAVE_GG[5][cnt] = GGSW.p_ref[0].y;
    SAVE_GG[6][cnt] = GGSW.p_out.x;
    SAVE_GG[7][cnt] = GGSW.p_out.y;
    SAVE_GG[8][cnt] = GGSW.COM_LIPM.x;
    SAVE_GG[9][cnt] = GGSW.COM_LIPM.y;

    SAVE_GG[10][cnt] = GGSW.dCOM_ref.x;
    SAVE_GG[11][cnt] = GGSW.dCOM_ref.y;
    SAVE_GG[12][cnt] = GGSW.cZMP.x;
    SAVE_GG[13][cnt] = GGSW.cZMP.y;
    SAVE_GG[14][cnt] = GGSW.ZMP_global.x;
    SAVE_GG[15][cnt] = GGSW.ZMP_global.y;
    SAVE_GG[16][cnt] = GGSW.cZMP_proj.x;
    SAVE_GG[17][cnt] = GGSW.cZMP_proj.y;

    SAVE_GG[18][cnt] = GGSW.RS.IMUangle.x;
    SAVE_GG[19][cnt] = GGSW.RS.IMUangle.y;
    SAVE_GG[20][cnt] = GGSW.RS.IMUangle.z;

    SAVE_GG[21][cnt] = GGSW.X_obs[0];
    SAVE_GG[22][cnt] = GGSW.Y_obs[0];
    SAVE_GG[23][cnt] = GGSW.X_obs[1];
    SAVE_GG[24][cnt] = GGSW.Y_obs[1];
    SAVE_GG[25][cnt] = GGSW.RS.IMULocalW.x;
    SAVE_GG[26][cnt] = GGSW.RS.IMULocalW.y;

    SAVE_GG[27][cnt] = GGSW.dT;//dT_est;//dT_buf[0];
    SAVE_GG[28][cnt] = GGSW.pRF_ref.x;
    SAVE_GG[29][cnt] = GGSW.pLF_ref.x;

    SAVE_GG[30][cnt] = GGSW.pRF_ref.z;
    SAVE_GG[31][cnt] = GGSW.pLF_ref.z;
    SAVE_GG[32][cnt] = GGSW.ZMP_error_local.y;

    SAVE_GG[33][cnt] = GGSW.RS.CSP.pCOM.x; // COM_measure x
    SAVE_GG[34][cnt] = GGSW.RS.CSP.pCOM.y; // COM_measure y

    SAVE_GG[35][cnt] = GGSW.uCOM.x;
    SAVE_GG[36][cnt] = GGSW.uCOM.y;

    SAVE_GG[37][cnt] = GGSW.CP_m_filtered.x; // temp x
    SAVE_GG[38][cnt] = GGSW.CP_m_filtered.y; // temp y

    SAVE_GG[39][cnt] = GGSW.COM_error_local.y;
    SAVE_GG[40][cnt] = GGSW.SDB[GGSW.step_phase].swingFoot;

    SAVE_GG[41][cnt] = GGSW.COM_m.x;
    SAVE_GG[42][cnt] = GGSW.COM_m.y;
    SAVE_GG[43][cnt] = GGSW.COM_damping_con.x;
    SAVE_GG[44][cnt] = GGSW.COM_damping_con.y;

    SAVE_GG[45][cnt] = GGSW.CP_m.y; // ZMP x estimation
    SAVE_GG[46][cnt] = GGSW.CP_m.x;
    SAVE_GG[47][cnt] = 0;
    SAVE_GG[48][cnt] = GGSW.pRF_ref.x;
    SAVE_GG[49][cnt] = GGSW.pRF_ref.y;

    SAVE_GG[50][cnt] = GGSW.RS.F_RF.z;
    SAVE_GG[51][cnt] = GGSW.RS.F_LF.z;
    SAVE_GG[52][cnt] = GGSW.pLF_ref.x;
    SAVE_GG[53][cnt] = GGSW.pLF_ref.y;
    SAVE_GG[54][cnt] = GGSW.RS.CSV.dpCOM.y;

    SAVE_GG[55][cnt] = GGSW.RS.CSV.dpCOM.x;
    SAVE_GG[56][cnt] = GGSW.RS.CSV.dpCOM.y;
    SAVE_GG[57][cnt] = GGSW.Fz_diff_error;

    SAVE_GG[58][cnt] = GGSW.ACC_RF_filtered.x;
    SAVE_GG[59][cnt] = GGSW.ACC_RF_filtered.y;
    SAVE_GG[60][cnt] = GGSW.ACC_LF_filtered.x;
    SAVE_GG[61][cnt] = GGSW.ACC_LF_filtered.y;

    SAVE_GG[62][cnt] = GGSW.Ye_obs[0];
    SAVE_GG[63][cnt] = GGSW.Ye_obs[1];
    SAVE_GG[64][cnt] = GGSW.Xe_obs[0];
    SAVE_GG[65][cnt] = GGSW.Xe_obs[1];
    SAVE_GG[66][cnt] = GGSW.L_roll_obs[0];
    SAVE_GG[67][cnt] = GGSW.LHR_con_deg;

    SAVE_GG[68][cnt] = GGSW.dCOM_m_diff.x;
    SAVE_GG[69][cnt] = GGSW.dCOM_m_diff.y;
    SAVE_GG[70][cnt] = GGSW.dCOM_m_imu.x;
    SAVE_GG[71][cnt] = GGSW.dCOM_m_imu.y;
    SAVE_GG[72][cnt] = 0;
    SAVE_GG[73][cnt] = 0;

    SAVE_GG[74][cnt] = GGSW.RS.M_RF.x;
    SAVE_GG[75][cnt] = GGSW.RS.M_RF.y;
    SAVE_GG[76][cnt] = GGSW.RS.F_RF.z;
    SAVE_GG[77][cnt] = GGSW.RS.M_LF.x;
    SAVE_GG[78][cnt] = GGSW.RS.M_LF.y;
    SAVE_GG[79][cnt] = GGSW.RS.F_LF.z;

    SAVE_GG[80][cnt] = GGSW.RF_z_dz_ddz[0];
    SAVE_GG[81][cnt] = GGSW.RF_z_dz_ddz[1];
    SAVE_GG[82][cnt] = GGSW.LF_z_dz_ddz[0];
    SAVE_GG[83][cnt] = GGSW.LF_z_dz_ddz[1];
    SAVE_GG[84][cnt] = GGSW.RF_landing_flag;
    SAVE_GG[85][cnt] = GGSW.LF_landing_flag;

    SAVE_GG[86][cnt] = GGSW.pRF_landing.z;
    SAVE_GG[87][cnt] = GGSW.pLF_landing.z;
    SAVE_GG[88][cnt] = 0;
    SAVE_GG[89][cnt] = 0;

    SAVE_GG[90][cnt] = GGSW.COM_y_dy_ddy_SA[0];
    SAVE_GG[91][cnt] = GGSW.COM_y_dy_ddy_SA[1];
    SAVE_GG[92][cnt] = GGSW.COM_y_dy_ddy_SA[2];
    SAVE_GG[93][cnt] = GGSW.COM_x_dx_ddx_SA[0];
    SAVE_GG[94][cnt] = GGSW.COM_x_dx_ddx_SA[1];
    SAVE_GG[95][cnt] = GGSW.COM_x_dx_ddx_SA[2];
    SAVE_GG[96][cnt] = GGSW.p_out_SA.y;
    SAVE_GG[97][cnt] = GGSW.p_out_SA.x;
    SAVE_GG[98][cnt] = GGSW.p_ref_SA[0].y;
    SAVE_GG[99][cnt] = GGSW.p_ref_SA[0].x;

    SAVE_GG[100][cnt] = GGSW.COM_SA_ref.y;
    SAVE_GG[101][cnt] = GGSW.COM_SA_ref.x;
    SAVE_GG[102][cnt] = GGSW.dCOM_SA_ref.y;
    SAVE_GG[103][cnt] = GGSW.dCOM_SA_ref.x;
    SAVE_GG[104][cnt] = GGSW.t_now;
    SAVE_GG[105][cnt] = GGSW.CP_SA_ref_local.x;
    SAVE_GG[106][cnt] = GGSW.CP_SA_ref_local.y;
    SAVE_GG[107][cnt] = 0;
    SAVE_GG[108][cnt] = GGSW.Landing_delXY.x;
    SAVE_GG[109][cnt] = GGSW.Landing_delXY.y;

    SAVE_GG[110][cnt] = GGSW.CP_error_lf.x;
    SAVE_GG[111][cnt] = GGSW.CP_error_lf.y;
    SAVE_GG[112][cnt] = GGSW.cZMP_SA_lf.x;
    SAVE_GG[113][cnt] = GGSW.cZMP_SA_lf.y;
    SAVE_GG[114][cnt] = GGSW.RF_y_dy_ddy[0];
    SAVE_GG[115][cnt] = GGSW.RF_y_dy_ddy[1];
    SAVE_GG[116][cnt] = GGSW.pRF_landing.x;
    SAVE_GG[117][cnt] = GGSW.pRF_landing.y;
    SAVE_GG[118][cnt] = 0;
    SAVE_GG[119][cnt] = 0;

    SAVE_GG[120][cnt] = GGSW.CP_error_rf.x;
    SAVE_GG[121][cnt] = GGSW.CP_error_rf.y;
    SAVE_GG[122][cnt] = GGSW.cZMP_SA_rf.x;
    SAVE_GG[123][cnt] = GGSW.cZMP_SA_rf.y;
    SAVE_GG[124][cnt] = GGSW.LF_y_dy_ddy[0];
    SAVE_GG[125][cnt] = GGSW.LF_y_dy_ddy[1];
    SAVE_GG[126][cnt] = GGSW.pLF_landing.x;
    SAVE_GG[127][cnt] = GGSW.pLF_landing.y;
    SAVE_GG[128][cnt] = 0;
    SAVE_GG[129][cnt] = 0;

    SAVE_GG[130][cnt] = GGSW.del_u_f.x;
    SAVE_GG[131][cnt] = GGSW.del_u_f.y;
    SAVE_GG[132][cnt] = GGSW.del_b_f.x;
    SAVE_GG[133][cnt] = GGSW.del_b_f.y;
    SAVE_GG[134][cnt] = GGSW.new_T;
    SAVE_GG[135][cnt] = GGSW.del_u_g.x;
    SAVE_GG[136][cnt] = GGSW.del_u_g.y;
    SAVE_GG[137][cnt] = GGSW.del_b_g.x;
    SAVE_GG[138][cnt] = GGSW.del_b_g.y;
    SAVE_GG[139][cnt] = GGSW.SA_Enable_flag;

    SAVE_GG[140][cnt] = GGSW.del_u_f_filtered.x;
    SAVE_GG[141][cnt] = GGSW.del_u_f_filtered.y;
    SAVE_GG[142][cnt] = GGSW.del_b_f_filtered.x;
    SAVE_GG[143][cnt] = GGSW.del_b_f_filtered.y;
    SAVE_GG[144][cnt] = GGSW.new_T_filtered;
    SAVE_GG[145][cnt] = GGSW.UD_flag;
    SAVE_GG[146][cnt] = GGSW.LandingZ_des;
    SAVE_GG[147][cnt] = 0;
    SAVE_GG[148][cnt] = GGSW.Pelv_roll_ref;
    SAVE_GG[149][cnt] = GGSW.Pelv_pitch_ref;

    SAVE_GG[150][cnt] = GGSW.Omega_pitch;
    SAVE_GG[151][cnt] = GGSW.Omega_roll;
    SAVE_GG[152][cnt] = GGSW.Omega_pitch_filtered;
    SAVE_GG[153][cnt] = GGSW.Omega_roll_filtered;
    SAVE_GG[154][cnt] = GGSW.del_b0_Nf.x;
    SAVE_GG[155][cnt] = GGSW.del_b0_Nf.y;
    SAVE_GG[156][cnt] = GGSW.b0_Nf.x;
    SAVE_GG[157][cnt] = GGSW.b0_Nf.y;
    SAVE_GG[158][cnt] = GGSW.SDB[GGSW.step_phase].t;
    SAVE_GG[159][cnt] = GGSW.T_nom;

    SAVE_GG[160][cnt] = GGSW.del_u_Nf.x;
    SAVE_GG[161][cnt] = GGSW.del_u_Nf.y;
    SAVE_GG[162][cnt] = GGSW.del_b_Nf.x;
    SAVE_GG[163][cnt] = GGSW.del_b_Nf.y;
    SAVE_GG[164][cnt] = GGSW.new_T_Nf;
    SAVE_GG[165][cnt] = GGSW.del_u_Nf_filtered.x;
    SAVE_GG[166][cnt] = GGSW.del_u_Nf_filtered.y;
    SAVE_GG[167][cnt] = GGSW.del_b_Nf_filtered.x;
    SAVE_GG[168][cnt] = GGSW.del_b_Nf_filtered.y;
    SAVE_GG[169][cnt] = GGSW.new_T_Nf_filtered;

    SAVE_GG[170][cnt] = GGSW.dz_com_ctrl;
    SAVE_GG[171][cnt] = GGSW.z_com_ctrl;
    SAVE_GG[172][cnt] = 0;
    SAVE_GG[173][cnt] = 0;
    SAVE_GG[174][cnt] = 0;
    SAVE_GG[175][cnt] = 0;
    SAVE_GG[176][cnt] = 0;
    SAVE_GG[177][cnt] = GGSW.G_R_g_pitroll_rpy.r;
    SAVE_GG[178][cnt] = GGSW.G_R_g_pitroll_rpy.p;
    SAVE_GG[179][cnt] = GGSW.G_R_g_pitroll_rpy.y;

    SAVE_GG[180][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHY].id][MC_ID_CH_Pairs[RHY].ch].CurrentPosition;
    SAVE_GG[181][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHR].id][MC_ID_CH_Pairs[RHR].ch].CurrentPosition;
    SAVE_GG[182][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentPosition;
    SAVE_GG[183][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition;
    SAVE_GG[184][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition;
    SAVE_GG[185][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition;
    SAVE_GG[186][cnt] = 0;
    SAVE_GG[187][cnt] = 0;
    SAVE_GG[188][cnt] = 0;
    SAVE_GG[189][cnt] = 0;

    SAVE_GG[190][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHY].id][MC_ID_CH_Pairs[LHY].ch].CurrentPosition;
    SAVE_GG[191][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHR].id][MC_ID_CH_Pairs[LHR].ch].CurrentPosition;
    SAVE_GG[192][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentPosition;
    SAVE_GG[193][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LKN].id][MC_ID_CH_Pairs[LKN].ch].CurrentPosition;
    SAVE_GG[194][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition;
    SAVE_GG[195][cnt] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition;
    SAVE_GG[196][cnt] = 0;
    SAVE_GG[197][cnt] = 0;
    SAVE_GG[198][cnt] = 0;
    SAVE_GG[199][cnt] = 0;

    SAVE_GG[200][cnt] = WBmotion->LJ.RHY*R2D + RHY_con_deg;
    SAVE_GG[201][cnt] = WBmotion->LJ.RHR*R2D + RHR_con_deg;
    SAVE_GG[202][cnt] = WBmotion->LJ.RHP*R2D + RHP_con_deg;
    SAVE_GG[203][cnt] = WBmotion->LJ.RKN*R2D;
    SAVE_GG[204][cnt] = RA1_ref_deg;  //RAP board
    SAVE_GG[205][cnt] = RA2_ref_deg;  //RAR board
    SAVE_GG[206][cnt] = 0;
    SAVE_GG[207][cnt] = RHY_con_deg;
    SAVE_GG[208][cnt] = RHR_con_deg;
    SAVE_GG[209][cnt] = RHP_con_deg;

    SAVE_GG[210][cnt] = WBmotion->LJ.LHY*R2D + LHY_con_deg;
    SAVE_GG[211][cnt] = WBmotion->LJ.LHR*R2D + LHR_con_deg;
    SAVE_GG[212][cnt] = WBmotion->LJ.LHP*R2D + LHP_con_deg;
    SAVE_GG[213][cnt] = WBmotion->LJ.LKN*R2D;
    SAVE_GG[214][cnt] = LA1_ref_deg; //LAP board
    SAVE_GG[215][cnt] = LA2_ref_deg; //LAR board
    SAVE_GG[216][cnt] = 0;
    SAVE_GG[217][cnt] = LHY_con_deg;
    SAVE_GG[218][cnt] = LHR_con_deg;
    SAVE_GG[219][cnt] = LHP_con_deg;

    SAVE_GG[220][cnt] = GGSW.AnkleTorque_ref[0]; //RAR ref torque
    SAVE_GG[221][cnt] = GGSW.AnkleTorque_ref[1]; //RAP ref torque
    SAVE_GG[222][cnt] = GGSW.AnkleTorque_ref[2]; //LAR ref torque
    SAVE_GG[223][cnt] = GGSW.AnkleTorque_ref[3]; //LAP ref torque
    SAVE_GG[224][cnt] = GGSW.RF_Fz_ref;
    SAVE_GG[225][cnt] = GGSW.LF_Fz_ref;
    SAVE_GG[226][cnt] = 0;
    SAVE_GG[227][cnt] = 0;
    SAVE_GG[228][cnt] = 0;
    SAVE_GG[229][cnt] = GGSW.Alpha_dsp;

    SAVE_GG[230][cnt] = GGSW.DSP_time_flag;
    SAVE_GG[231][cnt] = GGSW.DSP_force_flag;
    SAVE_GG[232][cnt] = GGSW.RF_Fz_ref;
    SAVE_GG[233][cnt] = GGSW.LF_Fz_ref;
    SAVE_GG[234][cnt] = GGSW.Fz_diff_ref;
    SAVE_GG[235][cnt] = GGSW.Fz_diff;
    SAVE_GG[236][cnt] = GGSW.dz_ctrl;
    SAVE_GG[237][cnt] = GGSW.dz_ctrl_filtered;
    SAVE_GG[238][cnt] = GGSW.z_ctrl;
    SAVE_GG[239][cnt] = GGSW.z_ctrl_filtered;

    SAVE_GG[240][cnt] = GGSW.dRF_angle_ctrl.x;
    SAVE_GG[241][cnt] = GGSW.dRF_angle_ctrl.y;
    SAVE_GG[242][cnt] = GGSW.RF_angle_ctrl.x;
    SAVE_GG[243][cnt] = GGSW.RF_angle_ctrl.y;
    SAVE_GG[244][cnt] = GGSW.dLF_angle_ctrl.x;
    SAVE_GG[245][cnt] = GGSW.dLF_angle_ctrl.y;
    SAVE_GG[246][cnt] = GGSW.LF_angle_ctrl.x;
    SAVE_GG[247][cnt] = GGSW.LF_angle_ctrl.y;
    SAVE_GG[248][cnt] = 0;
    SAVE_GG[249][cnt] = 0;

    SAVE_GG[250][cnt] = GGSW.RS.IMUangle_comp.x;
    SAVE_GG[251][cnt] = GGSW.RS.IMUangle_comp.y;
    SAVE_GG[252][cnt] = GGSW.COM_m_comp.x;
    SAVE_GG[253][cnt] = GGSW.COM_m_comp.y;
    SAVE_GG[254][cnt] = GGSW.dCOM_m_comp.x;
    SAVE_GG[255][cnt] = GGSW.dCOM_m_comp.y;
    SAVE_GG[256][cnt] = GGSW.CP_m_comp.x;
    SAVE_GG[257][cnt] = GGSW.CP_m_comp.y;
    SAVE_GG[258][cnt] = GGSW.cZMP_TC_proj.x;
    SAVE_GG[259][cnt] = GGSW.cZMP_TC_proj.y;

    SAVE_GG[260][cnt] = GGSW.COM_e_imu_local.x;
    SAVE_GG[261][cnt] = GGSW.COM_e_imu_local.y;
    SAVE_GG[262][cnt] = GGSW.dCOM_e_imu_local.x;
    SAVE_GG[263][cnt] = GGSW.dCOM_e_imu_local.y;
    SAVE_GG[264][cnt] = GGSW.dsp_ctrl_gain;
    SAVE_GG[265][cnt] = GGSW.dsp_tilt_gain;
    SAVE_GG[266][cnt] = GGSW.cZMP_dsp_global.y;
    SAVE_GG[267][cnt] = GGSW.cZMP_dsp_global.x;
    SAVE_GG[268][cnt] = GGSW.cZMP_dsp_global_proj.y;
    SAVE_GG[269][cnt] = GGSW.cZMP_dsp_global_proj.x;

    SAVE_GG[270][cnt] = GGSW.COM_m_comp_filtered.x;
    SAVE_GG[271][cnt] = GGSW.COM_m_comp_filtered.y;
    SAVE_GG[272][cnt] = GGSW.dCOM_m_comp_filtered.x;
    SAVE_GG[273][cnt] = GGSW.dCOM_m_comp_filtered.y;
    SAVE_GG[274][cnt] = GGSW.Pelv_pitch_acc_ref;
    SAVE_GG[275][cnt] = GGSW.Pelv_pitch_vel_ref;
    SAVE_GG[276][cnt] = GGSW.Pelv_pitch_ref;
    SAVE_GG[277][cnt] = GGSW.Pelv_roll_acc_ref;
    SAVE_GG[278][cnt] = GGSW.Pelv_roll_vel_ref;
    SAVE_GG[279][cnt] = GGSW.Pelv_roll_ref;

    SAVE_GG[280][cnt] = GGSW.p_ref_con_error_filtered;
    SAVE_GG[281][cnt] = GGSW.COM_m_filtered.x;
    SAVE_GG[282][cnt] = GGSW.COM_m_filtered.y;
    SAVE_GG[283][cnt] = GGSW.COM_m_filtered.z;
    SAVE_GG[284][cnt] = GGSW.dCOM_m_filtered.x;
    SAVE_GG[285][cnt] = GGSW.dCOM_m_filtered.y;
    SAVE_GG[286][cnt] = GGSW.dCOM_m_filtered.z;
    SAVE_GG[287][cnt] = GGSW.COM_SA_LIPM.x;
    SAVE_GG[288][cnt] = GGSW.COM_SA_LIPM.y;
    SAVE_GG[289][cnt] = GGSW.COM_SA_LIPM.z;
    SAVE_GG[290][cnt] = GGSW.dCOM_SA_LIPM.x;
    SAVE_GG[291][cnt] = GGSW.dCOM_SA_LIPM.y;
    SAVE_GG[292][cnt] = GGSW.dCOM_SA_LIPM.z;
    SAVE_GG[293][cnt] = GGSW.p_ref_offset_y;
    SAVE_GG[294][cnt] = GGSW.R_roll_obs.x;
    SAVE_GG[295][cnt] = GGSW.R_roll_obs.y;
    SAVE_GG[296][cnt] = GGSW.R_roll_obs.z;
    SAVE_GG[297][cnt] = GGSW.L_roll_obs.x;
    SAVE_GG[298][cnt] = GGSW.L_roll_obs.y;
    SAVE_GG[299][cnt] = GGSW.L_roll_obs.z;
    SAVE_GG[300][cnt] = GGSW.RF_landing_angle.r;
    SAVE_GG[301][cnt] = GGSW.RF_landing_angle.p;
    SAVE_GG[302][cnt] = GGSW.RF_landing_angle.y;
    SAVE_GG[303][cnt] = GGSW.LF_landing_angle.r;
    SAVE_GG[304][cnt] = GGSW.LF_landing_angle.p;
    SAVE_GG[305][cnt] = GGSW.LF_landing_angle.y;
    SAVE_GG[306][cnt] = GGSW.X_RTorsoYaw.x;
    SAVE_GG[307][cnt] = GGSW.X_RTorsoYaw.y;
    SAVE_GG[308][cnt] = GGSW.X_RTorsoYaw.z;
    SAVE_GG[309][cnt] = GGSW.X_LTorsoYaw.x;
    SAVE_GG[310][cnt] = GGSW.X_LTorsoYaw.y;
    SAVE_GG[311][cnt] = GGSW.X_LTorsoYaw.z;

    SAVE_GG[312][cnt] = GGSW.R_roll_compen_deg;
    SAVE_GG[313][cnt] = GGSW.L_roll_compen_deg;
    SAVE_GG[314][cnt] = GGSW.R_knee_compen_deg;
    SAVE_GG[315][cnt] = GGSW.L_knee_compen_deg;

    SAVE_GG[316][cnt] = GGSW.Y_zmp_e_sum;
    SAVE_GG[317][cnt] = GGSW.X_zmp_e_sum;
    SAVE_GG[318][cnt] = GGSW.p_ref_for_COM[0].y;
    SAVE_GG[319][cnt] = GGSW.p_ref_for_COM[0].x;

    SAVE_GG[320][cnt] = GGSW.RS.IMULocalW.z;

    SAVE_GG[321][cnt] = LHY_con_deg;
    SAVE_GG[322][cnt] = RHY_con_deg;
    SAVE_GG[323][cnt] = LHR_con_deg;
    SAVE_GG[324][cnt] = RHR_con_deg;
    SAVE_GG[325][cnt] = LHP_con_deg;
    SAVE_GG[326][cnt] = RHP_con_deg;
    SAVE_GG[327][cnt] = LKN_con_deg;
    SAVE_GG[328][cnt] = RKN_con_deg;

    SAVE_GG[328][cnt] = GGSW.ZMP_local.x;
    SAVE_GG[329][cnt] = GGSW.ZMP_local.y;
    SAVE_GG[330][cnt] = GGSW.ZMP_local.z;
    SAVE_GG[331][cnt] = GGSW.MaxFoot_y_cur;
    SAVE_GG[332][cnt] = userData->FLAG_receivedROS;
    SAVE_GG[333][cnt] = userData->FLAG_sendROS;

    SAVE_GG[334][cnt] = GGSW.step_status;
    SAVE_GG[335][cnt] = GGSW.ROSWalk_status;

    SAVE_GG[336][cnt] = userData->pel_pose[0];
    SAVE_GG[337][cnt] = userData->pel_pose[1];
    SAVE_GG[338][cnt] = userData->pel_pose[2];

    SAVE_GG[339][cnt] = WBmotion->pPel_3x1[0];
    SAVE_GG[340][cnt] = WBmotion->pPel_3x1[1];
    SAVE_GG[341][cnt] = WBmotion->pPel_3x1[2];
    SAVE_GG[342][cnt] = GGSW.step_phase_change_flag;
}

void save_all()
{
    printf("walk finished and saved%d\n",HBPW.k);
    FILE* ffp2 = fopen("/home/rainbow/Desktop/HBtest_Walking_Data_prev1.txt","w");
    for(int i=0;i<HBPW.k;i++)
    {
        for(int j=0;j<SAVEN;j++)
        {
            fprintf(ffp2,"%f\t",SAVE[j][i]);
        }
        fprintf(ffp2,"\n");
    }

    fclose(ffp2);
    printf("save done\n");
}

void save_all_gg()
{
    printf("ggwalk finished and saved%d\n",GGSW.k);
    FILE* ffp3 = fopen("/home/rainbow/Desktop/HBtest_Walking_Data_prev2.txt","w");
    for(int i=0;i<GGSW.k;i++)
    {
        for(int j=0;j<SAVEN;j++)
        {
            fprintf(ffp3,"%f\t",SAVE_GG[j][i]);
        }
        fprintf(ffp3,"\n");
    }

    fclose(ffp3);
    printf("gg save done\n");
}

void init_StateEstimator()
{
    ////Initialize State Estimator--------------------------------------------------------------------
    vec3 F_RF(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
    vec3 F_LF(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
    vec3 M_RF(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,0);
    vec3 M_LF(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,0);
    quat IMUquat(sharedSEN->IMU[0].Q[0], sharedSEN->IMU[0].Q[1], sharedSEN->IMU[0].Q[2], sharedSEN->IMU[0].Q[3]);
    RST_ini.F_RF = F_RF; RST_ini.F_LF = F_LF; RST_ini.M_RF = M_RF; RST_ini.M_LF = M_LF;

    RST_ini.cRF = false; RST_ini.cLF = false;
    if(F_RF.norm() > 20) RST_ini.cRF = true;
    if(F_LF.norm() > 20) RST_ini.cLF = true;
    RST_ini.CSP.pPel = vec3(WBmotion->pPel_3x1); RST_ini.CSP.qPel = IMUquat; RST_ini.CSP.pCOM = vec3(WBmotion->pCOM_3x1);
    RST_ini.CSP.pRF = vec3(WBmotion->pRF_3x1); RST_ini.CSP.qRF = quat(WBmotion->qRF_4x1);
    RST_ini.CSP.pLF = vec3(WBmotion->pLF_3x1); RST_ini.CSP.qLF = quat(WBmotion->qLF_4x1);

    RST_ini.CSV.dpPel = vec3(); RST_ini.CSV.dqPel = vec3(); RST_ini.CSV.dpCOM  =vec3();
    RST_ini.CSV.dpRF = vec3(); RST_ini.CSV.dqRF = vec3();
    RST_ini.CSV.dpLF = vec3(); RST_ini.CSV.dqLF = vec3();
    RST_ini.JSP.pPel = RST_ini.CSP.pPel; RST_ini.JSP.qPel = RST_ini.CSP.qPel;
    RST_ini.JSV.dpPel = vec3(); RST_ini.JSV.dqPel = vec3();

    for(int i=0; i<12; i++)
    {
        RST_ini.JSP.JSP_Array[i] = getEnc(i);
        RST_ini.JSV.JSV_Array[i] = 0.0;
    }
    //for gazelle
    double RAP_deg, RAR_deg;
    GK.FK_diff_Ankle_right(getEnc(RAP)*R2D, getEnc(RAR)*R2D, 0, 0, RAP_deg, RAR_deg);
    RST_ini.JSP.JSP_Array[RAP] = RAP_deg*D2R;
    RST_ini.JSP.JSP_Array[RAR] = RAR_deg*D2R;

    double LAP_deg, LAR_deg;
    GK.FK_diff_Ankle_left(getEnc(LAP)*R2D, getEnc(LAR)*R2D, 0, 0, LAP_deg, LAR_deg);
    RST_ini.JSP.JSP_Array[LAP] = LAP_deg*D2R;
    RST_ini.JSP.JSP_Array[LAR] = LAR_deg*D2R;

    RST_ini.Qnow = RST_ini.getQnow(bp_rbdl.Robot->q_size);
    RST_ini.dQnow = RST_ini.getdQnow(bp_rbdl.Robot->qdot_size);

    SE.Initialize_StateEst(RST_ini);
    ////---------------------------------------------------------------------------------------------------------------
}

doubles quat2doubles(quat _quat){
    doubles q;
    for(int i=0;i<4;i++){
        q[i] = _quat[i];
    }
    return q;
}
