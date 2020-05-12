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

//hyoin

//#include "../../SHARE/Headers/ik_math2.h"
#include "../../../share/Headers/kine_drc_hubo2.h"
#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/kine_drc_hubo4.h"
#include "ManualCAN.h"
//#include "Oinverse.h"
#include "HB_inverse.h"
#include "BasicMatrix.h"
//#include "kine_drc_hubo_tj.h"
#include "joint.h"

#include"../../Gazelle_Ankle_Kine/Gazelle_kine.h"

#define PODO_AL_NAME       "WalkReady"
#define SAVEN       390

using namespace std;
const int RB_SUCCESS = true;
const int RB_FAIL = false;

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
#define ROW_data_debug2 20000
#define COL_data_debug2 100
double   JW_Data_Debug[COL_data_debug2][ROW_data_debug2];
double   JW_Data_Debug_Filtered[COL_data_debug2][ROW_data_debug2];
int HitMotionCnt = 0;
const static int ReadyStart = 0;
const static int ReadyEnd = 1200;
const static int HitStart = ReadyEnd+1;
const static int HitEnd = 4400;


void save(int cnt);
void save_all();
int save_cnt = 0;

int cntforcountsleep = 0;
void countsleep(int microsec)
{
    cntforcountsleep = 0;
    while(microsec/5.0/1000.0>cntforcountsleep)
    {
        usleep(1*1000);//1ms sleep
    }
}


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
Gazelle_Kine GK;
// =====================================================

// Command Set =========================================
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY,
    WALKREADY_HIT_READY,
    WALKREADY_HIT_HIT,
    WALKREADY_HIT_RETURN,
    WALKREADY_HIT_INIT_POS,
    WALKREADY_WST_RESPOND,
    WALKREADY_SAVE
};
// =====================================================

int WB_FLAG = 0;
void DemoReady();
void GotoWalkReadyPos();
void GotoWalkReadyPos_OI();
void GotoWalkReadyPos_OneLeg(int left_or_right);
void GotoJumpReady();
void GotoHomePos();

void WBIK_PARA_CHANGE();

// Manual CAN for GainOverride
//int	PushCANMessage(MANUAL_CAN MCData);
//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration);
//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2);
//int RBindicator(unsigned char number);//0 off n n*100ms on/off -1 on 0x85 0,-1 on off
// hyoin
//CKINE_DRC_HUBO_TJ kine_drc_hubo;
CKINE_DRC_HUBO2 kine_drc_hubo;
KINE_DRC_HUBO4 kine_drc_hubo4;
HB_inverse kine_oi;
ArmJoints AJ;
LegJoints LJ;

double WBIK_Q0[34] = {0.,};
double WBIK_Q[34] = {0.,},Qub[34]={0.,};

int infinityTest = false;
int infinityToggle = 0;
int infinityCnt = 0;

void WholebodyInitialize();
void get_zmp2();

double  X_ZMP_Local,Y_ZMP_Local,X_ZMP_Global,Y_ZMP_Global,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_REF_Global,Y_ZMP_REF_Global;
double GLOBAL_ZMP_REF_X,GLOBAL_ZMP_REF_Y;
double com1,com2;
double t_thread=0.,t0,t1,t2;
double lamda;
double T10,T21;
double a1,b1,c1;
double a2,b2,c2;
double zc,xc,yc,xRF,yRF,zRF,xLF,yLF,zLF;
double LeftPitch;
double RSR_demo,LSR_demo;
double AddJoint;

double des_pCOM_3x1[3]={0,}, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3]={0,}, des_qRF_4x1[4]={1,0,0,0}, des_pLF_3x1[3]={0,}, des_qLF_4x1[4]={1,0,0,0};
int DemoFlag;

double I_ZMP_CON_X=0.f,I_ZMP_CON_Y=0.f;
double I_ZMP_CON_X_last=0.f,I_ZMP_CON_Y_last=0.f;
double Old_I_ZMP_CON_X=0.f,Old_I_ZMP_CON_Y=0.f;
double  LPF_Del_PC_X_DSP_XZMP_CON = 0., LPF_Del_PC_Y_DSP_YZMP_CON = 0.;
double  LPF_Del_PC_X_SSP_XZMP_CON = 0., LPF_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON = 0., Del_PC_Y_DSP_YZMP_CON = 0., Old_Del_PC_X_DSP_XZMP_CON = 0., Old_Del_PC_Y_DSP_YZMP_CON = 0.,
        Del_PC_X_SSP_XZMP_CON = 0., Del_PC_Y_SSP_YZMP_CON = 0., Old_Del_PC_X_SSP_XZMP_CON = 0., Old_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Old_Del_PC_X_DSP_XZMP_CON2 = 0;
double  Old_Del_PC_Y_DSP_YZMP_CON2 = 0;

const double DEL_T = 0.005;
const double    OFFSET_ELB = -20.0;
const double    OFFSET_RSR = -15.0;
const double    OFFSET_LSR = 15.0;
double curRSR,curLSR;
double  final_gain_DSP_ZMP_CON = 0., final_gain_SSP_ZMP_CON = 0.;
unsigned int CNT_final_gain_DSP_ZMP_CON = 0,  CNT_final_gain_SSP_ZMP_CON = 0;
void Kirk_Control();
void Kirk_Control_ssp();
void ZMP_intergral_control();
// DSP ZMP Controller
double  kirkZMPCon_XP1(double u, double ZMP, int zero);
double  kirkZMPCon_YP1(double u, double ZMP, int zero);
double  kirkZMPCon_XP2(double u, double ZMP, int zero);
double  kirkZMPCon_YP2(double u, double ZMP, int zero);

enum FTNAME{
    RAFT = 0,
    LAFT
};
enum DEMOFLAG
{
    FOOTUP=100,
    KNEEDOWN,
    FOOTDOWN,
    BOW,
    HIT_HIT,
    HIT_READY,
    HIT_RETURN
};


//SAVE
int Scnt = 0;
double SAVE[SAVEN][50000];

int NRL_JOINT_TEST_FLAG = false;
int NRL_JOINT_TEST_STATE = 0;

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
    FILE_LOG(logERROR) << "KK";

	// Termination signal ---------------------------------
	signal(SIGTERM, CatchSignals);   // "kill" from shell
	signal(SIGINT, CatchSignals);    // Ctrl-c
	signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

	// Block memory swapping ------------------------------
	mlockall(MCL_CURRENT|MCL_FUTURE);

    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

//	// Get PODO No. ---------------------------------------
//	if(argc == 1){
//        FILE_LOG(logERROR) << "No input argument";
//        return 0;
//	}
//	else{
//		QString argStr;
//		argStr.sprintf("%s", argv[1]);
//		PODO_NO = argStr.toInt();
//        cout << endl << endl;
//		cout << "======================================================================" << endl;
//		cout << ">>> Process WalkReady is activated..!!" << endl;
//		cout << ">>> PODO NAME: WALKREADY" << endl;
//		cout << ">>> PODO NO: " << PODO_NO << endl;
//        cout << "======================================================================" << endl;
//	}


	// Initialize RBCore -----------------------------------
    if(RBInitialize() == -1)
        isTerminated = -1;

    kine_oi = HB_inverse();

	// User command cheking --------------------------------
	while(isTerminated == 0)
	{
		usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
        case 3050:
            cout << ">>> COMMAND: NRL JOINT TEST" << endl;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)
                NRL_JOINT_TEST_FLAG = true;
            else
                NRL_JOINT_TEST_FLAG = false;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
            break;
        case WALKREADY_SAVE:
        {
            save_all();
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
            break;
        }
		case WALKREADY_GO_HOMEPOS:
			cout << ">>> COMMAND: WALKREADY_GO_HOMEPOS" << endl;

			joint->RefreshToCurrentReference();
			joint->SetAllMotionOwner();
			GotoHomePos();
           // RBindicator(10);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
			break;
		case WALKREADY_GO_WALKREADYPOS:
			cout << ">>> COMMAND: WALKREADY_GO_WALKREADYPOS" << endl;
            sharedCMD->CommandAccept[PODO_NO] = true;
            usleep(500*1000);
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0){
                GotoWalkReadyPos_OI();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 1){
                //right stance
                GotoWalkReadyPos_OneLeg(-1);
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 2){
                GotoJumpReady();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 3){
                //left stance
                GotoWalkReadyPos_OneLeg(1);
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else{
                GotoWalkReadyPos();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
			break;

        case WALKREADY_WST_RESPOND:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;

            cout << ">>> COMMAND: Waist Respond" << endl;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            joint->SetMoveJoint(WST, 5.0, 1000, MOVE_ABSOLUTE);
            usleep(1050*1000);
            joint->SetMoveJoint(WST, 0.0, 700, MOVE_ABSOLUTE);



            break;

		default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
			break;
		}
	}
    FILE_LOG(logERROR) << "Process WalkReady is terminated" << endl;
	return 0;
}
// ------------- -------------------------------------------------------------------------------- //
float target_ang[NO_OF_JOINTS] = {
    -3.0, -15.0, -20.0, 90.0, -80.0, -15.0,
    3.0, 15.0, -20.0, 90.0, -80.0, 15.0,
    60.0, -30.0, -3.0, -100.0, -3.0, -30.0,
    60.0, 30.0, 3.0, -100.0, 3.0, 30.0,
    3.0,
    -3.0, 0.0, 3.0, 0.0,
    0.5, 0.5,
    0.0, 0.0, 0.0, 0.0
};


// --------------------------------------------------------------------------------------------- //
void RBTaskThread(void *)
{
	while(isTerminated == 0)
    {
        save(save_cnt);
        save_cnt++;
        joint->MoveAllJoint();
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

        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
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
void CatchSignals(int _signal)
{
	switch(_signal)
	{
	case SIGHUP:
	case SIGINT:     // Ctrl-c
	case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
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
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
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
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
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
    if(rt_task_create(&rtFlagCon, "WALKREADY_FLAG", 0, 95, 0) == 0){
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
        FILE_LOG(logERROR) << "Fail to create Flag real-time thread";
        return -1;
    }

    if(rt_task_create(&rtTaskCon, "WALKREADY_TASK", 0, 90, 0) == 0){
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

void WBIK_PARA_CHANGE(){
    // NONE
    //hyoin robot
//    kine_drc_hubo4.C_Torso[0] = -0.035-0.015;
//    kine_drc_hubo4.C_Torso[1] =  0;
//    kine_drc_hubo4.m_Torso = 28.6+1;
//    kine_drc_hubo4.m_RightWrist = 3.7 ;
//    kine_drc_hubo4.m_LeftWrist = 3.7 ;

    //jungwoo robot
    kine_drc_hubo4.C_Torso[0] = 0.000941-0.055;//-0.02;
    kine_drc_hubo4.C_Torso[1] =  0;
    kine_drc_hubo4.m_Torso = 24.98723;
    kine_drc_hubo4.m_RightWrist = 4.5;
    kine_drc_hubo4.m_LeftWrist = 4.5;
    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo.L_FOOT = 0.113;
}

void GotoWalkReadyPos(){
    double postime = 3000.0;

    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.80;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

//    Qub[idRSR] = (10.+ OFFSET_RSR)*D2R;
//    Qub[idLSR] = (-10.+ OFFSET_LSR)*D2R;
    Qub[idRSR] = (10.)*D2R;
    Qub[idLSR] = (-10.)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

//    Qub[idREB] = (-130. + OFFSET_ELB)*D2R;
//    Qub[idLEB] = (-130. + OFFSET_ELB)*D2R;
    Qub[idREB] = (-130. )*D2R;
    Qub[idLEB] = (-130. )*D2R;

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
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);

    double total_mass;
   double total_lower_mass;
   total_mass          = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_Torso + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot+kine_drc_hubo.m_LeftLowerArm+kine_drc_hubo.m_RightLowerArm+kine_drc_hubo.m_LeftUpperArm+kine_drc_hubo.m_RightUpperArm + kine_drc_hubo.m_LeftHand + kine_drc_hubo.m_RightHand + kine_drc_hubo.m_LeftWrist+kine_drc_hubo.m_RightWrist;
   total_lower_mass    = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot;
   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
   cout<<"!!!"<<endl;
}

void GotoWalkReadyPos_OI(){
    double postime = 3000.0;

    double RSP_ref,RSR_ref,RSY_ref,REB_ref,RWY_ref,RWP_ref,LSP_ref,LSR_ref,LSY_ref,LEB_ref,LWY_ref,LWP_ref;
    double WST_ref, RWY2_ref, LWY2_ref;

    RSP_ref = 20.0;
    RSR_ref = 0.0;
    RSY_ref = 0.0;
    REB_ref = -30;
    RWY_ref = 0.0;
    RWP_ref = 0.0;

    LSP_ref = 20.0;
    LSR_ref = 0.0;
    LSY_ref = 0.0;
    LEB_ref = -30;
    LWY_ref = 0.0;
    LWP_ref = 0.0;

    WST_ref = 0.0;

    RWY2_ref = 0.0;
    LWY2_ref = 0.0;

    joint->SetMoveJoint(RSP, RSP_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, RSR_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, RSY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, REB_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, RWY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, RWP_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, LSP_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, LSR_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, LSY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, LEB_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, LWY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, LWP_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, WST_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, RWY2_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, LWY2_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    printf("new walkready\n");
    des_pCOM_3x1[0] = 0.0;
    des_pCOM_3x1[1] = 0.0;
    des_pCOM_3x1[2] = 0.74;//0.77;

    des_qPEL_4x1[0] = 1.0;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -(kine_oi.P2HR + kine_oi.HR2HPy);
    des_pRF_3x1[2] = 0;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_oi.P2HR + kine_oi.HR2HPy;
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    //Oinverse version
    AJ.RSP = RSP_ref*D2R;
    AJ.RSR = RSR_ref*D2R + OFFSET_RSR*D2R;
    AJ.RSY = RSY_ref*D2R;
    AJ.REB = REB_ref*D2R + OFFSET_ELB*D2R;
    AJ.RWY = RWY_ref*D2R;
    AJ.RWP = RWP_ref*D2R;
    AJ.RF1 = RWY2_ref*D2R;

    AJ.LSP = LSP_ref*D2R;
    AJ.LSR = LSR_ref*D2R + OFFSET_LSR*D2R;
    AJ.LSY = LSY_ref*D2R;
    AJ.LEB = LEB_ref*D2R + OFFSET_ELB*D2R;
    AJ.LWY = LWY_ref*D2R;
    AJ.LWP = LWP_ref*D2R;
    AJ.LF1 = LWY2_ref*D2R;

    AJ.WST = WST_ref*D2R;

    LJ.RHY = 0;
    LJ.RHR = 0;
    LJ.RHP = -20*D2R;
    LJ.RKN = 40*D2R;
    LJ.RAP = -20*D2R;
    LJ.RAR = 0;

    LJ.LHY = 0;
    LJ.LHR = 0;
    LJ.LHP = -20*D2R;
    LJ.LKN = 40*D2R;
    LJ.LAP = -20*D2R;
    LJ.LAR = 0;

    LJ.pPel = vec3(0,0,0);
    LJ.qPel = quat(vec3(1,0,0),0.0);

    //kine_oi.setUB(kine_oi.FKCOM_UB(AJ));


//    LJ = kine_oi.IK_COM(vec3(des_pCOM_3x1),quat(des_qPEL_4x1)
//                        , vec3(des_pRF_3x1), quat(des_qRF_4x1)
//                        , vec3(des_pLF_3x1), quat(des_qLF_4x1));

    vec3 des_pRF_new = vec3(des_pRF_3x1[0], des_pRF_3x1[1], des_pRF_3x1[2] + 0.000);
    LJ = kine_oi.IK_COM(vec3(des_pCOM_3x1),quat(des_qPEL_4x1),des_pRF_new, quat(des_qRF_4x1),vec3(des_pLF_3x1), quat(des_qLF_4x1));

    cout<<"q1= "<<LJ.RHY<<" q2= "<<LJ.RHR<<" q3= "<<LJ.RHP<<" q4= "<<LJ.RKN<<" q5= "<<LJ.RAP<<" q6= "<<LJ.RAR<<endl;

    LJ.qPel = quat(des_qPEL_4x1);

    joint->SetMoveJoint(RHY, LJ.RHY*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, LJ.RHR*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, LJ.RHP*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, LJ.RKN*R2D, postime, MOVE_ABSOLUTE);

    //for gazelle
    double RA1_deg, RA2_deg;
    GK.IK_Ankle_right(LJ.RAP*R2D, LJ.RAR*R2D, RA1_deg, RA2_deg);
    joint->SetMoveJoint(RAP, RA1_deg, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, RA2_deg, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(RAP, LJ.RAP*R2D, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RAR, LJ.RAR*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, LJ.LHY*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, LJ.LHR*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, LJ.LHP*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, LJ.LKN*R2D, postime, MOVE_ABSOLUTE);

    //for gazelle
    double LA1_deg, LA2_deg;
    GK.IK_Ankle_left(LJ.LAP*R2D, LJ.LAR*R2D, LA1_deg, LA2_deg);
    joint->SetMoveJoint(LAP, LA1_deg, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, LA2_deg, postime, MOVE_ABSOLUTE);
    \
//    joint->SetMoveJoint(LAP, LJ.LAP*R2D, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LAR, LJ.LAR*R2D, postime, MOVE_ABSOLUTE);

    cout<<LJ.RHY*R2D<<endl;
    cout<<LJ.RHR*R2D<<endl;
    cout<<LJ.RHP*R2D<<endl;
    cout<<LJ.RKN*R2D<<endl;
    cout<<LJ.RAP*R2D<<endl;
    cout<<LJ.RAR*R2D<<endl;


   double total_mass;
   double total_lower_mass;
   double m_rleg = kine_oi.m_rhy+kine_oi.m_rhr+kine_oi.m_rhp+kine_oi.m_rkn+kine_oi.m_rap+kine_oi.m_rar;
   double m_lleg = kine_oi.m_lhy+kine_oi.m_lhr+kine_oi.m_lhp+kine_oi.m_lkn+kine_oi.m_lap+kine_oi.m_lar;
   total_lower_mass = m_rleg + m_lleg + kine_oi.m_pel;

   double m_rarm = kine_oi.m_rsp+kine_oi.m_rsr+kine_oi.m_rsy+kine_oi.m_reb+kine_oi.m_rwy+kine_oi.m_rwp+kine_oi.m_rf1;
   double m_larm = kine_oi.m_lsp+kine_oi.m_lsr+kine_oi.m_lsy+kine_oi.m_leb+kine_oi.m_lwy+kine_oi.m_rwp+kine_oi.m_lf1;
   total_mass = total_lower_mass + m_rarm + m_larm + kine_oi.m_torso;

   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
   cout<<"ookkkk"<<endl;
}

void GotoWalkReadyPos_OneLeg(int left_or_right){
    // 1  --> left leg stance
    // -1 --> right leg stance
    double postime = 3000.0;

    double RSP_ref,RSR_ref,RSY_ref,REB_ref,RWY_ref,RWP_ref,LSP_ref,LSR_ref,LSY_ref,LEB_ref,LWY_ref,LWP_ref;
    double WST_ref, RWY2_ref, LWY2_ref;

    RSP_ref = 20.0;
    RSR_ref = 0.0;
    RSY_ref = 0.0;
    REB_ref = -30;
    RWY_ref = 0.0;
    RWP_ref = 0.0;

    LSP_ref = 20.0;
    LSR_ref = 0.0;
    LSY_ref = 0.0;
    LEB_ref = -30;
    LWY_ref = 0.0;
    LWP_ref = 0.0;

    WST_ref = 0.0;

    RWY2_ref = 0.0;
    LWY2_ref = 0.0;

    joint->SetMoveJoint(RSP, RSP_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, RSR_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, RSY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, REB_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, RWY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, RWP_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, LSP_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, LSR_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, LSY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, LEB_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, LWY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, LWP_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, WST_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, RWY2_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, LWY2_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    if(left_or_right == -1){
        des_pCOM_3x1[0] = 0.0;
        des_pCOM_3x1[1] = 0.0;
        des_pCOM_3x1[2] = 0.77;

        des_qPEL_4x1[0] = 1.0;
        des_qPEL_4x1[1] = 0.;
        des_qPEL_4x1[2] = 0.;
        des_qPEL_4x1[3] = 0.;

        des_pRF_3x1[0] = 0.;
        des_pRF_3x1[1] = 0;
        des_pRF_3x1[2] = 0;

        quat qRF_des = quat();//quat(vec3(1,0,0), -1.5*D2R);
        des_qRF_4x1[0] = qRF_des.w;
        des_qRF_4x1[1] = qRF_des.x;
        des_qRF_4x1[2] = qRF_des.y;
        des_qRF_4x1[3] = qRF_des.z;

        des_pLF_3x1[0] = 0.;
        des_pLF_3x1[1] = kine_oi.P2HR*2 + kine_oi.HR2HPy*2;
        des_pLF_3x1[2] = 0.05;//0.0;

        des_qLF_4x1[0] = 1.;
        des_qLF_4x1[1] = 0.;
        des_qLF_4x1[2] = 0.;
        des_qLF_4x1[3] = 0.;
    }
    if(left_or_right == 1){
        des_pCOM_3x1[0] = 0.0;
        des_pCOM_3x1[1] = 0.0;
        des_pCOM_3x1[2] = 0.77;

        des_qPEL_4x1[0] = 1.0;
        des_qPEL_4x1[1] = 0.;
        des_qPEL_4x1[2] = 0.;
        des_qPEL_4x1[3] = 0.;

        des_pRF_3x1[0] = 0.;
        des_pRF_3x1[1] = -kine_oi.P2HR*2 - kine_oi.HR2HPy*2;
        des_pRF_3x1[2] = 0.05;//0.0;

        des_qRF_4x1[0] = 1.;
        des_qRF_4x1[1] = 0.;
        des_qRF_4x1[2] = 0.;
        des_qRF_4x1[3] = 0.;

        des_pLF_3x1[0] = 0.;
        des_pLF_3x1[1] = 0;
        des_pLF_3x1[2] = 0;

        quat qLF_des = quat();//quat(vec3(1,0,0), 1.5*D2R);
        des_qLF_4x1[0] = qLF_des.w;
        des_qLF_4x1[1] = qLF_des.x;
        des_qLF_4x1[2] = qLF_des.y;
        des_qLF_4x1[3] = qLF_des.z;
    }

    //Oinverse version
    AJ.RSP = RSP_ref*D2R;
    AJ.RSR = RSR_ref*D2R + OFFSET_RSR*D2R;
    AJ.RSY = RSY_ref*D2R;
    AJ.REB = REB_ref*D2R + OFFSET_ELB*D2R;
    AJ.RWY = RWY_ref*D2R;
    AJ.RWP = RWP_ref*D2R;
    AJ.RF1 = RWY2_ref*D2R;

    AJ.LSP = LSP_ref*D2R;
    AJ.LSR = LSR_ref*D2R + OFFSET_LSR*D2R;
    AJ.LSY = LSY_ref*D2R;
    AJ.LEB = LEB_ref*D2R + OFFSET_ELB*D2R;
    AJ.LWY = LWY_ref*D2R;
    AJ.LWP = LWP_ref*D2R;
    AJ.LF1 = LWY2_ref*D2R;

    AJ.WST = WST_ref*D2R;

    LJ.RHY = 0;
    LJ.RHR = 0;
    LJ.RHP = -20*D2R;
    LJ.RKN = 40*D2R;
    LJ.RAP = -20*D2R;
    LJ.RAR = 0;

    LJ.LHY = 0;
    LJ.LHR = 0;
    LJ.LHP = -20*D2R;
    LJ.LKN = 40*D2R;
    LJ.LAP = -20*D2R;
    LJ.LAR = 0;

    LJ.pPel = vec3(0,0,0);
    LJ.qPel = quat(vec3(1,0,0),0.0);

    //kine_oi.setUB(kine_oi.FKCOM_UB(AJ));


    LJ = kine_oi.IK_COM(vec3(des_pCOM_3x1),quat(des_qPEL_4x1)
                        , vec3(des_pRF_3x1), quat(des_qRF_4x1)
                        , vec3(des_pLF_3x1), quat(des_qLF_4x1));
    LJ.qPel = quat(des_qPEL_4x1);


    joint->SetMoveJoint(RHY, LJ.RHY*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, LJ.RHR*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, LJ.RHP*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, LJ.RKN*R2D, postime, MOVE_ABSOLUTE);

    //for gazelle
    double RA1_deg, RA2_deg;
    GK.IK_Ankle_right(LJ.RAP*R2D, LJ.RAR*R2D, RA1_deg, RA2_deg);
    joint->SetMoveJoint(RAP, RA1_deg, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, RA2_deg, postime, MOVE_ABSOLUTE);

    cout<<"RA1_deg : "<<RA1_deg<<"RA2_deg: "<<RA2_deg<<endl;
    cout<<"RAR_Deg: "<<LJ.RAR*R2D<<endl;

//    joint->SetMoveJoint(RAP, LJ.RAP*R2D, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RAR, LJ.RAR*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, LJ.LHY*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, LJ.LHR*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, LJ.LHP*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, LJ.LKN*R2D, postime, MOVE_ABSOLUTE);

    //for gazelle
    double LA1_deg, LA2_deg;
    GK.IK_Ankle_left(LJ.LAP*R2D, LJ.LAR*R2D, LA1_deg, LA2_deg);
    joint->SetMoveJoint(LAP, LA1_deg, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, LA2_deg, postime, MOVE_ABSOLUTE);
    \
//    joint->SetMoveJoint(LAP, LJ.LAP*R2D, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LAR, LJ.LAR*R2D, postime, MOVE_ABSOLUTE);

    cout<<LJ.RHY*R2D<<endl;
    cout<<LJ.RHR*R2D<<endl;
    cout<<LJ.RHP*R2D<<endl;
    cout<<LJ.RKN*R2D<<endl;
    cout<<LJ.RAP*R2D<<endl;
    cout<<LJ.RAR*R2D<<endl;


   double total_mass;
   double total_lower_mass;
   double m_rleg = kine_oi.m_rhy+kine_oi.m_rhr+kine_oi.m_rhp+kine_oi.m_rkn+kine_oi.m_rap+kine_oi.m_rar;
   double m_lleg = kine_oi.m_lhy+kine_oi.m_lhr+kine_oi.m_lhp+kine_oi.m_lkn+kine_oi.m_lap+kine_oi.m_lar;
   total_lower_mass = m_rleg + m_lleg + kine_oi.m_pel;

   double m_rarm = kine_oi.m_rsp+kine_oi.m_rsr+kine_oi.m_rsy+kine_oi.m_reb+kine_oi.m_rwy+kine_oi.m_rwp+kine_oi.m_rf1;
   double m_larm = kine_oi.m_lsp+kine_oi.m_lsr+kine_oi.m_lsy+kine_oi.m_leb+kine_oi.m_lwy+kine_oi.m_rwp+kine_oi.m_lf1;
   total_mass = total_lower_mass + m_rarm + m_larm + kine_oi.m_torso;

   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
   cout<<"ookk"<<endl;
}


void GotoJumpReady(){
    double postime = 3000.0;

    double RSP_ref,RSR_ref,RSY_ref,REB_ref,RWY_ref,RWP_ref,LSP_ref,LSR_ref,LSY_ref,LEB_ref,LWY_ref,LWP_ref;
    double WST_ref, RWY2_ref, LWY2_ref;

    RSP_ref = 20.0;
    RSR_ref = 0.0;
    RSY_ref = 0.0;
    REB_ref = -30;
    RWY_ref = 0.0;
    RWP_ref = 0.0;

    LSP_ref = 20.0;
    LSR_ref = 0.0;
    LSY_ref = 0.0;
    LEB_ref = -30;
    LWY_ref = 0.0;
    LWP_ref = 0.0;

    WST_ref = 0.0;

    RWY2_ref = 0.0;
    LWY2_ref = 0.0;

    joint->SetMoveJoint(RSP, RSP_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, RSR_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, RSY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, REB_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, RWY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, RWP_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, LSP_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, LSR_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, LSY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, LEB_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, LWY_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, LWP_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, WST_ref, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, RWY2_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, LWY2_ref, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];


    des_pCOM_3x1[0] = 0.0;
    des_pCOM_3x1[1] = 0.0;
    des_pCOM_3x1[2] = 0.55;

    quat Pel_temp_q = quat(vec3(0,1,0),20*D2R);//*quat(vec3(1,0,0),-5*D2R);

    des_qPEL_4x1[0] = Pel_temp_q.w;//1.0;
    des_qPEL_4x1[1] = Pel_temp_q.x;//0.;
    des_qPEL_4x1[2] = Pel_temp_q.y;//0.;
    des_qPEL_4x1[3] = Pel_temp_q.z;//0.;


    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -(kine_oi.P2HR + kine_oi.HR2HPy); //-kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = (kine_oi.P2HR + kine_oi.HR2HPy);//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    //Oinverse version
    AJ.RSP = RSP_ref*D2R;
    AJ.RSR = RSR_ref*D2R + OFFSET_RSR*D2R;
    AJ.RSY = RSY_ref*D2R;
    AJ.REB = REB_ref*D2R + OFFSET_ELB*D2R;
    AJ.RWY = RWY_ref*D2R;
    AJ.RWP = RWP_ref*D2R;
    AJ.RF1 = RWY2_ref*D2R;

    AJ.LSP = LSP_ref*D2R;
    AJ.LSR = LSR_ref*D2R + OFFSET_LSR*D2R;
    AJ.LSY = LSY_ref*D2R;
    AJ.LEB = LEB_ref*D2R + OFFSET_ELB*D2R;
    AJ.LWY = LWY_ref*D2R;
    AJ.LWP = LWP_ref*D2R;
    AJ.LF1 = LWY2_ref*D2R;

    AJ.WST = WST_ref*D2R;

    LJ.RHY = 0;
    LJ.RHR = 0;
    LJ.RHP = -20*D2R;
    LJ.RKN = 40*D2R;
    LJ.RAP = -20*D2R;
    LJ.RAR = 0;

    LJ.LHY = 0;
    LJ.LHR = 0;
    LJ.LHP = -20*D2R;
    LJ.LKN = 40*D2R;
    LJ.LAP = -20*D2R;
    LJ.LAR = 0;

    LJ.pPel = vec3(0,0,0);
    LJ.qPel = quat(vec3(1,0,0),0.0);

    //kine_oi.setUB(kine_oi.FKCOM_UB(AJ));


//    LJ = kine_oi.IK_COM(vec3(des_pCOM_3x1),quat(des_qPEL_4x1)
//                        , vec3(des_pRF_3x1), quat(des_qRF_4x1)
//                        , vec3(des_pLF_3x1), quat(des_qLF_4x1));
    LJ = kine_oi.IK_COM(vec3(des_pCOM_3x1),quat(des_qPEL_4x1),vec3(des_pRF_3x1), quat(des_qRF_4x1),vec3(des_pLF_3x1), quat(des_qLF_4x1));

    cout<<"q1= "<<LJ.RHY<<" q2= "<<LJ.RHR<<" q3= "<<LJ.RHY<<" q4= "<<LJ.RKN<<" q5= "<<LJ.RAP<<" q6= "<<LJ.RAR<<endl;

    LJ.qPel = quat(des_qPEL_4x1);

    joint->SetMoveJoint(RHY, LJ.RHY*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, LJ.RHR*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, LJ.RHP*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, LJ.RKN*R2D, postime, MOVE_ABSOLUTE);

    //for gazelle
    double RA1_deg, RA2_deg;
    GK.IK_Ankle_right(LJ.RAP*R2D, LJ.RAR*R2D, RA1_deg, RA2_deg);
    joint->SetMoveJoint(RAP, RA1_deg, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, RA2_deg, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(RAP, LJ.RAP*R2D, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RAR, LJ.RAR*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, LJ.LHY*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, LJ.LHR*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, LJ.LHP*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, LJ.LKN*R2D, postime, MOVE_ABSOLUTE);

    //for gazelle
    double LA1_deg, LA2_deg;
    GK.IK_Ankle_left(LJ.LAP*R2D, LJ.LAR*R2D, LA1_deg, LA2_deg);
    joint->SetMoveJoint(LAP, LA1_deg, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, LA2_deg, postime, MOVE_ABSOLUTE);
    \
//    joint->SetMoveJoint(LAP, LJ.LAP*R2D, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LAR, LJ.LAR*R2D, postime, MOVE_ABSOLUTE);

    cout<<LJ.RHY*R2D<<endl;
    cout<<LJ.RHR*R2D<<endl;
    cout<<LJ.RHP*R2D<<endl;
    cout<<LJ.RKN*R2D<<endl;
    cout<<LJ.RAP*R2D<<endl;
    cout<<LJ.RAR*R2D<<endl;


   double total_lower_mass;
   double m_rleg = kine_oi.m_rhy+kine_oi.m_rhr+kine_oi.m_rhp+kine_oi.m_rkn+kine_oi.m_rap+kine_oi.m_rar;
   double m_lleg = kine_oi.m_lhy+kine_oi.m_lhr+kine_oi.m_lhp+kine_oi.m_lkn+kine_oi.m_lap+kine_oi.m_lar;
   total_lower_mass = m_rleg + m_lleg + kine_oi.m_pel;


   printf("total mass : %f\n",total_lower_mass);
   cout<<"Jump Ready Go..!"<<endl;
}

void GotoHomePos()
{
    WB_FLAG = false;
    double postime = 3000.0;
    for(int i=0; i<NO_OF_JOINTS; i++)
        joint->SetMoveJoint(i, 0.0, postime, MOVE_ABSOLUTE);

    //for gazelle
    double RA1_deg, RA2_deg;
    GK.IK_Ankle_right(0,0,RA1_deg, RA2_deg);
    joint->SetMoveJoint(RAP,RA1_deg,postime,MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR,RA2_deg,postime,MOVE_ABSOLUTE);

    double LA1_deg, LA2_deg;
    GK.IK_Ankle_left(0,0,LA1_deg, LA2_deg);
    joint->SetMoveJoint(LAP,LA1_deg,postime,MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR,LA2_deg,postime,MOVE_ABSOLUTE);
}

void WholebodyInitialize()
{
    WBIK_PARA_CHANGE();
    Qub[idRSP] = 0.*D2R;
    Qub[idRSR] = -40.*D2R;
    Qub[idRSY] = -30.*D2R;
    Qub[idREB] = -70.*D2R;

    Qub[idLSP] = 0.*D2R;
    Qub[idLSR] = 40.*D2R;
    Qub[idLSY] = 30.*D2R;
    Qub[idLEB] = -70.*D2R;

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
        qtRZ(0.,qCenter);


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
void Kirk_Control()
{
    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

    Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

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

    Del_PC_X_SSP_XZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_XP1(-Del_PC_X_SSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_SSP_YZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_YP1(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);
    //printf(" Del_PC_Y_SSP_YZMP_CON = %f\n",Del_PC_Y_SSP_YZMP_CON);
    LPF_Del_PC_X_SSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_SSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_SSP_XZMP_CON;
    LPF_Del_PC_Y_SSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_SSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_SSP_YZMP_CON;

    if(CNT_final_gain_SSP_ZMP_CON < 40) CNT_final_gain_SSP_ZMP_CON++;

    Old_Del_PC_X_SSP_XZMP_CON = Del_PC_X_SSP_XZMP_CON;
    Old_Del_PC_Y_SSP_YZMP_CON = Del_PC_Y_SSP_YZMP_CON;
}
void ZMP_intergral_control()
{
    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;

}
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

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

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

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 80.0) y = 80.0;
    else if(y < -80.0) y = -80.0;

    return y ;
}


void save_all()
{
    FILE* ffp2 = fopen("/home/rainbow/Desktop/HBtest_Walking_Data_prev3.txt","w");
    for(int i=0;i<save_cnt;i++)
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

void save(int cnt)
{
    SAVE[0][cnt] = sharedSEN->ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
    SAVE[1][cnt] = sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition;
    SAVE[2][cnt] = sharedSEN->ENCODER[MC_GetID(RAP)][MC_GetCH(RAP)].CurrentPosition;
    SAVE[3][cnt] = sharedSEN->ENCODER[MC_GetID(RAR)][MC_GetCH(RAR)].CurrentPosition;
    SAVE[4][cnt] = sharedSEN->ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentReference;
    SAVE[5][cnt] = sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentReference;
    SAVE[6][cnt] = sharedSEN->ENCODER[MC_GetID(RAP)][MC_GetCH(RAP)].CurrentReference;
    SAVE[7][cnt] = sharedSEN->ENCODER[MC_GetID(RAR)][MC_GetCH(RAR)].CurrentReference;

    SAVE[8][cnt] = sharedSEN->ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentPosition;
    SAVE[9][cnt] = sharedSEN->ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
    SAVE[10][cnt] = sharedSEN->ENCODER[MC_GetID(LAP)][MC_GetCH(LAP)].CurrentPosition;
    SAVE[11][cnt] = sharedSEN->ENCODER[MC_GetID(LAR)][MC_GetCH(LAR)].CurrentPosition;
    SAVE[12][cnt] = sharedSEN->ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentReference;
    SAVE[13][cnt] = sharedSEN->ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference;
    SAVE[14][cnt] = sharedSEN->ENCODER[MC_GetID(LAP)][MC_GetCH(LAP)].CurrentReference;
    SAVE[15][cnt] = sharedSEN->ENCODER[MC_GetID(LAR)][MC_GetCH(LAR)].CurrentReference;
}
