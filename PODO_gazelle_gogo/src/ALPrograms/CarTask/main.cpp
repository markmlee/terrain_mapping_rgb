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
//Motion check
#include <time.h>
#include "BasicMatrix.h"

#include "joint.h"
#include "taskmotion.h"
#include "CarTask.h"
#include "MotionChecker.h"
#include "functions.h"
#include "ManualCAN.h"


#define PODO_AL_NAME       "CarTask_AL"

#define RIGHT               true
#define LEFT                false

#define RH_UB_LH_PC         0
#define RH_LH_UB            1
#define RF_LF_LB            2
#define GLOBAL_WBIK         3

using namespace std;

// Functions ===========================================
// Signal handler
void CatchSignals(int _signal);
int HasAnyOwnership();
int AskALNumber(char *alname);

// Real-time thread for control
void RBTaskThread(void *);
void RBFlagThread(void *);

// Initialization
int RBInitialize(void);
// =====================================================

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
// PODO No.
int PODO_NO;
// --------------------------------------------------------------------------------------------- //


JointControlClass *joint;
// CKINE_DRC_HUBO kine_drc_hubo;
int CheckMotionOwned();
// WBIK functins-------------------------------------------------------------------------------- //
// Variables
int             WB_FLAG = false;
long            LimitedJoint;
long            LimitType;

TaskMotion      *WBmotion;

// MotionChecker *MC;


// *** Your PODO-AL command set *** //

enum TASK_FLAG
{
    IDLE = 0,
    TASK_GRAVITY_COMP,
    TASK_GAIN_TUNING,
    TASK_GO_POS,
    TASK_FORCE_CONTROL,
    TASK_HYBRID_CONTROL,
    RH_APPROACH_CONTROL,
    TASK_POSITION_CONTROL,
    TASK_POSITION_CONTROL_RIGHT,
    TASK_POSITION_CONTROL_LEFT,
    TASK_JOINT_FORCE_CONTROL,
    TASK_JOINT_FORCE_CONTROL_RIGHT,
    TASK_JOINT_FORCE_CONTROL_LEFT,
    TASK_XYZ_POSITION_CONTROL,
    TASK_XYZ_POSITION_CONTROL_RIGHT,
    TASK_XYZ_POSITION_CONTROL_LEFT
};

enum FTNAME{
    RAFT = 0,
    LAFT,
    RWFT,
    LWFT
};



int _task_flag = 0;

// *** Your PODO-AL command set *** //
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

    // Initialize RBCore -----------------------------------
    if( RBInitialize() == -1 ) isTerminated = -1;
    cout << "======================================================================" << endl;

    // Initialize internal joint classes -------------------
    joint = new JointControlClass(sharedREF, sharedSEN, sharedCMD, PODO_NO);
    joint->RefreshToCurrentReference();
    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, joint);

    // Force Control
    _task_flag = 0;


    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
        case CarTask_AL_GAIN:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)//right arm
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]==0)//low gain
                {
                    cout<<"Right arm gain LOW"<<endl;
                    //RBBoardSetSwitchingMode(2,15,SW_MODE_NON_COMPLEMENTARY);  //RSY REB
                    //RBBoardSetSwitchingMode(2,16,SW_MODE_NON_COMPLEMENTARY);  //RWY RWP
//                    MCBoardSetSwitchingMode(2, 15, SW_MODE_NON_COMPLEMENTARY);  //RSY REB
//                    MCBoardSetSwitchingMode(2, 16, SW_MODE_NON_COMPLEMENTARY);  //RWY RWP

//                    unsigned int temp_gain=1;
//                    RBJointGainOverride(2,15,temp_gain,temp_gain,1000);//RSY REB
//                    RBJointGainOverride(2,16,temp_gain,temp_gain,1000);//RWY RWP

                    //MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 3000, 27, 0);
                    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);

                    //MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
                    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);

                    //MCsetFrictionParameter(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 720, 10, 0);
                    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, ENABLE);

                    //MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 700,14, 0);
                    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);

                    //MCsetFrictionParameter(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 280, 120, 0);
                    //MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, ENABLE);


                    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch ,30,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch ,30,1000);  //REB
                    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch ,30,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch ,30,1000);  //RWP
                    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 30,1000);  //RWY2

                    //sharedData->STATE_COMMAND = TCMD_CAR_RIGHT_GAIN_OVER_DONE;
                }
                else//high gain
                {
                    cout<<"Right arm gain HIGH"<<endl;
//                    RBBoardSetSwitchingMode(2,15,SW_MODE_COMPLEMENTARY);
//                    RBBoardSetSwitchingMode(2,16,SW_MODE_COMPLEMENTARY);
//                    MCBoardSetSwitchingMode(2, 15, SW_MODE_COMPLEMENTARY);  //RSY REB
//                    MCBoardSetSwitchingMode(2, 16, SW_MODE_COMPLEMENTARY);  //RWY RWP

//                    RBJointGainOverride(2,15,1000,1000,2000);//RSY REB
//                    RBJointGainOverride(2,16,1000,1000,2000);//RWY RWP

                    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);

                    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);

                    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);

                    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);

                    //MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);

                    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch ,0,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch ,0,1000);  //REB
                    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch ,0,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch ,0,1000);  //RWP
                    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,1000);  //RWY2
                }
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1) //left arm
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]==0)//low gain
                {
                    cout<<"Left arm gain Low"<<endl;
//                    RBBoardSetSwitchingMode(3,19,SW_MODE_NON_COMPLEMENTARY);
//                    RBBoardSetSwitchingMode(3,20,SW_MODE_NON_COMPLEMENTARY);
                    MCBoardSetSwitchingMode(3,19, SW_MODE_NON_COMPLEMENTARY);  //LSY LEB
                    MCBoardSetSwitchingMode(3,20, SW_MODE_NON_COMPLEMENTARY);  //LWY LWP

//                    unsigned int temp_gain=1;
//                    RBJointGainOverride(3,19,temp_gain,temp_gain,1000);//LSY LEB
//                    RBJointGainOverride(3,20,temp_gain,temp_gain,1000);//LWY LWP
                    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch ,30,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch ,30,1000);  //LEB
                    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch ,30,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch ,30,1000);  //LWP
                    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 30,1000);  //RWY2

                    //sharedData->STATE_COMMAND = TCMD_CAR_LEFT_GAIN_OVER_DONE;
                }
                else//high gain
                {
                    cout<<"Left arm gain HIGH"<<endl;
//                    RBBoardSetSwitchingMode(3,19,SW_MODE_COMPLEMENTARY);
//                    RBBoardSetSwitchingMode(3,20,SW_MODE_COMPLEMENTARY);
                    MCBoardSetSwitchingMode(3,19, SW_MODE_COMPLEMENTARY);  //LSY LEB
                    MCBoardSetSwitchingMode(3,20, SW_MODE_COMPLEMENTARY);  //LWY LWP

//                    RBJointGainOverride(3,19,1000,1000,1000);//LSY LEB
//                    RBJointGainOverride(3,20,1000,1000,1000);//LWY LWP
                    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch ,0,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch ,0,1000);  //LEB
                    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch ,0,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch ,0,1000);  //LWP
                    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0,1000);  //RWY2
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        case CarTask_AL_HAND_GRAB:
            joint->SetMotionOwner(RHAND);
            joint->SetMotionOwner(LHAND);
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){ // RH grab
                joint->SetJointRefAngle(RHAND, 125);
                usleep(4010*1000);
                joint->SetJointRefAngle(RHAND, 0);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){ // RH release
                joint->SetJointRefAngle(RHAND, -125);
                usleep(4010*1000);
                joint->SetJointRefAngle(RHAND, 0);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2){ // LH grab
                joint->SetJointRefAngle(LHAND, 125);
                usleep(4010*1000);
                joint->SetJointRefAngle(LHAND, 0);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 3){ // LH release
                joint->SetJointRefAngle(LHAND, -125);
                usleep(4010*1000);
                joint->SetJointRefAngle(LHAND, 0);
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        case CarTask_AL_CAR_TASK_POSE:
            cout<<"Car Task Pose..!!"<<endl;
            WB_FLAG = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)
                Change_Pos_CAR2(); // pre pose
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)
                Change_Pos_CAR1(); // final pose
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2)
                GotoHomePos();

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_GRAVITY_COMP:
            cout<<"Gravity Compensation start!!"<<endl;
            Enc_request(1);
            usleep(1000*5);
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){ // Right Arm
                ZeroGainRightArm();
                for(int i=RSP; i<=RWP ; i++){
                    joint->SetMotionOwner(i);
                }
                joint->SetMotionOwner(RWY2);
                GravityComp_Right_Flag = true;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){ // Left Arm
                ZeroGainLeftArm();
                for(int i=LSP; i<=LWP ; i++){
                    joint->SetMotionOwner(i);
                }
                joint->SetMotionOwner(LWY2);
                GravityComp_Left_Flag = true;
            }

            usleep(1000*5);
            _task_flag = TASK_GRAVITY_COMP;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_GAIN_TUNING:
            Enc_request(1);

            usleep(1000*20);
            gain_tuning(1, sharedCMD->COMMAND[PODO_NO].USER_PARA_INT);
            _task_flag = TASK_GAIN_TUNING;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_GO_POS:
            go_pos(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0], sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_SAVE_ENC:
            save_calib(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0], sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_ZERO_GAIN:
            Enc_request(1);
            usleep(1000*500);
            _task_flag = IDLE;
            //zero_gain();

            ZeroGainLeftArm();
            ZeroGainRightArm();


            //---------------for Manual CAN Test------------------------
//            Enc_request(1);
//            usleep(1000*500);
//            zero_gain();
//            usleep(1000*1000);
//            MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4,100,0,0);
//            _task_flag = IDLE;

//            for(int i=LSP; i<=LWP ; i++){
//                joint->SetMotionOwner(i);
//            }
//            joint->SetMotionOwner(LWY2);

//            for(int i=RSP; i<=RWP ; i++){
//                joint->SetMotionOwner(i);
//            }
//            joint->SetMotionOwner(RWY2);



            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_GO_POSITION_MODE:
            Enc_request(1);
            _task_flag = IDLE;
            usleep(1000*50);
            if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) < 0.01){
                cout<<"Encoder is off, Check the Encoder!!"<<endl;
            }
            else{
                ForceToPositionControl_LeftArm();
                ForceToPositionControl_RightArm();
                cout<<"Position control on!!"<<endl;
            }
            //------for Manual CAN TEST

            //--Right ARM
//            MCJointRequestEncoderPosition(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 1);
//            usleep(20*1000);
//            cout<<"RSP Encoder Position : "<<sharedData->CurrentPosition[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch]<<endl;


//            joint->RefreshToCurrentEncoder_TEST();
//            joint->SetMotionOwner(RSP);

//            cout<<"current joint ref: "<<joint->GetJointRefAngle(RSP)<<endl;
//            joint->SetJointRefAngle(RSP,sharedData->CurrentPosition[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch]);
//            cout<<"current ref ang : "<<joint->GetJointRefAngle(RSP)<<endl;

//            usleep(100*1000);

//            MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
//            MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);

//            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 65,2000); //--RSP

//            usleep(2000*2000);

//            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,1000); //--RSP



            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_FORCE_CONTROL:

            Enc_request(1);
            usleep(1000*5);
            zero_gain();
            usleep(1000*5);

            for(int i=LSP; i<=LWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(LWY2);

            for(int i=RSP; i<=RWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(RWY2);

            RH_Fx = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            RH_Fy = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            RH_Fz = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            LH_Fx = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
            LH_Fy = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
            LH_Fz = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];

            cout<<"LH_Fx :"<<LH_Fx<<endl<<"LH_Fy :"<<LH_Fy<<endl<<"LH_Fz :"<<LH_Fz<<endl;
            cout<<"RH_Fx :"<<RH_Fx<<endl<<"RH_Fy :"<<RH_Fy<<endl<<"RH_Fz :"<<RH_Fz<<endl;

            //for JFT Experiment
            Data_index = 0;
            _task_flag = TASK_FORCE_CONTROL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_HYBRID_CONTROL:
            cout<<"Hybrid Control Start..!!"<<endl;

            Enc_request(1);
            usleep(1000*500);
            joint->RefreshToCurrentEncoder();
            //joint->RefreshToCurrentReference();
            WB_FLAG = false;


            zero_gain();
            usleep(1000*50);

            for(int i=LSP; i<=LWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(LWY2);

            for(int i=RSP; i<=RWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(RWY2);

            RH_x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            RH_y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            RH_Fz = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            LH_x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
            LH_y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
            LH_Fz = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
            LSR_angle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
            RSR_angle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];

            cout<<"LH_x :"<<LH_x<<"   LH_y :"<<LH_y<<"   LH_Fz :"<<LH_Fz<<"   LElb: "<<LSR_angle<<endl;
            cout<<"RH_x :"<<RH_x<<"   RH_y :"<<RH_y<<"   RH_Fz :"<<RH_Fz<<"   RElb: "<<RSR_angle<<endl;

            _task_flag = TASK_HYBRID_CONTROL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_REFRESH:
        {
            cout<<"Refresh hand pos!"<<endl;
            double pRH_3x1[3];
            double pLH_3x1[3];
            double LSR_angle;
            double RSR_angle;
            FK_Pos_RightArm(pRH_3x1);
            FK_Pos_LeftArm(pLH_3x1);

            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
            RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR;

            cout<<"RH_x: "<<pRH_3x1[0]<<"   RH_y :"<<pRH_3x1[1]<<endl;
            cout<<"LH_x: "<<pLH_3x1[0]<<"   LH_y :"<<pLH_3x1[1]<<"   LElb: "<<LSR_angle<<endl;

            userData->M2G.pRH[0] = pRH_3x1[0];
            userData->M2G.pRH[1] = pRH_3x1[1];
            userData->M2G.Relb = RSR_angle;

            userData->M2G.pLH[0] = pLH_3x1[0];
            userData->M2G.pLH[1] = pLH_3x1[1];
            userData->M2G.Lelb = LSR_angle;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

        case CarTask_AL_PRINT_STATE:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){ // Current joint Referrence
                joint->RefreshToCurrentReference();
                PrintJointRef();
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
                break;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){ // Current WB Refference
                WB_FLAG = false;
                usleep(10*1000);
                joint->RefreshToCurrentReference();
                WBmotion->ResetGlobalCoord(0);//Coord
                WBmotion->StopAll();// Trajectory making ...
                WBmotion->RefreshToCurrentReferencePELV_LH();
                PrintWBIKinfo();
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2){ // Current Joint pos
                if(joint->RefreshToCurrentEncoder() == 0){
                    cout<<"Encoder is Off!!"<<endl;
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
                    break;
                }
                PrintJointRef();
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
                break;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 3){ // Current WB pos //UB
                WB_FLAG = false;
                usleep(10*1000);
                if(joint->RefreshToCurrentEncoder() == 0){
                    cout<<"Encoder is Off!!"<<endl;
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
                    break;
                }
                WBmotion->ResetGlobalCoord(0);//Coord
                WBmotion->StopAll();// Trajectory making ...
                WBmotion->RefreshToCurrentReferenceUB();     // Local UB origin
                PrintWBIKinfo();
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CT_REFRESH_POSITION:
        {
            Enc_request(1);
            cout<<"Refresh hand pos!"<<endl;
            double pRH_3x1[3];
            double qRH_4x1[4];
            double pLH_3x1[3];
            double qLH_4x1[4];
            double LSR_angle;
            double RSR_angle;
            double RElb, LElb;
            FK_Pos_Ori_Elb_RightArm(pRH_3x1, qRH_4x1, RElb);
            FK_Pos_Ori_Elb_LeftArm(pLH_3x1, qLH_4x1, LElb);

            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
            RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR;

//            cout<<"RH_x: "<<pRH_3x1[0]<<"   RH_y :"<<pRH_3x1[1]<<endl;
//            cout<<"LH_x: "<<pLH_3x1[0]<<"   LH_y :"<<pLH_3x1[1]<<"   LElb: "<<LSR_angle<<endl;

            quat cur_quatRH = quat(qRH_4x1[0], qRH_4x1[1], qRH_4x1[2], qRH_4x1[3]);
            rpy cur_rpyRH = rpy(cur_quatRH);

            quat cur_quatLH = quat(qLH_4x1[0], qLH_4x1[1], qLH_4x1[2], qLH_4x1[3]);
            rpy cur_rpyLH = rpy(cur_quatLH);

            userData->M2G.pRH[0] = pRH_3x1[0];
            userData->M2G.pRH[1] = pRH_3x1[1];
            userData->M2G.pRH[2] = pRH_3x1[2];

            userData->M2G.qRH[0] = cur_rpyRH.r;
            userData->M2G.qRH[1] = cur_rpyRH.p;
            userData->M2G.qRH[2] = cur_rpyRH.y;

            userData->M2G.Relb = RSR_angle;

            userData->M2G.pLH[0] = pLH_3x1[0];
            userData->M2G.pLH[1] = pLH_3x1[1];
            userData->M2G.pLH[2] = pLH_3x1[2];

            userData->M2G.qLH[0] = cur_rpyLH.r;
            userData->M2G.qLH[1] = cur_rpyLH.p;
            userData->M2G.qLH[2] = cur_rpyLH.y;

            userData->M2G.Lelb = LSR_angle;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }
        case CarTask_AL_CT_POSITION_CONTROL:
        {
            cout<<"Position Control(By Force) Start..!!"<<endl;

            Enc_request(1);
            usleep(1000*500);
            joint->RefreshToCurrentEncoder();
            //joint->RefreshToCurrentReference();
            WB_FLAG = false;


            ZeroGainLeftArm();
            ZeroGainRightArm();
            usleep(1000*500);

            for(int i=LSP; i<=LWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(LWY2);

            for(int i=RSP; i<=RWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(RWY2);

            double _RH_roll, _RH_pitch, _RH_yaw, _LH_roll, _LH_pitch, _LH_yaw;

            RH_x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            RH_y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            RH_z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            RSR_angle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

            _RH_roll = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4]*D2R;
            _RH_pitch = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5]*D2R;
            _RH_yaw = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6]*D2R;


            LH_x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
            LH_y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
            LH_z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[9];
            LSR_angle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[10];

            _LH_roll = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[11]*D2R;
            _LH_pitch = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[12]*D2R;
            _LH_yaw = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[13]*D2R;

            rpy cur_rpyRH = rpy(_RH_roll, _RH_pitch, _RH_yaw);
            quat cur_quatRH = quat(cur_rpyRH);
            rpy cur_rpyLH = rpy(_LH_roll, _LH_pitch, _LH_yaw);
            quat cur_quatLH = quat(cur_rpyLH);

            qRH_4x1[0] = cur_quatRH.w;
            qRH_4x1[1] = cur_quatRH.x;
            qRH_4x1[2] = cur_quatRH.y;
            qRH_4x1[3] = cur_quatRH.z;

            qLH_4x1[0] = cur_quatLH.w;
            qLH_4x1[1] = cur_quatLH.x;
            qLH_4x1[2] = cur_quatLH.y;
            qLH_4x1[3] = cur_quatLH.z;

//            cout<<"LH_x :"<<LH_x<<"   LH_y :"<<LH_y<<"   LH_Fz :"<<LH_Fz<<"   LElb: "<<LSR_angle<<endl;
//            cout<<"RH_x :"<<RH_x<<"   RH_y :"<<RH_y<<"   RH_Fz :"<<RH_Fz<<"   RElb: "<<RSR_angle<<endl;

            _task_flag = TASK_POSITION_CONTROL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }
        case CarTask_AL_JOINT_FORCE_CTRL_TEST:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){
                Enc_request(1);
                usleep(500*1000);

                RSP_angle = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition;
                RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition + OFFSET_RSR;
                RSY_angle = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition;
                REB_angle = sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+ OFFSET_ELB;
                RWY_angle = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition;
                RWP_angle = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition;
                RF1_angle = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition;

                _task_flag = TASK_JOINT_FORCE_CONTROL_RIGHT;
            }
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){
                Enc_request(1);
                usleep(500*1000);

                LSP_angle = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition;
                LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition + OFFSET_LSR;
                LSY_angle = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition;
                LEB_angle = sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition + OFFSET_ELB;
                LWY_angle = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition;
                LWP_angle = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition;
                LF1_angle = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition;
                _task_flag = TASK_JOINT_FORCE_CONTROL_LEFT;
            }
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2){
                _task_flag = IDLE;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_CAR_RELEASE:
        {
            cout<<"Release Start Button Clicked!"<<endl;
            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceUB();
            joint->SetAllMotionOwner();

            // Current Hand Quaternion
            quat qRH = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
            mat3 RH_mat = mat3(qRH);
            vec3 GoalPoint;
            vec3 Cur_pRH = vec3(WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
            vec3 Dist = vec3(0, 0, 0.08);
            vec3 Del_pRH = RH_mat*Dist;
            double cur_RELB = WBmotion->RElb_ang*R2D;

            GoalPoint = Cur_pRH + Del_pRH;
            WBIK_mode = RH_LH_UB;
            WB_FLAG = true;

            cout<<"RKN : "<<fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) <<endl;

            if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) < 1.0){
                cout<<"Encoder is off, Simulation Release!!"<<endl;
            }
            else{
                cout<<"Encoder is on, Experiment Release!!"<<endl;
                joint->SetJointRefAngle(RHAND, -125);
                while(sharedSEN->ENCODER[MC_GetID(RHAND)][MC_GetCH(RHAND)].CurrentPosition < 33.0){
                    static int try_count = 0;
                    joint->SetJointRefAngle(RHAND, -125);
                    usleep(500*1000);
                    if(try_count == 10){
                        joint->SetJointRefAngle(RHAND, 125);
                        usleep(500*1000);
                        try_count = 0;
                        //TCMD : RF release jam!!!!
                    }
                    try_count++;
                    cout<<"RF release Try count : "<< try_count<<endl;
                }
            }
            WBmotion->addRHPosInfo(GoalPoint.x, GoalPoint.y, GoalPoint.z,2);
            WBmotion->addRElbPosInfo(cur_RELB-5,2);
            usleep(2020*1000);
            joint->SetJointRefAngle(RHAND, 0);
            cout<<"Release Sequence finished"<<endl;

            //sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_CAR_TURNING_POSE;
            break;
        }

        case CarTask_AL_CAR_TURNING_POSE:
        {


//            WB_FLAG = false;
//            usleep(10*1000);
//            joint->RefreshToCurrentReference();
//            WBmotion->ResetGlobalCoord(0);//Coord
//            WBmotion->StopAll();// Trajectory making ...
//            WBmotion->RefreshToCurrentReferenceUB();
//            joint->SetAllMotionOwner();
//            WBIK_mode = RH_LH_UB;
//            WB_FLAG = true;
//            usleep(20*1000);


//            WBmotion->addRElbPosInfo(-144.916133,2);
//            WBmotion->addRHPosInfo(0.374264, -0.06683, 0.603546,2);
//            doubles RH_ori(4);
//            RH_ori[0] = 0.70785;
//            RH_ori[1] = -0.06195;
//            RH_ori[2] = -0.100166;
//            RH_ori[3] = 0.696468;
//            WBmotion->addRHOriInfo(RH_ori,2);
//            usleep(500*1000);
//            joint->SetJointRefAngle(RHAND,125);
//            usleep(3010*1000);
//            joint->SetJointRefAngle(RHAND, 0);
            WB_FLAG = false;

            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch ,15,1000);  //REB

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            double moving_time = 2500;
//            joint->SetMoveJoint(RSP, -171.81, moving_time, MODE_ABSOLUTE);
//            joint->SetMoveJoint(RSR, 4.31, moving_time, MODE_ABSOLUTE);
//            joint->SetMoveJoint(RSY, 141.04, moving_time, MODE_ABSOLUTE);
//            joint->SetMoveJoint(REB, -82.63, moving_time, MODE_ABSOLUTE);
//            joint->SetMoveJoint(RWY, -150.0, moving_time, MODE_ABSOLUTE);
//            joint->SetMoveJoint(RWP, 66.19, moving_time, MODE_ABSOLUTE);
            joint->SetMoveJoint(RSP, -177.81, moving_time, MODE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 4.31, moving_time, MODE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 141.04, moving_time, MODE_ABSOLUTE);
            joint->SetMoveJoint(REB, -88.63, moving_time, MODE_ABSOLUTE);
            joint->SetMoveJoint(RWY, -162.0, moving_time, MODE_ABSOLUTE);
            joint->SetMoveJoint(RWP, 66.19, moving_time, MODE_ABSOLUTE);

            if(joint->GetJointRefAngle(RWY2) > 180.0 || joint->GetJointRefAngle(RWY2) < -180.0){
                moving_time = fabs(joint->GetJointRefAngle(RWY2))*1800/100.0;
                joint->SetMoveJoint(RWY2, 41.0, moving_time,MODE_ABSOLUTE);
            }
            else{
                joint->SetMoveJoint(RWY2, 41.0, moving_time, MODE_ABSOLUTE);
            }


            usleep((int)(moving_time)*1000);
            usleep(100*1000);

            cout<<"Turning Pos Finished!!"<<endl;
            //sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_CAR_TURNING;
            break;
        }
        case CarTask_AL_CAR_TURNING:
            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ...
            WBmotion->RefreshToCurrentReferencePELV_LH();
            joint->SetAllMotionOwner();
            WBIK_mode = RH_UB_LH_PC;
            WB_FLAG = true;
            usleep(20*1000);
            WBmotion->addWSTPosInfo(0,6);
            WBmotion->addLElbPosInfo(0,6);
            //WBmotion->addRHPosInfo(0,0,0,3);
            joint->SetJointRefAngle(RHAND,125);
            usleep(6010*1000);
            joint->SetJointRefAngle(RHAND,0);

            //WB_FLAG = false;

            //sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_RH_GRAB_POS;
            break;

        case CarTask_AL_RH_GRAB_POS:
            RH_Grab_pos_SQ_No = 0;

            RH_grab_pos_flag = true;   //
            
            Descending_flag = false;
            Static_walk_flag = false;
            Stabilizing_RH_flag = false;
            Stabilizing_LH_flag = false;
            WalkReady_pos_flag = false;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_SEND_ROI:
        {
            cout<<"RH_grab start"<<endl;
            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceUB();
            joint->SetAllMotionOwner();

            //double grab_pos[3];

            vec3 posY_diff_rel = vec3(0,-0.66,0);
            vec3 posZ_diff_glo = vec3(0,0,-0.165);
            vec3 cur_pLH;
            quat cur_qLH;

            if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) < 0.01){
                cout<<"Encoder is off, Simulation Mode on!"<<endl;
                //----for Simulation--------------------------------------------------------------------------------------
                cur_pLH = vec3(WBmotion->pLH_3x1[0],WBmotion->pLH_3x1[1],WBmotion->pLH_3x1[2]);
                cur_qLH = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
                //---------------------------------------------------------------------------------------------------------
            }
            else{
                cout<<"Encoder is On, Experiment mode!"<<endl;
                //----for Experiment--------------------------------------------------------------------------------------
                Enc_request(1);
                usleep(500*1000);
                double pLH_3x1_FK[3];
                double qLH_4x1_FK[4];
                double LElb_FK;
                FK_Pos_Ori_Elb_LeftArm(pLH_3x1_FK, qLH_4x1_FK, LElb_FK);
                cur_pLH = vec3(pLH_3x1_FK[0],pLH_3x1_FK[1],pLH_3x1_FK[2]);
                cur_qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3]);
                //--------------------------------------------------------------------------------------------------------
            }



            mat3 LH_mat = mat3(cur_qLH);
            vec3 posY_diff_glo = LH_mat*posY_diff_rel;
            vec3 RH_grab_pos_non_vision_vec = cur_pLH + posY_diff_glo + posZ_diff_glo;

            RH_grab_pos_non_vision[0] = RH_grab_pos_non_vision_vec.x;
            RH_grab_pos_non_vision[1] = RH_grab_pos_non_vision_vec.y;
            RH_grab_pos_non_vision[2] = RH_grab_pos_non_vision_vec.z;


            float ROI_size = 0.3;

            //sharedData->STATE_PARA_FLOAT[0] = RH_grab_pos_non_vision[0];//0.35;//
            //sharedData->STATE_PARA_FLOAT[1] = RH_grab_pos_non_vision[1];//-0.5;//
            //sharedData->STATE_PARA_FLOAT[2] = RH_grab_pos_non_vision[2];//0.83;//
            //sharedData->STATE_PARA_FLOAT[3] = ROI_size;
            //sharedData->STATE_COMMAND = TCMD_CAR_SEND_3D_ROI_CENTER;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

        case CarTask_AL_CHECK_RH_GRAB_POS_VISION:
        {
            vec3 bar_on_point(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0], sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1], sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2]);
            vec3 bar_vec(sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3], sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4], sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5]);


            double t = (RH_grab_pos_non_vision[2] - bar_on_point.z)/bar_vec.z;

            RH_grab_pos_vision[0] = bar_vec.x*t + bar_on_point.x;
            RH_grab_pos_vision[1] = bar_vec.y*t + bar_on_point.y;
            RH_grab_pos_vision[2] = RH_grab_pos_non_vision[2];

            if(sqrtp((RH_grab_pos_vision[0] - RH_grab_pos_non_vision[0])*(RH_grab_pos_vision[0] - RH_grab_pos_non_vision[0])
                 + (RH_grab_pos_vision[1] - RH_grab_pos_non_vision[1])*(RH_grab_pos_vision[1] - RH_grab_pos_non_vision[1])) < 0.03 ){
                //sharedData->STATE_PARA_CHAR[0] = 1; //good
            }
            else{
                //sharedData->STATE_PARA_CHAR[0] = 0; //bad
            }

            //sharedData->STATE_PARA_FLOAT[0] = RH_grab_pos_non_vision[0]; //x
            //sharedData->STATE_PARA_FLOAT[1] = RH_grab_pos_non_vision[1]; //y
            //sharedData->STATE_PARA_FLOAT[2] = RH_grab_pos_non_vision[2]; //z

            //sharedData->STATE_PARA_FLOAT[3] = RH_grab_pos_vision[0]; //x
            //sharedData->STATE_PARA_FLOAT[4] = RH_grab_pos_vision[1]; //y
            //sharedData->STATE_PARA_FLOAT[5] = RH_grab_pos_vision[2]; //z

            //sharedData->STATE_COMMAND = TCMD_CAR_SEND_RH_GRAB_POS_VISION;

            //cout<<"vis x: "<<RH_grab_pos_vision[0]<<", vis y: "<<RH_grab_pos_vision[1]<<" ok? : "<<sharedData->STATE_PARA_CHAR[0]<<endl;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

        case CarTask_AL_MODIFY_VISION_DATA:
        {
            RH_grab_pos_vision[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            RH_grab_pos_vision[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            RH_grab_pos_vision[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];

            if(sqrtp((RH_grab_pos_vision[0] - RH_grab_pos_non_vision[0])*(RH_grab_pos_vision[0] - RH_grab_pos_non_vision[0])
                 + (RH_grab_pos_vision[1] - RH_grab_pos_non_vision[1])*(RH_grab_pos_vision[1] - RH_grab_pos_non_vision[1])) < 0.03 ){
                //sharedData->STATE_PARA_CHAR[0] = 1; //good
            }
            else{
                //sharedData->STATE_PARA_CHAR[0] = 0; //bad
            }

            //sharedData->STATE_PARA_FLOAT[0] = RH_grab_pos_non_vision[0]; //x
            //sharedData->STATE_PARA_FLOAT[1] = RH_grab_pos_non_vision[1]; //y
            //sharedData->STATE_PARA_FLOAT[2] = RH_grab_pos_non_vision[2]; //z

            //sharedData->STATE_PARA_FLOAT[3] = RH_grab_pos_vision[0]; //x
            //sharedData->STATE_PARA_FLOAT[4] = RH_grab_pos_vision[1]; //y
            //sharedData->STATE_PARA_FLOAT[5] = RH_grab_pos_vision[2]; //z

            //sharedData->STATE_COMMAND = TCMD_CAR_SEND_RH_GRAB_POS_VISION;

            //cout<<"vis x: "<<RH_grab_pos_vision[0]<<", vis y: "<<RH_grab_pos_vision[1]<<" ok? : "<<sharedData->STATE_PARA_CHAR[0]<<endl;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

            break;
        }

        case CarTask_AL_GRAB_RH_VISION_DATA:
        {
            cout<<"RH_grab start with Vision data"<<endl;
            WB_FLAG = false;
            usleep(20*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferencePELV_LH();
            joint->SetAllMotionOwner();

            vec3 grab_pos(RH_grab_pos_vision[0], RH_grab_pos_vision[1], RH_grab_pos_vision[2]);

            double via1_pos[3], via2_pos[3], via3_pos[3], via4_pos[3];
            //doubles via1_quat(4),via2_quat(4),via3_quat(4);
            double L = 0.05; // Sliding up distance
            double l = 0.06;  // first normal distance
            double l1 = 0.03; // second normal distance
            double bar_ang = 56*D2R;

            via4_pos[0] = grab_pos.x;
            via4_pos[1] = grab_pos.y + l1*sin(bar_ang);
            via4_pos[2] = grab_pos.z - l1*cos(bar_ang);

            via3_pos[0] = via4_pos[0];
            via3_pos[1] = via4_pos[1] - L*cos(bar_ang);
            via3_pos[2] = via4_pos[2] - L*sin(bar_ang);

            via2_pos[0] = via3_pos[0];
            via2_pos[1] = via3_pos[1] + l*sin(bar_ang);
            via2_pos[2] = via3_pos[2] - l*cos(bar_ang);

            via1_pos[0] = via2_pos[0] + 0.15;
            via1_pos[1] = via2_pos[1];
            via1_pos[2] = via2_pos[2];


            WBIK_mode = RH_LH_UB;
            WB_FLAG = true;

            WBmotion->addRHPosInfo(via1_pos[0],via1_pos[1],via1_pos[2],2);
            quat cur_quat_RH(-0.158854,-0.119086,-0.862637,-0.465232);
            quat RH_rot(vec3(1,0,0),3*D2Rf);
            quat des_quat_RH = cur_quat_RH; //cur_quat_RH*RH_rot;

            doubles qRH(4);
            qRH[0] = des_quat_RH.w;
            qRH[1] = des_quat_RH.x;
            qRH[2] = des_quat_RH.y;
            qRH[3] = des_quat_RH.z;

            WBmotion->addRHOriInfo(qRH,2);
            joint->SetJointRefAngle(RHAND,-125);
            usleep(2010*1000);
            WBmotion->addRHPosInfo(via2_pos[0],via2_pos[1],via2_pos[2],2);
            usleep(2010*1000);
            joint->SetJointRefAngle(RHAND,0);
            WBmotion->addRHPosInfo(via3_pos[0],via3_pos[1],via3_pos[2],2);
            usleep(2010*1000);
            WBmotion->addRHPosInfo(via4_pos[0],via4_pos[1],via4_pos[2],2);
            WBmotion->addRElbPosInfo(WBmotion->RElb_ang*R2D - 15,2);
            usleep(2010*1000);

            //FT Sensor on and Sensor Nulling
            MCWristFTsensorNull(2, 53); //RWFT
            usleep(1000*1000);

            _task_flag = RH_APPROACH_CONTROL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

        case CarTask_AL_GRAB_RH:
        {
            cout<<"RH_grab start"<<endl;
            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceUB();
            joint->SetAllMotionOwner();

            //double grab_pos[3];

            vec3 posY_diff_rel = vec3(0,-0.66,0);
            vec3 posZ_diff_glo = vec3(0,0,-0.165);
            vec3 cur_pLH;
            quat cur_qLH;

            if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) < 0.0001){
                cout<<"Encoder is off, Simulation Mode on!"<<endl;
                //----for Simulation--------------------------------------------------------------------------------------
                cur_pLH = vec3(WBmotion->pLH_3x1[0],WBmotion->pLH_3x1[1],WBmotion->pLH_3x1[2]);
                cur_qLH = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
                //---------------------------------------------------------------------------------------------------------
            }
            else{
                cout<<"Encoder is On, Experiment mode!"<<endl;
                //----for Experiment--------------------------------------------------------------------------------------
                Enc_request(1);
                usleep(500*1000);
                double pLH_3x1_FK[3];
                double qLH_4x1_FK[4];
                double LElb_FK;
                FK_Pos_Ori_Elb_LeftArm(pLH_3x1_FK, qLH_4x1_FK, LElb_FK);
                cur_pLH = vec3(pLH_3x1_FK[0],pLH_3x1_FK[1],pLH_3x1_FK[2]);
                cur_qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3]);
                //--------------------------------------------------------------------------------------------------------
            }



            mat3 LH_mat = mat3(cur_qLH);
            vec3 posY_diff_glo = LH_mat*posY_diff_rel;
            vec3 grab_pos = cur_pLH + posY_diff_glo + posZ_diff_glo;

            double via1_pos[3], via2_pos[3], via3_pos[3], via4_pos[3];
            //doubles via1_quat(4),via2_quat(4),via3_quat(4);
            double L = 0.05; // Sliding up distance
            double l = 0.06;  // first normal distance
            double l1 = 0.03; // second normal distance
            double bar_ang = 56*D2R;

            via4_pos[0] = grab_pos.x;
            via4_pos[1] = grab_pos.y + l1*sin(bar_ang);
            via4_pos[2] = grab_pos.z - l1*cos(bar_ang);

            via3_pos[0] = via4_pos[0];
            via3_pos[1] = via4_pos[1] - L*cos(bar_ang);
            via3_pos[2] = via4_pos[2] - L*sin(bar_ang);

            via2_pos[0] = via3_pos[0];
            via2_pos[1] = via3_pos[1] + l*sin(bar_ang);
            via2_pos[2] = via3_pos[2] - l*cos(bar_ang);

            via1_pos[0] = via2_pos[0] + 0.15;
            via1_pos[1] = via2_pos[1];
            via1_pos[2] = via2_pos[2];


            WBIK_mode = RH_LH_UB;
            WB_FLAG = true;

            WBmotion->addRHPosInfo(via1_pos[0],via1_pos[1],via1_pos[2],2);
            quat cur_quat_RH(-0.158854,-0.119086,-0.862637,-0.465232);
            quat RH_rot(vec3(1,0,0),3*D2Rf);
            quat des_quat_RH = cur_quat_RH;//cur_quat_RH*RH_rot;

            doubles qRH(4);
            qRH[0] = des_quat_RH.w;
            qRH[1] = des_quat_RH.x;
            qRH[2] = des_quat_RH.y;
            qRH[3] = des_quat_RH.z;

            WBmotion->addRHOriInfo(qRH,2);
            joint->SetJointRefAngle(RHAND,-125);
            usleep(2010*1000);
            WBmotion->addRHPosInfo(via2_pos[0],via2_pos[1],via2_pos[2],2);
            usleep(2010*1000);
            joint->SetJointRefAngle(RHAND,0);
            WBmotion->addRHPosInfo(via3_pos[0],via3_pos[1],via3_pos[2],2);
            usleep(2010*1000);
            WBmotion->addRHPosInfo(via4_pos[0],via4_pos[1],via4_pos[2],2);
            WBmotion->addRElbPosInfo(WBmotion->RElb_ang*R2D - 15,2);
            usleep(2010*1000);

            //FT Sensor on and Sensor Nulling
            MCWristFTsensorNull(2, 53); //RWFT
            usleep(1000*1000);

            _task_flag = RH_APPROACH_CONTROL;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

        case CarTask_AL_GO_DESCENDING_POS:
        {
            WB_FLAG = false;
            usleep(5000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceUB();
            joint->SetAllMotionOwner();

            //------force control----------------
            if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) < 0.01){
                cout<<"Encoder is off, Simulation Mode on!"<<endl;
                RH_x = WBmotion->pRH_3x1[0]-0.1;
                RH_y = WBmotion->pRH_3x1[1];
                RSR_angle = joint->GetJointRefAngle(RSR)+OFFSET_RSR-5.0;
                RH_Fz = -80;
                LH_x = WBmotion->pLH_3x1[0]-0.1;
                LH_y = WBmotion->pLH_3x1[1];
                LSR_angle = joint->GetJointRefAngle(LSR)+OFFSET_LSR;
                LH_Fz = -50;
            }
            else{
                cout<<"Encoder is On, Experiment mode!"<<endl;
                double pLH_3x1_FK[3];
                double pRH_3x1_FK[3];

                FK_Pos_LeftArm(pLH_3x1_FK);
                FK_Pos_RightArm(pRH_3x1_FK);
                RH_x = pRH_3x1_FK[0] - 0.1;
                RH_y = pRH_3x1_FK[1];
                RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR-5.0;
                RH_Fz = -80;
                LH_x = pLH_3x1_FK[0] - 0.1;
                LH_y = pLH_3x1_FK[1];
                LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
                LH_Fz = -50;
            }
            Enc_request(1);
            usleep(1000*50);
            joint->RefreshToCurrentEncoder();
            //joint->RefreshToCurrentReference();
            zero_gain();
            usleep(1000*10);
            _task_flag = TASK_HYBRID_CONTROL;
            joint->SetJointRefAngle(RHAND, 125);
            //---------------------------------------
            //usleep(2000*1000);

            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceLB();
            joint->SetAllMotionOwner();

            double via1_pos_RF[3], via2_pos_RF[3], via3_pos_RF[3], via3_pos_LF[3];
            doubles via1_quat_RF(4),via2_quat_RF(4),via3_quat_RF(4);

            via1_pos_RF[0] = WBmotion->pRF_3x1[0] + 0.07;
            via1_pos_RF[1] = WBmotion->pRF_3x1[1];
            via1_pos_RF[2] = WBmotion->pRF_3x1[2] + 0.07;

            via2_pos_RF[0] = 0.514736 + 0.05;
            via2_pos_RF[1] = -0.105-0.02;
            via2_pos_RF[2] = -0.426808 + 0.07;

            via2_quat_RF[0] = 0.999962;
            via2_quat_RF[1] = 0.0;
            via2_quat_RF[2] = -0.0087272;
            via2_quat_RF[3] = 0.0;

            via3_pos_RF[0] = 0.514736;
            via3_pos_RF[1] = -0.105;
            via3_pos_RF[2] = -0.426808;

            via3_pos_LF[0] = 0.514736;
            via3_pos_LF[1] = 0.105;
            via3_pos_LF[2] = -0.426808;

            WBIK_mode = RF_LF_LB;
            WB_FLAG = true;

            WBmotion->addRFPosInfo(via1_pos_RF[0],via1_pos_RF[1],via1_pos_RF[2],2);
            usleep(2100*1000);
            WBmotion->addRFPosInfo(via2_pos_RF[0],via2_pos_RF[1],via2_pos_RF[2],2);
            WBmotion->addRFOriInfo(via2_quat_RF,2);
            usleep(2100*1000);
            WBmotion->addRFPosInfo(via3_pos_RF[0],via3_pos_RF[1],via3_pos_RF[2],1.5);
            WBmotion->addLFPosInfo(via3_pos_LF[0],via3_pos_LF[1],via3_pos_LF[2],1.5);
            usleep(1600*1000);


            //sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_START_DESCENDING;
            break;
        }
        case CarTask_AL_START_DESCENDING:
            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceUB();
            joint->SetAllMotionOwner();

//            Enc_request(1);
//            usleep(1000*5);
//            if(joint->RefreshToCurrentEncoder() == 0){
//                cout<<"Encoder is off!!"<<endl;
//                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
//                break;
//            }
//            zero_gain();
//            usleep(1000*5);

//            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
//            RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR;
            RH_x = WBmotion->pRH_3x1[0]-0.1;
            RH_y = WBmotion->pRH_3x1[1];
            LH_x = WBmotion->pLH_3x1[0]-0.1;
            LH_y = WBmotion->pLH_3x1[1];
            RH_Fz = -100;
            LH_Fz = -100;

            usleep(2000*1000);
            Descending_SQ_No = 1;
            _task_flag = TASK_HYBRID_CONTROL;
            Descending_flag = true;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;


        case CarTask_AL_STATIC_WALK:
            cout<<"Static Walk 1 Step Start!"<<endl;

            StaticWalk_SQ_No = 0;

            RH_grab_pos_flag = false;
            Descending_flag = false;
            Static_walk_flag = true;     //
            Stabilizing_RH_flag = false;
            Stabilizing_LH_flag = false;
            WalkReady_pos_flag = false;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_STABILIZING:
            cout<<"Stablizing RH"<<endl;

            Stabilizing_RH_SQ_No = 0;
            RH_grab_pos_flag = false;
            Descending_flag = false;
            Static_walk_flag = false;
            Stabilizing_RH_flag = true;     //
            Stabilizing_LH_flag = false;
            WalkReady_pos_flag = false;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_STABILIZING_LEFT:
            cout<<"Stabilizing LH"<<endl;

            Stabilizing_LH_SQ_No = 0;
            RH_grab_pos_flag = false;
            Descending_flag = false;
            Static_walk_flag = false;
            Stabilizing_RH_flag = false;
            Stabilizing_LH_flag = true; //
            WalkReady_pos_flag = false;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_WALKREADY_POS:
            cout<<"Go Walk Ready Pos"<<endl;
            GoWalkReady_SQ_No = 0;
            RH_grab_pos_flag = false;
            Descending_flag = false;
            Static_walk_flag = false;
            Stabilizing_RH_flag = false;
            Stabilizing_LH_flag = false;
            WalkReady_pos_flag = true;   //

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;



        case CarTask_AL_WRIST_FT_NULLING:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){
                cout<<"LWFT NULLING!!"<<endl;
                MCWristFTsensorNull(3, 54); //LWFT
                usleep(1000*1000);
                //sharedData->STATE_COMMAND = TCMD_CAR_LWFT_NULL_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]== 0){
                cout<<"RWFT NULLING!!"<<endl;
                MCWristFTsensorNull(2, 53); //RWFT
                usleep(1000*1000);
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

        case CarTask_AL_HOME_POS: // Slow Home Pos
            cout << ">>> COMMAND: WALKREADY_GO_HOMEPOS" << endl;

            WB_FLAG = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            GotoHomePos();

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        case CarTask_AL_RH_RE_GRAB_POS:
        {
            cout<<"RH Re grab pos"<<endl;
            WB_FLAG = false;
            usleep(10*1000);
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferencePELV_LH();
            joint->SetAllMotionOwner();

            double via1_pos[3], via2_pos[3], via3_pos[3], via4_pos[3];
            //doubles via1_quat(4),via2_quat(4),via3_quat(4);
            double L = 0.05; // Sliding down distance
            double l = 0.08;  // first normal distance
            double l1 = 0.05; // second normal distance
            double bar_ang = 56*D2R;

            via1_pos[0] = WBmotion->pRH_3x1[0];
            via1_pos[1] = WBmotion->pRH_3x1[1] + l1*sin(bar_ang);
            via1_pos[2] = WBmotion->pRH_3x1[2] - l1*cos(bar_ang);

            via2_pos[0] = via1_pos[0];
            via2_pos[1] = via1_pos[1] - L*cos(bar_ang);
            via2_pos[2] = via1_pos[2] - L*sin(bar_ang);

            via3_pos[0] = via2_pos[0];
            via3_pos[1] = via2_pos[1] + l*sin(bar_ang);
            via3_pos[2] = via2_pos[2] - l*cos(bar_ang);

            via4_pos[0] = via3_pos[0] + 0.15;
            via4_pos[1] = via3_pos[1];
            via4_pos[2] = via3_pos[2];


            WBIK_mode = RH_UB_LH_PC;
            WB_FLAG = true;

            joint->SetJointRefAngle(RHAND,-125);
            usleep(3010*1000);
            WBmotion->addRHPosInfo(via1_pos[0],via1_pos[1],via1_pos[2],1);
            usleep(1010*1000);
            WBmotion->addRHPosInfo(via2_pos[0],via2_pos[1],via2_pos[2],2);
            WBmotion->addRElbPosInfo(WBmotion->RElb_ang*R2D + 15,2);
            usleep(2010*1000);
            joint->SetJointRefAngle(RHAND,0);
            WBmotion->addRHPosInfo(via3_pos[0],via3_pos[1],via3_pos[2],2);
            usleep(2010*1000);
            WBmotion->addRHPosInfo(via4_pos[0],via4_pos[1],via4_pos[2],2);
            usleep(2010*1000);

            // right high gain
            cout<<"Right arm gain HIGH"<<endl;
            MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch ,0,1000);  //RSY
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch ,0,1000);  //REB
            MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch ,0,1000);  //RSY
            MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch ,0,1000);  //RWP
            MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,1000);  //RWY2
            usleep(1000*1000);

            // Give_ROI_TO_
            ////sharedData->STATE_COMMAND = TCMD_CAR_READY_TO_SCAN_MANUAL_ROI;

//            double postime=2000.;

//           joint->SetMoveJoint(RSP, -84.12, postime, MOVE_ABSOLUTE);
//           joint->SetMoveJoint(RSR, 24.48, postime, MOVE_ABSOLUTE);
//           joint->SetMoveJoint(RSY, 16.98, postime, MOVE_ABSOLUTE);
//           joint->SetMoveJoint(REB, -53.5, postime, MOVE_ABSOLUTE);
//           joint->SetMoveJoint(RWY, 98.5862, postime, MOVE_ABSOLUTE);
//           joint->SetMoveJoint(RWP, 74.86, postime, MOVE_ABSOLUTE);

//           joint->SetMoveJoint(RWY2, -113.2, postime, MOVE_ABSOLUTE);
//           usleep(2010*1000);

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }
        case CarTask_Al_AFTER_STATIC_WALK_POS:
            WB_FLAG = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            After_StaticWalk_pos();
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        case CarTask_AL_SAVE_DATA: //for JFT experiment
        {
            unsigned int saveIndex = Data_index;
            FILE* fp;
            unsigned int i;
            fp = fopen("RWFT_Z.txt", "w");

            for(i=0 ; i<saveIndex ; i++)
            {
                fprintf(fp, "%.2f \n", RWFT_Z_Data[i]);
            }
            fclose(fp);

            cout<<saveIndex<<"JFR Data Saved"<<endl;
            saveIndex=0;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

       //------------------------------Egress Mode------------------------------------------
        case EgressTask_AL_EGRESS_POS:
            cout<<"Egress Task Pose..!!"<<endl;
            WB_FLAG = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)
                Change_Pos_Egress1();
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)
                Change_Pos_Egress2();
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2)
                Egress_Driving_Pos();


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;

          default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_NO_ACT;
            break;
        }

    }


    cout << ">>> Process CarTask is terminated..!!" << endl;
    return 0;
}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void RBTaskThread(void *)
{

    while(isTerminated == 0){

        switch(_task_flag)
        {
        case IDLE:
            break;
        case TASK_GRAVITY_COMP:
            //gravity_compensation();
            if(GravityComp_Left_Flag == true)
                GravityCompensation_LeftArm();
            if(GravityComp_Right_Flag == true)
                GravityCompensation_RightArm();
            break;
        case TASK_GAIN_TUNING:
            gain_tuning(0, NULL);
            break;
        case TASK_FORCE_CONTROL:
            ForceControl_RightArm(RH_Fx,RH_Fy,RH_Fz);
            ForceControl_LeftArm(LH_Fx,LH_Fy,LH_Fz);

            //For JFT experiment
            RWFT_Z_Data[Data_index] = sharedSEN->FT[RWFT].Fz;
            Data_index++;
            if(Data_index >= 100000-10) Data_index = 0;
            break;

        case TASK_HYBRID_CONTROL:
            ZForceXYPositionControl_RightArm(RH_x,RH_y,RH_Fz,RSR_angle);
            ZForceXYPositionControl_LeftArm(LH_x,LH_y,LH_Fz,LSR_angle);
            break;
        case TASK_POSITION_CONTROL:
            PositionControlByForce_RightArm(RH_x, RH_y, RH_z, qRH_4x1, RSR_angle);
            PositionControlByForce_LeftArm(LH_x, LH_y, LH_z, qLH_4x1, LSR_angle);
            break;
        case TASK_POSITION_CONTROL_RIGHT:
            PositionControlByForce_RightArm(RH_x, RH_y, RH_z, qRH_4x1, RSR_angle);
            break;
        case TASK_POSITION_CONTROL_LEFT:
            PositionControlByForce_LeftArm(LH_x, LH_y, LH_z, qLH_4x1, LSR_angle);
            break;
        case TASK_JOINT_FORCE_CONTROL:
            JointForceControl_RightArm(RSP_angle,RSR_angle,RSY_angle,REB_angle,RWY_angle,RWP_angle,RF1_angle);
            JointForceControl_LeftArm(LSP_angle,LSR_angle,LSY_angle,LEB_angle,LWY_angle,LWP_angle,LF1_angle);
            break;
        case TASK_JOINT_FORCE_CONTROL_RIGHT:
            JointForceControl_RightArm(RSP_angle,RSR_angle,RSY_angle,REB_angle,RWY_angle,RWP_angle,RF1_angle);
            break;
        case TASK_JOINT_FORCE_CONTROL_LEFT:
            JointForceControl_LeftArm(LSP_angle,LSR_angle,LSY_angle,LEB_angle,LWY_angle,LWP_angle,LF1_angle);
            break;
        case TASK_XYZ_POSITION_CONTROL:
            XYZPositionControl_RightArm(RH_x,RH_y,RH_z,RSR_angle);
            XYZPositionControl_LeftArm(LH_x,LH_y,LH_z, LSR_angle);
            break;
        case TASK_XYZ_POSITION_CONTROL_RIGHT:
            XYZPositionControl_RightArm(RH_x,RH_y,RH_z,RSR_angle);
            break;
        case TASK_XYZ_POSITION_CONTROL_LEFT:
            XYZPositionControl_LeftArm(LH_x,LH_y,LH_z, LSR_angle);
            break;
        case RH_APPROACH_CONTROL:
            RHApproachControl();
            break;
        }

        if(RH_grab_pos_flag == true) RHGrabPos();
        else if(Descending_flag == true) DescendingSequence();
        else if(Static_walk_flag == true) StaticWalkingSequence();
        else if(Stabilizing_RH_flag == true) Stabilizing_control_RH(); //Stabilizing_control();
        else if(Stabilizing_LH_flag == true) Stabilizing_control_LH(); //Stabilizing_control_Left();
        else if(WalkReady_pos_flag == true) GoWalkReady_sequence_new(); //GoWalkReay_sequence();

//        static int counter=0;
//        if(counter == 100){
//            cout<<"RWFTx : "<<sharedSEN->FTFx[RWFT]<<", RWFTy"<<sharedSEN->FTFy[RWFT]<<", RWFTz"<<sharedSEN->FTFz[RWFT]<<endl;
//            cout<<"LWFTx : "<<sharedSEN->FTFx[LWFT]<<", LWFTy"<<sharedSEN->FTFy[LWFT]<<", LWFTz"<<sharedSEN->FTFz[LWFT]<<endl<<endl;
//            counter = 0;
//        }
//        counter++;


        if(WB_FLAG == true)
        {
            WBmotion->updateAll();

            if(WBIK_mode == RH_UB_LH_PC) WBmotion->WBIK_UB_PELV_LH();
            else if(WBIK_mode == RH_LH_UB) WBmotion->WBIK_UB();
            else if(WBIK_mode == RF_LF_LB) WBmotion->WBIK_LB();
            else if(WBIK_mode == GLOBAL_WBIK) WBmotion->WBIK();

            if(WBIK_mode != RH_LH_UB && WBIK_mode != RH_UB_LH_PC){
                for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);
            }
            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }

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

        //if(HasAnyOwnership()){
            {
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}

int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS-2;i++)
    {
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
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
int RBInitialize(void)
{
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
    if(rt_task_create(&rtFlagCon, "CARTASK_FLAG", 0, 95, 0) == 0){
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

    if(rt_task_create(&rtTaskCon, "CARTASK_TASK", 0, 90, 0) == 0){
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

// --------------------------------------------------------------------------------------------- //
void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    WBmotion->ResetGlobalCoord(_mode);//Coord

    WBmotion->StopAll();// Trajectory making ...

    if(_mode==0)
        WBmotion->RefreshToCurrentReferenceUB();
    else
        WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;


}

void PrintWBIKinfo(void)
{
    printf("--------------------POS INFO------------------------------\n");
    printf("RH pos: %f %f %f\n",WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
    printf("LH pos: %f %f %f\n",WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2]);
    printf("--------------------ROT INFO--------------------\n");
    printf("RH rot: %f %f %f %f\n",WBmotion->qRH_4x1[0], WBmotion->qRH_4x1[1], WBmotion->qRH_4x1[2], WBmotion->qRH_4x1[3]);
    printf("LH rot: %f %f %f %f\n",WBmotion->qLH_4x1[0], WBmotion->qLH_4x1[1], WBmotion->qLH_4x1[2], WBmotion->qLH_4x1[3]);
    printf("--------------------POS  FOOT-------------------\n");
    printf("Foot R: %f %f %f\n",WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2]);
    printf("Foot L: %f %f %f\n",WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], WBmotion->pLF_3x1[2]);
    printf("--------------------ROT INFO--------------------\n");
    printf("RF rot: %f %f %f %f\n",WBmotion->qRF_4x1[0], WBmotion->qRF_4x1[1], WBmotion->qRF_4x1[2], WBmotion->qRF_4x1[3]);
    printf("LF rot: %f %f %f %f\n",WBmotion->qLF_4x1[0], WBmotion->qLF_4x1[1], WBmotion->qLF_4x1[2], WBmotion->qLF_4x1[3]);
    printf("--------------------RELB INFO-------------------\n");
    printf("RI ELB: %f\n",WBmotion->RElb_ang*R2D);
    printf("LE ELB: %f\n",WBmotion->LElb_ang*R2D);
    printf("--------------------COMP INFO-------------------\n");
    printf("COM X : %f\n",WBmotion->pCOM_2x1[0]);
    printf("COM Y : %f\n",WBmotion->pCOM_2x1[1]);
    printf("--------------------PEL XY POS------------------\n");
    printf("PEL Z : %f\n",WBmotion->pPelZ);
}

// --------------------------------------------------------------------------------------------- //
void Change_Pos_CAR1(void){ // final pos3
    double postime;

    if(pos_status == 0){
        postime = 10000.;
    }
    if(pos_status == 1 || pos_status == 2){
        postime = 5000.0;
    }
    else postime = 10000.0;

//    joint->SetMoveJoint(RSP, -146.251, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, -1.73288, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, 106.212, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -91.7745, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, -106.796, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 75.104, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, -209.739, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, 42.1075, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, -57.4884, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -96.8959, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 52.0206, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 86.8282, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(WST, -90.0, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(RWY2, -23.8895, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY2, -37.9602, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(RHY, -19.8576, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RHR, -5.30472, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RHP, -97.8237, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RKN, 88.6488, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RAP, 23.8626, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RAR, 1.03785, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LHY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LHR, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LHP, -101.69, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LKN, 100.6, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LAP, -0.908739, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LAR, 0.0, postime, MOVE_ABSOLUTE);
    //usleep(10000*1000);
    joint->SetMoveJoint(RSP, -142.61, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -4.75, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 107.13, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -92.05, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -102.59, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 75.18, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -212.16, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 41.199, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -58.27, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -103.47, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 57.45, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 88.89, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, -90.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -50.15, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, -30.73, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RHY, -19.86, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, -5.30, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, -97.82, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, 91.65, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, 23.80, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, 1.04, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, -101.69, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, 101.6, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, -0.91, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, 0.0, postime, MOVE_ABSOLUTE);
    //sharedData->STATE_COMMAND = TCMD_CAR_STARTING_POS_DONE;
    pos_status = 1;
}

void Change_Pos_CAR2(void){ // pre pose
    double postime;

    if(pos_status == 0){
        postime = 10000.;
    }
    if(pos_status == 1 || pos_status == 2){
        postime = 5000.0;
    }
    else postime = 10000.0;

    joint->SetMoveJoint(RSP, -126.251, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -41.73288, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 106.212, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -126.7745, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -106.796, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 15.104, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -209.739, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 62.1075, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -57.4884, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -126.8959, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 52.0206, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 86.8282, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, -90.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -23.8895, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, -37.9602, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RHY, -19.8576, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, -5.30472, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, -97.8237, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, 91.6488, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, 23.8626, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, 1.03785, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, -101.69, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, 101.6, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, -0.908739, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, 0.0, postime, MOVE_ABSOLUTE);
    //usleep(10000*1000);
    //sharedData->STATE_COMMAND = TCMD_CAR_STARTING_POS_DONE;
    pos_status = 2;
}

void GotoHomePos(){
    double postime = 10000.0;
    for(int i=0; i<NO_OF_JOINTS; i++)
        joint->SetMoveJoint(i, 0.0, postime, MOVE_ABSOLUTE);
    pos_status = 0;
}

void After_StaticWalk_pos(void){
    double postime=2000.;
    joint->SetMoveJoint(RSP, -169.12, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -5.48, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, -5.98, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -88.5, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 19.5862, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 69.86, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -6.2, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -177.12, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -11.48, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -3.98, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -59.5, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 5.5862, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 59.86, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LWY2, 5.2, postime, MOVE_ABSOLUTE);
}




void PrintJointRef(void){
    cout<<"RHY :"<<joint->GetJointRefAngle(RHY)<<" RHR :"<<joint->GetJointRefAngle(RHR)<<" RHP :"<<joint->GetJointRefAngle(RHP)<<endl;
    cout<<"RKN: "<<joint->GetJointRefAngle(RKN)<<" RAP: "<<joint->GetJointRefAngle(RAP)<<" RAR: "<<joint->GetJointRefAngle(RAR)<<endl;
    cout<<"LHY: "<<joint->GetJointRefAngle(LHY)<<" LHR: "<<joint->GetJointRefAngle(LHR)<<" LHP: "<<joint->GetJointRefAngle(LHP)<<endl;
    cout<<"LKN: "<<joint->GetJointRefAngle(LKN)<<" LAP: "<<joint->GetJointRefAngle(LAP)<<" LAR: "<<joint->GetJointRefAngle(LAR)<<endl;
    cout<<"RSP: "<<joint->GetJointRefAngle(RSP)<<" RSR: "<<joint->GetJointRefAngle(RSR)<<" RSY: "<<joint->GetJointRefAngle(RSY)<<endl;
    cout<<"REB: "<<joint->GetJointRefAngle(REB)<<" RWY: "<<joint->GetJointRefAngle(RWY)<<" RWP: "<<joint->GetJointRefAngle(RWP)<<endl;
    cout<<"LSP: "<<joint->GetJointRefAngle(LSP)<<" LSR: "<<joint->GetJointRefAngle(LSR)<<" LSY: "<<joint->GetJointRefAngle(LSY)<<endl;
    cout<<"LEB: "<<joint->GetJointRefAngle(LEB)<<" LWY: "<<joint->GetJointRefAngle(LWY)<<" LWP: "<<joint->GetJointRefAngle(LWP)<<endl;
    cout<<"WST: "<<joint->GetJointRefAngle(WST)<<endl;
    cout<<"RWY2: "<<joint->GetJointRefAngle(RWY2)<<" RF2: "<<joint->GetJointRefAngle(RHAND)<<endl;
    cout<<"LF1: "<<joint->GetJointRefAngle(LWY2)<<" LF2: "<<joint->GetJointRefAngle(LHAND)<<endl;
}

void RHGrabPos(void){
    static unsigned int sequence_counter = 0;
    switch(RH_Grab_pos_SQ_No){
    case 0:
        sequence_counter = 0;
        RH_Grab_pos_SQ_No = 1;
        break;
    case 1:
        if(sequence_counter == 0){
            cout<<"RH Grab Pos Start!!"<<endl;
            WB_FLAG = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
        }
        if(sequence_counter == 20){
            double postime=1500.;

//            joint->SetMoveJoint(RSP, -177.433, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RSR, 14.9, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RSY, 131.2, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(REB, -86.3, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RWY, -149.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RWP, 62.0, postime, MOVE_ABSOLUTE);

//            joint->SetMoveJoint(RWY2, 26.2, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(RSP, -166.433, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 14.75, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 134.2, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, -76.3, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, -149.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, 64.88, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(RWY2, 32.96, postime, MOVE_ABSOLUTE);
        }
        if(sequence_counter == 330){
            double postime=2500.;

            joint->SetMoveJoint(RSP, -95.433, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 93.2, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, -60.3, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, -77.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, 89.0, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(RWY2, -92.2, postime, MOVE_ABSOLUTE);
        }
        if(sequence_counter == 840){
             double postime=3000.;

            joint->SetMoveJoint(RSP, -84.12, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 24.48, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 16.98, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, -53.5, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, 98.5862, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, 74.86, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(RWY2, -113.2, postime, MOVE_ABSOLUTE);

            joint->SetJointRefAngle(RHAND,-125);
        }
        if(sequence_counter == 1400){
            // right high gain
            sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = 0; // right arm
            sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] = 1; // high gain
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_GAIN;
        }
        sequence_counter++;
        if(sequence_counter >= 1650){
            RH_Grab_pos_SQ_No = 0;
            sequence_counter = 0;
            joint->SetJointRefAngle(RHAND, 0);
            RH_grab_pos_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_READY_TO_SCAN;
        }

        break;
    }
}

void RHApproachControl(void){
    static unsigned int ApproachingTime = 0;
    static bool approach_done_flag = false;
    static bool gainover_done_flag = false;
    if(sharedSEN->FT[RWFT].Fz < 25.0){
        quat qRH = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        mat3 RH_mat = mat3(qRH);
        vec3 Cur_pRH = vec3(WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
        vec3 Dist = vec3(0, 0, -0.0002);
        vec3 Del_pRH = RH_mat*Dist;
        vec3 GoalPoint;

        GoalPoint = Cur_pRH + Del_pRH;

        WBmotion->addRHPosInfo(GoalPoint.x, GoalPoint.y, GoalPoint.z,0.005);
        ApproachingTime++;
    }
    else if(sharedSEN->FT[RWFT].Fz > 15.0 && gainover_done_flag == false){
        cout<<"Right arm gain LOW"<<endl;
//        RBBoardSetSwitchingMode(2,15,SW_MODE_NON_COMPLEMENTARY); //RSY REB
//        RBBoardSetSwitchingMode(2,16,SW_MODE_NON_COMPLEMENTARY); //RWY RWP
        MCBoardSetSwitchingMode(2,15,SW_MODE_NON_COMPLEMENTARY); //RSY REB
        MCBoardSetSwitchingMode(2,16,SW_MODE_NON_COMPLEMENTARY); //RWY RWP

//        unsigned int temp_gain=1;
//        RBJointGainOverride(2,15,temp_gain,temp_gain,1000);//RSY REB
//        RBJointGainOverride(2,16,temp_gain,temp_gain,1000);//RWY RWP
        MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 50,1000);
        MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 20,1000);
        MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 30,1000);
//        MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 40,1000);
        MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 40,1000);
        joint->SetJointRefAngle(RHAND,125);
        gainover_done_flag = true;
    }
    else if(sharedSEN->FT[RWFT].Fz > 25.0 && approach_done_flag == false){
        cout<<"Approach is Done!!"<<endl;
        ApproachingTime = 0;
        approach_done_flag = true;

        cout<<"Right arm gain LOW"<<endl;
//        RBBoardSetSwitchingMode(2,15,SW_MODE_NON_COMPLEMENTARY); //RSY REB
//        RBBoardSetSwitchingMode(2,16,SW_MODE_NON_COMPLEMENTARY); //RWY RWP
        MCBoardSetSwitchingMode(2,15,SW_MODE_NON_COMPLEMENTARY); //RSY REB
        MCBoardSetSwitchingMode(2,16,SW_MODE_NON_COMPLEMENTARY); //RWY RWP

//        unsigned int temp_gain=1;
//        RBJointGainOverride(2,15,temp_gain,temp_gain,1000);//RSY REB
//        RBJointGainOverride(2,16,temp_gain,temp_gain,1000);//RWY RWP
        MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 60,1000);
        MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 45,1000);
        MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 55,1000);
        MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 40,1000);
        MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 40,1000);


//        //RH Grab
//        sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = 0;
//        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_HAND_GRAB;
        joint->SetJointRefAngle(RHAND,125);
    }
    if(approach_done_flag == true){
        quat qRH = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        mat3 RH_mat = mat3(qRH);
        vec3 Cur_pRH = vec3(WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
        vec3 Dist = vec3(0, 0, -0.0002);
        vec3 Del_pRH = RH_mat*Dist;
        vec3 GoalPoint;

        GoalPoint = Cur_pRH + Del_pRH;

        WBmotion->addRHPosInfo(GoalPoint.x, GoalPoint.y, GoalPoint.z,0.005);
        ApproachingTime++;
        if(ApproachingTime >= 200){
            _task_flag = IDLE;
            ApproachingTime = 0;
            approach_done_flag = false;
            gainover_done_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_RH_GRAB_DONE;
        }
    }
    if(ApproachingTime > 1000){
        _task_flag = IDLE;
        cout<<"Approaching Time is Done!!"<<endl;
        gainover_done_flag = false;
        //sharedData->STATE_COMMAND = TCMD_CAR_RH_GRAB_DONE;
    }

}

void DescendingSequence(void){
    static unsigned int sequence_counter = 0;

    switch(Descending_SQ_No){
    case 0:
        sequence_counter = 0;
        Descending_SQ_No = 1;
        break;
    case 1:
        if(sequence_counter == 0){
            cout<<"Descending Sequence Start!!"<<endl;
            // RH grab
            sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = 0;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_HAND_GRAB;

            WB_FLAG = false;
            joint->SetMoveJoint(RHP,-85,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN,69,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP,14,2000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHP,-85,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN,69,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP,14,2000,MOVE_ABSOLUTE);
        }
\
        sequence_counter++;
        if(sequence_counter >= 430){
            Descending_SQ_No = 2;
            sequence_counter = 0;
        }
        break;
    case 2:
        if(sequence_counter == 0){
            RH_x = 0.26;
            LH_x = 0.26;
            WB_FLAG = false;
            joint->SetMoveJoint(RHP,-70,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN,56,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP,-23,2000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHP,-70,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN,56,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP,-23,2000,MOVE_ABSOLUTE);
        }
\
        sequence_counter++;
        if(sequence_counter >= 430){
            Descending_SQ_No = 3;
            sequence_counter = 0;
        }
        break;
    case 3:
        if(sequence_counter == 0){
            // RH grab
            sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = 0;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_HAND_GRAB;

            RH_x = 0.2;
            LH_x = 0.2;
            // Release Ankle pitch
            //RAP
//            RBBoardSetSwitchingMode(0,4, SW_MODE_NON_COMPLEMENTARY);
//            RBJointGainOverride(0,4,0,1000,1);
            MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, SW_MODE_NON_COMPLEMENTARY);
            MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, 100, 500);

            //LAP
//            RBBoardSetSwitchingMode(1,10, SW_MODE_NON_COMPLEMENTARY);
//            RBJointGainOverride(1,10,0,1000,1);
            MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, SW_MODE_NON_COMPLEMENTARY);
            MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, 100, 500);

            WB_FLAG = false;
            joint->SetMoveJoint(RHP,-58,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN,53,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP,-23,2000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHP,-58,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN,53,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP,-23,2000,MOVE_ABSOLUTE);
        }
\
        sequence_counter++;
        if(sequence_counter >= 430){
            Descending_SQ_No = 4;
            sequence_counter = 0;
        }
        break;
    case 4:
        if(sequence_counter == 0){
            RH_x = 0.15;
            LH_x = 0.15;
            WB_FLAG = false;
            joint->SetMoveJoint(RHP,-47,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN,50,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP,-23,2000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHP,-47,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN,50,2000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP,-23,2000,MOVE_ABSOLUTE);
        }
\
        sequence_counter++;
        if(sequence_counter >= 430){
            Descending_SQ_No = 5;
            sequence_counter = 0;
        }
        break;
    case 5:
        if(sequence_counter == 0){
            // RH grab
//            sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = 0;
//            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = CarTask_AL_HAND_GRAB;
            joint->SetJointRefAngle(RHAND,125);

            RH_x = -0.1;
            LH_x = -0.1;
            WB_FLAG = false;
            joint->SetMoveJoint(RHP,-23,3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN,46,3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP,-23,3000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHP,-23,3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN,46,3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP,-23,3000,MOVE_ABSOLUTE);
        }
\
        sequence_counter++;
        if(sequence_counter >= 630){
            Descending_SQ_No = 6;
            sequence_counter = 0;
        }
        break;
    case 6:
        if(sequence_counter == 0){
            RH_x = -0.22;
            LH_x = -0.22;
            WB_FLAG = false;
            joint->SetMoveJoint(RHP,-10.85,3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN,42.3,3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP,-10.9,3000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHP,-10.85, 3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN,42.3, 3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP,-10.9,3000,MOVE_ABSOLUTE);
        }
        sequence_counter++;
        if(sequence_counter >= 650){
            Descending_SQ_No = 0;
            sequence_counter = 0;
            Descending_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_DESCEND_DONE;
        }
        break;
    }
}

//void StaticWalkingSequence(void){
//    static unsigned int sequence_counter = 0;

//    switch(StaticWalk_SQ_No){
//    case 0:
//        sequence_counter = 0;
//        StaticWalk_SQ_No = 1;
//        break;
//    case 1:     // Ankle lock and stand upright
//        if(sequence_counter == 0){
//            cout<<"SQ:1  Ankle lock and stand upright!!"<<endl;
//            Enc_request(1);
//            joint->RefreshToCurrentEncoder(); //
//            //joint->RefreshToCurrentReference(); //for experiment
//            joint->SetMotionOwner(RAP);
//            joint->SetMotionOwner(LAP);
////            RBenableFrictionCompensation(0,4,DISABLE,DISABLE);
////            RBBoardSetSwitchingMode(0,4, SW_MODE_COMPLEMENTARY);
////            RBenableFrictionCompensation(1,10,DISABLE,DISABLE);
////            RBBoardSetSwitchingMode(1,10,SW_MODE_COMPLEMENTARY);

////            RBJointGainOverride(0,4,1000,1000,1000);
////            RBJointGainOverride(1,10,1000,1000,1000);
//            MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, DISABLE);
//            MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, SW_MODE_COMPLEMENTARY);
//            MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, DISABLE);
//            MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, SW_MODE_COMPLEMENTARY);

//            MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch,0,1000);
//            MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch,0,1000);
//        }
//        if(sequence_counter == 210){
//            // Whole body Init
//            WB_FLAG = false;
//            //Enc_request(1);
//            //joint->RefreshToCurrentEncoder(); // current encoder feedback
//            joint->RefreshToCurrentReference();
//        }
//        if(sequence_counter == 220){
//            WBmotion->ResetGlobalCoord(0);//Coord
//            WBmotion->StopAll();// Trajectory making ..
//            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
//            joint->SetAllMotionOwner();
//            WBIK_mode = RF_LF_LB;
//            WB_FLAG = true;
//        }
//        if(sequence_counter == 250){
//            doubles _ori(4);
//            _ori[0] = 1;
//            _ori[1] = 0;
//            _ori[2] = 0;
//            _ori[3] = 0;
//            WBmotion->addRFOriInfo(_ori,1.5);
//            WBmotion->addLFOriInfo(_ori,1.5);
//        }
//        sequence_counter++;
//        if(sequence_counter >= 570){
//            StaticWalk_SQ_No = 2;
//            sequence_counter = 0;
//        }
//        break;
//    case 2:
//    {
//        static double pPEL[2] = {0,0};
//        static double pRH_3x1_FK[3];
//        static double pLH_3x1_FK[3];

//        if(sequence_counter == 0){
//            cout<<"SQ:2  First static walk Start!!"<<endl;
//            WB_FLAG = false;

//            //joint->RefreshToCurrentReference();
//        }
//        if(sequence_counter == 10){
//            joint->RefreshToCurrentReference(); //
//            WBmotion->ResetGlobalCoord(0);//Coord
//            WBmotion->StopAll();// Trajectory making ..
//            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
//            joint->SetAllMotionOwner();
//            WBIK_mode = RF_LF_LB;
//            WB_FLAG = true;
//            FK_Pos_RightArm(pRH_3x1_FK);
//            FK_Pos_LeftArm(pLH_3x1_FK);
//        }
//        if(sequence_counter == 20){
//            //WBmotion->addPELPosInfo(WBmotion->pPelZ - 0.06,1);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], -0.782418, 2);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], -0.782418, 2);
//        }
//        if(sequence_counter == 450){
//            RH_x = pRH_3x1_FK[0];
//            RH_y = pRH_3x1_FK[1];
//            RH_Fz = 20;
////            RH_z = WBmotion->pRH_3x1[2];
////            for(int i=0; i<4; i++){
////                qRH_4x1[i] = WBmotion->qRH_4x1[i];
////            }
//            RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR + 5.0;

//            LH_x = pLH_3x1_FK[0];
//            LH_y = pLH_3x1_FK[1];
//            LH_Fz = 20;
////            LH_z = WBmotion->pLH_3x1[2];
////            for(int i=0; i<4 ;i++){
////                qLH_4x1[i] = WBmotion->qLH_4x1[i];
////            }
//            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR+5.0;
//            _task_flag = TASK_HYBRID_CONTROL;
//        }
//        if(sequence_counter == 480){
//            //WBmotion->addCOMInfo(0,0.1,2);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 1.5);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 1.5);

//            // Shoulder Roll
//            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR+5.0;
//        }
//        if(sequence_counter > 480 && sequence_counter < 780){
//            pPEL[0] = 0;
//            pPEL[1] = pPEL[1]+(0.2/300.0);
//        }
//        if(sequence_counter == 790){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2] + 0.08,1);
//        }
//        if(sequence_counter == 1000){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]+0.25, WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2],1);
//        }
//        if(sequence_counter == 1210){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2] - 0.08,1.5);
//        }
//        if(sequence_counter == 1520){
//            //WBmotion->addCOMInfo(0.1,-0.1,4);
////            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.15, WBmotion->pRF_3x1[1] + 0.2, WBmotion->pRF_3x1[2], 4);
////            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.15, WBmotion->pLF_3x1[1] + 0.2, WBmotion->pLF_3x1[2], 4);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.045, WBmotion->pRF_3x1[1] + 0.2, WBmotion->pRF_3x1[2], 3);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.045, WBmotion->pLF_3x1[1] + 0.2, WBmotion->pLF_3x1[2], 3);
//        }
//        if(sequence_counter > 1520 && sequence_counter < 2120){
//            //pPEL[0] = pPEL[0] + (0.17/800);
//            //pPEL[0] = pPEL[0] - (0.1/800);
//            pPEL[0] = pPEL[0] + (0.15/600);
//            pPEL[1] = pPEL[1] - (0.4/600);
//        }
//        if(sequence_counter == 2130){
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]+0.08,1);
//        }
//        if(sequence_counter == 2340){
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]+0.25, WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2],1);
//        }
//        if(sequence_counter == 2550){
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]-0.08,1.5);
//        }
//        if(sequence_counter == 2860){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.015, WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 1.5);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.015, WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 1.5);
//        }
//        if(sequence_counter > 2860 && sequence_counter < 3160){
//            pPEL[0] = pPEL[0];
//            pPEL[1] = pPEL[1] + (0.2/300.0);
//        }
////        if(sequence_counter == 3970){
////            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]+0.07, WBmotion->pRF_3x1[1], -0.782418, 3);
////            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]+0.07, WBmotion->pLF_3x1[1], -0.782418, 3);
////            //WBmotion->addRFPosInfo(0.061,-0.105,-0.782418, 2);
////            //WBmotion->addRFPosInfo(0.061,0.105,-0.782418, 2);
////        }
////        if(sequence_counter > 3970 && sequence_counter < 4570){
////            pPEL[0] = pPEL[0] - (0.05/600);
////            pPEL[1] = pPEL[1];
////        }
//        RH_x = pRH_3x1_FK[0] - pPEL[0];
//        RH_y = pRH_3x1_FK[1] - pPEL[1];
//        LH_x = pLH_3x1_FK[0] - pPEL[0];
//        LH_y = pLH_3x1_FK[1] - pPEL[1];

//        sequence_counter++;
//        if(sequence_counter >= 3200 ){
////            StaticWalk_SQ_No = 3;
////            sequence_counter = 0;
//            StaticWalk_SQ_No = 0;
//            sequence_counter = 0;
//            Static_walk_flag = false;
//            //sharedData->STATE_COMMAND = TCMD_CAR_STATIC_WALKING_DONE;
//            cout<<"Static walking is done!"<<endl;
//        }
//        break;
//    }
//    case 3:
//    {
//        static double pPEL[2] = {0,0};
//        static double pRH_3x1_FK[3];
//        static double pLH_3x1_FK[3];

//        if(sequence_counter == 0){
//            cout<<"SQ:3  Second static walk Start!!"<<endl;
//            WB_FLAG = false;
////            Enc_request(1);
////            joint->RefreshToCurrentReference();
//        }
//        if(sequence_counter == 10){
//            joint->RefreshToCurrentEncoder(); // current encoder feedback
//            WBmotion->ResetGlobalCoord(0);//Coord
//            WBmotion->StopAll();// Trajectory making ..
//            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
//            joint->SetAllMotionOwner();
//            WBIK_mode = RF_LF_LB;
//            WB_FLAG = true;
//            FK_Pos_RightArm(pRH_3x1_FK);
//            FK_Pos_LeftArm(pLH_3x1_FK);
//        }
//        if(sequence_counter == 20){
//            RH_x = pRH_3x1_FK[0];
//            RH_y = pRH_3x1_FK[1];
//            RH_Fz = 10;
////            RH_z = WBmotion->pRH_3x1[2];
////            for(int i=0; i<4; i++){
////                qRH_4x1[i] = WBmotion->qRH_4x1[i];
////            }
//            //RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR;

//            LH_x = pLH_3x1_FK[0];
//            LH_y = pLH_3x1_FK[1];
//            LH_Fz = 10;
////            LH_z = WBmotion->pLH_3x1[2];
////            for(int i=0; i<4 ;i++){
////                qLH_4x1[i] = WBmotion->qLH_4x1[i];
////            }
//            //LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
//            _task_flag = TASK_HYBRID_CONTROL;
//        }
//        if(sequence_counter == 70){
//            //WBmotion->addCOMInfo(0,0.1,2);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 2);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 2);

//        }
//        if(sequence_counter > 70 && sequence_counter < 470){
//            pPEL[0] = 0;
//            pPEL[1] = pPEL[1]+(0.2/400);
//        }
//        if(sequence_counter == 470){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2] + 0.08,1);
//        }
//        if(sequence_counter == 680){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]+0.15, WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2],1);
//        }
//        if(sequence_counter == 890){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2] - 0.08,2);
//        }
//        if(sequence_counter == 1300){
//            //WBmotion->addCOMInfo(0.1,-0.1,4);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.15, WBmotion->pRF_3x1[1] + 0.2, WBmotion->pRF_3x1[2], 4);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.15, WBmotion->pLF_3x1[1] + 0.2, WBmotion->pLF_3x1[2], 4);
//        }
//        if(sequence_counter > 1300 && sequence_counter < 2100){
//            pPEL[0] = pPEL[0] + (0.15/800);
//            pPEL[1] = pPEL[1] - (0.4/800);
//        }
//        if(sequence_counter == 2110){
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]+0.08,1);
//        }
//        if(sequence_counter == 2320){
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]+0.15, WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2],1);
//        }
//        if(sequence_counter == 2530){
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]-0.08,2);
//        }
//        if(sequence_counter == 2940){
//            //WBmotion->addCOMInfo(0.1,0,3);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 3);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 3);
//        }
//        if(sequence_counter > 2940 && sequence_counter < 3540){
//            pPEL[0] = pPEL[0];
//            pPEL[1] = pPEL[1] + (0.2/600);
//        }

//        RH_x = pRH_3x1_FK[0] - pPEL[0];
//        RH_y = pRH_3x1_FK[1] - pPEL[1];
//        LH_x = pLH_3x1_FK[0] - pPEL[0];
//        LH_y = pLH_3x1_FK[1] - pPEL[1];

//        sequence_counter++;
//        if(sequence_counter >= 3550 ){
//            StaticWalk_SQ_No = 0;
//            sequence_counter = 0;
//            Static_walk_flag = false;
//            //sharedData->STATE_COMMAND = TCMD_CAR_STATIC_WALKING_DONE;
//            cout<<"Static walking is done!"<<endl;
//        }
//        break;
//    }

//    }
//}

void StaticWalkingSequence(void){
    static unsigned int sequence_counter = 0;

    switch(StaticWalk_SQ_No){
    case 0:
        sequence_counter = 0;
        StaticWalk_SQ_No = 1;
        break;
    case 1:     // Ankle lock and stand upright
        if(sequence_counter == 0){
            cout<<"SQ:1  Ankle lock and stand upright!!"<<endl;
            Enc_request(1);
            joint->RefreshToCurrentEncoder(); //
            //joint->RefreshToCurrentReference(); //for experiment
            joint->SetMotionOwner(RAP);
            joint->SetMotionOwner(LAP);
//            RBenableFrictionCompensation(0,4,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(0,4, SW_MODE_COMPLEMENTARY);
//            RBenableFrictionCompensation(1,10,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(1,10,SW_MODE_COMPLEMENTARY);

//            RBJointGainOverride(0,4,1000,1000,1000);
//            RBJointGainOverride(1,10,1000,1000,1000);
            MCenableFrictionCompensation(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, SW_MODE_COMPLEMENTARY);

            MCJointGainOverride(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, JOINT_INFO[RAP].mch,0,1000);
            MCJointGainOverride(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, JOINT_INFO[LAP].mch,0,1000);
        }
        if(sequence_counter == 210){
            // Whole body Init

            WB_FLAG = false;
            //Enc_request(1);
            //joint->RefreshToCurrentEncoder(); // current encoder feedback
            joint->RefreshToCurrentReference();
        }
        if(sequence_counter == 220){
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
            joint->SetAllMotionOwner();
            WBIK_mode = RF_LF_LB;
            WB_FLAG = true;
        }
        if(sequence_counter == 250){
            doubles _ori(4);
            _ori[0] = 1;
            _ori[1] = 0;
            _ori[2] = 0;
            _ori[3] = 0;
            WBmotion->addRFOriInfo(_ori,1.5);
            WBmotion->addLFOriInfo(_ori,1.5);
        }
        sequence_counter++;
        if(sequence_counter >= 570){
            StaticWalk_SQ_No = 2;
            sequence_counter = 0;
        }
        break;
    case 2:
    {
        static bool Ank_stabilize_flag = false;
        static double pRH_3x1_FK[3];
        static double pLH_3x1_FK[3];

        if(sequence_counter == 0){
            WB_FLAG = false;

            //joint->RefreshToCurrentReference();
        }
        if(sequence_counter == 20){
            joint->RefreshToCurrentReference(); //
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
            joint->SetAllMotionOwner();
            WBIK_mode = RF_LF_LB;
            WB_FLAG = true;
            FK_Pos_RightArm(pRH_3x1_FK);
            FK_Pos_LeftArm(pLH_3x1_FK);
        }
        if(sequence_counter == 50){
            //WBmotion->addPELPosInfo(WBmotion->pPelZ - 0.06,1);
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], -0.782418, 2);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], -0.782418, 2);
        }
        if(sequence_counter == 500){
            WB_FLAG = false;
            RH_x = pRH_3x1_FK[0];
            RH_y = pRH_3x1_FK[1];
            RH_Fz = 20;
//            RH_z = WBmotion->pRH_3x1[2];
//            for(int i=0; i<4; i++){
//                qRH_4x1[i] = WBmotion->qRH_4x1[i];
//            }
            RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR + 5.0;

            LH_x = pLH_3x1_FK[0];
            LH_y = pLH_3x1_FK[1];
            LH_Fz = 20;
//            LH_z = WBmotion->pLH_3x1[2];
//            for(int i=0; i<4 ;i++){
//                qLH_4x1[i] = WBmotion->qLH_4x1[i];
//            }
            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR+5.0;
            _task_flag = TASK_HYBRID_CONTROL;
        }
        if(sequence_counter > 600){
            double Ank_Moment_My = (sharedSEN->FT[RAFT].My + sharedSEN->FT[LAFT].My)/2.0;
            if(Ank_Moment_My > 25.0){
                RH_x = RH_x - 0.00007;
                LH_x = LH_x - 0.00007;
                Ank_stabilize_flag = false;
            }
            else if(Ank_Moment_My < -25.0){
                RH_x = RH_x + 0.00007;
                LH_x = LH_x + 0.00007;
                Ank_stabilize_flag = false;
            }
            else{
                Ank_stabilize_flag = true;
            }
            if(sequence_counter%100 == 0){
                cout<<"Ank Moment : "<<Ank_Moment_My<<endl;
            }
        }
        sequence_counter++;
        if(Ank_stabilize_flag == true || sequence_counter >= 2600){
            Ank_stabilize_flag = false;
            StaticWalk_SQ_No = 3;
            sequence_counter = 0;
            //sharedData->STATE_COMMAND = TCMD_CAR_ANK_STABILIZATION_DONE;
        }
        break;
    }


    case 3:
    {
        static double pPEL[2] = {0,0};
        static double pRH_3x1_FK[3];
        static double pLH_3x1_FK[3];

        if(sequence_counter == 0){
            cout<<"SQ:3  First static walk Start!!"<<endl;

            //joint->RefreshToCurrentReference();
        }
        if(sequence_counter == 10){
            joint->RefreshToCurrentReference(); //
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
            joint->SetAllMotionOwner();
            WBIK_mode = RF_LF_LB;
            WB_FLAG = true;
            FK_Pos_RightArm(pRH_3x1_FK);
            FK_Pos_LeftArm(pLH_3x1_FK);
        }
//        if(sequence_counter == 20){
//            //WBmotion->addPELPosInfo(WBmotion->pPelZ - 0.06,1);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], -0.782418, 2);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], -0.782418, 2);
//        }
        if(sequence_counter == 15){
            RH_x = pRH_3x1_FK[0];
            RH_y = pRH_3x1_FK[1];
            RH_Fz = 20;
//            RH_z = WBmotion->pRH_3x1[2];
//            for(int i=0; i<4; i++){
//                qRH_4x1[i] = WBmotion->qRH_4x1[i];
//            }
           // RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR + 5.0;

            LH_x = pLH_3x1_FK[0];
            LH_y = pLH_3x1_FK[1];
            LH_Fz = 20;
//            LH_z = WBmotion->pLH_3x1[2];
//            for(int i=0; i<4 ;i++){
//                qLH_4x1[i] = WBmotion->qLH_4x1[i];
//            }
           // LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR+5.0;
            _task_flag = TASK_HYBRID_CONTROL;
        }
        if(sequence_counter == 20){
            //WBmotion->addCOMInfo(0,0.1,2);
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 1.5);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 1.5);

            // Shoulder Roll
            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR+5.0;
        }
        if(sequence_counter > 20 && sequence_counter < 320){
            pPEL[0] = 0;
            pPEL[1] = pPEL[1]+(0.2/300.0);
        }
        if(sequence_counter == 330){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2] + 0.08,1);
        }
        if(sequence_counter == 540){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]+0.25, WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2],1);
        }
        if(sequence_counter == 750){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2] - 0.08,1.5);
        }
        if(sequence_counter == 1060){
            //WBmotion->addCOMInfo(0.1,-0.1,4);
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.15, WBmotion->pRF_3x1[1] + 0.2, WBmotion->pRF_3x1[2], 4);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.15, WBmotion->pLF_3x1[1] + 0.2, WBmotion->pLF_3x1[2], 4);
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.045, WBmotion->pRF_3x1[1] + 0.2, WBmotion->pRF_3x1[2], 3);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.045, WBmotion->pLF_3x1[1] + 0.2, WBmotion->pLF_3x1[2], 3);
        }
        if(sequence_counter > 1060 && sequence_counter < 1660){
            //pPEL[0] = pPEL[0] + (0.17/800);
            //pPEL[0] = pPEL[0] - (0.1/800);
            pPEL[0] = pPEL[0] + (0.15/600);
            pPEL[1] = pPEL[1] - (0.4/600);
        }
        if(sequence_counter == 1670){
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]+0.08,1);
        }
        if(sequence_counter == 1880){
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]+0.25, WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2],1);
        }
        if(sequence_counter == 2090){
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]-0.08,1.5);
        }
        if(sequence_counter == 2400){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.015, WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 1.5);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.015, WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 1.5);
        }
        if(sequence_counter > 2400 && sequence_counter < 2700){
            pPEL[0] = pPEL[0];
            pPEL[1] = pPEL[1] + (0.2/300.0);
        }
//        if(sequence_counter == 3970){
//            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]+0.07, WBmotion->pRF_3x1[1], -0.782418, 3);
//            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]+0.07, WBmotion->pLF_3x1[1], -0.782418, 3);
//            //WBmotion->addRFPosInfo(0.061,-0.105,-0.782418, 2);
//            //WBmotion->addRFPosInfo(0.061,0.105,-0.782418, 2);
//        }
//        if(sequence_counter > 3970 && sequence_counter < 4570){
//            pPEL[0] = pPEL[0] - (0.05/600);
//            pPEL[1] = pPEL[1];
//        }
        RH_x = pRH_3x1_FK[0] - pPEL[0];
        RH_y = pRH_3x1_FK[1] - pPEL[1];
        LH_x = pLH_3x1_FK[0] - pPEL[0];
        LH_y = pLH_3x1_FK[1] - pPEL[1];

        sequence_counter++;
        if(sequence_counter >= 2750 ){
//            StaticWalk_SQ_No = 3;
//            sequence_counter = 0;
            StaticWalk_SQ_No = 0;
            sequence_counter = 0;
            Static_walk_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_STATIC_WALKING_DONE;
            cout<<"Static walking is done!"<<endl;
        }
        break;
    }
    case 4:
    {
        static double pPEL[2] = {0,0};
        static double pRH_3x1_FK[3];
        static double pLH_3x1_FK[3];

        if(sequence_counter == 0){
            cout<<"SQ:4  Second static walk Start!!"<<endl;
            WB_FLAG = false;
//            Enc_request(1);
//            joint->RefreshToCurrentReference();
        }
        if(sequence_counter == 10){
            joint->RefreshToCurrentEncoder(); // current encoder feedback
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceLB(); // Lower Body
            joint->SetAllMotionOwner();
            WBIK_mode = RF_LF_LB;
            WB_FLAG = true;
            FK_Pos_RightArm(pRH_3x1_FK);
            FK_Pos_LeftArm(pLH_3x1_FK);
        }
        if(sequence_counter == 20){
            RH_x = pRH_3x1_FK[0];
            RH_y = pRH_3x1_FK[1];
            RH_Fz = 10;
//            RH_z = WBmotion->pRH_3x1[2];
//            for(int i=0; i<4; i++){
//                qRH_4x1[i] = WBmotion->qRH_4x1[i];
//            }
            //RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR;

            LH_x = pLH_3x1_FK[0];
            LH_y = pLH_3x1_FK[1];
            LH_Fz = 10;
//            LH_z = WBmotion->pLH_3x1[2];
//            for(int i=0; i<4 ;i++){
//                qLH_4x1[i] = WBmotion->qLH_4x1[i];
//            }
            //LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
            _task_flag = TASK_HYBRID_CONTROL;
        }
        if(sequence_counter == 70){
            //WBmotion->addCOMInfo(0,0.1,2);
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 2);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 2);

        }
        if(sequence_counter > 70 && sequence_counter < 470){
            pPEL[0] = 0;
            pPEL[1] = pPEL[1]+(0.2/400);
        }
        if(sequence_counter == 470){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2] + 0.08,1);
        }
        if(sequence_counter == 680){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]+0.15, WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2],1);
        }
        if(sequence_counter == 890){
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2] - 0.08,2);
        }
        if(sequence_counter == 1300){
            //WBmotion->addCOMInfo(0.1,-0.1,4);
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0]-0.15, WBmotion->pRF_3x1[1] + 0.2, WBmotion->pRF_3x1[2], 4);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]-0.15, WBmotion->pLF_3x1[1] + 0.2, WBmotion->pLF_3x1[2], 4);
        }
        if(sequence_counter > 1300 && sequence_counter < 2100){
            pPEL[0] = pPEL[0] + (0.15/800);
            pPEL[1] = pPEL[1] - (0.4/800);
        }
        if(sequence_counter == 2110){
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]+0.08,1);
        }
        if(sequence_counter == 2320){
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0]+0.15, WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2],1);
        }
        if(sequence_counter == 2530){
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1],WBmotion->pLF_3x1[2]-0.08,2);
        }
        if(sequence_counter == 2940){
            //WBmotion->addCOMInfo(0.1,0,3);
            WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1] - 0.1, WBmotion->pRF_3x1[2], 3);
            WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1] - 0.1, WBmotion->pLF_3x1[2], 3);
        }
        if(sequence_counter > 2940 && sequence_counter < 3540){
            pPEL[0] = pPEL[0];
            pPEL[1] = pPEL[1] + (0.2/600);
        }

        RH_x = pRH_3x1_FK[0] - pPEL[0];
        RH_y = pRH_3x1_FK[1] - pPEL[1];
        LH_x = pLH_3x1_FK[0] - pPEL[0];
        LH_y = pLH_3x1_FK[1] - pPEL[1];

        sequence_counter++;
        if(sequence_counter >= 3550 ){
            StaticWalk_SQ_No = 0;
            sequence_counter = 0;
            Static_walk_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_STATIC_WALKING_DONE;
            cout<<"Static walking is done!"<<endl;
        }
        break;
    }

    }
}

void Stabilizing_control_RH(void){
    static unsigned int sequence_counter = 0;
    switch(Stabilizing_RH_SQ_No){
    case 0:
        sequence_counter = 0;
        Stabilizing_RH_SQ_No = 1;
        break;
    case 1: // Refresh to Current pos
    {
        static double pRH_3x1_FK[3];
        static double qRH_4x1_FK[4];
        static double pLH_3x1_FK[3];
        static double qLH_4x1_FK[4];
        static double RElb_FK, LElb_FK;

        if(sequence_counter == 0){
            cout<<"SQ:1  Refresh to Current Hand pos!!"<<endl;
            WB_FLAG = false;
            joint->RefreshToCurrentEncoder(); // current encoder feedback
        }
        if(sequence_counter == 20){
           FK_Pos_Ori_Elb_RightArm(pRH_3x1_FK, qRH_4x1_FK, RElb_FK);
           FK_Pos_Ori_Elb_LeftArm(pLH_3x1_FK, qLH_4x1_FK, LElb_FK);
//            FK_Pos_RightArm(pRH_3x1_FK);
//            FK_Pos_LeftArm(pLH_3x1_FK);
        }
        if(sequence_counter == 30){
            RH_x = pRH_3x1_FK[0];
            RH_y = pRH_3x1_FK[1];
            RH_z = pRH_3x1_FK[2];
            for(int i=0; i<4; i++){
                qRH_4x1[i] = qRH_4x1_FK[i];
            }
            RSR_angle = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR;

            LH_x = pLH_3x1_FK[0];
            LH_y = pLH_3x1_FK[1];
            LH_z = pLH_3x1_FK[2];
            for(int i=0; i<4 ;i++){
                qLH_4x1[i] = qLH_4x1_FK[i];
            }
            LSR_angle = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR;
            //LSR_angle = 15;
            _task_flag = TASK_XYZ_POSITION_CONTROL_RIGHT;
        }
        sequence_counter++;
        if(sequence_counter >= 230){
            Stabilizing_RH_SQ_No = 2;
            sequence_counter = 0;
        }
        break;
    }
    case 2:
    {
        static double pRH_3x1_FK[3];
        static double qRH_4x1_FK[4];
        static double RElb_FK;
        const double force_threshold = 15;
        static vec3 Cur_pRH;
        static vec3 GoalPoint;
        static double move_lengthX = 0;
        static double move_lengthY = 0;
        static double move_lengthZ = 0;
        static bool z_OK_flag = false;
        static bool x_OK_flag = false;
        static bool y_OK_flag = false;

        if(sequence_counter == 0){
            cout<<"SQ:2  RH Force Release Start!!"<<endl;
            WB_FLAG = false;
            joint->RefreshToCurrentEncoder(); // current encoder feedback
        }
        if(sequence_counter == 20){
            FK_Pos_Ori_Elb_RightArm(pRH_3x1_FK, qRH_4x1_FK, RElb_FK);
            Cur_pRH = vec3(pRH_3x1_FK[0], pRH_3x1_FK[1], pRH_3x1_FK[2]);
        }
        if(sequence_counter > 20){
            if(sharedSEN->FT[RWFT].Fz > force_threshold){
                quat qRH = quat(qRH_4x1_FK[0],qRH_4x1_FK[1],qRH_4x1_FK[2],qRH_4x1_FK[3])  ;
                mat3 RH_mat = mat3(qRH);
                move_lengthZ = move_lengthZ + 0.00007;
                vec3 Delta = vec3(0,0,move_lengthZ);
                vec3 Del_pRH = RH_mat*Delta;
                GoalPoint = Cur_pRH + Del_pRH;
                z_OK_flag = false;
            }
            else if(sharedSEN->FT[RWFT].Fz < -force_threshold){
                quat qRH = quat(qRH_4x1_FK[0],qRH_4x1_FK[1],qRH_4x1_FK[2],qRH_4x1_FK[3])  ;
                mat3 RH_mat = mat3(qRH);
                move_lengthZ = move_lengthZ - 0.00007;
                vec3 Delta = vec3(0,0,move_lengthZ);
                vec3 Del_pRH = RH_mat*Delta;
                GoalPoint = Cur_pRH + Del_pRH;
                z_OK_flag = false;
            }
            else z_OK_flag = true;

            if(sharedSEN->FT[RWFT].Fx > force_threshold){
                quat qRH = quat(qRH_4x1_FK[0],qRH_4x1_FK[1],qRH_4x1_FK[2],qRH_4x1_FK[3])  ;
                mat3 RH_mat = mat3(qRH);
                move_lengthX = move_lengthX + 0.00007;
                vec3 Delta = vec3(move_lengthX,0,0);
                vec3 Del_pRH = RH_mat*Delta;
                GoalPoint = Cur_pRH + Del_pRH;
                x_OK_flag = false;
            }
            else if(sharedSEN->FT[RWFT].Fx < -force_threshold){
                quat qRH = quat(qRH_4x1_FK[0],qRH_4x1_FK[1],qRH_4x1_FK[2],qRH_4x1_FK[3])  ;
                mat3 RH_mat = mat3(qRH);
                move_lengthX = move_lengthX - 0.00007;
                vec3 Delta = vec3(move_lengthX,0,0);
                vec3 Del_pRH = RH_mat*Delta;
                GoalPoint = Cur_pRH + Del_pRH;
                x_OK_flag = false;
            }
            else x_OK_flag = true;

            if(sharedSEN->FT[RWFT].Fy > force_threshold){
                quat qRH = quat(qRH_4x1_FK[0],qRH_4x1_FK[1],qRH_4x1_FK[2],qRH_4x1_FK[3])  ;
                mat3 RH_mat = mat3(qRH);
                move_lengthY = move_lengthY + 0.00007;
                vec3 Delta = vec3(0,move_lengthY,0);
                vec3 Del_pRH = RH_mat*Delta;
                GoalPoint = Cur_pRH + Del_pRH;
                y_OK_flag = false;
            }
            else if(sharedSEN->FT[RWFT].Fy < -force_threshold){
                quat qRH = quat(qRH_4x1_FK[0],qRH_4x1_FK[1],qRH_4x1_FK[2],qRH_4x1_FK[3])  ;
                mat3 RH_mat = mat3(qRH);
                move_lengthY = move_lengthY - 0.00007;
                vec3 Delta = vec3(0,move_lengthY,0);
                vec3 Del_pRH = RH_mat*Delta;
                GoalPoint = Cur_pRH + Del_pRH;
                y_OK_flag = false;
            }
            else y_OK_flag = true;

            if(sequence_counter%200 == 0){
                cout<<"RH Stabilizing..."<<endl;
            }
        }


        sequence_counter++;
        //if(sharedSEN->FTFx[RWFT]*sharedSEN->FTFx[RWFT] + sharedSEN->FTFy[RWFT]*sharedSEN->FTFy[RWFT] + sharedSEN->FTFz[RWFT]*sharedSEN->FTFz[RWFT] < 100){
        if(x_OK_flag == true && y_OK_flag == true && z_OK_flag == true){
            Stabilizing_RH_SQ_No = 3;
            sequence_counter = 0;
            cout<<"RH stabilizing is done!!"<<endl;
        }
        if(sequence_counter >= 3000){
            Stabilizing_RH_SQ_No = 3;
            sequence_counter = 0;
            cout<<"RH stabilizing time over!!"<<endl;
        }

        RH_x = GoalPoint.x;
        RH_y = GoalPoint.y;
        RH_z = GoalPoint.z;
        break;
    }
    case 3:
    {
        if(sequence_counter == 0){
            cout<<"SQ:3  RH refresh position and gain override!!"<<endl;
            WB_FLAG = false;

        }
        if(sequence_counter == 50){
            //joint->RefreshToCurrentReference();
            joint->RefreshToCurrentEncoder();
//            if(joint->RefreshToCurrentEncoder() == 0){ //Encoder is off!!
//                Stabilizing_RH_flag = false;
//                Stabilizing_RH_SQ_No = 0;
//                sequence_counter = 0;
//                cout<<"Encoder is off!! stabilizing RH is terminated"<<endl;
//            }
            //for experiment

            for(int i=RSP; i<=RWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(RWY2);
        }
        if(sequence_counter == 70){
//            //--RSP
//            RBenableFrictionCompensation(2,13,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(2,13, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);

//            //--RSR
//            RBenableFrictionCompensation(2,14,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(2,14, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);

//            //--RSY, REB
//            RBenableFrictionCompensation(2,15,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(2,15, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);

            MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);

//            //---RWY, RWP
//            RBenableFrictionCompensation(2,16,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(2,16, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);

            MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);

//            //---RWY2
//            RBenableFrictionCompensation(3,36,DISABLE,DISABLE);
            MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);
        }

        if(sequence_counter == 80){
//            //---GainOverride recover
//            RBJointGainOverride(2,13,1,1,1000); //--RSP
//            RBJointGainOverride(2,14,1,1,1000); //--RSR
//            RBJointGainOverride(2,15,1,1,1000); //--RSY, REB
//            RBJointGainOverride(2,16,1,1,1000); //---RWY, RWP
//            RBJointGainOverride(2,36,1,1,1000); // RWY2
            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch,25,1000);
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch,25,1000);
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch,40,1000);
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch,40,1000);
            MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch,40,1000);
            MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch,65,1000);
            MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch,70,1000);

        }
        if(sequence_counter == 150){
            //-- Release open loop command
            _task_flag = TASK_XYZ_POSITION_CONTROL_LEFT;
            MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);
        }
        sequence_counter++;
        if(sequence_counter >= 300){
            Stabilizing_RH_SQ_No = 4;
            sequence_counter = 0;
        }
        break;
    }
    case 4:
    {
        cout<<"RKN : "<<fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition)<<endl;
        if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) <= 1.0){
            cout<<"Encoder is off, Simulation RH stabilize!!"<<endl;
            Stabilizing_RH_SQ_No = 5;
            sequence_counter = 0;
        }
        else{
            if(sequence_counter == 0){
                cout<<"SQ:4  RF2 Release!!"<<endl;
                // RH release
                joint->SetJointRefAngle(RHAND, -125);
            }
            if(sequence_counter >= 200 && sequence_counter%100 == 0){
                if(sharedSEN->ENCODER[MC_GetID(RHAND)][MC_GetCH(RHAND)].CurrentPosition < 33.0){
                        static int try_count = 0;
                        joint->SetJointRefAngle(RHAND, -125);
                        if(try_count == 10){
                            joint->SetJointRefAngle(RHAND, 125);
                            try_count = 0;
                            cout<<"RF2 release jam, retry release!!"<<endl;
                            //TCMD : RF2 release jam!!!!
                        }
                        try_count++;
                        cout<<"RF2 release Try count : "<< try_count<<endl;
                }
                else{
                    Stabilizing_RH_SQ_No = 5;
                    sequence_counter = 0;
                }
            }
            sequence_counter++;
        }
        break;
    }
    case 5:
    { 
//        if(sequence_counter == 50){
//            joint->SetMoveJoint(RSP,9,2000,MOVE_RELATIVE);
//            joint->SetMoveJoint(REB,-22,2000,MOVE_RELATIVE);
//            joint->SetMoveJoint(RWP,15,2000,MOVE_RELATIVE);
//        }
        if(sequence_counter == 50){
            int posTime = 4000;
            joint->SetMoveJoint(RSP,-114.0,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR,13.8,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY,8.27,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB,-129.6,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY,43.2,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP,26.7,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY2,2.88,posTime,MOVE_ABSOLUTE);
        }
        if(sequence_counter == 900){
            int posTime = 5000;
            joint->SetMoveJoint(RSP,28.0,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR,3.8,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY,3.0,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP,33.0,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY2,0.0,posTime,MOVE_ABSOLUTE);
        }

        if(sequence_counter == 1600){
            // RH grab
            joint->SetJointRefAngle(RHAND, 125);
        }
        if(sequence_counter == 1950){
//            RBJointGainOverride(2,13,1000,1000,1000); //--RSP
//            RBJointGainOverride(2,14,1000,1000,1000); //--RSR
//            RBJointGainOverride(2,15,1000,1000,1000); //--RSY, REB
//            RBJointGainOverride(2,16,1000,1000,1000); //---RWY, RWP
//            RBJointGainOverride(2,36,1000,1000,1000); // RWY2
            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0, 1000);
        }
        if(sequence_counter == 2150){
            MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);
        }
        sequence_counter++;
        if(sequence_counter >= 2300){
            joint->SetJointRefAngle(RHAND, 0);
            Stabilizing_RH_SQ_No = 0;
            sequence_counter = 0;
            Stabilizing_RH_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_RH_STABILIZING_DONE;
        }
        break;
    }
    }
}

void Stabilizing_control_LH(void){
    static unsigned int sequence_counter = 0;
    switch(Stabilizing_LH_SQ_No){
    case 0:
        sequence_counter = 0;
        Stabilizing_LH_SQ_No = 1;
        break;
    case 1:
    {
        if(sequence_counter == 0){
            cout<<"SQ1 : RH counter mass position!"<<endl;
            Enc_request(1);
            WB_FLAG = false;
        }
        if(sequence_counter == 10){
            //joint->RefreshToCurrentEncoder(); //Encoder Feedback;
            joint->RefreshToCurrentReference();
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->RefreshToCurrentReferenceUB();
            joint->SetAllMotionOwner();
            WBIK_mode = RH_LH_UB;
            WB_FLAG = true;
        }
        if(sequence_counter == 20){
            WBmotion->addRHPosInfo(0.52457, -0.2824, 0.09174, 2);
            WBmotion->addRElbPosInfo(-22.3465,2);
            doubles RH_ori(4);
            RH_ori[0] = 0.867961;
            RH_ori[1] = -0.015146;
            RH_ori[2] = -0.494713;
            RH_ori[3] = -0.040905;

            WBmotion->addRHOriInfo(RH_ori,2);

        }
        sequence_counter++;
        if(sequence_counter >= 430){
            Stabilizing_LH_SQ_No = 2;
            sequence_counter = 0;
        }
        break;
    }
    case 2:
    {
        static double pLH_3x1_FK[3];
        static double qLH_4x1_FK[4];
        static double LElb_FK;
        const double force_threshold = 8.0;
        static vec3 Cur_pLH;
        static vec3 GoalPoint;
        static double move_lengthX = 0;
        static double move_lengthY = 0;
        static double move_lengthZ = 0;
        static bool z_OK_flag = false;
        static bool x_OK_flag = false;
        static bool y_OK_flag = false;


        if(sequence_counter == 0){
            cout<<"SQ:4  LH force release start!!"<<endl;
            WB_FLAG = false;
            joint->RefreshToCurrentEncoder(); // current encoder feedback

            z_OK_flag = false;
            x_OK_flag = false;
            y_OK_flag = false;

            move_lengthX = 0;
            move_lengthY = 0;
            move_lengthZ = 0;
        }
        if(sequence_counter == 20){
            FK_Pos_Ori_Elb_LeftArm(pLH_3x1_FK, qLH_4x1_FK, LElb_FK);
            Cur_pLH = vec3(pLH_3x1_FK[0], pLH_3x1_FK[1], pLH_3x1_FK[2]);
            LH_x = pLH_3x1_FK[0];
            LH_y = pLH_3x1_FK[1];
            LH_z = pLH_3x1_FK[2];
            for(int i=0; i<4; i++){
                qLH_4x1[i] = qLH_4x1_FK[i];
            }
            _task_flag = TASK_XYZ_POSITION_CONTROL_LEFT;
        }

        if(sequence_counter > 50){
            if(sharedSEN->FT[LWFT].Fz > force_threshold){
                quat qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3])  ;
                mat3 LH_mat = mat3(qLH);
                move_lengthZ = move_lengthZ + 0.00007;
                vec3 Delta = vec3(0,0,move_lengthZ);
                vec3 Del_pLH = LH_mat*Delta;
                GoalPoint = Cur_pLH + Del_pLH;
                z_OK_flag = false;
            }
            else if(sharedSEN->FT[LWFT].Fz < -force_threshold){
                quat qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3])  ;
                mat3 LH_mat = mat3(qLH);
                move_lengthZ = move_lengthZ - 0.00007;
                vec3 Delta = vec3(0,0,move_lengthZ);
                vec3 Del_pLH = LH_mat*Delta;
                GoalPoint = Cur_pLH + Del_pLH;
                z_OK_flag = false;
            }
            else z_OK_flag = true;

            if(sharedSEN->FT[LWFT].Fx > force_threshold){
                quat qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3])  ;
                mat3 LH_mat = mat3(qLH);
                move_lengthX = move_lengthX + 0.00007;
                vec3 Delta = vec3(move_lengthX,0,0);
                vec3 Del_pLH = LH_mat*Delta;
                GoalPoint = Cur_pLH + Del_pLH;
                x_OK_flag = false;
            }
            else if(sharedSEN->FT[LWFT].Fx < -force_threshold){
                quat qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3])  ;
                mat3 LH_mat = mat3(qLH);
                move_lengthX = move_lengthX - 0.00007;
                vec3 Delta = vec3(move_lengthX,0,0);
                vec3 Del_pLH = LH_mat*Delta;
                GoalPoint = Cur_pLH + Del_pLH;
                x_OK_flag = false;
            }
            else x_OK_flag = true;

            if(sharedSEN->FT[LWFT].Fy > force_threshold){
                quat qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3])  ;
                mat3 LH_mat = mat3(qLH);
                move_lengthY = move_lengthY + 0.00007;
                vec3 Delta = vec3(0,move_lengthY,0);
                vec3 Del_pLH = LH_mat*Delta;
                GoalPoint = Cur_pLH + Del_pLH;
                y_OK_flag = false;
            }
            else if(sharedSEN->FT[LWFT].Fy < -force_threshold){
                quat qLH = quat(qLH_4x1_FK[0],qLH_4x1_FK[1],qLH_4x1_FK[2],qLH_4x1_FK[3])  ;
                mat3 LH_mat = mat3(qLH);
                move_lengthY = move_lengthY - 0.00007;
                vec3 Delta = vec3(0,move_lengthY,0);
                vec3 Del_pLH = LH_mat*Delta;
                GoalPoint = Cur_pLH + Del_pLH;
                y_OK_flag = false;
            }
            else y_OK_flag = true;


            LH_x = GoalPoint.x;
            LH_y = GoalPoint.y;
            LH_z = GoalPoint.z;

            if(sequence_counter%200 == 0){
                cout<<"LH Stabilizing..."<<endl;
            }
        }

        sequence_counter++;
//        if(sharedSEN->FTFx[LWFT]*sharedSEN->FTFx[LWFT] + sharedSEN->FTFy[LWFT]*sharedSEN->FTFy[LWFT] + sharedSEN->FTFz[LWFT]*sharedSEN->FTFz[LWFT] < 50){
        if(x_OK_flag == true && y_OK_flag == true && z_OK_flag == true){
            Stabilizing_LH_SQ_No = 3;
            sequence_counter = 0;
            cout<<"LH stabilizing is done!!"<<endl;
        }
        if(sequence_counter >= 3000){
            Stabilizing_LH_SQ_No = 3;
            sequence_counter = 0;
            cout<<"LH stabilizing time is over!!"<<endl;
            cout<<"Sequence Finished!!"<<endl;
        }

        break;
    }
    case 3:
    {
        if(sequence_counter == 0){
            cout<<"SQ:3  LH refresh pos and gain override!!"<<endl;
            WB_FLAG = false;

        }
        if(sequence_counter == 50){
//            joint->RefreshToCurrentReference();
            joint->RefreshToCurrentEncoder();
//            if(joint->RefreshToCurrentEncoder() == 0){ //Encoder is off!!
//                Stabilizing_LH_flag = false;
//                Stabilizing_LH_SQ_No = 0;
//                sequence_counter = 0;
//                cout<<"Encoder is off!! stabilizing RH is terminated"<<endl;
//            }
            // for experiment

            for(int i=LSP; i<=LWP ; i++){
                joint->SetMotionOwner(i);
            }
            joint->SetMotionOwner(LWY2);
        }

        if(sequence_counter == 70){
            //--LSP
//            RBenableFrictionCompensation(3,17,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(3,17, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);

            //--LSR
//            RBenableFrictionCompensation(3,18,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(3,18, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);

            //--LSY, LEB
//            RBenableFrictionCompensation(3,19,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(3,19, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);

            //---LWY, LWP
//            RBenableFrictionCompensation(3,20,DISABLE,DISABLE);
//            RBBoardSetSwitchingMode(3,20, SW_MODE_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_NON_COMPLEMENTARY);
            MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, DISABLE);
            MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_NON_COMPLEMENTARY);

            //---LWY2
//            RBenableFrictionCompensation(3,37,DISABLE,DISABLE);
            MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, DISABLE);
        }
        if(sequence_counter == 100){
            //---GainOverride recover
//            RBJointGainOverride(3,17,1,1,2000); //--LSP
//            RBJointGainOverride(3,18,1,1,2000); //--LSR
//            RBJointGainOverride(3,19,1,1,2000); //--LSY, LEB
//            RBJointGainOverride(3,20,1,1,2000); //---LWY, LWP
//            RBJointGainOverride(3,37,1,1,2000); // LWY2
            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 25, 1000);
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 25, 1000);
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 40, 1000);
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 40, 1000);
            MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 40, 1000);
            MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 60, 1000);
            MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 50, 1000);

        }
        if(sequence_counter == 200){
            //-- Release open loop command
//            RBJointOLCurrentCommand2ch(3, 17, 0, 0, 0x05); //--LSP
//            RBJointOLCurrentCommand2ch(3, 18, 0, 0, 0x05); //--LSR
//            RBJointOLCurrentCommand2ch(3, 19, 0, 0, 0x05); //--LSY, LEB
//            RBJointOLCurrentCommand2ch(3, 20, 0, 0, 0x05); //---LWY, LWP
//            RBJointOLCurrentCommand2ch(3, 37, 0, 0, 0x05); // LWY2
            _task_flag = IDLE;
            MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
            MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);
        }
        sequence_counter++;
        if(sequence_counter >= 250){
            Stabilizing_LH_SQ_No = 4;
            sequence_counter = 0;
        }
        break;
    }
    case 4:
    {
        if(fabs(sharedSEN->ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition) < 0.01){
            cout<<"Encoder is off, Simulation LH stabilize!!"<<endl;
            Stabilizing_LH_SQ_No = 5;
            sequence_counter = 0;
        }
        else{
            if(sequence_counter == 0){
                cout<<"SQ:4  LF2 Release!!"<<endl;
                // LH release
                joint->SetJointRefAngle(LHAND, -125);
            }
            if(sequence_counter >= 200 && sequence_counter%100 == 0){
                if(sharedSEN->ENCODER[MC_GetID(LHAND)][MC_GetCH(LHAND)].CurrentPosition < 33.0){
                        static int try_count = 0;
                        joint->SetJointRefAngle(LHAND, -125);
                        if(try_count == 10){
                            joint->SetJointRefAngle(LHAND, 125);
                            try_count = 0;
                            cout<<"LF2 release jam, retry release!!"<<endl;
                            //TCMD : LF release jam!!!!
                        }
                        try_count++;
                        cout<<"LF release Try count : "<< try_count<<endl;
                }
                else{
                    Stabilizing_LH_SQ_No = 5;
                    sequence_counter = 0;
                }
            }
            sequence_counter++;
        }
        break;
    }
    case 5:
    {
//        if(sequence_counter == 50){
//            int posTime = 2000;
//            joint->SetMoveJoint(LSP, 8.0,posTime,MOVE_RELATIVE);
//            joint->SetMoveJoint(LEB, -25.0, posTime, MOVE_RELATIVE);
//            joint->SetMoveJoint(LWP, 10.0, posTime, MOVE_RELATIVE);
//        }
        if(sequence_counter == 50){
            WB_FLAG = false;
            joint->RefreshToCurrentReference();
            WBmotion->StopAll();// Trajectory making ..
            WBmotion->ResetGlobalCoord(0);
            WBmotion->RefreshToCurrentReferenceUB();
            WBIK_mode = RH_LH_UB;
            WB_FLAG = true;
        }
        if(sequence_counter == 60){
            //RH counter mass position
            WBmotion->addRHPosInfo(0.27, -0.2824, 0.1085, 2);
            doubles RH_ori(4);
            RH_ori[0] = 0.8679;
            RH_ori[1] = -0.015146;
            RH_ori[2] = -0.49471;
            RH_ori[3] = -0.040905;
            WBmotion->addRHOriInfo(RH_ori,2);
        }

        if(sequence_counter == 470){
            WB_FLAG = false;
            int posTime = 4000;
            joint->SetMoveJoint(LSP, -134.0,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSR, -3.94, posTime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSY, -4.33, posTime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LEB, -134.6,posTime,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY, -10.1, posTime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWP, 50.0, posTime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY2, 21.7,posTime,MOVE_ABSOLUTE);
        }

        if(sequence_counter == 1100){
            //LH grab
            joint->SetJointRefAngle(LHAND,125);

            // usleep time should be considered!!!
        }
        sequence_counter++;

        if(sequence_counter >= 1550){
            joint->SetJointRefAngle(LHAND,0);
            Stabilizing_LH_SQ_No = 0;
            sequence_counter = 0;
            Stabilizing_LH_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_LH_STABILIZING_DONE;
        }
        break;
    }
}
}

void GoWalkReady_sequence_new(void){
    static unsigned int sequence_counter = 0;
    switch(GoWalkReady_SQ_No){
    case 0:
        sequence_counter = 0;
        GoWalkReady_SQ_No = 1;
        break;

    case 1:
    {
        if(sequence_counter == 0){
            //cout<<"Position Lock!!"<<endl;
            WB_FLAG = false;
            _task_flag = IDLE;

        }
        sequence_counter++;
        if(sequence_counter >= 30){
            GoWalkReady_SQ_No = 2;
            sequence_counter = 0;
        }
        break;
    }
    case 2:
        if(sequence_counter == 0){
            cout<<"Go to Walk Ready pos"<<endl;
            WB_FLAG = false;
            joint->SetAllMotionOwner();
            double postime = 4000.;
            joint->SetMoveJoint(RSP, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, -140, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, -30.0, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(LSP, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LEB, -140.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWP, -30.0, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);

//            joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//            joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);

//            joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//            joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);

//            joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHP, -40.66, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN, 75.02, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP, -34.35, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHP, -40.66, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN, 75.02, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP, -34.35, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
            joint->SetJointRefAngle(RHAND, 125);
            joint->SetJointRefAngle(LHAND, 125);
        }
        if(sequence_counter == 850){
//            RBJointGainOverride(3,17,1000,1000,1000); //--LSP
//            RBJointGainOverride(3,18,1000,1000,1000); //--LSR
//            RBJointGainOverride(3,19,1000,1000,1000); //--LSY, LEB
//            RBJointGainOverride(3,20,1000,1000,1000); //---LWY, LWP
//            RBJointGainOverride(3,37,1000,1000,1000); //---LWY2
            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 0, 1000);
            MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 0, 2000);
            MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0, 2000);
        }

        if(sequence_counter == 870){
            MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_COMPLEMENTARY);
            MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_COMPLEMENTARY);
            joint->SetJointRefAngle(RHAND, 0);
            joint->SetJointRefAngle(LHAND, 0);
        }
        sequence_counter++;
        if(sequence_counter >= 900){
            GoWalkReady_SQ_No = 0;
            sequence_counter = 0;
            WalkReady_pos_flag = false;
            //sharedData->STATE_COMMAND = TCMD_CAR_WALK_REDEAY_DONE;
        }
        break;
    }
}

//-------------------Egress mode-----------------------------------------------------------------
void Change_Pos_Egress1(void){
    double postime=10000.;

    joint->SetMoveJoint(RSP, -146.251, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -1.73288, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 106.212, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -91.7745, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -106.796, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 75.104, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -209.739, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 42.1075, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -57.4884, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -96.8959, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 52.0206, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 86.8282, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, -90.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -23.8895, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, -37.9602, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RHY, -19.8576, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, -5.30472, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, -97.8237, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, 88.6488, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, 23.8626, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, 1.03785, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, -101.69, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, 100.6, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, -0.908739, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, 0.0, postime, MOVE_ABSOLUTE);
    usleep(10000*1000);
}

void Change_Pos_Egress2(void){
    double postime=10000.;

    joint->SetMoveJoint(RSP, -145.34, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -8.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 107.60, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -98.27, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -103.52, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 74.50, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -199.68, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 32.89, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -64.2, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -92.17, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 36.18, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 80.66, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, -90.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -24.53, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, -31.04, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RHY, -119.86, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, -5.30472, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, -97.8237, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, 88.6488, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, -6.14, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, 1.04, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, -101.69, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, 100.6, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, -0.91, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, 0.0, postime, MOVE_ABSOLUTE);
    usleep(10000*1000);
}

void Egress_Driving_Pos(void){
    double postime=10000.;

    joint->SetMoveJoint(RSP, -127.78, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 3.15, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 103.99, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -98.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -87.519, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 77.50, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -188.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 45.27, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -42.42, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -92.46, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 49.41, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 72.18, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, -33.06, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -24.53, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, -49.61, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RHY, -40.92, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, -3.98, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, -93.09, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, 78.31, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, -4.52, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, -7.98, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, 55.11, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, -7.03, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, -86.74, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, 81.23, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, 40.16, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, 13.67, postime, MOVE_ABSOLUTE);
    usleep(10000*1000);
}

