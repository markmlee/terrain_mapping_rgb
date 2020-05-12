

#include "BasicFiles/BasicSetting.h"


// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *jCon;
TaskMotion              *WBmotion;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;


int WB_FLAG = false;
int IsMotionOwnerOmitted();

// STATE MACHINE
#define STATUS_IDLE                 1
#define STATUS_OPERATE              0
char STATUS_FLAG=STATUS_IDLE;

// Change Pos Function
unsigned long pos_change_count=0;
char WALKtoWHEEL_FLAG=false;
char WHEELtoWALK_FLAG=false;
void WALKtoWHEEL_FUNCTION(void);
void WHEELtoWALK_FUNCTION(void);

enum OMNIWHEEL_ALCOMMAND
{
    OMNIWHEEL_AL_NO_ACT = 100,
    OMNIWHEEL_AL_CHANGEPOS
};

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "DRCWheel");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, jCon);


    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case OMNIWHEEL_AL_CHANGEPOS:
            FILE_LOG(logSUCCESS) << "Command: OMNIWHEEL_AL_CHANGEPOS received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){
                double check_1 = jCon->GetJointRefAngle(RKN);
                double check_2 = jCon->GetJointRefAngle(LKN);
                if((check_1 >130) || (check_2 >130)){
                    printf("Wrong Wheel-Walk pos change Command...!!!!!!!\n");
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
                    usleep(5*1000);
                    break;
                }
                STATUS_FLAG=STATUS_OPERATE;
                printf("-------------GOTO wheel pos transform---------!!!!\n");

                WB_FLAG = false;
                usleep(10*1000);
                jCon->SetMoveJoint(RHY, 0., 450, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHR, 0., 450, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RAR, 0., 450, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHY, 0., 450, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHR, 0., 450, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LAR, 0., 450, MOVE_ABSOLUTE);
                usleep(460*1000);

                pos_change_count=0;
                WHEELtoWALK_FLAG=false;
                WALKtoWHEEL_FLAG=true;

                usleep(20*1000);
                int whilecnt = 0;
                while(1){
                    if(WALKtoWHEEL_FLAG == false){
                        printf("Pos change success...!!!\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=600){
                        printf("Pos change time out...!!!\n");
                        break;
                    }
                    usleep(50*1000);
                }
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)
            {
                double check_1 = jCon->GetJointRefAngle(RKN);
                double check_2 = jCon->GetJointRefAngle(LKN);
                if((check_1 <130) || (check_2 <130)){
                    printf("Wrong Wheel-Walk pos change Command...!!!!!!!\n");
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
                    usleep(5*1000);
                    break;
                }

                double check_3 = fabs(jCon->GetJointRefAngle(RSP)-40);
                double check_4 = fabs(jCon->GetJointRefAngle(REB)+130);
                if((check_3>10.) || (check_4>10)){
                    WB_FLAG = false;
                    usleep(10*1000);
                }

                STATUS_FLAG=STATUS_OPERATE;
                printf("-------------GOTO walkready pos transform---------!!!!\n");

//                if(REAL_MODE == true)
//                    Knee_Gain_Return();

                pos_change_count=0;
                WALKtoWHEEL_FLAG=false;
                WHEELtoWALK_FLAG=true;

                usleep(20*1000);
                int whilecnt = 0;
                while(1){
                    if(WHEELtoWALK_FLAG == false){
                        printf("Pos change success...!!!\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=600){
                        printf("Pos change time out...!!!\n");
                        break;
                    }
                    usleep(50*1000);
                }
            }

            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



//==============================//
// Task Thread
//==============================//
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {

        if(WB_FLAG == true)
        {
            WBmotion->updateAll();
            WBmotion->WBIK();

            for(int i=RHY; i<=LAR; i++) jCon->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            jCon->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            jCon->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            jCon->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            jCon->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            jCon->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            jCon->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            jCon->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            jCon->SetJointRefAngle(RWY2,WBmotion->Q_filt_34x1[idRWY2]*R2D);

            jCon->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            jCon->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            jCon->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            jCon->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            jCon->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            jCon->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            jCon->SetJointRefAngle(LWY2,WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(IsMotionOwnerOmitted())
                WB_FLAG = false;
        }

        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}
//==============================//




int IsMotionOwnerOmitted()
{
    for(int i=0;i<NO_OF_JOINTS;i++){
        if(sharedCMD->MotionOwner[MC_GetID(i)][MC_GetCH(i)] != PODO_NO)
            return true;
    }
    return false;
}

