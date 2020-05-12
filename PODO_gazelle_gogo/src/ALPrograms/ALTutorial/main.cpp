

#include "BasicFiles/BasicSetting.h"


// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *jCon;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;


enum TUTORIAL_COMMAND
{
    TUTORIAL_NO_ACT = 100,
    TUTORIAL_FINGER_CONTROL
};

void FingerControl(char right_left, char finger, char current);

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
    sprintf(__AL_NAME, "ALTutorial");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    jCon->SetMotionOwner(0);
    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case 999:
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        case TUTORIAL_FINGER_CONTROL:
            FILE_LOG(logSUCCESS) << "Command TUTORIAL_FINGER_CONTROL received..";
            FingerControl(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0],
                    sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1],
                    sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2]);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = TUTORIAL_NO_ACT;
            break;
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



void FingerControl(char right_left, char finger, char current){
    if(right_left == 0){
        // Right Hand
        if(finger == 0){
            jCon->SetMotionOwner(RHAND);
            jCon->SetMoveJoint(RHAND, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 1){
            jCon->SetMotionOwner(RF1);
            jCon->SetMoveJoint(RF1, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 2){
            jCon->SetMotionOwner(RF2);
            jCon->SetMoveJoint(RF2, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 3){
            jCon->SetMotionOwner(RF3);
            jCon->SetMoveJoint(RF3, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 4){
            jCon->SetMotionOwner(RF4);
            jCon->SetMoveJoint(RF4, current, 5, MOVE_ABSOLUTE);
        }else if(finger == -1){
            jCon->SetMotionOwner(RHAND);
            jCon->SetMotionOwner(RF1);
            jCon->SetMotionOwner(RF2);
            jCon->SetMotionOwner(RF3);
            jCon->SetMotionOwner(RF4);
            jCon->SetMoveJoint(RHAND, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RF1, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RF2, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RF3, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RF4, current, 5, MOVE_ABSOLUTE);
        }
    }else if(right_left = 1){
        // Left Hand
        if(finger == 0){
            jCon->SetMotionOwner(LHAND);
            jCon->SetMoveJoint(LHAND, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 1){
            jCon->SetMotionOwner(LF1);
            jCon->SetMoveJoint(LF1, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 2){
            jCon->SetMotionOwner(LF2);
            jCon->SetMoveJoint(LF2, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 3){
            jCon->SetMotionOwner(LF3);
            jCon->SetMoveJoint(LF3, current, 5, MOVE_ABSOLUTE);
        }else if(finger == 4){
            jCon->SetMotionOwner(LF4);
            jCon->SetMoveJoint(LF4, current, 5, MOVE_ABSOLUTE);
        }else if(finger == -1){
            jCon->SetMotionOwner(LHAND);
            jCon->SetMotionOwner(LF1);
            jCon->SetMotionOwner(LF2);
            jCon->SetMotionOwner(LF3);
            jCon->SetMotionOwner(LF4);
            jCon->SetMoveJoint(LHAND, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LF1, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LF2, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LF3, current, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LF4, current, 5, MOVE_ABSOLUTE);
        }
    }
}

//==============================//
// Task Thread
//==============================//
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
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
