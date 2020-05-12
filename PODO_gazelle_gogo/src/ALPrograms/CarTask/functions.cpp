#include "functions.h"
#include "joint.h"
#include <QFile>
#include <iostream>

#include "CT_DRC_HUBO2.h"


using namespace std;

CT_DRC_HUBO2 _ct_hubo2;

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern JointControlClass *joint;


#define OFFSET_ELB      -20.0
#define OFFSET_RSR      -15.0
#define OFFSET_LSR       15.0

// --------------------------------------------------------------------------------------------- //
void Enc_request(int on)
{
    while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND != NO_ACT);

    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;
//    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = on;

    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
    cout<<"Enc_request On"<<endl;
}


// --------------------------------------------------------------------------------------------- //
int zero_gain()
{
    ZeroGainLeftArm();
    usleep(20*1000);
    ZeroGainRightArm();

//    printf(">> zero gain \n");

    //------for Manual CAN Test--------------------------------------
//    //MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 100, 8, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
//    //MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
//    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 100, 10);
//    usleep(5000);

    return 0;
}

int ZeroGainLeftArm(){
    //-- lsp
    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1500, 130, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 100, 10);
    usleep(5000);

    //-- lsr
    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 2000, 150, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 100, 10);
    usleep(5000);

    //--- lsy, leb
    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 2000, 150, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 100, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 800,80, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 100, 10);
    usleep(5000);

    //--- lwy, lwp
    MCsetFrictionParameter(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 1500, 230, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 100, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 1500, 200, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 100, 10);
    usleep(5000);

    //---- lwy2
    MCsetFrictionParameter(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 1100, 180, 0);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 100, 10);
    usleep(5000);


    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);
    usleep(5000);



    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}

int ZeroGainRightArm(){
    //-- Rsp
    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1500, 100, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 100, 10);
    usleep(5000);

    //-- Rsr
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 2000, 150, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 100, 10);
    usleep(5000);

    //--- Rsy, Reb
    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 2000, 150, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 100, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 800, 70, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 100, 10);
    usleep(5000);

    //--- Rwy, Rwp
    MCsetFrictionParameter(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 1500, 180, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 100, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 1500,230, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 100, 10);
    usleep(5000);

    //---- Rwy2
    MCsetFrictionParameter(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 1250, 170, 0);
    usleep(5000);
    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, SW_MODE_NON_COMPLEMENTARY);
    usleep(5000);
    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, ENABLE);
    usleep(5000);
    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 100, 10);
    usleep(5000);

    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
    usleep(5000);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);
    usleep(5000);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}

int ForceToPositionControl(){
    if(joint->RefreshToCurrentEncoder() == 0){
        return 0; //Encoder is off!!
    }
    else{
        //joint->SetAllMotionOwner();
        for(int i=LSP; i<=LWP ; i++){
            joint->SetMotionOwner(i);
        }
        joint->SetMotionOwner(LWY2);

        usleep(2000);

//        //--LSP
////        RBenableFrictionCompensation(3,17,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(3,17, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);

//        //--LSR
////        RBenableFrictionCompensation(3,18,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(3,18, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);

//        //--LSY, LEB
////        RBenableFrictionCompensation(3,19,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(3,19, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);


//        //---LWY, LWP
////        RBenableFrictionCompensation(3,20,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(3,20, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch,JOINT_INFO[LWY].bno, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch,JOINT_INFO[LWP].bno, SW_MODE_COMPLEMENTARY);

        //---LWY2
//        RBenableFrictionCompensation(3,37,DISABLE,DISABLE);
        MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, DISABLE);



        for(int i=RSP; i<=RWP ; i++){
            joint->SetMotionOwner(i);
        }
        joint->SetMotionOwner(RWY2);

        usleep(2000);

//        //--RSP
////        RBenableFrictionCompensation(2,13,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(2,13, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);

//        //--RSR
////        RBenableFrictionCompensation(2,14,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(2,14, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);

//        //--RSY, REB
////        RBenableFrictionCompensation(2,15,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(2,15, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);

//        //---RWY, RWP
////        RBenableFrictionCompensation(2,16,DISABLE,DISABLE);
////        RBBoardSetSwitchingMode(2,16, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch,JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
//        MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);
//        MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch,JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);

//        //---RWY2
////        RBenableFrictionCompensation(3,36,DISABLE,DISABLE);
//        MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);

        //---GainOverride recover
//        RBJointGainOverride(3,17,50,50,2000); //--LSP
//        RBJointGainOverride(3,18,50,50,2000); //--LSR
//        RBJointGainOverride(3,19,50,50,2000); //--LSY, LEB
//        RBJointGainOverride(3,20,50,50,2000); //---LWY, LWP
//        RBJointGainOverride(3,37,50,50,2000); // LWY2

//        RBJointGainOverride(2,13,50,50,2000); //--RSP
//        RBJointGainOverride(2,14,50,50,2000); //--RSR
//        RBJointGainOverride(2,15,50,50,2000); //--RSY, REB
//        RBJointGainOverride(2,16,50,50,2000); //---RWY, RWP
//        RBJointGainOverride(2,36,50,50,2000); // RWY2
//        MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 65,2000); //--LSP
//        MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 65,2000); //--LSR
//        MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 65,2000); //--LSY
//        MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 65,2000); //--LEB
//        MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 65,2000); //---LWY
//        MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 65,2000); //--LWP
        MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 65,2000); //--LF1

//        MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 65,2000); //--RSP
//        MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 65,2000); //--RSR
//        MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 65,2000); //--RSY
//        MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 65,2000); //--REB
//        MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 65,2000); //---RWY
//        MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 65,2000); //--RWP
//        MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 65,2000); //--RF1


        usleep(2000*2000);
//        RBJointGainOverride(3,17,1000,1000,1000); //--LSP
//        RBJointGainOverride(3,18,1000,1000,1000); //--LSR
//        RBJointGainOverride(3,19,1000,1000,1000); //--LSY, LEB
//        RBJointGainOverride(3,20,1000,1000,1000); //---LWY, LWP
//        RBJointGainOverride(3,37,1000,1000,1000); //---LWY2

//        RBJointGainOverride(2,13,1000,1000,1000); //--RSP
//        RBJointGainOverride(2,14,1000,1000,1000); //--RSR
//        RBJointGainOverride(2,15,1000,1000,1000); //--RSY, REB
//        RBJointGainOverride(2,16,1000,1000,1000); //---RWY, RWP
//        RBJointGainOverride(2,36,1000,1000,1000); //---RWY2
//        MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,1000); //--LSP
//        MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,1000); //--LSR
//        MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,1000); //--LSY
//        MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,1000); //--LEB
//        MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 0,1000); //---LWY
//        MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 0,1000); //--LWP
        MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0,1000); //--LF1

//        MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,1000); //--RSP
//        MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,1000); //--RSR
//        MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,1000); //--RSY
//        MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,1000); //--REB
//        MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 0,1000); //---RWY
//        MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 0,1000); //--RWP
//        MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,1000); //--RF1
        MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);

    }
    return 1;
}

int ForceToPositionControl_LeftArm(){
    if(joint->RefreshToCurrentEncoder() == 0){
        return 0; //Encoder is off!!
    }
    else{
        //joint->SetAllMotionOwner();
        for(int i=LSP; i<=LWP ; i++){
            joint->SetMotionOwner(i);
        }
        joint->SetMotionOwner(LWY2);

        usleep(2000);

        //--LSP
//        RBenableFrictionCompensation(3,17,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,17, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);

        //--LSR
//        RBenableFrictionCompensation(3,18,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,18, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);

        //--LSY, LEB
//        RBenableFrictionCompensation(3,19,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,19, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);


        //---LWY, LWP
//        RBenableFrictionCompensation(3,20,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(3,20, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch,JOINT_INFO[LWY].bno, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch,JOINT_INFO[LWP].bno, SW_MODE_COMPLEMENTARY);

        //---LWY2
//        RBenableFrictionCompensation(3,37,DISABLE,DISABLE);
        MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, DISABLE);


        //---GainOverride recover
//        RBJointGainOverride(3,17,50,50,2000); //--LSP
//        RBJointGainOverride(3,18,50,50,2000); //--LSR
//        RBJointGainOverride(3,19,50,50,2000); //--LSY, LEB
//        RBJointGainOverride(3,20,50,50,2000); //---LWY, LWP
//        RBJointGainOverride(3,37,50,50,2000); // LWY2
        MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 65,1000); //--LSP
        MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 65,1000); //--LSR
        MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 65,1000); //--LSY
        MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 50,1000); //--LEB
        MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 65,1000); //---LWY
        MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 65,1000); //--LWP
        MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 65,1000); //--LF1

        MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);

        usleep(1000*1000);
//        RBJointGainOverride(3,17,1000,1000,1000); //--LSP
//        RBJointGainOverride(3,18,1000,1000,1000); //--LSR
//        RBJointGainOverride(3,19,1000,1000,1000); //--LSY, LEB
//        RBJointGainOverride(3,20,1000,1000,1000); //---LWY, LWP
//        RBJointGainOverride(3,37,1000,1000,1000); //---LWY2
        MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,1000); //--LSP
        MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,1000); //--LSR
        MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,1000); //--LSY
        MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,1000); //--LEB
        MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 0,1000); //---LWY
        MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 0,1000); //--LWP
        MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0,1000); //--LF1


    }
    return 1;

    //Enc_request(DISABLE);
}

int ForceToPositionControl_RightArm(){
    if(joint->RefreshToCurrentEncoder() == 0){
        return 0; //Encoder is off!!
    }
    else{
        //joint->SetAllMotionOwner();
        for(int i=RSP; i<=RWP ; i++){
            joint->SetMotionOwner(i);
        }
        joint->SetMotionOwner(RWY2);

        usleep(2000);

        //--RSP
//        RBenableFrictionCompensation(2,13,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,13, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);

        //--RSR
//        RBenableFrictionCompensation(2,14,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,14, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);

        //--RSY, REB
//        RBenableFrictionCompensation(2,15,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,15, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);

        //---RWY, RWP
//        RBenableFrictionCompensation(2,16,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(2,16, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch,JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
        MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);
        MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch,JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);

        //---RWY2
//        RBenableFrictionCompensation(3,36,DISABLE,DISABLE);
        MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);

        //---GainOverride recover
//        RBJointGainOverride(2,13,50,50,2000); //--RSP
//        RBJointGainOverride(2,14,50,50,2000); //--RSR
//        RBJointGainOverride(2,15,50,50,2000); //--RSY, REB
//        RBJointGainOverride(2,16,50,50,2000); //---RWY, RWP
//        RBJointGainOverride(2,36,50,50,2000); // RWY2
        MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 65,1000); //--RSP
        MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 65,1000); //--RSR
        MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 65,1000); //--RSY
        MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 65,1000); //--REB
        MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 65,1000); //---RWY
        MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 65,1000); //--RWP
        MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 65,1000); //--RF1

        MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
        MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);
        usleep(1000*1000);
//        RBJointGainOverride(2,13,1000,1000,1000); //--RSP
//        RBJointGainOverride(2,14,1000,1000,1000); //--RSR
//        RBJointGainOverride(2,15,1000,1000,1000); //--RSY, REB
//        RBJointGainOverride(2,16,1000,1000,1000); //---RWY, RWP
//        RBJointGainOverride(2,36,1000,1000,1000); //---RWY2
        MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,1000); //--RSP
        MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,1000); //--RSR
        MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,1000); //--RSY
        MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,1000); //--REB
        MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 0,1000); //---RWY
        MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 0,1000); //--RWP
        MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,1000); //--RF1
    }
    return 1;

    //Enc_request(DISABLE);
}

// --------------------------------------------------------------------------------------------- //
int gravity_compensation()
{
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};
    int i;
//    static int counter = 0;

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_LeftArm(Q_7x1, qBase_4x1, 0, torque_7x1);

    cur_7x1[0] = torque_OLcurrent_map_LSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_LSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_LSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_LEB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_LWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_LWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_LWY2(torque_7x1[6]);

    for(i=0; i<7; i++)
    {
        if(cur_7x1[i] > 300)              cur_7x1[i] = 300;
        else if(cur_7x1[i] < -300)        cur_7x1[i] = -300;
    }

//    cout << "LSP: " << Q_7x1[0]*R2D << ", " << torque_7x1[0] << ", " << cur_7x1[0] << endl;

//    RBJointOLCurrentCommand2ch(3, 17, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, cur_7x1[6], 0, 0);

//    if(counter%200 == 0){
//        cout<<"Torque :"<<torque_7x1[0]<<", "<<torque_7x1[1]<<", "<<torque_7x1[2]<<endl;
//    }
//    counter++;

    return 0;
}

int GravityCompensation_LeftArm(){
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};
    int i;
//    static int counter = 0;

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_LeftArm(Q_7x1, qBase_4x1, 0, torque_7x1);

    cur_7x1[0] = torque_OLcurrent_map_LSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_LSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_LSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_LEB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_LWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_LWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_LWY2(torque_7x1[6]);

    for(i=0; i<7; i++)
    {
        if(cur_7x1[i] > 300)              cur_7x1[i] = 300;
        else if(cur_7x1[i] < -300)        cur_7x1[i] = -300;
    }

//    cout << "LSP: " << Q_7x1[0]*R2D << ", " << torque_7x1[0] << ", " << cur_7x1[0] << endl;

//    RBJointOLCurrentCommand2ch(3, 17, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, cur_7x1[6], 0, 0);

//    if(counter%200 == 0){
//        cout<<"Torque :"<<torque_7x1[0]<<", "<<torque_7x1[1]<<", "<<torque_7x1[2]<<endl;
//    }
//    counter++;

    return 0;
}

int GravityCompensation_RightArm(){
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};
    int i;
//    static int counter = 0;

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_RightArm(Q_7x1, qBase_4x1, 0, torque_7x1);

    cur_7x1[0] = torque_OLcurrent_map_RSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_RSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_RSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_REB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_RWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_RWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_RWY2(torque_7x1[6]);

    for(i=0; i<7; i++)
    {
        if(cur_7x1[i] > 300)              cur_7x1[i] = 300;
        else if(cur_7x1[i] < -300)        cur_7x1[i] = -300;
    }

//    cout << "LSP: " << Q_7x1[0]*R2D << ", " << torque_7x1[0] << ", " << cur_7x1[0] << endl;

//    RBJointOLCurrentCommand2ch(2, 13, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 14, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 15, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(2, 16, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(2, 36, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, cur_7x1[6], 0, 0);

//    if(counter%200 == 0){
//        cout<<"Torque :"<<torque_7x1[0]<<", "<<torque_7x1[1]<<", "<<torque_7x1[2]<<endl;
//    }
//    counter++;

    return 0;
}



// --------------------------------------------------------------------------------------------- //
int gain_tuning(int update, int cmd[])
{
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    static short cur = 0;
    const int id_temp = 3;

    FILE *fp = NULL;

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

//    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
//    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition + OFFSET_RSR)*D2R;
//    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
//    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition + OFFSET_ELB)*D2R;
//    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
//    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
//    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_LeftArm(Q_7x1, qBase_4x1, 0, torque_7x1);
    //_ct_hubo2.get_gravity_RightArm(Q_7x1, qBase_4x1, 0, torque_7x1);

    if(update == 1 && cmd!=NULL)
    {
        if(cmd[0] == 1)
            cur = cmd[1];
        else if(cmd[0] == 2)
        {
            fp = fopen("../../ALBUILD/CarTask/ct_tuning/ct_tuning_RWP.txt","a");
            fprintf(fp,"%lf %lf %d\n", Q_7x1[id_temp]*R2D, torque_7x1[id_temp], cur);
            fclose(fp);
        }
    }

    if(cur > 1000)
        cur = 1000;
    else if(cur < -1000)
        cur = -1000;

//    RBJointOLCurrentCommand2ch(3, 17, cur, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, cur, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, cur, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, cur, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, cur, 0, 0x05);

//    RBJointOLCurrentCommand2ch(2, 13, cur, 0, 0x05); //RSP
//    RBJointOLCurrentCommand2ch(2, 14, cur, 0, 0x05); //RSR
//    RBJointOLCurrentCommand2ch(2, 15, cur, 0, 0x05); //RSY REB
//    RBJointOLCurrentCommand2ch(2, 16, cur, 0, 0x05); //RWY RWP
//    RBJointOLCurrentCommand2ch(2, 36, cur, 0, 0x05); //RF1

//    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, cur, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, cur, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
//      MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, cur, 0, 0);
//   MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
//    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 3, cur, 3, cur);
//        MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, cur, 0, cur);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, cur, 0, 0);

//    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, cur, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, cur, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 0, cur, 4, cur);
//    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, cur, 0, cur);
//    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, cur, 0, cur);

//    cout << "LSP: " << Q_7x1[0]*R2D << ", " << torque_7x1[0] << ", " << cur << endl;
//    cout << "LSR: " << Q_7x1[1]*R2D << ", " << torque_7x1[1] << ", " << cur << endl;
//    cout << "LSY: " << Q_7x1[2]*R2D << ", " << torque_7x1[2] << ", " << cur << endl;
//    cout << "LEB: " << Q_7x1[3]*R2D << ", " << torque_7x1[3] << ", " << cur << endl;
//    cout << "LWY: " << Q_7x1[4]*R2D << ", " << torque_7x1[4] << ", " << cur << endl;
//    cout << "LWP: " << Q_7x1[5]*R2D << ", " << torque_7x1[5] << ", " << cur << endl;
//    cout << "LWY2: " << Q_7x1[6]*R2D << ", " << torque_7x1[6] << ", " << cur << endl;



    return 0;
}

// --------------------------------------------------------------------------------------------- //
#define SIGN(x) (x>=0 ? 1:-1)
int torque_OLcurrent_map_LSP(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 6;
    const double t_table[6] = {0, 3.0, 5.0, 7.5, 10.5, 18.1};
    const double c_table[6] = {0, 30,   40, 50, 70, 100};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LSR(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 5;
    const double t_table[5] = {0, 6.8, 13.0, 16.0, 20.3};
    const double c_table[5] = {0, 50, 80, 100, 130};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LSY(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 6;
    const double t_table[6] = {0, 1.9, 3.8, 5.7, 6.9, 9.9};
    const double c_table[6] = {0, 30, 50, 60, 80, 100};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LEB(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    // From Right ELB==================================
    const int sign = -1;
    const int n = 5;
    const double t_table[5] = {0, 1.2, 4.01, 6.6, 8.27};
    const double c_table[5] = {0, 20, 50, 80, 110};
    //==================================================

    /*const int sign = -1;
    const int n = 4;
    const double t_table[4] = {0, 2.8, 6.4, 8.5};
    const double c_table[4] = {0, 50, 70, 90};*/ //Original for Left ELB prgrammed at DRC

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LWY(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 3;
    const double t_table[3] = {0, 0.54, 0.92};
    const double c_table[3] = {0, 60,   80};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LWP(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 4;
    const double t_table[4] = {0, 1.0, 1.3, 1.5};
    const double c_table[4] = {0, 50, 60, 70};

//    const double t_table[3] = {0, 0.96, 1.16};
//    const double c_table[3] = {0, 30, 50};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_LWY2(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 3;
    const double t_table[3] = {0, 1.8, 4.8};
    const double c_table[3] = {0, 20, 60};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RSP(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 7;
    const double t_table[7] = {0, 3.2, 6.3, 9.1, 12.8, 17.8, 22.5};
    const double c_table[7] = {0, 30,   40, 50, 70, 110, 130};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RSR(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 6;
    const double t_table[6] = {0, 2.4, 4.7, 6.7,15, 21.36};
    const double c_table[6] = {0, 30, 40,  50,100, 140};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RSY(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 6;
    const double t_table[6] = {0, 2.2, 4.1, 6.0, 7.2, 10.3};
    const double c_table[6] = {0, 30, 50, 60, 80, 100};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_REB(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 5;
    const double t_table[5] = {0, 1.2, 4.01, 6.6, 8.27};
    const double c_table[5] = {0, 20, 50, 80, 110};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RWY(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 5;
//    const double t_table[3] = {0, 0.9, 1.};
//    const double c_table[3] = {0, 30,   40};
    const double t_table[5] = {0, 0.9, 1.0, 1.2, 1.4};
    const double c_table[5] = {0, 60,   80, 100, 130};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RWP(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 3;
//    const double t_table[2] = {0, 2};
//    const double c_table[2] = {0, 30};
    const double t_table[3] = {0, 0.96, 1.16};
    const double c_table[3] = {0, 30, 50};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

int torque_OLcurrent_map_RWY2(double torque_Nm)
{
    double t = fabs(torque_Nm);
    int i;
    int curr = 0;

    const int sign = -1;
    const int n = 4;
    //const double t_table[2] = {0, 2};
    //const double c_table[2] = {0, 30};
    const double t_table[4] = {0, 3.7, 4.4, 5.5};
    const double c_table[4] = {0, 30, 50, 60};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(torque_Nm));
}

//--------------------------------------------------
int go_pos(int n, int hand)
{    
    const double des_pHand[50][3] = {
        {0.55, -0.05, 0.65},
        {0.55, -0.1, 0.65},
        {0.55, -0.15, 0.65},
        {0.55, -0.2, 0.65},
        {0.55, -0.25, 0.65},
        {0.55, -0.3, 0.65},
        {0.55, -0.35, 0.65},
        {0.55, -0.4, 0.65},
        {0.55, -0.45, 0.65},
        {0.55, -0.5, 0.65},

        {0.55, -0.5, 0.55},
        {0.55, -0.45, 0.55},
        {0.55, -0.4, 0.55},
        {0.55, -0.35, 0.55},
        {0.55, -0.3, 0.55},
        {0.55, -0.25, 0.55},
        {0.55, -0.2, 0.55},
        {0.55, -0.15, 0.55},
        {0.55, -0.1, 0.55},
        {0.55, -0.05, 0.55},

        {0.55, -0.05, 0.45},
        {0.55, -0.1, 0.45},
        {0.55, -0.15, 0.45},
        {0.55, -0.2, 0.45},
        {0.55, -0.25, 0.45},
        {0.55, -0.3, 0.45},
        {0.55, -0.35, 0.45},
        {0.55, -0.4, 0.45},
        {0.55, -0.45, 0.45},
        {0.55, -0.5, 0.45},

        {0.55, -0.5, 0.35},
        {0.55, -0.45, 0.35},
        {0.55, -0.4, 0.35},
        {0.55, -0.35, 0.35},
        {0.55, -0.3, 0.35},
        {0.55, -0.25, 0.35},
        {0.55, -0.2, 0.35},
        {0.55, -0.15, 0.35},
        {0.55, -0.1, 0.35},
        {0.55, -0.05, 0.35},

        {0.55, -0.05, 0.25},
        {0.55, -0.1, 0.25},
        {0.55, -0.15, 0.25},
        {0.55, -0.2, 0.25},
        {0.55, -0.25, 0.25},
        {0.55, -0.3, 0.25},
        {0.55, -0.35, 0.25},
        {0.55, -0.4, 0.25},
        {0.55, -0.45, 0.25},
        {0.55, -0.5, 0.25},
    };

    const double time = 3000;
    double des_pLH_3x1[3];
    double des_qLH_4x1[4] = {0.707107, 0, -0.707107, 0};
    double des_LElb_ang = 60*D2R;
    double des_pRH_3x1[3];
    double des_qRH_4x1[4] = {0.707107, 0, -0.707107, 0};
    double des_RElb_ang = -60*D2R;
    double Q_34x1[34];

    _ct_hubo2.Reset_Q0();
    _ct_hubo2.get_Q0(Q_34x1);
    Q_34x1[idX] = 0;
    Q_34x1[idY] = 0;
    Q_34x1[idZ] = 0;
    Q_34x1[idQ0] = 1;
    Q_34x1[idQ1] = 0;
    Q_34x1[idQ2] = 0;
    Q_34x1[idQ3] = 0;

    Q_34x1[idRWY] = 0;
    Q_34x1[idRWP] = 0;
    Q_34x1[idRWY2] = 0;
    Q_34x1[idLWY] = 0;
    Q_34x1[idLWP] = 0;
    Q_34x1[idLWY2] = 0;
    _ct_hubo2.set_Q0(Q_34x1);

    if(hand == -1)
    {
        _ct_hubo2.FK_LeftHand_Local(Q_34x1, LOCAL_UB, des_pLH_3x1, des_qLH_4x1, des_LElb_ang);

        memcpy(des_pRH_3x1, des_pHand[n%50], 3*sizeof(double));
        _ct_hubo2.IK_UpperBody_Local(des_pRH_3x1, des_qRH_4x1, des_RElb_ang, LOCAL_UB,
                des_pLH_3x1,des_qLH_4x1, des_LElb_ang, LOCAL_UB,
                0,
                Q_34x1);

        joint->RefreshToCurrentReference();
        joint->SetAllMotionOwner();

        joint->SetMoveJoint(LSP, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSR, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSY, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LEB, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWY, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWP, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWY2, 0, time, MOVE_ABSOLUTE);

        joint->SetMoveJoint(RSP, Q_34x1[idRSP]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSR, Q_34x1[idRSR]*R2D-OFFSET_RSR, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSY, Q_34x1[idRSY]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(REB, Q_34x1[idREB]*R2D-OFFSET_ELB, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWY, Q_34x1[idRWY]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWP, Q_34x1[idRWP]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWY2, Q_34x1[idRWY2]*R2D, time, MOVE_ABSOLUTE);
    }
    else if(hand == 1)
    {
        _ct_hubo2.FK_RightHand_Local(Q_34x1, LOCAL_UB, des_pRH_3x1, des_qRH_4x1, des_RElb_ang);

        memcpy(des_pLH_3x1, des_pHand[n%50], 3*sizeof(double));
        des_pLH_3x1[1] *= -1;
        _ct_hubo2.IK_UpperBody_Local(des_pRH_3x1, des_qRH_4x1, des_RElb_ang, LOCAL_UB,
                des_pLH_3x1,des_qLH_4x1, des_LElb_ang, LOCAL_UB,
                0,
                Q_34x1);

        joint->RefreshToCurrentReference();
        joint->SetAllMotionOwner();

        joint->SetMoveJoint(LSP, Q_34x1[idLSP]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSR, Q_34x1[idLSR]*R2D-OFFSET_LSR, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSY, Q_34x1[idLSY]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LEB, Q_34x1[idLEB]*R2D-OFFSET_ELB, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWY, Q_34x1[idLWY]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWP, Q_34x1[idLWP]*R2D, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWY2, Q_34x1[idLWY2]*R2D, time, MOVE_ABSOLUTE);

        joint->SetMoveJoint(RSP, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSR, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSY, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(REB, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWY, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWP, 0, time, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWY2, 0, time, MOVE_ABSOLUTE);
    }

    cout << "go pos - done" << endl;

    return 0;
}


void save_calib(int n, int hand)
{
    FILE *fp;

    if(hand == -1)
    {
        fp = fopen("../../ALBUILD/CT_test/calib_right_joint_data.txt","a");
        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf\n",n,
                sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition);
    }
    else
    {
        fp = fopen("../../ALBUILD/CT_test/calib_left_joint_data.txt","a");
        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf\n",n,
                sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition,
                sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition);
    }

    fclose(fp);
}

void ForceControl_LeftArm(double Fx, double Fy, double Fz){

    //Gravity compensation
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};
    //int i;
//    static int counter = 0;

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_LeftArm(Q_7x1, qBase_4x1, 0, torque_7x1);

//  mapping End effector force to joint torque
    //double ENC_34x1[34] ={0,};
    double jvLH_3x34[3*34];
    double jwLH_3x34[3*34];
    double jvLELB_3x34[3*34];


//    for(int i=0; i<34; i++){
//          ENC_34x1[i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition;
//    }
//    ENC_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition;
//    ENC_34x1[idLSR] = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition;
//    ENC_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition;
//    ENC_34x1[idLEB] = sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition;
//    ENC_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition;
//    ENC_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition;
//    ENC_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition;

//    ENC_34x1[idLSP] = Q_7x1[0];
//    ENC_34x1[idLSR] = Q_7x1[1];
//    ENC_34x1[idLSY] = Q_7x1[2];
//    ENC_34x1[idLEB] = Q_7x1[3];
//    ENC_34x1[idLWY] = Q_7x1[4];
//    ENC_34x1[idLWP] = Q_7x1[5];
//    ENC_34x1[idLWY2] = Q_7x1[6];

    double Q_34x1[34];
    memset(Q_34x1,0, 34*sizeof(double));
    Q_34x1[idQ0] = 1;
    memcpy(&Q_34x1[idLSP], Q_7x1, 7*sizeof(double));

    _ct_hubo2.Jacob_LeftHand(Q_34x1, jvLH_3x34, jwLH_3x34, jvLELB_3x34);

    torque_7x1[0] += jvLH_3x34[idLSP]*Fx + jvLH_3x34[34+idLSP]*Fy + jvLH_3x34[68+idLSP]*Fz; //LSP Torque
    torque_7x1[1] += jvLH_3x34[idLSR]*Fx + jvLH_3x34[34+idLSR]*Fy + jvLH_3x34[68+idLSR]*Fz; //LSR Torque
    torque_7x1[2] += jvLH_3x34[idLSY]*Fx + jvLH_3x34[34+idLSY]*Fy + jvLH_3x34[68+idLSY]*Fz; //LSY Torque
    torque_7x1[3] += jvLH_3x34[idLEB]*Fx + jvLH_3x34[34+idLEB]*Fy + jvLH_3x34[68+idLEB]*Fz; //LEB Torque
    torque_7x1[4] += jvLH_3x34[idLWY]*Fx + jvLH_3x34[34+idLWY]*Fy + jvLH_3x34[68+idLWY]*Fz; //LWY Torque
    torque_7x1[5] += jvLH_3x34[idLWP]*Fx + jvLH_3x34[34+idLWP]*Fy + jvLH_3x34[68+idLWP]*Fz; //LWP Torque
    torque_7x1[6] += jvLH_3x34[idLWY2]*Fx + jvLH_3x34[34+idLWY2]*Fy + jvLH_3x34[68+idLWY2]*Fz; //LF1 Torque



    cur_7x1[0] = torque_OLcurrent_map_LSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_LSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_LSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_LEB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_LWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_LWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_LWY2(torque_7x1[6]);

    for(int i=0; i<7; i++)
    {
        if(cur_7x1[i] > 500)              cur_7x1[i] = 500;
        else if(cur_7x1[i] < -500)        cur_7x1[i] = -500;
    }

//    RBJointOLCurrentCommand2ch(3, 17, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, cur_7x1[6], 0, 0);

//    static int counter = 0;
//    counter++;
//    if(counter == 200){
//        for(int i=idLSP;i<=idLWY2 ; i++){
//            //cout<<"torque : "<<i<<" "<<jvLH_3x34[i]*Fx + jvLH_3x34[34+i]*Fy + jvLH_3x34[68+i]*Fz<<endl;
//            cout<<"jacob "<<i<<" "<<jvLH_3x34[i]<<endl;
//        }
//        cout<<endl<<endl;
//        cout<<"gravity torque"<<torque_7x1[0]<<endl;
//        cout<<"cur :"<<cur_7x1[0]<<endl;

//        counter = 0;
//    }
}

void ForceTorqueControl_LeftArm(double Fx, double Fy, double Fz, double Mx, double My, double Mz){

    //Gravity compensation
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};
    //int i;
//    static int counter = 0;

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_LeftArm(Q_7x1, qBase_4x1, 0, torque_7x1);

//  mapping End effector force to joint torque
    //double ENC_34x1[34] ={0,};
    double jvLH_3x34[3*34];
    double jwLH_3x34[3*34];
    double jvLELB_3x34[3*34];


//    for(int i=0; i<34; i++){
//          ENC_34x1[i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition;
//    }
//    ENC_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition;
//    ENC_34x1[idLSR] = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition;
//    ENC_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition;
//    ENC_34x1[idLEB] = sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition;
//    ENC_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition;
//    ENC_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition;
//    ENC_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition;

//    ENC_34x1[idLSP] = Q_7x1[0];
//    ENC_34x1[idLSR] = Q_7x1[1];
//    ENC_34x1[idLSY] = Q_7x1[2];
//    ENC_34x1[idLEB] = Q_7x1[3];
//    ENC_34x1[idLWY] = Q_7x1[4];
//    ENC_34x1[idLWP] = Q_7x1[5];
//    ENC_34x1[idLWY2] = Q_7x1[6];

    double Q_34x1[34];
    memset(Q_34x1,0, 34*sizeof(double));
    Q_34x1[idQ0] = 1;
    memcpy(&Q_34x1[idLSP], Q_7x1, 7*sizeof(double));

    _ct_hubo2.Jacob_LeftHand(Q_34x1, jvLH_3x34, jwLH_3x34, jvLELB_3x34);

    torque_7x1[0] += jvLH_3x34[idLSP]*Fx + jvLH_3x34[34+idLSP]*Fy + jvLH_3x34[68+idLSP]*Fz + jwLH_3x34[idLSP]*Mx + jwLH_3x34[34+idLSP]*My + jwLH_3x34[64+idLSP]*Mz; //LSP Torque
    torque_7x1[1] += jvLH_3x34[idLSR]*Fx + jvLH_3x34[34+idLSR]*Fy + jvLH_3x34[68+idLSR]*Fz + jwLH_3x34[idLSR]*Mx + jwLH_3x34[34+idLSR]*My + jwLH_3x34[64+idLSR]*Mz; //LSR Torque
    torque_7x1[2] += jvLH_3x34[idLSY]*Fx + jvLH_3x34[34+idLSY]*Fy + jvLH_3x34[68+idLSY]*Fz + jwLH_3x34[idLSY]*Mx + jwLH_3x34[34+idLSY]*My + jwLH_3x34[64+idLSY]*Mz; //LSY Torque
    torque_7x1[3] += jvLH_3x34[idLEB]*Fx + jvLH_3x34[34+idLEB]*Fy + jvLH_3x34[68+idLEB]*Fz + jwLH_3x34[idLEB]*Mx + jwLH_3x34[34+idLEB]*My + jwLH_3x34[64+idLEB]*Mz; //LEB Torque
    torque_7x1[4] += jvLH_3x34[idLWY]*Fx + jvLH_3x34[34+idLWY]*Fy + jvLH_3x34[68+idLWY]*Fz + jwLH_3x34[idLWY]*Mx + jwLH_3x34[34+idLWY]*My + jwLH_3x34[64+idLWY]*Mz; //LWY Torque
    torque_7x1[5] += jvLH_3x34[idLWP]*Fx + jvLH_3x34[34+idLWP]*Fy + jvLH_3x34[68+idLWP]*Fz + jwLH_3x34[idLWP]*Mx + jwLH_3x34[34+idLWP]*My + jwLH_3x34[64+idLWP]*Mz; //LWP Torque
    torque_7x1[6] += jvLH_3x34[idLWY2]*Fx + jvLH_3x34[34+idLWY2]*Fy + jvLH_3x34[68+idLWY2]*Fz + jwLH_3x34[idLWY2]*Mx + jwLH_3x34[34+idLWY2]*My + jwLH_3x34[64+idLWY2]*Mz; //LF1 Torque



    cur_7x1[0] = torque_OLcurrent_map_LSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_LSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_LSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_LEB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_LWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_LWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_LWY2(torque_7x1[6]);

    for(int i=0; i<7; i++)
    {
        if(cur_7x1[i] > 500)              cur_7x1[i] = 500;
        else if(cur_7x1[i] < -500)        cur_7x1[i] = -500;
    }

//    RBJointOLCurrentCommand2ch(3, 17, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, cur_7x1[6], 0, 0);

}

void ForceControl_RightArm(double Fx, double Fy, double Fz){
    //Gravity compensation
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_RightArm(Q_7x1, qBase_4x1, 0, torque_7x1);

//  mapping End effector force to joint torque
    //double ENC_34x1[34] ={0,};
    double jvRH_3x34[3*34];
    double jwRH_3x34[3*34];
    double jvRELB_3x34[3*34];

//    ENC_34x1[idRSP] = Q_7x1[0];
//    ENC_34x1[idRSR] = Q_7x1[1];
//    ENC_34x1[idRSY] = Q_7x1[2];
//    ENC_34x1[idREB] = Q_7x1[3];
//    ENC_34x1[idRWY] = Q_7x1[4];
//    ENC_34x1[idRWP] = Q_7x1[5];
//    ENC_34x1[idRWY2] = Q_7x1[6];

    double Q_34x1[34];
    memset(Q_34x1,0, 34*sizeof(double));
    Q_34x1[idQ0] = 1;
    memcpy(&Q_34x1[idRSP], Q_7x1, 7*sizeof(double));

    _ct_hubo2.Jacob_RightHand(Q_34x1, jvRH_3x34, jwRH_3x34, jvRELB_3x34);

    torque_7x1[0] += jvRH_3x34[idRSP]*Fx + jvRH_3x34[34+idRSP]*Fy + jvRH_3x34[68+idRSP]*Fz; //RSP Torque
    torque_7x1[1] += jvRH_3x34[idRSR]*Fx + jvRH_3x34[34+idRSR]*Fy + jvRH_3x34[68+idRSR]*Fz; //RSR Torque
    torque_7x1[2] += jvRH_3x34[idRSY]*Fx + jvRH_3x34[34+idRSY]*Fy + jvRH_3x34[68+idRSY]*Fz; //RSY Torque
    torque_7x1[3] += jvRH_3x34[idREB]*Fx + jvRH_3x34[34+idREB]*Fy + jvRH_3x34[68+idREB]*Fz; //REB Torque
    torque_7x1[4] += jvRH_3x34[idRWY]*Fx + jvRH_3x34[34+idRWY]*Fy + jvRH_3x34[68+idRWY]*Fz; //RWY Torque
    torque_7x1[5] += jvRH_3x34[idRWP]*Fx + jvRH_3x34[34+idRWP]*Fy + jvRH_3x34[68+idRWP]*Fz; //RWP Torque
    torque_7x1[6] += jvRH_3x34[idRWY2]*Fx + jvRH_3x34[34+idRWY2]*Fy + jvRH_3x34[68+idRWY2]*Fz; //RF1 Torque



    cur_7x1[0] = torque_OLcurrent_map_RSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_RSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_RSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_REB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_RWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_RWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_RWY2(torque_7x1[6]);

    for(int i=0; i<7; i++)
    {
        if(cur_7x1[i] > 300)              cur_7x1[i] = 300;
        else if(cur_7x1[i] < -300)        cur_7x1[i] = -300;
    }

//    RBJointOLCurrentCommand2ch(2, 13, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 14, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 15, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(2, 16, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(2, 36, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, cur_7x1[6], 0, 0);
}

void ForceTorqueControl_RightArm(double Fx, double Fy, double Fz, double Mx, double My, double Mz){
    //Gravity compensation
    double Q_7x1[7] = {0,};
    double qBase_4x1[4] = {1, 0, 0, 0};
    double torque_7x1[7] = {0,};
    int cur_7x1[7] = {0,};

    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    _ct_hubo2.get_gravity_RightArm(Q_7x1, qBase_4x1, 0, torque_7x1);

//  mapping End effector force to joint torque
    //double ENC_34x1[34] ={0,};
    double jvRH_3x34[3*34];
    double jwRH_3x34[3*34];
    double jvRELB_3x34[3*34];

//    ENC_34x1[idRSP] = Q_7x1[0];
//    ENC_34x1[idRSR] = Q_7x1[1];
//    ENC_34x1[idRSY] = Q_7x1[2];
//    ENC_34x1[idREB] = Q_7x1[3];
//    ENC_34x1[idRWY] = Q_7x1[4];
//    ENC_34x1[idRWP] = Q_7x1[5];
//    ENC_34x1[idRWY2] = Q_7x1[6];

    double Q_34x1[34];
    memset(Q_34x1,0, 34*sizeof(double));
    Q_34x1[idQ0] = 1;
    memcpy(&Q_34x1[idRSP], Q_7x1, 7*sizeof(double));

    _ct_hubo2.Jacob_RightHand(Q_34x1, jvRH_3x34, jwRH_3x34, jvRELB_3x34);

    torque_7x1[0] += jvRH_3x34[idRSP]*Fx + jvRH_3x34[34+idRSP]*Fy + jvRH_3x34[68+idRSP]*Fz + jwRH_3x34[idRSP]*Mx + jwRH_3x34[34+idRSP]*My + jwRH_3x34[64+idRSP]*Mz; //RSP Torque
    torque_7x1[1] += jvRH_3x34[idRSR]*Fx + jvRH_3x34[34+idRSR]*Fy + jvRH_3x34[68+idRSR]*Fz + jwRH_3x34[idRSR]*Mx + jwRH_3x34[34+idRSR]*My + jwRH_3x34[64+idRSR]*Mz; //RSR Torque
    torque_7x1[2] += jvRH_3x34[idRSY]*Fx + jvRH_3x34[34+idRSY]*Fy + jvRH_3x34[68+idRSY]*Fz + jwRH_3x34[idRSY]*Mx + jwRH_3x34[34+idRSY]*My + jwRH_3x34[64+idRSY]*Mz; //RSY Torque
    torque_7x1[3] += jvRH_3x34[idREB]*Fx + jvRH_3x34[34+idREB]*Fy + jvRH_3x34[68+idREB]*Fz + jwRH_3x34[idREB]*Mx + jwRH_3x34[34+idREB]*My + jwRH_3x34[64+idREB]*Mz; //REB Torque
    torque_7x1[4] += jvRH_3x34[idRWY]*Fx + jvRH_3x34[34+idRWY]*Fy + jvRH_3x34[68+idRWY]*Fz + jwRH_3x34[idRWY]*Mx + jwRH_3x34[34+idRWY]*My + jwRH_3x34[64+idRWY]*Mz; //RWY Torque
    torque_7x1[5] += jvRH_3x34[idRWP]*Fx + jvRH_3x34[34+idRWP]*Fy + jvRH_3x34[68+idRWP]*Fz + jwRH_3x34[idRSP]*Mx + jwRH_3x34[34+idRSP]*My + jwRH_3x34[64+idRSP]*Mz; //RWP Torque
    torque_7x1[6] += jvRH_3x34[idRWY2]*Fx + jvRH_3x34[34+idRWY2]*Fy + jvRH_3x34[68+idRWY2]*Fz + jwRH_3x34[idRWY2]*Mx + jwRH_3x34[34+idRWY2]*My + jwRH_3x34[64+idRWY2]*Mz; //RF1 Torque



    cur_7x1[0] = torque_OLcurrent_map_RSP(torque_7x1[0]);
    cur_7x1[1] = torque_OLcurrent_map_RSR(torque_7x1[1]);
    cur_7x1[2] = torque_OLcurrent_map_RSY(torque_7x1[2]);
    cur_7x1[3] = torque_OLcurrent_map_REB(torque_7x1[3]);
    cur_7x1[4] = torque_OLcurrent_map_RWY(torque_7x1[4]);
    cur_7x1[5] = torque_OLcurrent_map_RWP(torque_7x1[5]);
    cur_7x1[6] = torque_OLcurrent_map_RWY2(torque_7x1[6]);

    for(int i=0; i<7; i++)
    {
        if(cur_7x1[i] > 300)              cur_7x1[i] = 300;
        else if(cur_7x1[i] < -300)        cur_7x1[i] = -300;
    }

//    RBJointOLCurrentCommand2ch(2, 13, cur_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 14, cur_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 15, cur_7x1[2], cur_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(2, 16, cur_7x1[4], cur_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(2, 36, cur_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, cur_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, cur_7x1[1], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, cur_7x1[2], 4, cur_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, cur_7x1[4], 4, cur_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, cur_7x1[6], 0, 0);
}

int ZForceXYPositionControl_RightArm(double x_ref, double y_ref, double Fz_ref, double RElb_ref){
    double XYposition_error[2];
    static double XYposition_error_before[2] ={0.};
    double pRH_3x1[3];
    double qRH_4x1[4];
    double RElb;
    double Fx, Fy;
    const double Pgain = 230;
    const double Dgain = 26;
    double XYposition_error_dot[2];
    double cur_RElb;
    double RElb_error;
    double RElb_error_dot;
    double RSR_Torque;
    double Pgain_elb = 1.3;
    double Dgain_elb = 0.1;
    double RSR_current;


//    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
//    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
//    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
//    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
//    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
//    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
//    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

//    double Q_34x1[34];
//    memset(Q_34x1,0, 34*sizeof(double));
//    Q_34x1[idQ0] = 1;
//    memcpy(&Q_34x1[idRSP], Q_7x1, 7*sizeof(double));



    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

//    for(int i=RHY; i<=LAR; i++){
//        Qin_34x1[idRHY+i] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]*D2R;
//    }
//    Qin_34x1[idWST] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[WST].id][MC_ID_CH_Pairs[WST].ch]*D2R;

//    Qin_34x1[idRSP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch]*D2R;
//    Qin_34x1[idRSR] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]+OFFSET_RSR)*D2R;
//    Qin_34x1[idRSY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSY].id][MC_ID_CH_Pairs[RSY].ch]*D2R;
//    Qin_34x1[idREB] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[REB].id][MC_ID_CH_Pairs[REB].ch]+OFFSET_ELB)*D2R;
//    Qin_34x1[idRWY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWY].id][MC_ID_CH_Pairs[RWY].ch]*D2R;
//    Qin_34x1[idRWP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWP].id][MC_ID_CH_Pairs[RWP].ch]*D2R;
//    Qin_34x1[idRWY2] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWY2].id][MC_ID_CH_Pairs[RWY2].ch]*D2R;

//    Qin_34x1[idLSP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSP].id][MC_ID_CH_Pairs[LSP].ch]*D2R;
//    Qin_34x1[idLSR] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSR].id][MC_ID_CH_Pairs[LSR].ch]+OFFSET_LSR)*D2R;
//    Qin_34x1[idLSY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSY].id][MC_ID_CH_Pairs[LSY].ch]*D2R;
//    Qin_34x1[idLEB] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[LEB].id][MC_ID_CH_Pairs[LEB].ch]+OFFSET_ELB)*D2R;
//    Qin_34x1[idLWY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWY].id][MC_ID_CH_Pairs[LWY].ch]*D2R;
//    Qin_34x1[idLWP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWP].id][MC_ID_CH_Pairs[LWP].ch]*D2R;
//    Qin_34x1[idLWY2] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWY2].id][MC_ID_CH_Pairs[LWY2].ch]*D2R;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.FK_RightHand_Global(Qin_34x1, pRH_3x1, qRH_4x1, RElb);

    XYposition_error[0] = x_ref - pRH_3x1[0];
    XYposition_error[1] = y_ref - pRH_3x1[1];

    XYposition_error_dot[0] = (XYposition_error[0] - XYposition_error_before[0])/0.005;
    XYposition_error_dot[1] = (XYposition_error[1] - XYposition_error_before[1])/0.005;

    XYposition_error_before[0] = XYposition_error[0];
    XYposition_error_before[1] = XYposition_error[1];

    Fx = Pgain*(XYposition_error[0]) + Dgain*XYposition_error_dot[0];
    Fy = Pgain*(XYposition_error[1]) + Dgain*XYposition_error_dot[1];

    ForceControl_RightArm(Fx, Fy, Fz_ref);

    // Elb angle Control
    cur_RElb = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR);

    RElb_error = RElb_ref - cur_RElb;

    static double RElb_error_before = RElb_error;
    RElb_error_dot = (RElb_error - RElb_error_before)/0.005;

    RElb_error_before = RElb_error;

    RSR_Torque = Pgain_elb*RElb_error + Dgain_elb*RElb_error_dot;

    RSR_current = torque_OLcurrent_map_RSP(RSR_Torque);

//    RBJointOLCurrentCommand2ch(2, 14, RSR_current, 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, RSR_current, 0, 0);



//    static unsigned int counter=0;
//    if(counter == 200){
//        cout<<"cartesian x :"<<pRH_3x1[0]<<"  cartesian y :"<<pRH_3x1[1]<<endl;
//        cout<<"R Force x :"<<Fx<<"  R Force y :"<<Fy<<endl;
//        counter = 0;
//    }
//    counter++;

    return 0;
}

int XYZPositionControl_RightArm(double x_ref, double y_ref, double z_ref, double RElb_ref){
    double XYZposition_error[3];
    static double XYZposition_error_before[3] ={0.};
    double pRH_3x1[3];
    double qRH_4x1[4];
    double RElb;
    double Fx, Fy, Fz;
    const double Pgain = 330;
    const double Dgain = 25;
    double XYZposition_error_dot[3];
    double cur_RElb;
    double RElb_error;
    double RElb_error_dot;
    double RSR_Torque;
    double Pgain_elb = 1;
    double Dgain_elb = 0.1;
    double RSR_current;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.FK_RightHand_Global(Qin_34x1, pRH_3x1, qRH_4x1, RElb);

    XYZposition_error[0] = x_ref - pRH_3x1[0];
    XYZposition_error[1] = y_ref - pRH_3x1[1];
    XYZposition_error[2] = z_ref - pRH_3x1[2];

    XYZposition_error_dot[0] = (XYZposition_error[0] - XYZposition_error_before[0])/0.005;
    XYZposition_error_dot[1] = (XYZposition_error[1] - XYZposition_error_before[1])/0.005;
    XYZposition_error_dot[2] = (XYZposition_error[2] - XYZposition_error_before[2])/0.005;

    XYZposition_error_before[0] = XYZposition_error[0];
    XYZposition_error_before[1] = XYZposition_error[1];
    XYZposition_error_before[2] = XYZposition_error[2];

    Fx = Pgain*(XYZposition_error[0]) + Dgain*XYZposition_error_dot[0];
    Fy = Pgain*(XYZposition_error[1]) + Dgain*XYZposition_error_dot[1];
    Fz = Pgain*(XYZposition_error[2]) + Dgain*XYZposition_error_dot[2];

    ForceControl_RightArm(Fx, Fy, Fz);

    // Elb angle Control
    cur_RElb = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR);

    RElb_error = RElb_ref - cur_RElb;

    static double RElb_error_before = RElb_error;
    RElb_error_dot = (RElb_error - RElb_error_before)/0.005;

    RElb_error_before = RElb_error;

    RSR_Torque = Pgain_elb*RElb_error + Dgain_elb*RElb_error_dot;

    RSR_current = torque_OLcurrent_map_RSP(RSR_Torque);

//    RBJointOLCurrentCommand2ch(2, 14, RSR_current, 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, RSR_current, 0, 0);

    return 0;

}

int PositionControlByForce_RightArm(double x_ref, double y_ref, double z_ref, double *qRH_4x1_ref, double RElb_ref){
    double XYZposition_error[3];
    static double XYZposition_error_before[3] ={0,};
    //double rpyRH[3];
    double rpy_angle_error[3];
    double rpy_angle_error_before[3] = {0,};
    double rpy_angle_error_dot[3];
    double pRH_3x1[3];
    double qRH_4x1[4];
    double RElb;
    double Fx, Fy, Fz, Mx, My, Mz;
    const double Pgain = 330;
    const double Dgain = 25;
    const double angPgain = 0.5;
    const double angDgain = 0.1;
    double XYZposition_error_dot[3];
    double cur_RElb;
    double RElb_error;
    double RElb_error_dot;
    double RSR_Torque;
    double Pgain_elb = 1;
    double Dgain_elb = 0.1;
    double RSR_current;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.FK_RightHand_Global(Qin_34x1, pRH_3x1, qRH_4x1, RElb);

    XYZposition_error[0] = x_ref - pRH_3x1[0];
    XYZposition_error[1] = y_ref - pRH_3x1[1];
    XYZposition_error[2] = z_ref - pRH_3x1[2];

    XYZposition_error_dot[0] = (XYZposition_error[0] - XYZposition_error_before[0])/0.005;
    XYZposition_error_dot[1] = (XYZposition_error[1] - XYZposition_error_before[1])/0.005;
    XYZposition_error_dot[2] = (XYZposition_error[2] - XYZposition_error_before[2])/0.005;

    XYZposition_error_before[0] = XYZposition_error[0];
    XYZposition_error_before[1] = XYZposition_error[1];
    XYZposition_error_before[2] = XYZposition_error[2];

    Fx = Pgain*(XYZposition_error[0]) + Dgain*XYZposition_error_dot[0];
    Fy = Pgain*(XYZposition_error[1]) + Dgain*XYZposition_error_dot[1];
    Fz = Pgain*(XYZposition_error[2]) + Dgain*XYZposition_error_dot[2];

    quat cur_quatRH = quat(qRH_4x1[0], qRH_4x1[1], qRH_4x1[2], qRH_4x1[3]);
    quat ref_quatRH = quat(qRH_4x1_ref[0], qRH_4x1_ref[1], qRH_4x1_ref[2], qRH_4x1_ref[3]);
//    rpy cur_rpyRH = rpy(cur_quatRH);
//    rpy ref_rpyRH = rpy(ref_quatRH);
    double quat_error[3];
    quat_error[0] = cur_quatRH.w*ref_quatRH.x - ref_quatRH.w*cur_quatRH.x - (-ref_quatRH.z*cur_quatRH.y + ref_quatRH.y*cur_quatRH.z);
    quat_error[1] = cur_quatRH.w*ref_quatRH.y - ref_quatRH.w*cur_quatRH.y - (ref_quatRH.z*cur_quatRH.x - ref_quatRH.x*cur_quatRH.z);
    quat_error[2] = cur_quatRH.w*ref_quatRH.z - ref_quatRH.w*cur_quatRH.z - (-ref_quatRH.y*cur_quatRH.x + ref_quatRH.x*cur_quatRH.y);

    rpy_angle_error[0] = quat_error[0];
    rpy_angle_error[1] = quat_error[1];
    rpy_angle_error[2] = quat_error[2];

    rpy_angle_error_dot[0] = (rpy_angle_error[0] - rpy_angle_error_before[0])/0.005;
    rpy_angle_error_dot[1] = (rpy_angle_error[1] - rpy_angle_error_before[1])/0.005;
    rpy_angle_error_dot[2] = (rpy_angle_error[2] - rpy_angle_error_before[2])/0.005;

    rpy_angle_error_before[0] = rpy_angle_error[0];
    rpy_angle_error_before[1] = rpy_angle_error[1];
    rpy_angle_error_before[2] = rpy_angle_error[2];

    Mx = angPgain*rpy_angle_error[0] + angDgain*rpy_angle_error_dot[0];
    My = angPgain*rpy_angle_error[1] + angDgain*rpy_angle_error_dot[1];
    Mz = angPgain*rpy_angle_error[2] + angDgain*rpy_angle_error_dot[2];

    ForceTorqueControl_RightArm(Fx, Fy, Fz, Mx, My, Mz);

    // Elb angle Control
    cur_RElb = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR);

    RElb_error = RElb_ref - cur_RElb;

    static double RElb_error_before = RElb_error;
    RElb_error_dot = (RElb_error - RElb_error_before)/0.005;

    RElb_error_before = RElb_error;

    RSR_Torque = Pgain_elb*RElb_error + Dgain_elb*RElb_error_dot;

    RSR_current = torque_OLcurrent_map_RSP(RSR_Torque);

//    RBJointOLCurrentCommand2ch(2, 14, RSR_current, 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, RSR_current, 0, 0);

    return 0;
}

int ZForceXYPositionControl_LeftArm(double x_ref, double y_ref, double Fz_ref, double LElb_ref){
    double XYposition_error[2];
    static double XYposition_error_before[2] ={0,};
    double pLH_3x1[3];
    double qLH_4x1[4];
    double LElb;
    double Fx, Fy;
    const double Pgain = 230;
    const double Dgain = 26;
    double XYposition_error_dot[2];
    double cur_LElb;
    double LElb_error;
    double LElb_error_dot;
    double LSR_Torque;
    double Pgain_elb = 1.3;
    double Dgain_elb = 0.1;
    double LSR_current;


//    Q_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
//    Q_7x1[1] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
//    Q_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
//    Q_7x1[3] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
//    Q_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
//    Q_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
//    Q_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

//    double Q_34x1[34];
//    memset(Q_34x1,0, 34*sizeof(double));
//    Q_34x1[idQ0] = 1;
//    memcpy(&Q_34x1[idRSP], Q_7x1, 7*sizeof(double));



    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

//    for(int i=RHY; i<=LAR; i++){
//        Qin_34x1[idRHY+i] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]*D2R;
//    }
//    Qin_34x1[idWST] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[WST].id][MC_ID_CH_Pairs[WST].ch]*D2R;

//    Qin_34x1[idRSP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch]*D2R;
//    Qin_34x1[idRSR] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]+OFFSET_RSR)*D2R;
//    Qin_34x1[idRSY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSY].id][MC_ID_CH_Pairs[RSY].ch]*D2R;
//    Qin_34x1[idREB] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[REB].id][MC_ID_CH_Pairs[REB].ch]+OFFSET_ELB)*D2R;
//    Qin_34x1[idRWY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWY].id][MC_ID_CH_Pairs[RWY].ch]*D2R;
//    Qin_34x1[idRWP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWP].id][MC_ID_CH_Pairs[RWP].ch]*D2R;
//    Qin_34x1[idRWY2] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWY2].id][MC_ID_CH_Pairs[RWY2].ch]*D2R;

//    Qin_34x1[idLSP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSP].id][MC_ID_CH_Pairs[LSP].ch]*D2R;
//    Qin_34x1[idLSR] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSR].id][MC_ID_CH_Pairs[LSR].ch]+OFFSET_LSR)*D2R;
//    Qin_34x1[idLSY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSY].id][MC_ID_CH_Pairs[LSY].ch]*D2R;
//    Qin_34x1[idLEB] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[LEB].id][MC_ID_CH_Pairs[LEB].ch]+OFFSET_ELB)*D2R;
//    Qin_34x1[idLWY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWY].id][MC_ID_CH_Pairs[LWY].ch]*D2R;
//    Qin_34x1[idLWP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWP].id][MC_ID_CH_Pairs[LWP].ch]*D2R;
//    Qin_34x1[idLWY2] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWY2].id][MC_ID_CH_Pairs[LWY2].ch]*D2R;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.FK_LeftHand_Global(Qin_34x1, pLH_3x1, qLH_4x1, LElb);

    XYposition_error[0] = x_ref - pLH_3x1[0];
    XYposition_error[1] = y_ref - pLH_3x1[1];

    XYposition_error_dot[0] = (XYposition_error[0] - XYposition_error_before[0])/0.005;
    XYposition_error_dot[1] = (XYposition_error[1] - XYposition_error_before[1])/0.005;

    XYposition_error_before[0] = XYposition_error[0];
    XYposition_error_before[1] = XYposition_error[1];

    Fx = Pgain*(XYposition_error[0]) + Dgain*XYposition_error_dot[0];
    Fy = Pgain*(XYposition_error[1]) + Dgain*XYposition_error_dot[1];

    ForceControl_LeftArm(Fx, Fy, Fz_ref);

    // Elb angle control
    cur_LElb = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR);

    LElb_error = LElb_ref - cur_LElb;

    static double LElb_error_before = LElb_error;
    LElb_error_dot = (LElb_error - LElb_error_before)/0.005;

    LElb_error_before = LElb_error;

    LSR_Torque = Pgain_elb*LElb_error + Dgain_elb*LElb_error_dot;

    LSR_current = torque_OLcurrent_map_LSP(LSR_Torque);

//    RBJointOLCurrentCommand2ch(3, 18, LSR_current, 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, LSR_current, 0, 0);


//    static unsigned int counter=0;
//    if(counter == 200){
//        cout<<"cartesian x :"<<pLH_3x1[0]<<"  cartesian y :"<<pLH_3x1[1]<<endl;
//        cout<<"L Force x :"<<Fx<<"  L Force y :"<<Fy<<endl;
//        counter = 0;
//    }
//    counter++;

    return 0;
}

int XYZPositionControl_LeftArm(double x_ref, double y_ref, double z_ref, double LElb_ref){
    double XYZposition_error[3];
    static double XYZposition_error_before[3] ={0,};
    double pLH_3x1[3];
    double qLH_4x1[4];
    double LElb;
    double Fx, Fy, Fz;
    const double Pgain = 330;
    const double Dgain = 25;
    double XYZposition_error_dot[3];
    double cur_LElb;
    double LElb_error;
    double LElb_error_dot;
    double LSR_Torque;
    double Pgain_elb = 1;
    double Dgain_elb = 0.1;
    double LSR_current;


    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

//    for(int i=RHY; i<=LAR; i++){
//        Qin_34x1[idRHY+i] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]*D2R;
//    }
//    Qin_34x1[idWST] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[WST].id][MC_ID_CH_Pairs[WST].ch]*D2R;

//    Qin_34x1[idRSP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch]*D2R;
//    Qin_34x1[idRSR] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]+OFFSET_RSR)*D2R;
//    Qin_34x1[idRSY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSY].id][MC_ID_CH_Pairs[RSY].ch]*D2R;
//    Qin_34x1[idREB] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[REB].id][MC_ID_CH_Pairs[REB].ch]+OFFSET_ELB)*D2R;
//    Qin_34x1[idRWY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWY].id][MC_ID_CH_Pairs[RWY].ch]*D2R;
//    Qin_34x1[idRWP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWP].id][MC_ID_CH_Pairs[RWP].ch]*D2R;
//    Qin_34x1[idRWY2] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[RWY2].id][MC_ID_CH_Pairs[RWY2].ch]*D2R;

//    Qin_34x1[idLSP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSP].id][MC_ID_CH_Pairs[LSP].ch]*D2R;
//    Qin_34x1[idLSR] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSR].id][MC_ID_CH_Pairs[LSR].ch]+OFFSET_LSR)*D2R;
//    Qin_34x1[idLSY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LSY].id][MC_ID_CH_Pairs[LSY].ch]*D2R;
//    Qin_34x1[idLEB] = (sharedData->CurrentJointReference[MC_ID_CH_Pairs[LEB].id][MC_ID_CH_Pairs[LEB].ch]+OFFSET_ELB)*D2R;
//    Qin_34x1[idLWY] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWY].id][MC_ID_CH_Pairs[LWY].ch]*D2R;
//    Qin_34x1[idLWP] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWP].id][MC_ID_CH_Pairs[LWP].ch]*D2R;
//    Qin_34x1[idLWY2] = sharedData->CurrentJointReference[MC_ID_CH_Pairs[LWY2].id][MC_ID_CH_Pairs[LWY2].ch]*D2R;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.FK_LeftHand_Global(Qin_34x1, pLH_3x1, qLH_4x1, LElb);

    XYZposition_error[0] = x_ref - pLH_3x1[0];
    XYZposition_error[1] = y_ref - pLH_3x1[1];
    XYZposition_error[2] = z_ref - pLH_3x1[2];

    XYZposition_error_dot[0] = (XYZposition_error[0] - XYZposition_error_before[0])/0.005;
    XYZposition_error_dot[1] = (XYZposition_error[1] - XYZposition_error_before[1])/0.005;
    XYZposition_error_dot[2] = (XYZposition_error[2] - XYZposition_error_before[2])/0.005;

    XYZposition_error_before[0] = XYZposition_error[0];
    XYZposition_error_before[1] = XYZposition_error[1];
    XYZposition_error_before[2] = XYZposition_error[2];

    Fx = Pgain*(XYZposition_error[0]) + Dgain*XYZposition_error_dot[0];
    Fy = Pgain*(XYZposition_error[1]) + Dgain*XYZposition_error_dot[1];
    Fz = Pgain*(XYZposition_error[2]) + Dgain*XYZposition_error_dot[2];

    ForceControl_LeftArm(Fx, Fy, Fz);

    // Elb angle control
    cur_LElb = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR);

    LElb_error = LElb_ref - cur_LElb;

    static double LElb_error_before = LElb_error;
    LElb_error_dot = (LElb_error - LElb_error_before)/0.005;

    LElb_error_before = LElb_error;

    LSR_Torque = Pgain_elb*LElb_error + Dgain_elb*LElb_error_dot;

    LSR_current = torque_OLcurrent_map_LSP(LSR_Torque);

//    RBJointOLCurrentCommand2ch(3, 18, LSR_current, 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, LSR_current, 0, 0);



//    static unsigned int counter=0;
//    if(counter == 200){
//        cout<<"cartesian x :"<<pLH_3x1[0]<<"  cartesian y :"<<pLH_3x1[1]<<endl;
//        cout<<"L Force x :"<<Fx<<"  L Force y :"<<Fy<<endl;
//        counter = 0;
//    }
//    counter++;

    return 0;
}

int PositionControlByForce_LeftArm(double x_ref, double y_ref, double z_ref, double qLH_4x1_ref[], double LElb_ref){
    double XYZposition_error[3];
    static double XYZposition_error_before[3] ={0,};
    double rpy_angle_error[3];
    double rpy_angle_error_before[3] = {0,};
    double rpy_angle_error_dot[3];
    double pLH_3x1[3];
    double qLH_4x1[4];
    double LElb;
    double Fx, Fy, Fz, Mx, My, Mz;
    const double Pgain = 330;
    const double Dgain = 25;
    const double angPgain = 0.5;
    const double angDgain = 0.1;
    double XYZposition_error_dot[3];
    double cur_LElb;
    double LElb_error;
    double LElb_error_dot;
    double LSR_Torque;
    double Pgain_elb = 1;
    double Dgain_elb = 0.1;
    double LSR_current;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;


    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    _ct_hubo2.FK_LeftHand_Global(Qin_34x1, pLH_3x1, qLH_4x1, LElb);

    XYZposition_error[0] = x_ref - pLH_3x1[0];
    XYZposition_error[1] = y_ref - pLH_3x1[1];
    XYZposition_error[2] = z_ref - pLH_3x1[2];

    XYZposition_error_dot[0] = (XYZposition_error[0] - XYZposition_error_before[0])/0.005;
    XYZposition_error_dot[1] = (XYZposition_error[1] - XYZposition_error_before[1])/0.005;
    XYZposition_error_dot[2] = (XYZposition_error[2] - XYZposition_error_before[2])/0.005;

    XYZposition_error_before[0] = XYZposition_error[0];
    XYZposition_error_before[1] = XYZposition_error[1];
    XYZposition_error_before[2] = XYZposition_error[2];

    Fx = Pgain*(XYZposition_error[0]) + Dgain*XYZposition_error_dot[0];
    Fy = Pgain*(XYZposition_error[1]) + Dgain*XYZposition_error_dot[1];
    Fz = Pgain*(XYZposition_error[2]) + Dgain*XYZposition_error_dot[2];

    quat cur_quatLH = quat(qLH_4x1[0], qLH_4x1[1], qLH_4x1[2], qLH_4x1[3]);
    quat ref_quatLH = quat(qLH_4x1_ref[0], qLH_4x1_ref[1], qLH_4x1_ref[2], qLH_4x1_ref[3]);
    double quat_error[3];

    quat_error[0] = cur_quatLH.w*ref_quatLH.x - ref_quatLH.w*cur_quatLH.x - (-ref_quatLH.z*cur_quatLH.y + ref_quatLH.y*cur_quatLH.z);
    quat_error[1] = cur_quatLH.w*ref_quatLH.y - ref_quatLH.w*cur_quatLH.y - (ref_quatLH.z*cur_quatLH.x - ref_quatLH.x*cur_quatLH.z);
    quat_error[2] = cur_quatLH.w*ref_quatLH.z - ref_quatLH.w*cur_quatLH.z - (-ref_quatLH.y*cur_quatLH.x + ref_quatLH.x*cur_quatLH.y);

//    rpy cur_rpyLH = rpy(cur_quatLH);
//    rpy ref_rpyLH = rpy(ref_quatLH);

//    rpy_angle_error[0] = ref_rpyLH.r - cur_rpyLH.r;
//    rpy_angle_error[1] = ref_rpyLH.p - cur_rpyLH.p;
//    rpy_angle_error[2] = ref_rpyLH.y - cur_rpyLH.y;
    rpy_angle_error[0] = quat_error[0];
    rpy_angle_error[1] = quat_error[1];
    rpy_angle_error[2] = quat_error[2];

    rpy_angle_error_dot[0] = (rpy_angle_error[0] - rpy_angle_error_before[0])/0.005;
    rpy_angle_error_dot[1] = (rpy_angle_error[1] - rpy_angle_error_before[1])/0.005;
    rpy_angle_error_dot[2] = (rpy_angle_error[2] - rpy_angle_error_before[2])/0.005;

    rpy_angle_error_before[0] = rpy_angle_error[0];
    rpy_angle_error_before[1] = rpy_angle_error[1];
    rpy_angle_error_before[2] = rpy_angle_error[2];

    Mx = angPgain*rpy_angle_error[0] + angDgain*rpy_angle_error_dot[0];
    My = angPgain*rpy_angle_error[1] + angDgain*rpy_angle_error_dot[1];
    Mz = angPgain*rpy_angle_error[2] + angDgain*rpy_angle_error_dot[2];


    ForceTorqueControl_LeftArm(Fx, Fy, Fz, Mx, My, Mz);

    // Elb angle control
    cur_LElb = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR);

    LElb_error = LElb_ref - cur_LElb;

    static double LElb_error_before = LElb_error;
    LElb_error_dot = (LElb_error - LElb_error_before)/0.005;

    LElb_error_before = LElb_error;

    LSR_Torque = Pgain_elb*LElb_error + Dgain_elb*LElb_error_dot;

    LSR_current = torque_OLcurrent_map_LSP(LSR_Torque);

//    RBJointOLCurrentCommand2ch(3, 18, LSR_current, 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, LSR_current, 0, 0);


//    static unsigned int counter=0;
//    if(counter == 200){
//        cout<<"cartesian x :"<<pLH_3x1[0]<<"  cartesian y :"<<pLH_3x1[1]<<endl;
//        cout<<"L Force x :"<<Fx<<"  L Force y :"<<Fy<<endl;
//        counter = 0;
//    }
//    counter++;

    return 0;
}

int FK_Pos_RightArm(double _pRH_3x1[]){
    double pRH_3x1[3];
    double qRH_4x1[4];
    double RElb;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    //cout<<sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]<<endl;

    _ct_hubo2.FK_RightHand_Global(Qin_34x1, pRH_3x1, qRH_4x1, RElb);

     //cout<<"RH_x: "<<pRH_3x1[0]<<"   RH_y :"<<pRH_3x1[1]<<endl;

    _pRH_3x1[0] = pRH_3x1[0];
    _pRH_3x1[1] = pRH_3x1[1];
    _pRH_3x1[2] = pRH_3x1[2];

    return 0;
}

int FK_Pos_Ori_Elb_RightArm(double _pRH_3x1[], double _qRH_4x1[], double &_RElb){
    double pRH_3x1[3];
    double qRH_4x1[4];
    double RElb;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    //cout<<sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]<<endl;

    _ct_hubo2.FK_RightHand_Global(Qin_34x1, pRH_3x1, qRH_4x1, RElb);

     //cout<<"RH_x: "<<pRH_3x1[0]<<"   RH_y :"<<pRH_3x1[1]<<endl;

    _pRH_3x1[0] = pRH_3x1[0];
    _pRH_3x1[1] = pRH_3x1[1];
    _pRH_3x1[2] = pRH_3x1[2];

    for(int i=0; i<4 ;i++){
        _qRH_4x1[i] = qRH_4x1[i];
    }

    _RElb = RElb;
    return 0;
}


int FK_Pos_LeftArm(double _pLH_3x1[]){
    double pLH_3x1[3];
    double qLH_4x1[4];
    double LElb;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    //cout<<sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]<<endl;

    _ct_hubo2.FK_LeftHand_Global(Qin_34x1, pLH_3x1, qLH_4x1, LElb);

     //cout<<"RH_x: "<<pRH_3x1[0]<<"   RH_y :"<<pRH_3x1[1]<<endl;

    _pLH_3x1[0] = pLH_3x1[0];
    _pLH_3x1[1] = pLH_3x1[1];
    _pLH_3x1[2] = pLH_3x1[2];
    return 0;
}

int FK_Pos_Ori_Elb_LeftArm(double _pLH_3x1[], double _qLH_4x1[], double &_LElb){
    double pLH_3x1[3];
    double qLH_4x1[4];
    double LElb;

    double Qin_34x1[34];

    memset(Qin_34x1,0, 34*sizeof(double));
    Qin_34x1[idQ0] = 1;

    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition*D2R;
    }
    Qin_34x1[idWST] = sharedSEN->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition*D2R;

    Qin_34x1[idRSP] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition*D2R;
    Qin_34x1[idRSR] = (sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition*D2R;
    Qin_34x1[idREB] = (sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition*D2R;
    Qin_34x1[idRWP] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition*D2R;
    Qin_34x1[idRWY2] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition*D2R;

    Qin_34x1[idLSP] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition*D2R;
    Qin_34x1[idLSR] = (sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition*D2R;
    Qin_34x1[idLEB] = (sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition*D2R;
    Qin_34x1[idLWP] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition*D2R;
    Qin_34x1[idLWY2] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition*D2R;

    //cout<<sharedData->CurrentJointReference[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch]<<endl;

    _ct_hubo2.FK_LeftHand_Global(Qin_34x1, pLH_3x1, qLH_4x1, LElb);

     //cout<<"RH_x: "<<pRH_3x1[0]<<"   RH_y :"<<pRH_3x1[1]<<endl;

    _pLH_3x1[0] = pLH_3x1[0];
    _pLH_3x1[1] = pLH_3x1[1];
    _pLH_3x1[2] = pLH_3x1[2];

    for(int i=0; i<4; i++){
        _qLH_4x1[i] = qLH_4x1[i];
    }
    _LElb = LElb;
    return 0;
}

void JointForceControl_RightArm(double RSP_ang, double RSR_ang, double RSY_ang, double REB_ang, double RWY_ang, double RWP_ang, double RF1_ang){
    double ref_pos_7x1[7] = {RSP_ang, RSR_ang, RSY_ang, REB_ang, RWY_ang, RWP_ang, RF1_ang};
    double cur_pos_7x1[7];
    double error_7x1[7];
    double error_dot_7x1[7];
    static double error_7x1_before[7];
    double torque_7x1[7];
    double current_7x1[7];
    const double Pgain = 1;
    const double Dgain = 0.1;

    cur_pos_7x1[0] = sharedSEN->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition;
    cur_pos_7x1[1] = sharedSEN->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition + OFFSET_RSR;
    cur_pos_7x1[2] = sharedSEN->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition;
    cur_pos_7x1[3] = sharedSEN->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition+ OFFSET_ELB;
    cur_pos_7x1[4] = sharedSEN->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition;
    cur_pos_7x1[5] = sharedSEN->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition;
    cur_pos_7x1[6] = sharedSEN->ENCODER[MC_GetID(RWY2)][MC_GetCH(RWY2)].CurrentPosition;

    // Angle control
    for(int i=0; i<7; i++){
        error_7x1[i] = ref_pos_7x1[i] - cur_pos_7x1[i];
        error_dot_7x1[i] = (error_7x1[i] - error_7x1_before[i])/0.005;
        error_7x1_before[i] = error_7x1[i];
        if(i == 0 || i == 1){
            torque_7x1[i] = 4*Pgain*error_7x1[i] + 4*Dgain*error_dot_7x1[i];
        }
        else if(i == 2){
            torque_7x1[i] = 2*Pgain*error_7x1[i] + 2*Dgain*error_dot_7x1[i];
        }
        else if(i == 3){
            torque_7x1[i] = 4*Pgain*error_7x1[i] + 4*Dgain*error_dot_7x1[i];
        }
        else{
            torque_7x1[i] = Pgain*error_7x1[i] + Dgain*error_dot_7x1[i];
        }
    }

    current_7x1[0] = torque_OLcurrent_map_RSP(torque_7x1[0]);
    current_7x1[1] = torque_OLcurrent_map_RSR(torque_7x1[1]);
    current_7x1[2] = torque_OLcurrent_map_RSY(torque_7x1[2]);
    current_7x1[3] = torque_OLcurrent_map_REB(torque_7x1[3]);
    current_7x1[4] = torque_OLcurrent_map_RWY(torque_7x1[4]);
    current_7x1[5] = torque_OLcurrent_map_RWP(torque_7x1[5]);
    current_7x1[6] = torque_OLcurrent_map_RWY2(torque_7x1[6]);



    for(int i=0; i<7; i++)
    {
        if(current_7x1[i] > 300)              current_7x1[i] = 300;
        else if(current_7x1[i] < -300)        current_7x1[i] = -300;
    }

//    RBJointOLCurrentCommand2ch(2, 13, current_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 14, current_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 15, current_7x1[2], current_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(2, 16, current_7x1[4], current_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(2, 36, current_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, current_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, current_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, current_7x1[2], 4, current_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, current_7x1[4], 4, current_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, current_7x1[6], 0, 0);
}

void JointForceControl_LeftArm(double LSP_ang, double LSR_ang, double LSY_ang, double LEB_ang, double LWY_ang, double LWP_ang, double LF1_ang){
    double ref_pos_7x1[7] = {LSP_ang, LSR_ang, LSY_ang, LEB_ang, LWY_ang, LWP_ang, LF1_ang};
    double cur_pos_7x1[7];
    double error_7x1[7];
    double error_dot_7x1[7];
    static double error_7x1_before[7];
    double torque_7x1[7];
    double current_7x1[7];
    const double Pgain = 1;
    const double Dgain = 0.1;

    cur_pos_7x1[0] = sharedSEN->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentPosition;
    cur_pos_7x1[1] = sharedSEN->ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentPosition + OFFSET_LSR;
    cur_pos_7x1[2] = sharedSEN->ENCODER[MC_GetID(LSY)][MC_GetCH(LSY)].CurrentPosition;
    cur_pos_7x1[3] = sharedSEN->ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentPosition+ OFFSET_ELB;
    cur_pos_7x1[4] = sharedSEN->ENCODER[MC_GetID(LWY)][MC_GetCH(LWY)].CurrentPosition;
    cur_pos_7x1[5] = sharedSEN->ENCODER[MC_GetID(LWP)][MC_GetCH(LWP)].CurrentPosition;
    cur_pos_7x1[6] = sharedSEN->ENCODER[MC_GetID(LWY2)][MC_GetCH(LWY2)].CurrentPosition;

    // Angle control
    for(int i=0; i<7; i++){
        error_7x1[i] = ref_pos_7x1[i] - cur_pos_7x1[i];
        error_dot_7x1[i] = (error_7x1[i] - error_7x1_before[i])/0.005;
        error_7x1_before[i] = error_7x1[i];
        if(i == 0 || i == 1){
            torque_7x1[i] = 4*Pgain*error_7x1[i] + 4*Dgain*error_dot_7x1[i];
        }
        else if(i == 2){
            torque_7x1[i] = 2*Pgain*error_7x1[i] + 2*Dgain*error_dot_7x1[i];
        }
        else if(i == 3){
            torque_7x1[i] = 4*Pgain*error_7x1[i] + 4*Dgain*error_dot_7x1[i];
        }
        else{
            torque_7x1[i] = Pgain*error_7x1[i] + Dgain*error_dot_7x1[i];
        }

    }

    current_7x1[0] = torque_OLcurrent_map_LSP(torque_7x1[0]);
    current_7x1[1] = torque_OLcurrent_map_LSR(torque_7x1[1]);
    current_7x1[2] = torque_OLcurrent_map_LSY(torque_7x1[2]);
    current_7x1[3] = torque_OLcurrent_map_LEB(torque_7x1[3]);
    current_7x1[4] = torque_OLcurrent_map_LWY(torque_7x1[4]);
    current_7x1[5] = torque_OLcurrent_map_LWP(torque_7x1[5]);
    current_7x1[6] = torque_OLcurrent_map_LWY2(torque_7x1[6]);



    for(int i=0; i<7; i++)
    {
        if(current_7x1[i] > 300)              current_7x1[i] = 300;
        else if(current_7x1[i] < -300)        current_7x1[i] = -300;
    }

//    RBJointOLCurrentCommand2ch(3, 17, current_7x1[0], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, current_7x1[1], 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, current_7x1[2], current_7x1[3], 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, current_7x1[4], current_7x1[5], 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, current_7x1[6], 0, 0x05);
    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, current_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, current_7x1[0], 0, 0);
    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, current_7x1[2], 4, current_7x1[3]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, current_7x1[4], 4, current_7x1[5]);
    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, current_7x1[6], 0, 0);

}
