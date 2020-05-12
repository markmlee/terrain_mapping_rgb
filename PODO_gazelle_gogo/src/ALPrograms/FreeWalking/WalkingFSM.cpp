#include "WalkingFSM.h"
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/UserSharedMemory.h"

//// Shared memory
using namespace std;
double fsmDEL_T = 0.005;
const double RATIO = 10;
//double tempFSM->TIME_DSP = 0.5,tempFSM->TIME_SSP = 2.5;
//double tempFSM->TimeRatio = 1,tempFSM->tempFSM->TIME_SSP_Ratio = 1;
const double RATIO_ZMP_Y = 1.0;

//double tempFSM->_com_offset,tempFSM->_temp_lz,tempFSM->_temp_rz, tempFSM->_temp_lx,tempFSM->_temp_rx,tempFSM->_alpha = 6.0f/15.0f,tempFSM->_block_height = 0.15f,tempFSM->_time_diff = 0.0f;
//int tempFSM->_foot_height_diff,tempFSM->_dsp_foot_height_diff;


//double tempFSM->_temp_add_x = 0.05f;
//bool tempFSM->_normal_walking_flag = true;
//int tempFSM->walking_mode;
//double tempFSM->FootUpHeight = 0;//0.08;
//double tempFSM->FootUpTime = 0.35;

//pRBAState	targetStates[NUM_OF_STATE];


enum{
    NOPE =0,
    OKIE
};

enum{
    DOUBLE = 0,
    SINGLE
};
enum{
    TERRAIN = 0,
    NORMAL
};
// ============================================================================================================
void DSPStateInit::ExecuteBeginState(){

    cout << ">> this is ExecuteBeginState() : DSPStateInit..." << endl;
    finishFlag = false;

   // DSPTime = 0.2;
    ITimer = 0.0;
    pWFSM tempFSM = CAST_WFSM(parentFSM);

    printf(">>>>>>>>>>>>>SSP time : %f  DSP time : %f \n" , tempFSM->TIME_SSP, tempFSM->TIME_DSP);

    if(tempFSM->walking_mode == TERRAIN_WALKING || tempFSM->walking_mode == LADDER_WALKING|| tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP)
    {
        tempFSM->isTerrain = TERRAIN;

        printf("----------------------TERRAIN-----------------------\n");
    }else
    {
        tempFSM->isTerrain = NORMAL;

        printf("----------------------NORMAL-----------------------\n");
    }


    tempFSM->SUPPORT_STATE = DOUBLE;


    tempFSM->_temp_lz = 0.0f;
    tempFSM->_temp_rz = 0.0f;



//    tempFSM->TIME_DSP = walkingFSM_shared_data->terrain_variable[0];
//    tempFSM->TIME_SSP = walkingFSM_shared_data->terrain_variable[1];


    int isSamePos = tempFSM->FootSelector();

//    tempFSM->_alpha = walkingFSM_shared_data->terrain_variable[2]/tempFSM->_block_height;

    //printf("shared data test dsp:%f   ssp:%f  alpha: %f\n",tempFSM->TIME_DSP,tempFSM->TIME_SSP,tempFSM->_alpha);

    DSPTime = tempFSM->TIME_DSP;
    tempFSM->CurrentSTime = DSPTime;

    // this is the initial position checker
    // it checks whether current foot positions are same with first DSP Scheduler positions
    if(isSamePos == FOOT_SAME){
        if(tempFSM->DSPScheduler.size() > 0)
            tempFSM->MoveSchedulerToRecent(1);
        else
            tempFSM->DSPRecentTask.pop_front();

        // move information of ZMP toward foot position of next stance leg
        int foot = tempFSM->FootSelector();
        double targetX, targetY, target_com_z;

            tempFSM->TimeRatio = 1.;
        //printf("dsp time = %f\n",DSPTime);
//        tempFSM->CurrentSTime = DSPTime;

        if(foot == FOOT_ERROR){
            cout << ">> Foot Selection Error..!!(DSPinit)" << endl;
            ITimer = DSPTime;
            finishFlag = true;
        }else if(foot == FOOT_LEFT){

//            if(tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP && fabs(tempFSM->RightFoot->Pos[0] - tempFSM->LeftFoot->Pos[0]) > 0.28f && tempFSM->DSPRecentTask[0].Left[0] - tempFSM->LeftFoot->Pos[0] < -0.2f)
             if(tempFSM->heel_pitching_mode == 1 && tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP && fabs(tempFSM->RightFoot->Pos[0] - tempFSM->LeftFoot->Pos[0]) > 0.39f && tempFSM->DSPRecentTask[0].Left[0] - tempFSM->LeftFoot->Pos[0] < -0.2f)
            {
                cout<<"+++++++++++LEFT Back++++++++++++++"<<endl;

                cout<<"+++++DSPINIT Back LEFT" <<tempFSM->LeftFoot->Pos[0]<<endl;
                cout<<"+++++DSPINIT Back LEFT" <<tempFSM->LeftFoot->Pos[1]<<endl;
                cout<<"+++++DSPINIT Back LEFT" <<tempFSM->LeftFoot->Pos[2]<<endl;

                cout<<"+++++DSPINIT Back Right" <<tempFSM->RightFoot->Pos[0]<<endl;
                cout<<"+++++DSPINIT Back Right" <<tempFSM->RightFoot->Pos[1]<<endl;
                cout<<"+++++DSPINIT Back Right" <<tempFSM->RightFoot->Pos[2]<<endl;


                tempFSM->LL_Heel_Configurtion(tempFSM->LeftFoot->Pos[0], tempFSM->LeftFoot->Pos[1],tempFSM->LeftFoot->Pos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch,tempFSM->_LHpos);
                cout<<"+++++DSPINIT Back Heel" <<tempFSM->_LHpos[0]<<endl;
                cout<<"+++++DSPINIT Back Heel" <<tempFSM->_LHpos[1]<<endl;
                cout<<"+++++DSPINIT Back Heel" <<tempFSM->_LHpos[2]<<endl;
                cout<<"+++++DSPINIT Back Heel yaw" <<tempFSM->LeftFoot->Yaw<<endl;
                cout<<"+++++DSPINIT Back Heel roll" <<tempFSM->LeftFoot->Roll<<endl;
                cout<<"+++++DSPINIT Back Heel pitch" <<tempFSM->LeftFoot->Pitch<<endl;

                //---------heel pitching
//                AddHeelPitching_x(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){

                tempFSM->LeftFoot->AddXFifth(tempFSM->LeftFoot->Pos[0], DSPTime*3.f/10.0f);
                tempFSM->LeftFoot->AddYFifth(tempFSM->LeftFoot->Pos[1], DSPTime*3.f/10.0f);
                tempFSM->LeftFoot->AddZFifth(tempFSM->LeftFoot->Pos[2], DSPTime*3.f/10.0f);
                tempFSM->LeftFoot->AddYawFifth(tempFSM->LeftFoot->Yaw, DSPTime*3.f/10.0f);
                tempFSM->LeftFoot->AddPitchFifth(tempFSM->LeftFoot->Pitch, DSPTime*3.f/10.0f);
                tempFSM->LeftFoot->AddRollFifth(tempFSM->LeftFoot->Roll, DSPTime*3.f/10.0f);

                tempFSM->LeftFoot->AddHeelPitching_x(DSPTime*7.f/10.0f,-30.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
                tempFSM->LeftFoot->AddHeelPitching_y(DSPTime*7.f/10.0f,-30.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
                tempFSM->LeftFoot->AddHeelPitching_z(DSPTime*7.f/10.0f,-30.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);

                tempFSM->LeftFoot->AddHeelPitching_yaw(DSPTime*7.f/10.0f,-30.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
                tempFSM->LeftFoot->AddHeelPitching_roll(DSPTime*7.f/10.0f,-30.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
                tempFSM->LeftFoot->AddHeelPitching_pitch(DSPTime*7.f/10.0f,-30.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);

            }
//            else if(tempFSM->walking_mode == TERRAIN_WALKING && fabs(tempFSM->RightFoot->Pos[0] - tempFSM->LeftFoot->Pos[0]) > 0.28f && tempFSM->DSPRecentTask[0].Left[0] - tempFSM->LeftFoot->Pos[0] > 0.2f)
//            {

//                cout<<"+++++++++++LEFT Forward++++++++++++++"<<endl;

//                cout<<"+++++DSPINIT Forward LEFT" <<tempFSM->LeftFoot->Pos[0]<<endl;
//                cout<<"+++++DSPINIT Forward LEFT" <<tempFSM->LeftFoot->Pos[1]<<endl;
//                cout<<"+++++DSPINIT Forward LEFT" <<tempFSM->LeftFoot->Pos[2]<<endl;

//                cout<<"+++++DSPINIT Forward Right" <<tempFSM->RightFoot->Pos[0]<<endl;
//                cout<<"+++++DSPINIT Forward Right" <<tempFSM->RightFoot->Pos[1]<<endl;
//                cout<<"+++++DSPINIT Forward Right" <<tempFSM->RightFoot->Pos[2]<<endl;

//                tempFSM->Forward_LL_Heel_Configurtion(tempFSM->LeftFoot->Pos[0], tempFSM->LeftFoot->Pos[1],tempFSM->LeftFoot->Pos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch,tempFSM->_LHpos);

//                cout<<"+++++DSPINIT Forward Heel" <<tempFSM->_LHpos[0]<<endl;
//                cout<<"+++++DSPINIT Forward Heel" <<tempFSM->_LHpos[1]<<endl;
//                cout<<"+++++DSPINIT Forward Heel" <<tempFSM->_LHpos[2]<<endl;
//                //---------heel pitching
////                AddHeelPitching_x(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
//                tempFSM->LeftFoot->AddHeelPitching_x(DSPTime,45.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
//                tempFSM->LeftFoot->AddHeelPitching_y(DSPTime,45.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
//                tempFSM->LeftFoot->AddHeelPitching_z(DSPTime,45.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);

//                tempFSM->LeftFoot->AddHeelPitching_yaw(DSPTime,45.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
//                tempFSM->LeftFoot->AddHeelPitching_roll(DSPTime,45.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);
//                tempFSM->LeftFoot->AddHeelPitching_pitch(DSPTime,45.0f,tempFSM->_LHpos[0], tempFSM->_LHpos[1],tempFSM->_LHpos[2],tempFSM->LeftFoot->Yaw,tempFSM->LeftFoot->Roll,tempFSM->LeftFoot->Pitch);

//            }



            tempFSM->CompData->AddRightFootCos(2,tempFSM->WindowSize*fsmDEL_T);
            tempFSM->CompData->AddRightFootCos(2,-0.0035,DSPTime);
            tempFSM->PrevFoot = FOOT_LEFT;
            tempFSM->CurrentState = STATE_DSP_INIT_LF;
            if(tempFSM->IsZeroStep == true){
                targetX = tempFSM->RightFoot->Pos[0];
                targetY = tempFSM->RightFoot->Pos[1];
            }else{
                targetX = tempFSM->DSPRecentTask[0].Right[0];
                targetY = tempFSM->DSPRecentTask[0].Right[1];// + 0.03*sin(-tempFSM->DSPRecentTask[0].RRoll*D2Rf);

                //----2015.3.18
                target_com_z = tempFSM->DSPRecentTask[0].COMz[2];

            }
            tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime);

            //AddComp
//            tempFSM->CompData->AddJointsFifth(JRAR,DSPTime-0.05*tempFSM->TimeRatio);
//            tempFSM->CompData->AddJointsFifth(JRAR,0.07,0.15*tempFSM->TimeRatio);


//            if(tempFSM->_normal_walking_flag == false){
//                if(tempFSM->_dsp_foot_height_diff == true){
//                    tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*tempFSM->_block_height ,DSPTime,0.0,0.0);
//                    printf("DSPSTATE LEFT UNEVEN %f \n",tempFSM->_block_height);
//                }else{
//                    tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
//                    printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
//                }
//            }else{
//                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
//                printf("DSPSTATE RIGHT EVEN %f \n",tempFSM->_block_height);
//            }

            //--------------------comz 2015.3.18
            tempFSM->CompData->AddCOMFifth(2,target_com_z,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);

            printf("DSPStateInit comz : %f \n", target_com_z);


            if(tempFSM->_normal_walking_flag == false){
                if(tempFSM->_dsp_foot_height_diff == true){
                    tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height ,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
                    printf("DSPSTATE LEFT UNEVEN %f \n",tempFSM->_block_height);
                }else{
                    tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
                    printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
                }
            }else{
                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
                printf("DSPSTATE RIGHT EVEN %f \n",tempFSM->_block_height);
            }





        }else if(foot == FOOT_RIGHT){


//            if(tempFSM->walking_mode == TERRAIN_WALKING && fabs(tempFSM->DSPRecentTask[0].Right[0] - tempFSM->RightFoot->Pos[0]) > 0.1f)
//            if(tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP && fabs(tempFSM->RightFoot->Pos[0] - tempFSM->LeftFoot->Pos[0]) > 0.28f && tempFSM->DSPRecentTask[0].Right[0] - tempFSM->RightFoot->Pos[0] < -0.2f)
            if(tempFSM->heel_pitching_mode == 1 && tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP && fabs(tempFSM->RightFoot->Pos[0] - tempFSM->LeftFoot->Pos[0]) > 0.39f && tempFSM->DSPRecentTask[0].Right[0] - tempFSM->RightFoot->Pos[0] < -0.2f)
            {
                cout<<"+++++++++++Right Back++++++++++++++"<<endl;

                cout<<"+++++DSPINIT Back LEFT" <<tempFSM->LeftFoot->Pos[0]<<endl;
                cout<<"+++++DSPINIT Back LEFT" <<tempFSM->LeftFoot->Pos[1]<<endl;
                cout<<"+++++DSPINIT Back LEFT" <<tempFSM->LeftFoot->Pos[2]<<endl;

                cout<<"+++++DSPINIT Back Right" <<tempFSM->RightFoot->Pos[0]<<endl;
                cout<<"+++++DSPINIT Back Right" <<tempFSM->RightFoot->Pos[1]<<endl;
                cout<<"+++++DSPINIT Back Right" <<tempFSM->RightFoot->Pos[2]<<endl;


                tempFSM->RL_Heel_Configurtion(tempFSM->RightFoot->Pos[0], tempFSM->RightFoot->Pos[1],tempFSM->RightFoot->Pos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch,tempFSM->_RHpos);
                cout<<"+++++DSPINIT Back Heel" <<tempFSM->_RHpos[0]<<endl;
                cout<<"+++++DSPINIT Back Heel" <<tempFSM->_RHpos[1]<<endl;
                cout<<"+++++DSPINIT Back Heel" <<tempFSM->_RHpos[2]<<endl;

                cout<<"+++++DSPINIT Back Heel yaw" <<tempFSM->RightFoot->Yaw<<endl;
                cout<<"+++++DSPINIT Back Heel roll" <<tempFSM->RightFoot->Roll<<endl;
                cout<<"+++++DSPINIT Back Heel pitch" <<tempFSM->RightFoot->Pitch<<endl;


                tempFSM->RightFoot->AddXFifth(tempFSM->RightFoot->Pos[0], DSPTime*3.f/10.0f);
                tempFSM->RightFoot->AddYFifth(tempFSM->RightFoot->Pos[1], DSPTime*3.f/10.0f);
                tempFSM->RightFoot->AddZFifth(tempFSM->RightFoot->Pos[2], DSPTime*3.f/10.0f);
                tempFSM->RightFoot->AddYawFifth(tempFSM->RightFoot->Yaw, DSPTime*3.f/10.0f);
                tempFSM->RightFoot->AddPitchFifth(tempFSM->RightFoot->Pitch, DSPTime*3.f/10.0f);
                tempFSM->RightFoot->AddRollFifth(tempFSM->RightFoot->Roll, DSPTime*3.f/10.0f);

                //---------heel pitching
//                AddHeelPitching_x(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
                tempFSM->RightFoot->AddHeelPitching_x(DSPTime*7.f/10.0f,-30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
                tempFSM->RightFoot->AddHeelPitching_y(DSPTime*7.f/10.0f,-30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
                tempFSM->RightFoot->AddHeelPitching_z(DSPTime*7.f/10.0f,-30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);

                tempFSM->RightFoot->AddHeelPitching_yaw(DSPTime*7.f/10.0f,-30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
                tempFSM->RightFoot->AddHeelPitching_roll(DSPTime*7.f/10.0f,-30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
                tempFSM->RightFoot->AddHeelPitching_pitch(DSPTime*7.f/10.0f,-30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);

            }
//            else if(tempFSM->walking_mode == TERRAIN_WALKING && fabs(tempFSM->RightFoot->Pos[0] - tempFSM->LeftFoot->Pos[0]) > 0.28f && tempFSM->DSPRecentTask[0].Right[0] - tempFSM->RightFoot->Pos[0] > 0.2f)
//            {

//                cout<<"+++++++++++Right Forward++++++++++++++"<<endl;

//                cout<<"+++++DSPINIT Forward LEFT" <<tempFSM->LeftFoot->Pos[0]<<endl;
//                cout<<"+++++DSPINIT Forward LEFT" <<tempFSM->LeftFoot->Pos[1]<<endl;
//                cout<<"+++++DSPINIT Forward LEFT" <<tempFSM->LeftFoot->Pos[2]<<endl;

//                cout<<"+++++DSPINIT Forward Right" <<tempFSM->RightFoot->Pos[0]<<endl;
//                cout<<"+++++DSPINIT Forward Right" <<tempFSM->RightFoot->Pos[1]<<endl;
//                cout<<"+++++DSPINIT Forward Right" <<tempFSM->RightFoot->Pos[2]<<endl;


//                tempFSM->Forward_RL_Heel_Configurtion(tempFSM->RightFoot->Pos[0], tempFSM->RightFoot->Pos[1],tempFSM->RightFoot->Pos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch,tempFSM->_RHpos);

//                cout<<"+++++DSPINIT Forward Heel" <<tempFSM->_RHpos[0]<<endl;
//                cout<<"+++++DSPINIT Forward Heel" <<tempFSM->_RHpos[1]<<endl;
//                cout<<"+++++DSPINIT Forward Heel" <<tempFSM->_RHpos[2]<<endl;

//                //---------heel pitching
////                AddHeelPitching_x(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
//                tempFSM->RightFoot->AddHeelPitching_x(DSPTime,30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
//                tempFSM->RightFoot->AddHeelPitching_y(DSPTime,30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
//                tempFSM->RightFoot->AddHeelPitching_z(DSPTime,30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);

//                tempFSM->RightFoot->AddHeelPitching_yaw(DSPTime,30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
//                tempFSM->RightFoot->AddHeelPitching_roll(DSPTime,30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);
//                tempFSM->RightFoot->AddHeelPitching_pitch(DSPTime,30.0f,tempFSM->_RHpos[0], tempFSM->_RHpos[1],tempFSM->_RHpos[2],tempFSM->RightFoot->Yaw,tempFSM->RightFoot->Roll,tempFSM->RightFoot->Pitch);

//            }

            tempFSM->CompData->AddLeftFootCos(2,tempFSM->WindowSize*fsmDEL_T);
            tempFSM->CompData->AddLeftFootCos(2,-0.0035,DSPTime);
            tempFSM->PrevFoot = FOOT_RIGHT;
            tempFSM->CurrentState = STATE_DSP_INIT_RF;
            if(tempFSM->IsZeroStep == true){
                targetX = tempFSM->LeftFoot->Pos[0];
                targetY = tempFSM->LeftFoot->Pos[1];
            }else{
                targetX = tempFSM->DSPRecentTask[0].Left[0];
                targetY = tempFSM->DSPRecentTask[0].Left[1];//  + 0.03*sin(-tempFSM->DSPRecentTask[0].LRoll*D2Rf);
                target_com_z = tempFSM->DSPRecentTask[0].COMz[2];
            }
            tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime);

            //AddComp
//            tempFSM->CompData->AddJointsFifth(JLAR,DSPTime-0.05*tempFSM->TimeRatio);
//            tempFSM->CompData->AddJointsFifth(JLAR,0.07,0.15*tempFSM->TimeRatio);


            //--------------------2015.3.18 comz

            tempFSM->CompData->AddCOMFifth(2,target_com_z,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
            //--------------------2015.3.18

            printf("DSPStateInit comz : %f \n", target_com_z);



            if(tempFSM->_normal_walking_flag == false){
                if(tempFSM->_dsp_foot_height_diff == true){
                    tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height ,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
                    printf("DSPSTATE RIGHT UNEVEN %f \n",tempFSM->_block_height);
                }else{
                    tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
                    printf("DSPSTATE RIGHT EVEN %f \n",tempFSM->_block_height);
                }
            }else{
                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime+tempFSM->WindowSize*0.5*fsmDEL_T,0.0,0.0);
                printf("DSPSTATE RIGHT EVEN %f \n",tempFSM->_block_height);
            }


        }
        else {
            tempFSM->CurrentState = STATE_DSP_INIT_RF;
        }
        // add first 2 step dummy information
        for(int i=0; i<tempFSM->WindowSize; i++){
            tempFSM->DataUpdate();
            tempFSM->CompData->UpdateData();
            //cout<<i<<endl;
        }
    }else{
        cout << ">> Initial Position in Different..!!" << endl;
        ITimer = DSPTime+fsmDEL_T;
        tempFSM->ClearAll();
    }


    tempFSM->_temp_lz = tempFSM->LeftFoot->Pos[2];
    tempFSM->_temp_rz = tempFSM->RightFoot->Pos[2];


    tempFSM->_temp_lx = tempFSM->LeftFoot->Pos[0];
    tempFSM->_temp_rx = tempFSM->RightFoot->Pos[0];

    cout<<"LEFT FOOT -> POS[2]"<<tempFSM->LeftFoot->Pos[2];

    cout<<"RIGHT FOOT -> POS[2]"<<tempFSM->LeftFoot->Pos[2];


}
void DSPStateInit::ExecuteDoingState(){
    pWFSM tempFSM = CAST_WFSM(parentFSM);
    tempFSM->CurrentITime = ITimer;

    tempFSM->LeftFoot->UpdateData();
    tempFSM->RightFoot->UpdateData();
    tempFSM->ZMPPos->UpdateData();
    tempFSM->CompData->UpdateData();

    ITimer += fsmDEL_T;
    //cout << ">> time = "<<ITimer<<" , dspTime = "<< DSPTime<< endl;
    if(ITimer >= DSPTime)
        finishFlag = true;
}
void DSPStateInit::ExecuteEndState(){
    cout << ">> this is ExecuteEndState() : DSPStateInit..." << endl;
}
// ============================================================================================================


// ============================================================================================================
void DSPState::ExecuteBeginState(){


    cout << ">> this is ExecuteBeginState() : DSPState..." << endl;
    finishFlag = false;
    pWFSM tempFSM = CAST_WFSM(parentFSM);

    printf(">>>>>>>>>>>>>SSP time : %f  DSP time : %f \n" , tempFSM->TIME_SSP, tempFSM->TIME_DSP);

    if(tempFSM->walking_mode == TERRAIN_WALKING || tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP|| tempFSM->walking_mode == LADDER_WALKING)
    {
        tempFSM->isTerrain = TERRAIN;

        printf("----------------------TERRAIN-----------------------\n");
    }else
    {
        tempFSM->isTerrain = NORMAL;

        printf("----------------------NORMAL-----------------------\n");
    }


    tempFSM->SUPPORT_STATE = DOUBLE;

    int foot = tempFSM->FootSelector();

        tempFSM->TimeRatio = 1.;

    DSPTime = tempFSM->TIME_DSP*tempFSM->TimeRatio;
    HoldTime = 0.0;
    ITimer = 0.0;
    HTimer = 0.0;


    tempFSM->CurrentSTime = DSPTime;
    HoldTime = tempFSM->HoldTime;
    //cout << "HoldTime: " << HoldTime << endl;
    // move information of ZMP toward foot position of next stance leg

    double targetX, targetY, target_com_z;
    if(foot == FOOT_ERROR){
        cout << ">> Foot Selection Error..!!(DSP)" << endl;
        ITimer = DSPTime;
        finishFlag = true;
    }else if(foot == FOOT_LEFT){
        if(tempFSM->PrevFoot == FOOT_LEFT){
            targetX = (tempFSM->RightFoot->Pos[0] + tempFSM->LeftFoot->Pos[0])/2.0;
            targetY = (tempFSM->RightFoot->Pos[1] + tempFSM->LeftFoot->Pos[1])/2.0;
            target_com_z = tempFSM->DSPRecentTask[0].COMz[2];

            tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);

            tempFSM->ZMPPos->AddXZMPFifth(targetX, HoldTime);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, HoldTime);

            targetX = tempFSM->RightFoot->Pos[0];
            targetY = tempFSM->RightFoot->Pos[1];

            tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);
        }else{
            if(HoldTime>=fsmDEL_T)
            {
                targetX = (tempFSM->RightFoot->Pos[0] + tempFSM->LeftFoot->Pos[0])/2.0;
                targetY = (tempFSM->RightFoot->Pos[1] + tempFSM->LeftFoot->Pos[1])/2.0;
                target_com_z = tempFSM->DSPRecentTask[0].COMz[2];


                tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);

                tempFSM->ZMPPos->AddXZMPFifth(targetX, HoldTime);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, HoldTime);

                targetX = tempFSM->DSPRecentTask[0].Right[0];
                targetY = tempFSM->DSPRecentTask[0].Right[1];// + 0.03*sin(-tempFSM->DSPRecentTask[0].RRoll*D2Rf);

                tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);
            }
            else
            {
                targetX = tempFSM->DSPRecentTask[0].Right[0];
                targetY = tempFSM->DSPRecentTask[0].Right[1];//  + 0.03*sin(-tempFSM->DSPRecentTask[0].RRoll*D2Rf);
                target_com_z = tempFSM->DSPRecentTask[0].COMz[2];

                tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime*6./6.);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime*6./6.);
            }
        }

        tempFSM->PrevFoot = FOOT_LEFT;
        tempFSM->CurrentState = STATE_DSP_LF;

        //AddComp
        tempFSM->CompData->AddJointsFifth(JLAR,0.0  ,DSPTime/2.0);
        //tempFSM->CompData->AddJointsFifth(JRAR,DSPTime-0.05*tempFSM->TimeRatio);
        //tempFSM->CompData->AddJointsFifth(JRAR,0.07,0.15*tempFSM->TimeRatio);

        tempFSM->CompData->AddLeftFootCos(2,0,DSPTime/2.);
        tempFSM->CompData->AddRightFootCos(2,0,DSPTime/2.);
        tempFSM->CompData->AddRightFootCos(2,-0.0035,DSPTime/2.);

//        if(tempFSM->_normal_walking_flag == false){
//            if(tempFSM->_dsp_foot_height_diff == true){
//                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*tempFSM->_block_height ,DSPTime,0.0,0.0);
//                //printf("DSPSTATE LEFT UNEVEN %f \n",tempFSM->_block_height);
//            }else{
//                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
//                //printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
//            }
//        }else{
//            tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
//            //printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
//        }

        //--------------------2015.3.18 comz
        if(HoldTime>=fsmDEL_T)
        {
            tempFSM->CompData->AddCOMFifth(2,target_com_z,DSPTime + HoldTime,0.0,0.0);
        }else
        {
            tempFSM->CompData->AddCOMFifth(2,target_com_z,DSPTime,0.0,0.0);
        }
        //--------------------2015.3.18


        if(tempFSM->_normal_walking_flag == false){
            if(tempFSM->_dsp_foot_height_diff == true){
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height ,DSPTime + HoldTime,0.0,0.0);
                //printf("DSPSTATE LEFT UNEVEN %f \n",tempFSM->_block_height);
            }else{
                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
                //printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
            }
        }else{
            tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
            //printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
        }


    }else if(foot == FOOT_RIGHT){
        if(tempFSM->PrevFoot == FOOT_RIGHT){ //
            targetX = (tempFSM->RightFoot->Pos[0] + tempFSM->LeftFoot->Pos[0])/2.0;
            targetY = (tempFSM->RightFoot->Pos[1] + tempFSM->LeftFoot->Pos[1])/2.0;
            target_com_z = tempFSM->DSPRecentTask[0].COMz[2];

            tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);

            tempFSM->ZMPPos->AddXZMPFifth(targetX, HoldTime);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, HoldTime);

            targetX = tempFSM->LeftFoot->Pos[0];
            targetY = tempFSM->LeftFoot->Pos[1];

            tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
            tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);

        }else{
            if(HoldTime>=fsmDEL_T)
            {
                targetX = (tempFSM->RightFoot->Pos[0] + tempFSM->LeftFoot->Pos[0])/2.0;
                targetY = (tempFSM->RightFoot->Pos[1] + tempFSM->LeftFoot->Pos[1])/2.0;
                target_com_z = tempFSM->DSPRecentTask[0].COMz[2];

                tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);

                tempFSM->ZMPPos->AddXZMPFifth(targetX, HoldTime);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, HoldTime);

                targetX = tempFSM->DSPRecentTask[0].Left[0];
                targetY = tempFSM->DSPRecentTask[0].Left[1];//  + 0.03*sin(-tempFSM->DSPRecentTask[0].LRoll*D2Rf);

                tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime/2.0);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime/2.0);
            }
            else
            {
                targetX = tempFSM->DSPRecentTask[0].Left[0];
                targetY = tempFSM->DSPRecentTask[0].Left[1];// + 0.03*sin(-tempFSM->DSPRecentTask[0].LRoll*D2Rf);
                target_com_z = tempFSM->DSPRecentTask[0].COMz[2];

                tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime*6./6.);
                tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime*6./6.);
            }
        }
        tempFSM->PrevFoot = FOOT_RIGHT;
        tempFSM->CurrentState = STATE_DSP_RF;

        //AddComp
        tempFSM->CompData->AddJointsFifth(JRAR,0.0  ,DSPTime/2.0);
//        tempFSM->CompData->AddJointsFifth(JLAR,DSPTime-0.05*tempFSM->TimeRatio);
//        tempFSM->CompData->AddJointsFifth(JLAR,0.07,0.15*tempFSM->TimeRatio);

        tempFSM->CompData->AddRightFootCos(2,0,DSPTime/2.);
        tempFSM->CompData->AddLeftFootCos(2,0,DSPTime/2.);
        tempFSM->CompData->AddLeftFootCos(2,-0.0035,DSPTime/2.);

//        if(tempFSM->_normal_walking_flag == false){
//            if(tempFSM->_dsp_foot_height_diff == true){
//                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*tempFSM->_block_height ,DSPTime,0.0,0.0);
//                printf("DSPSTATE RIGHT UNEVEN %f \n",tempFSM->_block_height);
//            }else{
//                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
//                printf("DSPSTATE RIGHT EVEN %f \n",tempFSM->_block_height);
//            }
//        }else{
//            tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
//            printf("DSPSTATE RIGHT EVEN %f \n",tempFSM->_block_height);
//        }

        //--------------------2015.3.18 comz
        if(HoldTime>=fsmDEL_T)
        {
            tempFSM->CompData->AddCOMFifth(2,target_com_z,DSPTime + HoldTime,0.0,0.0);
        }else
        {
            tempFSM->CompData->AddCOMFifth(2,target_com_z,DSPTime,0.0,0.0);
        }
        //--------------------2015.3.18


        if(tempFSM->_normal_walking_flag == false){
            if(tempFSM->_dsp_foot_height_diff == true){
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height ,DSPTime + HoldTime,0.0,0.0);
                //printf("DSPSTATE LEFT UNEVEN %f \n",tempFSM->_block_height);
            }else{
                tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
                //printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
            }
        }else{
            tempFSM->CompData->AddCOMFifth(1,-(1.0f-tempFSM->_alpha)*0.0f ,DSPTime,0.0,0.0);
            //printf("DSPSTATE LEFT EVEN %f \n",tempFSM->_block_height);
        }


    }
    tempFSM->_temp_lz = tempFSM->LeftFoot->Pos[2];
    tempFSM->_temp_rz = tempFSM->RightFoot->Pos[2];

    tempFSM->_temp_lx = tempFSM->LeftFoot->Pos[0];
    tempFSM->_temp_rx = tempFSM->RightFoot->Pos[0];

}
void DSPState::ExecuteDoingState(){
    pWFSM tempFSM = CAST_WFSM(parentFSM);
    tempFSM->CurrentITime = ITimer;

    tempFSM->LeftFoot->UpdateData();
    tempFSM->RightFoot->UpdateData();
    tempFSM->ZMPPos->UpdateData();
    tempFSM->CompData->UpdateData();

    ITimer += fsmDEL_T;
    if(ITimer >= DSPTime){
        HTimer += fsmDEL_T;
        if(HTimer >= HoldTime)
            finishFlag = true;
    }
}
void DSPState::ExecuteEndState(){
    cout << ">> this is ExecuteEndState() : DSPState..." << endl;
}
// ============================================================================================================


// ============================================================================================================
void DSPStateFinal::ExecuteBeginState(){
    cout << ">> this is ExecuteBeginState() : DSPStateFinal..." << endl;
    finishFlag = false;

    pWFSM tempFSM = CAST_WFSM(parentFSM);
    printf(">>>>>>>>>>>>>SSP time : %f  DSP time : %f \n" , tempFSM->TIME_SSP, tempFSM->TIME_DSP);

    if(tempFSM->walking_mode == TERRAIN_WALKING || tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP|| tempFSM->walking_mode == LADDER_WALKING )
    {
        tempFSM->isTerrain = TERRAIN;

        printf("----------------------TERRAIN-----------------------\n");
    }else
    {
        tempFSM->isTerrain = NORMAL;

        printf("----------------------NORMAL-----------------------\n");
    }
    tempFSM->SUPPORT_STATE = DOUBLE;

    tempFSM->TimeRatio = 1.;
    DSPTime = tempFSM->TIME_DSP*tempFSM->TimeRatio;
    //DSPTime = 0.2;
    ITimer = 0.0;


    tempFSM->CurrentSTime = DSPTime;
    // this is the last moving ZMP to middle point of its both foot
    double targetX, targetY;

    tempFSM->CurrentState = STATE_DSP_FINAL_RFLF;

    targetX = (tempFSM->DSPRecentTask[0].Left[0] + tempFSM->DSPRecentTask[0].Right[0])/2.0;
    targetY = (tempFSM->DSPRecentTask[0].Left[1] + tempFSM->DSPRecentTask[0].Right[1])/2.0;

    tempFSM->ZMPPos->AddXZMPFifth(targetX, DSPTime*6./6.0);
    //printf("+++++++++++++ zmp targetX = %f\n",targetX);
    tempFSM->ZMPPos->AddYZMPFifth(targetY, DSPTime*6./6.0);

    //AddComp
    tempFSM->CompData->AddJointsFifth(JLAR,0.0  ,DSPTime/2.);
    tempFSM->CompData->AddJointsFifth(JRAR,0.0  ,DSPTime/2.);
    tempFSM->CompData->AddJointsFifth(JLHR,0.0  ,DSPTime/2.);
    tempFSM->CompData->AddJointsFifth(JRHR,0.0  ,DSPTime/2.);
    tempFSM->CompData->AddJointsFifth(JLAP,0.0  ,DSPTime/2.);
    tempFSM->CompData->AddJointsFifth(JRAP,0.0  ,DSPTime/2.);

    //tempFSM->CompData->AddRightFootFifth(2,0,DSPTime);
    //tempFSM->CompData->AddLeftFootFifth(2,0,DSPTime);
    tempFSM->_temp_lz = tempFSM->LeftFoot->Pos[2];
    tempFSM->_temp_rz = tempFSM->RightFoot->Pos[2];

    tempFSM->_temp_lx = tempFSM->LeftFoot->Pos[0];
    tempFSM->_temp_rx = tempFSM->RightFoot->Pos[0];

    tempFSM->CompData->AddLeftFootCos(2,0,DSPTime);
    tempFSM->CompData->AddRightFootCos(2,0,DSPTime);
}
void DSPStateFinal::ExecuteDoingState(){
    pWFSM tempFSM = CAST_WFSM(parentFSM);
    tempFSM->CurrentITime = ITimer;

    tempFSM->LeftFoot->UpdateData();
    tempFSM->RightFoot->UpdateData();
    tempFSM->ZMPPos->UpdateData();
    tempFSM->CompData->UpdateData();

    ITimer += fsmDEL_T;
    if(ITimer >= DSPTime+3.0){
        finishFlag = true;
        tempFSM->CurrentState = STATE_FINISHED;
    }
}
void DSPStateFinal::ExecuteEndState(){
    cout << ">> this is ExecuteEndState() : DSPFinalState..." << endl;
}
// ============================================================================================================


// ============================================================================================================
void SSPStateForward::ExecuteBeginState(){
    cout << ">> this is ExecuteBeginState() : SSPForwardState..." << endl;
    finishFlag = false;
    ITimer = 0.0;

    pWFSM tempFSM = CAST_WFSM(parentFSM);
    printf(">>>>>>>>>>>>>SSP time : %f  DSP time : %f \n" , tempFSM->TIME_SSP, tempFSM->TIME_DSP);

    if(tempFSM->walking_mode == TERRAIN_WALKING || tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP|| tempFSM->walking_mode == LADDER_WALKING)
    {
        tempFSM->isTerrain = TERRAIN;

        printf("----------------------TERRAIN-----------------------\n");
    }else
    {
        tempFSM->isTerrain = NORMAL;

        printf("----------------------NORMAL-----------------------\n");
    }
    tempFSM->SUPPORT_STATE = SINGLE;


    static double targetX=0., targetY=0., targetZ=0., targetYaw=0., target_com_z=0., target_com_z2=0., targetRoll=0., targetPitch=0.;

    int foot = tempFSM->FootSelector();

    //printf("tempFSM->DSPScheduler.size() = %d\n",tempFSM->DSPRecentTask.size());
//    if(fabs(tempFSM->StepLength)>0.55)
//        SSPTime = tempFSM->TIME_SSP + 0.1;
//    else if((fabs(tempFSM->StepLength)>0.45)&&(fabs(tempFSM->StepLength)<=0.55))
//        SSPTime = tempFSM->TIME_SSP + 0.05;
//    else
        SSPTime = tempFSM->TIME_SSP*tempFSM->TIME_SSP_Ratio;

    tempFSM->CurrentSTime = SSPTime;

    //cout << "(" << foot << ")";
    if(foot == FOOT_ERROR){
        cout << ">> Foot Selection Error..!!(SSP)" << endl;
        ITimer = SSPTime;
        finishFlag = true;
    }else if(foot == FOOT_LEFT)
    {
        //cout << "Left: ";
        tempFSM->PrevFoot = FOOT_LEFT;
        tempFSM->CurrentState = STATE_SSP_LF;

        if(tempFSM->IsZeroStep == true){
            targetX = tempFSM->LeftFoot->Pos[0];
            targetY = tempFSM->LeftFoot->Pos[1];
//            target_com_z = tempFSM->DSPRecentTask[0].COMz[0];
        }else{
            targetX = tempFSM->DSPRecentTask[0].Left[0];
            targetY = tempFSM->DSPRecentTask[0].Left[1];
            targetYaw = tempFSM->DSPRecentTask[0].LYaw;
            targetRoll = tempFSM->DSPRecentTask[0].LRoll;
            targetPitch = tempFSM->DSPRecentTask[0].LPitch;
            target_com_z = tempFSM->DSPRecentTask[0].COMz[0];// + tempFSM->RightFoot->Pos[2];
            target_com_z2 = tempFSM->DSPRecentTask[0].COMz[1];// + tempFSM->RightFoot->Pos[2];
            printf("SSP Right  prelf %f   com :%f, %f\n",tempFSM->RightFoot->Pos[2],target_com_z,target_com_z2);
            printf("SSP LEFT %.3f  %.3f  %.3f  %.3f  %.3f  %.3f  %.3f \n",targetX,targetY,tempFSM->DSPRecentTask[0].Left[2],targetYaw,targetRoll,targetPitch,target_com_z);
        }

        if((tempFSM->walking_mode == NORMAL_WALKING)||(tempFSM->walking_mode == GOAL_WALKING))
        {
//            tempFSM->CompData->AddRightFootFifth(2,-0.004,SSPTime-0.2);
//            tempFSM->CompData->AddRightFootFifth(2,0,0.6);
            printf("SSPSTATEFORWARD normal walking \n");

            targetZ = tempFSM->DSPRecentTask[0].Left[2];
            tempFSM->FootDownTarget_LF = tempFSM->DSPRecentTask[0].Left[2];
            tempFSM->FootDownTarget_RF = tempFSM->DSPRecentTask[0].Right[2];

            tempFSM->FootUpHeight = tempFSM->LeftFoot->Pos[2] + 0.08;
//tempFSM->FootUpHeight = tempFSM->LeftFoot->Pos[2]+ 0.00;
            tempFSM->FootUpTime = SSPTime/2.0;
            tempFSM->LeftFoot->AddXFifth(targetX, SSPTime-0.1);
            tempFSM->LeftFoot->AddYFifth(targetY, SSPTime-0.1);

            tempFSM->LeftFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);//changed 0.05->0.06
            tempFSM->LeftFoot->AddZCos(tempFSM->LeftFoot->Pos[2], SSPTime/2.0);
//            tempFSM->LeftFoot->AddZCos(tempFSM->LeftFoot->Pos[2], SSPTime/2.0-0.1);
//            tempFSM->LeftFoot->AddZCos(tempFSM->LeftFoot->Pos[2] - 0.03, 0.2);
//            tempFSM->LeftFoot->AddZCos(tempFSM->LeftFoot->Pos[2], 0.5);

            tempFSM->LeftFoot->AddYawFifth(targetYaw, SSPTime);
            tempFSM->_time_diff = 0.0f;

            tempFSM->_temp_lz = targetZ;
            tempFSM->_temp_lx = targetX;

        }else if(tempFSM->walking_mode == TERRAIN_WALKING|| tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP)
        {
//            tempFSM->CompData->AddRightFootFifth(2,-0.008,SSPTime-0.01);
//            tempFSM->CompData->AddRightFootFifth(2,0,0.25);

            if(tempFSM->IsUpward == OKIE)
            {
                printf("TERRAIN TASK SSPSTATEFORWARD UP WALKING \n");
                tempFSM->LeftFoot->AddXCos(SSPTime*4.0f/10.0f);
                tempFSM->LeftFoot->AddXCos(targetX, SSPTime*4.f/10.0f);

                //tempFSM->LeftFoot->AddYCos(targetY, SSPTime);
                tempFSM->LeftFoot->AddYCos(SSPTime*4.0f/10.0f);
                tempFSM->LeftFoot->AddYCos(targetY, SSPTime*4.f/10.0f);

                targetZ = tempFSM->DSPRecentTask[0].Left[2];
                tempFSM->FootUpHeight = targetZ + 0.11f;
                tempFSM->FootUpTime = SSPTime*4.0f/10.0f;
                tempFSM->LeftFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
                tempFSM->LeftFoot->AddZCos(SSPTime*3.0f/10.0f);
                if(tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP || tempFSM->walking_mode == TERRAIN_WALKING)
                    tempFSM->LeftFoot->AddZCos(targetZ+0.002, SSPTime*3.0f/10.0f);
                else
                    tempFSM->LeftFoot->AddZCos(targetZ, SSPTime*3.0f/10.0f);

                tempFSM->CompData->AddJointsFifth(JLHY,SSPTime*0.1);
                tempFSM->CompData->AddJointsFifth(JLHY,10,SSPTime*0.4);
                tempFSM->CompData->AddJointsFifth(JLHY,  0,SSPTime*0.4);

                tempFSM->LeftFoot->AddYawCos(SSPTime*0.5);
                tempFSM->LeftFoot->AddYawCos(targetYaw, SSPTime*0.4);

                tempFSM->LeftFoot->AddRollCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddRollCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddRollCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddRollCos(8.0f,SSPTime*0.35f);
//                tempFSM->LeftFoot->AddRollCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddRollCos(targetRoll,SSPTime*0.15);

                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddPitchCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

                tempFSM->_temp_lz = targetZ;
                tempFSM->_temp_lx = targetX;

                tempFSM->_time_diff = 0.0f;
            }
            else
            {
                printf("TERRAIN TASK SSPSTATEFORWARD DOWN WALKING \n");
                tempFSM->LeftFoot->AddXCos(SSPTime*0.9f/4.0f);
                tempFSM->LeftFoot->AddXCos(targetX, SSPTime*1.6f/4.0f);

                tempFSM->LeftFoot->AddYCos(SSPTime*0.9f/4.0f);
                tempFSM->LeftFoot->AddYCos(targetY, SSPTime*1.6f/4.0f);

                targetZ = tempFSM->DSPRecentTask[0].Left[2];

                tempFSM->FootUpHeight = tempFSM->_temp_lz + 0.105f;

//                tempFSM->FootUpHeight = tempFSM->RightFoot->Pos[2]+0.1;

                printf("tempFSM->FootUpHeight: %f   targetZ: %f \n",tempFSM->FootUpHeight,targetZ);
                tempFSM->FootUpTime = SSPTime*0.9/4.0f;
                tempFSM->LeftFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
                tempFSM->LeftFoot->AddZCos(SSPTime*1.6/4.0f);
                if(tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP || tempFSM->walking_mode == TERRAIN_WALKING)
                    tempFSM->LeftFoot->AddZCos(targetZ+0.002, SSPTime*1.5f/4.0f);
                else
                    tempFSM->LeftFoot->AddZCos(targetZ, SSPTime*1.5f/4.0f);
                tempFSM->CompData->AddJointsFifth(JLHY,SSPTime*0.1);
                tempFSM->CompData->AddJointsFifth(JLHY,10,SSPTime*0.4);
                tempFSM->CompData->AddJointsFifth(JLHY,  0,SSPTime*0.4);

                tempFSM->LeftFoot->AddYawCos(SSPTime*0.5);
                tempFSM->LeftFoot->AddYawCos(targetYaw, SSPTime*0.4);

                tempFSM->LeftFoot->AddRollCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddRollCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddRollCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddRollCos(8.0f,SSPTime*0.35f);
//                tempFSM->LeftFoot->AddRollCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddRollCos(targetRoll,SSPTime*0.2f);

                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddPitchCos(0.0f,SSPTime*0.15f);
                tempFSM->LeftFoot->AddPitchCos(8.0f,SSPTime*0.15f);
                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.3f);
                tempFSM->LeftFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

                tempFSM->_temp_lz = targetZ;
                tempFSM->_temp_lx = targetX;
                tempFSM->_time_diff = 0.0f;

            }

        }else if(tempFSM->walking_mode == LADDER_WALKING)
        {
//            tempFSM->CompData->AddRightFootFifth(2,-0.008,SSPTime-0.01);
//            tempFSM->CompData->AddRightFootFifth(2,0,0.25);

            if(tempFSM->IsUpward == OKIE)
            {
                printf("STAIR TASK SSPSTATEFORWARD UP WALKING \n");
                tempFSM->LeftFoot->AddXCos(SSPTime*1.0f/10.0f);
                tempFSM->LeftFoot->AddXCos(tempFSM->_temp_lx + tempFSM->_temp_add_x,SSPTime*1.5f/10.0f);
                tempFSM->LeftFoot->AddXCos(tempFSM->_temp_lx,SSPTime*1.5f/10.0f);
                tempFSM->LeftFoot->AddXCos(targetX, SSPTime*4.f/10.0f);

                //tempFSM->LeftFoot->AddYCos(targetY, SSPTime);
                tempFSM->LeftFoot->AddYCos(SSPTime*4.0f/10.0f);
                tempFSM->LeftFoot->AddYCos(targetY, SSPTime*4.f/10.0f);

                targetZ = tempFSM->DSPRecentTask[0].Left[2];
                tempFSM->FootUpHeight = targetZ + 0.05f;
                tempFSM->FootUpTime = SSPTime*4.0f/10.0f;
                tempFSM->LeftFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
                tempFSM->LeftFoot->AddZCos(SSPTime*3.0f/10.0f);
                tempFSM->LeftFoot->AddZCos(targetZ, SSPTime*3.0f/10.0f);
                tempFSM->LeftFoot->AddYawCos(targetYaw, SSPTime*1.5/4.);

                tempFSM->LeftFoot->AddRollCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddRollCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddRollCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddRollCos(targetRoll,SSPTime*0.2);

                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddPitchCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

                tempFSM->_temp_lz = targetZ;
                tempFSM->_temp_lx = targetX;

                tempFSM->_time_diff = 0.0f;
            }
            else
            {
                printf("STAIR TASK SSPSTATEFORWARD DOWN WALKING \n");
                tempFSM->LeftFoot->AddXCos(SSPTime*0.7f/4.0f);
                tempFSM->LeftFoot->AddXCos(targetX, SSPTime*1.8f/4.0f);

                tempFSM->LeftFoot->AddYCos(SSPTime*0.7f/4.0f);
                tempFSM->LeftFoot->AddYCos(targetY, SSPTime*1.8f/4.0f);

                targetZ = tempFSM->DSPRecentTask[0].Left[2];

                tempFSM->FootUpHeight = tempFSM->_temp_lz + 0.08f;

//                tempFSM->FootUpHeight = tempFSM->RightFoot->Pos[2]+0.1;

                printf("tempFSM->FootUpHeight: %f   targetZ: %f \n",tempFSM->FootUpHeight,targetZ);
                tempFSM->FootUpTime = SSPTime*0.7/4.0f;
                tempFSM->LeftFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
                tempFSM->LeftFoot->AddZCos(SSPTime*1.8/4.0f);
                tempFSM->LeftFoot->AddZCos(targetZ, SSPTime*1.5f/4.0f);
                tempFSM->LeftFoot->AddYawCos(targetYaw, SSPTime*1.5/4.0f);

                tempFSM->LeftFoot->AddRollCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddRollCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddRollCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddRollCos(targetRoll,SSPTime*0.2f);

                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.1f);
                tempFSM->LeftFoot->AddPitchCos(0.0f,SSPTime*0.2f);
                tempFSM->LeftFoot->AddPitchCos(SSPTime*0.4f);
                tempFSM->LeftFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

                tempFSM->_temp_lz = targetZ;
                tempFSM->_temp_lx = targetX;
                tempFSM->_time_diff = 0.0f;

            }


        }

            //AddComp
            tempFSM->CompData->AddJointsFifth(JRAR,0.07,0.2);
            tempFSM->CompData->AddJointsFifth(JRAR,SSPTime-0.15);
            tempFSM->CompData->AddJointsFifth(JRAR,0.0,0.2);

            tempFSM->CompData->AddJointsFifth(JRAP,-1.5,0.1);
            tempFSM->CompData->AddJointsFifth(JRAP,SSPTime-0.3);
            tempFSM->CompData->AddJointsFifth(JRAP,0.0,0.1);

            tempFSM->CompData->AddJointsFifth(JRHR,-0.2,0.2);
            tempFSM->CompData->AddJointsFifth(JRHR,SSPTime-0.1);
            tempFSM->CompData->AddJointsFifth(JRHR,0,0.2);

    //        tempFSM->CompData->AddRightFootFifth(2,-0.008,SSPTime-0.01);
    //        tempFSM->CompData->AddRightFootFifth(2,0,0.25);

            //--------------------comz

            tempFSM->CompData->AddCOMFifth(2,target_com_z,SSPTime*0.5,0.0,0.0);
            tempFSM->CompData->AddCOMFifth(2,target_com_z2,SSPTime*0.5,0.0,0.0);

        if(tempFSM->_normal_walking_flag == false){
            if(tempFSM->_foot_height_diff == true){
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height,SSPTime*0.5f,0.0,0.0);
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height + (target_com_z2 - target_com_z),SSPTime*0.5f,0.0,0.0);
            }else{
               tempFSM->CompData->AddCOMFifth(1,tempFSM->_alpha*0.0f,SSPTime,0.0,0.0);
            }
        }else{
            tempFSM->CompData->AddCOMFifth(1,0.0f,SSPTime,0.0,0.0);
        }
        printf("ssp time_diff %f \n",SSPTime);

    }else if(foot == FOOT_RIGHT){
        //cout << "Right: ";
        tempFSM->PrevFoot = FOOT_RIGHT;
        tempFSM->CurrentState = STATE_SSP_RF;

        if(tempFSM->IsZeroStep == true){
            targetX = tempFSM->RightFoot->Pos[0];
            targetY = tempFSM->RightFoot->Pos[1];
//            target_com_z = tempFSM->DSPRecentTask[0].COMz[1];
        }else{
            targetX = tempFSM->DSPRecentTask[0].Right[0];
            targetY = tempFSM->DSPRecentTask[0].Right[1];
            targetYaw = tempFSM->DSPRecentTask[0].RYaw;
            targetRoll = tempFSM->DSPRecentTask[0].RRoll;
            targetPitch = tempFSM->DSPRecentTask[0].RPitch;
            target_com_z = tempFSM->DSPRecentTask[0].COMz[0];// + tempFSM->LeftFoot->Pos[2];
            target_com_z2 = tempFSM->DSPRecentTask[0].COMz[1];// + tempFSM->LeftFoot->Pos[2];
            printf("SSP Right  prelf %f   com :%f, %f\n",tempFSM->LeftFoot->Pos[2],target_com_z,target_com_z2);
           printf("SSP RIGHT %.3f,  %.3f,  %.3f,  %.3f,  %.3f,  %.3f,  %.3f\n",targetX,targetY,tempFSM->DSPRecentTask[0].Right[2],targetYaw,targetRoll,targetPitch,target_com_z);
        }

        if((tempFSM->walking_mode == NORMAL_WALKING)||(tempFSM->walking_mode == GOAL_WALKING))
        {
//            tempFSM->CompData->AddLeftFootFifth(2,-0.004,SSPTime-0.2);
//            tempFSM->CompData->AddLeftFootFifth(2,0,0.6);
              //---normal walking
            printf("SSPSTATEFORWARD normal walking\n");

            targetZ = tempFSM->DSPRecentTask[0].Right[2];
            tempFSM->FootDownTarget_LF = tempFSM->DSPRecentTask[0].Left[2];
            tempFSM->FootDownTarget_RF = tempFSM->DSPRecentTask[0].Right[2];

            tempFSM->FootUpHeight = tempFSM->RightFoot->Pos[2] +0.08;
//            tempFSM->FootUpHeight = tempFSM->RightFoot->Pos[2]+0.0;
            tempFSM->FootUpTime = SSPTime/2.0;
            tempFSM->RightFoot->AddXFifth(targetX, SSPTime-0.1);
            tempFSM->RightFoot->AddYFifth(targetY, SSPTime-0.1);

            tempFSM->RightFoot->AddZCos(tempFSM->FootUpHeight,tempFSM->FootUpTime);//changed 0.05->0.06
            tempFSM->RightFoot->AddZCos(tempFSM->RightFoot->Pos[2]+0.0, SSPTime/2.0);
//            tempFSM->RightFoot->AddZCos(tempFSM->RightFoot->Pos[2], SSPTime/2.0-0.1);
//            tempFSM->RightFoot->AddZCos(tempFSM->RightFoot->Pos[2] - 0.03, 0.2);
//            tempFSM->RightFoot->AddZCos(tempFSM->RightFoot->Pos[2], 0.5);

            tempFSM->RightFoot->AddYawFifth(targetYaw, SSPTime);
            tempFSM->_time_diff = 0.0f;

//            tempFSM->_temp_rz = targetZ;
//            tempFSM->_temp_rx = targetX;

        }else if(tempFSM->walking_mode == TERRAIN_WALKING|| tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP)
        {
//            tempFSM->CompData->AddLeftFootFifth(2,-0.008,SSPTime-0.01);
//            tempFSM->CompData->AddLeftFootFifth(2,0,0.25);
            if(tempFSM->IsUpward == OKIE)
            {
                printf("TERRAIN TASK SSPSTATEFORWARD UP WALKING \n");
               tempFSM->RightFoot->AddXCos(SSPTime*4.0f/10.0f);
               tempFSM->RightFoot->AddXCos(targetX, SSPTime*4.0f/10.0f);

               //tempFSM->RightFoot->AddYCos(targetY, SSPTime);
               tempFSM->RightFoot->AddYCos(SSPTime*4.0f/10.0f);
               tempFSM->RightFoot->AddYCos(targetY, SSPTime*4.5f/10.0f);

               targetZ = tempFSM->DSPRecentTask[0].Right[2];
               tempFSM->FootUpHeight = targetZ + 0.11f;
               tempFSM->FootUpTime = SSPTime*4.0/10.0f;
               tempFSM->RightFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
               tempFSM->RightFoot->AddZCos(SSPTime*3.5f/10.0f);
               if(tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP || tempFSM->walking_mode == TERRAIN_WALKING)
                    tempFSM->RightFoot->AddZCos(targetZ+0.002, SSPTime*2.5f/10.0f);
               else
                    tempFSM->RightFoot->AddZCos(targetZ, SSPTime*2.5f/10.0f);
               tempFSM->CompData->AddJointsFifth(JRHY,SSPTime*0.1);
               tempFSM->CompData->AddJointsFifth(JRHY,-10,SSPTime*0.4);
               tempFSM->CompData->AddJointsFifth(JRHY,  0,SSPTime*0.4);

               tempFSM->RightFoot->AddYawCos(SSPTime*0.5);
               tempFSM->RightFoot->AddYawCos(targetYaw, SSPTime*0.4);

               tempFSM->RightFoot->AddRollCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddRollCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddRollCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddRollCos(-8.0f,SSPTime*0.35f);
//               tempFSM->RightFoot->AddRollCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddRollCos(targetRoll,SSPTime*0.15f);

               tempFSM->RightFoot->AddPitchCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddPitchCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddPitchCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

                tempFSM->_temp_rz = targetZ;
                tempFSM->_temp_rx = targetX;

                tempFSM->_time_diff = 0.0f;
            }else
            {
              printf("TERRAIN TASK SSPSTATEFORWARD DOWN WALKING \n");
               tempFSM->RightFoot->AddXCos(SSPTime*0.9/4.f);
               tempFSM->RightFoot->AddXCos(targetX, SSPTime*1.6f/4.0f);

               tempFSM->RightFoot->AddYCos(SSPTime*0.9/4.f);
               tempFSM->RightFoot->AddYCos(targetY,SSPTime*1.6f/4.0f);

               targetZ = tempFSM->DSPRecentTask[0].Right[2];

//               tempFSM->FootUpHeight = tempFSM->RightFoot->Pos[2]+0.1;

               tempFSM->FootUpHeight = tempFSM->_temp_rz + 0.105f;

               tempFSM->FootUpTime = SSPTime*0.9/4.f;
               tempFSM->RightFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
               tempFSM->RightFoot->AddZCos(SSPTime*1.6/4.0);
               if(tempFSM->walking_mode == TERRAIN_WALKING_ONE_STEP || tempFSM->walking_mode == TERRAIN_WALKING)
                    tempFSM->RightFoot->AddZCos(targetZ+0.002, SSPTime*1.5/4.0);
               else
                    tempFSM->RightFoot->AddZCos(targetZ, SSPTime*1.5/4.0);

               tempFSM->CompData->AddJointsFifth(JRHY,SSPTime*0.1);
               tempFSM->CompData->AddJointsFifth(JRHY,-10,SSPTime*0.4);
               tempFSM->CompData->AddJointsFifth(JRHY,  0,SSPTime*0.4);

               tempFSM->RightFoot->AddYawCos(SSPTime*0.5);
               tempFSM->RightFoot->AddYawCos(targetYaw, SSPTime*0.4);

               tempFSM->RightFoot->AddRollCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddRollCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddRollCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddRollCos(-8.0f,SSPTime*0.35f);
//               tempFSM->RightFoot->AddRollCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddRollCos(targetRoll,SSPTime*0.15f);

//               tempFSM->RightFoot->AddPitchCos(SSPTime*0.1f);
//               tempFSM->RightFoot->AddPitchCos(0.0f,SSPTime*0.2f);
//               tempFSM->RightFoot->AddPitchCos(SSPTime*0.4f);
//               tempFSM->RightFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

               tempFSM->RightFoot->AddPitchCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddPitchCos(0.0f,SSPTime*0.15f);
               tempFSM->RightFoot->AddPitchCos(8.0f,SSPTime*0.15f);
               tempFSM->RightFoot->AddPitchCos(SSPTime*0.3f);
               tempFSM->RightFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

               tempFSM->_temp_rz = targetZ;
               tempFSM->_temp_rx = targetX;
               tempFSM->_time_diff = 0.0f;
            }

        }else if(tempFSM->walking_mode == LADDER_WALKING)
        {
//            tempFSM->CompData->AddLeftFootFifth(2,-0.008,SSPTime-0.01);
//            tempFSM->CompData->AddLeftFootFifth(2,0,0.25);
            if(tempFSM->IsUpward == OKIE)
            {
                printf("STAIR TASK SSPSTATEFORWARD UP WALKING \n");
               tempFSM->RightFoot->AddXCos(SSPTime*1.0f/10.0f);
               tempFSM->RightFoot->AddXCos(tempFSM->_temp_rx + tempFSM->_temp_add_x, SSPTime*1.5f/10.0f);
               tempFSM->RightFoot->AddXCos(tempFSM->_temp_rx, SSPTime*1.5f/10.0f);
               tempFSM->RightFoot->AddXCos(targetX, SSPTime*4.0f/10.0f);

               //tempFSM->RightFoot->AddYCos(targetY, SSPTime);
               tempFSM->RightFoot->AddYCos(SSPTime*4.0f/10.0f);
               tempFSM->RightFoot->AddYCos(targetY, SSPTime*4.5f/10.0f);

               targetZ = tempFSM->DSPRecentTask[0].Right[2];
               tempFSM->FootUpHeight = targetZ + 0.05f;
               tempFSM->FootUpTime = SSPTime*4.0/10.0f;
               tempFSM->RightFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
               tempFSM->RightFoot->AddZCos(SSPTime*3.5f/10.0f);
               tempFSM->RightFoot->AddZCos(targetZ, SSPTime*2.5f/10.0f);
               tempFSM->RightFoot->AddYawCos(targetYaw, SSPTime*1.5/4.0);

               tempFSM->RightFoot->AddRollCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddRollCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddRollCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddRollCos(targetRoll,SSPTime*0.2f);

               tempFSM->RightFoot->AddPitchCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddPitchCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddPitchCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

                tempFSM->_temp_rz = targetZ;
                tempFSM->_temp_rx = targetX;

                tempFSM->_time_diff = 0.0f;
            }else
            {
              printf("STAIR TASK SSPSTATEFORWARD DOWN WALKING \n");
               tempFSM->RightFoot->AddXCos(SSPTime*0.7/4.f);
               tempFSM->RightFoot->AddXCos(targetX, SSPTime*1.8f/4.0f);

               tempFSM->RightFoot->AddYCos(SSPTime*0.7/4.f);
               tempFSM->RightFoot->AddYCos(targetY,SSPTime*1.8f/4.0f);

               targetZ = tempFSM->DSPRecentTask[0].Right[2];

//               tempFSM->FootUpHeight = tempFSM->RightFoot->Pos[2]+0.1;

               tempFSM->FootUpHeight = tempFSM->_temp_rz + 0.08f;

               tempFSM->FootUpTime = SSPTime*0.7/4.f;
               tempFSM->RightFoot->AddZCos(tempFSM->FootUpHeight, tempFSM->FootUpTime);
               tempFSM->RightFoot->AddZCos(SSPTime*1.8/4.0);
               tempFSM->RightFoot->AddZCos(targetZ, SSPTime*1.5/4.0);
               tempFSM->RightFoot->AddYawCos(targetYaw, SSPTime*1.5/4.0);

               tempFSM->RightFoot->AddRollCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddRollCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddRollCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddRollCos(targetRoll,SSPTime*0.2f);

               tempFSM->RightFoot->AddPitchCos(SSPTime*0.1f);
               tempFSM->RightFoot->AddPitchCos(0.0f,SSPTime*0.2f);
               tempFSM->RightFoot->AddPitchCos(SSPTime*0.4f);
               tempFSM->RightFoot->AddPitchCos(targetPitch,SSPTime*0.2f);

               tempFSM->_temp_rz = targetZ;
               tempFSM->_temp_rx = targetX;
               tempFSM->_time_diff = 0.0f;
            }

        }


        //AddComp
        tempFSM->CompData->AddJointsFifth(JLAR,0.07,0.2);
        tempFSM->CompData->AddJointsFifth(JLAR,SSPTime-0.15);
        tempFSM->CompData->AddJointsFifth(JLAR,0.0,0.2);

        tempFSM->CompData->AddJointsFifth(JLAP,-1.5,0.1);
        tempFSM->CompData->AddJointsFifth(JLAP,SSPTime-0.3);
        tempFSM->CompData->AddJointsFifth(JLAP,0.0,0.1);

        tempFSM->CompData->AddJointsFifth(JLHR,-0.2,0.2);
        tempFSM->CompData->AddJointsFifth(JLHR,SSPTime-0.1);
        tempFSM->CompData->AddJointsFifth(JLHR,0,0.2);



        //--------------------comz
        tempFSM->CompData->AddCOMFifth(2,target_com_z,SSPTime*0.5f,0.0f,0.0f);
        tempFSM->CompData->AddCOMFifth(2,target_com_z2,SSPTime*0.5f,0.0f,0.0f);

        if(tempFSM->_normal_walking_flag == false){
            if(tempFSM->_foot_height_diff == true){
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height,SSPTime*0.5f,0.0,0.0);
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_block_height + (target_com_z2 -target_com_z),SSPTime*0.5f,0.0,0.0);
            }else{
                tempFSM->CompData->AddCOMFifth(1,tempFSM->_alpha*0.0f,SSPTime,0.0,0.0);
            }
        }else{
            tempFSM->CompData->AddCOMFifth(1,0.0f,SSPTime,0.0,0.0);
        }


//        printf("ssp time_diff %f \n",SSPTime);

    }

}
void SSPStateForward::ExecuteDoingState(){
    pWFSM tempFSM = CAST_WFSM(parentFSM);
    tempFSM->CurrentITime = ITimer;

    tempFSM->LeftFoot->UpdateData();
    tempFSM->RightFoot->UpdateData();

    tempFSM->ZMPPos->UpdateData();
    tempFSM->CompData->UpdateData();

    //cout << ".";
    ITimer += fsmDEL_T;
    if(ITimer >= SSPTime)
        finishFlag = true;
}
void SSPStateForward::ExecuteEndState(){
    cout << ">> this is ExecuteEndState() : SSPForwardState..." << endl;
}
// ============================================================================================================


// ============================================================================================================
WalkingFSM::WalkingFSM(){
    DSPStateInit	*init		= new DSPStateInit(this);
    DSPState		*normal		= new DSPState(this);
    DSPStateFinal	*final		= new DSPStateFinal(this);
    SSPStateForward	*sspFwd		= new SSPStateForward(this);

    FootUpTime = 0.35;
    FootUpHeight = 0.;
    TIME_DSP = 0.2;
    TIME_SSP = 0.8;
    TimeRatio = 1.;
    TIME_SSP_Ratio = 1.;

    _alpha = 6.0f/15.0f;
    _block_height = 0.15f;
    _time_diff = 0.0f;


    _temp_add_x = 0.07f;
    _normal_walking_flag = true;
    targetStates[STATE_DSP_INIT]	= init;
    targetStates[STATE_DSP]			= normal;
    targetStates[STATE_DSP_FINAL]	= final;
    targetStates[STATE_SSP_FORWARD] = sspFwd;


    RightFoot = new FootClass(fsmDEL_T);
    LeftFoot = new FootClass(fsmDEL_T);
    ZMPPos = new ZMPClass(fsmDEL_T);

    CompData = new AddCompClass(fsmDEL_T);

    PrevFoot = FOOT_SAME;
    IsLastSSP = false;

    WindowSize = 600;
    LeftInfos.clear();
    RightInfos.clear();
    ZMPInfos.clear();

    AddJointInfos.clear();
    AddRightFootInfos.clear();
    AddLeftFootInfos.clear();

    //-------------com
    AddComInfos.clear();

    //-------------virtual com
    VirtualComInfos.clear();
    VirtualComVelInfos.clear();
    VirtualComAccInfos.clear();

    //-------------foot vel
    LeftInfosVel.clear();
    RightInfosVel.clear();


    FootUpInfos.clear();
    StateInfos.clear();
    STimeInfos.clear();
    ITimeInfos.clear();

    DSPScheduler.clear();
    DSPRecentTask.clear();
}


void WalkingFSM::Update(){
    RBAFSM::Update();
    DataUpdate();
}

void WalkingFSM::DataUpdate(){

    doubles tempR = RightFoot->Pos;
    tempR.push_back(RightFoot->Yaw);
    tempR.push_back(RightFoot->Roll);
    tempR.push_back(RightFoot->Pitch);

    RightInfos.push_back(tempR);

    doubles tempL = LeftFoot->Pos;
    tempL.push_back(LeftFoot->Yaw);
    tempL.push_back(LeftFoot->Roll);
    tempL.push_back(LeftFoot->Pitch);

    LeftInfos.push_back(tempL);

    ZMPInfos.push_back(ZMPPos->ZMP);

    AddJointInfos.push_back(CompData->AddJoint);
    AddRightFootInfos.push_back(CompData->AddRightFoot);
    AddLeftFootInfos.push_back(CompData->AddLeftFoot);

    //-------------add com
    AddComInfos.push_back(CompData->AddCOM);
    VirtualComInfos.push_back(CompData->AddCOM);
    VirtualComVelInfos.push_back(CompData->AddCOMVel);
    VirtualComAccInfos.push_back(CompData->AddCOMAcc);

    //---foot vel trajectory
    LeftInfosVel.push_back(LeftFoot->Vel);
    RightInfosVel.push_back(RightFoot->Vel);


    ints tempState;
    tempState.clear();
    tempState.push_back(CurrentState);
    tempState.push_back(IsUpward);

    doubles tempFootUp;
    tempFootUp.clear();
    tempFootUp.push_back(FootUpHeight);
    tempFootUp.push_back(FootUpTime);
    tempFootUp.push_back(FootDownTarget_RF);
    tempFootUp.push_back(FootDownTarget_LF);
    tempFootUp.push_back(SideStepLength);

    FootUpInfos.push_back(tempFootUp);
    StateInfos.push_back(tempState);
    STimeInfos.push_back(CurrentSTime);
    ITimeInfos.push_back(CurrentITime);

    if(RightInfos.size() > WindowSize)
        RightInfos.pop_front();
    if(LeftInfos.size() > WindowSize)
        LeftInfos.pop_front();
    if(ZMPInfos.size() > WindowSize)
        ZMPInfos.pop_front();

    if(AddJointInfos.size() > WindowSize)
        AddJointInfos.pop_front();
    if(AddRightFootInfos.size() > WindowSize)
        AddRightFootInfos.pop_front();
    if(AddLeftFootInfos.size() > WindowSize)
        AddLeftFootInfos.pop_front();


    //------------add com
    if(AddComInfos.size() > WindowSize)
        AddComInfos.pop_front();

    //add virtual com
    if(VirtualComInfos.size() > WindowSize)
        VirtualComInfos.pop_front();

    if(VirtualComVelInfos.size() > WindowSize)
        VirtualComVelInfos.pop_front();

    if(VirtualComAccInfos.size() > WindowSize)
        VirtualComAccInfos.pop_front();

    if(LeftInfosVel.size() > WindowSize)
       LeftInfosVel.pop_front();

    if(RightInfosVel.size() > WindowSize)
       RightInfosVel.pop_front();



    if(FootUpInfos.size() > WindowSize)
        FootUpInfos.pop_front();
    if(StateInfos.size() > WindowSize)
        StateInfos.pop_front();
    if(STimeInfos.size() > WindowSize)
        STimeInfos.pop_front();
    if(ITimeInfos.size() > WindowSize)
        ITimeInfos.pop_front();
}

void WalkingFSM::CheckStateTransition(){
    if(m_pCurrentState == NULL){
        if(DSPScheduler.size() > 2){
            MoveSchedulerToRecent(3);
        }else if(DSPScheduler.size() > 1){
            MoveSchedulerToRecent(2);
        }else
            return;

        StateTransition(*targetStates[STATE_DSP_INIT]);
        CAST_DSPINIT(targetStates[STATE_DSP_INIT])->DSPInfo = DSPRecentTask[RECENT_CURRENT];
    }else{
        switch(CAST_WSTATE(m_pCurrentState)->GetState()){
        case STATE_DSP_INIT:
            if(CAST_WSTATE(m_pCurrentState)->IsFinished()){

                    StateTransition(*targetStates[STATE_SSP_FORWARD]);

                if(DSPRecentTask.size() == 1){
                    IsLastSSP = true;
                }else{
                    IsLastSSP = false;
                }
            }

//            SUPPORT_STATE = DOUBLE;
            break;
        case STATE_DSP:
            if(CAST_WSTATE(m_pCurrentState)->IsFinished()){

                    StateTransition(*targetStates[STATE_SSP_FORWARD]);

                if(DSPRecentTask.size() == 1){
                    IsLastSSP = true;
                }else{
                    IsLastSSP = false;
                }
            }

//            SUPPORT_STATE = DOUBLE;
            break;
        case STATE_DSP_FINAL:
            if(CAST_WSTATE(m_pCurrentState)->IsFinished()){
                m_pCurrentState = NULL;
            }

//            SUPPORT_STATE = DOUBLE;
            break;
        case STATE_SSP_FORWARD:
            if(CAST_WSTATE(m_pCurrentState)->IsFinished()){
                if(DSPScheduler.size() > 0){
                    MoveSchedulerToRecent(1);
                    StateTransition(*targetStates[STATE_DSP]);
                }else{
                    DSPRecentTask.pop_front();
                    if(DSPRecentTask.size() == 0){
                        StateTransition(*targetStates[STATE_DSP_FINAL]);
                    }else{
                        StateTransition(*targetStates[STATE_DSP]);
                    }
                }
            }

//            SUPPORT_STATE = SINGLE;
            break;

        }
    }
}

void WalkingFSM::UpdateRecentTask(DSPTask &task){
    DSPRecentTask.push_back(task);
    if(DSPRecentTask.size() >= 4)
        DSPRecentTask.pop_front();
}

void WalkingFSM::MoveSchedulerToRecent(int n){
    for(int i=0; i<n; i++){
        UpdateRecentTask(DSPScheduler[0]);
        DSPScheduler.pop_front();
    }
}
void WalkingFSM::SetInitialFootPlace(DSPTask task){
    LeftFoot->SetInitialPosition(task.Left[0], task.Left[1], task.Left[2], task.LYaw, task.LRoll, task.LPitch);
    RightFoot->SetInitialPosition(task.Right[0], task.Right[1], task.Right[2], task.RYaw, task.RRoll, task.RPitch);
    ZMPPos->SetInitialZMP(task.ZMP[0], task.ZMP[1]);
    CompData->SetInitialAddJoint();
    //CompData->SetInitialCOM(task.COMz[0]);
}
void WalkingFSM::SetInitialFootPlace2(DSPTask task){
    LeftFoot->SetInitialPosition(task.Left[0], task.Left[1], task.Left[2], task.LYaw, task.LRoll, task.LPitch);
    RightFoot->SetInitialPosition(task.Right[0], task.Right[1], task.Right[2], task.RYaw, task.RRoll, task.RPitch);
    ZMPPos->SetInitialZMP(task.ZMP[0], task.ZMP[1]);
    CompData->SetInitialCOM(task.COMz[0]);
    CompData->SetInitialAddJoint();
}
int	WalkingFSM::FootSelector(){
    double errTh = 0.0003;
    //double zeroTh = 0.002;
    double left[4], right[4];
    double lx,ly,rx,ry,lth,rth;
    double com_z;
    left[0] = lx = LeftFoot->Pos[0];
    left[1] = ly = LeftFoot->Pos[1];
    left[2] = lth = LeftFoot->Pos[2];
    left[3] = lth = LeftFoot->Yaw;
    right[0] = rx = RightFoot->Pos[0];
    right[1] = ry = RightFoot->Pos[1];
    right[2] = rth = RightFoot->Pos[2];
    right[3] = rth = RightFoot->Yaw;
    com_z = CompData->AddCOM[2];

    printf("Foot selector com_z: %f \n",com_z);

    HoldTime = DSPRecentTask[0].HTime;

    double lx_n,lx_nn,ly_n,ly_nn;
    double rx_n,rx_nn,ry_n,ry_nn;
    double theta = (lth + rth)/2.;

    lx_n = lx-(rx+lx)/2.;
    ly_n = ly-(ry+ly)/2.;
    rx_n = rx-(rx+lx)/2.;
    ry_n = ry-(ry+ly)/2.;

    lx_nn =  lx_n*cos(theta*D2Rf) + ly_n*sin(theta*D2Rf);
    ly_nn = -lx_n*sin(theta*D2Rf) + ly_n*cos(theta*D2Rf);

    rx_nn =  rx_n*cos(theta*D2Rf) + ry_n*sin(theta*D2Rf);
    ry_nn = -rx_n*sin(theta*D2Rf) + ry_n*cos(theta*D2Rf);

    double lx2,ly2,rx2,ry2,lth2,rth2;
    lx2 = DSPRecentTask[0].Left[0];
    ly2 = DSPRecentTask[0].Left[1];
    lth2 = DSPRecentTask[0].LYaw;
    rx2 = DSPRecentTask[0].Right[0];
    ry2 = DSPRecentTask[0].Right[1];
    rth2 = DSPRecentTask[0].RYaw;

    double lx_n2,lx_nn2,ly_n2,ly_nn2;
    double rx_n2,rx_nn2,ry_n2,ry_nn2;
    double theta2 = (lth2 + rth2)/2.;

    lx_n2 = lx2-(rx2+lx2)/2.;
    ly_n2 = ly2-(ry2+ly2)/2.;

    rx_n2 = rx2-(rx2+lx2)/2.;
    ry_n2 = ry2-(ry2+ly2)/2.;

    lx_nn2 =  lx_n2*cos(theta2*D2Rf) + ly_n2*sin(theta2*D2Rf);
    ly_nn2 = -lx_n2*sin(theta2*D2Rf) + ly_n2*cos(theta2*D2Rf);

    rx_nn2 =  rx_n2*cos(theta2*D2Rf) + ry_n2*sin(theta2*D2Rf);
    ry_nn2 = -rx_n2*sin(theta2*D2Rf) + ry_n2*cos(theta2*D2Rf);

    //printf("lx=%f,lx2=%f,ly=%f,ly2=%f,rx=%f,rx2=%f,ry=%f,ry2=%f,lth=%f,lth2=%f,rth=%f,rth2=%f \n",lx,lx2,ly,ly2,rx,rx2,ry,ry2,lth,lth2,rth,rth2);
    //printf("lx=%f,lx2=%f,ly=%f,ly2=%f,rx=%f,rx2=%f,ry=%f,ry2=%f,theta=%f,theta2=%f \n",lx_nn,lx_nn2,ly_nn,ly_nn2,rx_nn,rx_nn2,ry_nn,ry_nn2,theta,theta2);

//    printf("foot selector rx: %f   ry: %f  rz: %f  ---  lx: %f  ly: %f  lz: %f \n",right[0],right[1],right[2],left[0],left[1],left[2]);
//    printf("foot selector rx: %f   ry: %f  rz: %f  ---  lx: %f  ly: %f  lz: %f \n",DSPRecentTask[0].Right[0],DSPRecentTask[0].Right[1],DSPRecentTask[0].Right[2],DSPRecentTask[0].Left[0],DSPRecentTask[0].Left[1],DSPRecentTask[0].Left[2]);
    // in the same position
    if(fabs(left[0]-DSPRecentTask[0].Left[0]) < errTh  &&  fabs(left[1]-DSPRecentTask[0].Left[1]) < errTh && fabs(left[3]-DSPRecentTask[0].LYaw) < errTh  &&
    fabs(right[0]-DSPRecentTask[0].Right[0]) < errTh  &&  fabs(right[1]-DSPRecentTask[0].Right[1]) < errTh && fabs(right[3]-DSPRecentTask[0].RYaw) < errTh)
    {

//        com_offset = 0.0f;
        return FOOT_SAME;

    }
    else if(fabs(left[0]-DSPRecentTask[0].Left[0])  < errTh   &&  fabs(left[1]-DSPRecentTask[0].Left[1])   < errTh  && fabs(left[3]-DSPRecentTask[0].LYaw)  < errTh) {

        IsZeroStep = false;
        StepLength = (rx_nn2-rx_nn);
        SideStepLength = (ry_nn2-ry_nn)*2.;

        if(rth!=rth2) SideStepLength = 0.;

        if((right[2] > DSPRecentTask[0].Right[2])&& isTerrain == TERRAIN)// && fabs(right[2] - DSPRecentTask[0].Right[2]) > 0.02f)
        {
            printf("foot selector -> R Walking Down!!\n");

//            printf("Foot selector right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);

            IsUpward = NOPE; //dw
            _normal_walking_flag = false ;

            if(fabs(DSPRecentTask[0].Right[2] - left[2] ) > 0.05f)
            {
                _foot_height_diff = true;
//                printf("foot selector -> Right and left foot Height diff \n");

            }else{
                _foot_height_diff = false;
//                printf("foot selector -> R and left foot Height not diff \n ");
            }

            _block_height = -0.15f;

            //-----------15.02.11
            if(SUPPORT_STATE == DOUBLE)
            {
//                _block_height = right[2] - left[2] + com_z;// + DSPRecentTask[0].COMz[0];
//                _block_height =  com_z - left[2];
                _block_height =  DSPRecentTask[0].COMz[2] - left[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> DSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[2]);
                printf("----------------blcok height: %f \n",_block_height);

            }else
            {
//                _block_height =  DSPRecentTask[0].COMz[0] - com_z;//right[2] - left[2] + DSPRecentTask[0].COMz[0];
                _block_height =  DSPRecentTask[0].COMz[0] - left[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> SSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[0]);
                printf("----------------blcok height: %f \n",_block_height);
            }



        }else if((right[2] < DSPRecentTask[0].Right[2])&& isTerrain == TERRAIN)// && fabs(right[2] - DSPRecentTask[0].Right[2]) > 0.02f)
        {
            printf("foot selector -> R Walking Up!!\n");

//            printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);

            IsUpward = OKIE;
            _normal_walking_flag = false ;

            if(fabs(left[2] - DSPRecentTask[0].Right[2]) > 0.05f)
            {
                _foot_height_diff = true;
//                printf("foot selector -> R Height diff \n");
            }else
            {
                _foot_height_diff = false;
//                printf("foot selector -> R Height not diff \n");
            }

            _block_height = 0.15f;//walkingFSM_shared_data->terrain_variable[3];

            //-----------15.02.11
            if(SUPPORT_STATE == DOUBLE)
            {
//                _block_height = right[2] - left[2] + com_z;// + DSPRecentTask[0].COMz[0];
//                _block_height =  com_z - left[2];
                _block_height =  DSPRecentTask[0].COMz[2] - left[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> DSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[2]);
                printf("----------------blcok height: %f \n",_block_height);

            }else
            {
//                _block_height =  DSPRecentTask[0].COMz[0] - com_z;//right[2] - left[2] + DSPRecentTask[0].COMz[0];
                _block_height =  DSPRecentTask[0].COMz[0] - left[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> SSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[0]);
                printf("----------------blcok height: %f \n",_block_height);
            }




        }else
        {
            printf("foot selector -> R Height not diff & Normal Walking!!\n");
            _block_height = 0.0f;
            _normal_walking_flag = true ;
            _foot_height_diff = false;
        }

        if(StepLength>0)            _temp_add_x = -0.07f;
        else if(StepLength<0)       _temp_add_x = 0.07f;
        else                        _temp_add_x = 0.0f;


//        printf("_temp_add_x = %f\n",_temp_add_x);
        if(fabs(left[2] - right[2]) > 0.05f)
        {
            _dsp_foot_height_diff = true;
//            printf("foot selector-> R _dsp_foot_height_diff \n");
        }else
        {
            _dsp_foot_height_diff = false;
//            printf("foot selector-> R _dsp_foot_height_same \n");
        }

 //       _block_height = StepHeight = (DSPRecentTask[0].Right[2] -left[2]);

//        com_z2 = DSPRecentTask[0].COMz[0];
//        com_offset = (com_z2-com_z) - ;

//        _com_offset = DSPRecentTask[0].COMz[0];

//        _alpha = 0.05f/_block_height;// _com_offset/_block_height;
//        if(_block_height <0.001f && _block_height>-0.001f)  _alpha = 0.0f;

        TIME_SSP_Ratio = 1 + fabs(rth-rth2)*0.15/90 + fabs(rx_nn2-rx_nn)*0.1/0.2 + fabs(ry_nn2-ry_nn)*0.1/0.2;
        printf("IsUpward = %d\n",IsUpward);
        return FOOT_RIGHT;
    }
    else if(fabs(right[0]-DSPRecentTask[0].Right[0]) < errTh   &&  fabs(right[1]-DSPRecentTask[0].Right[1]) < errTh   &&  fabs(right[3]-DSPRecentTask[0].RYaw) < errTh){

        IsZeroStep = false;
        StepLength = (lx_nn2-lx_nn);
        SideStepLength = (ly_nn2-ly_nn)*2.;
        if(lth!=lth2) SideStepLength = 0.;


        if((left[2] > DSPRecentTask[0].Left[2])&& isTerrain == TERRAIN)// && fabs(left[2] - DSPRecentTask[0].Left[2]) > 0.02f)
        {
            printf("foot selector ->L Walking Down!!\n");

//            printf("----------------left: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);

            IsUpward = NOPE;
            _normal_walking_flag = false ;

            if(fabs(right[2] - DSPRecentTask[0].Left[2]) > 0.05f)
            {
//                printf("foot selector ->L Height diff \n");
                _foot_height_diff = true;
            }else
            {
//                printf("foot selector ->L Height not diff \n");
                _foot_height_diff = false;
            }

            _block_height = -0.15f;

            //-----------15.02.11
            if(SUPPORT_STATE == DOUBLE)
            {
//                _block_height = left[2] - right[2] + com_z;// + DSPRecentTask[0].COMz[0];
//                _block_height = com_z - right[2];
                _block_height = DSPRecentTask[0].COMz[2] - right[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> DSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[2]);
                printf("----------------blcok height: %f \n",_block_height);

            }else
            {
//                _block_height =  DSPRecentTask[0].COMz[0] - com_z;//right[2] - left[2] + DSPRecentTask[0].COMz[0];
                _block_height =  DSPRecentTask[0].COMz[0] - right[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> SSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[0]);
                printf("----------------blcok height: %f \n",_block_height);
            }



        }
        else if((left[2] < DSPRecentTask[0].Left[2])&& isTerrain == TERRAIN)// && fabs(left[2] - DSPRecentTask[0].Left[2]) > 0.02f)
        {
            printf("foot selector ->L Walking Up!!\n");


            IsUpward = OKIE;
            _normal_walking_flag = false ;


            if( fabs(right[2] - DSPRecentTask[0].Left[2])  > 0.05f)
            {
//                printf("foot selector ->L Height diff \n");
                _foot_height_diff = true;
            }else
            {
//                printf("foot selector ->L Height not diff \n");
                _foot_height_diff = false;
            }

            _block_height = 0.15;

            //-----------15.02.11
            if(SUPPORT_STATE == DOUBLE)
            {
//                _block_height = left[2] - right[2] + com_z;// + DSPRecentTask[0].COMz[0];
                 _block_height = DSPRecentTask[0].COMz[2]- right[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> DSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[0]);
                printf("----------------blcok height: %f \n",_block_height);


            }else
            {
//                _block_height =  DSPRecentTask[0].COMz[0] - com_z;//right[2] - left[2] + DSPRecentTask[0].COMz[0];
                _block_height =  DSPRecentTask[0].COMz[0] - right[2];
                printf(">>>>>>>>>>>>>>>>>>>>>> SSP STATE <<<<<<<<<<<<<<<<<<<<\n");
                printf("----------------left: %f  DSPrecent: %f \n",left[2],DSPRecentTask[0].Left[2]);
                printf("----------------right: %f  DSPrecent: %f \n",right[2],DSPRecentTask[0].Right[2]);
                printf("----------------COMz : %f  DSPrecent COMz : %f\n",com_z,DSPRecentTask[0].COMz[0]);
                printf("----------------blcok height: %f \n",_block_height);

            }

        }else
        {
            printf("foot selector ->L Height not diff & Normal Walking!! \n");

            _block_height = 0.0f;
            _foot_height_diff = false;
            _normal_walking_flag = true ;
        }


        if(fabs(left[2] - right[2]) > 0.05f)
        {
            _dsp_foot_height_diff = true;
//            printf("foot selector -> L _dsp_foot_height_diff \n");
        }else
        {
//            printf("foot selector -> L _dsp_foot_heightf_same \n");
            _dsp_foot_height_diff = false;
        }

        if(StepLength>0)            _temp_add_x = -0.07f;
        else if(StepLength<0)       _temp_add_x = 0.07f;
        else                        _temp_add_x = 0.0f;

//        printf("_temp_add_x = %f\n",_temp_add_x);
//        _block_height = StepHeight = (DSPRecentTask[0].Left[2] -right[2]);

//        com_z2 = DSPRecentTask[0].COMz[1];
//        com_offset = com_z2-com_z;

//        _com_offset = DSPRecentTask[0].COMz[0];

//        _alpha = 0.05f/_block_height;
//        if(_block_height <0.001f && _block_height>-0.001f)  _alpha = 0.0f;

         TIME_SSP_Ratio = 1 + fabs(lth-lth2)*0.15/90+ fabs(lx_nn2-lx_nn)*0.1/0.2 + fabs(ly_nn2-ly_nn)*0.1/0.2;
         printf("IsUpward = %d\n",IsUpward);
        return FOOT_LEFT;
    }
    else{
        printf("foot selector : FOOT_ERROR\n");
        printf("Prev LF(%f,%f),(%f),RF(%f,%f),(%f)\n",left[0],left[1],left[3],right[0],right[1],right[3]);
        printf("Next LF(%f,%f),(%f),RF(%f,%f),(%f)\n",DSPRecentTask[0].Left[0],DSPRecentTask[0].Left[1],DSPRecentTask[0].LYaw,DSPRecentTask[0].Right[0],DSPRecentTask[0].Right[1],DSPRecentTask[0].RYaw);
        return FOOT_ERROR;
    }


}

int WalkingFSM::FootSelectorNext(){
    double errTh = 0.0001;
    double left[2], right[2];
    left[0] = DSPRecentTask[1].Left[0];
    left[1] = DSPRecentTask[1].Left[1];
    right[0] = DSPRecentTask[1].Right[0];
    right[1] = DSPRecentTask[1].Right[1];

    // in the same position
    if(fabs(left[0]-DSPRecentTask[0].Left[0]) < errTh  &&  fabs(left[1]-DSPRecentTask[0].Left[1]) < errTh &&
        fabs(right[0]-DSPRecentTask[0].Right[0]) < errTh  &&  fabs(right[1]-DSPRecentTask[0].Right[1]) < errTh)
        return FOOT_SAME;

    if(fabs(left[0]-DSPRecentTask[0].Left[0]) < errTh  &&  fabs(left[1]-DSPRecentTask[0].Left[1]) < errTh)
        return FOOT_RIGHT;
    if(fabs(right[0]-DSPRecentTask[0].Right[0]) < errTh  &&  fabs(right[1]-DSPRecentTask[0].Right[1]) < errTh)
        return FOOT_LEFT;
    else
        return FOOT_ERROR;
}

void WalkingFSM::ClearAll(){
    DSPRecentTask.clear();
    DSPScheduler.clear();
    m_pCurrentState = NULL;
    m_pNewState = NULL;
}



void WalkingFSM::LL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3])
{
    double Ori[3][3];
    double offset[3];

    //---------------foot edge configuration

    offset[0] = -0.12f;
    offset[1] = 0.00f;
    offset[2] = 0.0f;//-kine_drc_hubo.L_FOOT;//-0.1f;



    //------------------------------------Foot edge pos & ori

//    Ori[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    Ori[0][1] = sin(t1)*cos(t2);
//    Ori[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);

//    Ori[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    Ori[1][1] = cos(t1)*cos(t2);
//    Ori[1][2] = -sin(t1)*sin(t3)+cos(t1)*sin(t2)*cos(t3);

//    Ori[2][0] = -cos(t2)*sin(t3);
//    Ori[2][1] = -sin(t2);
//    Ori[2][2] = cos(t2)*cos(t3);

    Ori[0][0] = cos(t1*D2Rf)*cos(t3*D2Rf)-sin(t3*D2Rf)*sin(t1*D2Rf)*sin(t2*D2Rf);
    Ori[0][1] = -sin(t1*D2Rf)*cos(t2*D2Rf);
    Ori[0][2] = cos(t1*D2Rf)*sin(t3*D2Rf)+cos(t3*D2Rf)*sin(t1*D2Rf);

    Ori[1][0] = sin(t1*D2Rf)*cos(t3*D2Rf)+sin(t3*D2Rf)*cos(t1*D2Rf)*sin(t2*D2Rf);
    Ori[1][1] = cos(t1*D2Rf)*cos(t2*D2Rf);
    Ori[1][2] = sin(t1*D2Rf)*sin(t3*D2Rf)-cos(t1*D2Rf)*sin(t2*D2Rf)*cos(t3*D2Rf);

    Ori[2][0] = -cos(t2*D2Rf)*sin(t3*D2Rf);
    Ori[2][1] = sin(t2*D2Rf);
    Ori[2][2] = cos(t2*D2Rf)*cos(t3*D2Rf);


    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    printf("%f  %f  %f  \n", Ori[0][0],Ori[0][1],Ori[0][2] );
    printf("%f  %f  %f  \n", Ori[1][0],Ori[1][1],Ori[1][2] );
    printf("%f  %f  %f  \n", Ori[2][0],Ori[2][1],Ori[2][2] );


    //----------------Foot Pos & ori
    Heel_pos[0] = Fpos_x + (Ori[0][0]*offset[0] + Ori[0][1]*offset[1] + Ori[0][2]*offset[2]);
    Heel_pos[1] = Fpos_y + (Ori[1][0]*offset[0] + Ori[1][1]*offset[1] + Ori[1][2]*offset[2]);
    Heel_pos[2] = Fpos_z + (Ori[2][0]*offset[0] + Ori[2][1]*offset[1] + Ori[2][2]*offset[2]);
    printf("-----------------Left Heel pos: %f,%f,%f -----------------\n",Heel_pos[0],Heel_pos[1],Heel_pos[2]);
//    Rodrigues(axis7,0.0,temp_FOOT);

//    mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);


//        printf("================= leg collision check================\n");
//        printf("%f   %f   %f \n",WBIK_Q2[0],WBIK_Q2[1],WBIK_Q2[2]);
//        printf("================= leg collision check================\n");
//        printf("RHY pos: %f    %f    %f \n",P_RHY[0],P_RHY[1],P_RHY[2]);
//        printf("RHR pos: %f    %f    %f \n",P_RHR[0],P_RHR[1],P_RHR[2]);
//        printf("RHP pos: %f    %f    %f \n",P_RHP[0],P_RHP[1],P_RHP[2]);
//        printf("RKN pos: %f    %f    %f \n",P_RKN[0],P_RKN[1],P_RKN[2]);
//        printf("RAP pos: %f    %f    %f \n",P_RAP[0],P_RAP[1],P_RAP[2]);
//        printf("RAR pos: %f    %f    %f \n",P_RAR[0],P_RAR[1],P_RAR[2]);



//    for(int i=0;i<4;i++)
//    {
//        for(int j=0 ; j<3; j++)
//        {
//            _LF_edge[i][j] = P_EDGE[i][j];
//            pos[i][j] = P_EDGE[i][j];
//        }
//    }




}
// --------------------------------------------------------------------------------------------- //


void WalkingFSM::Forward_LL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3])
{
    double Ori[3][3];
    double offset[3];

    //---------------foot edge configuration

    offset[0] = 0.12f;
    offset[1] = 0.00f;
    offset[2] = 0.0f;//-kine_drc_hubo.L_FOOT;//-0.1f;



    //------------------------------------Foot edge pos & ori

//    Ori[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    Ori[0][1] = sin(t1)*cos(t2);
//    Ori[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);

//    Ori[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    Ori[1][1] = cos(t1)*cos(t2);
//    Ori[1][2] = -sin(t1)*sin(t3)+cos(t1)*sin(t2)*cos(t3);

//    Ori[2][0] = -cos(t2)*sin(t3);
//    Ori[2][1] = -sin(t2);
//    Ori[2][2] = cos(t2)*cos(t3);

    Ori[0][0] = cos(t1*D2Rf)*cos(t3*D2Rf)-sin(t3*D2Rf)*sin(t1*D2Rf)*sin(t2*D2Rf);
    Ori[0][1] = -sin(t1*D2Rf)*cos(t2*D2Rf);
    Ori[0][2] = cos(t1*D2Rf)*sin(t3*D2Rf)+cos(t3*D2Rf)*sin(t1*D2Rf);

    Ori[1][0] = sin(t1*D2Rf)*cos(t3*D2Rf)+sin(t3*D2Rf)*cos(t1*D2Rf)*sin(t2*D2Rf);
    Ori[1][1] = cos(t1*D2Rf)*cos(t2*D2Rf);
    Ori[1][2] = sin(t1*D2Rf)*sin(t3*D2Rf)-cos(t1*D2Rf)*sin(t2*D2Rf)*cos(t3*D2Rf);

    Ori[2][0] = -cos(t2*D2Rf)*sin(t3*D2Rf);
    Ori[2][1] = sin(t2*D2Rf);
    Ori[2][2] = cos(t2*D2Rf)*cos(t3*D2Rf);



    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    printf("%f  %f  %f  \n", Ori[0][0],Ori[0][1],Ori[0][2] );
    printf("%f  %f  %f  \n", Ori[1][0],Ori[1][1],Ori[1][2] );
    printf("%f  %f  %f  \n", Ori[2][0],Ori[2][1],Ori[2][2] );


    //----------------Foot Pos & ori
    Heel_pos[0] = Fpos_x + (Ori[0][0]*offset[0] + Ori[0][1]*offset[1] + Ori[0][2]*offset[2]);
    Heel_pos[1] = Fpos_y + (Ori[1][0]*offset[0] + Ori[1][1]*offset[1] + Ori[1][2]*offset[2]);
    Heel_pos[2] = Fpos_z + (Ori[2][0]*offset[0] + Ori[2][1]*offset[1] + Ori[2][2]*offset[2]);
    printf("-----------------Left Heel pos: %f,%f,%f -----------------\n",Heel_pos[0],Heel_pos[1],Heel_pos[2]);
//    Rodrigues(axis7,0.0,temp_FOOT);

//    mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);


//        printf("================= leg collision check================\n");
//        printf("%f   %f   %f \n",WBIK_Q2[0],WBIK_Q2[1],WBIK_Q2[2]);
//        printf("================= leg collision check================\n");
//        printf("RHY pos: %f    %f    %f \n",P_RHY[0],P_RHY[1],P_RHY[2]);
//        printf("RHR pos: %f    %f    %f \n",P_RHR[0],P_RHR[1],P_RHR[2]);
//        printf("RHP pos: %f    %f    %f \n",P_RHP[0],P_RHP[1],P_RHP[2]);
//        printf("RKN pos: %f    %f    %f \n",P_RKN[0],P_RKN[1],P_RKN[2]);
//        printf("RAP pos: %f    %f    %f \n",P_RAP[0],P_RAP[1],P_RAP[2]);
//        printf("RAR pos: %f    %f    %f \n",P_RAR[0],P_RAR[1],P_RAR[2]);



//    for(int i=0;i<4;i++)
//    {
//        for(int j=0 ; j<3; j++)
//        {
//            _LF_edge[i][j] = P_EDGE[i][j];
//            pos[i][j] = P_EDGE[i][j];
//        }
//    }




}
// --------------------------------------------------------------------------------------------- //




//---------------------Heel
void WalkingFSM::RL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3])
{
    double Ori[3][3];
    double offset[3];

    //---------------foot edge configuration

    offset[0] = -0.12f;
    offset[1] = 0.00f;
    offset[2] = 0.0f;//-kine_drc_hubo.L_FOOT;//-0.1f;



    //------------------------------------Foot edge pos & ori

    Ori[0][0] = cos(t1*D2Rf)*cos(t3*D2Rf)-sin(t3*D2Rf)*sin(t1*D2Rf)*sin(t2*D2Rf);
    Ori[0][1] = -sin(t1*D2Rf)*cos(t2*D2Rf);
    Ori[0][2] = cos(t1*D2Rf)*sin(t3*D2Rf)+cos(t3*D2Rf)*sin(t1*D2Rf);

    Ori[1][0] = sin(t1*D2Rf)*cos(t3*D2Rf)+sin(t3*D2Rf)*cos(t1*D2Rf)*sin(t2*D2Rf);
    Ori[1][1] = cos(t1*D2Rf)*cos(t2*D2Rf);
    Ori[1][2] = sin(t1*D2Rf)*sin(t3*D2Rf)-cos(t1*D2Rf)*sin(t2*D2Rf)*cos(t3*D2Rf);

    Ori[2][0] = -cos(t2*D2Rf)*sin(t3*D2Rf);
    Ori[2][1] = sin(t2*D2Rf);
    Ori[2][2] = cos(t2*D2Rf)*cos(t3*D2Rf);



    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    printf("%f  %f  %f  \n", Ori[0][0],Ori[0][1],Ori[0][2] );
    printf("%f  %f  %f  \n", Ori[1][0],Ori[1][1],Ori[1][2] );
    printf("%f  %f  %f  \n", Ori[2][0],Ori[2][1],Ori[2][2] );


    //----------------Foot Pos & ori
    Heel_pos[0] = Fpos_x + (Ori[0][0]*offset[0] + Ori[0][1]*offset[1] + Ori[0][2]*offset[2]);
    Heel_pos[1] = Fpos_y + (Ori[1][0]*offset[0] + Ori[1][1]*offset[1] + Ori[1][2]*offset[2]);
    Heel_pos[2] = Fpos_z + (Ori[2][0]*offset[0] + Ori[2][1]*offset[1] + Ori[2][2]*offset[2]);
    printf("-----------------Right Heel pos: %f,%f,%f -----------------\n",Heel_pos[0],Heel_pos[1],Heel_pos[2]);
//    Rodrigues(axis7,0.0,temp_FOOT);

//    mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);


}



//---------------------Heel
void WalkingFSM::Forward_RL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3])
{
    double Ori[3][3];
    double offset[3];


    //---------------foot edge configuration

    offset[0] = 0.12f;
    offset[1] = 0.00f;
    offset[2] = 0.0f;//-kine_drc_hubo.L_FOOT;//-0.1f;



    //------------------------------------Foot edge pos & ori

    Ori[0][0] = cos(t1*D2Rf)*cos(t3*D2Rf)-sin(t3*D2Rf)*sin(t1*D2Rf)*sin(t2*D2Rf);
    Ori[0][1] = -sin(t1*D2Rf)*cos(t2*D2Rf);
    Ori[0][2] = cos(t1*D2Rf)*sin(t3*D2Rf)+cos(t3*D2Rf)*sin(t1*D2Rf);

    Ori[1][0] = sin(t1*D2Rf)*cos(t3*D2Rf)+sin(t3*D2Rf)*cos(t1*D2Rf)*sin(t2*D2Rf);
    Ori[1][1] = cos(t1*D2Rf)*cos(t2*D2Rf);
    Ori[1][2] = sin(t1*D2Rf)*sin(t3*D2Rf)-cos(t1*D2Rf)*sin(t2*D2Rf)*cos(t3*D2Rf);

    Ori[2][0] = -cos(t2*D2Rf)*sin(t3*D2Rf);
    Ori[2][1] = sin(t2*D2Rf);
    Ori[2][2] = cos(t2*D2Rf)*cos(t3*D2Rf);


    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    printf("%f  %f  %f  \n", Ori[0][0],Ori[0][1],Ori[0][2] );
    printf("%f  %f  %f  \n", Ori[1][0],Ori[1][1],Ori[1][2] );
    printf("%f  %f  %f  \n", Ori[2][0],Ori[2][1],Ori[2][2] );


    //----------------Foot Pos & ori
    Heel_pos[0] = Fpos_x + (Ori[0][0]*offset[0] + Ori[0][1]*offset[1] + Ori[0][2]*offset[2]);
    Heel_pos[1] = Fpos_y + (Ori[1][0]*offset[0] + Ori[1][1]*offset[1] + Ori[1][2]*offset[2]);
    Heel_pos[2] = Fpos_z + (Ori[2][0]*offset[0] + Ori[2][1]*offset[1] + Ori[2][2]*offset[2]);
    printf("-----------------Right Heel pos: %f,%f,%f -----------------\n",Heel_pos[0],Heel_pos[1],Heel_pos[2]);
//    Rodrigues(axis7,0.0,temp_FOOT);

//    mat3by3x3by3(R_RAR,temp_FOOT,R_FOOT);


}

