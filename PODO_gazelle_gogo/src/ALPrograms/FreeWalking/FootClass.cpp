//#include "FootClass.h"


////========================================================================================================
//FootClass::FootClass(double _tick){
//    Pos = doubles(3);
//    Vel = doubles(3);
//    Yaw = 0.0;
//    Pitch = 0.0;
//    Roll = 0.0;


//    //Quat = doubles(4);
//    PosHandler = TRHandlers(3);

//    PosHandler[0]	= new TrajectoryHandler(ORDER_1, _tick);
//    PosHandler[1]	= new TrajectoryHandler(ORDER_1, _tick);
//    PosHandler[2]	= new TrajectoryHandler(ORDER_1, _tick);
//    YawHandler = new TrajectoryHandler(ORDER_1, _tick);


//    //---
//    PitchHandler = new TrajectoryHandler(ORDER_1,_tick);
//    RollHandler = new TrajectoryHandler(ORDER_1,_tick);



//  //  QuatHandler		= new TrajectoryHandler(ORDER_QUAT, _tick);
//}
//void FootClass::RefreshData(){
//    Pos[0] = PosHandler[0]->GetRetValue()[0];
//    Pos[1] = PosHandler[1]->GetRetValue()[0];
//    Pos[2] = PosHandler[2]->GetRetValue()[0];

//    //---
//    Vel[0] = PosHandler[0]->GetRetValue()[1];
//    Vel[1] = PosHandler[1]->GetRetValue()[1];
//    Vel[2] = PosHandler[2]->GetRetValue()[1];
//    Yaw = YawHandler->GetRetValue()[0];

//    //---
//    Pitch = PitchHandler->GetRetValue()[0];
//    Roll = RollHandler->GetRetValue()[0];

// //   Quat = QuatHandler->GetRetValue();
//}
//void FootClass::UpdateData(){

//    Yaw = YawHandler->UpdateTrajectory()[0];
//    //--
//    Pitch = PitchHandler->UpdateTrajectory()[0];
//    Roll = RollHandler->UpdateTrajectory()[0];


//////-----------
//    doubles temppos(3);

//    for(int i=0; i<3; i++)
//    {
//        temppos = PosHandler[i]->UpdateTrajectory();
//        Pos[i] = temppos[0];
//        Vel[i] = temppos[1];//PosHandler[i]->GetRetValue()[1];//tempcom[1];
//    }
////    Pos[0]	= PosHandler[0]->UpdateTrajectory()[0];
////    Pos[1]	= PosHandler[1]->UpdateTrajectory()[0];
////    Pos[2]	= PosHandler[2]->UpdateTrajectory()[0];

////	Quat	= QuatHandler->UpdateTrajectory();
//}
//void FootClass::UpdateData2(){

//    Yaw = YawHandler->UpdateTrajectory()[0];

//    double temp1 = PosHandler[0]->UpdateTrajectory()[0];
//    double temp2 = PosHandler[1]->UpdateTrajectory()[0];

//    Pos[0]	= temp1*cos(Yaw*D2Rf) + temp2*sin(Yaw*D2Rf);

//    Pos[1]	= -temp1*sin(Yaw*D2Rf) + temp2*cos(Yaw*D2Rf);
//    Pos[2]	= PosHandler[2]->UpdateTrajectory()[0];

////    Pos[0]	= PosHandler[0]->UpdateTrajectory()[0];
////    Pos[1]	= PosHandler[1]->UpdateTrajectory()[0];
////    Pos[2]	= PosHandler[2]->UpdateTrajectory()[0];

////	Quat	= QuatHandler->UpdateTrajectory();
//}
//int FootClass::SetInitialPosition(const double x, const double y, const double z, const double yaw, const double roll, const double pitch){
//    int ret = RB_SUCCESS;
//    doubles tempX(1),tempY(1),tempZ(1), tempYaw(1), tempPitch(1), tempRoll(1);
//    tempX[0] = x; tempX[1] = 0.0; tempX[2] = 0.0;
//    tempY[0] = y; tempY[1] = 0.0; tempY[2] = 0.0;
//    tempZ[0] = z; tempZ[1] = 0.0; tempZ[2] = 0.0;
//    tempYaw[0] = yaw;
////---
//    tempPitch[0] = pitch;
//    tempRoll[0] = roll;

//    if(PosHandler[0]->SetRetValue(tempX) == RB_FAIL) ret = RB_FAIL;
//    if(PosHandler[1]->SetRetValue(tempY) == RB_FAIL) ret = RB_FAIL;
//    if(PosHandler[2]->SetRetValue(tempZ) == RB_FAIL) ret = RB_FAIL;
//    if(YawHandler->SetRetValue(tempYaw) == RB_FAIL) ret = RB_FAIL;
//    if(PitchHandler->SetRetValue(tempPitch) == RB_FAIL) ret = RB_FAIL;
//    if(RollHandler->SetRetValue(tempRoll) == RB_FAIL) ret = RB_FAIL;
//    RefreshData();
//    return ret;
//}
//int FootClass::SetInitialOrientation(const double qw, const double qx, const double qy, const double qz){
//    int ret = RB_SUCCESS;
//    doubles tempQ(4);
//    tempQ[0] = qw;
//    tempQ[1] = qx;
//    tempQ[2] = qy;
//    tempQ[3] = qz;

//    if(QuatHandler->SetRetValue(tempQ) == RB_FAIL) ret = RB_FAIL;
//    return ret;
//}

//void FootClass::AddXFifth(const double _pos, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
//    PosHandler[0]->AddTrajInfo(tempTR);
//}
//void FootClass::AddXFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[0]->AddTrajInfo(tempTR);
//}
//void FootClass::AddXCos(const double _pos, const double _t){
//    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
//    PosHandler[0]->AddTrajInfo(tempTR);
//}
//void FootClass::AddXCos(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[0]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYFifth(const double _pos, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
//    PosHandler[1]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[1]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYThird(const double _pos, const double _t, const double _vel){
//    TrajectoryPoly3rd *tempTR = new TrajectoryPoly3rd(_t, _pos, _vel);
//    PosHandler[1]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYThird(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[1]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYCos(const double _pos, const double _t){
//    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
//    PosHandler[1]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYCos(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[1]->AddTrajInfo(tempTR);
//}

//void FootClass::AddZFifth(const double _pos, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
//    PosHandler[2]->AddTrajInfo(tempTR);
//}
//void FootClass::AddZFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[2]->AddTrajInfo(tempTR);
//}
//void FootClass::AddZCos(const double _pos, const double _t){
//    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
//    PosHandler[2]->AddTrajInfo(tempTR);
//}
//void FootClass::AddZCos(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PosHandler[2]->AddTrajInfo(tempTR);
//}
//void FootClass::AddYawFifth(const double _pos, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
//    YawHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddYawFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    YawHandler->AddTrajInfo(tempTR);
//}

////----pitch
//void FootClass::AddPitchCos(const double _pos, const double _t){
//    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
//    PitchHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddPitchCos(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PitchHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddPitchFifth(const double _pos, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
//    PitchHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddPitchFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    PitchHandler->AddTrajInfo(tempTR);
//}



////----Roll
//void FootClass::AddRollCos(const double _pos, const double _t){
//    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
//    RollHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddRollCos(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    RollHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddRollFifth(const double _pos, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
//    RollHandler->AddTrajInfo(tempTR);
//}
//void FootClass::AddRollFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    RollHandler->AddTrajInfo(tempTR);
//}



//void FootClass::AddQuatSlerp(const double _qw, const double _qx, const double _qy, const double _qz, const double _t){
//    doubles tempD(4);
//    tempD[0] = _qw;		tempD[1] = _qx;		tempD[2] = _qy;		tempD[3] = _qz;
//    TrajectorySlerpExp *tempTR = new TrajectorySlerpExp(_t, tempD);
//    QuatHandler->AddTrajInfo(tempTR);
//}


////========================================================================================================

//ZMPClass::ZMPClass(double _tick){
//    ZMP = doubles(3);
//    ZMPHandler = TRHandlers(3);

//    ZMPHandler[0]	= new TrajectoryHandler(ORDER_1, _tick);
//    ZMPHandler[1]	= new TrajectoryHandler(ORDER_1, _tick);
//}
//void ZMPClass::RefreshData(){
//    ZMP[0] = ZMPHandler[0]->GetRetValue()[0];
//    ZMP[1] = ZMPHandler[1]->GetRetValue()[0];
//}
//void ZMPClass::UpdateData(){
//    ZMP[0]	= ZMPHandler[0]->UpdateTrajectory()[0];
//    ZMP[1]	= ZMPHandler[1]->UpdateTrajectory()[0];
//}
//int ZMPClass::SetInitialZMP(const double xzmp, const double yzmp){
//    int ret = RB_SUCCESS;
//    doubles tempX(1),tempY(1);
//    tempX[0] = xzmp;
//    tempY[0] = yzmp;

//    if(ZMPHandler[0]->SetRetValue(tempX) == RB_FAIL) ret = RB_FAIL;
//    if(ZMPHandler[1]->SetRetValue(tempY) == RB_FAIL) ret = RB_FAIL;
//    RefreshData();
//    return ret;
//}

//void ZMPClass::AddXZMPFifth(const double _xzmp, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _xzmp, _vel, _acc);
//    ZMPHandler[0]->AddTrajInfo(tempTR);
//}
//void ZMPClass::AddXZMPFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    ZMPHandler[0]->AddTrajInfo(tempTR);
//}

//void ZMPClass::AddYZMPFifth(const double _yzmp, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _yzmp, _vel, _acc);
//    ZMPHandler[1]->AddTrajInfo(tempTR);
//}
//void ZMPClass::AddYZMPFifth(const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    ZMPHandler[1]->AddTrajInfo(tempTR);
//}

//// ====================================================================================

//AddCompClass::AddCompClass(double _tick){
//    AllocateData(AddJoint, JNUM);
//    AllocateData(AddCOM, 3);
//    AllocateData(AddCOMVel, 3);
//    AllocateData(AddCOMAcc, 3);
//    AllocateData(AddLeftFoot, 3);
//    AllocateData(AddRightFoot, 3);
//    JointHandler = TRHandlers(JNUM);
//    COMHandler = TRHandlers(3);
//    LFHandler = TRHandlers(3);
//    RFHandler = TRHandlers(3);

//    for(int i=0; i<JNUM; i++)
//        JointHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
//    for(int i=0; i<3; i++)
//        COMHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
//    for(int i=0; i<3; i++)
//        LFHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
//    for(int i=0; i<3; i++)
//        RFHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
//}
//void AddCompClass::RefreshData(){
//    for(int i=0; i<JNUM; i++)
//        AddJoint[i] = JointHandler[i]->GetRetValue()[0];
//    for(int i=0; i<3; i++)
//        AddCOM[i] = COMHandler[i]->GetRetValue()[0];
//    for(int i=0; i<3; i++)
//        AddLeftFoot[i] = LFHandler[i]->GetRetValue()[0];
//    for(int i=0; i<3; i++)
//        AddRightFoot[i] = RFHandler[i]->GetRetValue()[0];
//}
////void AddCompClass::UpdateData(){
////    for(int i=0; i<JNUM; i++)
////        AddJoint[i] = JointHandler[i]->UpdateTrajectory()[0];
////    for(int i=0; i<3; i++)
////        AddCOM[i] = COMHandler[i]->UpdateTrajectory()[0];
////    for(int i=0; i<3; i++)
////        AddLeftFoot[i] = LFHandler[i]->UpdateTrajectory()[0];
////    for(int i=0; i<3; i++)
////        AddRightFoot[i] = RFHandler[i]->UpdateTrajectory()[0];
////}

//void AddCompClass::UpdateData(){
//    for(int i=0; i<JNUM; i++)
//    {
//        AddJoint[i] = JointHandler[i]->UpdateTrajectory()[0];
//    }
////    for(int i=0; i<3; i++)
////    {
////        AddCOM[i] = COMHandler[i]->UpdateTrajectory()[0];
////        AddCOMVel[i] = COMHandler[i]->UpdateTrajectory()[1];
////        AddCOMAcc[i] = COMHandler[i]->UpdateTrajectory()[2];
////    }

//    doubles tempcom(3);
////        doubles tempcom;
//    for(int i=0; i<3; i++)
//    {
//        tempcom = COMHandler[i]->UpdateTrajectory();
//        AddCOM[i] = tempcom[0];
//        AddCOMVel[i] = COMHandler[i]->GetRetValue()[1];//tempcom[1];
//        AddCOMAcc[i] = COMHandler[i]->GetRetValue()[2];//tempcom[2];
//    }

//    for(int i=0; i<3; i++)
//    {
//        AddLeftFoot[i] = LFHandler[i]->UpdateTrajectory()[0];
//    }
//    for(int i=0; i<3; i++)
//    {
//        AddRightFoot[i] = RFHandler[i]->UpdateTrajectory()[0];
//    }
//}

//void AddCompClass::AddJointsFifth(const int jointNum, const double _des, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
//    JointHandler[jointNum]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddJointsFifth(const int jointNum, const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    JointHandler[jointNum]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddCOMFifth(const int xyz, const double _des, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
//    COMHandler[xyz]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddCOMFifth(const int xyz, const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    COMHandler[xyz]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddLeftFootFifth(const int xyz, const double _des, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
//    LFHandler[xyz]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddLeftFootFifth(const int xyz, const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    LFHandler[xyz]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddRightFootFifth(const int xyz, const double _des, const double _t, const double _vel, const double _acc){
//    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
//    RFHandler[xyz]->AddTrajInfo(tempTR);
//}
//void AddCompClass::AddRightFootFifth(const int xyz, const double _t){
//    TrajectoryConst *tempTR = new TrajectoryConst(_t);
//    RFHandler[xyz]->AddTrajInfo(tempTR);
//}



#include "FootClass.h"


//========================================================================================================
FootClass::FootClass(double _tick){

    Pos = doubles(3);
    Vel = doubles(3);

    Yaw = 0.0;
    Pitch = 0.0;
    Roll = 0.0;

    PosHandler = TRHandlers(3);

    PosHandler[0]	= new TrajectoryHandler(ORDER_1, _tick);
    PosHandler[1]	= new TrajectoryHandler(ORDER_1, _tick);
    PosHandler[2]	= new TrajectoryHandler(ORDER_1, _tick);
    YawHandler = new TrajectoryHandler(ORDER_1, _tick);
    PitchHandler = new TrajectoryHandler(ORDER_1,_tick);
    RollHandler = new TrajectoryHandler(ORDER_1,_tick);


}


void FootClass::RefreshData(){
    Pos[0] = PosHandler[0]->GetRetValue()[0];
    Pos[1] = PosHandler[1]->GetRetValue()[0];
    Pos[2] = PosHandler[2]->GetRetValue()[0];

    //---
    Vel[0] = PosHandler[0]->GetRetValue()[1];
    Vel[1] = PosHandler[1]->GetRetValue()[1];
    Vel[2] = PosHandler[2]->GetRetValue()[1];

    Yaw = YawHandler->GetRetValue()[0];
    Pitch = PitchHandler->GetRetValue()[0];
    Roll = RollHandler->GetRetValue()[0];

}
void FootClass::UpdateData(){

    Yaw = YawHandler->UpdateTrajectory()[0];
    Pitch = PitchHandler->UpdateTrajectory()[0];
    Roll = RollHandler->UpdateTrajectory()[0];

////-----------
    doubles temppos(3);

    for(int i=0; i<3; i++)
    {
        temppos = PosHandler[i]->UpdateTrajectory();
        Pos[i] = temppos[0];
        Vel[i] = temppos[1];//PosHandler[i]->GetRetValue()[1];//tempcom[1];
    }

//    printf("velx: %f  %f  %f\n",Vel[0],Vel[1],Vel[2]);

//    Pos[0]	= PosHandler[0]->UpdateTrajectory()[0];
//    Pos[1]	= PosHandler[1]->UpdateTrajectory()[0];
//    Pos[2]	= PosHandler[2]->UpdateTrajectory()[0];

}
void FootClass::UpdateData2(){

    Yaw = YawHandler->UpdateTrajectory()[0];

    double temp1 = PosHandler[0]->UpdateTrajectory()[0];
    double temp2 = PosHandler[1]->UpdateTrajectory()[0];

    Pos[0]	= temp1*cos(Yaw*D2Rf) + temp2*sin(Yaw*D2Rf);

    Pos[1]	= -temp1*sin(Yaw*D2Rf) + temp2*cos(Yaw*D2Rf);
    Pos[2]	= PosHandler[2]->UpdateTrajectory()[0];

//    Pos[0]	= PosHandler[0]->UpdateTrajectory()[0];
//    Pos[1]	= PosHandler[1]->UpdateTrajectory()[0];
//    Pos[2]	= PosHandler[2]->UpdateTrajectory()[0];

//	Quat	= QuatHandler->UpdateTrajectory();
}
int FootClass::SetInitialPosition(const double x, const double y, const double z, const double yaw, const double roll, const double pitch){
    int ret = RB_SUCCESS;
    doubles tempX(3),tempY(3),tempZ(3), tempYaw(1), tempPitch(1), tempRoll(1);
    tempX[0] = x; tempX[1] = 0.0; tempX[2] = 0.0;
    tempY[0] = y; tempY[1] = 0.0; tempY[2] = 0.0;
    tempZ[0] = z; tempZ[1] = 0.0; tempZ[2] = 0.0;
    tempYaw[0] = yaw;
//---
    tempPitch[0] = pitch;
    tempRoll[0] = roll;

    if(PosHandler[0]->SetRetValue(tempX) == RB_FAIL) ret = RB_FAIL;
    if(PosHandler[1]->SetRetValue(tempY) == RB_FAIL) ret = RB_FAIL;
    if(PosHandler[2]->SetRetValue(tempZ) == RB_FAIL) ret = RB_FAIL;
    if(YawHandler->SetRetValue(tempYaw) == RB_FAIL) ret = RB_FAIL;
    if(PitchHandler->SetRetValue(tempPitch) == RB_FAIL) ret = RB_FAIL;
    if(RollHandler->SetRetValue(tempRoll) == RB_FAIL) ret = RB_FAIL;
    RefreshData();
    return ret;
}
int FootClass::SetInitialOrientation(const double qw, const double qx, const double qy, const double qz){
    int ret = RB_SUCCESS;
    doubles tempQ(4);
    tempQ[0] = qw;
    tempQ[1] = qx;
    tempQ[2] = qy;
    tempQ[3] = qz;

//    if(QuatHandler->SetRetValue(tempQ) == RB_FAIL) ret = RB_FAIL;
    return ret;
}

void FootClass::AddXFifth(const double _pos, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
    PosHandler[0]->AddTrajInfo(tempTR);
}
void FootClass::AddXFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[0]->AddTrajInfo(tempTR);
}
void FootClass::AddXCos(const double _pos, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
    PosHandler[0]->AddTrajInfo(tempTR);
}
void FootClass::AddXCos(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[0]->AddTrajInfo(tempTR);
}
void FootClass::AddYFifth(const double _pos, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
    PosHandler[1]->AddTrajInfo(tempTR);
}
void FootClass::AddYFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[1]->AddTrajInfo(tempTR);
}
void FootClass::AddYThird(const double _pos, const double _t, const double _vel){
    TrajectoryPoly3rd *tempTR = new TrajectoryPoly3rd(_t, _pos, _vel);
    PosHandler[1]->AddTrajInfo(tempTR);
}
void FootClass::AddYThird(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[1]->AddTrajInfo(tempTR);
}
void FootClass::AddYCos(const double _pos, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
    PosHandler[1]->AddTrajInfo(tempTR);
}
void FootClass::AddYCos(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[1]->AddTrajInfo(tempTR);
}

void FootClass::AddZFifth(const double _pos, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
    PosHandler[2]->AddTrajInfo(tempTR);
}
void FootClass::AddZFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[2]->AddTrajInfo(tempTR);
}
void FootClass::AddZCos(const double _pos, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
    PosHandler[2]->AddTrajInfo(tempTR);
}
void FootClass::AddZCos(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PosHandler[2]->AddTrajInfo(tempTR);
}
void FootClass::AddYawFifth(const double _pos, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
    YawHandler->AddTrajInfo(tempTR);
}
void FootClass::AddYawFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    YawHandler->AddTrajInfo(tempTR);
}
void FootClass::AddYawCos(const double _pos, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
    YawHandler->AddTrajInfo(tempTR);
}
void FootClass::AddYawCos(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    YawHandler->AddTrajInfo(tempTR);
}

//----pitch
void FootClass::AddPitchCos(const double _pos, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
    PitchHandler->AddTrajInfo(tempTR);
}
void FootClass::AddPitchCos(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PitchHandler->AddTrajInfo(tempTR);
}
void FootClass::AddPitchFifth(const double _pos, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
    PitchHandler->AddTrajInfo(tempTR);
}
void FootClass::AddPitchFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    PitchHandler->AddTrajInfo(tempTR);
}



//----Roll
void FootClass::AddRollCos(const double _pos, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _pos);
    RollHandler->AddTrajInfo(tempTR);
}
void FootClass::AddRollCos(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    RollHandler->AddTrajInfo(tempTR);
}
void FootClass::AddRollFifth(const double _pos, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _pos, _vel, _acc);
    RollHandler->AddTrajInfo(tempTR);
}
void FootClass::AddRollFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    RollHandler->AddTrajInfo(tempTR);
}

//Heel pitching
//double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch
void FootClass::AddHeelPitching_x(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
    TrajectoryHeelPitching_x *tempTR = new TrajectoryHeelPitching_x(_t, _rotang, _posx, _posy, _posz,  _yaw,  _roll,  _pitch);
    PosHandler[0]->AddTrajInfo(tempTR);
}


//Heel pitching
//double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch
void FootClass::AddHeelPitching_y(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
    TrajectoryHeelPitching_y *tempTR = new TrajectoryHeelPitching_y(_t, _rotang, _posx, _posy, _posz,  _yaw,  _roll,  _pitch);
    PosHandler[1]->AddTrajInfo(tempTR);
}

//Heel pitching
//double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch
void FootClass::AddHeelPitching_z(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
    TrajectoryHeelPitching_z *tempTR = new TrajectoryHeelPitching_z(_t, _rotang, _posx, _posy, _posz,  _yaw,  _roll,  _pitch);
    PosHandler[2]->AddTrajInfo(tempTR);
}

//Heel pitching
//double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch
void FootClass::AddHeelPitching_yaw(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
    TrajectoryHeelPitching_yaw *tempTR = new TrajectoryHeelPitching_yaw(_t, _rotang, _posx, _posy, _posz,  _yaw,  _roll,  _pitch);
    YawHandler->AddTrajInfo(tempTR);
}

//Heel pitching
//double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch
void FootClass::AddHeelPitching_roll(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
    TrajectoryHeelPitching_roll *tempTR = new TrajectoryHeelPitching_roll(_t, _rotang, _posx, _posy, _posz,  _yaw,  _roll,  _pitch);
    RollHandler->AddTrajInfo(tempTR);
}

//Heel pitching
//double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch
void FootClass::AddHeelPitching_pitch(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch){
    TrajectoryHeelPitching_pitch *tempTR = new TrajectoryHeelPitching_pitch(_t, _rotang, _posx, _posy, _posz,  _yaw,  _roll,  _pitch);
    PitchHandler->AddTrajInfo(tempTR);
}




//========================================================================================================

ZMPClass::ZMPClass(double _tick){
    ZMP = doubles(3);
    ZMPHandler = TRHandlers(3);

    ZMPHandler[0]	= new TrajectoryHandler(ORDER_1, _tick);
    ZMPHandler[1]	= new TrajectoryHandler(ORDER_1, _tick);
}
void ZMPClass::RefreshData(){
    ZMP[0] = ZMPHandler[0]->GetRetValue()[0];
    ZMP[1] = ZMPHandler[1]->GetRetValue()[0];
}
void ZMPClass::UpdateData(){
    ZMP[0]	= ZMPHandler[0]->UpdateTrajectory()[0];
    ZMP[1]	= ZMPHandler[1]->UpdateTrajectory()[0];
}
int ZMPClass::SetInitialZMP(const double xzmp, const double yzmp){
    int ret = RB_SUCCESS;
    doubles tempX(1),tempY(1);
    tempX[0] = xzmp;
    tempY[0] = yzmp;

    if(ZMPHandler[0]->SetRetValue(tempX) == RB_FAIL) ret = RB_FAIL;
    if(ZMPHandler[1]->SetRetValue(tempY) == RB_FAIL) ret = RB_FAIL;
    RefreshData();
    return ret;
}

void ZMPClass::AddXZMPFifth(const double _xzmp, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _xzmp, _vel, _acc);
    ZMPHandler[0]->AddTrajInfo(tempTR);
}
void ZMPClass::AddXZMPFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    ZMPHandler[0]->AddTrajInfo(tempTR);
}

void ZMPClass::AddYZMPFifth(const double _yzmp, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _yzmp, _vel, _acc);
    ZMPHandler[1]->AddTrajInfo(tempTR);
}
void ZMPClass::AddYZMPFifth(const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    ZMPHandler[1]->AddTrajInfo(tempTR);
}

// ====================================================================================

AddCompClass::AddCompClass(double _tick){
    AllocateData(AddJoint, JNUM);
    AllocateData(AddCOM, 3);
    AllocateData(AddCOMVel, 3);
    AllocateData(AddCOMAcc, 3);
    AllocateData(AddLeftFoot, 3);
    AllocateData(AddRightFoot, 3);
    JointHandler = TRHandlers(JNUM);
    COMHandler = TRHandlers(3);
    LFHandler = TRHandlers(3);
    RFHandler = TRHandlers(3);

    for(int i=0; i<JNUM; i++)
        JointHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
    for(int i=0; i<3; i++)
        COMHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
    for(int i=0; i<3; i++)
        LFHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
    for(int i=0; i<3; i++)
        RFHandler[i] = new TrajectoryHandler(ORDER_1, _tick);
}
void AddCompClass::RefreshData(){
    for(int i=0; i<JNUM; i++)
        AddJoint[i] = JointHandler[i]->GetRetValue()[0];
    for(int i=0; i<3; i++)
        AddCOM[i] = COMHandler[i]->GetRetValue()[0];
    for(int i=0; i<3; i++)
        AddLeftFoot[i] = LFHandler[i]->GetRetValue()[0];
    for(int i=0; i<3; i++)
        AddRightFoot[i] = RFHandler[i]->GetRetValue()[0];
}
//void AddCompClass::UpdateData(){
//    for(int i=0; i<JNUM; i++)
//        AddJoint[i] = JointHandler[i]->UpdateTrajectory()[0];
//    for(int i=0; i<3; i++)
//        AddCOM[i] = COMHandler[i]->UpdateTrajectory()[0];
//    for(int i=0; i<3; i++)
//        AddLeftFoot[i] = LFHandler[i]->UpdateTrajectory()[0];
//    for(int i=0; i<3; i++)
//        AddRightFoot[i] = RFHandler[i]->UpdateTrajectory()[0];
//}
int AddCompClass::SetInitialCOM(const double com1){
    int ret = RB_SUCCESS;
    doubles tempCom(3);
    tempCom[0] = com1; tempCom[1] = 0.0; tempCom[2] = 0.0;

    if(COMHandler[2]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;

    RefreshData();
    return ret;
}

int AddCompClass::SetInitialAddJoint(){
    int ret = RB_SUCCESS;
    doubles tempCom(3);
    tempCom[0] = 0; tempCom[1] = 0.0; tempCom[2] = 0.0;

    if(JointHandler[JRAR]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;
    if(JointHandler[JLAR]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;
    if(JointHandler[JRAP]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;
    if(JointHandler[JLAP]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;
    if(JointHandler[JRHR]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;
    if(JointHandler[JLHR]->SetRetValue(tempCom) == RB_FAIL) ret = RB_FAIL;


    RefreshData();
    return ret;
}

//int FootClass::SetInitialPosition(const double x, const double y, const double z, const double yaw, const double roll, const double pitch){
//    int ret = RB_SUCCESS;
//    doubles tempX(3),tempY(3),tempZ(3), tempYaw(1), tempPitch(1), tempRoll(1);
//    tempX[0] = x; tempX[1] = 0.0; tempX[2] = 0.0;
//    tempY[0] = y; tempY[1] = 0.0; tempY[2] = 0.0;
//    tempZ[0] = z; tempZ[1] = 0.0; tempZ[2] = 0.0;
//    tempYaw[0] = yaw;
////---
//    tempPitch[0] = pitch;
//    tempRoll[0] = roll;

//    if(PosHandler[0]->SetRetValue(tempX) == RB_FAIL) ret = RB_FAIL;
//    if(PosHandler[1]->SetRetValue(tempY) == RB_FAIL) ret = RB_FAIL;
//    if(PosHandler[2]->SetRetValue(tempZ) == RB_FAIL) ret = RB_FAIL;
//    if(YawHandler->SetRetValue(tempYaw) == RB_FAIL) ret = RB_FAIL;
//    if(PitchHandler->SetRetValue(tempPitch) == RB_FAIL) ret = RB_FAIL;
//    if(RollHandler->SetRetValue(tempRoll) == RB_FAIL) ret = RB_FAIL;
//    RefreshData();
//    return ret;
//}
void AddCompClass::UpdateData(){
    for(int i=0; i<JNUM; i++)
    {
        AddJoint[i] = JointHandler[i]->UpdateTrajectory()[0];
    }
//    for(int i=0; i<3; i++)
//    {
//        AddCOM[i] = COMHandler[i]->UpdateTrajectory()[0];
//        AddCOMVel[i] = COMHandler[i]->UpdateTrajectory()[1];
//        AddCOMAcc[i] = COMHandler[i]->UpdateTrajectory()[2];
//    }

    doubles tempcom(3);
//        doubles tempcom;
    for(int i=0; i<3; i++)
    {
        tempcom = COMHandler[i]->UpdateTrajectory();
        AddCOM[i] = tempcom[0];
        AddCOMVel[i] = tempcom[1];
        AddCOMAcc[i] = tempcom[2];
    }

    for(int i=0; i<3; i++)
    {
        AddLeftFoot[i] = LFHandler[i]->UpdateTrajectory()[0];
    }
    for(int i=0; i<3; i++)
    {
        AddRightFoot[i] = RFHandler[i]->UpdateTrajectory()[0];
    }
}

void AddCompClass::AddJointsFifth(const int jointNum, const double _des, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
    JointHandler[jointNum]->AddTrajInfo(tempTR);
}
void AddCompClass::AddJointsFifth(const int jointNum, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    JointHandler[jointNum]->AddTrajInfo(tempTR);
}
void AddCompClass::AddCOMFifth(const int xyz, const double _des, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
    COMHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddCOMFifth(const int xyz, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    COMHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddCOMCos(const int xyz, const double _des, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _des);
    COMHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddCOMCos(const int xyz, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    COMHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddLeftFootFifth(const int xyz, const double _des, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
    LFHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddLeftFootFifth(const int xyz, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    LFHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddRightFootFifth(const int xyz, const double _des, const double _t, const double _vel, const double _acc){
    TrajectoryPoly5th *tempTR = new TrajectoryPoly5th(_t, _des, _vel, _acc);
    RFHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddRightFootFifth(const int xyz, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    RFHandler[xyz]->AddTrajInfo(tempTR);
}

void AddCompClass::AddLeftFootCos(const int xyz, const double _des, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _des);
    LFHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddLeftFootCos(const int xyz, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    LFHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddRightFootCos(const int xyz, const double _des, const double _t){
    TrajectoryCosine *tempTR = new TrajectoryCosine(_t, _des);
    RFHandler[xyz]->AddTrajInfo(tempTR);
}
void AddCompClass::AddRightFootCos(const int xyz, const double _t){
    TrajectoryConst *tempTR = new TrajectoryConst(_t);
    RFHandler[xyz]->AddTrajInfo(tempTR);
}
