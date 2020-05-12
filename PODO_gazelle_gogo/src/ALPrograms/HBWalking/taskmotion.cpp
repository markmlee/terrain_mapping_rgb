#include "taskmotion.h"
#include "BasicFiles/BasicJoint.h"

#define D2R			1.745329251994330e-2
#define R2D			5.729577951308232e1



TaskMotion::TaskMotion(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, JointControlClass *_joint){
    Shm_ref = _shm_ref;
    Shm_sen = _shm_sen;
    Shm_com = _shm_com;

    Joint = _joint;

    for(int i=0; i<3; i++){
        wbPosRF[i] = new TrajectoryHandler(ORDER_1, 0.002);
        wbPosLF[i] = new TrajectoryHandler(ORDER_1, 0.002);
        wbPosCOM[i] = new TrajectoryHandler(ORDER_1, 0.002);
    }
    wbPosWST = new TrajectoryHandler(ORDER_1, 0.002);
    wbPosPelZ = new TrajectoryHandler(ORDER_1, 0.002);


    wbOriRF = new TrajectoryHandler(ORDER_QUAT, 0.002);
    wbOriLF = new TrajectoryHandler(ORDER_QUAT, 0.002);
    wbOriPel = new TrajectoryHandler(ORDER_QUAT, 0.0052);

    WBTYPE = WB_POINT_TO_POINT;
    MomentFlag = false;
}

TaskMotion::~TaskMotion()
{
}

void TaskMotion::ResetGlobalCoord(int RF_OR_LF_OR_PC_MidFoot){//do_nothing_now.

    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::StopAll(void){
    for(int i=0; i<3; i++){
        wbPosRF[i]->StopAndEraseAll();
        wbPosLF[i]->StopAndEraseAll();
        wbPosCOM[i]->StopAndEraseAll();
    }
    wbPosPelZ->StopAndEraseAll();
    wbPosWST->StopAndEraseAll();
    wbOriRF->StopAndEraseAll();
    wbOriLF->StopAndEraseAll();
    wbOriPel->StopAndEraseAll();

    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::RefreshToCurrentReference(void){

    //put joint values here.///////////////////////////////////////
    LJ.RHY = Joint->GetJointRefAngle(RHY)*D2R;
    LJ.RHR = Joint->GetJointRefAngle(RHR)*D2R;
    LJ.RHP = Joint->GetJointRefAngle(RHP)*D2R;
    LJ.RKN = Joint->GetJointRefAngle(RKN)*D2R;
    LJ.RAP = Joint->GetJointRefAngle(RAP)*D2R;
    LJ.RAR = Joint->GetJointRefAngle(RAR)*D2R;
    //for gazelle
    double RAP_deg, RAR_deg;
    GK.FK_diff_Ankle_right(Joint->GetJointRefAngle(RAP), Joint->GetJointRefAngle(RAR), 0, 0, RAP_deg, RAR_deg);
    LJ.RAP = RAP_deg*D2R;
    LJ.RAR = RAR_deg*D2R;

    cout<<"RA1_deg : "<<Joint->GetJointRefAngle(RAP)<<"  RA2_deg : "<<Joint->GetJointRefAngle(RAR)<<endl;
    cout<<"RAR_deg : "<<RAR_deg<<endl;

    LJ.LHY = Joint->GetJointRefAngle(LHY)*D2R;
    LJ.LHR = Joint->GetJointRefAngle(LHR)*D2R;
    LJ.LHP = Joint->GetJointRefAngle(LHP)*D2R;
    LJ.LKN = Joint->GetJointRefAngle(LKN)*D2R;
    LJ.LAP = Joint->GetJointRefAngle(LAP)*D2R;
    LJ.LAR = Joint->GetJointRefAngle(LAR)*D2R;
    //for gazelle
    double LAP_deg, LAR_deg;
    GK.FK_diff_Ankle_left(Joint->GetJointRefAngle(LAP), Joint->GetJointRefAngle(LAR),0,0,LAP_deg, LAR_deg);
    LJ.LAP = LAP_deg*D2R;
    LJ.LAR = LAR_deg*D2R;

    //put arm joint values
    AJ.RSP = Joint->GetJointRefAngle(RSP)*D2R;
    AJ.RSR = Joint->GetJointRefAngle(RSR)*D2R + OFFSET_RSR*D2R;
    AJ.RSY = Joint->GetJointRefAngle(RSY)*D2R;
    AJ.REB = Joint->GetJointRefAngle(REB)*D2R + OFFSET_ELB*D2R;
    AJ.RWY = Joint->GetJointRefAngle(RWY)*D2R;
    AJ.RWP = Joint->GetJointRefAngle(RWP)*D2R;
    AJ.RF1 = Joint->GetJointRefAngle(RF1)*D2R;

    AJ.LSP = Joint->GetJointRefAngle(LSP)*D2R;
    AJ.LSR = Joint->GetJointRefAngle(LSR)*D2R + OFFSET_LSR*D2R;
    AJ.LSY = Joint->GetJointRefAngle(LSY)*D2R;
    AJ.LEB = Joint->GetJointRefAngle(LEB)*D2R + OFFSET_ELB*D2R;
    AJ.LWY = Joint->GetJointRefAngle(LWY)*D2R;
    AJ.LWP = Joint->GetJointRefAngle(LWP)*D2R;
    AJ.LF1 = Joint->GetJointRefAngle(LF1)*D2R;

    AJ.WST = Joint->GetJointRefAngle(WST)*D2R;

    //kine_oi.setUB(kine_oi.FKCOM_UB(AJ));

    LJ.pPel = vec3(0,0,0);
    LJ.qPel = quat(vec3(1,0,0),0.0);

    FeetPos FP = kine_oi.FK(LJ);//now, just start from pel0=0, pelori = 0;
    //pPelZ =0;



    for(int i=0; i<3; i++){
        pRF_3x1[i] = FP.pRF[i];
        pLF_3x1[i] = FP.pLF[i];
        pCOM_3x1[i] = FP.pCOM[i];
        pPel_3x1[i] = LJ.pPel[i];  //hyobin added
        wbPosRF[i]->SetRetValue(pRF_3x1[i]);
        wbPosLF[i]->SetRetValue(pLF_3x1[i]);
    }
    for(int i=0;i<4;i++)
    {
        qRF_4x1[i] = FP.qRF[i];
        qLF_4x1[i] = FP.qLF[i];
    }
    wbPosPelZ->SetRetValue(0);
    wbPosCOM[0]->SetRetValue(pCOM_3x1[0]);
    wbPosCOM[1]->SetRetValue(pCOM_3x1[1]);
    wbPosCOM[2]->SetRetValue(pCOM_3x1[2]);
    wbPosWST->SetRetValue(Joint->GetJointRefAngle(WST));


    doubles tempRF, tempLF, tempPEL;
    quat tq = quat();
    for(int i=0; i<4; i++){
        tempRF.push_back(qRF_4x1[i]);
        tempLF.push_back(qLF_4x1[i]);
        tempPEL.push_back(tq[i]);
        qPEL_4x1[i] = tq[i];
    }
    wbOriRF->SetRetValue(tempRF);
    wbOriLF->SetRetValue(tempLF);
    wbOriPel->SetRetValue(tempPEL);

    LJ_old = LJ;
    AJ_old = AJ;
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::RefreshToCurrentReference_HB(void){

    //put joint values here.///////////////////////////////////////
    LJ.RHY = Joint->GetJointRefAngle(RHY)*D2R;
    LJ.RHR = Joint->GetJointRefAngle(RHR)*D2R;
    LJ.RHP = Joint->GetJointRefAngle(RHP)*D2R;
    LJ.RKN = Joint->GetJointRefAngle(RKN)*D2R;
    LJ.RAP = Joint->GetJointRefAngle(RAP)*D2R;
    LJ.RAR = Joint->GetJointRefAngle(RAR)*D2R;
    //for gazelle
    double RAP_deg, RAR_deg;
    GK.FK_diff_Ankle_right(Joint->GetJointRefAngle(RAP), Joint->GetJointRefAngle(RAR), 0, 0, RAP_deg, RAR_deg);
    LJ.RAP = RAP_deg*D2R;
    LJ.RAR = RAR_deg*D2R;

    cout<<"RA1_deg : "<<Joint->GetJointRefAngle(RAP)<<"  RA2_deg : "<<Joint->GetJointRefAngle(RAR)<<endl;
    cout<<"RAR_deg : "<<RAR_deg<<endl;

    LJ.LHY = Joint->GetJointRefAngle(LHY)*D2R;
    LJ.LHR = Joint->GetJointRefAngle(LHR)*D2R;
    LJ.LHP = Joint->GetJointRefAngle(LHP)*D2R;
    LJ.LKN = Joint->GetJointRefAngle(LKN)*D2R;
    LJ.LAP = Joint->GetJointRefAngle(LAP)*D2R;
    LJ.LAR = Joint->GetJointRefAngle(LAR)*D2R;
    //for gazelle
    double LAP_deg, LAR_deg;
    GK.FK_diff_Ankle_left(Joint->GetJointRefAngle(LAP), Joint->GetJointRefAngle(LAR),0,0,LAP_deg, LAR_deg);
    LJ.LAP = LAP_deg*D2R;
    LJ.LAR = LAR_deg*D2R;

    //put arm joint values
    AJ.RSP = Joint->GetJointRefAngle(RSP)*D2R;
    AJ.RSR = Joint->GetJointRefAngle(RSR)*D2R + OFFSET_RSR*D2R;
    AJ.RSY = Joint->GetJointRefAngle(RSY)*D2R;
    AJ.REB = Joint->GetJointRefAngle(REB)*D2R + OFFSET_ELB*D2R;
    AJ.RWY = Joint->GetJointRefAngle(RWY)*D2R;
    AJ.RWP = Joint->GetJointRefAngle(RWP)*D2R;
    AJ.RF1 = Joint->GetJointRefAngle(RF1)*D2R;

    AJ.LSP = Joint->GetJointRefAngle(LSP)*D2R;
    AJ.LSR = Joint->GetJointRefAngle(LSR)*D2R + OFFSET_LSR*D2R;
    AJ.LSY = Joint->GetJointRefAngle(LSY)*D2R;
    AJ.LEB = Joint->GetJointRefAngle(LEB)*D2R + OFFSET_ELB*D2R;
    AJ.LWY = Joint->GetJointRefAngle(LWY)*D2R;
    AJ.LWP = Joint->GetJointRefAngle(LWP)*D2R;
    AJ.LF1 = Joint->GetJointRefAngle(LF1)*D2R;

    AJ.WST = Joint->GetJointRefAngle(WST)*D2R;

    //kine_oi.setUB(kine_oi.FKCOM_UB(AJ));

    LJ.pPel = vec3(0,0,0);
    LJ.qPel = quat(vec3(1,0,0),0.0);

    FeetPos FP = kine_oi.FK(LJ);//now, just start from pel0=0, pelori = 0;
    //pPelZ =0;



    pPelZ = LJ.pPel.z;

    for(int i=0; i<3; i++){
        pRF_3x1[i] = FP.pRF[i];
        pLF_3x1[i] = FP.pLF[i];
        pCOM_3x1[i] = FP.pCOM[i];
        pPel_3x1[i] = LJ.pPel[i];  //hyobin added
    }
    for(int i=0;i<4;i++)
    {
        qRF_4x1[i] = FP.qRF[i];
        qLF_4x1[i] = FP.qLF[i];
        qPEL_4x1[i] = LJ.qPel[i];
    }

    des_pPELz_HB = LJ.pPel.z;

    for(int i=0; i<3; i++){
        des_pRF_3x1_HB[i] = FP.pRF[i];
        des_pLF_3x1_HB[i] = FP.pLF[i];
        des_pCOM_3x1_HB[i] = FP.pCOM[i];
    }
    for(int i=0;i<4;i++)
    {
        des_qRF_4x1_HB[i] = FP.qRF[i];
        des_qLF_4x1_HB[i] = FP.qLF[i];
        des_qPEL_4x1_HB[i] = LJ.qPel[i];
    }
}


void TaskMotion::addCOMInfo(double _xCOM, double _yCOM, double _zCOM, double _sTime)
{
    TRInfo tempInfo;
	tempInfo = new TrajectoryCosine(_sTime, _xCOM);
    wbPosCOM[0]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryCosine(_sTime, _yCOM);
    wbPosCOM[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _zCOM);
    wbPosCOM[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addCOMInfo_HB(double _xCOM, double _yCOM, double _zCOM)
{
    des_pCOM_3x1_HB[0] = _xCOM;
    des_pCOM_3x1_HB[1] = _yCOM;
    des_pCOM_3x1_HB[2] = _zCOM;
}
void TaskMotion::addCOMInfo_xy_pelz_HB(double _xCOM, double _yCOM, double _pelz)
{
    des_pCOM_3x1_HB[0] = _xCOM;
    des_pCOM_3x1_HB[1] = _yCOM;
    des_pPELz_HB = _pelz;
}
void TaskMotion::addCOMInfo(double _sTime){
    TRInfo tempInfo;
	tempInfo = new TrajectoryConst(_sTime);
    wbPosCOM[0]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryConst(_sTime);
    wbPosCOM[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosCOM[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime){
    TRInfo tempInfo;
	tempInfo = new TrajectoryCosine(_sTime, _xLeg);
    wbPosRF[0]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryCosine(_sTime, _yLeg);
    wbPosRF[1]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryCosine(_sTime, _zLeg);
    wbPosRF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRFPosInfo_HB(double _xLeg, double _yLeg, double _zLeg){
    des_pRF_3x1_HB[0] = _xLeg;
    des_pRF_3x1_HB[1] = _yLeg;
    des_pRF_3x1_HB[2] = _zLeg;
}

void TaskMotion::addRFPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRF[0]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryConst(_sTime);
    wbPosRF[1]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryConst(_sTime);
    wbPosRF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addLFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime){
    TRInfo tempInfo;
	tempInfo = new TrajectoryCosine(_sTime, _xLeg);
    wbPosLF[0]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryCosine(_sTime, _yLeg);
    wbPosLF[1]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryCosine(_sTime, _zLeg);
    wbPosLF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLFPosInfo_HB(double _xLeg, double _yLeg, double _zLeg){
    des_pLF_3x1_HB[0] = _xLeg;
    des_pLF_3x1_HB[1] = _yLeg;
    des_pLF_3x1_HB[2] = _zLeg;
}
void TaskMotion::addLFPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLF[0]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryConst(_sTime);
    wbPosLF[1]->AddTrajInfo(tempInfo);
	tempInfo = new TrajectoryConst(_sTime);
    wbPosLF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}


void TaskMotion::addWSTPosInfo(double _wst, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _wst);
    wbPosWST->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addWSTPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosWST->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addPELPosInfo(double _pelz, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _pelz);
    wbPosPelZ->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addPELPosInfo_HB(double _pelz){
    des_pPELz_HB = _pelz;
}
void TaskMotion::addPELPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosPelZ->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}


void TaskMotion::addPELOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
	wbOriPel->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addPELOriInfo_HB(quat _qPel){
    des_qPEL_4x1_HB[0] = _qPel[0];
    des_qPEL_4x1_HB[1] = _qPel[1];
    des_qPEL_4x1_HB[2] = _qPel[2];
    des_qPEL_4x1_HB[3] = _qPel[3];
}

void TaskMotion::addPELOriInfo(double _sTime){
	TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
	wbOriPel->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRFOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
	wbOriRF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addRFOriInfo_HB(quat _qRF){
    des_qRF_4x1_HB[0] = _qRF[0];
    des_qRF_4x1_HB[1] = _qRF[1];
    des_qRF_4x1_HB[2] = _qRF[2];
    des_qRF_4x1_HB[3] = _qRF[3];
}
void TaskMotion::addRFOriInfo(double _sTime){
	TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
	wbOriRF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addLFOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
	wbOriLF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLFOriInfo_HB(quat _qLF){
    des_qLF_4x1_HB[0] = _qLF[0];
    des_qLF_4x1_HB[1] = _qLF[1];
    des_qLF_4x1_HB[2] = _qLF[2];
    des_qLF_4x1_HB[3] = _qLF[3];
}
void TaskMotion::addLFOriInfo(double _sTime){
	TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
	wbOriLF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;

//    delete tempInfo;
}


void TaskMotion::updateAll(){
    if(WBTYPE==WB_POINT_TO_POINT)
    {
        for(int i=0; i<3; i++){
            doubles _rf = wbPosRF[i]->UpdateTrajectory();
            doubles _lf = wbPosLF[i]->UpdateTrajectory();

            des_pRF_3x1[i] = _rf[0];
            des_pLF_3x1[i] = _lf[0];
        }

        doubles _com1 = wbPosCOM[0]->UpdateTrajectory();
        doubles _com2 = wbPosCOM[1]->UpdateTrajectory();
        doubles _com3 = wbPosCOM[2]->UpdateTrajectory();
        doubles _pelz = wbPosPelZ->UpdateTrajectory();
        doubles _wst = wbPosWST->UpdateTrajectory();

        des_pCOM_3x1[0] = _com1[0];
        des_pCOM_3x1[1] = _com2[0];
        des_pCOM_3x1[2] = _com3[0];
        des_pPELz = _pelz[0];
        des_rWST = _wst[0]*D2R;//*R2D;

        doubles _orf = wbOriRF->UpdateTrajectory();
        doubles _olf = wbOriLF->UpdateTrajectory();
        doubles _opel = wbOriPel->UpdateTrajectory();

        for(int i=0; i<4; i++){
            des_qRF_4x1[i] = _orf[i];
            des_qLF_4x1[i] = _olf[i];
            des_qPEL_4x1[i] = _opel[i];
        }
    }
    else if(WBTYPE==WB_ROTATE_VALVE)
    {

    }
}

void TaskMotion::updateAll_HB(){
//    cout<<"COM.x : "<<des_pCOM_3x1[0]<<"   COM.y : "<<des_pCOM_3x1[1]<<"  COM.z : "<<des_pCOM_3x1[2]<<endl;
//    cout<<"LF.x : "<<des_pLF_3x1[0]<<"   LF.y : "<<des_pLF_3x1[1]<<"  LF.z : "<<des_pLF_3x1[2]<<endl;
//    cout<<"RF.x : "<<des_pRF_3x1[0]<<"   RF.y : "<<des_pRF_3x1[1]<<"  RF.z : "<<des_pRF_3x1[2]<<endl;
}

void TaskMotion::WBIK(){

    //IK here
    if(MomentFlag==false)
    {
        LJ = kine_oi.IK_COM(vec3(des_pCOM_3x1_HB),quat(des_qPEL_4x1_HB)
                            ,vec3(des_pRF_3x1_HB),quat(des_qRF_4x1_HB)
                            , vec3(des_pLF_3x1_HB),quat(des_qLF_4x1_HB));
        LJ.qPel = quat(des_qPEL_4x1_HB);
    }
    else
    {
//        LJ = kine_oi.IK_COM_MM(vec3(des_pCOM_3x1)
//                            ,vec3(des_pRF_3x1),quat(des_qRF_4x1)
//                            , vec3(des_pLF_3x1),quat(des_qLF_4x1)
//                            ,AJ_old,LJ_old,LJ_old.pPel,LJ_old.qPel
//                               ,vec3(0,0,0));//not zero? optimize?
//        //modefy ddCOM to make total momentum zero
//        //control ddCOM? not to rotate total momentum?
//        //there is a problem when com moves forward/sideways
    }

    //----------------------------------------
    //FK

    FeetPos FP = kine_oi.FK(LJ);


    for(int i=0; i<3; i++){
        pPel_3x1[i] = LJ.pPel[i];
        pRF_3x1[i] = FP.pRF[i];
        pLF_3x1[i] = FP.pLF[i];
        pCOM_3x1[i] = FP.pCOM[i];
    }
    for(int i=0;i<4;i++)
    {
        qRF_4x1[i] = FP.qRF[i];
        qLF_4x1[i] = FP.qLF[i];
        qPEL_4x1[i] = FP.qPel[i];
    }
    //and put to values

     WBMODE_LASTUSED = 1;

    LJ_old = LJ;
    AJ_old = AJ;
}

void TaskMotion::WBIK_xy_pelz(){

    //IK here
    if(MomentFlag==false)
    {
        vec3 des_pRF_new = vec3(des_pRF_3x1_HB[0], des_pRF_3x1_HB[1], des_pRF_3x1_HB[2] + 0.000);

        LJ = kine_oi.IK_COM_xy(vec3(des_pCOM_3x1_HB), des_pPELz_HB, quat(des_qPEL_4x1_HB)
                            ,des_pRF_new,quat(des_qRF_4x1_HB)
                            , vec3(des_pLF_3x1_HB),quat(des_qLF_4x1_HB));
        LJ.qPel = quat(des_qPEL_4x1_HB);
    }
    else
    {
//        LJ = kine_oi.IK_COM_MM(vec3(des_pCOM_3x1)
//                            ,vec3(des_pRF_3x1),quat(des_qRF_4x1)
//                            , vec3(des_pLF_3x1),quat(des_qLF_4x1)
//                            ,AJ_old,LJ_old,LJ_old.pPel,LJ_old.qPel
//                               ,vec3(0,0,0));//not zero? optimize?
//        //modefy ddCOM to make total momentum zero
//        //control ddCOM? not to rotate total momentum?
//        //there is a problem when com moves forward/sideways
    }

    //----------------------------------------
    //FK

    FeetPos FP = kine_oi.FK(LJ);

    pPelZ = FP.pPelz;

    for(int i=0; i<3; i++){
        pPel_3x1[i] = LJ.pPel[i];
        pRF_3x1[i] = FP.pRF[i];
        pLF_3x1[i] = FP.pLF[i];
        pCOM_3x1[i] = FP.pCOM[i];
    }
    for(int i=0;i<4;i++)
    {
        qRF_4x1[i] = FP.qRF[i];
        qLF_4x1[i] = FP.qLF[i];
        qPEL_4x1[i] = FP.qPel[i];
    }
    //and put to values

     WBMODE_LASTUSED = 1;

    LJ_old = LJ;
    AJ_old = AJ;
}

LegJoints TaskMotion::WBIK2(quat _qRF, quat _qLF){

    //IK here

    LegJoints LJ_original;
    LJ_original = kine_oi.IK_COM(vec3(des_pCOM_3x1),quat(des_qPEL_4x1)
                        ,vec3(des_pRF_3x1),quat(_qRF)
                        , vec3(des_pLF_3x1),quat(_qLF));
    LJ_original.qPel = quat(des_qPEL_4x1);

    return LJ_original;

    //----------------------------------------
    //FK

//    FeetPos FP = kine_oi.FK(LJ);


//    for(int i=0; i<3; i++){
//        pPel_3x1[i] = LJ.pPel[i];
//        pRF_3x1[i] = FP.pRF[i];
//        pLF_3x1[i] = FP.pLF[i];
//        pCOM_3x1[i] = FP.pCOM[i];
//    }
//    for(int i=0;i<4;i++)
//    {
//        qRF_4x1[i] = FP.qRF[i];
//        qLF_4x1[i] = FP.qLF[i];
//        qPEL_4x1[i] = FP.qPel[i];
//    }
//    //and put to values

//     WBMODE_LASTUSED = 1;

//    LJ_old = LJ;
//    AJ_old = AJ;

}
