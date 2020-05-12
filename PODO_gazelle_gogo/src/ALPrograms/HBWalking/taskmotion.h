#ifndef TASKMOTION_H
#define TASKMOTION_H


#include "taskGeneral.h"
#include "BasicTrajectory.h"
#include "BasicFiles/BasicJoint.h"
#include "BasicMatrix.h"
//#include "Oinverse.h"
#include "HB_inverse.h"
#include "../../Gazelle_Ankle_Kine/Gazelle_kine.h"

using namespace rainbow;

class TaskMotion
{

    enum WB_TYPE
    {
        WB_POINT_TO_POINT,
        WB_ROTATE_VALVE
    };

public:
    explicit TaskMotion(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, JointControlClass *_joint);
    ~TaskMotion();

private:
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
    JointControlClass   *Joint;
public:
    WB_TYPE WBTYPE;
    int WBMODE_LASTUSED;

public:

    //CKINE_DRC_HUBO2 kine_drc;
   //
    bool    MomentFlag;
    HB_inverse kine_oi;
    Gazelle_Kine GK;
    LegJoints LJ, LJ_old;
    ArmJoints AJ, AJ_old;

    //--------------------------------------------------------------------------------//
	TRHandler   wbPosRF[3];
    TRHandler   wbPosLF[3];
	TRHandler   wbPosWST;
    TRHandler   wbPosCOM[3];
    TRHandler   wbPosPelZ;
	//--------------------------------------------------------------------------------//
	TRHandler   wbOriRF;
    TRHandler   wbOriLF;
	TRHandler   wbOriPel;
	//--------------------------------------------------------------------------------//

    double      pCOM_3x1[3];
    double      pPel_3x1[3];
    double      pPelZ;
    double      pRF_3x1[3];

    double      pLF_3x1[3];

    double      qRF_4x1[4];
    double      qLF_4x1[4];
    double      qPEL_4x1[4];
    ///////////////////////ENC
    double      Q_Enc_34x1[34];
    double      Enc_pCOM_3x1[3];
    double      Enc_pPelZ;
    double      Enc_pRF_3x1[3];
    double      Enc_pLF_3x1[3];

    double      Enc_qRF_4x1[4];
    double      Enc_qLF_4x1[4];
    double      Enc_qPEL_4x1[4];


    double      des_pCOM_3x1[3];
    double      des_pPELz;
    double      des_rWST;
    double      des_pRF_3x1[3];
    double      des_pLF_3x1[3];
    double      des_pRH_3x1[3];
    double      des_pLH_3x1[3];

    double      des_qRF_4x1[4];
    double      des_qLF_4x1[4];
    double      des_qPEL_4x1[4];

    double      des_pCOM_3x1_HB[3];
    double      des_pPELz_HB;
    double      des_rWST_HB;
    double      des_pRF_3x1_HB[3];
    double      des_pLF_3x1_HB[3];
    double      des_pRH_3x1_HB[3];
    double      des_pLH_3x1_HB[3];

    double      des_qRF_4x1_HB[4];
    double      des_qLF_4x1_HB[4];
    double      des_qPEL_4x1_HB[4];

    void    StopAll(void);
    void    ResetGlobalCoord(int RF_OR_LF_OR_PC_MidFoot);
    void    RefreshToCurrentReference(void);
    void    RefreshToCurrentReference_HB(void);

    void addCOMInfo(double _xCOM, double _yCOM, double _zCOM, double _sTime);
	void addCOMInfo(double _sTime);
    void addCOMInfo_HB(double _xCOM, double _yCOM, double _zCOM);
    void addCOMInfo_xy_pelz_HB(double _xCOM, double _yCOM, double _pelz);

	void addRFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime);
	void addRFPosInfo(double _sTime);
    void addRFPosInfo_HB(double _xLeg, double _yLeg, double _zLeg);

	void addLFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime);
	void addLFPosInfo(double _sTime);
    void addLFPosInfo_HB(double _xLeg, double _yLeg, double _zLeg);

	void addRHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime);
	void addRHPosInfo(double _sTime);

	void addLHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime);
	void addLHPosInfo(double _sTime);

    void addWSTPosInfo(double _wst, double _sTime);
    void addWSTPosInfo(double _sTime);

    void addBPitchPosInfo(double _wst, double _sTime);
    void addBPitchPosInfo(double _sTime);

    void addPELPosInfo(double _pelz, double _sTime);
    void addPELPosInfo(double _sTime);
    void addPELPosInfo_HB(double _pelz);

	void addPELOriInfo(doubles _quat, double _sTime);
	void addPELOriInfo(double _sTime);

    void addPELOriInfo_HB(quat _qPel);

	void addRFOriInfo(doubles _quat, double _sTime);
	void addRFOriInfo(double _sTime);

    void addRFOriInfo_HB(quat _qRF);

	void addLFOriInfo(doubles _quat, double _sTime);
	void addLFOriInfo(double _sTime);

    void addLFOriInfo_HB(quat _qLF);

	void addRHOriInfo(doubles _quat, double _sTime);
	void addRHOriInfo(double _sTime);
	void addLHOriInfo(doubles _quat, double _sTime);
	void addLHOriInfo(double _sTime);

    void addRElbPosInfo(double _angle, double _sTime);
    void addRElbPosInfo(double _sTime);
    void addLElbPosInfo(double _angle, double _sTime);
    void addLElbPosInfo(double _sTime);

	void updateAll();
    void updateAll_HB();
    void WBIK();
    void WBIK_xy_pelz();
    LegJoints WBIK2(quat _qRF, quat _qLF);
    //double limit_Qd(double Qd, double Qdd, double Qd_max, double dt);


};




#endif // TASKMOTION_H

