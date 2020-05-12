#ifndef ROBOTSTATEESTIMATOR_H
#define ROBOTSTATEESTIMATOR_H

#include <cmatrix>
#include "BasicFiles/BasicSetting.h"

//#include <robotstateestimator_parameter_drc_simulation.h>
//#include <robotstateestimator_parameter_hyobinbot.h>
//#include <robotstateestimator_parameter_hubo2.h>
#include <robotstateestimator_parameter_Gazelle.h>

#include "../../Gazelle_Ankle_Kine/Gazelle_kine.h"


typedef techsoft::matrix<double> Matrix;

typedef struct{
    double      FK_PELVIS_POSITION[3];  // Pelvis Position xyz (Forward kinematics based)
    double      FK_PELVIS_VELOCITY[3];  // Pelvis Velocity xyz (Forward kinematics based)

    double      FK_COM_POSITION[3];     // COM Position xyz (Forward kinematics based)
    double      FK_COM_VELOCITY[3];     // COM Velocity xyz (Forward kinematics based)
    double      FK_ZMP_POSITION[3];     // ZMP Position xyz (Forward kinematics + F/T)

    double      KF_COM_POSITION[2];     // COM Position xy (DLKF estimator based)
    double      KF_COM_VELOCITY[2];     // COM Velocity xy (DLKF estimator based)
    double      KF_COM_ACCELERATION[2]; // COM Acceleration xy (DLKF estimator based)
} OUTPUT_DATA;

typedef struct{
    double      FX;
    double      FY;
    double      FZ;
    double      MX;
    double      MY;
    double      MZ;
} FT_DATA;

typedef struct{
    double      ROLL;
    double      PITCH;
    double      YAW;
} IMU_DATA;

typedef struct{
    double      JOINT_ANGLE[NO_OF_JOINTS];
    FT_DATA     FT_SENSOR_R;
    FT_DATA     FT_SENSOR_L;
    IMU_DATA    IMU_SENSOR;
} INPUT_DATA;

class RobotStateEstimator
{
public:
//    pRBCORE_SHM_SENSOR  sharedSEN;
//    pRBCORE_SHM         sharedSEN;

    RobotStateEstimator();

    OUTPUT_DATA         OUTPUT;

    Gazelle_Kine        GK;

    // Functions
    void                READ_DATA_FROM_SENSOR(void);
    void                READ_DATA_FROM_FILE(int idx);
    void                READ_DATA_FROM_MANUAL_INPUT(INPUT_DATA _input);
    void                HSE_ESTIMATOR_ONOFF(bool _onoff, int _mode);
    void                HSE_ESTIMATOR(void);

    inline Matrix       Rotation_Matrix_X(double angle);
    inline Matrix       Rotation_Matrix_Y(double angle);
    inline Matrix       Rotation_Matrix_Z(double angle);

    // For the File-Read function
    Matrix              FILE_LOAD(char* filename, int n);
    char                *FILENAME;
    unsigned int        FILE_MAX_SIZE;
    Matrix              FILEIN;
    unsigned int        FILE_READ_IDX;
    unsigned int        FILE_ROW_NUM;
    unsigned int        FILE_COL_NUM;

    // Constant
    unsigned int        _nn;            // number of state
    unsigned int        _mm;            // number of measurement
    double              _g;             // gravity const
    double              _L;             // LIPM height
    double              _dt;            // discrete time gap
    double              _D2R;           // DEG to RAD
    double              _R2D;           // RAD to DEG
    double              _PI;            // pi

    // Sensor variable
    double              IN_RHY;
    double              IN_RHR;
    double              IN_RHP;
    double              IN_RKN;
    double              IN_RAP;
    double              IN_RAR;
    double              IN_LHY;
    double              IN_LHR;
    double              IN_LHP;
    double              IN_LKN;
    double              IN_LAP;
    double              IN_LAR;

    double              IN_RHY_PREV;
    double              IN_RHR_PREV;
    double              IN_RHP_PREV;
    double              IN_RKN_PREV;
    double              IN_RAP_PREV;
    double              IN_RAR_PREV;
    double              IN_LHY_PREV;
    double              IN_LHR_PREV;
    double              IN_LHP_PREV;
    double              IN_LKN_PREV;
    double              IN_LAP_PREV;
    double              IN_LAR_PREV;

    double              IN_WST;
    double              IN_WST_PREV;

    double              IN_RSP;
    double              IN_RSR;
    double              IN_RSY;
    double              IN_REB;
    double              IN_RWY;
    double              IN_RWP;
    double              IN_RF1;
    double              IN_LSP;
    double              IN_LSR;
    double              IN_LSY;
    double              IN_LEB;
    double              IN_LWY;
    double              IN_LWP;
    double              IN_LF1;

    double              IN_RSP_PREV;
    double              IN_RSR_PREV;
    double              IN_RSY_PREV;
    double              IN_REB_PREV;
    double              IN_RWY_PREV;
    double              IN_RWP_PREV;
    double              IN_RF1_PREV;
    double              IN_LSP_PREV;
    double              IN_LSR_PREV;
    double              IN_LSY_PREV;
    double              IN_LEB_PREV;
    double              IN_LWY_PREV;
    double              IN_LWP_PREV;
    double              IN_LF1_PREV;

    double              IN_RFT_FX;
    double              IN_RFT_FY;
    double              IN_RFT_FZ;
    double              IN_RFT_MX;
    double              IN_RFT_MY;
    double              IN_RFT_MZ;
    double              IN_LFT_FX;
    double              IN_LFT_FY;
    double              IN_LFT_FZ;
    double              IN_LFT_MX;
    double              IN_LFT_MY;
    double              IN_LFT_MZ;

    double              IN_IMU_X;
    double              IN_IMU_Y;
    double              IN_IMU_Z;
    double              IN_IMU_Z_OFFSET;

    // Matrix variable for general purpose
    Matrix              EYE_3X3;
    Matrix              EYE_8X8;
    Matrix              ZERO_3X1;
    Matrix              ZERO_2X1;
    Matrix              ZERO_NXN;
    Matrix              ZERO_NX1;
    Matrix              YINV_3X3;

    Matrix              TEMP1_3X1;
    Matrix              TEMP2_3X1;
    Matrix              TEMP3_3X1;

    // Matrix variable for Kalman Filter
    Matrix              A;
    Matrix              trA;
    Matrix              H;
    Matrix              trH;
    Matrix              Q;
    Matrix              R;

    Matrix              P;
    Matrix              Pp;
    Matrix              tempP;

    Matrix              ResiCOV;
    Matrix              ivResiCOV;
    Matrix              K;

    Matrix              x;
    Matrix              xp;
    Matrix              y;
    Matrix              yeq;

    // Kinematic constant
    double              P2H1;
    double              H12H2Y;
    double              H12H2Z;
    double              ULEG;
    double              LLEG;
    double              AP2AR;
    double              A2F;

    double              P2SC;
    double              SC2S;
    double              UARM;
    double              LARM;
    double              OFFELB;

    double              FOOT_LX1;
    double              FOOT_LX2;
    double              FOOT_LY1;
    double              FOOT_LY2;

    Matrix              offset_pel2sc;
    Matrix              offset_sc2rs;
    Matrix              offset_sc2ls;
    Matrix              offset_s2eb;
    Matrix              offset_eb2w;

    Matrix              offset_pel2rh1;
    Matrix              offset_pel2lh1;
    Matrix              offset_rh12rh2;
    Matrix              offset_lh12lh2;
    Matrix              offset_h22kn;
    Matrix              offset_kn2an1;
    Matrix              offset_an12an2;
    Matrix              offset_an22ft;

    // Mass properties
    double              m_rhy;
    double              m_rhr;
    double              m_rhp;
    double              m_rkn;
    double              m_rap;
    double              m_rar;

    double              m_lhy;
    double              m_lhr;
    double              m_lhp;
    double              m_lkn;
    double              m_lap;
    double              m_lar;

    double              m_rsp;
    double              m_rsr;
    double              m_rsy;
    double              m_reb;
    double              m_rwy;
    double              m_rwp;
    double              m_rf1;

    double              m_lsp;
    double              m_lsr;
    double              m_lsy;
    double              m_leb;
    double              m_lwy;
    double              m_lwp;
    double              m_lf1;

    double              m_torso;
    double              m_pel;

    double              m_lb;
    double              m_ub;
    double              m_tot;

    Matrix              c_rsp;
    Matrix              c_rsr;
    Matrix              c_rsy;
    Matrix              c_reb;
    Matrix              c_rwy;
    Matrix              c_rwp;
    Matrix              c_rf1;

    Matrix              c_lsp;
    Matrix              c_lsr;
    Matrix              c_lsy;
    Matrix              c_leb;
    Matrix              c_lwy;
    Matrix              c_lwp;
    Matrix              c_lf1;

    Matrix              c_torso;
    Matrix              c_pel;

    Matrix              c_rhy;
    Matrix              c_rhr;
    Matrix              c_rhp;
    Matrix              c_rkn;
    Matrix              c_rap;
    Matrix              c_rar;

    Matrix              c_lhy;
    Matrix              c_lhr;
    Matrix              c_lhp;
    Matrix              c_lkn;
    Matrix              c_lap;
    Matrix              c_lar;

    // Rotiation Matrix
    Matrix              R_IMU_X;
    Matrix              R_IMU_Y;
    Matrix              R_IMU_Z;
    Matrix              R_IMU_GLOBAL;

    Matrix              R_PEL;

    Matrix              R_RHY;
    Matrix              R_RHR;
    Matrix              R_RHP;
    Matrix              R_RKN;
    Matrix              R_RAP;
    Matrix              R_RAR;

    Matrix              R_LHY;
    Matrix              R_LHR;
    Matrix              R_LHP;
    Matrix              R_LKN;
    Matrix              R_LAP;
    Matrix              R_LAR;

    Matrix              R_WST;

    Matrix              R_RSP;
    Matrix              R_RSR;
    Matrix              R_RSY;
    Matrix              R_REB;
    Matrix              R_RWY;
    Matrix              R_RWP;
    Matrix              R_RF1;

    Matrix              R_LSP;
    Matrix              R_LSR;
    Matrix              R_LSY;
    Matrix              R_LEB;
    Matrix              R_LWY;
    Matrix              R_LWP;
    Matrix              R_LF1;

    Matrix              R_RH1;
    Matrix              R_RHIP;
    Matrix              R_RKNEE;
    Matrix              R_RFOOT;

    Matrix              R_LH1;
    Matrix              R_LHIP;
    Matrix              R_LKNEE;
    Matrix              R_LFOOT;

    Matrix              R_TORSO;

    Matrix              R_RSHD;
    Matrix              R_RELB;
    Matrix              R_RHAND;

    Matrix              R_LSHD;
    Matrix              R_LELB;
    Matrix              R_LHAND;

    // Important Vector
    Matrix              tch_point;
    Matrix              tch_point_delta;
    Matrix              tch_cal_pel_cur;
    Matrix              tch_cal_pel_prev;

    Matrix              PEL_POS;
    Matrix              PEL_VEL_DMETHOD;
    Matrix              PEL_KF_STATE;
    Matrix              PEL_KF_MEAS;
    Matrix              PEL_KF_GAIN_K;
    Matrix              PEL_KF_GAIN_A_KHA;
    Matrix              ZMP_POS1;
    Matrix              ZMP_POS2;
    Matrix              ZMP_OUTPUT;
    Matrix              FOOT_POS_R;
    Matrix              FOOT_POS_L;
    Matrix              COM_POS;
    Matrix              COM_WRT_POS;
    Matrix              COM_POS_PREV;
    Matrix              COM_VEL_DMETHOD;
    Matrix              COM_VEL_KF_STATE;
    Matrix              COM_VEL_KF_MEAS;
    Matrix              COM_VEL_KF_GAIN_K;
    Matrix              COM_VEL_KF_GAIN_A_KHA;
    Matrix              COM_WRT_ZMP;
    Matrix              COM_WRT_ZMP_INIT;
    Matrix              GLOBAL_FT_FORCE_SUM;

    // Position Vector
    Matrix              p_pel;

    Matrix              p_rh1;
    Matrix              p_rh2;
    Matrix              p_rkn;
    Matrix              p_ran1;
    Matrix              p_ran2;
    Matrix              p_rf;
    Matrix              p_r_zmp;
    Matrix              p_r_zmp_prev;

    Matrix              p_lh1;
    Matrix              p_lh2;
    Matrix              p_lkn;
    Matrix              p_lan1;
    Matrix              p_lan2;
    Matrix              p_lf;
    Matrix              p_l_zmp;
    Matrix              p_l_zmp_prev;

    Matrix              p_dsp_zmp;
    Matrix              p_dsp_zmp_prev;

    Matrix              p_sc;

    Matrix              p_rs;
    Matrix              p_reb;
    Matrix              p_rw;

    Matrix              p_ls;
    Matrix              p_leb;
    Matrix              p_lw;

    Matrix              pm_PEL;

    Matrix              pm_RHY;
    Matrix              pm_RHR;
    Matrix              pm_RHP;
    Matrix              pm_RKN;
    Matrix              pm_RAP;
    Matrix              pm_RAR;

    Matrix              pm_LHY;
    Matrix              pm_LHR;
    Matrix              pm_LHP;
    Matrix              pm_LKN;
    Matrix              pm_LAP;
    Matrix              pm_LAR;

    Matrix              pm_TORSO;

    Matrix              pm_RSP;
    Matrix              pm_RSR;
    Matrix              pm_RSY;
    Matrix              pm_REB;
    Matrix              pm_RWY;
    Matrix              pm_RWP;
    Matrix              pm_RF1;

    Matrix              pm_LSP;
    Matrix              pm_LSR;
    Matrix              pm_LSY;
    Matrix              pm_LEB;
    Matrix              pm_LWY;
    Matrix              pm_LWP;
    Matrix              pm_LF1;

    Matrix              pCOM_LB;
    Matrix              pCOM_UB;
    Matrix              pCOM_TOT;

    // Joint Velocity estimator
    Matrix              JKA;
    Matrix              JKK;
    Matrix              JY;

    Matrix              J_RHY;
    Matrix              J_RHR;
    Matrix              J_RHP;
    Matrix              J_RKN;
    Matrix              J_RAP;
    Matrix              J_RAR;

    Matrix              J_LHY;
    Matrix              J_LHR;
    Matrix              J_LHP;
    Matrix              J_LKN;
    Matrix              J_LAP;
    Matrix              J_LAR;

    Matrix              J_WST;

    Matrix              J_RSP;
    Matrix              J_RSR;
    Matrix              J_RSY;
    Matrix              J_REB;
    Matrix              J_RWY;
    Matrix              J_RWP;
    Matrix              J_RF1;

    Matrix              J_LSP;
    Matrix              J_LSR;
    Matrix              J_LSY;
    Matrix              J_LEB;
    Matrix              J_LWY;
    Matrix              J_LWP;
    Matrix              J_LF1;

    // Others
    bool                HSE_ONOFF;
    bool                HSE_RUN_FIRST;
    char                HSE_MODE;

    Matrix              local_zmp_R;
    Matrix              local_zmp_L;
    Matrix              local_zmp_R_prev;
    Matrix              local_zmp_L_prev;
    double              local_zmp_weight;
    double              local_zmp_weight_prev;
    double              CONTACT_TH;
    int                 CONTACT_STATUS_ESTI_CUR;
    int                 CONTACT_STATUS_ESTI_PREV;


};

#endif // ROBOTSTATEESTIMATOR_H
