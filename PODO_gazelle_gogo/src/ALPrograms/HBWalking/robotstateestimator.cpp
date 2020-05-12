#include "robotstateestimator.h"
#include "iostream"
#include "fstream"


RobotStateEstimator::RobotStateEstimator()
{
    FILE_MAX_SIZE = 1000000;
    FILE_READ_IDX = 0;
    FILE_ROW_NUM = 0;
    FILE_COL_NUM = 0;

    _nn = 8;
    _mm = 8;
    _g = 9.812;
    _L = CV_LIPM_H;
    _dt = 0.005;
    _D2R = 0.017453292519943;
    _R2D = 57.295779513082323;
    _PI = 3.141592653589793;

    EYE_3X3 = Matrix(3, 3, 0.0);
    for(int k=0;k<3;k++)
        EYE_3X3(k,k) = 1.;
    EYE_8X8 = Matrix(8, 8, 0.0);
    for(int k=0;k<8;k++)
        EYE_8X8(k,k) = 1.;
    ZERO_3X1 = Matrix(3, 1, 0.0);
    ZERO_2X1 = Matrix(2, 1, 0.0);
    ZERO_NXN = Matrix(_nn, _nn, 0.0);
    ZERO_NX1 = Matrix(_nn, 1, 0.0);
    YINV_3X3 = Matrix(3, 3, 0.0);
    YINV_3X3(0, 0) = 1.;
    YINV_3X3(1, 1) = -1.;
    YINV_3X3(2, 2) = 1.;

    TEMP1_3X1 = Matrix(3, 1, 0.0);
    TEMP2_3X1 = Matrix(3, 1, 0.0);
    TEMP3_3X1 = Matrix(3, 1, 0.0);

    A = Matrix(_nn, _nn, 0.0);
    trA = Matrix(_nn, _nn, 0.0);
    H = Matrix(_mm, _nn, 0.0);
    trH = Matrix(_nn, _mm, 0.0);
    Q = Matrix(_nn, _nn, 0.0);
    R = Matrix(_mm, _mm, 0.0);

    P = Matrix(_nn, _nn, 0.0);
    Pp = Matrix(_nn, _nn, 0.0);
    tempP = Matrix(_nn, _nn, 0.0);

    ResiCOV = Matrix(_mm, _mm, 0.0);
    ivResiCOV = Matrix(_mm, _mm, 0.0);
    K = Matrix(_nn, _mm, 0.0);

    x = Matrix(_nn, 1, 0.0);
    xp = Matrix(_nn, 1, 0.0);
    y = Matrix(_mm, 1, 0.0);
    yeq = Matrix(2, 1, 0.0);

    P2H1 = CV_LINK_L_P2H1;
    H12H2Y = CV_LINK_L_H12H2Y;
    H12H2Z = CV_LINK_L_H12H2Z;
    ULEG = CV_LINK_L_ULEG;
    LLEG = CV_LINK_L_LLEG;
    AP2AR = CV_LINK_L_AP2AR;
    A2F = CV_LINK_L_A2F;

    P2SC = CV_LINK_L_P2SC;
    SC2S = CV_LINK_L_SC2S;
    UARM = CV_LINK_L_UARM;
    LARM = CV_LINK_L_LARM;
    OFFELB = CV_LINK_L_OFFELB;

    FOOT_LX1 = CV_LINK_L_FOOT_LX_1;
    FOOT_LX2 = CV_LINK_L_FOOT_LX_2;
    FOOT_LY1 = CV_LINK_L_FOOT_LY_1;
    FOOT_LY2 = CV_LINK_L_FOOT_LY_2;

    double t_offset1[] = {0,0,P2SC};
    offset_pel2sc = Matrix(3,1, t_offset1);
    double t_offset2[] = {0,-SC2S,0};
    offset_sc2rs = Matrix(3,1, t_offset2);
    double t_offset3[] = {0,+SC2S,0};
    offset_sc2ls = Matrix(3,1, t_offset3);
    double t_offset4[] = {OFFELB,0,-UARM};
    offset_s2eb = Matrix(3,1, t_offset4);
    double t_offset5[] = {-OFFELB,0,-LARM};
    offset_eb2w = Matrix(3,1, t_offset5);

    double t_offset6[] = {0,-P2H1,0};
    offset_pel2rh1 = Matrix(3,1, t_offset6);
    double t_offset7[] = {0,+P2H1,0};
    offset_pel2lh1 = Matrix(3,1, t_offset7);
    double t_offset8[] = {0., -H12H2Y, -H12H2Z};
    offset_rh12rh2 = Matrix(3,1, t_offset8);
    double t_offset9[] = {0., +H12H2Y, -H12H2Z};
    offset_lh12lh2 = Matrix(3,1, t_offset9);
    double t_offset10[] = {0,0,-ULEG};
    offset_h22kn = Matrix(3,1, t_offset10);
    double t_offset11[] = {0,0,-LLEG};
    offset_kn2an1 = Matrix(3,1, t_offset11);
    double t_offset12[] = {0,0,-AP2AR};
    offset_an12an2 = Matrix(3,1, t_offset12);
    double t_offset13[] = {0,0,-A2F};
    offset_an22ft = Matrix(3,1, t_offset13);

    m_rhy = CV_MASS_M_HY;
    m_rhr = CV_MASS_M_HR;
    m_rhp = CV_MASS_M_HP;
    m_rkn = CV_MASS_M_KN;
    m_rap = CV_MASS_M_AP;
    m_rar = CV_MASS_M_AR;

    m_rsp = CV_MASS_M_SP;
    m_rsr = CV_MASS_M_SR;
    m_rsy = CV_MASS_M_SY;
    m_reb = CV_MASS_M_EB;
    m_rwy = CV_MASS_M_WY;
    m_rwp = CV_MASS_M_WP;
    m_rf1 = CV_MASS_M_F1;

    m_lhy = m_rhy;
    m_lhr = m_rhr;
    m_lhp = m_rhp;
    m_lkn = m_rkn;
    m_lap = m_rap;
    m_lar = m_rar;

    m_lsp = m_rsp;
    m_lsr = m_rsr;
    m_lsy = m_rsy;
    m_leb = m_reb;
    m_lwy = m_rwy;
    m_lwp = m_rwp;
    m_lf1 = m_rf1;

    m_torso = CV_MASS_M_TORSO;
    m_pel = CV_MASS_M_PEL;

    m_lb = m_rhy + m_rhr + m_rhp + m_rkn + m_rap + m_rar + m_lhy + m_lhr + m_lhp + m_lkn + m_lap + m_lar;
    m_ub = m_pel+m_torso + m_rsp + m_rsr + m_rsy + m_reb + m_rwy + m_rwp + m_rf1 + m_lsp + m_lsr + m_lsy + m_leb + m_lwy + m_lwp + m_lf1;
    m_tot = m_lb + m_ub;

    double c_offset1[] = CV_MCL_L_RSP;
    c_rsp = Matrix(3,1, c_offset1);
    double c_offset2[] = CV_MCL_L_RSR;
    c_rsr = Matrix(3,1, c_offset2);
    double c_offset3[] = CV_MCL_L_RSY;
    c_rsy = Matrix(3,1, c_offset3);
    double c_offset4[] = CV_MCL_L_REB;
    c_reb = Matrix(3,1, c_offset4);
    double c_offset5[] = CV_MCL_L_RWY;
    c_rwy = Matrix(3,1, c_offset5);
    double c_offset6[] = CV_MCL_L_RWP;
    c_rwp = Matrix(3,1, c_offset6);
    double c_offset7[] = CV_MCL_L_RF1;
    c_rf1 = Matrix(3,1, c_offset7);

    c_lsp = Matrix(3,1, 0.0);
    c_lsp = YINV_3X3*c_rsp;
    c_lsr = Matrix(3,1, 0.0);
    c_lsr = YINV_3X3*c_rsr;
    c_lsy = Matrix(3,1, 0.0);
    c_lsy = YINV_3X3*c_rsy;
    c_leb = Matrix(3,1, 0.0);
    c_leb = YINV_3X3*c_reb;
    c_lwy = Matrix(3,1, 0.0);
    c_lwy = YINV_3X3*c_rwy;
    c_lwp = Matrix(3,1, 0.0);
    c_lwp = YINV_3X3*c_rwp;
    c_lf1 = Matrix(3,1, 0.0);
    c_lf1 = YINV_3X3*c_rf1;

    double c_offset15[] = CV_MCL_L_TORSO;
    c_torso = Matrix(3,1, c_offset15);
    double c_offset16[] = CV_MCL_L_PEL;
    c_pel = Matrix(3,1, c_offset16);

    double c_offset17[] = CV_MCL_L_RHY;
    c_rhy = Matrix(3,1, c_offset17);
    double c_offset18[] = CV_MCL_L_RHR;
    c_rhr = Matrix(3,1, c_offset18);
    double c_offset19[] = CV_MCL_L_RHP;
    c_rhp = Matrix(3,1, c_offset19);
    double c_offset20[] = CV_MCL_L_RKN;
    c_rkn = Matrix(3,1, c_offset20);
    double c_offset21[] = CV_MCL_L_RAP;
    c_rap = Matrix(3,1, c_offset21);
    double c_offset22[] = CV_MCL_L_RAR;
    c_rar = Matrix(3,1, c_offset22);

    c_lhy = Matrix(3,1, 0.0);
    c_lhy = YINV_3X3*c_rhy;
    c_lhr = Matrix(3,1, 0.0);
    c_lhr = YINV_3X3*c_rhr;
    c_lhp = Matrix(3,1, 0.0);
    c_lhp = YINV_3X3*c_rhp;
    c_lkn = Matrix(3,1, 0.0);
    c_lkn = YINV_3X3*c_rkn;
    c_lap = Matrix(3,1, 0.0);
    c_lap = YINV_3X3*c_rap;
    c_lar = Matrix(3,1, 0.0);
    c_lar = YINV_3X3*c_rar;

    R_IMU_X = Matrix(3, 3, 0.0);
    R_IMU_Y = Matrix(3, 3, 0.0);
    R_IMU_Z = Matrix(3, 3, 0.0);
    R_IMU_GLOBAL = Matrix(3, 3, 0.0);

    R_PEL = Matrix(3, 3, 0.0);

    R_RHY = Matrix(3, 3, 0.0);
    R_RHR = Matrix(3, 3, 0.0);
    R_RHP = Matrix(3, 3, 0.0);
    R_RKN = Matrix(3, 3, 0.0);
    R_RAP = Matrix(3, 3, 0.0);
    R_RAR = Matrix(3, 3, 0.0);

    R_LHY = Matrix(3, 3, 0.0);
    R_LHR = Matrix(3, 3, 0.0);
    R_LHP = Matrix(3, 3, 0.0);
    R_LKN = Matrix(3, 3, 0.0);
    R_LAP = Matrix(3, 3, 0.0);
    R_LAR = Matrix(3, 3, 0.0);

    R_WST = Matrix(3, 3, 0.0);

    R_RSP = Matrix(3, 3, 0.0);
    R_RSR = Matrix(3, 3, 0.0);
    R_RSY = Matrix(3, 3, 0.0);
    R_REB = Matrix(3, 3, 0.0);
    R_RWY = Matrix(3, 3, 0.0);
    R_RWP = Matrix(3, 3, 0.0);
    R_RF1 = Matrix(3, 3, 0.0);

    R_LSP = Matrix(3, 3, 0.0);
    R_LSR = Matrix(3, 3, 0.0);
    R_LSY = Matrix(3, 3, 0.0);
    R_LEB = Matrix(3, 3, 0.0);
    R_LWY = Matrix(3, 3, 0.0);
    R_LWP = Matrix(3, 3, 0.0);
    R_LF1 = Matrix(3, 3, 0.0);

    R_RH1 = Matrix(3, 3, 0.0);
    R_RHIP = Matrix(3, 3, 0.0);
    R_RKNEE = Matrix(3, 3, 0.0);
    R_RFOOT = Matrix(3, 3, 0.0);

    R_LH1 = Matrix(3, 3, 0.0);
    R_LHIP = Matrix(3, 3, 0.0);
    R_LKNEE = Matrix(3, 3, 0.0);
    R_LFOOT = Matrix(3, 3, 0.0);

    R_TORSO = Matrix(3, 3, 0.0);

    R_RSHD = Matrix(3, 3, 0.0);
    R_RELB = Matrix(3, 3, 0.0);
    R_RHAND = Matrix(3, 3, 0.0);

    R_LSHD = Matrix(3, 3, 0.0);
    R_LELB = Matrix(3, 3, 0.0);
    R_LHAND = Matrix(3, 3, 0.0);

    tch_point = Matrix(3, 1, 0.0);
    tch_point_delta = Matrix(3, 1, 0.0);
    tch_cal_pel_cur = Matrix(3, 1, 0.0);
    tch_cal_pel_prev = Matrix(3, 1, 0.0);

    PEL_POS = Matrix(3, 1, 0.0);
    PEL_VEL_DMETHOD = Matrix(3, 1, 0.0);
    PEL_KF_STATE = Matrix(6, 1, 0.0);
    PEL_KF_MEAS = Matrix(6, 1, 0.0);
    PEL_KF_GAIN_K = Matrix(6, 6, 0.0);
    PEL_KF_GAIN_K(0,0) = PEL_KF_GAIN_K(1,1) = PEL_KF_GAIN_K(2,2) = 0.548625846017227;
    PEL_KF_GAIN_K(0,3) = PEL_KF_GAIN_K(1,4) = PEL_KF_GAIN_K(2,5) = 0.000634074459731120;
    PEL_KF_GAIN_K(3,0) = PEL_KF_GAIN_K(4,1) = PEL_KF_GAIN_K(5,2) = 0.0105679076621853;
    PEL_KF_GAIN_K(3,3) = PEL_KF_GAIN_K(4,4) = PEL_KF_GAIN_K(5,5) = 0.245622689172559;
    PEL_KF_GAIN_A_KHA = Matrix(6, 6, 0.0);
    PEL_KF_GAIN_A_KHA(0,0) = PEL_KF_GAIN_A_KHA(1,1) = PEL_KF_GAIN_A_KHA(2,2) = 0.451374153982773;
    PEL_KF_GAIN_A_KHA(0,3) = PEL_KF_GAIN_A_KHA(1,4) = PEL_KF_GAIN_A_KHA(2,5) = 0.00162279631018274;
    PEL_KF_GAIN_A_KHA(3,0) = PEL_KF_GAIN_A_KHA(4,1) = PEL_KF_GAIN_A_KHA(5,2) = -0.0105679076621853;
    PEL_KF_GAIN_A_KHA(3,3) = PEL_KF_GAIN_A_KHA(4,4) = PEL_KF_GAIN_A_KHA(5,5) = 0.754324471289130;
    ZMP_POS1 = Matrix(3, 1, 0.0);
    ZMP_POS2 = Matrix(3, 1, 0.0);
    ZMP_OUTPUT = Matrix(3, 1, 0.0);
    FOOT_POS_R = Matrix(3, 1, 0.0);
    FOOT_POS_L = Matrix(3, 1, 0.0);
    COM_POS = Matrix(3, 1, 0.0);
    COM_WRT_POS = Matrix(3, 1, 0.0);
    COM_POS_PREV = Matrix(3, 1, 0.0);
    COM_VEL_DMETHOD = Matrix(3, 1, 0.0);
    COM_VEL_KF_STATE = Matrix(6, 1, 0.0);
    COM_VEL_KF_MEAS = Matrix(6, 1, 0.0);
    COM_VEL_KF_GAIN_K = Matrix(6,6, 0.0);
    COM_VEL_KF_GAIN_K = PEL_KF_GAIN_K;
    COM_VEL_KF_GAIN_A_KHA = Matrix(6,6, 0.0);
    COM_VEL_KF_GAIN_A_KHA = PEL_KF_GAIN_A_KHA;
    COM_WRT_ZMP = Matrix(3, 1, 0.0);
    COM_WRT_ZMP_INIT = Matrix(3, 1, 0.0);
    GLOBAL_FT_FORCE_SUM = Matrix(3, 1, 0.0);

    p_pel = Matrix(3, 1, 0.0);

    p_rh1 = Matrix(3, 1, 0.0);
    p_rh2 = Matrix(3, 1, 0.0);
    p_rkn = Matrix(3, 1, 0.0);
    p_ran1 = Matrix(3, 1, 0.0);
    p_ran2 = Matrix(3, 1, 0.0);
    p_rf = Matrix(3, 1, 0.0);
    p_r_zmp = Matrix(3, 1, 0.0);
    p_r_zmp_prev = Matrix(3, 1, 0.0);

    p_lh1 = Matrix(3, 1, 0.0);
    p_lh2 = Matrix(3, 1, 0.0);
    p_lkn = Matrix(3, 1, 0.0);
    p_lan1 = Matrix(3, 1, 0.0);
    p_lan2 = Matrix(3, 1, 0.0);
    p_lf = Matrix(3, 1, 0.0);
    p_l_zmp = Matrix(3, 1, 0.0);
    p_l_zmp_prev = Matrix(3, 1, 0.0);

    p_dsp_zmp = Matrix(3, 1, 0.0);
    p_dsp_zmp_prev = Matrix(3, 1, 0.0);

    p_sc = Matrix(3, 1, 0.0);

    p_rs = Matrix(3, 1, 0.0);
    p_reb = Matrix(3, 1, 0.0);
    p_rw = Matrix(3, 1, 0.0);

    p_ls = Matrix(3, 1, 0.0);
    p_leb = Matrix(3, 1, 0.0);
    p_lw = Matrix(3, 1, 0.0);

    pm_PEL = Matrix(3, 1, 0.0);

    pm_RHY = Matrix(3, 1, 0.0);
    pm_RHR = Matrix(3, 1, 0.0);
    pm_RHP = Matrix(3, 1, 0.0);
    pm_RKN = Matrix(3, 1, 0.0);
    pm_RAP = Matrix(3, 1, 0.0);
    pm_RAR = Matrix(3, 1, 0.0);

    pm_LHY = Matrix(3, 1, 0.0);
    pm_LHR = Matrix(3, 1, 0.0);
    pm_LHP = Matrix(3, 1, 0.0);
    pm_LKN = Matrix(3, 1, 0.0);
    pm_LAP = Matrix(3, 1, 0.0);
    pm_LAR = Matrix(3, 1, 0.0);

    pm_TORSO = Matrix(3, 1, 0.0);

    pm_RSP = Matrix(3, 1, 0.0);
    pm_RSR = Matrix(3, 1, 0.0);
    pm_RSY = Matrix(3, 1, 0.0);
    pm_REB = Matrix(3, 1, 0.0);
    pm_RWY = Matrix(3, 1, 0.0);
    pm_RWP = Matrix(3, 1, 0.0);
    pm_RF1 = Matrix(3, 1, 0.0);

    pm_LSP = Matrix(3, 1, 0.0);
    pm_LSR = Matrix(3, 1, 0.0);
    pm_LSY = Matrix(3, 1, 0.0);
    pm_LEB = Matrix(3, 1, 0.0);
    pm_LWY = Matrix(3, 1, 0.0);
    pm_LWP = Matrix(3, 1, 0.0);
    pm_LF1 = Matrix(3, 1, 0.0);

    pCOM_LB = Matrix(3, 1, 0.0);
    pCOM_UB = Matrix(3, 1, 0.0);
    pCOM_TOT = Matrix(3, 1, 0.0);

    double temp4L_1[] = {1.,_dt,0.,1.};
    JKA = Matrix(2,2, temp4L_1);
    double temp4L_2[] = {0.096830869013954, 0.002357646278903, 0.846470589607076, 0.395953843235881};
    JKK = Matrix(2,2, temp4L_2);
    JY = Matrix(2,1, 0.0);

    J_RHY = Matrix(2, 1, 0.0);
    J_RHR = Matrix(2, 1, 0.0);
    J_RHP = Matrix(2, 1, 0.0);
    J_RKN = Matrix(2, 1, 0.0);
    J_RAP = Matrix(2, 1, 0.0);
    J_RAR = Matrix(2, 1, 0.0);

    J_LHY = Matrix(2, 1, 0.0);
    J_LHR = Matrix(2, 1, 0.0);
    J_LHP = Matrix(2, 1, 0.0);
    J_LKN = Matrix(2, 1, 0.0);
    J_LAP = Matrix(2, 1, 0.0);
    J_LAR = Matrix(2, 1, 0.0);

    J_WST = Matrix(2, 1, 0.0);

    J_RSP = Matrix(2, 1, 0.0);
    J_RSR = Matrix(2, 1, 0.0);
    J_RSY = Matrix(2, 1, 0.0);
    J_REB = Matrix(2, 1, 0.0);
    J_RWY = Matrix(2, 1, 0.0);
    J_RWP = Matrix(2, 1, 0.0);
    J_RF1 = Matrix(2, 1, 0.0);

    J_LSP = Matrix(2, 1, 0.0);
    J_LSR = Matrix(2, 1, 0.0);
    J_LSY = Matrix(2, 1, 0.0);
    J_LEB = Matrix(2, 1, 0.0);
    J_LWY = Matrix(2, 1, 0.0);
    J_LWP = Matrix(2, 1, 0.0);
    J_LF1 = Matrix(2, 1, 0.0);

    HSE_ONOFF = false;
    HSE_RUN_FIRST = false;
    HSE_MODE = 0;

    local_zmp_R = Matrix(3, 1, 0.0);
    local_zmp_L = Matrix(3, 1, 0.0);
    local_zmp_R_prev = Matrix(3, 1, 0.0);
    local_zmp_L_prev = Matrix(3, 1, 0.0);
    local_zmp_weight = 0.;
    local_zmp_weight_prev = 0.;

    CONTACT_TH = CV_CONTACT_TH_FORCE;
    CONTACT_STATUS_ESTI_CUR = 0;
    CONTACT_STATUS_ESTI_PREV = 0;

    IN_IMU_Z_OFFSET = 0.;
}


void RobotStateEstimator::READ_DATA_FROM_SENSOR(void){
//    IN_RHY = sharedSEN->ENCODER[MC_GetID(RHY)][MC_GetCH(RHY)].CurrentReference;
//    IN_RHY = sharedSEN->ENCODER[MC_GetID(RHY)][MC_GetCH(RHY)].CurrentReference;
    IN_RHY = sharedSEN->ENCODER[joint->Joints[RHR]->MCId][joint->Joints[RHR]->MCCh].CurrentPosition;
    IN_RHR = sharedSEN->ENCODER[joint->Joints[RHR]->MCId][joint->Joints[RHR]->MCCh].CurrentPosition;
    IN_RHP = sharedSEN->ENCODER[joint->Joints[RHP]->MCId][joint->Joints[RHP]->MCCh].CurrentPosition;
    IN_RKN = sharedSEN->ENCODER[joint->Joints[RKN]->MCId][joint->Joints[RKN]->MCCh].CurrentPosition;
    IN_RAP = sharedSEN->ENCODER[joint->Joints[RAP]->MCId][joint->Joints[RAP]->MCCh].CurrentPosition;
    IN_RAR = sharedSEN->ENCODER[joint->Joints[RAR]->MCId][joint->Joints[RAR]->MCCh].CurrentPosition;

    IN_LHY = sharedSEN->ENCODER[joint->Joints[LHY]->MCId][joint->Joints[LHY]->MCCh].CurrentPosition;
    IN_LHR = sharedSEN->ENCODER[joint->Joints[LHR]->MCId][joint->Joints[LHR]->MCCh].CurrentPosition;
    IN_LHP = sharedSEN->ENCODER[joint->Joints[LHP]->MCId][joint->Joints[LHP]->MCCh].CurrentPosition;
    IN_LKN = sharedSEN->ENCODER[joint->Joints[LKN]->MCId][joint->Joints[LKN]->MCCh].CurrentPosition;
    IN_LAP = sharedSEN->ENCODER[joint->Joints[LAP]->MCId][joint->Joints[LAP]->MCCh].CurrentPosition;
    IN_LAR = sharedSEN->ENCODER[joint->Joints[LAR]->MCId][joint->Joints[LAR]->MCCh].CurrentPosition;

    IN_WST = sharedSEN->ENCODER[joint->Joints[WST]->MCId][joint->Joints[WST]->MCCh].CurrentPosition;

    IN_RSP = 0;//sharedSEN->ENCODER[joint->Joints[RSP]->MCId][joint->Joints[RSP]->MCCh].CurrentPosition;
    IN_RSR = 0;//sharedSEN->ENCODER[joint->Joints[RSR]->MCId][joint->Joints[RSR]->MCCh].CurrentPosition;
    IN_RSY = 0;//sharedSEN->ENCODER[joint->Joints[RSY]->MCId][joint->Joints[RSY]->MCCh].CurrentPosition;
    IN_REB = 0;//sharedSEN->ENCODER[joint->Joints[REB]->MCId][joint->Joints[REB]->MCCh].CurrentPosition;
    IN_RWY = 0;//sharedSEN->ENCODER[joint->Joints[RWY]->MCId][joint->Joints[RWY]->MCCh].CurrentPosition;
    IN_RWP = 0;//sharedSEN->ENCODER[joint->Joints[RWP]->MCId][joint->Joints[RWP]->MCCh].CurrentPosition;
    IN_RF1 = 0.;

    IN_LSP = 0;//sharedSEN->ENCODER[joint->Joints[LSP]->MCId][joint->Joints[LSP]->MCCh].CurrentPosition;
    IN_LSR = 0;//sharedSEN->ENCODER[joint->Joints[LSR]->MCId][joint->Joints[LSR]->MCCh].CurrentPosition;
    IN_LSY = 0;//sharedSEN->ENCODER[joint->Joints[LSY]->MCId][joint->Joints[LSY]->MCCh].CurrentPosition;
    IN_LEB = 0;//sharedSEN->ENCODER[joint->Joints[LEB]->MCId][joint->Joints[LEB]->MCCh].CurrentPosition;
    IN_LWY = 0;//sharedSEN->ENCODER[joint->Joints[LWY]->MCId][joint->Joints[LWY]->MCCh].CurrentPosition;
    IN_LWP = 0;//sharedSEN->ENCODER[joint->Joints[LWP]->MCId][joint->Joints[LWP]->MCCh].CurrentPosition;
    IN_LF1 = 0.;

    //for gazelle
    double RAP_deg, RAR_deg;
    GK.FK_diff_Ankle_right(IN_RAP, IN_RAR, 0, 0, RAP_deg, RAR_deg);
    IN_RAP = RAP_deg;
    IN_RAR = RAR_deg;

    double LAP_deg, LAR_deg;
    GK.FK_diff_Ankle_left(IN_LAP, IN_LAR, 0, 0, LAP_deg, LAR_deg);
    IN_LAP = LAP_deg;
    IN_LAR = LAR_deg;

    IN_RFT_FX = sharedSEN->FT[0].Fx;
    IN_RFT_FY = sharedSEN->FT[0].Fy;
    IN_RFT_FZ = sharedSEN->FT[0].Fz;
    IN_RFT_MX = sharedSEN->FT[0].Mx;
    IN_RFT_MY = sharedSEN->FT[0].My;
    IN_RFT_MZ = sharedSEN->FT[0].Mz;

    IN_LFT_FX = sharedSEN->FT[1].Fx;
    IN_LFT_FY = sharedSEN->FT[1].Fy;
    IN_LFT_FZ = sharedSEN->FT[1].Fz;
    IN_LFT_MX = sharedSEN->FT[1].Mx;
    IN_LFT_MY = sharedSEN->FT[1].My;
    IN_LFT_MZ = sharedSEN->FT[1].Mz;

    IN_IMU_X = sharedSEN->IMU[0].Roll;
    IN_IMU_Y = sharedSEN->IMU[0].Pitch;
    IN_IMU_Z = sharedSEN->IMU[0].Yaw - IN_IMU_Z_OFFSET;
}

void RobotStateEstimator::READ_DATA_FROM_FILE(int idx){
    IN_RHY = FILEIN[idx][1];
    IN_RHR = FILEIN[idx][2];
    IN_RHP = FILEIN[idx][3];
    IN_RKN = FILEIN[idx][4];
    IN_RAP = FILEIN[idx][5];
    IN_RAR = FILEIN[idx][6];
    IN_LHY = FILEIN[idx][7];
    IN_LHR = FILEIN[idx][8];
    IN_LHP = FILEIN[idx][9];
    IN_LKN = FILEIN[idx][10];
    IN_LAP = FILEIN[idx][11];
    IN_LAR = FILEIN[idx][12];

    IN_RSP = FILEIN[idx][13];
    IN_RSR = FILEIN[idx][14];
    IN_RSY = FILEIN[idx][15];
    IN_REB = FILEIN[idx][16];
    IN_RWY = FILEIN[idx][17];
    IN_RWP = FILEIN[idx][18];
    IN_RF1 = FILEIN[idx][19];
    IN_LSP = FILEIN[idx][20];
    IN_LSR = FILEIN[idx][21];
    IN_LSY = FILEIN[idx][22];
    IN_LEB = FILEIN[idx][23];
    IN_LWY = FILEIN[idx][24];
    IN_LWP = FILEIN[idx][25];
    IN_LF1 = FILEIN[idx][26];

    IN_WST = FILEIN[idx][27];

    IN_RFT_FX = FILEIN[idx][28];
    IN_RFT_FY = FILEIN[idx][29];
    IN_RFT_FZ = FILEIN[idx][30];
    IN_RFT_MX = FILEIN[idx][31];
    IN_RFT_MY = FILEIN[idx][32];
    IN_RFT_MZ = FILEIN[idx][33];
    IN_LFT_FX = FILEIN[idx][34];
    IN_LFT_FY = FILEIN[idx][35];
    IN_LFT_FZ = FILEIN[idx][36];
    IN_LFT_MX = FILEIN[idx][37];
    IN_LFT_MY = FILEIN[idx][38];
    IN_LFT_MZ = FILEIN[idx][39];

    IN_IMU_X = FILEIN[idx][40];
    IN_IMU_Y = FILEIN[idx][41];
    IN_IMU_Z = FILEIN[idx][42];
}

void RobotStateEstimator::READ_DATA_FROM_MANUAL_INPUT(INPUT_DATA _input){
    IN_RHY = _input.JOINT_ANGLE[RHY];
    IN_RHR = _input.JOINT_ANGLE[RHR];
    IN_RHP = _input.JOINT_ANGLE[RHP];
    IN_RKN = _input.JOINT_ANGLE[RKN];
    IN_RAP = _input.JOINT_ANGLE[RAP];
    IN_RAR = _input.JOINT_ANGLE[RAR];
    IN_LHY = _input.JOINT_ANGLE[LHY];
    IN_LHR = _input.JOINT_ANGLE[LHR];
    IN_LHP = _input.JOINT_ANGLE[LHP];
    IN_LKN = _input.JOINT_ANGLE[LKN];
    IN_LAP = _input.JOINT_ANGLE[LAP];
    IN_LAR = _input.JOINT_ANGLE[LAR];

    IN_RSP = _input.JOINT_ANGLE[RSP];
    IN_RSR = _input.JOINT_ANGLE[RSR];
    IN_RSY = _input.JOINT_ANGLE[RSY];
    IN_REB = _input.JOINT_ANGLE[REB];
    IN_RWY = _input.JOINT_ANGLE[RWY];
    IN_RWP = _input.JOINT_ANGLE[RWP];
    IN_RF1 = _input.JOINT_ANGLE[RF1];
    IN_LSP = _input.JOINT_ANGLE[LSP];
    IN_LSR = _input.JOINT_ANGLE[LSR];
    IN_LSY = _input.JOINT_ANGLE[LSY];
    IN_LEB = _input.JOINT_ANGLE[LEB];
    IN_LWY = _input.JOINT_ANGLE[LWY];
    IN_LWP = _input.JOINT_ANGLE[LWP];
    IN_LF1 = _input.JOINT_ANGLE[LF1];

    IN_WST = _input.JOINT_ANGLE[WST];

    IN_RFT_FX = _input.FT_SENSOR_R.FX;
    IN_RFT_FY = _input.FT_SENSOR_R.FY;
    IN_RFT_FZ = _input.FT_SENSOR_R.FZ;
    IN_RFT_MX = _input.FT_SENSOR_R.MX;
    IN_RFT_MY = _input.FT_SENSOR_R.MY;
    IN_RFT_MZ = _input.FT_SENSOR_R.MZ;
    IN_LFT_FX = _input.FT_SENSOR_L.FX;
    IN_LFT_FY = _input.FT_SENSOR_L.FY;
    IN_LFT_FZ = _input.FT_SENSOR_L.FZ;
    IN_LFT_MX = _input.FT_SENSOR_L.MX;
    IN_LFT_MY = _input.FT_SENSOR_L.MY;
    IN_LFT_MZ = _input.FT_SENSOR_L.MZ;

    IN_IMU_X = _input.IMU_SENSOR.ROLL;
    IN_IMU_Y = _input.IMU_SENSOR.PITCH;
    IN_IMU_Z = _input.IMU_SENSOR.YAW;
}

void RobotStateEstimator::HSE_ESTIMATOR_ONOFF(bool _onoff, int _mode){
    if(_mode == 0){
        // Real Mode
        HSE_MODE = 0;
        IN_IMU_Z_OFFSET = sharedSEN->IMU[0].Yaw;
        cout<<"Real Mode"<<endl;
    }else if(_mode == 1){
        // Virtual Mode
        FILE_READ_IDX = 0;
        HSE_MODE = 1;
        cout<<"Virtual Mode"<<endl;
    }
    if(_onoff == true){
        cout<<"Estimation Start...!!!"<<endl;
        HSE_RUN_FIRST = true;
        HSE_ONOFF = true;
    }else{
        HSE_RUN_FIRST = false;
        HSE_ONOFF = false;
        cout<<"Estimation Stop...!!!"<<endl;
    }
}

inline Matrix RobotStateEstimator::Rotation_Matrix_X(double angle){
    double rangle = angle*_D2R;
    double rot_arr[] = {1, 0, 0, 0, cos(rangle), -sin(rangle), 0, sin(rangle), cos(rangle)};
    Matrix returnMatrix(3,3, rot_arr);
    return returnMatrix;
}

inline Matrix RobotStateEstimator::Rotation_Matrix_Y(double angle){
    double rangle = angle*_D2R;
    double rot_arr[] = {cos(rangle), 0, sin(rangle), 0, 1, 0, -sin(rangle), 0, cos(rangle)};
    Matrix returnMatrix(3,3, rot_arr);
    return returnMatrix;
}

inline Matrix RobotStateEstimator::Rotation_Matrix_Z(double angle){
    double rangle = angle*_D2R;
    double rot_arr[] = {cos(rangle), -sin(rangle), 0, sin(rangle), cos(rangle), 0, 0, 0, 1};
    Matrix returnMatrix(3,3, rot_arr);
    return returnMatrix;
}

Matrix RobotStateEstimator::FILE_LOAD(char* filename, int n){
    using namespace std;
    Matrix traj(n,FILE_MAX_SIZE);
    ifstream file;
    file.open(filename);

    double temp;
    int cnt = 0;
    int cnt1 = 0;

    while(file >> temp)
    {
        traj(cnt%n,cnt1) = temp;
        cnt++;

        if(cnt%n == 0)
        {
            cnt1++;
        }
    }

    traj.resize(n,cnt1);

    return traj;
}

void RobotStateEstimator::HSE_ESTIMATOR(void){
    // ---------------------------------------
    // Joint Velocity Estimator
    // ---------------------------------------
    if(HSE_RUN_FIRST == true){
        J_RHY(0,0) = IN_RHY;
        J_RHY(1,0) = 0.;
        J_RHR(0,0) = IN_RHR;
        J_RHR(1,0) = 0.;
        J_RHP(0,0) = IN_RHP;
        J_RHP(1,0) = 0.;
        J_RKN(0,0) = IN_RKN;
        J_RKN(1,0) = 0.;
        J_RAP(0,0) = IN_RAP;
        J_RAP(1,0) = 0.;
        J_RAR(0,0) = IN_RAR;
        J_RAR(1,0) = 0.;

        J_LHY(0,0) = IN_LHY;
        J_LHY(1,0) = 0.;
        J_LHR(0,0) = IN_LHR;
        J_LHR(1,0) = 0.;
        J_LHP(0,0) = IN_LHP;
        J_LHP(1,0) = 0.;
        J_LKN(0,0) = IN_LKN;
        J_LKN(1,0) = 0.;
        J_LAP(0,0) = IN_LAP;
        J_LAP(1,0) = 0.;
        J_LAR(0,0) = IN_LAR;
        J_LAR(1,0) = 0.;

        J_WST(0,0) = IN_WST;
        J_WST(1,0) = 0.;

        J_RSP(0,0) = IN_RSP;
        J_RSP(1,0) = 0.;
        J_RSR(0,0) = IN_RSR;
        J_RSR(1,0) = 0.;
        J_RSY(0,0) = IN_RSY;
        J_RSY(1,0) = 0.;
        J_REB(0,0) = IN_REB;
        J_REB(1,0) = 0.;
        J_RWY(0,0) = IN_RWY;
        J_RWY(1,0) = 0.;
        J_RWP(0,0) = IN_RWP;
        J_RWP(1,0) = 0.;
        J_RF1(0,0) = IN_RF1;
        J_RF1(1,0) = 0.;

        J_LSP(0,0) = IN_LSP;
        J_LSP(1,0) = 0.;
        J_LSR(0,0) = IN_LSR;
        J_LSR(1,0) = 0.;
        J_LSY(0,0) = IN_LSY;
        J_LSY(1,0) = 0.;
        J_LEB(0,0) = IN_LEB;
        J_LEB(1,0) = 0.;
        J_LWY(0,0) = IN_LWY;
        J_LWY(1,0) = 0.;
        J_LWP(0,0) = IN_LWP;
        J_LWP(1,0) = 0.;
        J_LF1(0,0) = IN_LF1;
        J_LF1(1,0) = 0.;

        IN_RHY_PREV = IN_RHY;
        IN_RHR_PREV = IN_RHR;
        IN_RHP_PREV = IN_RHP;
        IN_RKN_PREV = IN_RKN;
        IN_RAP_PREV = IN_RAP;
        IN_RAR_PREV = IN_RAR;

        IN_LHY_PREV = IN_LHY;
        IN_LHR_PREV = IN_LHR;
        IN_LHP_PREV = IN_LHP;
        IN_LKN_PREV = IN_LKN;
        IN_LAP_PREV = IN_LAP;
        IN_LAR_PREV = IN_LAR;

        IN_WST_PREV = IN_WST;

        IN_RSP_PREV = IN_RSP;
        IN_RSR_PREV = IN_RSR;
        IN_RSY_PREV = IN_RSY;
        IN_REB_PREV = IN_REB;
        IN_RWY_PREV = IN_RWY;
        IN_RWP_PREV = IN_RWP;
        IN_RF1_PREV = IN_RF1;

        IN_LSP_PREV = IN_LSP;
        IN_LSR_PREV = IN_LSR;
        IN_LSY_PREV = IN_LSY;
        IN_LEB_PREV = IN_LEB;
        IN_LWY_PREV = IN_LWY;
        IN_LWP_PREV = IN_LWP;
        IN_LF1_PREV = IN_LF1;
    }

    JY(0,0) = IN_RHY;
    JY(1,0) = (IN_RHY - IN_RHY_PREV)/_dt;
    J_RHY = JKA*J_RHY + JKK*(JY - JKA*J_RHY);
    JY(0,0) = IN_RHR;
    JY(1,0) = (IN_RHR - IN_RHR_PREV)/_dt;
    J_RHR = JKA*J_RHR + JKK*(JY - JKA*J_RHR);
    JY(0,0) = IN_RHP;
    JY(1,0) = (IN_RHP - IN_RHP_PREV)/_dt;
    J_RHP = JKA*J_RHP + JKK*(JY - JKA*J_RHP);
    JY(0,0) = IN_RKN;
    JY(1,0) = (IN_RKN - IN_RKN_PREV)/_dt;
    J_RKN = JKA*J_RKN + JKK*(JY - JKA*J_RKN);
    JY(0,0) = IN_RAP;
    JY(1,0) = (IN_RAP - IN_RAP_PREV)/_dt;
    J_RAP = JKA*J_RAP + JKK*(JY - JKA*J_RAP);
    JY(0,0) = IN_RAR;
    JY(1,0) = (IN_RAR - IN_RAR_PREV)/_dt;
    J_RAR = JKA*J_RAR + JKK*(JY - JKA*J_RAR);

    JY(0,0) = IN_LHY;
    JY(1,0) = (IN_LHY - IN_LHY_PREV)/_dt;
    J_LHY = JKA*J_LHY + JKK*(JY - JKA*J_LHY);
    JY(0,0) = IN_LHR;
    JY(1,0) = (IN_LHR - IN_LHR_PREV)/_dt;
    J_LHR = JKA*J_LHR + JKK*(JY - JKA*J_LHR);
    JY(0,0) = IN_LHP;
    JY(1,0) = (IN_LHP - IN_LHP_PREV)/_dt;
    J_LHP = JKA*J_LHP + JKK*(JY - JKA*J_LHP);
    JY(0,0) = IN_LKN;
    JY(1,0) = (IN_LKN - IN_LKN_PREV)/_dt;
    J_LKN = JKA*J_LKN + JKK*(JY - JKA*J_LKN);
    JY(0,0) = IN_LAP;
    JY(1,0) = (IN_LAP - IN_LAP_PREV)/_dt;
    J_LAP = JKA*J_LAP + JKK*(JY - JKA*J_LAP);
    JY(0,0) = IN_LAR;
    JY(1,0) = (IN_LAR - IN_LAR_PREV)/_dt;
    J_LAR = JKA*J_LAR + JKK*(JY - JKA*J_LAR);

    JY(0,0) = IN_WST;
    JY(1,0) = (IN_WST - IN_WST_PREV)/_dt;
    J_WST = JKA*J_WST + JKK*(JY - JKA*J_WST);

    JY(0,0) = IN_RSP;
    JY(1,0) = (IN_RSP - IN_RSP_PREV)/_dt;
    J_RSP = JKA*J_RSP + JKK*(JY - JKA*J_RSP);
    JY(0,0) = IN_RSR;
    JY(1,0) = (IN_RSR - IN_RSR_PREV)/_dt;
    J_RSR = JKA*J_RSR + JKK*(JY - JKA*J_RSR);
    JY(0,0) = IN_RSY;
    JY(1,0) = (IN_RSY - IN_RSY_PREV)/_dt;
    J_RSY = JKA*J_RSY + JKK*(JY - JKA*J_RSY);
    JY(0,0) = IN_REB;
    JY(1,0) = (IN_REB - IN_REB_PREV)/_dt;
    J_REB = JKA*J_REB + JKK*(JY - JKA*J_REB);
    JY(0,0) = IN_RWY;
    JY(1,0) = (IN_RWY - IN_RWY_PREV)/_dt;
    J_RWY = JKA*J_RWY + JKK*(JY - JKA*J_RWY);
    JY(0,0) = IN_RWP;
    JY(1,0) = (IN_RWP - IN_RWP_PREV)/_dt;
    J_RWP = JKA*J_RWP + JKK*(JY - JKA*J_RWP);
    JY(0,0) = IN_RF1;
    JY(1,0) = (IN_RF1 - IN_RF1_PREV)/_dt;
    J_RF1 = JKA*J_RF1 + JKK*(JY - JKA*J_RF1);

    JY(0,0) = IN_LSP;
    JY(1,0) = (IN_LSP - IN_LSP_PREV)/_dt;
    J_LSP = JKA*J_LSP + JKK*(JY - JKA*J_LSP);
    JY(0,0) = IN_LSR;
    JY(1,0) = (IN_LSR - IN_LSR_PREV)/_dt;
    J_LSR = JKA*J_LSR + JKK*(JY - JKA*J_LSR);
    JY(0,0) = IN_LSY;
    JY(1,0) = (IN_LSY - IN_LSY_PREV)/_dt;
    J_LSY = JKA*J_LSY + JKK*(JY - JKA*J_LSY);
    JY(0,0) = IN_LEB;
    JY(1,0) = (IN_LEB - IN_LEB_PREV)/_dt;
    J_LEB = JKA*J_LEB + JKK*(JY - JKA*J_LEB);
    JY(0,0) = IN_LWY;
    JY(1,0) = (IN_LWY - IN_LWY_PREV)/_dt;
    J_LWY = JKA*J_LWY + JKK*(JY - JKA*J_LWY);
    JY(0,0) = IN_LWP;
    JY(1,0) = (IN_LWP - IN_LWP_PREV)/_dt;
    J_LWP = JKA*J_LWP + JKK*(JY - JKA*J_LWP);
    JY(0,0) = IN_LF1;
    JY(1,0) = (IN_LF1 - IN_LF1_PREV)/_dt;
    J_LF1 = JKA*J_LF1 + JKK*(JY - JKA*J_LF1);

    IN_RHY_PREV = IN_RHY;
    IN_RHR_PREV = IN_RHR;
    IN_RHP_PREV = IN_RHP;
    IN_RKN_PREV = IN_RKN;
    IN_RAP_PREV = IN_RAP;
    IN_RAR_PREV = IN_RAR;

    IN_LHY_PREV = IN_LHY;
    IN_LHR_PREV = IN_LHR;
    IN_LHP_PREV = IN_LHP;
    IN_LKN_PREV = IN_LKN;
    IN_LAP_PREV = IN_LAP;
    IN_LAR_PREV = IN_LAR;

    IN_WST_PREV = IN_WST;

    IN_RSP_PREV = IN_RSP;
    IN_RSR_PREV = IN_RSR;
    IN_RSY_PREV = IN_RSY;
    IN_REB_PREV = IN_REB;
    IN_RWY_PREV = IN_RWY;
    IN_RWP_PREV = IN_RWP;
    IN_RF1_PREV = IN_RF1;

    IN_LSP_PREV = IN_LSP;
    IN_LSR_PREV = IN_LSR;
    IN_LSY_PREV = IN_LSY;
    IN_LEB_PREV = IN_LEB;
    IN_LWY_PREV = IN_LWY;
    IN_LWP_PREV = IN_LWP;
    IN_LF1_PREV = IN_LF1;

    // ---------------------------------------
    // Local ZMP calculator
    // ---------------------------------------
    local_zmp_R(0,0) = -IN_RFT_MY/IN_RFT_FZ;
    local_zmp_R(1,0) = +IN_RFT_MX/IN_RFT_FZ;

    local_zmp_L(0,0) = -IN_LFT_MY/IN_LFT_FZ;
    local_zmp_L(1,0) = +IN_LFT_MX/IN_LFT_FZ;


    if(local_zmp_R(0,0) > FOOT_LX1)
        local_zmp_R(0,0) = FOOT_LX1;
    if(local_zmp_R(0,0) < -FOOT_LX2)
        local_zmp_R(0,0) = -FOOT_LX2;
    if(local_zmp_R(1,0) > FOOT_LY1)
        local_zmp_R(1,0) = FOOT_LY1;
    if(local_zmp_R(1,0) < -FOOT_LY2)
        local_zmp_R(1,0) = -FOOT_LY2;

    if(local_zmp_L(0,0) > FOOT_LX1)
        local_zmp_L(0,0) = FOOT_LX1;
    if(local_zmp_L(0,0) < -FOOT_LX2)
        local_zmp_L(0,0) = -FOOT_LX2;
    if(local_zmp_L(1,0) > FOOT_LY2)
        local_zmp_L(1,0) = FOOT_LY2;
    if(local_zmp_L(1,0) < -FOOT_LY1)
        local_zmp_L(1,0) = -FOOT_LY1;

    local_zmp_R(0,0) = 0.;
    local_zmp_R(1,0) = 0.;
    local_zmp_L(0,0) = 0.;
    local_zmp_L(1,0) = 0.;

    local_zmp_weight = fabs(IN_LFT_FZ)/(fabs(IN_RFT_FZ) + fabs(IN_LFT_FZ));
    if(HSE_RUN_FIRST == true){
        local_zmp_R_prev = local_zmp_R;
        local_zmp_L_prev = local_zmp_L;
        local_zmp_weight_prev = local_zmp_weight;
    }
    // ---------------------------------------
    // Contact Status detection
    // ---------------------------------------
    double RFT_FMAG_raw = sqrt(IN_RFT_FX*IN_RFT_FX + IN_RFT_FY*IN_RFT_FY + IN_RFT_FZ*IN_RFT_FZ);
    double LFT_FMAG_raw = sqrt(IN_LFT_FX*IN_LFT_FX + IN_LFT_FY*IN_LFT_FY + IN_LFT_FZ*IN_LFT_FZ);

    if(HSE_RUN_FIRST == true)
        CONTACT_STATUS_ESTI_PREV = 0;

    if(RFT_FMAG_raw > CONTACT_TH){
        if(LFT_FMAG_raw > CONTACT_TH)
            CONTACT_STATUS_ESTI_CUR = CONTACT_STATUS_ESTI_PREV;
        else
            CONTACT_STATUS_ESTI_CUR = -1;
    }else{
        if(LFT_FMAG_raw > CONTACT_TH)
            CONTACT_STATUS_ESTI_CUR = 1;
        else
            CONTACT_STATUS_ESTI_CUR = CONTACT_STATUS_ESTI_PREV;
    }
    // ---------------------------------------
    // Joint position calculation
    // ---------------------------------------
    R_IMU_X = Rotation_Matrix_X(IN_IMU_X);
    R_IMU_Y = Rotation_Matrix_Y(IN_IMU_Y);
    R_IMU_Z = Rotation_Matrix_Z(IN_IMU_Z);
    R_IMU_GLOBAL = R_IMU_Z*R_IMU_Y*R_IMU_X;

    R_PEL = EYE_3X3;

    R_RHY = Rotation_Matrix_Z(IN_RHY);
    R_RHR = Rotation_Matrix_X(IN_RHR);
    R_RHP = Rotation_Matrix_Y(IN_RHP);
    R_RKN = Rotation_Matrix_Y(IN_RKN);
    R_RAP = Rotation_Matrix_Y(IN_RAP);
    R_RAR = Rotation_Matrix_X(IN_RAR);

    R_LHY = Rotation_Matrix_Z(IN_LHY);
    R_LHR = Rotation_Matrix_X(IN_LHR);
    R_LHP = Rotation_Matrix_Y(IN_LHP);
    R_LKN = Rotation_Matrix_Y(IN_LKN);
    R_LAP = Rotation_Matrix_Y(IN_LAP);
    R_LAR = Rotation_Matrix_X(IN_LAR);

    R_WST = Rotation_Matrix_Z(IN_WST);

    R_RSP = Rotation_Matrix_Y(IN_RSP);
    R_RSR = Rotation_Matrix_X(IN_RSR - 15.);
    R_RSY = Rotation_Matrix_Z(IN_RSY);
    R_REB = Rotation_Matrix_Y(IN_REB - 20.);
    R_RWY = Rotation_Matrix_Z(IN_RWY);
    R_RWP = Rotation_Matrix_Y(IN_RWP);
    R_RF1 = Rotation_Matrix_Z(IN_RF1);

    R_LSP = Rotation_Matrix_Y(IN_LSP);
    R_LSR = Rotation_Matrix_X(IN_LSR + 15.);
    R_LSY = Rotation_Matrix_Z(IN_LSY);
    R_LEB = Rotation_Matrix_Y(IN_LEB - 20.);
    R_LWY = Rotation_Matrix_Z(IN_LWY);
    R_LWP = Rotation_Matrix_Y(IN_LWP);
    R_LF1 = Rotation_Matrix_Z(IN_LF1);

    p_rh1 = p_pel + R_PEL*offset_pel2rh1;
    R_RH1 = R_PEL*R_RHY*R_RHR;
    p_rh2 = p_rh1 + R_RH1*offset_rh12rh2;
    R_RHIP = R_RH1*R_RHP;
    p_rkn = p_rh2 + R_RHIP*offset_h22kn;
    R_RKNEE = R_RHIP*R_RKN;
    p_ran1 = p_rkn + R_RKNEE*offset_kn2an1;
    p_ran2 = p_ran1 + R_RKNEE*R_RAP*offset_an12an2;
    R_RFOOT = R_RKNEE*R_RAP*R_RAR;
    p_rf = p_ran2 + R_RFOOT*offset_an22ft;
    p_r_zmp = p_ran2 + R_RFOOT*(offset_an22ft + local_zmp_R);
    p_r_zmp_prev = p_ran2 + R_RFOOT*(offset_an22ft + local_zmp_R_prev);

    p_lh1 = p_pel + R_PEL*offset_pel2lh1;
    R_LH1 = R_PEL*R_LHY*R_LHR;
    p_lh2 = p_lh1 + R_LH1*offset_lh12lh2;
    R_LHIP = R_LH1*R_LHP;
    p_lkn = p_lh2 + R_LHIP*offset_h22kn;
    R_LKNEE = R_LHIP*R_LKN;
    p_lan1 = p_lkn + R_LKNEE*offset_kn2an1;
    p_lan2 = p_lan1 + R_LKNEE*R_LAP*offset_an12an2;
    R_LFOOT = R_LKNEE*R_LAP*R_LAR;
    p_lf = p_lan2 + R_LFOOT*offset_an22ft;
    p_l_zmp = p_lan2 + R_LFOOT*(offset_an22ft + local_zmp_L);
    p_l_zmp_prev = p_lan2 + R_LFOOT*(offset_an22ft + local_zmp_L_prev);

    p_dsp_zmp = p_l_zmp*local_zmp_weight + p_r_zmp*(1-local_zmp_weight);
    p_dsp_zmp_prev = p_l_zmp_prev*local_zmp_weight_prev + p_r_zmp_prev*(1-local_zmp_weight_prev);

    local_zmp_R_prev = local_zmp_R;
    local_zmp_L_prev = local_zmp_L;
    local_zmp_weight_prev = local_zmp_weight;
    // ---------------------------------------
    // Touch Point Update / Pel Position calculation
    // ---------------------------------------
    if(HSE_RUN_FIRST == true){
        tch_point = ZERO_3X1;
        TEMP1_3X1 = (p_lf + p_rf)/2.;
        tch_point(0,0) = p_dsp_zmp(0,0) - TEMP1_3X1(0,0);
        //cout<<tch_point<<endl;
    }

    if(CONTACT_STATUS_ESTI_PREV == 0){
        if(CONTACT_STATUS_ESTI_CUR == 0)//DSP->DSP
            tch_point_delta = p_dsp_zmp - p_dsp_zmp_prev;
        else if(CONTACT_STATUS_ESTI_CUR == -1)//DSP->RSP
            tch_point_delta = p_r_zmp - p_dsp_zmp_prev;
        else if(CONTACT_STATUS_ESTI_CUR == 1)//DSP->LSP
            tch_point_delta = p_l_zmp - p_dsp_zmp_prev;
    }else if(CONTACT_STATUS_ESTI_PREV == -1){
        if(CONTACT_STATUS_ESTI_CUR == 0)//RSP->DSP
            tch_point_delta = p_dsp_zmp - p_r_zmp_prev;
        else if(CONTACT_STATUS_ESTI_CUR == -1)//RSP->RSP
            tch_point_delta = p_r_zmp - p_r_zmp_prev;
        else if(CONTACT_STATUS_ESTI_CUR == 1)//RSP->LSP
            tch_point_delta = p_l_zmp - p_r_zmp_prev;
    }else if(CONTACT_STATUS_ESTI_PREV == 1){
        if(CONTACT_STATUS_ESTI_CUR == 0)//LSP->DSP
            tch_point_delta = p_dsp_zmp - p_l_zmp_prev;
        else if(CONTACT_STATUS_ESTI_CUR == -1)//LSP->RSP
            tch_point_delta = p_r_zmp - p_l_zmp_prev;
        else if(CONTACT_STATUS_ESTI_CUR == 1)//LSP->LSP
            tch_point_delta = p_l_zmp - p_l_zmp_prev;
    }
    CONTACT_STATUS_ESTI_PREV = CONTACT_STATUS_ESTI_CUR;

    tch_point = tch_point + R_IMU_GLOBAL*tch_point_delta;

    if(CONTACT_STATUS_ESTI_CUR == 0)
        tch_cal_pel_cur = tch_point - R_IMU_GLOBAL*p_dsp_zmp;
    else if(CONTACT_STATUS_ESTI_CUR == -1)
        tch_cal_pel_cur = tch_point - R_IMU_GLOBAL*p_r_zmp;
    else if(CONTACT_STATUS_ESTI_CUR == 1)
        tch_cal_pel_cur = tch_point - R_IMU_GLOBAL*p_l_zmp;

    PEL_POS = tch_cal_pel_cur;

    // ---------------------------------------
    // ZMP position calculation
    // ---------------------------------------
    FOOT_POS_R = tch_cal_pel_cur + R_IMU_GLOBAL*p_rf;
    FOOT_POS_L = tch_cal_pel_cur + R_IMU_GLOBAL*p_lf;

    ZMP_POS1(0,0) = (-IN_LFT_MY - IN_RFT_MY + FOOT_POS_L(0,0)*IN_LFT_FZ + FOOT_POS_R(0,0)*IN_RFT_FZ)/(IN_RFT_FZ+IN_LFT_FZ);
    ZMP_POS1(1,0) = (+IN_LFT_MX + IN_RFT_MX + FOOT_POS_L(1,0)*IN_LFT_FZ + FOOT_POS_R(1,0)*IN_RFT_FZ)/(IN_RFT_FZ+IN_LFT_FZ);
    ZMP_POS1(2,0) = (FOOT_POS_R(2,0)*IN_RFT_FZ + FOOT_POS_L(2,0)*IN_LFT_FZ)/(IN_RFT_FZ+IN_LFT_FZ);

    TEMP1_3X1 = FOOT_POS_R + R_IMU_GLOBAL*R_RFOOT*local_zmp_R;
    TEMP2_3X1 = FOOT_POS_L + R_IMU_GLOBAL*R_LFOOT*local_zmp_L;
    ZMP_POS2 = (TEMP2_3X1*IN_LFT_FZ + TEMP1_3X1*IN_RFT_FZ)/(IN_RFT_FZ+IN_LFT_FZ);

    ZMP_OUTPUT = ZMP_POS2;

    // ---------------------------------------
    // Pelvis Velocity Calculation
    // ---------------------------------------
    if(HSE_RUN_FIRST == true)
        tch_cal_pel_prev = tch_cal_pel_cur;

    PEL_VEL_DMETHOD = (tch_cal_pel_cur - tch_cal_pel_prev)/_dt;
    tch_cal_pel_prev = tch_cal_pel_cur;

    // ---------------------------------------
    // Pelvis Velocity Kalman filter
    // ---------------------------------------
    if(HSE_RUN_FIRST == true){
        PEL_KF_STATE(0,0) = PEL_POS(0,0);
        PEL_KF_STATE(1,0) = PEL_POS(1,0);
        PEL_KF_STATE(2,0) = PEL_POS(2,0);
        PEL_KF_STATE(3,0) = PEL_KF_STATE(4,0) = PEL_KF_STATE(5,0) = 0.0;
    }
    PEL_KF_MEAS(0,0) = PEL_POS(0,0);
    PEL_KF_MEAS(1,0) = PEL_POS(1,0);
    PEL_KF_MEAS(2,0) = PEL_POS(2,0);
    PEL_KF_MEAS(3,0) = PEL_VEL_DMETHOD(0,0);
    PEL_KF_MEAS(4,0) = PEL_VEL_DMETHOD(1,0);
    PEL_KF_MEAS(5,0) = PEL_VEL_DMETHOD(2,0);

    PEL_KF_STATE = PEL_KF_GAIN_K*PEL_KF_MEAS + PEL_KF_GAIN_A_KHA*PEL_KF_STATE;

    // ---------------------------------------
    // COM Position Calculation
    // ---------------------------------------
    pm_RHY = p_rh1 + R_PEL*R_RHY*c_rhy;
    pm_RHR = p_rh1 + R_RH1*c_rhr;
    pm_RHP = p_rh2 + R_RHIP*c_rhp;
    pm_RKN = p_rkn + R_RKNEE*c_rkn;
    pm_RAP = p_ran1 + R_RKNEE*R_RAP*c_rap;
    pm_RAR = p_ran2 + R_RFOOT*c_rar;

    pm_LHY = p_lh1 + R_PEL*R_LHY*c_lhy;
    pm_LHR = p_lh1 + R_LH1*c_lhr;
    pm_LHP = p_lh2 + R_LHIP*c_lhp;
    pm_LKN = p_lkn + R_LKNEE*c_lkn;
    pm_LAP = p_lan1 + R_LKNEE*R_LAP*c_lap;
    pm_LAR = p_lan2 + R_LFOOT*c_lar;

    R_TORSO = R_PEL*R_WST;
    p_sc = p_pel + R_TORSO*offset_pel2sc;

    p_rs = p_sc + R_TORSO*offset_sc2rs;
    R_RSHD = R_TORSO*R_RSP*R_RSR*R_RSY;
    p_reb = p_rs + R_RSHD*offset_s2eb;
    R_RELB = R_RSHD*R_REB;
    p_rw = p_reb + R_RELB*offset_eb2w;
    R_RHAND = R_RELB*R_RWY*R_RWP*R_RF1;

    p_ls = p_sc + R_TORSO*offset_sc2ls;
    R_LSHD = R_TORSO*R_LSP*R_LSR*R_LSY;
    p_leb = p_ls + R_LSHD*offset_s2eb;
    R_LELB = R_LSHD*R_LEB;
    p_lw = p_leb + R_LELB*offset_eb2w;
    R_LHAND = R_LELB*R_LWY*R_LWP*R_LF1;

    pm_PEL = p_pel + R_PEL*c_pel;
    pm_TORSO = p_pel + R_TORSO*c_torso;

    pm_RSP = p_rs + R_TORSO*R_RSP*c_rsp;
    pm_RSR = p_rs + R_TORSO*R_RSP*R_RSR*c_rsr;
    pm_RSY = p_rs + R_RSHD*c_rsy;
    pm_REB = p_reb + R_RELB*c_reb;
    pm_RWY = p_rw + R_RELB*R_RWY*c_rwy;
    pm_RWP = p_rw + R_RELB*R_RWY*R_RWP*c_rwp;
    pm_RF1 = p_rw + R_RHAND*c_rf1;

    pm_LSP = p_ls + R_TORSO*R_LSP*c_lsp;
    pm_LSR = p_ls + R_TORSO*R_LSP*R_LSR*c_lsr;
    pm_LSY = p_ls + R_LSHD*c_lsy;
    pm_LEB = p_leb + R_LELB*c_leb;
    pm_LWY = p_lw + R_LELB*R_LWY*c_lwy;
    pm_LWP = p_lw + R_LELB*R_LWY*R_LWP*c_lwp;
    pm_LF1 = p_lw + R_LHAND*c_lf1;

    TEMP1_3X1 = pm_RHY*m_rhy + pm_RHR*m_rhr + pm_RHP*m_rhp + pm_RKN*m_rkn + pm_RAP*m_rap + pm_RAR*m_rar;
    TEMP2_3X1 = pm_LHY*m_lhy + pm_LHR*m_lhr + pm_LHP*m_lhp + pm_LKN*m_lkn + pm_LAP*m_lap + pm_LAR*m_lar;
    pCOM_LB = (TEMP1_3X1 + TEMP2_3X1)/m_lb;

    TEMP1_3X1 = pm_RSP*m_rsp + pm_RSR*m_rsr + pm_RSY*m_rsy + pm_REB*m_reb + pm_RWY*m_rwy + pm_RWP*m_rwp + pm_RF1*m_rf1;
    TEMP2_3X1 = pm_LSP*m_lsp + pm_LSR*m_lsr + pm_LSY*m_lsy + pm_LEB*m_leb + pm_LWY*m_lwy + pm_LWP*m_lwp + pm_LF1*m_lf1;
    TEMP3_3X1 = pm_PEL*m_pel + pm_TORSO*m_torso;
    pCOM_UB = (TEMP1_3X1 + TEMP2_3X1 + TEMP3_3X1)/m_ub;

    pCOM_TOT = (pCOM_LB*m_lb + pCOM_UB*m_ub)/m_tot;

    COM_POS = PEL_POS + R_IMU_GLOBAL*pCOM_TOT;
    COM_WRT_POS = pCOM_TOT;
    COM_WRT_ZMP = COM_POS - ZMP_OUTPUT;
    // ---------------------------------------
    // COM Velocity Calculation
    // ---------------------------------------
    if(HSE_RUN_FIRST == true)
        COM_POS_PREV = COM_POS;

    COM_VEL_DMETHOD = (COM_POS - COM_POS_PREV)/_dt;
    COM_POS_PREV = COM_POS;

    // ---------------------------------------
    // COM Velocity Kalman filter
    // ---------------------------------------
    if(HSE_RUN_FIRST == true){
        COM_VEL_KF_STATE(0,0) = COM_POS(0,0);
        COM_VEL_KF_STATE(1,0) = COM_POS(1,0);
        COM_VEL_KF_STATE(2,0) = COM_POS(2,0);
        COM_VEL_KF_STATE(3,0) = COM_VEL_KF_STATE(4,0) = COM_VEL_KF_STATE(5,0) = 0.0;
    }
    COM_VEL_KF_MEAS(0,0) = COM_POS(0,0);
    COM_VEL_KF_MEAS(1,0) = COM_POS(1,0);
    COM_VEL_KF_MEAS(2,0) = COM_POS(2,0);
    COM_VEL_KF_MEAS(3,0) = COM_VEL_DMETHOD(0,0);
    COM_VEL_KF_MEAS(4,0) = COM_VEL_DMETHOD(1,0);
    COM_VEL_KF_MEAS(5,0) = COM_VEL_DMETHOD(2,0);

    COM_VEL_KF_STATE = COM_VEL_KF_GAIN_K*COM_VEL_KF_MEAS + COM_VEL_KF_GAIN_A_KHA*COM_VEL_KF_STATE;

    // ---------------------------------------
    // FT sensor force-summation
    // ---------------------------------------
    TEMP1_3X1(0,0) = IN_RFT_FX;
    TEMP1_3X1(1,0) = IN_RFT_FY;
    TEMP1_3X1(2,0) = IN_RFT_FZ;
    TEMP2_3X1(0,0) = IN_LFT_FX;
    TEMP2_3X1(1,0) = IN_LFT_FY;
    TEMP2_3X1(2,0) = IN_LFT_FZ;
    TEMP3_3X1(0,0) = 0.;
    TEMP3_3X1(1,0) = 0.;
    TEMP3_3X1(2,0) = m_tot*_g;

    GLOBAL_FT_FORCE_SUM = R_IMU_GLOBAL*R_RFOOT*TEMP1_3X1 + R_IMU_GLOBAL*R_LFOOT*TEMP2_3X1 - TEMP3_3X1;
    // ---------------------------------------
    // Dynamic Estimator
    // ---------------------------------------
    if(HSE_RUN_FIRST == true){
        Q(0,0) = Q(1,1) = 0.4;
        Q(2,2) = Q(3,3) = 150;
        Q(4,4) = Q(5,5) = 10;
        Q(6,6) = Q(7,7) = 100;

        R(0,0) = R(1,1) = 0.005;
        R(2,2) = R(3,3) = 3;
        R(4,4) = R(5,5) = 80*1000000;
        R(6,6) = R(7,7) = 100;

        P = ZERO_NXN;
        P(0,0) = P(1,1) = 0.2;
        P(2,2) = P(3,3) = 0.5;
        P(4,4) = P(5,5) = 1;
        P(6,6) = P(7,7) = 5;

        x = ZERO_NX1;
        x(0,0) = COM_POS(0,0);
        x(1,0) = COM_POS(1,0);

        A(0,0) = A(1,1) = A(2,2) = A(3,3) = A(4,4) = A(5,5) = A(6,6) = A(7,7) = 1.;
        A(0,2) = A(1,3) = A(2,4) = A(3,5) = _dt;
        A(0,4) = A(1,5) = 0.5*_dt*_dt;
        A(2,6) = A(3,7) = 1.;

        H(0,0) = H(1,1) = H(4,4) = H(5,5) = H(6,6) = H(7,7) = 1.;
        H(2,0) = H(3,1) = 1.;
        H(2,4) = H(3,5) = -_L/_g;

        trA = ~A;
        yeq = ZERO_2X1;

        COM_WRT_ZMP_INIT = COM_WRT_ZMP;
    }
//    y(0,0) = COM_POS(0,0);
//    y(1,0) = COM_POS(1,0);
//    y(2,0) = ZMP_OUTPUT(0,0);
//    y(3,0) = ZMP_OUTPUT(1,0);
//    y(4,0) = GLOBAL_FT_FORCE_SUM(0,0)/m_tot;
//    y(5,0) = GLOBAL_FT_FORCE_SUM(1,0)/m_tot;
//    y(6,0) = yeq(0,0);
//    y(7,0) = yeq(1,0);

//    double Lp = _L + COM_WRT_ZMP(3,0) - COM_WRT_ZMP_INIT(3,0);
//    H(2,4) = -Lp/_g;
//    H(3,5) = -Lp/_g;
//    trH = ~H;

//    xp = A*x;
//    Pp = A*P*trA + Q;
//    ResiCOV = H*Pp*trH + R;
//    ivResiCOV = !ResiCOV;
//    K = Pp*trH*ivResiCOV;
//    x = xp + K*(y - H*xp);
//    tempP = EYE_8X8 - K*H;
//    P = tempP*Pp*~tempP + K*R*~K;
//    P = 0.5*(P+~P);

    yeq(0,0) = x(2,0) - xp(2,0);
    yeq(1,0) = x(3,0) - xp(3,0);
    // ---------------------------------------
    // Final Output Update
    // ---------------------------------------
    OUTPUT.FK_PELVIS_POSITION[0] = PEL_POS(0,0);
    OUTPUT.FK_PELVIS_POSITION[1] = PEL_POS(1,0);
    OUTPUT.FK_PELVIS_POSITION[2] = PEL_POS(2,0);

    OUTPUT.FK_PELVIS_VELOCITY[0] = PEL_KF_STATE(3,0);
    OUTPUT.FK_PELVIS_VELOCITY[1] = PEL_KF_STATE(4,0);
    OUTPUT.FK_PELVIS_VELOCITY[2] = PEL_KF_STATE(5,0);

    OUTPUT.FK_COM_POSITION[0] = COM_POS(0,0);
    OUTPUT.FK_COM_POSITION[1] = COM_POS(1,0);
    OUTPUT.FK_COM_POSITION[2] = COM_POS(2,0);

    OUTPUT.FK_COM_VELOCITY[0] = COM_VEL_KF_STATE(3,0);
    OUTPUT.FK_COM_VELOCITY[1] = COM_VEL_KF_STATE(4,0);
    OUTPUT.FK_COM_VELOCITY[2] = COM_VEL_KF_STATE(5,0);

    OUTPUT.FK_ZMP_POSITION[0] = ZMP_OUTPUT(0,0);
    OUTPUT.FK_ZMP_POSITION[1] = ZMP_OUTPUT(1,0);
    OUTPUT.FK_ZMP_POSITION[2] = ZMP_OUTPUT(2,0);

    OUTPUT.KF_COM_POSITION[0] = x(0,0);
    OUTPUT.KF_COM_POSITION[1] = x(1,0);
    OUTPUT.KF_COM_VELOCITY[0] = x(2,0);
    OUTPUT.KF_COM_VELOCITY[1] = x(3,0);
    OUTPUT.KF_COM_ACCELERATION[0] = x(4,0);
    OUTPUT.KF_COM_ACCELERATION[1] = x(5,0);

    if(HSE_RUN_FIRST == true)
        HSE_RUN_FIRST = false;
}



