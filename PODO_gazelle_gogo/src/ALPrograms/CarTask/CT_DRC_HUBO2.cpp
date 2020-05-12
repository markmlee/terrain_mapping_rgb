#include "CT_DRC_HUBO2.h"
#include "../../../share/Headers/ik_math2.h"
#include <math.h>
#include <string.h>

//-----------------------------------------------------------------------------
CT_DRC_HUBO2::CT_DRC_HUBO2()
//    g_3x1({0,0,-9.81})
{
    g_3x1[0] = 0;
    g_3x1[1] = 0;
    g_3x1[2] = -9.81;

    ct_version = 20141104;

    C_RightUpperArm[0] = 0.0057;
    C_RightUpperArm[1] = 0.0139;
    C_RightUpperArm[2] = -0.1081;

    C_RightLowerArm[0] = -0.0316;
    C_RightLowerArm[1] = -0.0025;
    C_RightLowerArm[2] = -0.1698;

    C_RightHand[0] = 0.0031;
    C_RightHand[1] = -0.0059;
    C_RightHand[2] = -0.0955;

    C_LeftHand[0] = C_RightHand[0];
    C_LeftHand[1] = -C_RightHand[1];
    C_LeftHand[2] = C_RightHand[2];

    C_LeftUpperArm[0] = C_RightUpperArm[0];
    C_LeftUpperArm[1] = -C_RightUpperArm[1];
    C_LeftUpperArm[2] = C_RightUpperArm[2];

    C_LeftLowerArm[0] = C_RightLowerArm[0];
    C_LeftLowerArm[1] = -C_RightLowerArm[1];
    C_LeftLowerArm[2] = C_RightLowerArm[2];

    m_RightUpperArm = 5.39;
    m_RightLowerArm = 1.88;
    m_RightHand = 1.27;
    m_LeftUpperArm = m_RightUpperArm;
    m_LeftLowerArm = m_RightLowerArm;
    m_LeftHand = m_RightHand;

}

//-----------------------------------------------------------------------------
CT_DRC_HUBO2::~CT_DRC_HUBO2()
{

}

//-----------------------------------------------------------------------------
int CT_DRC_HUBO2::get_gravity_RightArm(const double Q_7x1[], const double qBase_4x1[], double wst_ang,
                                   double torque_7x1[])
{
    double pWST_3x1[3], pRSHLD_3x1[3], pREB_3x1[3], pRWR_3x1[3];
    double qTOR_4x1[4], qRSP_4x1[4], qRSR_4x1[4], qRSY_4x1[4], qREB_4x1[4], qRWY_4x1[4], qRWP_4x1[4], qRWY2_4x1[4];
    double temp1_4x1[4];
    double temp2_3x1[3], temp3_3x1[3];
    double cRUA_3x1[3], cRLA_3x1[3], cRH_3x1[3];
    double axis_rsp[3], axis_rsr[3], axis_rsy[3], axis_reb[3], axis_rwy[3], axis_rwp[3], axis_rwy2[3];

    const double LINK_WST[3] = {0., 0., L_PC2WST};
    const double LINK_RSHLD[3] = {0., -L_SHOULDER2SHOULDER/2., L_WST2SHOULDER};
    const double LINK_UARM[3] = {L_ELB_OFFSET, 0., -L_UPPER_ARM};
    const double LINK_LARM[3] = {-L_ELB_OFFSET, 0., -L_LOWER_ARM};
    const double pBase_3x1[3] = {0, 0, 0};

    qtRZ(wst_ang, temp1_4x1);
    QTcross(qBase_4x1,temp1_4x1,qTOR_4x1);

    qtRY(Q_7x1[0], temp1_4x1);
    QTcross(qTOR_4x1,temp1_4x1,qRSP_4x1);

    qtRX(Q_7x1[1], temp1_4x1);
    QTcross(qRSP_4x1,temp1_4x1,qRSR_4x1);

    qtRZ(Q_7x1[2], temp1_4x1);
    QTcross(qRSR_4x1,temp1_4x1,qRSY_4x1);

    qtRY(Q_7x1[3], temp1_4x1);
    QTcross(qRSY_4x1,temp1_4x1,qREB_4x1);

    qtRZ(Q_7x1[4], temp1_4x1);
    QTcross(qREB_4x1,temp1_4x1,qRWY_4x1);

    qtRY(Q_7x1[5], temp1_4x1);
    QTcross(qRWY_4x1,temp1_4x1,qRWP_4x1);

    qtRZ(Q_7x1[6], temp1_4x1);
    QTcross(qRWP_4x1,temp1_4x1,qRWY2_4x1);

    QTtransform(qBase_4x1, LINK_WST, temp1_4x1);
    sum_vv(pBase_3x1,3, temp1_4x1, pWST_3x1);

    QTtransform(qTOR_4x1, LINK_RSHLD, temp1_4x1);
    sum_vv(pWST_3x1,3, temp1_4x1, pRSHLD_3x1);

    QTtransform(qRSY_4x1, LINK_UARM, temp1_4x1);
    sum_vv(pRSHLD_3x1,3, temp1_4x1, pREB_3x1);

    QTtransform(qREB_4x1, LINK_LARM, temp1_4x1);
    sum_vv(pREB_3x1,3, temp1_4x1, pRWR_3x1);

    QTtransform(qRSY_4x1, C_RightUpperArm, cRUA_3x1);
    sum_vv(pRSHLD_3x1,3,cRUA_3x1,cRUA_3x1);

    QTtransform(qREB_4x1, C_RightLowerArm, cRLA_3x1);
    sum_vv(pREB_3x1,3,cRLA_3x1,cRLA_3x1);

    QTtransform(qRWY2_4x1, C_RightHand, cRH_3x1);
    sum_vv(pRWR_3x1,3,cRH_3x1,cRH_3x1);

    QTtransform(qRSP_4x1, AXIS_Y, axis_rsp);
    QTtransform(qRSR_4x1, AXIS_X, axis_rsr);
    QTtransform(qRSY_4x1, AXIS_Z, axis_rsy);
    QTtransform(qREB_4x1, AXIS_Y, axis_reb);
    QTtransform(qRWY_4x1, AXIS_Z, axis_rwy);
    QTtransform(qRWP_4x1, AXIS_Y, axis_rwp);
    QTtransform(qRWY2_4x1, AXIS_Z, axis_rwy2);

    diff_vv(cRH_3x1,3,pRWR_3x1,temp2_3x1);
    cross(-m_RightHand, temp2_3x1, g_3x1, temp3_3x1);
    torque_7x1[6] = dot(temp3_3x1,3,axis_rwy2);
    torque_7x1[5] = dot(temp3_3x1,3,axis_rwp);
    torque_7x1[4] = dot(temp3_3x1,3,axis_rwy);

    temp2_3x1[0] = (m_RightHand*cRH_3x1[0] + m_RightLowerArm*cRLA_3x1[0])/(m_RightHand+m_RightLowerArm) - pREB_3x1[0];
    temp2_3x1[1] = (m_RightHand*cRH_3x1[1] + m_RightLowerArm*cRLA_3x1[1])/(m_RightHand+m_RightLowerArm) - pREB_3x1[1];
    temp2_3x1[2] = (m_RightHand*cRH_3x1[2] + m_RightLowerArm*cRLA_3x1[2])/(m_RightHand+m_RightLowerArm) - pREB_3x1[2];
    cross(-(m_RightHand+m_RightLowerArm), temp2_3x1, g_3x1, temp3_3x1);
    torque_7x1[3] = dot(temp3_3x1,3,axis_reb);

    temp2_3x1[0] = (m_RightHand*cRH_3x1[0] + m_RightLowerArm*cRLA_3x1[0] + m_RightUpperArm*cRUA_3x1[0])/(m_RightHand+m_RightLowerArm+m_RightUpperArm) - pRSHLD_3x1[0];
    temp2_3x1[1] = (m_RightHand*cRH_3x1[1] + m_RightLowerArm*cRLA_3x1[1] + m_RightUpperArm*cRUA_3x1[1])/(m_RightHand+m_RightLowerArm+m_RightUpperArm) - pRSHLD_3x1[1];
    temp2_3x1[2] = (m_RightHand*cRH_3x1[2] + m_RightLowerArm*cRLA_3x1[2] + m_RightUpperArm*cRUA_3x1[2])/(m_RightHand+m_RightLowerArm+m_RightUpperArm) - pRSHLD_3x1[2];
    cross(-(m_RightHand+m_RightLowerArm+m_RightUpperArm), temp2_3x1, g_3x1, temp3_3x1);
    torque_7x1[2] = dot(temp3_3x1,3,axis_rsy);
    torque_7x1[1] = dot(temp3_3x1,3,axis_rsr);
    torque_7x1[0] = dot(temp3_3x1,3,axis_rsp);

    return 0;
}

//-----------------------------------------------------------------------------
int CT_DRC_HUBO2::get_gravity_LeftArm(const double Q_7x1[], const double qBase_4x1[], double wst_ang,
                                  double torque_7x1[])
{
    double pWST_3x1[3], pLSHLD_3x1[3], pLEB_3x1[3], pLWR_3x1[3];
    double qTOR_4x1[4], qLSP_4x1[4], qLSR_4x1[4], qLSY_4x1[4], qLEB_4x1[4], qLWY_4x1[4], qLWP_4x1[4], qLWY2_4x1[4];
    double temp1_4x1[4];
    double temp2_3x1[3], temp3_3x1[3];
    double cLUA_3x1[3], cLLA_3x1[3], cLH_3x1[3];
    double axis_lsp[3], axis_lsr[3], axis_lsy[3], axis_leb[3], axis_lwy[3], axis_lwp[3], axis_lwy2[3];

    const double LINK_WST[3] = {0., 0., L_PC2WST};
    const double LINK_LSHLD[3] = {0., L_SHOULDER2SHOULDER/2., L_WST2SHOULDER};
    const double LINK_UARM[3] = {L_ELB_OFFSET, 0., -L_UPPER_ARM};
    const double LINK_LARM[3] = {-L_ELB_OFFSET, 0., -L_LOWER_ARM};
    const double pBase_3x1[3] = {0, 0, 0};

    qtRZ(wst_ang, temp1_4x1);
    QTcross(qBase_4x1,temp1_4x1,qTOR_4x1);

    qtRY(Q_7x1[0], temp1_4x1);
    QTcross(qTOR_4x1,temp1_4x1,qLSP_4x1);

    qtRX(Q_7x1[1], temp1_4x1);
    QTcross(qLSP_4x1,temp1_4x1,qLSR_4x1);

    qtRZ(Q_7x1[2], temp1_4x1);
    QTcross(qLSR_4x1,temp1_4x1,qLSY_4x1);

    qtRY(Q_7x1[3], temp1_4x1);
    QTcross(qLSY_4x1,temp1_4x1,qLEB_4x1);

    qtRZ(Q_7x1[4], temp1_4x1);
    QTcross(qLEB_4x1,temp1_4x1,qLWY_4x1);

    qtRY(Q_7x1[5], temp1_4x1);
    QTcross(qLWY_4x1,temp1_4x1,qLWP_4x1);

    qtRZ(Q_7x1[6], temp1_4x1);
    QTcross(qLWP_4x1,temp1_4x1,qLWY2_4x1);

    QTtransform(qBase_4x1, LINK_WST, temp1_4x1);
    sum_vv(pBase_3x1,3, temp1_4x1, pWST_3x1);

    QTtransform(qTOR_4x1, LINK_LSHLD, temp1_4x1);
    sum_vv(pWST_3x1,3, temp1_4x1, pLSHLD_3x1);

    QTtransform(qLSY_4x1, LINK_UARM, temp1_4x1);
    sum_vv(pLSHLD_3x1,3, temp1_4x1, pLEB_3x1);

    QTtransform(qLEB_4x1, LINK_LARM, temp1_4x1);
    sum_vv(pLEB_3x1,3, temp1_4x1, pLWR_3x1);

    QTtransform(qLSY_4x1, C_LeftUpperArm, cLUA_3x1);
    sum_vv(pLSHLD_3x1,3,cLUA_3x1,cLUA_3x1);

    QTtransform(qLEB_4x1, C_LeftLowerArm, cLLA_3x1);
    sum_vv(pLEB_3x1,3,cLLA_3x1,cLLA_3x1);

    QTtransform(qLWY2_4x1, C_LeftHand, cLH_3x1);
    sum_vv(pLWR_3x1,3,cLH_3x1,cLH_3x1);

    QTtransform(qLSP_4x1, AXIS_Y, axis_lsp);
    QTtransform(qLSR_4x1, AXIS_X, axis_lsr);
    QTtransform(qLSY_4x1, AXIS_Z, axis_lsy);
    QTtransform(qLEB_4x1, AXIS_Y, axis_leb);
    QTtransform(qLWY_4x1, AXIS_Z, axis_lwy);
    QTtransform(qLWP_4x1, AXIS_Y, axis_lwp);
    QTtransform(qLWY2_4x1, AXIS_Z, axis_lwy2);

    diff_vv(cLH_3x1,3,pLWR_3x1,temp2_3x1);
    cross(-m_LeftHand, temp2_3x1, g_3x1, temp3_3x1);
    torque_7x1[6] = dot(temp3_3x1,3,axis_lwy2);
    torque_7x1[5] = dot(temp3_3x1,3,axis_lwp);
    torque_7x1[4] = dot(temp3_3x1,3,axis_lwy);

    temp2_3x1[0] = (m_LeftHand*cLH_3x1[0] + m_LeftLowerArm*cLLA_3x1[0])/(m_LeftHand+m_LeftLowerArm) - pLEB_3x1[0];
    temp2_3x1[1] = (m_LeftHand*cLH_3x1[1] + m_LeftLowerArm*cLLA_3x1[1])/(m_LeftHand+m_LeftLowerArm) - pLEB_3x1[1];
    temp2_3x1[2] = (m_LeftHand*cLH_3x1[2] + m_LeftLowerArm*cLLA_3x1[2])/(m_LeftHand+m_LeftLowerArm) - pLEB_3x1[2];
    cross(-(m_LeftHand+m_LeftLowerArm), temp2_3x1, g_3x1, temp3_3x1);
    torque_7x1[3] = dot(temp3_3x1,3,axis_leb);

    temp2_3x1[0] = (m_LeftHand*cLH_3x1[0] + m_LeftLowerArm*cLLA_3x1[0] + m_LeftUpperArm*cLUA_3x1[0])/(m_LeftHand+m_LeftLowerArm+m_LeftUpperArm) - pLSHLD_3x1[0];
    temp2_3x1[1] = (m_LeftHand*cLH_3x1[1] + m_LeftLowerArm*cLLA_3x1[1] + m_LeftUpperArm*cLUA_3x1[1])/(m_LeftHand+m_LeftLowerArm+m_LeftUpperArm) - pLSHLD_3x1[1];
    temp2_3x1[2] = (m_LeftHand*cLH_3x1[2] + m_LeftLowerArm*cLLA_3x1[2] + m_LeftUpperArm*cLUA_3x1[2])/(m_LeftHand+m_LeftLowerArm+m_LeftUpperArm) - pLSHLD_3x1[2];
    cross(-(m_LeftHand+m_LeftLowerArm+m_LeftUpperArm), temp2_3x1, g_3x1, temp3_3x1);
    torque_7x1[2] = dot(temp3_3x1,3,axis_lsy);
    torque_7x1[1] = dot(temp3_3x1,3,axis_lsr);
    torque_7x1[0] = dot(temp3_3x1,3,axis_lsp);

    return 0;
}
