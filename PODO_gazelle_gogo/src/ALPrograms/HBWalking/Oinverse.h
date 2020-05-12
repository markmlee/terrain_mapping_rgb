#ifndef OINVERSE_H
#define OINVERSE_H

#include "BasicMatrix.h"
struct ArmJoints
{
 double RSP,RSR,RSY,REB,RWY,RWP,RF1;
 double LSP,LSR,LSY,LEB,LWY,LWP,LF1;
double WST;

};
struct LegJoints
{
    double RHY,RHR,RHP,RKN,RAP,RAR;
    double LHY,LHR,LHP,LKN,LAP,LAR;
    vec3 pPel;
    quat qPel;
    double WST;
};
struct Moments
{
    vec3 Linear;
    vec3 Angular;
};
struct FeetPos
{
  vec3 pRF;
  quat qRF;
  vec3 pLF;
  quat qLF;
  vec3 pCOM;
  quat qPel;
};


class Oinverse//after LIG, fill these values
{
    double err_max;

    double m_rhy;
    double m_rhr;
    double m_rhp;
    double m_rkn;
    double m_rap;
    double m_rar;

    double m_lhy;
    double m_lhr;
    double m_lhp;
    double m_lkn;
    double m_lap;
    double m_lar;



    double m_rsp;
    double m_rsr;
    double m_rsy;
    double m_reb;
    double m_rwy;
    double m_rwp;
    double m_rf1;

    double m_lsp;
    double m_lsr;
    double m_lsy;
    double m_leb;
    double m_lwy;
    double m_lwp;
    double m_lf1;

    double m_torso;
    double m_pel;

    double m_ub;

    double P2H;
    double ULEG;
    double LLEG;
    double AP2AR;
    double A2F;//not sure


    double P2SC;
    double SC2S;
    double UARM;
    double LARM;
    double OFFELB;

    vec3 offset_p2rh, offset_p2lh, offset_uleg, offset_lleg, offset_ankle, offset_foot;
    vec3 c_rhy, c_rhr, c_rhp, c_rkn, c_rap, c_rar;
    vec3 c_lhy, c_lhr, c_lhp, c_lkn, c_lap, c_lar;
    mat3 I_rhy, I_rhr, I_rhp, I_rkn, I_rap, I_rar;
    mat3 I_lhy, I_lhr, I_lhp, I_lkn, I_lap, I_lar;

    vec3 offset_p2s_center, offset_s_center2rs, offset_s_center2ls, offset_uarm, offset_larm, offset_elbow;
    vec3 c_rsp, c_rsr, c_rsy, c_reb, c_rwy, c_rwp, c_rf1;
    vec3 c_lsp, c_lsr, c_lsy, c_leb, c_lwy, c_lwp, c_lf1;
    mat3 I_rsp, I_rsr, I_rsy, I_reb, I_rwy, I_rwp, I_rf1;
    mat3 I_lsp, I_lsr, I_lsy, I_leb, I_lwy, I_lwp, I_lf1;


    vec3 c_torso, c_pel;
    mat3 I_torso, I_pel;

    vec3 c_ub;//final calculated result
    int QT2DC(const double qt_4x1[], double DC_3x3[])  // convert a quaternion to a direction cosine matrix
    {
        double temp = sqrtp(qt_4x1[0]*qt_4x1[0] + qt_4x1[1]*qt_4x1[1]
                            + qt_4x1[2]*qt_4x1[2] + qt_4x1[3]*qt_4x1[3]);
        double q0 = qt_4x1[0]/temp;
        double q1 = qt_4x1[1]/temp;
        double q2 = qt_4x1[2]/temp;
        double q3 = qt_4x1[3]/temp;
        DC_3x3[0] = 2.*(q0*q0 + q1*q1) - 1.;
        DC_3x3[1] = 2.*(q1*q2 - q0*q3);
        DC_3x3[2] = 2.*(q1*q3 + q0*q2);
        DC_3x3[3] = 2.*(q1*q2 + q0*q3);
        DC_3x3[4] = 2.*(q0*q0 + q2*q2) - 1.;
        DC_3x3[5] = 2.*(q2*q3 - q0*q1);
        DC_3x3[6] = 2.*(q1*q3 - q0*q2);
        DC_3x3[7] = 2.*(q2*q3 + q0*q1);
        DC_3x3[8] = 2.*(q0*q0 + q3*q3) - 1.;
        return 0;
    }
    int QT2YRP(const double qt_4x1[], double &yaw, double &rol, double &pit)
    {
        double dc[9];
        QT2DC(qt_4x1, dc);
        pit = atan2(-dc[2*3+0], dc[2*3+2]);
        yaw = atan2(dc[1*3+0]*cos(pit) + dc[1*3+2]*sin(pit), dc[0*3+0]*cos(pit) + dc[0*3+2]*sin(pit));
        rol = atan2(dc[2*3+1], dc[2*3+2]*cos(pit) - dc[2*3+0]*sin(pit));
        return 0;
    }
public:
    Oinverse()
    {
        //set initial something
        err_max = 1e-7;

        m_rhy = 0.01;
        m_rhr = 0.01;
        m_rhp = 6.3512;
        m_rkn = 1.9592;
        m_rap = 0.01;
        m_rar = 2.6626;

        m_lhy = 0.01;
        m_lhr = 0.01;
        m_lhp = 6.3512;
        m_lkn = 1.9592;
        m_lap = 0.01;
        m_lar = 2.6626;



//        m_rsp = 0.01;
//        m_rsr = 0.01;
//        m_rsy = 2.31;
//        m_reb = 0.5542;
//        m_rwy = 0.01;
//        m_rwp = 0.001;
//        m_rf1 = 0.001;

        m_rsp = 0.01;
        m_rsr = 0.01;
        m_rsy = 0.01;
        m_reb = 0.01;
        m_rwy = 0.01;
        m_rwp = 0.001;
        m_rf1 = 0.001;

//        m_lsp = 0.01;
//        m_lsr = 0.01;
//        m_lsy = 2.31;
//        m_leb = 0.5542;
//        m_lwy = 0.01;
//        m_lwp = 0.001;
//        m_lf1 = 0.001;

        m_lsp = 0.01;
        m_lsr = 0.01;
        m_lsy = 0.01;
        m_leb = 0.01;
        m_lwy = 0.01;
        m_lwp = 0.001;
        m_lf1 = 0.001;

        m_torso = 17.2025;
        m_pel = 3.876;

        P2H = 0.0885;
        ULEG = 0.28;
        LLEG = 0.28;
        AP2AR = 0.001;
        A2F = 0.12;//not sure


        P2SC = 0.3687; // pelvis to shoulder center
        SC2S = 0.2145; // shoulder center to shoulder
        UARM = 0.17914;
        LARM = 0.16261;
        OFFELB = 0.022;

        m_ub = m_pel+m_torso
                +m_rsp+m_rsr+m_rsy+m_reb+m_rwy+m_rwp+m_rf1
                +m_lsp+m_lsr+m_lsy+m_leb+m_lwy+m_lwp+m_lf1;
        //offsets_ub
        offset_p2s_center = vec3(0,0,0);
        offset_s_center2rs = vec3(0,-SC2S,P2SC);
        offset_s_center2ls = vec3(0,+SC2S,P2SC);
        offset_uarm = vec3(OFFELB,0,-UARM);
        offset_larm = vec3(-OFFELB,0,-LARM);
        //centerofmass_ub
        c_rsp = vec3(0.0, 0.0, 0.0);
        c_rsr = vec3(0.0, 0.0, 0.0);
        c_rsy = vec3(0.0062,0.0178,-0.0451);
        c_reb = vec3(0.0003,-0.0006,-0.047);
        c_rwy = vec3(0.0, 0.0, 0.0);
        c_rwp = vec3(0.0, 0.0, 0.0);
        c_rf1 = vec3(0.0, 0.0, 0.0);

        c_lsp = vec3(0.0, 0.0, 0.0);
        c_lsr = vec3(0.0, 0.0, 0.0);
        c_lsy = vec3(0.0062,-0.0178,-0.0451);
        c_leb = vec3(0.0003,0.0006,-0.047);
        c_lwy = vec3(0.0, 0.0, 0.0);
        c_lwp = vec3(0.0, 0.0, 0.0);
        c_lf1 = vec3(0.0, 0.0, 0.0);

        c_torso = vec3(0.00, 0.0, 0.476);
        c_pel = vec3(-0.0119, 0.0, 0.1323);

        //offsets_lb
        offset_p2rh = vec3(0,-P2H,0);
        offset_p2lh = vec3(0,+P2H,0);
        offset_uleg = vec3(0,0,-ULEG);
        offset_lleg = vec3(0,0,-LLEG);
        offset_ankle = vec3(0,0,-AP2AR);
        offset_foot = vec3(0,0,-A2F);

        //centerofmass_lb
        c_rhy = vec3(0.0, 0.0, 0.0);
        c_rhr = vec3(0.0, 0.0, 0.0);
        c_rhp = vec3(0.0175,-0.0099,-0.0995);
        c_rkn = vec3(0.0146,-0.0146,-0.1845);
        c_rap = vec3(0.0, 0.0, 0.0);
        c_rar = vec3(0.0216, -0.0014, -0.016);

        c_lhy = vec3(0.0, 0.0, 0.0);
        c_lhr = vec3(0.0, 0.0, 0.0);
        c_lhp = vec3(0.0175,0.0099,-0.0995);
        c_lkn = vec3(0.0146,0.0146,-0.1845);
        c_lap = vec3(0.0, 0.0, 0.0);
        c_lar = vec3(0.0216, 0.0014, -0.016);

     //   I_rar = mat3(1,0,0,0,0,0,0,0,0);
       // I_lar = mat3(1,0,0,0,0,0,0,0,0);



            }

            vec3 FKCOM_UB(ArmJoints AJ)
            {
                vec3 COM_UB;

                mat4 T_SC = mat4(offset_p2s_center,vec3(0,0,1),AJ.WST);

                mat4 T_RSP = T_SC * mat4(offset_s_center2rs,vec3(0,1,0),AJ.RSP);
                mat4 T_RSR = T_RSP * mat4(vec3(),vec3(1,0,0),AJ.RSR);
                mat4 T_RSY = T_RSR * mat4(vec3(),vec3(0,0,1),AJ.RSY);
                mat4 T_REB = T_RSY * mat4(offset_uarm,vec3(0,1,0),AJ.REB);
                mat4 T_RWY = T_REB * mat4(offset_larm,vec3(0,0,1),AJ.RWY);
                mat4 T_RWP = T_RWY * mat4(vec3(),vec3(0,1,0),AJ.RWP);
                mat4 T_RF1 = T_RWP * mat4(vec3(),vec3(0,0,1),AJ.RF1);

                mat4 T_LSP = T_SC * mat4(offset_s_center2ls, vec3(0,1,0),AJ.LSP);
                mat4 T_LSR = T_LSP * mat4(vec3(),vec3(1,0,0),AJ.LSR);
                mat4 T_LSY = T_LSR * mat4(vec3(),vec3(0,0,1),AJ.LSY);
                mat4 T_LEB = T_LSY * mat4(offset_uarm,vec3(0,1,0),AJ.LEB);
                mat4 T_LWY = T_LEB * mat4(offset_larm,vec3(0,0,1),AJ.LWY);
                mat4 T_LWP = T_LWY * mat4(vec3(),vec3(0,1,0),AJ.LWP);
                mat4 T_LF1 = T_LWP * mat4(vec3(),vec3(0,0,1),AJ.LF1);

                vec4 pCOM_pel = c_pel;
                vec4 pCOM_torso  = T_SC*c_torso;

                vec4 pCOM_rsp = T_RSP*c_rsp;
                vec4 pCOM_rsr = T_RSR*c_rsr;
                vec4 pCOM_rsy = T_RSY*c_rsy;
                vec4 pCOM_reb = T_REB*c_reb;
                vec4 pCOM_rwy = T_RWY*c_rwy;
                vec4 pCOM_rwp = T_RWP*c_rwp;
                vec4 pCOM_rf1 = T_RF1*c_rf1;

                vec4 pCOM_lsp = T_LSP*c_lsp;
                vec4 pCOM_lsr = T_LSR*c_lsr;
                vec4 pCOM_lsy = T_LSY*c_lsy;
                vec4 pCOM_leb = T_LEB*c_leb;
                vec4 pCOM_lwy = T_LWY*c_lwy;
                vec4 pCOM_lwp = T_LWP*c_lwp;
                vec4 pCOM_lf1 = T_LF1*c_lf1;

                double m_rarm = m_rsp+m_rsr+m_rsy+m_reb+m_rwy+m_rwp+m_rf1;
                double m_larm = m_lsp+m_lsr+m_lsy+m_leb+m_lwy+m_rwp+m_lf1;

                vec4 pCOM_rarm = (pCOM_rsp*m_rsp + pCOM_rsr*m_rsr + pCOM_rsy*m_rsy + pCOM_reb*m_reb
                                  + pCOM_rwy*m_rwy + pCOM_rwp*m_rwp + pCOM_rf1*m_rf1)/m_rarm;
                vec4 pCOM_larm = (pCOM_lsp*m_lsp + pCOM_lsr*m_lsr + pCOM_lsy*m_lsy + pCOM_leb*m_leb
                                  + pCOM_lwy*m_lwy + pCOM_lwp*m_lwp + pCOM_lf1*m_lf1)/m_larm;

                COM_UB = (pCOM_pel*m_pel + pCOM_torso*m_torso + pCOM_rarm*m_rarm + pCOM_larm*m_larm)/(m_rarm + m_larm + m_pel + m_torso);

//                //TTTTTTTTTTTTTTT
//                printf("m_ub %f COM_UB %f %f %f\n"
//                       ,(m_rarm + m_larm + m_pel + m_torso)
//                       ,(COM_UB.x,COM_UB.y,COM_UB.z));

                return COM_UB;//set initially first// auto set c_ub
            }
            int setUB(vec3 COM_UB)
            {
                c_ub = vec4(COM_UB);
                //printf("c_ub %f %f %f \n",c_ub.x,c_ub.y,c_ub.z);
            }

//            FeetPos FK(LegJoints LJ)
//            {
//                FeetPos rp;
//                vec3 pPel = LJ.pPel;
//                quat qPel  = LJ.qPel;
//                mat4 T_RHY = mat4(offset_p2rh,vec3(0,0,1),LJ.RHY);
//                mat4 T_RHR = T_RHY*mat4(vec3(),vec3(1,0,0),LJ.RHR);
//                mat4 T_RHP = T_RHR*mat4(vec3(),vec3(0,1,0),LJ.RHP);
//                mat4 T_RKN = T_RHP*mat4(offset_uleg,vec3(0,1,0),LJ.RKN);
//                mat4 T_RAP = T_RKN*mat4(offset_lleg,vec3(0,1,0),LJ.RAP);
//                mat4 T_RAR = T_RAP*mat4(offset_ankle,vec3(1,0,0),LJ.RAR);
//                mat4 T_RFOOT = T_RAR*mat4(offset_foot,vec3(0,1,0),0);


//                rp.pRF = mat3(qPel)*(T_RFOOT*vec3(0,0,0)) + pPel;
//                rp.qRF = quat(mat3(qPel)*mat3(T_RFOOT));

//                mat4 T_LHY = mat4(offset_p2lh,vec3(0,0,1),LJ.LHY);
//                mat4 T_LHR = T_LHY*mat4(vec3(),vec3(1,0,0),LJ.LHR);
//                mat4 T_LHP = T_LHR*mat4(vec3(),vec3(0,1,0),LJ.LHP);
//                mat4 T_LKN = T_LHP*mat4(offset_uleg,vec3(0,1,0),LJ.LKN);
//                mat4 T_LAP = T_LKN*mat4(offset_lleg,vec3(0,1,0),LJ.LAP);
//                mat4 T_LAR = T_LAP*mat4(offset_ankle,vec3(1,0,0),LJ.LAR);
//                mat4 T_LFOOT = T_LAR*mat4(offset_foot,vec3(0,1,0),0);

//                rp.pLF = mat3(qPel)*(T_LFOOT*vec3(0,0,0)) + pPel;
//                rp.qLF = quat(mat3(qPel)*mat3(T_LFOOT));

//                vec4 pCOM_ub = mat3(qPel)*c_ub + pPel;

//                vec4 pCOM_rhy = pPel + T_RHY*c_rhy;
//                vec4 pCOM_rhr = pPel + T_RHR*c_rhr;
//                vec4 pCOM_rhp = pPel + T_RHP*c_rhp;
//                vec4 pCOM_rkn = pPel + T_RKN*c_rkn;
//                vec4 pCOM_rap = pPel + T_RAP*c_rap;
//                vec4 pCOM_rar = pPel + T_RAR*c_rar;


//                vec4 pCOM_lhy = pPel + T_LHY*c_lhy;
//                vec4 pCOM_lhr = pPel + T_LHR*c_lhr;
//                vec4 pCOM_lhp = pPel + T_LHP*c_lhp;
//                vec4 pCOM_lkn = pPel + T_LKN*c_lkn;
//                vec4 pCOM_lap = pPel + T_LAP*c_lap;
//                vec4 pCOM_lar = pPel + T_LAR*c_lar;

//                double m_rleg = m_rhy+m_rhr+m_rhp+m_rkn+m_rap+m_rar;
//                double m_lleg = m_lhy+m_lhr+m_lhp+m_lkn+m_lap+m_lar;

//                vec4 pCOM_rleg = (pCOM_rhy*m_rhy + pCOM_rhr*m_rhr + pCOM_rhp*m_rhp + pCOM_rkn*m_rkn + pCOM_rap*m_rap + pCOM_rar*m_rar)/m_rleg;
//                vec4 pCOM_lleg = (pCOM_lhy*m_lhy + pCOM_lhr*m_lhr + pCOM_lhp*m_lhp + pCOM_lkn*m_lkn + pCOM_lap*m_lap + pCOM_lar*m_lar)/m_lleg;

//                //TTTTTTTTTTTTTTT
////                printf("EE m_ub %f m_rleg %f m_lleg %f pCOM_rleg %f %f %f pCOM_lleg %f %f %f \n"
////                       ,m_ub, m_rleg, m_lleg
////                       ,pCOM_rleg.x,pCOM_rleg.y,pCOM_rleg.z
////                       ,pCOM_lleg.x,pCOM_lleg.y,pCOM_lleg.z);


//                rp.pCOM = (pCOM_ub*m_ub + pCOM_rleg*m_rleg + pCOM_lleg*m_lleg)/(m_ub + m_rleg + m_lleg);

//                return rp;
//            }

            FeetPos FK(LegJoints LJ)
                        {
                            FeetPos rp;
                            rp.qPel = LJ.qPel;
                            vec3 pPel = LJ.pPel;
                            quat qPel  = LJ.qPel;
                            mat4 T_PEL = mat4(pPel,mat3(qPel));
                            mat4 T_RHY = T_PEL*mat4(offset_p2rh,vec3(0,0,1),LJ.RHY);
                            mat4 T_RHR = T_RHY*mat4(vec3(),vec3(1,0,0),LJ.RHR);
                            mat4 T_RHP = T_RHR*mat4(vec3(),vec3(0,1,0),LJ.RHP);
                            mat4 T_RKN = T_RHP*mat4(offset_uleg,vec3(0,1,0),LJ.RKN);
                            mat4 T_RAP = T_RKN*mat4(offset_lleg,vec3(0,1,0),LJ.RAP);
                            mat4 T_RAR = T_RAP*mat4(offset_ankle,vec3(1,0,0),LJ.RAR);
                            mat4 T_RFOOT = T_RAR*mat4(offset_foot,vec3(0,1,0),0);

                            rp.pRF = (T_RFOOT*vec3(0,0,0));
                            rp.qRF = quat(mat3(T_RFOOT));

                            mat4 T_LHY = T_PEL*mat4(offset_p2lh,vec3(0,0,1),LJ.LHY);
                            mat4 T_LHR = T_LHY*mat4(vec3(),vec3(1,0,0),LJ.LHR);
                            mat4 T_LHP = T_LHR*mat4(vec3(),vec3(0,1,0),LJ.LHP);
                            mat4 T_LKN = T_LHP*mat4(offset_uleg,vec3(0,1,0),LJ.LKN);
                            mat4 T_LAP = T_LKN*mat4(offset_lleg,vec3(0,1,0),LJ.LAP);
                            mat4 T_LAR = T_LAP*mat4(offset_ankle,vec3(1,0,0),LJ.LAR);
                            mat4 T_LFOOT = T_LAR*mat4(offset_foot,vec3(0,1,0),0);


                            rp.pLF = (T_LFOOT*vec3(0,0,0));
                            rp.qLF = quat(mat3(T_LFOOT));


                            vec4 pCOM_ub = T_PEL*c_ub;

                            vec4 pCOM_rhy =  T_RHY*c_rhy;
                            vec4 pCOM_rhr =  T_RHR*c_rhr;
                            vec4 pCOM_rhp =  T_RHP*c_rhp;
                            vec4 pCOM_rkn =  T_RKN*c_rkn;
                            vec4 pCOM_rap =  T_RAP*c_rap;
                            vec4 pCOM_rar =  T_RAR*c_rar;


                            vec4 pCOM_lhy =  T_LHY*c_lhy;
                            vec4 pCOM_lhr =  T_LHR*c_lhr;
                            vec4 pCOM_lhp =  T_LHP*c_lhp;
                            vec4 pCOM_lkn =  T_LKN*c_lkn;
                            vec4 pCOM_lap =  T_LAP*c_lap;
                            vec4 pCOM_lar =  T_LAR*c_lar;


                            double m_rleg = m_rhy+m_rhr+m_rhp+m_rkn+m_rap+m_rar;
                            double m_lleg = m_lhy+m_lhr+m_lhp+m_lkn+m_lap+m_lar;

                            vec4 pCOM_rleg = (pCOM_rhy*m_rhy + pCOM_rhr*m_rhr + pCOM_rhp*m_rhp + pCOM_rkn*m_rkn + pCOM_rap*m_rap + pCOM_rar*m_rar)/m_rleg;
                            vec4 pCOM_lleg = (pCOM_lhy*m_lhy + pCOM_lhr*m_lhr + pCOM_lhp*m_lhp + pCOM_lkn*m_lkn + pCOM_lap*m_lap + pCOM_lar*m_lar)/m_lleg;

                            //TTTTTTTTTTTTTTT
            //                printf("EE m_ub %f m_rleg %f m_lleg %f pCOM_rleg %f %f %f pCOM_lleg %f %f %f \n"
            //                       ,m_ub, m_rleg, m_lleg
            //                       ,pCOM_rleg.x,pCOM_rleg.y,pCOM_rleg.z
            //                       ,pCOM_lleg.x,pCOM_lleg.y,pCOM_lleg.z);


                            rp.pCOM = (pCOM_ub*m_ub + pCOM_rleg*m_rleg + pCOM_lleg*m_lleg)/(m_ub + m_rleg + m_lleg);

                            return rp;
                        }

            vec3 toOmega(mat3 New, mat3 Old, double dt)
            {
                mat3 tt = Old.inverse()*New;

                quat dtq = quat(tt);//dq/dt

                vec3 rtv;
                double dqt_d[4] = {dtq[0],dtq[1],dtq[2],dtq[3]};
               QT2YRP(dqt_d, (rtv.z), (rtv.x), (rtv.y));
                rtv = rtv*dt;//OK??? lets think more
                return rtv;
            }
    Moments FK_moments(ArmJoints AJ, LegJoints LJ, vec3 pPel, quat qPel,//do not move arms yet!
                       ArmJoints AJ_old, LegJoints LJ_old, vec3 pPel_old, quat qPel_old, double dt)
    {
        Moments MM;
        vec3 COM_UB, oCOM_UB;
        mat4 T_PEL = mat4(pPel,mat3(qPel));
        mat4 oT_PEL = mat4(pPel_old,mat3(qPel_old));


        mat4 T_SC = T_PEL * mat4(offset_p2s_center,vec3(0,0,1),AJ.WST);

        mat4 T_RSP = T_SC * mat4(offset_s_center2rs,vec3(0,1,0),AJ.RSP);
        mat4 T_RSR = T_RSP * mat4(vec3(),vec3(1,0,0),AJ.RSR);
        mat4 T_RSY = T_RSR * mat4(vec3(),vec3(0,0,1),AJ.RSY);
        mat4 T_REB = T_RSY * mat4(offset_uarm,vec3(0,1,0),AJ.REB);
        mat4 T_RWY = T_REB * mat4(offset_larm,vec3(0,0,1),AJ.RWY);
        mat4 T_RWP = T_RWY * mat4(vec3(),vec3(0,1,0),AJ.RWP);
        mat4 T_RF1 = T_RWP * mat4(vec3(),vec3(0,0,1),AJ.RF1);

        mat4 T_LSP = T_SC * mat4(offset_s_center2ls, vec3(0,1,0),AJ.LSP);
        mat4 T_LSR = T_LSP * mat4(vec3(),vec3(1,0,0),AJ.LSR);
        mat4 T_LSY = T_LSR * mat4(vec3(),vec3(0,0,1),AJ.LSY);
        mat4 T_LEB = T_LSY * mat4(offset_uarm,vec3(0,1,0),AJ.LEB);
        mat4 T_LWY = T_LEB * mat4(offset_larm,vec3(0,0,1),AJ.LWY);
        mat4 T_LWP = T_LWY * mat4(vec3(),vec3(0,1,0),AJ.LWP);
        mat4 T_LF1 = T_LWP * mat4(vec3(),vec3(0,0,1),AJ.LF1);

        vec4 pCOM_pel = T_PEL*c_pel;
        vec4 pCOM_torso  = T_SC*c_torso;

        vec4 pCOM_rsp = T_RSP*c_rsp;
        vec4 pCOM_rsr = T_RSR*c_rsr;
        vec4 pCOM_rsy = T_RSY*c_rsy;
        vec4 pCOM_reb = T_REB*c_reb;
        vec4 pCOM_rwy = T_RWY*c_rwy;
        vec4 pCOM_rwp = T_RWP*c_rwp;
        vec4 pCOM_rf1 = T_RF1*c_rf1;

        vec4 pCOM_lsp = T_LSP*c_lsp;
        vec4 pCOM_lsr = T_LSR*c_lsr;
        vec4 pCOM_lsy = T_LSY*c_lsy;
        vec4 pCOM_leb = T_LEB*c_leb;
        vec4 pCOM_lwy = T_LWY*c_lwy;
        vec4 pCOM_lwp = T_LWP*c_lwp;
        vec4 pCOM_lf1 = T_LF1*c_lf1;

        double m_rarm = m_rsp+m_rsr+m_rsy+m_reb+m_rwy+m_rwp+m_rf1;
        double m_larm = m_lsp+m_lsr+m_lsy+m_leb+m_lwy+m_rwp+m_lf1;

        vec4 pCOM_rarm = (pCOM_rsp*m_rsp + pCOM_rsr*m_rsr + pCOM_rsy*m_rsy + pCOM_reb*m_reb
                          + pCOM_rwy*m_rwy + pCOM_rwp*m_rwp  + pCOM_rf1*m_rf1)/m_rarm;
        vec4 pCOM_larm = (pCOM_lsp*m_lsp + pCOM_lsr*m_lsr + pCOM_lsy*m_lsy + pCOM_leb*m_leb
                          + pCOM_lwy*m_lwy + pCOM_lwp*m_lwp  + pCOM_lf1*m_lf1)/m_larm;

        COM_UB = (pCOM_pel*m_pel + pCOM_torso*m_torso + pCOM_rarm*m_rarm + pCOM_larm*m_larm)/(m_rarm + m_larm + m_pel + m_torso);


        mat4 oT_SC = oT_PEL *mat4(offset_p2s_center,vec3(0,0,1),AJ_old.WST);

        mat4 oT_RSP = oT_SC * mat4(offset_s_center2rs,vec3(0,1,0),AJ_old.RSP);
        mat4 oT_RSR = oT_RSP * mat4(vec3(),vec3(1,0,0),AJ_old.RSR);
        mat4 oT_RSY = oT_RSR * mat4(vec3(),vec3(0,0,1),AJ_old.RSY);
        mat4 oT_REB = oT_RSY * mat4(offset_uarm,vec3(0,1,0),AJ_old.REB);
        mat4 oT_RWY = oT_REB * mat4(offset_larm,vec3(0,0,1),AJ_old.RWY);
        mat4 oT_RWP = oT_RWY * mat4(vec3(),vec3(0,1,0),AJ_old.RWP);
        mat4 oT_RF1 = oT_RWP * mat4(vec3(),vec3(0,0,1),AJ_old.RF1);

        mat4 oT_LSP = oT_SC * mat4(offset_s_center2ls, vec3(0,1,0),AJ_old.LSP);
        mat4 oT_LSR = oT_LSP * mat4(vec3(),vec3(1,0,0),AJ_old.LSR);
        mat4 oT_LSY = oT_LSR * mat4(vec3(),vec3(0,0,1),AJ_old.LSY);
        mat4 oT_LEB = oT_LSY * mat4(offset_uarm,vec3(0,1,0),AJ_old.LEB);
        mat4 oT_LWY = oT_LEB * mat4(offset_larm,vec3(0,0,1),AJ_old.LWY);
        mat4 oT_LWP = oT_LWY * mat4(vec3(),vec3(0,1,0),AJ_old.LWP);
        mat4 oT_LF1 = oT_LWP * mat4(vec3(),vec3(0,0,1),AJ_old.LF1);

        vec4 opCOM_pel = oT_PEL*c_pel;
        vec4 opCOM_torso  = oT_SC*c_torso;

        vec4 opCOM_rsp = oT_RSP*c_rsp;
        vec4 opCOM_rsr = oT_RSR*c_rsr;
        vec4 opCOM_rsy = oT_RSY*c_rsy;
        vec4 opCOM_reb = oT_REB*c_reb;
        vec4 opCOM_rwy = oT_RWY*c_rwy;
        vec4 opCOM_rwp = oT_RWP*c_rwp;
        vec4 opCOM_rf1 = oT_RF1*c_rf1;

        vec4 opCOM_lsp = oT_LSP*c_lsp;
        vec4 opCOM_lsr = oT_LSR*c_lsr;
        vec4 opCOM_lsy = oT_LSY*c_lsy;
        vec4 opCOM_leb = oT_LEB*c_leb;
        vec4 opCOM_lwy = oT_LWY*c_lwy;
        vec4 opCOM_lwp = oT_LWP*c_lwp;
        vec4 opCOM_lf1 = oT_LF1*c_lf1;



        vec4 opCOM_rarm = ( opCOM_rsp*m_rsp + opCOM_rsr*m_rsr + opCOM_rsy*m_rsy + opCOM_reb*m_reb
                          + opCOM_rwy*m_rwy + opCOM_rwp*m_rwp + opCOM_rf1*m_rf1)/m_rarm;
        vec4 opCOM_larm = ( opCOM_lsp*m_lsp + opCOM_lsr*m_lsr + opCOM_lsy*m_lsy + opCOM_leb*m_leb
                          + opCOM_lwy*m_lwy + opCOM_lwp*m_lwp + opCOM_lf1*m_lf1)/m_larm;

        oCOM_UB = ( opCOM_pel*m_pel + opCOM_torso*m_torso + opCOM_rarm*m_rarm + opCOM_larm*m_larm)/(m_rarm + m_larm + m_pel + m_torso);


        //copy_of_fk_ub
        FeetPos rp;

        mat4 T_RHY = T_PEL *mat4(offset_p2rh,vec3(0,0,1),LJ.RHY);
        mat4 T_RHR = T_RHY*mat4(vec3(),vec3(1,0,0),LJ.RHR);
        mat4 T_RHP = T_RHR*mat4(vec3(),vec3(0,1,0),LJ.RHP);
        mat4 T_RKN = T_RHP*mat4(offset_uleg,vec3(0,1,0),LJ.RKN);
        mat4 T_RAP = T_RKN*mat4(offset_lleg,vec3(0,1,0),LJ.RAP);
        mat4 T_RAR = T_RAP*mat4(offset_ankle,vec3(1,0,0),LJ.RAR);
        mat4 T_RFOOT = T_RAR*mat4(offset_foot,vec3(0,1,0),0);


        rp.pRF = (T_RFOOT*vec3(0,0,0));
        rp.qRF = quat(mat3(T_RFOOT));

        mat4 T_LHY = T_PEL *mat4(offset_p2lh,vec3(0,0,1),LJ.LHY);
        mat4 T_LHR = T_LHY*mat4(vec3(),vec3(1,0,0),LJ.LHR);
        mat4 T_LHP = T_LHR*mat4(vec3(),vec3(0,1,0),LJ.LHP);
        mat4 T_LKN = T_LHP*mat4(offset_uleg,vec3(0,1,0),LJ.LKN);
        mat4 T_LAP = T_LKN*mat4(offset_lleg,vec3(0,1,0),LJ.LAP);
        mat4 T_LAR = T_LAP*mat4(offset_ankle,vec3(1,0,0),LJ.LAR);
        mat4 T_LFOOT = T_LAR*mat4(offset_foot,vec3(0,1,0),0);

        rp.pLF = (T_LFOOT*vec3(0,0,0));
        rp.qLF = quat(mat3(T_LFOOT));

        vec4 pCOM_ub = COM_UB ;

        vec4 pCOM_rhy = T_RHY*c_rhy;
        vec4 pCOM_rhr = T_RHR*c_rhr;
        vec4 pCOM_rhp = T_RHP*c_rhp;
        vec4 pCOM_rkn = T_RKN*c_rkn;
        vec4 pCOM_rap = T_RAP*c_rap;
        vec4 pCOM_rar = T_RAR*c_rar;


        vec4 pCOM_lhy =  T_LHY*c_lhy;
        vec4 pCOM_lhr =  T_LHR*c_lhr;
        vec4 pCOM_lhp =  T_LHP*c_lhp;
        vec4 pCOM_lkn =  T_LKN*c_lkn;
        vec4 pCOM_lap =  T_LAP*c_lap;
        vec4 pCOM_lar =  T_LAR*c_lar;

        double m_rleg = m_rhy+m_rhr+m_rhp+m_rkn+m_rap+m_rar;
        double m_lleg = m_lhy+m_lhr+m_lhp+m_lkn+m_lap+m_lar;

        vec4 pCOM_rleg = (pCOM_rhy*m_rhy + pCOM_rhr*m_rhr + pCOM_rhp*m_rhp + pCOM_rkn*m_rkn + pCOM_rap*m_rap + pCOM_rar*m_rar)/m_rleg;
        vec4 pCOM_lleg = (pCOM_lhy*m_lhy + pCOM_lhr*m_lhr + pCOM_lhp*m_lhp + pCOM_lkn*m_lkn + pCOM_lap*m_lap + pCOM_lar*m_lar)/m_lleg;



        rp.pCOM = (pCOM_ub*m_ub + pCOM_rleg*m_rleg + pCOM_lleg*m_lleg)/(m_ub + m_rleg + m_lleg);
        //copy of FK

        FeetPos rp_old;
        mat4 oT_RHY =  oT_PEL*mat4(offset_p2rh,vec3(0,0,1),LJ_old.RHY);
        mat4 oT_RHR = oT_RHY*mat4(vec3(),vec3(1,0,0),LJ_old.RHR);
        mat4 oT_RHP = oT_RHR*mat4(vec3(),vec3(0,1,0),LJ_old.RHP);
        mat4 oT_RKN = oT_RHP*mat4(offset_uleg,vec3(0,1,0),LJ_old.RKN);
        mat4 oT_RAP = oT_RKN*mat4(offset_lleg,vec3(0,1,0),LJ_old.RAP);
        mat4 oT_RAR = oT_RAP*mat4(offset_ankle,vec3(1,0,0),LJ_old.RAR);
        mat4 oT_RFOOT = oT_RAR*mat4(offset_foot,vec3(0,1,0),0);


        rp_old.pRF = (oT_RFOOT*vec3(0,0,0));
        rp_old.qRF = quat(mat3(oT_RFOOT));

        mat4 oT_LHY = oT_PEL*mat4(offset_p2lh,vec3(0,0,1),LJ_old.LHY);
        mat4 oT_LHR = oT_LHY*mat4(vec3(),vec3(1,0,0),LJ_old.LHR);
        mat4 oT_LHP = oT_LHR*mat4(vec3(),vec3(0,1,0),LJ_old.LHP);
        mat4 oT_LKN = oT_LHP*mat4(offset_uleg,vec3(0,1,0),LJ_old.LKN);
        mat4 oT_LAP = oT_LKN*mat4(offset_lleg,vec3(0,1,0),LJ_old.LAP);
        mat4 oT_LAR = oT_LAP*mat4(offset_ankle,vec3(1,0,0),LJ_old.LAR);
        mat4 oT_LFOOT = oT_LAR*mat4(offset_foot,vec3(0,1,0),0);

        rp_old.pLF = (oT_LFOOT*vec3(0,0,0));
        rp_old.qLF = quat(mat3(oT_LFOOT));

        vec4 opCOM_ub = oCOM_UB;

        vec4 opCOM_rhy =  oT_RHY*c_rhy;
        vec4 opCOM_rhr =  oT_RHR*c_rhr;
        vec4 opCOM_rhp =  oT_RHP*c_rhp;
        vec4 opCOM_rkn =  oT_RKN*c_rkn;
        vec4 opCOM_rap =  oT_RAP*c_rap;
        vec4 opCOM_rar =  oT_RAR*c_rar;


        vec4 opCOM_lhy = oT_LHY*c_lhy;
        vec4 opCOM_lhr = oT_LHR*c_lhr;
        vec4 opCOM_lhp = oT_LHP*c_lhp;
        vec4 opCOM_lkn = oT_LKN*c_lkn;
        vec4 opCOM_lap = oT_LAP*c_lap;
        vec4 opCOM_lar = oT_LAR*c_lar;

        vec4 opCOM_rleg = (opCOM_rhy*m_rhy + opCOM_rhr*m_rhr + opCOM_rhp*m_rhp + opCOM_rkn*m_rkn + opCOM_rap*m_rap + opCOM_rar*m_rar)/m_rleg;
        vec4 opCOM_lleg = (opCOM_lhy*m_lhy + opCOM_lhr*m_lhr + opCOM_lhp*m_lhp + opCOM_lkn*m_lkn + opCOM_lap*m_lap + opCOM_lar*m_lar)/m_lleg;



        rp_old.pCOM = (opCOM_ub*m_ub + opCOM_rleg*m_rleg + opCOM_lleg*m_lleg)/(m_ub + m_rleg + m_lleg);
        //old values

        MM.Linear = (m_ub + m_rleg + m_lleg)*(rp.pCOM-rp_old.pCOM)*dt;//linear momentum


        MM.Angular = vec3(0,0,0);

        //r cross mv + Iomega
        //for test


        MM.Angular = MM.Angular
            +vec3(cross(pCOM_rsp+opCOM_rsp-vec4(rp.pCOM+rp_old.pCOM),m_rsp*(pCOM_rsp-opCOM_rsp-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rsr+opCOM_rsr-vec4(rp.pCOM+rp_old.pCOM),m_rsr*(pCOM_rsr-opCOM_rsr-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rsy+opCOM_rsy-vec4(rp.pCOM+rp_old.pCOM),m_rsy*(pCOM_rsy-opCOM_rsy-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_reb+opCOM_reb-vec4(rp.pCOM+rp_old.pCOM),m_reb*(pCOM_reb-opCOM_reb-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rwy+opCOM_rwy-vec4(rp.pCOM+rp_old.pCOM),m_rwy*(pCOM_rwy-opCOM_rwy-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rwp+opCOM_rwp-vec4(rp.pCOM+rp_old.pCOM),m_rwp*(pCOM_rwp-opCOM_rwp-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rf1+opCOM_rf1-vec4(rp.pCOM+rp_old.pCOM),m_rf1*(pCOM_rf1-opCOM_rf1-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5

            +vec3(cross(pCOM_lsp+opCOM_lsp-vec4(rp.pCOM+rp_old.pCOM),m_lsp*(pCOM_lsp-opCOM_lsp-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lsr+opCOM_lsr-vec4(rp.pCOM+rp_old.pCOM),m_lsr*(pCOM_lsr-opCOM_lsr-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lsy+opCOM_lsy-vec4(rp.pCOM+rp_old.pCOM),m_lsy*(pCOM_lsy-opCOM_lsy-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_leb+opCOM_leb-vec4(rp.pCOM+rp_old.pCOM),m_leb*(pCOM_leb-opCOM_leb-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lwy+opCOM_lwy-vec4(rp.pCOM+rp_old.pCOM),m_lwy*(pCOM_lwy-opCOM_lwy-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lwp+opCOM_lwp-vec4(rp.pCOM+rp_old.pCOM),m_lwp*(pCOM_lwp-opCOM_lwp-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lf1+opCOM_lf1-vec4(rp.pCOM+rp_old.pCOM),m_lf1*(pCOM_lf1-opCOM_lf1-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5

            +vec3(cross(pCOM_torso+opCOM_torso-vec4(rp.pCOM+rp_old.pCOM),m_torso*(pCOM_torso-opCOM_torso-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_pel+opCOM_pel-vec4(rp.pCOM+rp_old.pCOM),m_pel*(pCOM_pel-opCOM_pel-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5


            +vec3(cross(pCOM_rhy+opCOM_rhy-vec4(rp.pCOM+rp_old.pCOM),m_rhy*(pCOM_rhy-opCOM_rhy-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rhr+opCOM_rhr-vec4(rp.pCOM+rp_old.pCOM),m_rhr*(pCOM_rhr-opCOM_rhr-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rhp+opCOM_rhp-vec4(rp.pCOM+rp_old.pCOM),m_rhp*(pCOM_rhp-opCOM_rhp-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rkn+opCOM_rkn-vec4(rp.pCOM+rp_old.pCOM),m_rkn*(pCOM_rkn-opCOM_rkn-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rap+opCOM_rap-vec4(rp.pCOM+rp_old.pCOM),m_rap*(pCOM_rap-opCOM_rap-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_rar+opCOM_rar-vec4(rp.pCOM+rp_old.pCOM),m_rar*(pCOM_rar-opCOM_rar-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5

            +vec3(cross(pCOM_lhy+opCOM_lhy-vec4(rp.pCOM+rp_old.pCOM),m_lhy*(pCOM_lhy-opCOM_lhy-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lhr+opCOM_lhr-vec4(rp.pCOM+rp_old.pCOM),m_lhr*(pCOM_lhr-opCOM_lhr-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lhp+opCOM_lhp-vec4(rp.pCOM+rp_old.pCOM),m_lhp*(pCOM_lhp-opCOM_lhp-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lkn+opCOM_lkn-vec4(rp.pCOM+rp_old.pCOM),m_lkn*(pCOM_lkn-opCOM_lkn-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lap+opCOM_lap-vec4(rp.pCOM+rp_old.pCOM),m_lap*(pCOM_lap-opCOM_lap-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5
            +vec3(cross(pCOM_lar+opCOM_lar-vec4(rp.pCOM+rp_old.pCOM),m_lar*(pCOM_lar-opCOM_lar-vec4(rp.pCOM-rp_old.pCOM))*dt))*0.5;

//    std::cout<<"RSP "<<pCOM_rsp.x<<" "<<pCOM_rsp.y<<" "<<pCOM_rsp.z<<std::endl;
//    std::cout<<"oRSP "<<opCOM_rsp.x<<" "<<opCOM_rsp.y<<" "<<opCOM_rsp.z<<std::endl;
//    std::cout<<"COM "<<rp.pCOM.x<<" "<<rp.pCOM.y<<" "<<rp.pCOM.z<<std::endl;
//    std::cout<<"oCOM "<<rp_old.pCOM.x<<" "<<rp_old.pCOM.y<<" "<<rp_old.pCOM.z<<std::endl;
//    vec4 TTT = vec4(rp.pCOM-rp_old.pCOM);
//    std::cout<<"delCOM "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    std::cout<<"pCOM_larm "<<pCOM_larm.x<<" "<<pCOM_larm.y<<" "<<pCOM_larm.z<<" "<<std::endl;
//    std::cout<<"pCOM_larmo"<<opCOM_larm.x<<" "<<opCOM_larm.y<<" "<<opCOM_larm.z<<" "<<std::endl;
//    TTT = pCOM_larm-opCOM_larm;
//    std::cout<<"delCOMlarm "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    TTT = pCOM_lsp-opCOM_lsp;
//    std::cout<<"dellsp "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    TTT = pCOM_lsr-opCOM_lsr;
//    std::cout<<"dellsr "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    TTT = pCOM_lsy-opCOM_lsy;
//    std::cout<<"dellsy "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    TTT = pCOM_leb-opCOM_leb;
//    std::cout<<"delleb "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    TTT = pCOM_lwp-opCOM_lwp;
//    std::cout<<"dellwp "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;
//    TTT = pCOM_lf1-opCOM_lf1;
//    std::cout<<"dellf1 "<<TTT.x<<" "<<TTT.y<<" "<<TTT.z<<" "<<std::endl;



        //r cross mv respect to COM
        MM.Angular = MM.Angular
            + mat3(T_RSP)*I_rsp * mat3(T_RSP).transpose() * toOmega(mat3(T_RSP),mat3(oT_RSP),dt)
            + mat3(T_RSR)*I_rsr * mat3(T_RSR).transpose() * toOmega(mat3(T_RSR),mat3(oT_RSR),dt)
            + mat3(T_RSY)*I_rsy * mat3(T_RSY).transpose() * toOmega(mat3(T_RSY),mat3(oT_RSY),dt)
            + mat3(T_REB)*I_reb * mat3(T_REB).transpose() * toOmega(mat3(T_REB),mat3(oT_REB),dt)
            + mat3(T_RWY)*I_rwy * mat3(T_RWY).transpose() * toOmega(mat3(T_RWY),mat3(oT_RWY),dt)
            + mat3(T_RWP)*I_rwp * mat3(T_RWP).transpose() * toOmega(mat3(T_RWP),mat3(oT_RWP),dt)
            + mat3(T_RF1)*I_rf1 * mat3(T_RF1).transpose() * toOmega(mat3(T_RF1),mat3(oT_RF1),dt)

            + mat3(T_LSP)*I_lsp * mat3(T_LSP).transpose() * toOmega(mat3(T_LSP),mat3(oT_LSP),dt)
            + mat3(T_LSR)*I_lsr * mat3(T_LSR).transpose() * toOmega(mat3(T_LSR),mat3(oT_LSR),dt)
            + mat3(T_LSY)*I_lsy * mat3(T_LSY).transpose() * toOmega(mat3(T_LSY),mat3(oT_LSY),dt)
            + mat3(T_LEB)*I_leb * mat3(T_LEB).transpose() * toOmega(mat3(T_LEB),mat3(oT_LEB),dt)
            + mat3(T_LWY)*I_lwy * mat3(T_LWY).transpose() * toOmega(mat3(T_LWY),mat3(oT_LWY),dt)
            + mat3(T_LWP)*I_lwp * mat3(T_LWP).transpose() * toOmega(mat3(T_LWP),mat3(oT_LWP),dt)
            + mat3(T_LF1)*I_lf1 * mat3(T_LF1).transpose() * toOmega(mat3(T_LF1),mat3(oT_LF1),dt)

            + mat3(T_PEL)*I_pel * mat3(T_PEL).transpose() * toOmega(mat3(T_PEL),mat3(oT_PEL),dt)

            + mat3(T_SC)*I_torso * mat3(T_SC).transpose() * toOmega(mat3(T_SC),mat3(oT_SC),dt)

            + mat3(T_RHY)*I_rhy * mat3(T_RHY).transpose() * toOmega(mat3(T_RHY),mat3(oT_RHY),dt)
            + mat3(T_RHR)*I_rhr * mat3(T_RHR).transpose() * toOmega(mat3(T_RHR),mat3(oT_RHR),dt)
            + mat3(T_RHP)*I_rhp * mat3(T_RHP).transpose() * toOmega(mat3(T_RHP),mat3(oT_RHP),dt)
            + mat3(T_RKN)*I_rkn * mat3(T_RKN).transpose() * toOmega(mat3(T_RKN),mat3(oT_RKN),dt)
            + mat3(T_RAP)*I_rap * mat3(T_RAP).transpose() * toOmega(mat3(T_RAP),mat3(oT_RAP),dt)
            + mat3(T_RAR)*I_rar * mat3(T_RAR).transpose() * toOmega(mat3(T_RAR),mat3(oT_RAR),dt)


            + mat3(T_LHY)*I_lhy * mat3(T_LHY).transpose() * toOmega(mat3(T_LHY),mat3(oT_LHY),dt)
            + mat3(T_LHR)*I_lhr * mat3(T_LHR).transpose() * toOmega(mat3(T_LHR),mat3(oT_LHR),dt)
            + mat3(T_LHP)*I_lhp * mat3(T_LHP).transpose() * toOmega(mat3(T_LHP),mat3(oT_LHP),dt)
            + mat3(T_LKN)*I_lkn * mat3(T_LKN).transpose() * toOmega(mat3(T_LKN),mat3(oT_LKN),dt)
            + mat3(T_LAP)*I_lap * mat3(T_LAP).transpose() * toOmega(mat3(T_LAP),mat3(oT_LAP),dt)
            + mat3(T_LAR)*I_lar * mat3(T_LAR).transpose() * toOmega(mat3(T_LAR),mat3(oT_LAR),dt);


        return MM;
        //now have to verify

    }


    LegJoints IK_pel(vec3 pPel, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF)
    {
        LegJoints LJ;

        LJ.pPel = pPel;
        LJ.qPel = qPel;
        vec3 pRPel = pPel+qPel*offset_p2rh;

        mat3 iqRF = mat3(qRF);
        iqRF.inverse();

        vec3 tpRPel = pRPel-pRF;//foot position to zero
             tpRPel = iqRF*tpRPel;//rotate to foot quat to be zero

        vec3 prAr = -offset_foot;
        vec3 Ar2rH = tpRPel-prAr;

        LJ.RAR = atan2(Ar2rH[1],Ar2rH[2]);

        mat3 tm1 = mat3(vec3(1,0,0),-LJ.RAR);
        vec3 tv1 = tm1*(-offset_ankle);
        vec3 prAp = prAr+tv1;
        vec3 Ap2rH = tpRPel-prAp;
        double l_Ap2rH = Ap2rH.norm();

        if(l_Ap2rH<0.1)
        {
            Ap2rH = Ap2rH/l_Ap2rH*0.1;
        }
        else if(l_Ap2rH>ULEG+LLEG-0.002)
        {
            Ap2rH = Ap2rH/l_Ap2rH*(ULEG+LLEG-0.002);
        }

        double t1 = (ULEG*ULEG +LLEG*LLEG  - l_Ap2rH*l_Ap2rH)/(2.0* LLEG*ULEG);
        if(t1>1) t1 = 1;
        else if(t1<-1) t1 = -1;
        LJ.RKN = M_PI-acos(t1);//knee

        double ang1 = (LLEG*LLEG + l_Ap2rH*l_Ap2rH - ULEG*ULEG)/(2.0*LLEG*l_Ap2rH);
        if(ang1>1) ang1 = 1;
        if(ang1<-1) ang1 = -1;
        ang1 = acos(ang1);

        double ang2 = (AP2AR*AP2AR + l_Ap2rH*l_Ap2rH - Ar2rH.norm()*Ar2rH.norm())/(2.0*AP2AR*l_Ap2rH);
        if(ang2>1) ang2 = 1;
        if(ang2<-1) ang2 = -1;
        ang2 = acos(ang2);

        tv1 = cross(Ap2rH,prAr-prAp);
        if(tv1[1] >0) LJ.RAP = ang2-ang1-M_PI;
        else if(tv1[1] <0) LJ.RAP = M_PI-(ang2+ang1);
        else LJ.RAP = ang1;

        tm1 = mat3(vec3(1,0,0),-LJ.RAR);
        mat3 tm2 = mat3(vec3(0,1,0),-LJ.RAP);
        tm1 = tm1*tm2;
        tm2 = mat3(vec3(0,1,0),-LJ.RKN);
        tm1 = tm1*tm2;
        mat3 iqp = mat3(qPel);
        iqp.inverse();
        tm1 =iqp*mat3(qRF)*tm1;

        quat tqq = quat(tm1);
        double ddd[4] = {tqq[0],tqq[1], tqq[2], tqq[3]};

        QT2YRP(ddd,LJ.RHY, LJ.RHR, LJ.RHP);

        /////////////////////////////////////////

        vec3 pLPel = pPel+qPel*offset_p2lh;

        mat3 iqLF = mat3(qLF);
        iqLF.inverse();

        vec3 tpLPel = pLPel-pLF;//foot position to zero
             tpLPel = iqLF*tpLPel;//rotate to foot quat to be zero

        vec3 plAr = -offset_foot;
        vec3 Ar2lH = tpLPel-plAr;

        LJ.LAR = atan2(Ar2lH[1],Ar2lH[2]);

        tm1 = mat3(vec3(1,0,0),-LJ.LAR);
        tv1 = tm1*(-offset_ankle);
        vec3 plAp = plAr+tv1;
        vec3 Ap2lH = tpLPel-plAp;
        double l_Ap2lH = Ap2lH.norm();

        if(l_Ap2lH<0.1)
        {
            Ap2lH = Ap2lH/l_Ap2lH*0.1;
        }
        else if(l_Ap2lH>ULEG+LLEG-0.002)
        {
            Ap2lH = Ap2lH/l_Ap2lH*(ULEG+LLEG-0.002);
        }

        t1 = (ULEG*ULEG +LLEG*LLEG  - l_Ap2lH*l_Ap2lH)/(2.0* LLEG*ULEG);
        if(t1>1) t1 = 1;
        else if(t1<-1) t1 = -1;
        LJ.LKN = M_PI-acos(t1);//knee

        ang1 = (LLEG*LLEG + l_Ap2lH*l_Ap2lH - ULEG*ULEG)/(2.0*LLEG*l_Ap2lH);
        if(ang1>1) ang1 = 1;
        if(ang1<-1) ang1 = -1;
        ang1 = acos(ang1);

        ang2 = (AP2AR*AP2AR + l_Ap2lH*l_Ap2lH - Ar2lH.norm()*Ar2lH.norm())/(2.0*AP2AR*l_Ap2lH);
        if(ang2>1) ang2 = 1;
        if(ang2<-1) ang2 = -1;
        ang2 = acos(ang2);

        tv1 = cross(Ap2lH,plAr-plAp);
        if(tv1[1] >0) LJ.LAP = ang2-ang1-M_PI;
        else if(tv1[1] <0) LJ.LAP = M_PI-(ang2+ang1);
        else LJ.LAP = ang1;

        tm1 = mat3(vec3(1,0,0),-LJ.LAR);
        tm2 = mat3(vec3(0,1,0),-LJ.LAP);
        tm1 = tm1*tm2;
        tm2 = mat3(vec3(0,1,0),-LJ.LKN);
        tm1 = tm1*tm2;
        iqp = mat3(qPel);
        iqp.inverse();
        tm1 =iqp*mat3(qLF)*tm1;

        tqq = quat(tm1);
        double ddd2[4] = {tqq[0],tqq[1], tqq[2], tqq[3]};

        QT2YRP(ddd2,LJ.LHY, LJ.LHR, LJ.LHP);

        return LJ;
    }
    LegJoints IK_COM(vec3 pCOM, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF)
    {

        LegJoints LJ;
        FeetPos FP;
        FP.pCOM = vec3(0,0,-1000);
        vec3 pPel = pCOM;//find this.
        int cnt = 0;
        while((FP.pCOM-pCOM).norm()>err_max)
        {
            LJ = IK_pel(pPel,qPel,pRF,qRF,pLF,qLF);//loop loop loop

            FP = FK(LJ);
            vec3 dpCOM = pCOM - FP.pCOM;
            pPel = pPel + dpCOM;//should be improved
            cnt++;
        }
        LJ.pPel = pPel;
        return LJ;
    }
    LegJoints IK_COM_MM(vec3 pCOM, vec3 pRF, quat qRF, vec3 pLF, quat qLF
                        , ArmJoints AJ, LegJoints LJ_old, vec3 pPel_old, quat qPel_old, vec3 Angular_ref)
    {
        Moments MM;
        MM.Angular = vec3(1000,0,0);
        LegJoints LJ;
        quat qPel = qPel_old;//will be changed using iteration
        int cnt = 0;
        while((MM.Angular-Angular_ref).norm()>1e-8)
        {
            setUB(FKCOM_UB(AJ));
            LJ = IK_COM(pCOM, qPel,pRF,qRF,pLF,qLF);
            MM = FK_moments(AJ,LJ,LJ.pPel,qPel,AJ,LJ_old,pPel_old,qPel_old,0.005);
            qPel = qPel*quat(mat3(vec3(1,0,0),20.0*MM.Angular.x));
            qPel = qPel*quat(mat3(vec3(0,1,0),20.0*MM.Angular.y));
            MM.Angular.z = Angular_ref.z;//in yaw rotation, moment from SSP foot should be applied
            //qPel = qPel*quat(mat3(vec3(0,0,1),5.0*MM.Angular.z));
           //AJ.WST += 0.1*MM.Angular.z;//no Yaw yet. not working...
            //not only in yaw rotation, there is momentum change due to ground reaction force.
            //let's

 //printf("iter %d Angular moment %g %g %g %g\n",cnt, MM.Angular.x, MM.Angular.y, MM.Angular.z,MM.Angular.norm());
            cnt++;
            if(cnt>40)
            {
                printf("iter %d Angular moment %g %g %g %g\n",cnt, MM.Angular.x, MM.Angular.y, MM.Angular.z,MM.Angular.norm());
                break;
            }

        }

        LJ.qPel = qPel;
        LJ.WST = AJ.WST;
        return LJ;
    }

};

#endif // OINVERSE_H
