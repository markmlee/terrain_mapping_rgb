#ifndef HBINVERSE
#define HBINVERSE
#define         PI          3.141592

#include "BasicMatrix.h"

using namespace std;

struct ArmJoints
{
    union{
        struct{
            double RSP,RSR,RSY,REB,RWY,RWP,RF1;
            double LSP,LSR,LSY,LEB,LWY,LWP,LF1;
        };
        double AJ_Array[14];
    };
    double WST;

    vec3 pPel;
    quat qPel;

};
struct LegJoints
{
    union{
        struct{
            double RHY,RHR,RHP,RKN,RAP,RAR;
            double LHY,LHR,LHP,LKN,LAP,LAR;
        };
        double LJ_Array[12];
    };

    vec3 pPel;
    quat qPel;
    double WST;
};
struct FeetPos
{
  vec3 pRF;
  quat qRF;
  vec3 pLF;
  quat qLF;
  vec3 pCOM;
  vec3 pPel;
  quat qPel;
  double pPelz;
};

struct ArmPos
{
    vec3 pRH;
    quat qRH;
    vec3 pLH;
    quat qLH;
    vec3 pCOM;
    vec3 pPel;
    quat qPel;
};

class HB_inverse
{
public:
    double err_max;

    double m_rhy;
    double m_rhr;
    double m_rhp;
    double m_rkn;
    double m_rap;
    double m_rar;
    double m_rleg;

    double m_lhy;
    double m_lhr;
    double m_lhp;
    double m_lkn;
    double m_lap;
    double m_lar;
    double m_lleg;

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
    double m_total;

    //double m_ub;

    double P2HR; // l1
    double HR2HPy; // l2
    double HR2HPz; // l3
    double ULEG;   // l4
    double LLEG;   // l5
    double A2F;    // l6


    double P2SC;
    double SC2S;
    double UARM;
    double LARM;
    double OFFELB;

    vec3 offset_p2rHR, offset_p2lHR, offset_rHR2rHP, offset_lHR2lHP, offset_uleg, offset_lleg, offset_foot;
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

    //vec3 c_ub;//final calculated result
    //mat3 I_ub;

    public:

    HB_inverse()
    {
        //set initial something
        err_max = 1e-5;

        m_rhy = 1.034;
        m_rhr = 2.055;
        m_rhp = 4.888;//3.888;//4.888;
        m_rkn = 0.676;
        m_rap = 0.029;
        m_rar = 0.919;//0.4;//0.919;
        m_rleg = m_rhy + m_rhr + m_rhp + m_rkn + m_rap + m_rar; //9.601

        m_lhy = 1.034;
        m_lhr = 2.055;
        m_lhp = 4.888;//3.888;//4.888;
        m_lkn = 0.676;
        m_lap = 0.029;
        m_lar = 0.919;//0.4;//.919;
        m_lleg = m_lhy + m_lhr + m_lhp + m_lkn + m_lap + m_lar;



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

        m_torso = 0.01;//17.2025;
        //no waste joint version, this pelvis contains torso mass
        m_pel = 23.0;//11.4;//10;//12.44;

        m_total = m_pel + m_rleg + m_lleg;

        P2HR = 0.06;  //l1
        HR2HPy = 0.0491;  //l2
        HR2HPz = 0.072;  //l3
        ULEG = 0.33;  //l4
        LLEG = 0.32249;  //l5
        A2F = 0.085; //l6


        P2SC = 0.3687; // pelvis to shoulder center
        SC2S = 0.15;//0.2145; // shoulder center to shoulder
        UARM = 0.3;//0.17914;
        LARM = 0.16261;
        OFFELB = 0.022;

//        m_ub = m_pel+m_torso
//                +m_rsp+m_rsr+m_rsy+m_reb+m_rwy+m_rwp+m_rf1
//                +m_lsp+m_lsr+m_lsy+m_leb+m_lwy+m_lwp+m_lf1;

        //offsets_ub
        offset_p2s_center = vec3(0,0,0);
        offset_s_center2rs = vec3(0,-SC2S,P2SC);
        offset_s_center2ls = vec3(0,+SC2S,P2SC);
        offset_uarm = vec3(0,0,-UARM); //vec3(OFFELB,0,-UARM);
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

        c_torso = vec3(0.00, 0.0, 0.00);

        //c_pel = vec3(-0.0119, 0.0, 0.1323);
        //no waste joint version, this pelvis contains torso mass

        //c_pel = vec3(-0.009, 0.00, 0.1);
        //c_pel = vec3(0.01, 0.00, 0.1);

        //c_pel = vec3(-0.05, 0, 0.087);
//        c_pel = vec3(-0.01, 0, 0.1323);
//        c_pel = vec3(+0.006, -0.00, 0.28);
        c_pel = vec3(+0.02, -0.00, 0.33);



        //offsets_lb
        offset_p2rHR = vec3(0,-P2HR,0);
        offset_p2lHR = vec3(0,+P2HR,0);
        offset_rHR2rHP = vec3(0, -HR2HPy, -HR2HPz);
        offset_lHR2lHP = vec3(0, HR2HPy, -HR2HPz);
        offset_uleg = vec3(0,0,-ULEG);
        offset_lleg = vec3(0,0,-LLEG);
        offset_foot = vec3(0,0,-A2F);

        //centerofmass_lb
        c_rhy = vec3(-0.019, -0.009, 0.0265);
        c_rhr = vec3(-0.00265, -0.07, -0.05);
        c_rhp = vec3(0.0165,-0.0119,-0.2236);
        c_rkn = vec3(0.005,0.0,-0.064);
        c_rap = vec3(0.0, 0.0, 0.0);
        c_rar = vec3(-0.0075, 0.00, -0.0484);

        c_lhy = vec3(-0.019, 0.009, 0.0265);
        c_lhr = vec3(-0.00265, 0.07, -0.05);
        c_lhp = vec3(0.0165,0.0119,-0.2236);
        c_lkn = vec3(0.005,0.0,-0.064);
        c_lap = vec3(0.0, 0.0, 0.0);
        c_lar = vec3(-0.0075, 0.00, -0.0484);

        //c_ub = vec3(0,0,0.2462);//every joints are 0

        //inertia_ub
        I_rsp = mat3(0.0052, -0.0013, 0, -0.0013, 0.0044, 0, 0, 0, 0.0072);
        I_rsr = mat3(0.0032, 0, 0, 0, 0.0037, 0, 0, 0, 0.0019);
        I_rsy = mat3(0.00242, 0, -0.0039, 0, 0.0249, -0.0011, -0.0039, -0.0011, 0.0034);
        I_reb = mat3(0.0010, 0, 0, 0, 0.0012, 0, 0, 0, 0.00056);
        I_rwy = mat3(0.013, 0, 0, 0, 0.013, 0, 0, 0, 0.00086);
        I_rwp = mat3( 0.00072, 0, 0, 0, 0.00065, 0, 0, 0, 0.00065);
        I_rf1 = mat3(0.000002, 0, 0, 0, 0.00000185, 0, 0, 0, 0.00000333 );

        I_lsp = mat3(0.0052, 0.0013, 0, -0.0013, 0.0044, 0, 0, 0, 0.0072);
        I_lsr = mat3(0.0032, 0, 0, 0, 0.0037, 0, 0, 0, 0.0019);
        I_lsy = mat3(0.00242, 0, -0.0039, 0, 0.0249, -0.0011, -0.0039, -0.0011, 0.0034);
        I_leb = mat3(0.0010, 0, 0, 0, 0.0012, 0, 0, 0, 0.00056);
        I_lwy = mat3(0.013, 0, 0, 0, 0.013, 0, 0, 0, 0.00086);
        I_lwp = mat3( 0.00072, 0, 0, 0, 0.00065, 0, 0, 0, 0.00065);
        I_lf1 = mat3(0.000002, 0, 0, 0, 0.00000185, 0, 0, 0, 0.00000333 );

        I_torso = mat3( 0.198, 0, 0, 0, 0.156, 0, 0, 0, 0.118);
        // no waste joint version, pelvis contains torso inertia
        //I_pel = mat3( 0.038, 0, 0, 0, 0.013, 0, 0, 0, 0.041);
        I_pel = mat3(0.20514, -0.0003, -0.013, -0.0003, 0.18345, 0.00031, -0.013, 0.0003, 0.0874);

        //I_ub = mat3(5.0473, -0.0026, -0.01791, -0.0026, 4.1145, -0.0022, -0.01791, -0.0022, 1.10299);//every joints are 0

        I_rhy = mat3(0.00128, -0.00, 0.000158, 0, 0.00173, -0.0002, 0.00015, -0.0002, 0.0012);
        I_rhr = mat3(0.0045, 0.001312, -0.0008, 0.001312, 0.005, -0.00011, -0.0008, -0.00010, 0.005);
        I_rhp = mat3(0.0547, 0.0003, -0.0039, 0.0039, 0.057, -0.003, -0.0039, -0.003, 0.0125);
        I_rkn = mat3(0.0065, 0, -0.00012, 0, 0.0067, 0.00, -0.000125, -0.00, 0.000375);
        I_rap = mat3(0.000002, 0, 0, 0, 0.000006, 0, 0, 0, 0.000007);
        I_rar = mat3(0.0023, -0.00, -0.0003, 0, 0.004, 0, -0.0002, 0, 0.005);

        I_lhy = mat3(0.00128, 0.00, 0.000158, 0, 0.00173, 0.0002, 0.00015, 0.0002, 0.0012);
        I_lhr = mat3(0.0045, -0.001312, -0.0008, -0.001312, 0.005, 0.00011, -0.0008, 0.00010, 0.005);
        I_lhp = mat3(0.0547, -0.0003, -0.0039, -0.0039, 0.057, 0.003, -0.0039, 0.003, 0.0125);
        I_lkn = mat3(0.0065, 0, -0.00012, 0, 0.0067, 0.00, -0.000125, -0.00, 0.000375);
        I_lap = mat3(0.000002, 0, 0, 0, 0.000006, 0, 0, 0, 0.000007);
        I_lar = mat3(0.0023, -0.00, -0.0003, 0, 0.004, 0, -0.0002, 0, 0.005);
    }

    vec3 FKCOM_UB(ArmJoints AJ)
    {
        vec3 COM_UB;

        mat4 T_SC = mat4(offset_p2s_center,vec3(0,0,1),AJ.WST);

        mat4 T_RSP = T_SC * mat4(offset_s_center2rs,vec3(0,1,0),AJ.RSP);
        mat4 T_RSR = T_RSP * mat4(vec3(),vec3(1,0,0),AJ.RSR);
//        mat4 T_RSY = T_RSR * mat4(vec3(),vec3(0,0,1),AJ.RSY);
//        mat4 T_REB = T_RSY * mat4(offset_uarm,vec3(0,1,0),AJ.REB);
//        mat4 T_RWY = T_REB * mat4(offset_larm,vec3(0,0,1),AJ.RWY);
//        mat4 T_RWP = T_RWY * mat4(vec3(),vec3(0,1,0),AJ.RWP);
//        mat4 T_RF1 = T_RWP * mat4(vec3(),vec3(0,0,1),AJ.RF1);

        mat4 T_LSP = T_SC * mat4(offset_s_center2ls, vec3(0,1,0),AJ.LSP);
        mat4 T_LSR = T_LSP * mat4(vec3(),vec3(1,0,0),AJ.LSR);
//        mat4 T_LSY = T_LSR * mat4(vec3(),vec3(0,0,1),AJ.LSY);
//        mat4 T_LEB = T_LSY * mat4(offset_uarm,vec3(0,1,0),AJ.LEB);
//        mat4 T_LWY = T_LEB * mat4(offset_larm,vec3(0,0,1),AJ.LWY);
//        mat4 T_LWP = T_LWY * mat4(vec3(),vec3(0,1,0),AJ.LWP);
//        mat4 T_LF1 = T_LWP * mat4(vec3(),vec3(0,0,1),AJ.LF1);

        vec4 pCOM_pel = c_pel;
//        vec4 pCOM_torso  = T_SC*c_torso;

        vec4 pCOM_rsp = T_RSP*c_rsp;
        vec4 pCOM_rsr = T_RSR*c_rsr;
//        vec4 pCOM_rsy = T_RSY*c_rsy;
//        vec4 pCOM_reb = T_REB*c_reb;
//        vec4 pCOM_rwy = T_RWY*c_rwy;
//        vec4 pCOM_rwp = T_RWP*c_rwp;
//        vec4 pCOM_rf1 = T_RF1*c_rf1;

        vec4 pCOM_lsp = T_LSP*c_lsp;
        vec4 pCOM_lsr = T_LSR*c_lsr;
//        vec4 pCOM_lsy = T_LSY*c_lsy;
//        vec4 pCOM_leb = T_LEB*c_leb;
//        vec4 pCOM_lwy = T_LWY*c_lwy;
//        vec4 pCOM_lwp = T_LWP*c_lwp;
//        vec4 pCOM_lf1 = T_LF1*c_lf1;

        double m_rarm = m_rsp+m_rsr;//+m_rsy+m_reb+m_rwy+m_rwp+m_rf1;
        double m_larm = m_lsp+m_lsr;//+m_lsy+m_leb+m_lwy+m_rwp+m_lf1;

        vec4 pCOM_rarm = (pCOM_rsp*m_rsp + pCOM_rsr*m_rsr);// + pCOM_rsy*m_rsy + pCOM_reb*m_reb + pCOM_rwy*m_rwy + pCOM_rwp*m_rwp + pCOM_rf1*m_rf1)/m_rarm;
        vec4 pCOM_larm = (pCOM_lsp*m_lsp + pCOM_lsr*m_lsr);// + pCOM_lsy*m_lsy + pCOM_leb*m_leb + pCOM_lwy*m_lwy + pCOM_lwp*m_lwp + pCOM_lf1*m_lf1)/m_larm;

        //COM_UB = (pCOM_pel*m_pel + pCOM_torso*m_torso + pCOM_rarm*m_rarm + pCOM_larm*m_larm)/(m_rarm + m_larm + m_pel + m_torso);
        COM_UB = (pCOM_rarm*m_rarm + pCOM_larm*m_larm)/(m_rarm + m_larm + m_pel);

//                //TTTTTTTTTTTTTTT
//                printf("m_ub %f COM_UB %f %f %f\n"
//                       ,(m_rarm + m_larm + m_pel + m_torso)
//                       ,(COM_UB.x,COM_UB.y,COM_UB.z));

        return COM_UB;//set initially first// auto set c_ub
    }
//    int setUB(vec3 COM_UB)
//    {
//        c_ub = vec4(COM_UB);
//        //printf("c_ub %f %f %f \n",c_ub.x,c_ub.y,c_ub.z);
//    }
    ArmPos FK_UB(ArmJoints AJ){
        ArmPos ap;
        ap.qPel = AJ.qPel;
        ap.pPel = AJ.pPel;

        vec3 pPel = AJ.pPel;
        quat qPel  = AJ.qPel;

        mat4 T_PEL = mat4(pPel,mat3(qPel));
        mat4 T_RSP = T_PEL * mat4(offset_s_center2rs,vec3(0,1,0),AJ.RSP);
        mat4 T_RSR = T_RSP * mat4(vec3(),vec3(1,0,0),AJ.RSR);
        mat4 T_RH = T_RSR * mat4(offset_uarm,vec3(0,1,0),0);

        ap.pRH = (T_RH*vec3(0,0,0));
        ap.qRH = quat(mat3(T_RH));

        mat4 T_LSP = T_PEL * mat4(offset_s_center2ls, vec3(0,1,0),AJ.LSP);
        mat4 T_LSR = T_LSP * mat4(vec3(),vec3(1,0,0),AJ.LSR);
        mat4 T_LH = T_LSR * mat4(offset_uarm,vec3(0,1,0),0);

        ap.pLH = (T_LH*vec3(0,0,0));
        ap.qLH = quat(mat3(T_LH));

        vec4 pCOM_rsp = T_RSP*c_rsp;
        vec4 pCOM_rsr = T_RSR*c_rsr;

        vec4 pCOM_lsp = T_LSP*c_lsp;
        vec4 pCOM_lsr = T_LSR*c_lsr;

        double m_rarm = m_rsp + m_rsr;
        double m_larm = m_lsp + m_lsr;

        vec4 pCOM_rarm = (pCOM_rsp*m_rsp + pCOM_rsr*m_rsr)/m_rarm;
        vec4 pCOM_larm = (pCOM_lsp*m_lsp + pCOM_lsr*m_lsr)/m_larm;

        ap.pCOM = (pCOM_rarm*m_rarm + pCOM_larm*m_larm)/(m_rarm + m_larm);

        return ap;
    }

    FeetPos FK(LegJoints LJ)
    {
        FeetPos rp;
        rp.qPel = LJ.qPel;
        rp.pPel = LJ.pPel;
        rp.pPelz = LJ.pPel.z;
        vec3 pPel = LJ.pPel;
        quat qPel  = LJ.qPel;
        mat4 T_PEL = mat4(pPel,mat3(qPel));
        mat4 T_RHY = T_PEL*mat4(offset_p2rHR,vec3(0,0,1),LJ.RHY);
        mat4 T_RHR = T_RHY*mat4(vec3(),vec3(1,0,0),LJ.RHR);
        mat4 T_RHP = T_RHR*mat4(offset_rHR2rHP,vec3(0,1,0),LJ.RHP);
        mat4 T_RKN = T_RHP*mat4(offset_uleg,vec3(0,1,0),LJ.RKN);
        mat4 T_RAP = T_RKN*mat4(offset_lleg,vec3(0,1,0),LJ.RAP);
        mat4 T_RAR = T_RAP*mat4(vec3(),vec3(1,0,0),LJ.RAR);
        mat4 T_RFOOT = T_RAR*mat4(offset_foot,vec3(0,1,0),0);

        rp.pRF = (T_RFOOT*vec3(0,0,0));
        rp.qRF = quat(mat3(T_RFOOT));

        mat4 T_LHY = T_PEL*mat4(offset_p2lHR,vec3(0,0,1),LJ.LHY);
        mat4 T_LHR = T_LHY*mat4(vec3(),vec3(1,0,0),LJ.LHR);
        mat4 T_LHP = T_LHR*mat4(offset_lHR2lHP,vec3(0,1,0),LJ.LHP);
        mat4 T_LKN = T_LHP*mat4(offset_uleg,vec3(0,1,0),LJ.LKN);
        mat4 T_LAP = T_LKN*mat4(offset_lleg,vec3(0,1,0),LJ.LAP);
        mat4 T_LAR = T_LAP*mat4(vec3(),vec3(1,0,0),LJ.LAR);
        mat4 T_LFOOT = T_LAR*mat4(offset_foot,vec3(0,1,0),0);


        rp.pLF = (T_LFOOT*vec3(0,0,0));
        rp.qLF = quat(mat3(T_LFOOT));


        vec4 pCOM_pel = T_PEL*vec4(c_pel);

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

        vec4 pCOM_rleg = (pCOM_rhy*m_rhy + pCOM_rhr*m_rhr + pCOM_rhp*m_rhp + pCOM_rkn*m_rkn + pCOM_rap*m_rap + pCOM_rar*m_rar)/m_rleg;
        vec4 pCOM_lleg = (pCOM_lhy*m_lhy + pCOM_lhr*m_lhr + pCOM_lhp*m_lhp + pCOM_lkn*m_lkn + pCOM_lap*m_lap + pCOM_lar*m_lar)/m_lleg;

        //TTTTTTTTTTTTTTT
//                printf("EE m_ub %f m_rleg %f m_lleg %f pCOM_rleg %f %f %f pCOM_lleg %f %f %f \n"
//                       ,m_ub, m_rleg, m_lleg
//                       ,pCOM_rleg.x,pCOM_rleg.y,pCOM_rleg.z
//                       ,pCOM_lleg.x,pCOM_lleg.y,pCOM_lleg.z);


        rp.pCOM = (pCOM_pel*m_pel + pCOM_rleg*m_rleg + pCOM_lleg*m_lleg)/(m_pel + m_rleg + m_lleg);

        return rp;
    }

    LegJoints IK_pel(vec3 pPel, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF)
    {
        LegJoints LJ;

        LJ.pPel = pPel;
        LJ.qPel = qPel;

        mat3 mPEL = mat3(qPel);

        //------------------------- Right Leg---------------------------
        vec3 g_HR = pPel + mat3(qPel)*offset_p2rHR;

        //solve q6(RAR)
        mat3 mRF = mat3(qRF);
        mat3 imRF = mat3(qRF); imRF.transpose();

        vec3 g_F2HR = g_HR - pRF; // put foot position to zero;
        vec3 f_F2HR = imRF*g_F2HR; // ref frame changed to foot frame

        vec3 f_F2AR = -offset_foot;
        vec3 f_AR2HR = f_F2HR - f_F2AR;

//        tan(q6) = -(f_AR2HR.y - l2cos(q6))/(f_AR2HR.z - l2cos(q6))
//        tan(q6/2) = t , cos(q6) = 2t/(1+t^2) , sin(q6) = (1 - t^2)/(1 + t^2)
//        t  = (Z +- (Y^2 + Z^2 - l2^2)^(1/2))/(Y + l2)

        double t = -(f_AR2HR.z - sqrt(f_AR2HR.y*f_AR2HR.y + f_AR2HR.z*f_AR2HR.z - HR2HPy*HR2HPy))/(f_AR2HR.y + HR2HPy);

        LJ.RAR = 2*atan(t); //q6  -pi/2 ~ pi/2

        // solve q2(RHR) and q1(RHY) by Analytic Approch
        // Homogeneus Transformation matrix
        vec3 g_PEL2F = pRF - pPel;

        mat3 imPEL = mat3(qPel);
        imPEL.inverse();

        mat3 rot0toF = imPEL*mRF;
        mat3 irot0toF = rot0toF; irot0toF.transpose();
        vec3 pel_PEL2F = imPEL*g_PEL2F;

        mat4 T0toF = mat4(pel_PEL2F,rot0toF);

        mat3 rot6toF = rot6toF.eye();
        mat3 irot6toF = rot6toF; irot6toF.transpose();

        mat4 T6toF = mat4(offset_foot, rot6toF);

        mat3 rot5to6 = mat3(vec3(1,0,0),LJ.RAR);

        mat3 irot5to6 = rot5to6;
        irot5to6.transpose();
//        cout<<"irot5to6: "<<endl;
//        cout<<irot5to6.m00<<", "<<irot5to6.m01<<", "<<irot5to6.m02<<endl;
//        cout<<irot5to6.m10<<", "<<irot5to6.m11<<", "<<irot5to6.m12<<endl;
//        cout<<irot5to6.m20<<", "<<irot5to6.m21<<", "<<irot5to6.m22<<endl;

        mat4 T5to6 = mat4(vec3(0,0,0),rot5to6);

        mat3 rot0to5 = rot0toF*irot5to6;


        // 0T5 = 0TF*6TF'*5T6'
        mat4 T0to5 = T0toF*mat4(-offset_foot, irot6toF)*mat4(-irot5to6*vec3(0,0,0), irot5to6);

        LJ.RHR = asin(rot0to5.m21); // rot0to5.m21 = sin(q2)  -PI/2 ~ PI/2

        double SINq1 = -rot0to5.m01/cos(LJ.RHR); //rot0to5.m01 = -cos(q2)*sin(q1);
        double COSq1 = rot0to5.m11/cos(LJ.RHR); //rot0to5.m11 = cos(q1)*cos(q2);

        LJ.RHY = atan2(SINq1, COSq1);

        // solve q3+q4+q5 (RHP+RKN+RAP)
        mat3 rot0to1 = mat3(vec3(0,0,1),LJ.RHY);
        mat3 irot0to1 = rot0to1; irot0to1.transpose();
        mat3 rot1to2 = mat3(vec3(1,0,0),LJ.RHR);
        mat3 irot1to2 = rot1to2; irot1to2.transpose();
        mat3 rot2to5 = irot1to2*irot0to1*rot0to5;

//      rot2to5 = [  cos(q3 + q4 + q5), 0, sin(q3 + q4 + q5)]
//                [                  0, 1,                 0]
//                [ -sin(q3 + q4 + q5), 0, cos(q3 + q4 + q5)]

        double COSq345 = rot2to5.m00;
        double SINq345 = rot2to5.m02;

        double q345 = atan2(SINq345, COSq345);

        // solve q4
        vec3 pel_HR2HP = rot0to1*rot1to2*offset_rHR2rHP;
//        cout<<"pel_HR2HP: "<<pel_HR2HP.x<<", "<<pel_HR2HP.y<<", "<<pel_HR2HP.z<<endl;
        vec3 f_HR2HP = irot0toF*pel_HR2HP;
//        cout<<"f_HR2HP: "<<f_HR2HP.x<<", "<<f_HR2HP.y<<", "<<f_HR2HP.z<<endl;
        vec3 f_AR2HP = f_AR2HR + f_HR2HP;
//        cout<<"f_AR2HP: "<<f_AR2HP.x<<", "<<f_AR2HP.y<<", "<<f_AR2HP.z<<endl;

        double l_AR2HP = f_AR2HP.norm();
//        cout<<"l_AR2HP: "<<l_AR2HP<<endl;

        if(l_AR2HP > ULEG + LLEG - 0.01){
            l_AR2HP = ULEG + LLEG - 0.01;
            std::cout<<"right Knee goes to Singular position"<<std::endl;
        }

        double COS_PI_minus_q4 = (ULEG*ULEG + LLEG*LLEG - l_AR2HP*l_AR2HP)/(2*ULEG*LLEG);

        if(COS_PI_minus_q4 > 1){
            COS_PI_minus_q4 = 1;
        }
        else if(COS_PI_minus_q4 < -1){
            COS_PI_minus_q4 = -1;
        }

        LJ.RKN = PI - acos(COS_PI_minus_q4);

        //solve q3
        //T2to5 = T1to2'*T0to1'*T0to5;
        mat4 T2to5 = mat4(-irot1to2*vec3(0,0,0), irot1to2)*mat4(-irot0to1*offset_p2rHR, irot0to1)*T0to5;

        // T2to5(1,4) = - l5*(cos(q3)*sin(q4) + cos(q4)*sin(q3)) - l4*sin(q3)
        // T2to5(3,4) = - l3 - l5*(cos(q3)*cos(q4) - sin(q3)*sin(q4)) - l4*cos(q3)

        double A1 = -LLEG*sin(LJ.RKN);
        double A2 = -LLEG*cos(LJ.RKN) - ULEG;
        double A3 = T2to5.m03;

        double B1 = -LLEG*cos(LJ.RKN) - ULEG;
        double B2 = LLEG*sin(LJ.RKN);
        double B3 = HR2HPz + T2to5.m23;

        double SINq3 = (A3*B1 - B3*A1)/(A2*B1 - A1*B2);
        double COSq3 = (A3*B2 - B3*A2)/(A1*B2 - A2*B1);

        LJ.RHP = atan2(SINq3, COSq3);

        // solve q5 (RAP)
        LJ.RAP = q345 - LJ.RKN - LJ.RHP;


        //------------------------- Left Leg---------------------------
        g_HR = pPel + mat3(qPel)*offset_p2lHR;

        //solve q6(RAR)
        mat3 mLF = mat3(qLF);
        mat3 imLF = mLF; imLF.transpose();

        g_F2HR = g_HR - pLF; // put foot position to zero;
        f_F2HR = imLF*g_F2HR; // ref frame changed to foot frame

        f_F2AR = -offset_foot;
        f_AR2HR = f_F2HR - f_F2AR;

//        tan(q6) = -(f_AR2HR.y + l2cos(q6))/(f_AR2HR.z + l2cos(q6))
//        tan(q6/2) = t , cos(q6) = 2t/(1+t^2) , sin(q6) = (1 - t^2)/(1 + t^2)
//        t  = (Z +- (Y^2 + Z^2 - l2^2)^(1/2))/(Y - l2)

        t = -(f_AR2HR.z - sqrt(f_AR2HR.y*f_AR2HR.y + f_AR2HR.z*f_AR2HR.z - HR2HPy*HR2HPy))/(f_AR2HR.y - HR2HPy);

        LJ.LAR = 2*atan(t); //q6  -pi/2 ~ pi/2

        // solve q2(RHR) and q1(RHY) by Analytic Approch
        // Homogeneus Transformation matrix
        g_PEL2F = pLF - pPel;

        rot0toF = imPEL*mLF;
        irot0toF = rot0toF; irot0toF.transpose();
        pel_PEL2F = imPEL*g_PEL2F;

        T0toF = mat4(pel_PEL2F,rot0toF);

        rot6toF = rot6toF.eye();
        irot6toF = rot6toF; irot6toF.transpose();

        T6toF = mat4(offset_foot, rot6toF);

        rot5to6 = mat3(vec3(1,0,0),LJ.LAR);
        irot5to6 = rot5to6;
        irot5to6.transpose();

        T5to6 = mat4(vec3(0,0,0),rot5to6);

        rot0to5 = rot0toF*irot5to6;

        // 0T5 = 0TF*6TF'*5T6'
        T0to5 = T0toF*mat4(-irot6toF*offset_foot, irot6toF)*mat4(-irot5to6*vec3(0,0,0), irot5to6);

        LJ.LHR = asin(rot0to5.m21); // rot0to5.m21 = sin(q2)  -PI/2 ~ PI/2

        SINq1 = -rot0to5.m01/cos(LJ.LHR); //rot0to5.m01 = -cos(q2)*sin(q1);
        COSq1 = rot0to5.m11/cos(LJ.LHR); //rot0to5.m11 = cos(q1)*cos(q2);

        LJ.LHY = atan2(SINq1, COSq1);

        // solve q3+q4+q5 (RHP+RKN+RAP)
        rot0to1 = mat3(vec3(0,0,1),LJ.LHY);
        irot0to1 = rot0to1; irot0to1.transpose();
        rot1to2 = mat3(vec3(1,0,0),LJ.LHR);
        irot1to2 = rot1to2; irot1to2.transpose();
        rot2to5 = irot1to2*irot0to1*rot0to5;

//      rot2to5 = [  cos(q3 + q4 + q5), 0, sin(q3 + q4 + q5)]
//                [                  0, 1,                 0]
//                [ -sin(q3 + q4 + q5), 0, cos(q3 + q4 + q5)]

        COSq345 = rot2to5.m00;
        SINq345 = rot2to5.m02;

        q345 = atan2(SINq345, COSq345);

        // solve q4
        pel_HR2HP = rot0to1*rot1to2*offset_lHR2lHP;
        f_HR2HP = rot0toF.transpose()*pel_HR2HP;
        f_AR2HP = f_AR2HR + f_HR2HP;

        l_AR2HP = f_AR2HP.norm();

        if(l_AR2HP > ULEG + LLEG - 0.01){
            l_AR2HP = ULEG + LLEG - 0.01;
            std::cout<<"left Knee goes to Singular position"<<std::endl;
        }

        COS_PI_minus_q4 = (ULEG*ULEG + LLEG*LLEG - l_AR2HP*l_AR2HP)/(2*ULEG*LLEG);

        if(COS_PI_minus_q4 > 1){
            COS_PI_minus_q4 = 1;
        }
        else if(COS_PI_minus_q4 < -1){
            COS_PI_minus_q4 = -1;
        }

        LJ.LKN = PI - acos(COS_PI_minus_q4);

        //solve q3
        //T2to5 = T1to2'*T0to1'*T0to5;
        T2to5 = mat4(-irot1to2*vec3(0,0,0), irot1to2)*mat4(-irot0to1*offset_p2lHR, irot0to1)*T0to5;

        // T2to5(1,4) = - l5*(cos(q3)*sin(q4) + cos(q4)*sin(q3)) - l4*sin(q3)
        // T2to5(3,4) = - l3 - l5*(cos(q3)*cos(q4) - sin(q3)*sin(q4)) - l4*cos(q3)

        A1 = -LLEG*sin(LJ.LKN);
        A2 = -LLEG*cos(LJ.LKN) - ULEG;
        A3 = T2to5.m03;

        B1 = -LLEG*cos(LJ.LKN) - ULEG;
        B2 = LLEG*sin(LJ.LKN);
        B3 = HR2HPz + T2to5.m23;

        SINq3 = (A3*B1 - B3*A1)/(A2*B1 - A1*B2);
        COSq3 = (A3*B2 - B3*A2)/(A1*B2 - A2*B1);

        LJ.LHP = atan2(SINq3, COSq3);

        // solve q5 (RAP)
        LJ.LAP = q345 - LJ.LKN - LJ.LHP;



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

    LegJoints IK_COM_xy(vec3 pCOM_des, double pPelz, quat qPel, vec3 pRF, quat qRF, vec3 pLF, quat qLF)
    {

        LegJoints LJ;
        FeetPos FP;
        FP.pCOM = vec3(-10,0,pPelz);
        vec3 pPel = pCOM_des;//find this.
        pPel.z = pPelz;
        int cnt = 0;

        while(((FP.pCOM.x - pCOM_des.x)*(FP.pCOM.x - pCOM_des.x) + (FP.pCOM.y - pCOM_des.y)*(FP.pCOM.y - pCOM_des.y)) > err_max*err_max)
        {
            LJ = IK_pel(pPel,qPel,pRF,qRF,pLF,qLF);//loop loop loop

            FP = FK(LJ);
            vec3 dpCOM = pCOM_des - FP.pCOM;
            dpCOM.z = 0;

            pPel = pPel + dpCOM;//should be improved
            pPel.z = pPelz;
            cnt++;
            //cout<<"cnt: "<<cnt<<"pel.x : "<<pPel.x<<"  pel.y : "<<pPel.y<<"  pel.z"<<pPel.z<<endl;
        }
        LJ.pPel = pPel;
        return LJ;
    }




};

#endif // HBINVERSE

