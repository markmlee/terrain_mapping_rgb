#ifndef HB_STATE_EST
#define HB_STATE_EST
#include "BP_RBDL.h"
#include "HB_types.h"
#include "HB_inverse.h"
#include "HB_functions.h"

#define RF_CONTACT         -1
#define LF_CONTACT          1
#define DSP_CONTACT         0

using namespace std;

class HB_SE
{
    HB_inverse Hi;
    BP_RBDL bp_rbdl;

public:
    RobotSensor ROBOTSEN;
    RobotStates ROBOTSTATE;
    double dt, Contact_th;
    double g;

    //contact EST
    bool RFCT, LFCT, RFCTold, LFCTold;

    //EST FK
    bool firstCon;

    vec3 last_pCOM, last_dCOM;
    int Qcnt;
    vec3 pPelold;
    vec3 g_pRF_onGnd, g_pLF_onGnd;
    quat g_qRF_onGnd, g_qLF_onGnd;
    int Contact_state;


    HB_SE(){
        dt = 0.005;
        g = 9.81;
        Qcnt = 0;

        //contact threashold
        Contact_th = 10; //10 N

        ROBOTSTATE.Qnow = VectorNd::Zero(bp_rbdl.Robot->q_size);
        ROBOTSTATE.dQnow = VectorNd::Zero(bp_rbdl.Robot->dof_count);
        ROBOTSTATE.Xnow = VectorNd::Zero(bp_rbdl.Robot->dof_count);
        ROBOTSTATE.dXnow = VectorNd::Zero(bp_rbdl.Robot->dof_count);

        ROBOTSTATE.cRF = ROBOTSTATE.cLF = true;

        RFCT = true;
        LFCT = true;
        RFCTold = true;
        LFCTold = true;

        Contact_state = DSP_CONTACT;
    }

    void contact_est(){
        Contact_th = 20.0;
        if(ROBOTSTATE.F_RF.norm()>Contact_th){
            RFCT = true;
        }
        else{
            RFCT = false;
        }
        if(ROBOTSTATE.F_LF.norm()>Contact_th){
            LFCT = true;
        }
        else{
            LFCT = false;
        }

        if(RFCT && RFCTold){
            ROBOTSTATE.cRF = true;
            g_pRF_onGnd = ROBOTSTATE.CSP.pRF;
            g_qRF_onGnd = ROBOTSTATE.CSP.qRF;
        }
        else if(!RFCT && !RFCTold){
            ROBOTSTATE.cRF = false;
        }

        if(LFCT && LFCTold){
            ROBOTSTATE.cLF = true;
            g_pLF_onGnd = ROBOTSTATE.CSP.pLF;
            g_qLF_onGnd = ROBOTSTATE.CSP.qLF;
        }
        else if(!LFCT && !LFCTold){
            ROBOTSTATE.cLF = false;
        }

        if(ROBOTSTATE.cRF == true && ROBOTSTATE.cLF == false) Contact_state = RF_CONTACT;
        if(ROBOTSTATE.cRF == false && ROBOTSTATE.cLF == true) Contact_state = LF_CONTACT;
        if(ROBOTSTATE.cRF == true && ROBOTSTATE.cLF == true) ;//Contact_state = DSP_CONTACT;
        if(ROBOTSTATE.cRF == false && ROBOTSTATE.cLF == false); //if the both foot are not in contact, maintain previous Contact_state

        RFCTold = RFCT;
        LFCTold = LFCT;

    }

    RobotStates StateEst(RobotSensor _ROBOTSEN){
        ROBOTSEN = _ROBOTSEN;
        //JSP copy (execpt pPel)
        ROBOTSTATE.JSP = ROBOTSEN.JSP;
        ROBOTSTATE.JSP.qPel = ROBOTSEN.IMUquat;
        ROBOTSTATE.CSP.qPel = ROBOTSEN.IMUquat;

        //JSV copy (execpt dpPel)
        ROBOTSTATE.JSV = ROBOTSEN.JSV;
        ROBOTSTATE.JSV.dqPel = ROBOTSEN.IMUomega;
        ROBOTSTATE.CSV.dqPel = ROBOTSEN.IMUomega;

        //Sensor Value copy
        ROBOTSTATE.IMUangle = ROBOTSEN.IMUangle;
        ROBOTSTATE.IMUangle_comp = ROBOTSEN.IMUangle_comp;
        ROBOTSTATE.IMUomega = ROBOTSEN.IMUomega;
        ROBOTSTATE.IMUquat = ROBOTSEN.IMUquat;
        ROBOTSTATE.IMULocalW = ROBOTSEN.IMULocalW;

        ROBOTSTATE.F_RF = ROBOTSEN.F_RF;
        ROBOTSTATE.F_LF = ROBOTSEN.F_LF;
        ROBOTSTATE.M_RF = ROBOTSEN.M_RF;
        ROBOTSTATE.M_LF = ROBOTSEN.M_LF;

        ROBOTSTATE.ACC_RF = ROBOTSEN.ACC_RF;
        ROBOTSTATE.ACC_LF = ROBOTSEN.ACC_LF;


        if(ROBOTSTATE.IMUomega.x>100)  ROBOTSTATE.IMUomega.x = 100;
        if(ROBOTSTATE.IMUomega.x<-100) ROBOTSTATE.IMUomega.x = -100;
        if(ROBOTSTATE.IMUomega.y>100)  ROBOTSTATE.IMUomega.y = 100;
        if(ROBOTSTATE.IMUomega.y<-100) ROBOTSTATE.IMUomega.y = -100;
        if(ROBOTSTATE.IMUomega.z>100)  ROBOTSTATE.IMUomega.z = 100;
        if(ROBOTSTATE.IMUomega.z<-100) ROBOTSTATE.IMUomega.z = -100;

/*
        //contact measure & update absolute foot position
        contact_est();


        // pPel
        LegJoints LJ;
        for(int i=0; i<12 ; i++){
            LJ.LJ_Array[i] = ROBOTSTATE.JSP.JSP_Array[i];
        }
        LJ.pPel = vec3();
        LJ.qPel = quat(); //assume pelvis is fixed in some frame
        LJ.WST = 0.0;
        
        FeetPos FP = Hi.FK(LJ);
        mat3 g_R_pel = mat3(ROBOTSTATE.IMUquat);

        if(Contact_state == RF_CONTACT){
            //global pPel
            //calc ZMP in RF frame
            vec3 rf_RF2ZMP = ZMP_calc_global(vec3(), quat(), ROBOTSEN.F_RF, ROBOTSEN.M_RF, vec3(), quat(), vec3(),vec3());

            mat3 pel_R_rf = mat3(FP.qRF);

            vec3 pel_RF2ZMP = pel_R_rf*rf_RF2ZMP;

            vec3 pel_RF2PEL = FP.pPel - FP.pRF;

            vec3 pel_ZMP2PEL = pel_RF2PEL - pel_RF2ZMP;

            vec3 g_ZMP2PEL = g_R_pel*pel_ZMP2PEL; //std::cout<<"g_ZMP2PEL: "<<g_ZMP2PEL.x<<", "<<g_ZMP2PEL.y<<", "<<g_ZMP2PEL.z<<")"<<std::endl;

            vec3 g_RF2ZMP = g_R_pel*pel_RF2ZMP; //std::cout<<"g_RF2ZMP: "<<g_RF2ZMP.x<<", "<<g_RF2ZMP.y<<", "<<g_RF2ZMP.z<<")"<<std::endl;

            vec3 g_pPel = g_pRF_onGnd + g_RF2ZMP + g_ZMP2PEL; //std::cout<<"g_pRF_onGnd: "<<g_pRF_onGnd.x<<", "<<g_pRF_onGnd.y<<", "<<g_pRF_onGnd.z<<")"<<std::endl;




            ROBOTSTATE.JSP.pPel = g_pPel;
            ROBOTSTATE.CSP.pPel = g_pPel;

            //global RF (on Gnd)
            ROBOTSTATE.CSP.pRF = g_pRF_onGnd;
            ROBOTSTATE.CSP.qRF = g_R_pel*pel_R_rf; //g_qRF_onGnd;

            //global LF (in the Air)
            vec3 pel_PEL2LF = FP.pLF - FP.pPel;
            mat3 pel_R_lf = mat3(FP.qLF);
            vec3 g_PEL2LF = g_R_pel*pel_PEL2LF;
            vec3 g_pLF = g_pPel + g_PEL2LF;

            ROBOTSTATE.CSP.pLF = g_pLF;
            ROBOTSTATE.CSP.qLF = g_R_pel*pel_R_lf;

            //global COM
            vec3 pel_PEL2COM = FP.pCOM - FP.pPel;
            vec3 g_pCOM = g_pPel + g_R_pel*pel_PEL2COM;

            ROBOTSTATE.CSP.pCOM = g_pCOM;
        }
        if(Contact_state == LF_CONTACT){
            //global pPel
            //calc ZMP in LF frame
            vec3 lf_LF2ZMP = ZMP_calc_global(vec3(), quat(), vec3(), vec3(), vec3(), quat(), ROBOTSEN.F_LF,ROBOTSEN.M_LF);

            mat3 pel_R_lf = mat3(FP.qLF);

            vec3 pel_LF2ZMP = pel_R_lf*lf_LF2ZMP;

            vec3 pel_LF2PEL = FP.pPel - FP.pLF;

            vec3 pel_ZMP2PEL = pel_LF2PEL - pel_LF2ZMP;

            vec3 g_ZMP2PEL = g_R_pel*pel_ZMP2PEL;

            vec3 g_LF2ZMP = g_R_pel*pel_LF2ZMP;

            vec3 g_pPel = g_pLF_onGnd + g_LF2ZMP + g_ZMP2PEL;

            ROBOTSTATE.JSP.pPel = g_pPel;
            ROBOTSTATE.CSP.pPel = g_pPel;

            //global LF (on Gnd)
            ROBOTSTATE.CSP.pLF = g_pLF_onGnd;
            ROBOTSTATE.CSP.qLF = g_R_pel*pel_R_lf;//g_qLF_onGnd;

            //global RF (in the Air)
            vec3 pel_PEL2RF = FP.pRF - FP.pPel;
            mat3 pel_R_rf = mat3(FP.qRF);
            vec3 g_PEL2RF = g_R_pel*pel_PEL2RF;
            vec3 g_pRF = g_pPel + g_PEL2RF;

            ROBOTSTATE.CSP.pRF = g_pRF;
            ROBOTSTATE.CSP.qRF = g_R_pel*pel_R_rf;

            //global COM
            vec3 pel_PEL2COM = FP.pCOM - FP.pPel;
            vec3 g_pCOM = g_pPel + g_R_pel*pel_PEL2COM;

            ROBOTSTATE.CSP.pCOM = g_pCOM;
        }
        if(Contact_state == DSP_CONTACT){
            // global pel
//            vec3 g_ZMP = ZMP_calc_global(g_pRF_onGnd, g_qRF_onGnd, ROBOTSEN.F_RF, ROBOTSEN.M_RF,
//                                         g_pLF_onGnd, g_qLF_onGnd, ROBOTSEN.F_LF, ROBOTSEN.M_LF);
//            vec3 pel_ZMP = ZMP_calc_global(FP.pRF, FP.qRF, ROBOTSEN.F_RF, ROBOTSEN.M_RF,
//                                           FP.pLF, FP.qLF, ROBOTSEN.F_LF, ROBOTSEN.M_RF);
            vec3 g_ZMP = (g_pRF_onGnd + g_pLF_onGnd)/2;
            vec3 pel_ZMP = (FP.pRF + FP.pLF)/2;

            vec3 pel_ZMP2PEL = FP.pPel - pel_ZMP;
            vec3 g_ZMP2PEL = g_R_pel*pel_ZMP2PEL;

            vec3 g_pPel = g_ZMP + g_ZMP2PEL;

            ROBOTSTATE.JSP.pPel = g_pPel;
            ROBOTSTATE.CSP.pPel = g_pPel;

            // global COM
            vec3 pel_PEL2COM = FP.pCOM - FP.pPel;
            vec3 g_pCOM = g_pPel + g_R_pel*pel_PEL2COM;

            ROBOTSTATE.CSP.pCOM = g_pCOM;

            mat3 pel_R_rf = mat3(FP.qRF);
            mat3 pel_R_lf = mat3(FP.qLF);

            //global RF
            ROBOTSTATE.CSP.pRF = g_pRF_onGnd;
            ROBOTSTATE.CSP.qRF = g_R_pel*pel_R_rf;//g_qRF_onGnd;

            //global LF
            ROBOTSTATE.CSP.pLF = g_pLF_onGnd;
            ROBOTSTATE.CSP.qLF = g_R_pel*pel_R_lf;//g_qLF_onGnd;
        }

        ROBOTSTATE.Qnow = ROBOTSTATE.getQnow(bp_rbdl.Robot->q_size);

        // dpPel
        LJointSpacePOS JSPtemp = ROBOTSTATE.JSP;
        JSPtemp.pPel = vec3();
        JSPtemp.qPel = quat();
        VectorNd Qtemp = JSPtemp.getQnow(bp_rbdl.Robot->q_size);

        LJointSpaceVEL JSVtemp = ROBOTSTATE.JSV;
        JSVtemp.dpPel = vec3();
        JSVtemp.dqPel = vec3();
        VectorNd dQtemp = JSVtemp.getdQnow(bp_rbdl.Robot->qdot_size);

        bp_rbdl.CalcEndeffectorJacobian3D(Qtemp); //pelvis frame jacobian //JacobianRF3D, JacobianLF3D
        //bp_rbdl.CalcCOMJacobian3D(Qtemp);  //pelvis frame com jacobian  // JacobianCOMall3D

        vec3 g_dpPel_angular;
        vec3 g_dpPel_linear;

        vec3 pel_dpPel_linear;
        VectorNd _pel_dpPel_linear;
        vec3 g_dpPel;

        if(Contact_state == RF_CONTACT || Contact_state == DSP_CONTACT){
            g_dpPel_angular = cross(ROBOTSEN.IMUomega, (ROBOTSTATE.CSP.pPel - g_pRF_onGnd));
            _pel_dpPel_linear = -(bp_rbdl.JacobianRF3D*dQtemp);
            pel_dpPel_linear = vec3(_pel_dpPel_linear[0],_pel_dpPel_linear[1],_pel_dpPel_linear[2]);
            g_dpPel_linear = g_R_pel*pel_dpPel_linear;

            g_dpPel = g_dpPel_angular + g_dpPel_linear;
            ROBOTSTATE.CSV.dpPel = g_dpPel;
            ROBOTSTATE.JSV.dpPel = g_dpPel;
        }
        else if( Contact_state == LF_CONTACT || Contact_state == DSP_CONTACT){
            g_dpPel_angular = cross(ROBOTSEN.IMUomega, (ROBOTSTATE.CSP.pPel - g_pLF_onGnd));
            _pel_dpPel_linear = -(bp_rbdl.JacobianLF3D*dQtemp);
            pel_dpPel_linear = vec3(_pel_dpPel_linear[0],_pel_dpPel_linear[1],_pel_dpPel_linear[2]);
            g_dpPel_linear = g_R_pel*pel_dpPel_linear;

            g_dpPel = g_dpPel_angular + g_dpPel_linear;
            ROBOTSTATE.CSV.dpPel = g_dpPel;
            ROBOTSTATE.JSV.dpPel = g_dpPel;
        }

        ROBOTSTATE.dQnow = ROBOTSTATE.getdQnow(bp_rbdl.Robot->qdot_size);


        // dpRF, dqRF, dpLF, dqLF, dpCOM
        bp_rbdl.CalcEndeffectorJacobian6D(ROBOTSTATE.Qnow); // global jacobian // JacobianRF3D_pos,ori, JacobianLF3D_pos,ori

        Vector3d temp_dpRF = bp_rbdl.JacobianRF3D_pos*ROBOTSTATE.dQnow;
        Vector3d temp_dpLF = bp_rbdl.JacobianLF3D_pos*ROBOTSTATE.dQnow;

        Vector3d temp_dqRF = bp_rbdl.JacobianRF3D_ori*ROBOTSTATE.dQnow;
        Vector3d temp_dqLF = bp_rbdl.JacobianLF3D_ori*ROBOTSTATE.dQnow;


        ROBOTSTATE.CSV.dpRF = vec3(temp_dpRF[0],temp_dpRF[1],temp_dpRF[2]);
        ROBOTSTATE.CSV.dpLF = vec3(temp_dpLF[0],temp_dpLF[1],temp_dpLF[2]);

        ROBOTSTATE.CSV.dqRF = vec3(temp_dqRF[0],temp_dqRF[1],temp_dqRF[2]);
        ROBOTSTATE.CSV.dqLF = vec3(temp_dqLF[0],temp_dqLF[1],temp_dqLF[2]);

        bp_rbdl.CalcCOMMomentum(ROBOTSTATE.Qnow,ROBOTSTATE.dQnow);
        ROBOTSTATE.CSV.dpCOM = vec3(bp_rbdl.dCOM[0],bp_rbdl.dCOM[1],bp_rbdl.dCOM[2]);

        */


        return ROBOTSTATE;

    }

    void Initialize_StateEst(RobotStates _Initial_ROBOTSTATE){
        ROBOTSTATE = _Initial_ROBOTSTATE;
        if(ROBOTSTATE.cRF == true){
            RFCT = true;
            RFCTold = true;
        }
        if(ROBOTSTATE.cLF == true){
            LFCT = true;
            LFCTold = true;
        }
    }

//    void SensorInput(RobotSensor _ROBOTSEN){
//        ROBOTSEN = _ROBOTSEN;
//        ROBOTSTATE.JSP = ROBOTSEN.JSP;
//        ROBOTSTATE.JSV = ROBOTSEN.JSV;
//        ROBOTSTATE.IMUangle = ROBOTSEN.IMUangle;
//        ROBOTSTATE.IMUomega = ROBOTSEN.IMUomega;
//        if(ROBOTSTATE.IMUomega.x>100)  ROBOTSTATE.IMUomega.x = 100;
//        if(ROBOTSTATE.IMUomega.x<-100) ROBOTSTATE.IMUomega.x = -100;
//        if(ROBOTSTATE.IMUomega.y>100)  ROBOTSTATE.IMUomega.y = 100;
//        if(ROBOTSTATE.IMUomega.y<-100) ROBOTSTATE.IMUomega.y = -100;
//        if(ROBOTSTATE.IMUomega.z>100)  ROBOTSTATE.IMUomega.z = 100;
//        if(ROBOTSTATE.IMUomega.z<-100) ROBOTSTATE.IMUomega.z = -100;
//    }





};

#endif // HB_STATE_EST

