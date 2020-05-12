#ifndef HB_DYNAMICS_WALK
#define HB_DYNAMICS_WALK

#include "BasicMatrix.h"
#include <unistd.h>
#include "../../share/Headers/RBSharedMemory.h"
#include <QVector>
#include "HB_functions.h"
#include "BP_RBDL.h"
#include "HB_types.h"
#include "HB_inverse.h"

#define     RFoot              -1
#define     DSP                 0
#define     SSP                 2
#define     LFoot               1
//#define     dt                  0.005

#define     PI                  3.141592
#define     SIGN(x)            (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define     ABS(a)             (((a)<(0)) ? -(a):(a))

using namespace std;

class HB_DynamicWalk
{
public:
    //Basic objects
    RobotStates ROBOTSTATE, ROBOTSTATE_old;
    REFERENCE REF;
    DesiredStates DS, DS_stop;
    BP_RBDL bp_rbdl;
    HB_inverse Hi;

    //Walking variables
    const double dt = 0.005;
    double w, zc;
    double t_step, t_start, t_end, real_t_step, t_total, t_elapsed, dT, dT_original, dT_min,t_stable, t_foot_now, step_len_x, step_len_y;
    int  R_or_L, k, real_time_adjust_count;
    char step_phase_change_flag, real_step_phase_change_flag, real_time_adjust_counter_start_flag;
    unsigned int step_phase,real_step_phase, N_step;
    double init_foot_width;
    char swingFoot;
    vec3 COM_est, dCOM_est, ddCOM_est, ZMP_est, COM_m, COM_m_old, CP_m_filtered, COM_m_filtered,COM_m_filtered1,COM_m_filtered2,dCOM_m_filtered,dCOM_m_filtered1,dCOM_m_filtered2, dCOM_m, ddCOM_m, COM_old, dCOM_old, COM, dCOM, ddCOM, ddCOM_con, dCOM_con, COM_con;
    vec3 COM_choreonoid, dCOM_choreonoid, dCOM_choreonoid_filtered, ddCOM_con_filetered;
    vec3 CP, CP_m, CP_eos, COM_p, dCOM_p, CP_p, ZMP_p, ZMP_ref, ZMP_ref_filtered, COM_cps, dCOM_cps, COM_cpt, dCOM_cpt, CP_cps, ZMP_cps, CP_cpt, ZMP_cpt, cZMP_proj_MAX;
    vec3 pPel_old, pRF, pRF_old, pLF, pLF_old, F_RF, F_LF, M_RF, M_LF, IMUangle, IMUomega, ZMP_global, ZMP_global_filtered,ZMP_global_filtered_old,dZMP_global_filtered, ZMP_local;
    vec3 ZMP_ref_gap, total_CP_eos_gap, CP_eos_gap, CP_eos_gap_old;
    vec3 CP_eos_gap_RF, CP_eos_gap_LF, CP_eos_modi_RF, CP_eos_modi_LF, CP_eos_modi_RF_old, CP_eos_modi_LF_old;
    vec3 F_RF_filtered, F_LF_filtered, M_RF_filtered, M_LF_filtered;
    quat qRF, qRF_old, qLF, qLF_old, qPel, qPel_old;
    quat qPel_m;
    vec3 cZMP, cZMP_proj, CP_eos_modi;
    double dsp_ratio, step_time_gap, step_time_gap_old,FootUp_height, step_time_modi, step_time_modi_old, pFoot_y;
    vec3 RF_z_dz_ddz_old, LF_z_dz_ddz_old, RF_z_dz_ddz_pattern, LF_z_dz_ddz_pattern, RF_x_dx_ddx_old, LF_x_dx_ddx_old, RF_y_dy_ddy_old, LF_y_dy_ddy_old;
    vec3 RF_x_dx_ddx_pattern, LF_x_dx_ddx_pattern, RF_y_dy_ddy_pattern, LF_y_dy_ddy_pattern;
    int dt_gain1, dt_gain2, dt_gain3;
    QVector<STEP_DATA> step_data_buf, step_data_buf_original;  // x, y, R or L, time
    bool walk_done_flag, RF_landing_flag, LF_landing_flag;
    double SAVE[80][200000];

    // Landing Control
    char support_phase;


    //Functions
    DesiredStates InvDyn_Control(RobotStates _ROBOSTATE, REFERENCE _REF);
    DesiredStates StandUp(RobotStates _ROBOTSTATE, RobotStates RS_ini);
    void set_step(vec3 _COM_ini, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, double _t_step, double _N_step, double _step_len_x, double _step_len_y, int _RL_first);
    DesiredStates Walk(RobotStates _ROBOTSTATE);
    void MeasurementInput(RobotStates _RST);
    VectorNd dIK_Solver_COM(REFERENCE _REF);
    VectorNd ddIK_Solver_COM(REFERENCE _REF);
    void save_onestep(int cnt);
    void save_all();


    HB_DynamicWalk(){
        DS.ddQdes = VectorNd::Zero(bp_rbdl.Robot->dof_count);
        DS.Tdes = VectorNd::Zero(bp_rbdl.Robot->dof_count-6);
        DS.Fdes = VectorNd::Zero(6*2);
        DS.Xdes = VectorNd::Zero(bp_rbdl.Robot->dof_count + bp_rbdl.Robot->dof_count-6 + 6*2);

        DS_stop.ddQdes = VectorNd::Zero(bp_rbdl.Robot->dof_count);
        DS_stop.Tdes = VectorNd::Zero(bp_rbdl.Robot->dof_count-6);
        DS_stop.Fdes = VectorNd::Zero(6*2);
        DS_stop.Xdes = VectorNd::Zero(bp_rbdl.Robot->dof_count + bp_rbdl.Robot->dof_count-6 + 6*2);

        //Walking variables
        FootUp_height = 0.13;

        zc = 0.70;
        w = sqrt(9.81/zc);
        step_phase = 0;
        real_step_phase = 0;
        COM_m_old = vec3(0,0,0);
        COM_m_filtered = vec3(0,0,0);
        COM_m_filtered2 = vec3(0,0,0);
        ddCOM_con_filetered = vec3(0,0,0);
        COM_old = vec3(0,0,0);
        dCOM_old = COM_old;
        ZMP_global_filtered = vec3(0,0,0);
        ZMP_global_filtered_old = vec3(0,0,0);
        k = 0; t_elapsed = 0;
        dT_min = t_step*0.02;
        step_phase_change_flag = 1;
        CP_eos_gap = vec3(0,0,0);

        CP_eos_gap_old = vec3(0,0,0);
        step_time_gap = 0;
        step_time_gap_old = 0;
        t_foot_now = 0.0;

        ZMP_ref_gap = vec3(0,0,0);

        dCOM_con = vec3(0,0,0);
        ddCOM_con = vec3(0,0,0);
        walk_done_flag = false;
    }

};

DesiredStates HB_DynamicWalk::InvDyn_Control(RobotStates _ROBOTSTATE, REFERENCE _REF){
    double D_ratio = 1.0;
    double wn = 20.0;
    double Tlim = 100.0;

    ROBOTSTATE = _ROBOTSTATE;
    REF = _REF;

    REF.Qref[0] = 0.0;REF.Qref[1] = 0.0;REF.Qref[2] = 0.0;REF.Qref[3] = 0.0;REF.Qref[4] = 0.0;REF.Qref[5] = 0.0; // floating body
    REF.Qref[bp_rbdl.Robot->q_size-1] = 1.0;

    //REF.Qref[6] = 0.0;REF.Qref[7] = 0.0;REF.Qref[8] = 0.0;REF.Qref[9] = 0.0;REF.Qref[10] = 0.0;REF.Qref[11] = 0.0; // Right Leg



    ROBOTSTATE.JSP.pPel = vec3(0,0,0);
    ROBOTSTATE.JSP.qPel = quat(1,0,0,0);
    ROBOTSTATE.JSV.dpPel = vec3(0,0,0);
    ROBOTSTATE.JSV.dqPel = vec3(0,0,0);


//    for(int i=0;i<12;i++){
//        cout<<"ROBOTSTATE "<<ROBOTSTATE.JSP.JSP_Array[i]<<endl;
//    }

    VectorNd temp_Qnow = ROBOTSTATE.getQnow(bp_rbdl.Robot->q_size);
    VectorNd temp_dQnow = ROBOTSTATE.getdQnow(bp_rbdl.Robot->qdot_size);



    //cout<<"qref: "<<REF.Qref<<endl;

    for(int i=0; i<temp_dQnow.size(); i++){
//            if(i==6||i==7||i==8||i==9||i==10||i==11) z = 1.0;//ankle gain
//            else z=1.0;
//            if(i==10||i==11||i==16||i==17) wn = 20.0;//ankle gain
//            else wn = 1.0;//hip&knee gain

        DS.ddQdes[i] = REF.ddQref[i] + 2*D_ratio*wn*(REF.dQref[i] -temp_dQnow[i]) + wn*wn*(REF.Qref[i] - temp_Qnow[i]);
//            cout << "RE.dQref : " << RE.dQref.transpose() << endl;
//            cout << "ROBOSTATE.dQnow : " << ROBOSTATE.dQnow.transpose() <<endl;
//            cout << "difference : " << 2*z*wn*(RE.dQref.transpose() - ROBOSTATE.dQnow.transpose()) <<endl << endl;

//            cout << "angle diff : " << (RE.Qref[i] - ROBOSTATE.Qnow[i])*R2Df << endl;

    }




    bp_rbdl.CalcInverseDynamics(temp_Qnow, temp_dQnow, DS.ddQdes);

    for (int i=0; i<temp_dQnow.size()-6;i++){
        DS.Tdes[i] = bp_rbdl.Tau[i+6];
        if(DS.Tdes[i]>Tlim) {DS.Tdes[i]=Tlim; cout << "LIM!!" <<endl;}
        if(DS.Tdes[i]<-Tlim) {DS.Tdes[i]=-Tlim;  cout << "LIM!!" <<endl;}
    }


    return DS;

}

DesiredStates HB_DynamicWalk::StandUp(RobotStates _ROBOTSTATE, RobotStates RS_ini){
    //pelvis pos & ori, foot pos & ori, no q, dq, ddq ref(no swing foot)

    ROBOTSTATE = _ROBOTSTATE;

    //Stand up reference generation
    //pelvis pos & ori
    double StdupHeight = RS_ini.CSP.pCOM.z;
//    if(ROBOTSTATE.cRF == true && ROBOTSTATE.cLF == false){
//        REF.CSP.pCOM = vec3(ROBOTSTATE.CSP.pRF.x, ROBOTSTATE.CSP.pRF.y, StdupHeight);
//        REF.cRF = true; REF.cLF = false;
//    }
//    else if(ROBOTSTATE.cRF == false && ROBOTSTATE.cLF == true){
//        REF.CSP.pCOM = vec3(ROBOTSTATE.CSP.pLF.x, ROBOTSTATE.CSP.pLF.y, StdupHeight);
//        REF.cRF = false; REF.cLF = true;
//    }
//    else if(ROBOTSTATE.cRF == true && ROBOTSTATE.cLF == true){
//        vec3 midFoot = (ROBOTSTATE.CSP.pRF + ROBOTSTATE.CSP.pLF)/2;
//        REF.CSP.pCOM = vec3(midFoot.x, midFoot.y, StdupHeight);
//        REF.cRF = true; REF.cLF = true;
//    }

    vec3 midFoot = (ROBOTSTATE.CSP.pRF + ROBOTSTATE.CSP.pLF)/2;
    REF.CSP.pCOM = vec3(midFoot.x, midFoot.y, StdupHeight);

    REF.CSP.qPel = quat();
    REF.JSP.qPel = REF.CSP.qPel;

    //REF contact
    REF.RFdn = true; REF.LFdn = true;
    REF.RFup = false; REF.LFup = false;
    REF.cRF = true; REF.cLF = true;


    //RF, LF pos & ori (current foot pos & ori)
    REF.CSP.pRF = ROBOTSTATE.CSP.pRF;
    REF.CSP.qRF = ROBOTSTATE.CSP.qRF;
    REF.CSP.pLF = ROBOTSTATE.CSP.pLF;
    REF.CSP.qLF = ROBOTSTATE.CSP.qLF;

    // vel & Acc
    REF.CSV.dpPel = vec3();
    REF.CSV.dqPel = vec3();
    REF.JSV.dpPel = vec3();
    REF.JSV.dqPel = vec3();

    REF.CSV.dpRF = vec3();
    REF.CSV.dqRF = vec3();
    REF.CSV.dpLF = vec3();
    REF.CSV.dqLF = vec3();

    REF.CSA.ddpPel = vec3();
    REF.CSA.ddqPel = vec3();
    REF.JSA.ddpPel = vec3();
    REF.JSA.ddqPel = vec3();

    REF.CSA.ddpRF = vec3();
    REF.CSA.ddqRF = vec3();
    REF.CSA.ddpLF = vec3();
    REF.CSA.ddpLF = vec3();

    LegJoints LJ = Hi.IK_COM(REF.CSP.pCOM, REF.CSP.qPel, REF.CSP.pRF, REF.CSP.qRF, REF.CSP.pLF, REF.CSP.qLF);

    //pelvis reference
    REF.CSP.pPel = LJ.pPel;
    REF.JSP.pPel = REF.CSP.pPel;

    //make Qref, dQref, ddQref

    for(int i=0; i<12 ; i++){
        REF.JSP.JSP_Array[i] = LJ.LJ_Array[i];
    }
    REF.Qref = REF.getQref(bp_rbdl.Robot->q_size);

    REF.dQref = REF.getdQref(bp_rbdl.Robot->dof_count);
    REF.ddQref = REF.getddQref(bp_rbdl.Robot->dof_count);

    //calc QP
    DS.Xdes = bp_rbdl.calc_QP(ROBOTSTATE, REF);
    DS.ddQdes = DS.Xdes.segment(0,ROBOTSTATE.dQnow.size());
    DS.Tdes = DS.Xdes.segment(ROBOTSTATE.dQnow.size(),12);
    DS.Fdes = DS.Xdes.segment(30,DS.Xdes.size()-30);

    return DS;

}

void HB_DynamicWalk::set_step(vec3 _COM_ini, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, double _t_step, double _N_step, double _step_len_x, double _step_len_y, int _RL_first){
    // Initialize variable-----------------------------------------------------------------
    t_step = _t_step;
    t_start = t_step/3;
    real_t_step = t_start;
    t_end = t_step*1.1;
    t_stable = t_step*2;
    N_step = _N_step;
    R_or_L = _RL_first;
    k = 0; t_elapsed = 0;
    dT_min = 15*dt;//t_step*0.02;
    dT = real_t_step;
    dT_original = real_t_step;
    t_total = 0;
    step_len_x = _step_len_x;
    step_len_y = _step_len_y;

    step_phase = 0;
    real_step_phase = 0;
    step_phase_change_flag = 1;
    real_step_phase_change_flag = 0;
    real_time_adjust_counter_start_flag = 0;

    cout<<"COM_initial value : ("<<_COM_ini.x<<", "<<_COM_ini.y<<")"<<endl;

    COM = _COM_ini;
    dCOM = vec3(0,0,0);
    ddCOM = vec3(0,0,0);
    COM_old = _COM_ini;
    dCOM_old = vec3(0,0,0);
    COM_m_old = _COM_ini;
    COM_m_filtered = _COM_ini;
    COM_m_filtered2 = _COM_ini;
    ddCOM_con_filetered = vec3(0,0,0);
    COM_cps = _COM_ini;
    dCOM_cps = vec3(0,0,0);
    COM_cpt = _COM_ini;
    dCOM_cpt = vec3(0,0,0);

    ZMP_global_filtered = vec3(0,0,0);
    ZMP_global_filtered_old = vec3(0,0,0);
    step_data_buf.clear();

    ZMP_ref_gap = vec3(0,0,0);

    swingFoot = DSP;
    dsp_ratio = 0.05;
    t_foot_now = 0.0;
    RF_z_dz_ddz_old = vec3(_pRF.z, 0, 0);
    LF_z_dz_ddz_old = vec3(_pLF.z, 0, 0);
    RF_x_dx_ddx_old = vec3(_pRF.x, 0, 0);
    LF_x_dx_ddx_old = vec3(_pLF.x, 0, 0);
    RF_y_dy_ddy_old = vec3(_pRF.y, 0, 0);
    LF_y_dy_ddy_old = vec3(_pLF.y, 0, 0);
    RF_z_dz_ddz_pattern = RF_z_dz_ddz_old;
    LF_z_dz_ddz_pattern = LF_z_dz_ddz_old;
    RF_x_dx_ddx_pattern = RF_x_dx_ddx_old;
    LF_x_dx_ddx_pattern = LF_x_dx_ddx_old;
    RF_y_dy_ddy_pattern = RF_y_dy_ddy_old;
    LF_y_dy_ddy_pattern = LF_y_dy_ddy_old;

    dCOM_con = vec3(0,0,0);
    ddCOM_con = vec3(0,0,0);


    pRF_old = _pRF;
    pLF_old = _pLF;
    pRF = _pRF;
    pLF = _pLF;
    qRF = _qRF;
    qLF = _qLF;

    walk_done_flag = false;

    CP_eos_gap = vec3(0,0,0);
    CP_eos_gap_old = vec3(0,0,0);
    step_time_gap_old = 0;
    CP_eos_modi = vec3();

    CP_eos_gap_RF = vec3(0,0,0);
    CP_eos_modi_RF = vec3(0,0,0);
    CP_eos_modi_RF_old = vec3(0,0,0);
    CP_eos_gap_LF = vec3(0,0,0);
    CP_eos_modi_LF = vec3(0,0,0);
    CP_eos_modi_LF_old = vec3(0,0,0);

    RF_landing_flag = true;
    LF_landing_flag = true;

    step_time_gap = 0;
    step_time_modi = 0;

    dt_gain1 = 5;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
    dt_gain2 = 55;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
    dt_gain3 = 10;  // for Ankle torque control
    //--------------------------------------------------------------------------------------

    // foot step & timing buffer-----------------------------------------------------------
    for(int i=0;i<N_step+2;i++){
        if(i == 0){
            if(R_or_L == 1) // LFoot
                step_data_buf.push_back({_pLF.x, _pLF.y + 0.01, DSP, t_start});
            else
                step_data_buf.push_back({_pRF.x, _pRF.y - 0.01, DSP, t_start});
        }
        else if(i == N_step+1){ // last phase
            step_data_buf.push_back({step_data_buf.back().x, 0, R_or_L, t_end});
            step_data_buf.push_back({step_data_buf.back().x, step_data_buf.back().y, DSP, t_stable});
            step_data_buf.push_back({step_data_buf.back().x, step_data_buf.back().y, DSP, t_stable});
        }
        else if(i == N_step){
            step_data_buf.push_back({step_data_buf.back().x + _step_len_x, R_or_L*_step_len_y/2, R_or_L, t_step});
        }
        else
            step_data_buf.push_back({step_data_buf.back().x + _step_len_x, R_or_L*_step_len_y/2, R_or_L, t_step});
        R_or_L *= -1;
    }
    step_data_buf_original = step_data_buf;
    //-------------------------------------------------------------------------------------

}

DesiredStates HB_DynamicWalk::Walk(RobotStates _ROBOTSTATE){
    // Measurement input
    MeasurementInput(_ROBOTSTATE);

    // COM & foot step & timing planning
    if(step_phase_change_flag == 1){
        t_step = step_data_buf[step_phase].t;
        dT = t_step;
        dT_original = t_step;

        step_time_modi = 0;

        step_phase_change_flag = 0;

        //cout<<"stpe_phase: "<<step_phase<<"R or L"<<step_data_buf[step_phase].R_or_L<<endl;
    }

    if(real_step_phase_change_flag == 1){
        t_foot_now = 0.0;
        real_t_step = t_step;

        step_time_gap = 0;

        if(step_data_buf[real_step_phase].R_or_L == 0) swingFoot = DSP;
        else if(step_data_buf[real_step_phase].R_or_L == -1) swingFoot = RFoot;
        else swingFoot = LFoot;
        real_step_phase_change_flag = 0;

        //cout<<"real step phase: "<<step_phase<<"R or L"<<step_data_buf[step_phase].R_or_L<<endl;
    }

    // Calc CP eos in each iteration---------------------------------------------------------
    // Using next two step, design next CP eos
    double b;
    b = exp(w*step_data_buf[step_phase + 1].t);
    CP_eos.x = 1/b*step_data_buf[step_phase+1].x - (1 - b)/b*step_data_buf[step_phase].x;
    CP_eos.y = 1/b*step_data_buf[step_phase+1].y - (1 - b)/b*step_data_buf[step_phase].y;
    CP_eos.z = 0;
    //----------------------------------------------------------------------------------------


    CP_cps = COM_cps + dCOM_cps/w;  //for CPS pattern
    CP_cpt = COM_cpt + dCOM_cpt/w;  //fpr CPT pattern
    CP_m = COM_m + dCOM_m/w; // measured Capture point

    // CPS approach
    ZMP_cps = 1/(1 - exp(w*dT))*CP_eos - exp(w*dT)/(1 - exp(w*dT))*CP_cps;

    COM_cps = cosh(w*dt)*COM_cps + sinh(w*dt)/w*dCOM_cps + (1-cosh(w*dt))*ZMP_cps;
    dCOM_cps = w*sinh(w*dt)*COM_cps + cosh(w*dt)*dCOM_cps - w*sinh(w*dt)*ZMP_cps;
    COM_cps.z = zc;
    dCOM_cps.z = 0;

    ZMP_cpt = 1/(1 - exp(w*dt*dt_gain1))*CP_cps - exp(w*dt*dt_gain1)/(1 - exp(w*dt*dt_gain1))*CP_cpt;

    COM_cpt = cosh(w*dt)*COM_cpt + sinh(w*dt)/w*dCOM_cpt + (1-cosh(w*dt))*ZMP_cpt;
    dCOM_cpt = w*sinh(w*dt)*COM_cpt + cosh(w*dt)*dCOM_cpt - w*sinh(w*dt)*ZMP_cpt;
    COM_cpt.z = zc;
    dCOM_cpt.z = 0;

    cZMP = 1/(1 - exp(w*dt*dt_gain2))*CP_cpt - exp(w*dt*dt_gain2)/(1 - exp(w*dt*dt_gain2))*CP_m;
    cZMP_proj = zmpProjectionToSP_large(cZMP, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered, -0.01);
    cZMP_proj_MAX = zmpProjectionToSP_large(cZMP, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered,5.0);
    //cout<<"ref com("<<COM_cpt.x<<", "<<COM_cpt.y<<", "<<COM_cpt.z<<")"<<endl;

    double k_gain_y = 0.005;
    double k_gain_x = 0.004;
    double Fz_both = F_RF_filtered.z + F_LF_filtered.z;

    vec3 ZMP_m_local = Calc_local(pRF_old, qRF_old, pLF_old, qLF_old, ZMP_global);
    vec3 cZMP_proj_local = Calc_local(pRF_old, qRF_old, pLF_old, qLF_old, cZMP_proj);

    ddCOM_con.y = k_gain_y*Fz_both/zc*(ZMP_global.y - cZMP_proj.y);
    ddCOM_con.x = k_gain_x*Fz_both/zc*(ZMP_global.x - cZMP_proj.x);
    ddCOM_con.z = 0;


     //Foot trajectory generation*************************************************************************************************************************

     if(swingFoot == RFoot){
         if(F_RF_filtered.z <= 40 && t_foot_now < real_t_step/2.0 && RF_landing_flag == true) RF_landing_flag = false;

         if(F_RF_filtered.z > 40 && t_foot_now > real_t_step/2.0 && RF_landing_flag == false){
             RF_landing_flag = true;

             CP_eos_gap_RF.y = pRF.y - step_data_buf_original[real_step_phase].y;
             CP_eos_gap_RF.x = pRF.x - step_data_buf_original[real_step_phase].x;
         }

         // Original Z pattern
         RF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height, -0.04);

         // Original X pattern
         RF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_x_dx_ddx_old, step_data_buf[real_step_phase].x);

         // Original Y pattern
         if(step_data_buf[real_step_phase+1].R_or_L == 0)
             pFoot_y = step_data_buf[real_step_phase].y - step_len_y/2;
         else
             pFoot_y = step_data_buf[real_step_phase].y;

         RF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_y_dy_ddy_old, pFoot_y);

     }
     else if(swingFoot == LFoot){
         if(F_LF_filtered.z <= 40 && t_foot_now < real_t_step/2.0 && LF_landing_flag == true) LF_landing_flag = false;

         if(F_LF_filtered.z > 40 && t_foot_now > real_t_step/2.0 && LF_landing_flag == false){
             LF_landing_flag = true;

             CP_eos_gap_LF.y = pLF.y - step_data_buf_original[real_step_phase].y;
             CP_eos_gap_LF.x = pLF.x - step_data_buf_original[real_step_phase].x;
         }

         // Original Z pattern
         LF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height, -0.04);

         // Original X pattern
         LF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_x_dx_ddx_old, step_data_buf[real_step_phase].x);

         // Original Y pattern
         if(step_data_buf[real_step_phase+1].R_or_L == 0)
             pFoot_y = step_data_buf[real_step_phase].y + step_len_y/2;
         else
             pFoot_y = step_data_buf[real_step_phase].y;

         LF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_y_dy_ddy_old, pFoot_y);


     }
     else{
         RF_landing_flag = true;
         LF_landing_flag = true;
         RF_z_dz_ddz_old = vec3(pRF.z, 0,0);
         RF_x_dx_ddx_old = vec3(pRF.x, 0,0);
         RF_y_dy_ddy_old = vec3(pRF.y, 0,0);

         LF_z_dz_ddz_old = vec3(pLF.z, 0,0);
         LF_x_dx_ddx_old = vec3(pLF.x, 0,0);
         LF_y_dy_ddy_old = vec3(pLF.y, 0,0);

     }

     pRF.z = RF_z_dz_ddz_pattern[0];
     pLF.z = LF_z_dz_ddz_pattern[0];

     pRF.x = RF_x_dx_ddx_pattern[0];
     pLF.x = LF_x_dx_ddx_pattern[0];

     pRF.y = RF_y_dy_ddy_pattern[0];
     pLF.y = LF_y_dy_ddy_pattern[0];

     RF_z_dz_ddz_old = RF_z_dz_ddz_pattern;
     RF_x_dx_ddx_old = RF_x_dx_ddx_pattern;
     RF_y_dy_ddy_old = RF_y_dy_ddy_pattern;

     LF_z_dz_ddz_old = LF_z_dz_ddz_pattern;
     LF_x_dx_ddx_old = LF_x_dx_ddx_pattern;
     LF_y_dy_ddy_old = LF_y_dy_ddy_pattern;


     // Foot placement control----------------------------------------------------------------------------------
     if(real_step_phase > 0 && real_step_phase < N_step){
         ZMP_ref_gap =  cZMP_proj - cZMP_proj_MAX; ZMP_ref_gap.z = 0;


         // Step timing adjustment--------------------------------------------------------------
//         double dstep_time_gap = 0;

//         double min_step_time = 0.2;


//         if(ZMP_ref_gap.x > 0.003 || ZMP_ref_gap.x < -0.003 || ZMP_ref_gap.y > 0.003 || ZMP_ref_gap.y < -0.003){
//             vec3 temp = CP_m - cZMP_proj; temp.z = 0;
//             dstep_time_gap = -2*(1 - exp(w*dT))/(w*exp(w*dT))*dot(temp,ZMP_ref_gap)/dot(temp);
//             //cout<<"Error step_time_gap: "<<step_time_gap<<endl;
//         }
//         else{
//             dstep_time_gap = -step_time_gap/20/dt;
//             //cout<<"Return step_time_gap: "<<step_time_gap<<endl;
//         }

//         step_time_gap += dstep_time_gap*dt;

//         step_time_modi = StepTimeLimiter(step_time_gap, step_time_modi_old, dT, min_step_time);


//         dT = dT_original + step_time_modi;
//         real_t_step = t_step + step_time_modi;
//         step_data_buf[step_phase].t = step_data_buf_original[step_phase].t + step_time_modi;

//         step_time_modi_old = step_time_modi;



          //Step Position Adjustment--------------------------------------------------------------
         if(step_data_buf[step_phase].R_or_L == LFoot && LF_landing_flag == false){
             vec3 dCP_eos = vec3();
             if((ZMP_ref_gap.y > 0.03 || ZMP_ref_gap.y < -0.03) && (step_phase == real_step_phase)){
                 dCP_eos.y = 1*(1 - exp(w*dT))*ZMP_ref_gap.y;
                 //cout<<"dCP_eos.y = "<<dCP_eos.y<<endl;
             }
             else if((ZMP_ref_gap.y < 0.01 || ZMP_ref_gap.y > -0.01)){
                 dCP_eos.y = -CP_eos_gap_LF.y/(5.0)/dt;
             }

             if((ZMP_ref_gap.x > 0.005 || ZMP_ref_gap.x < -0.005) && (step_phase == real_step_phase)){
                 dCP_eos.x = (1 - exp(w*dT))*ZMP_ref_gap.x;
             }
             else if((ZMP_ref_gap.x < 0.005 || ZMP_ref_gap.x > -0.005)){
                 dCP_eos.x = -CP_eos_gap_LF.x/(5.0)/dt;
             }

             dCP_eos.z = 0;

             CP_eos_gap_LF = CP_eos_gap_LF + dCP_eos*dt;

     //        CP_eos_modi = CP_eos_Limitor(CP_eos_gap, step_data_buf_original[step_phase].R_or_L,
     //                                     step_data_buf_original[step_phase].x, step_data_buf_original[step_phase].y, pRF_old, pLF_old, dT);
             CP_eos_modi_LF = CP_eos_gap_LF;

             cout<<"CP_eos_modi_LF: "<<CP_eos_modi_LF.y<<endl;


         }
         if(step_data_buf[step_phase].R_or_L == RFoot && RF_landing_flag == false){
             vec3 dCP_eos = vec3();
             if((ZMP_ref_gap.y > 0.03 || ZMP_ref_gap.y < -0.03) && (step_phase == real_step_phase)){
                 dCP_eos.y = 1*(1 - exp(w*dT))*ZMP_ref_gap.y;
                 //cout<<"dCP_eos.y = "<<dCP_eos.y<<endl;
             }
             else if((ZMP_ref_gap.y < 0.01 || ZMP_ref_gap.y > -0.01)){
                 dCP_eos.y = -CP_eos_gap_RF.y/(5.0)/dt;
             }

             if((ZMP_ref_gap.x > 0.005 || ZMP_ref_gap.x < -0.005) && (step_phase == real_step_phase)){
                 dCP_eos.x = (1 - exp(w*dT))*ZMP_ref_gap.x;
             }
             else if((ZMP_ref_gap.x < 0.005 || ZMP_ref_gap.x > -0.005)){
                 dCP_eos.x = -CP_eos_gap_RF.x/(5.0)/dt;
             }

             dCP_eos.z = 0;

             CP_eos_gap_RF = CP_eos_gap_RF + dCP_eos*dt;

     //        CP_eos_modi = CP_eos_Limitor(CP_eos_gap, step_data_buf_original[step_phase].R_or_L,
     //                                     step_data_buf_original[step_phase].x, step_data_buf_original[step_phase].y, pRF_old, pLF_old, dT);
             CP_eos_modi_RF = CP_eos_gap_RF;

             cout<<"CP_eos_modi_RF: "<<CP_eos_modi_RF.y<<endl;

         }

         if(step_data_buf[step_phase].R_or_L == RFoot){
             step_data_buf[step_phase].y = step_data_buf_original[step_phase].y + CP_eos_modi_RF.y;
             step_data_buf[step_phase].x = step_data_buf_original[step_phase].x + CP_eos_modi_RF.x;

         }
         else if(step_data_buf[step_phase].R_or_L == LFoot){
             step_data_buf[step_phase].y = step_data_buf_original[step_phase].y + CP_eos_modi_LF.y;
             step_data_buf[step_phase].x = step_data_buf_original[step_phase].x + CP_eos_modi_LF.x;

         }
         else if(step_data_buf[step_phase].R_or_L == LFoot){ // I dont know ;;;
             step_data_buf[step_phase].y = step_data_buf_original[step_phase].y;
             step_data_buf[step_phase].x = step_data_buf_original[step_phase].x;
         }

         CP_eos_modi_RF_old = CP_eos_modi_RF;
         CP_eos_modi_LF_old = CP_eos_modi_LF;



     }


     ////=================================== Calc QP ========================================================
     /// Setup REF
     /// IK
     LegJoints LJ = Hi.IK_COM(COM_cpt, quat(), pRF, qRF, pLF, qLF);

     /// Ref contact
     double foot_up_ratio = 0.45;
     double t_up  = real_t_step*foot_up_ratio*(1.0 - dsp_ratio);
     double t_half_dsp = real_t_step*dsp_ratio/2.0;
     double t_down = real_t_step - t_up - 2*t_half_dsp;

     if(swingFoot == RFoot){
         if(t_foot_now <  t_half_dsp - 0.5*dt){
             REF.cRF = true; REF.RFdn = true; REF.RFup = false;
         }
         else if(t_foot_now < t_up + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up
             REF.cRF = false; REF.RFdn = false; REF.RFup = true;
         }
         else if(t_foot_now < t_up + t_half_dsp + t_down - 0.5*dt){                  // foot down, 5th order trajectory down
             REF.cRF = false; REF.RFdn = true; REF.RFup = false;
         }
         else{
             REF.cRF = true; REF.RFdn = true; REF.RFup = false;
         }
     }
     if(swingFoot == LFoot){
         if(t_foot_now <  t_half_dsp - 0.5*dt){
             REF.cLF = true; REF.LFdn = true; REF.LFup = false;
         }
         else if(t_foot_now < t_up + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up
             REF.cLF = false; REF.LFdn = false; REF.LFup = true;
         }
         else if(t_foot_now < t_up + t_half_dsp + t_down - 0.5*dt){                  // foot down, 5th order trajectory down
             REF.cLF = false; REF.LFdn = true; REF.LFup = false;
         }
         else{
             REF.cLF = true; REF.LFdn = true; REF.LFup = false;
         }
     }


     /// CSP and JSP
     // COM & PEL
     REF.CSP.pCOM = COM_cpt;
     REF.CSP.pPel = LJ.pPel;
     REF.JSP.pPel = LJ.pPel;

     REF.CSP.qPel = quat();
     REF.JSP.qPel = quat();

     //RF, LF pos & ori
     REF.CSP.pRF = vec3(RF_x_dx_ddx_pattern[0], RF_y_dy_ddy_pattern[0], RF_z_dz_ddz_pattern[0]);
     REF.CSP.qRF = quat();
     REF.CSP.pLF = vec3(LF_x_dx_ddx_pattern[0], LF_y_dy_ddy_pattern[0], LF_z_dz_ddz_pattern[0]);
     REF.CSP.qLF = quat();

     /// CSV
     // dCOM & dPEL
     REF.CSV.dpCOM = dCOM_cpt;
     REF.CSV.dpPel = dCOM_cpt; //dpPel ref same as dpCOM ref
     REF.CSV.dqPel = vec3();

     REF.JSV.dpPel = dCOM_cpt;
     REF.JSV.dqPel = vec3();

     //RF, LF pos & ori
     REF.CSV.dpRF = vec3(RF_x_dx_ddx_pattern[1], RF_y_dy_ddy_pattern[1], RF_z_dz_ddz_pattern[1]);
     REF.CSV.dqRF = vec3();
     REF.CSV.dpLF = vec3(LF_x_dx_ddx_pattern[1], LF_y_dy_ddy_pattern[1], LF_z_dz_ddz_pattern[1]);
     REF.CSV.dqLF = vec3();

     /// CSA
     // dCOM & dPEL
     REF.CSA.ddpCOM = ddCOM_con;
     REF.CSA.ddpPel = vec3();
     REF.CSA.ddqPel = vec3();
     REF.JSA.ddpPel = vec3();
     REF.JSA.ddqPel = vec3();

     REF.CSA.ddpRF = vec3(RF_x_dx_ddx_pattern[2], RF_y_dy_ddy_pattern[2], RF_z_dz_ddz_pattern[2]);
     REF.CSA.ddqRF = vec3();
     REF.CSA.ddpLF = vec3(LF_x_dx_ddx_pattern[2], LF_y_dy_ddy_pattern[2], LF_z_dz_ddz_pattern[2]);
     REF.CSA.ddpLF = vec3();

     //make Qref, dQref, ddQref
     for(int i=0; i<12 ; i++){
         REF.JSP.JSP_Array[i] = LJ.LJ_Array[i];
     }

//     for(int i=0; i<12 ; i++){
//         REF.JSV.JSV_Array[i] = 0.0;
//     }
//     for(int i=0; i<12 ; i++){
//         REF.JSA.JSA_Array[i] = 0.0;
//     }

     REF.Qref = REF.getQref(bp_rbdl.Robot->q_size);
     REF.dQref = dIK_Solver_COM(REF);
     REF.ddQref = ddIK_Solver_COM(REF);

     //cout<<"ref com("<<REF.CSP.pCOM.x<<", "<<REF.CSP.pCOM.y<<", "<<REF.CSP.pCOM.z<<")"<<endl;

     //calc QP
     DS.Xdes = bp_rbdl.calc_QP(ROBOTSTATE, REF);
     DS.ddQdes = DS.Xdes.segment(0,ROBOTSTATE.dQnow.size());
     DS.Tdes = DS.Xdes.segment(ROBOTSTATE.dQnow.size(),12);
     DS.Fdes = DS.Xdes.segment(30,DS.Xdes.size()-30);
     ////==================================================================================================================================


     // when robot falls, all torque goes to 0
//     rpy qPel_rpy(qPel_m);
//     if(ABS(qPel_rpy.r*R2D) > 30.0 || ABS(qPel_rpy.p*R2D) > 30.0){
//         cout<<"robot fall down"<<endl;
//         walk_done_flag = true;
//     }


     // for iterative run
     if(dT > dT_min){
         dT -= dt;
         dT_original -= dt;
     }
     else{
         dT = dT_min;
         dT_original = dT_min;
     }

     k++;
     t_total += dt;
     t_foot_now += dt;

     save_onestep(k);

     if(t_total - t_elapsed >= real_t_step){
         step_phase_change_flag = 1;
         real_time_adjust_counter_start_flag = 1;
         step_phase++;
         t_elapsed += real_t_step;
         if(step_phase > N_step + 2) walk_done_flag = true;

     }

     if(real_time_adjust_counter_start_flag == 1){
         real_time_adjust_count++;
         if(real_time_adjust_count >= dt_gain1){
             real_step_phase++;
             real_time_adjust_counter_start_flag = 0;
             real_time_adjust_count = 0;
             real_step_phase_change_flag = 1;   // this flag changes dt_gain1*dt lator than "step_phase_change_flag"
         }
     }

     return DS;

}

void HB_DynamicWalk::MeasurementInput(RobotStates _RST){
    ROBOTSTATE = _RST;

    F_RF=ROBOTSTATE.F_RF; F_LF=ROBOTSTATE.F_LF; M_RF=ROBOTSTATE.M_RF; M_LF=ROBOTSTATE.M_LF; pPel_old = ROBOTSTATE.CSP.pPel;
    IMUangle=ROBOTSTATE.IMUangle; IMUomega=ROBOTSTATE.IMUomega;
    pRF_old=ROBOTSTATE.CSP.pRF; pLF_old=ROBOTSTATE.CSP.pLF; qRF_old=ROBOTSTATE.CSP.qRF; qLF_old=ROBOTSTATE.CSP.qLF;

    dCOM_old = ROBOTSTATE.CSV.dpCOM;
    COM_old = ROBOTSTATE.CSP.pCOM;  qPel_old=ROBOTSTATE.CSP.qPel;

    // COM estimation
    COM_est = ROBOTSTATE.CSP.pCOM;
    dCOM_est = ROBOTSTATE.CSV.dpCOM;

    // ZMP calculation
    ZMP_global = ZMP_calc_global(pRF_old, qRF_old, F_RF, M_RF, pLF_old, qLF_old, F_LF, M_LF);

    // ZMP Filtering
    double alpha = 1/(1 + 2*PI*dt*25);
    ZMP_global_filtered = alpha*ZMP_global_filtered + (1 - alpha)*ZMP_global;
    dZMP_global_filtered = (ZMP_global_filtered - ZMP_global_filtered_old)/dt;
    ZMP_global_filtered_old = ZMP_global_filtered;

    //FT Sensor Filtering
    double alpha_FT = 1/(1 + 2*PI*dt*60);
    F_RF_filtered = F_RF_filtered*alpha_FT + F_RF*(1 - alpha_FT);
    F_LF_filtered = F_LF_filtered*alpha_FT + F_LF*(1 - alpha_FT);
    M_RF_filtered = M_RF_filtered*alpha_FT + M_RF*(1 - alpha_FT);
    M_LF_filtered = M_LF_filtered*alpha_FT + M_LF*(1 - alpha_FT);

    // Measured Global COM
    //COM_m = COM_measurment(_COM_kine, ZMP_global, IMUangle, _qPel_kine);
    COM_m = ROBOTSTATE.CSP.pCOM;
    dCOM_m = ROBOTSTATE.CSV.dpCOM;

    // Measured Global qPel
    qPel_m = ROBOTSTATE.CSP.qPel;


    double alpha1 = 1/(1 + 2*PI*dt*8);  // for ankle torque generation
    COM_m_filtered = alpha1*COM_m_filtered + (1 - alpha1)*COM_m;
    dCOM_m_filtered = alpha1*dCOM_m_filtered + (1 - alpha1)*dCOM_m;

    double alpha2 = 1/(1 + 2*PI*dt*3.0);  // for ddCOM_con generation
    COM_m_filtered2 = alpha2*COM_m_filtered2 + (1 - alpha2)*COM_m;
    dCOM_m_filtered2 = alpha2*dCOM_m_filtered2 + (1 - alpha2)*dCOM_m;

}

VectorNd HB_DynamicWalk::dIK_Solver_COM(REFERENCE _REF){

    VectorNd dXref = VectorNd::Zero(bp_rbdl.Robot->dof_count);

    dXref[0] =  _REF.CSV.dpCOM.x;
    dXref[1] =  _REF.CSV.dpCOM.y;
    dXref[2] =  _REF.CSV.dpCOM.z;
    dXref[3] =  _REF.CSV.dqPel.x;
    dXref[4] =  _REF.CSV.dqPel.y;
    dXref[5] =  _REF.CSV.dqPel.z;
    dXref[6] =  _REF.CSV.dpRF.x;
    dXref[7] =  _REF.CSV.dpRF.y;
    dXref[8] =  _REF.CSV.dpRF.z;
    dXref[9] =  _REF.CSV.dqRF.x;
    dXref[10] = _REF.CSV.dqRF.y;
    dXref[11] = _REF.CSV.dqRF.z;
    dXref[12] = _REF.CSV.dpLF.x;
    dXref[13] = _REF.CSV.dpLF.y;
    dXref[14] = _REF.CSV.dpLF.z;
    dXref[15] = _REF.CSV.dqLF.x;
    dXref[16] = _REF.CSV.dqLF.y;
    dXref[17] = _REF.CSV.dqLF.z;

    bp_rbdl.CalcEndeffectorJacobian6D(_REF.Qref); //JacobianRF3D_pos & ori  JacobianLF3D_pos & ori
    bp_rbdl.CalcCOMJacobian3D(_REF.Qref); // Calc JacobianCOMall3D

    bp_rbdl.JQref = MatrixNd::Zero(bp_rbdl.Robot->dof_count,bp_rbdl.Robot->dof_count);
    bp_rbdl.JQref.block(0,0,3,bp_rbdl.Robot->dof_count) = bp_rbdl.JacobianCOMall3D;
    bp_rbdl.JQref.block(3,3,3,3) = MatrixNd::Identity(3,3);
    bp_rbdl.JQref.block(6,0,3,bp_rbdl.Robot->dof_count) = bp_rbdl.JacobianRF3D_pos;
    bp_rbdl.JQref.block(9,0,3,bp_rbdl.Robot->dof_count) = bp_rbdl.JacobianRF3D_ori;
    bp_rbdl.JQref.block(12,0,3,bp_rbdl.Robot->dof_count) = bp_rbdl.JacobianLF3D_pos;
    bp_rbdl.JQref.block(15,0,3,bp_rbdl.Robot->dof_count) = bp_rbdl.JacobianLF3D_ori;

    bp_rbdl.JQref_inv = bp_rbdl.JQref.inverse();

    return bp_rbdl.JQref_inv*dXref;
}

VectorNd HB_DynamicWalk::ddIK_Solver_COM(REFERENCE _REF){
    ////dIK_Solver_COM() should be done earlier
    //bp_rbdl.JQref & bp_rbdl.JQref_inv calculation is in the 'dIK_Solver_COM'

    //dX = J*dQ
    //ddX = dJ*dQ + J*ddQ
    //J*ddQ = ddX - dJ*dQ
    //ddQ = J_inv*(ddX - dJ*dQ)

    VectorNd ddXref = VectorNd::Zero(bp_rbdl.Robot->dof_count);

    ddXref[0] = _REF.CSA.ddpCOM.x;
    ddXref[1] = _REF.CSA.ddpCOM.y;
    ddXref[2] = _REF.CSA.ddpCOM.z;
    ddXref[3] = _REF.CSA.ddqPel.x;
    ddXref[4] = _REF.CSA.ddqPel.y;
    ddXref[5] = _REF.CSA.ddqPel.z;
    ddXref[6] = _REF.CSA.ddpRF.x;
    ddXref[7] = _REF.CSA.ddpRF.y;
    ddXref[8] = _REF.CSA.ddpRF.z;
    ddXref[9] =  _REF.CSA.ddqRF.x;
    ddXref[10] = _REF.CSA.ddqRF.y;
    ddXref[11] = _REF.CSA.ddqRF.z;
    ddXref[12] = _REF.CSA.ddpLF.x;
    ddXref[13] = _REF.CSA.ddpLF.y;
    ddXref[14] = _REF.CSA.ddpLF.z;
    ddXref[15] = _REF.CSA.ddqLF.x;
    ddXref[16] = _REF.CSA.ddqLF.y;
    ddXref[17] = _REF.CSA.ddqLF.z;

    VectorNd ddqZero = VectorNd::Zero(bp_rbdl.Robot->qdot_size);
    Vector3d dJdQpCOM = (Hi.m_pel * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_pel,bp_rbdl.v2V(Hi.c_pel))
                         + Hi.m_rhy * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rhy,bp_rbdl.v2V(Hi.c_rhy))
                         + Hi.m_rhr * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rhr,bp_rbdl.v2V(Hi.c_rhr))
                         + Hi.m_rhp * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rhp,bp_rbdl.v2V(Hi.c_rhp))
                         + Hi.m_rkn * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rkn,bp_rbdl.v2V(Hi.c_rkn))
                         + Hi.m_rap * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rap,bp_rbdl.v2V(Hi.c_rap))
                         + Hi.m_rar * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rar,bp_rbdl.v2V(Hi.c_rar))
                         + Hi.m_lhy * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lhy,bp_rbdl.v2V(Hi.c_lhy))
                         + Hi.m_lhr * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lhr,bp_rbdl.v2V(Hi.c_lhr))
                         + Hi.m_lhp * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lhp,bp_rbdl.v2V(Hi.c_lhp))
                         + Hi.m_lkn * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lkn,bp_rbdl.v2V(Hi.c_lkn))
                         + Hi.m_lap * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lap,bp_rbdl.v2V(Hi.c_lap))
                         + Hi.m_lar * CalcPointAcceleration(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lar,bp_rbdl.v2V(Hi.c_lar)))/Hi.m_total;

    Vector3d dJdQqPel = Vector3d::Zero(3); //dJ/dt = 0
    VectorNd tempRF = CalcPointAcceleration6D(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_rar,bp_rbdl.v2V(Hi.offset_foot));
    VectorNd tempLF = CalcPointAcceleration6D(*(bp_rbdl.Robot),_REF.Qref,_REF.dQref,ddqZero,bp_rbdl.n_lar,bp_rbdl.v2V(Hi.offset_foot));

    Vector3d dJdqRF = tempRF.head(3);
    Vector3d dJdqLF = tempLF.head(3);

    Vector3d dJdpRF = tempRF.tail(3);
    Vector3d dJdpLF = tempLF.tail(3);

    VectorNd dJdq = VectorNd::Zero(18);
    dJdq.segment(0,3) = dJdQpCOM;
    dJdq.segment(3,3) = dJdQqPel;
    dJdq.segment(6,3) = dJdpRF;
    dJdq.segment(9,3) = dJdqRF;
    dJdq.segment(12,3) = dJdpLF;
    dJdq.segment(15,3) = dJdqLF;

    return bp_rbdl.JQref_inv*(ddXref - dJdq);
}

void HB_DynamicWalk::save_onestep(int cnt)
{
    if(k<200000)
    {
        SAVE[0][cnt] = COM.x;
        SAVE[1][cnt] = CP.x;
        SAVE[2][cnt] = CP_cps.x;
        SAVE[3][cnt] = 0;
        SAVE[4][cnt] = CP_m_filtered.x;
        SAVE[5][cnt] = COM.y;
        SAVE[6][cnt] = CP.y;
        SAVE[7][cnt] = CP_cps.y;
        SAVE[8][cnt] = 0;
        SAVE[9][cnt] = CP_m_filtered.y;

        SAVE[10][cnt] = COM_cps.x;
        SAVE[11][cnt] = COM_cps.y;
        SAVE[12][cnt] = ZMP_cps.x;
        SAVE[13][cnt] = ZMP_cps.y;
        SAVE[14][cnt] = ZMP_global.x;
        SAVE[15][cnt] = ZMP_global.y;
        SAVE[16][cnt] = CP_eos.x;
        SAVE[17][cnt] = CP_eos.y;

        SAVE[18][cnt] = IMUangle.x;
        SAVE[19][cnt] = IMUangle.y;
        SAVE[20][cnt] = IMUangle.z;

        SAVE[21][cnt] = COM_cpt.x;
        SAVE[22][cnt] = COM_cpt.y;
        SAVE[23][cnt] = ZMP_cpt.x;
        SAVE[24][cnt] = ZMP_cpt.y;
        SAVE[25][cnt] = CP_cpt.x;
        SAVE[26][cnt] = CP_cpt.y;

        SAVE[27][cnt] = dT;//dT_est;//dT_buf[0];
        SAVE[28][cnt] = step_time_gap;
        SAVE[29][cnt] = step_time_modi;

        SAVE[30][cnt] = pRF.z;
        SAVE[31][cnt] = pLF.z;
        SAVE[32][cnt] = 0;

        SAVE[33][cnt] = dCOM_m_filtered.y; // COM_measure x
        SAVE[34][cnt] = COM_m.y; // COM_measure y

        SAVE[35][cnt] = CP_eos_gap_LF.y;
        SAVE[36][cnt] = CP_eos_gap_RF.y;

        SAVE[37][cnt] = CP_eos_modi_RF.y; // temp x
        SAVE[38][cnt] = CP_eos_modi_LF.y; // temp y

        SAVE[39][cnt] = 0;//COM_est.x + dCOM_est.x/w; // CP estimation x
        SAVE[40][cnt] = 0;// + dCOM_est.y/w; // CP estimation y

        SAVE[41][cnt] = COM_est.x; // COM x estimation
        SAVE[42][cnt] = COM_est.y; // COM y estimation
        SAVE[43][cnt] = dCOM_est.x; // dCOM x estimation
        SAVE[44][cnt] = dCOM_est.y; // dCOM y estimation

        SAVE[45][cnt] = CP_m.y; // ZMP x estimation
        SAVE[46][cnt] = ZMP_global_filtered.x;
        SAVE[47][cnt] = ZMP_global_filtered.y;
        SAVE[48][cnt] = pRF.x;
        SAVE[49][cnt] = pRF.y;

        SAVE[50][cnt] = F_RF_filtered.z;
        SAVE[51][cnt] = F_LF_filtered.z;
        SAVE[52][cnt] = pFoot_y;
        SAVE[53][cnt] = dCOM.y;
        SAVE[54][cnt] = pLF.y;

        SAVE[55][cnt] = COM_old.x;
        SAVE[56][cnt] = COM_old.y;
        SAVE[57][cnt] = ZMP_est.y;

        SAVE[58][cnt] = COM_choreonoid.x;
        SAVE[59][cnt] = COM_choreonoid.y;
        SAVE[60][cnt] = dCOM_choreonoid.x;
        SAVE[61][cnt] = dCOM_choreonoid.y;

        SAVE[62][cnt] = ZMP_ref_gap.y;
        SAVE[63][cnt] = CP_eos_gap.y;
        SAVE[64][cnt] = CP_eos_modi.y;

        SAVE[65][cnt] = (double)real_step_phase_change_flag;
        SAVE[66][cnt] = (double)step_phase_change_flag;
        SAVE[67][cnt] = step_data_buf[real_step_phase].y;



        //SAVE[47][cnt] = realCOM.z;//
    }
}

void HB_DynamicWalk::save_all()
{
    printf("walk finished %d\n",k);
    FILE* ffp = fopen("/home/rainbow/Desktop/HBtest_Walking_Data.txt","w");
    for(int i=0;i<k;i++)
    {
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
                ,SAVE[0][i]
                ,SAVE[1][i]
                ,SAVE[2][i]
                ,SAVE[3][i]
                ,SAVE[4][i]
                ,SAVE[5][i]
                ,SAVE[6][i]
                ,SAVE[7][i]
                ,SAVE[8][i]
                ,SAVE[9][i]
                );
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
                ,SAVE[10][i]
                ,SAVE[11][i]
                ,SAVE[12][i]
                ,SAVE[13][i]
                ,SAVE[14][i]
                ,SAVE[15][i]
                ,SAVE[16][i]
                ,SAVE[17][i]
                ,SAVE[18][i]
                ,SAVE[19][i]
                );
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
                ,SAVE[20][i]
                ,SAVE[21][i]
                ,SAVE[22][i]
                ,SAVE[23][i]
                ,SAVE[24][i]
                ,SAVE[25][i]
                ,SAVE[26][i]
                ,SAVE[27][i]
                ,SAVE[28][i]
                ,SAVE[29][i]
                );
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
                ,SAVE[30][i]
                ,SAVE[31][i]
                ,SAVE[32][i]
                ,SAVE[33][i]
                ,SAVE[34][i]
                ,SAVE[35][i]
                ,SAVE[36][i]
                ,SAVE[37][i]
                ,SAVE[38][i]
                ,SAVE[39][i]
                );
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
                ,SAVE[40][i]
                ,SAVE[41][i]
                ,SAVE[42][i]
                ,SAVE[43][i]
                ,SAVE[44][i]
                ,SAVE[45][i]
                ,SAVE[46][i]
                ,SAVE[47][i]
                ,SAVE[48][i]
                ,SAVE[49][i]
                ,SAVE[50][i]
                ,SAVE[51][i]
                );
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"
                ,SAVE[52][i]
                ,SAVE[53][i]
                ,SAVE[54][i]
                ,SAVE[55][i]
                ,SAVE[56][i]
                ,SAVE[57][i]
                ,SAVE[58][i]
                ,SAVE[59][i]
                ,SAVE[60][i]
                ,SAVE[61][i]
                ,SAVE[62][i]
                ,SAVE[63][i]
                ,SAVE[64][i]
                ,SAVE[65][i]
                ,SAVE[66][i]
                ,SAVE[67][i]
                );
    }
    fclose(ffp);
}


#endif // HB_DYNAMICS_WALK

