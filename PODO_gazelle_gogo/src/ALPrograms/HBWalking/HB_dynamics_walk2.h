#ifndef HB_DYNAMIC_WALK2
#define HB_DYNAMIC_WALK2

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

class HB_DynamicWalk2
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
    double dT_min, t_now, lp, Tmin, Tmax, Lmin, Lmax, Wmin, Wmax, Tnom, Lnom, Wnom, bxnom, bynom;
    double Lnom_global, Wnom_global;
    double t_step, dT, t_total, t_elapsed;
    vec3 CP_eos, desFoot, pRF_des, pLF_des, pRF_m, pLF_m, pPel_m;
    quat qRF_des, qRF_m, qLF_des, qLF_m, qPel_m;
    
    int  R_or_L, k, real_time_adjust_count;
    char step_phase_change_flag, real_step_phase_change_flag, real_time_adjust_counter_start_flag;
    unsigned int step_phase,real_step_phase, N_step;
    double init_foot_width;
    char swingFoot;
    vec3 COM_est, dCOM_est, ddCOM_est, ZMP_est, COM_m, COM_m_old, CP_m_filtered, COM_m_filtered,dCOM_m_filtered, dCOM_m, ddCOM_m, COM_old, dCOM_old, COM, dCOM, ddCOM, ddCOM_con, dCOM_con, COM_con;
    vec3 COM_choreonoid, dCOM_choreonoid, dCOM_choreonoid_filtered, ddCOM_con_filetered;
    vec3 CP, CP_m, COM_p, dCOM_p, CP_p, ZMP_p, ZMP_ref, ZMP_ref_filtered;
    vec3 pPel_old, pRF, pRF_old, pLF, pLF_old, F_RF, F_LF, M_RF, M_LF, IMUangle, IMUomega, ZMP_global, ZMP_global_filtered,ZMP_global_filtered_old,dZMP_global_filtered, ZMP_local;
    vec3 ZMP_ref_gap, total_CP_eos_gap, CP_eos_gap, CP_eos_gap_old;
    vec3 CP_eos_gap_RF, CP_eos_gap_LF, CP_eos_modi_RF, CP_eos_modi_LF, CP_eos_modi_RF_old, CP_eos_modi_LF_old;
    vec3 F_RF_filtered, F_LF_filtered, M_RF_filtered, M_LF_filtered;
    quat qRF, qRF_old, qLF, qLF_old, qPel, qPel_old;
    vec3 cZMP, cZMP_proj, CP_eos_modi;
    double dsp_ratio,FootUp_height;
    vec3 RF_z_dz_ddz_old, LF_z_dz_ddz_old, RF_z_dz_ddz_pattern, LF_z_dz_ddz_pattern, RF_x_dx_ddx_old, LF_x_dx_ddx_old, RF_y_dy_ddy_old, LF_y_dy_ddy_old;
    vec3 RF_x_dx_ddx_pattern, LF_x_dx_ddx_pattern, RF_y_dy_ddy_pattern, LF_y_dy_ddy_pattern;
    int dt_gain1;
    bool walk_done_flag, RF_landing_flag, LF_landing_flag;
    double SAVE[80][200000];

    // Landing Control
    char support_phase;


    //Functions
    DesiredStates InvDyn_Control(RobotStates _ROBOSTATE, REFERENCE _REF);
    DesiredStates StandUp(RobotStates _ROBOTSTATE, RobotStates RS_ini);
    void set_step(vec3 _COM_ini, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, double _lp, int _RL_first);
    DesiredStates Walk(RobotStates _ROBOTSTATE, double _Vx, double _Vy, double _W);
    void MeasurementInput(RobotStates _RST);
    VectorNd dIK_Solver_COM(REFERENCE _REF);
    VectorNd ddIK_Solver_COM(REFERENCE _REF);
    VectorNd StepOptimizer(int stanceLeg_index);
    void save_onestep(int cnt);
    void save_all();


    HB_DynamicWalk2(){
        DS.ddQdes = VectorNd::Zero(bp_rbdl.Robot->dof_count);
        DS.Tdes = VectorNd::Zero(bp_rbdl.Robot->dof_count-6);
        DS.Fdes = VectorNd::Zero(6*2);
        DS.Xdes = VectorNd::Zero(bp_rbdl.Robot->dof_count + bp_rbdl.Robot->dof_count-6 + 6*2);

        DS_stop.ddQdes = VectorNd::Zero(bp_rbdl.Robot->dof_count);
        DS_stop.Tdes = VectorNd::Zero(bp_rbdl.Robot->dof_count-6);
        DS_stop.Fdes = VectorNd::Zero(6*2);
        DS_stop.Xdes = VectorNd::Zero(bp_rbdl.Robot->dof_count + bp_rbdl.Robot->dof_count-6 + 6*2);

        //Walking variables
        FootUp_height = 0.09;

        zc = 0.70;
        w = sqrt(9.81/zc);
        step_phase = 0;
        COM_m_old = vec3(0,0,0);

        COM_old = vec3(0,0,0);
        dCOM_old = COM_old;

        ZMP_global_filtered = vec3(0,0,0);
        ZMP_global_filtered_old = vec3(0,0,0);

        k = 0;

        step_phase_change_flag = 1;

        dCOM_con = vec3(0,0,0);
        ddCOM_con = vec3(0,0,0);
        walk_done_flag = false;
    }

};

void HB_DynamicWalk2::set_step(vec3 _COM_ini, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, double _lp, int _RL_first){
    // Initialize variable-----------------------------------------------------------------
    R_or_L = _RL_first;
    k = 0;

    lp = _lp;
    t_now = 0;
    t_step = 0.7;
    dT = t_step;
    dT_min = 15*dt;//t_step*0.02;
    t_elapsed = 0;
    t_total = 0;

    if(R_or_L == -1){
        swingFoot = LFoot;
        desFoot = _pLF;
    }
    if(R_or_L == 1){
        swingFoot = RFoot;
        desFoot = _pRF;
    }

    step_phase = 0;
    step_phase_change_flag = 1;

    cout<<"COM_initial value : ("<<_COM_ini.x<<", "<<_COM_ini.y<<")"<<endl;

    COM = _COM_ini;
    dCOM = vec3(0,0,0);
    ddCOM = vec3(0,0,0);
    COM_old = _COM_ini;
    dCOM_old = vec3(0,0,0);
    COM_m_old = _COM_ini;
    COM_m_filtered = _COM_ini;
    ddCOM_con_filetered = vec3(0,0,0);

    ZMP_global_filtered = vec3(0,0,0);
    ZMP_global_filtered_old = vec3(0,0,0);

    dsp_ratio = 0.05;

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


    RF_landing_flag = true;
    LF_landing_flag = true;


    dt_gain1 = 5;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)

}

DesiredStates HB_DynamicWalk2::Walk(RobotStates _ROBOTSTATE, double _Vx, double _Vy, double _W){
    // _Vx : desired walking speed (sagital)
    // _Vy : desired walking speed (lateral)
    // _W  : desired rotate speed (Yaw)

    MeasurementInput(_ROBOTSTATE);

    int n;
    if(step_phase_change_flag == 1){
        cout<<"step phase changed"<<endl;
        cout<<"swingFoot: "<<(int)swingFoot<<endl;
        // switch swingFoot
        if(swingFoot == LFoot) swingFoot = RFoot;
        if(swingFoot == RFoot) swingFoot = LFoot;

        //// ----------------- boundary value----------------------------------------

        Tmin = 0.5;
        Tmax = 1.5;
        Lmin = -0.5;
        Lmax = 0.5;
        if(swingFoot == LFoot){ // right foot stance
            // pRFy + 0.1 < pLFy = pRFy + lp + W < pRFy + 0.8;
            Wmin = 0.1 - lp;
            Wmax = 0.5 - lp;
            n = 1;
        }
        else if(swingFoot == RFoot){ // left foot stance
            // pLFy - 0.8 < pRFy = pLFy - (lp - W) < pLFy - 0.1
            Wmin = lp - 0.5;
            Wmax = lp - 0.1;
            n = 2;
        }

        ////------------------------determine nominal value---------------------------------------------
        double B1_l, B1_u, B2_l, B2_u, B3_l, B3_u, Bl, Bu;
        if(_Vx == 0 || _Vy == 0){
            if(_Vx == 0){
                B1_l = -100;  B1_u = 100;
            }
            if(_Vy == 0){
                B2_l = -100;  B2_u = 100;
            }
        }
        else{
            B1_l = Lmin/fabs(_Vx);    B1_u = Lmax/fabs(_Vx);
            B2_l = Wmin/fabs(_Vy);    B2_u = Wmax/fabs(_Vy);
        }
        B3_l = Tmin;        B3_u = Tmax;





        //find max B_l
        Bl = B1_l;
        if(Bl < B2_l) Bl = B2_l;
        if(Bl < B3_l) Bl = B3_l;

        //find min B_u
        Bu = B1_u;
        if(Bu > B2_u) Bu = B2_u;
        if(Bu > B3_u) Bu = B3_u;

        Tnom = (Bl + Bu)/2;
        Lnom = _Vx*(Bl + Bu)/2;
        Wnom = _Vy*(Bl + Bu)/2;

        if(swingFoot == LFoot){
            Lnom_global = pRF_m.x + Lnom;
            Wnom_global = pRF_m.y + lp + Wnom;
        }
        else if(swingFoot == RFoot){
            Lnom_global = pLF_m.x + Lnom;
            Wnom_global = pLF_m.y - lp + Wnom;
        }

        bxnom = Lnom/(exp(w*Tnom) - 1);
        bynom = ((-1)^n)*lp/(1 + exp(w*Tnom)) - Wnom/(1 - exp(w*Tnom));


        t_now = 0;
        t_step = Tnom;

        step_phase_change_flag = 0;

        //cout<<"stpe_phase: "<<step_phase<<"R or L"<<step_data_buf[step_phase].R_or_L<<endl;
    }

    // swing foot & time optimization
    VectorNd X = StepOptimizer(n); //X = [uTx, uTy, tau, bx, by]'  //X = [desFootx, desFooty, exp(w*T), bx, by]'

    cout<<"total_t = "<<t_total<<" n = "<<n<<"  X = {"<<X(0)<<", "<<X(1)<<", "<<(1/w)*log(X(2))<<", "<<X(3)<<", "<<X(4)<<"}"<<endl;

    desFoot = vec3(X(0),X(1),0);
    t_step = (1/w)*log(X(2));
    dT = t_step - t_now;
    if(dT < dT_min) dT = dT_min;

    CP_eos = vec3(desFoot.x + X(3), desFoot.y + X(4),0);

    //// ------------------ddCOM generation----------------------------------------------------------
    cZMP = 1/(1 - exp(w*dT))*CP_eos - exp(w*dT)/(1 - exp(w*dT))*CP_m;
    cZMP_proj = zmpProjectionToSP_large(cZMP, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered, -0.01);

    double k_gain_y = 0.005;
    double k_gain_x = 0.004;
    double Fz_both = F_RF_filtered.z + F_LF_filtered.z;

    vec3 ZMP_m_local = Calc_local(pRF_old, qRF_old, pLF_old, qLF_old, ZMP_global);
    vec3 cZMP_proj_local = Calc_local(pRF_old, qRF_old, pLF_old, qLF_old, cZMP_proj);

    ddCOM_con.y = k_gain_y*Fz_both/zc*(ZMP_m_local.y - cZMP_proj_local.y);
    ddCOM_con.x = k_gain_x*Fz_both/zc*(ZMP_m_local.x - cZMP_proj_local.x);
    ddCOM_con.z = 0;

    dCOM = dCOM + ddCOM_con*dt;
    COM = COM + dCOM*dt;
    ////---------------------------------------------------------------------------------------------


    if(swingFoot == RFoot){
        if(F_RF_filtered.z <= 40 && t_now < t_step/2.0 && RF_landing_flag == true) RF_landing_flag = false;

        if(F_RF_filtered.z > 40 && t_now > t_step/2.0 && RF_landing_flag == false){
            RF_landing_flag = true;
        }

        // Z pattern
        RF_z_dz_ddz_pattern = FootZ_trajectory(t_step, t_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height, -0.01);

        // X pattern
        RF_x_dx_ddx_pattern = FootX_trajectory(t_step, t_now, dsp_ratio, RF_x_dx_ddx_old, desFoot.x);

        // Y pattern
        RF_y_dy_ddy_pattern = FootY_trajectory(t_step, t_now, dsp_ratio, RF_y_dy_ddy_old, desFoot.y);

    }
    else if(swingFoot == LFoot){
        if(F_LF_filtered.z <= 40 && t_now < t_step/2.0 && LF_landing_flag == true) LF_landing_flag = false;

        if(F_LF_filtered.z > 40 && t_now > t_step/2.0 && LF_landing_flag == false){
            LF_landing_flag = true;
        }

        // Z pattern
        LF_z_dz_ddz_pattern = FootZ_trajectory(t_step, t_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height, -0.01);

        // X pattern
        LF_x_dx_ddx_pattern = FootX_trajectory(t_step, t_now, dsp_ratio, LF_x_dx_ddx_old, desFoot.x);

        // Y pattern
        LF_y_dy_ddy_pattern = FootY_trajectory(t_step, t_now, dsp_ratio, LF_y_dy_ddy_old, desFoot.y);


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

    ////=================================== Calc QP ========================================================
    /// Setup REF
    /// IK
    LegJoints LJ = Hi.IK_COM(COM_m, qPel_m, pRF, qRF, pLF, qLF);

    /// Ref contact
    double foot_up_ratio = 0.45;
    double t_up  = t_step*foot_up_ratio*(1.0 - dsp_ratio);
    double t_half_dsp = t_step*dsp_ratio/2.0;
    double t_down = t_step - t_up - 2*t_half_dsp;

    if(swingFoot == RFoot){
        if(t_now <  t_half_dsp - 0.5*dt){
            REF.cRF = true; REF.RFdn = true; REF.RFup = false;
        }
        else if(t_now < t_up + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up
            REF.cRF = false; REF.RFdn = false; REF.RFup = true;
        }
        else if(t_now < t_up + t_half_dsp + t_down - 0.5*dt){                  // foot down, 5th order trajectory down
            REF.cRF = false; REF.RFdn = true; REF.RFup = false;
        }
        else{
            REF.cRF = true; REF.RFdn = true; REF.RFup = false;
        }
    }
    if(swingFoot == LFoot){
        if(t_now <  t_half_dsp - 0.5*dt){
            REF.cLF = true; REF.LFdn = true; REF.LFup = false;
        }
        else if(t_now < t_up + t_half_dsp - 0.5*dt){             //foot rising, 5th order trajectory up
            REF.cLF = false; REF.LFdn = false; REF.LFup = true;
        }
        else if(t_now < t_up + t_half_dsp + t_down - 0.5*dt){                  // foot down, 5th order trajectory down
            REF.cLF = false; REF.LFdn = true; REF.LFup = false;
        }
        else{
            REF.cLF = true; REF.LFdn = true; REF.LFup = false;
        }
    }


    /// CSP and JSP
    // COM & PEL
    REF.CSP.pCOM = COM;
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
    REF.CSV.dpCOM = dCOM;
    REF.CSV.dpPel = vec3();
    REF.CSV.dqPel = vec3();

    REF.JSV.dpPel = vec3();
    REF.JSV.dqPel = vec3();

    //RF, LF pos & ori
    REF.CSV.dpRF = vec3(RF_x_dx_ddx_pattern[1], RF_y_dy_ddy_pattern[1], RF_z_dz_ddz_pattern[1]);
    REF.CSV.dqRF = vec3();
    REF.CSV.dpLF = vec3(LF_x_dx_ddx_pattern[1], LF_y_dy_ddy_pattern[1], LF_z_dz_ddz_pattern[1]);
    REF.CSV.dqLF = vec3();

    /// CSA
    // dCOM & dPEL
    REF.CSA.ddpCOM = ddCOM_con;
    REF.CSA.ddpPel = ddCOM_con;
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

    k++;
    t_total += dt;
    t_now += dt;

    save_onestep(k);



    if(t_total - t_elapsed >= t_step){
        step_phase_change_flag = 1;
        step_phase++;
        t_elapsed += t_step;
    }

     cout<<"t_total : "<<t_total<<" t_elapsed: "<<t_elapsed<<" t_step: "<<t_step<<"flag: "<<(int)step_phase_change_flag<<endl;

    return DS;
}

void HB_DynamicWalk2::MeasurementInput(RobotStates _RST){
    ROBOTSTATE = _RST;

    F_RF=ROBOTSTATE.F_RF; F_LF=ROBOTSTATE.F_LF; M_RF=ROBOTSTATE.M_RF; M_LF=ROBOTSTATE.M_LF; pPel_m = ROBOTSTATE.CSP.pPel;
    IMUangle=ROBOTSTATE.IMUangle; IMUomega=ROBOTSTATE.IMUomega;
    pRF_m=ROBOTSTATE.CSP.pRF; pLF_m=ROBOTSTATE.CSP.pLF; qRF_m=ROBOTSTATE.CSP.qRF; qLF_m=ROBOTSTATE.CSP.qLF;

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

    CP_m = COM_m + dCOM_m/w; // measured Capture point

    // Measured Global qPel
    qPel_m = ROBOTSTATE.CSP.qPel;


    double alpha1 = 1/(1 + 2*PI*dt*8);  // for ankle torque generation
    COM_m_filtered = alpha1*COM_m_filtered + (1 - alpha1)*COM_m;
    dCOM_m_filtered = alpha1*dCOM_m_filtered + (1 - alpha1)*dCOM_m;


}

VectorNd HB_DynamicWalk2::StepOptimizer(int stanceLeg_index){
    //// ---------------------Variables-------------------------------------------
    OW_CPLEX OC;
    VectorNd X;

    ////---------------------- QP Solve-------------------------------------------------------
    //QP settings
    std::vector<MatrixNd> As, Bs;
    std::vector<double> Ws;

    MatrixNd A0, B0;
    double W0;
    
    double Lmin_global, Lmax_global, Wmin_global, Wmax_global;
    
    if(stanceLeg_index == 1){ //Right Leg stance
        Lmin_global = pRF_m.x + Lmin;
        Lmax_global = pRF_m.x + Lmax;
        Wmin_global = pRF_m.y + lp + Wmin; // min y range of swing foot
        Wmax_global = pRF_m.y + lp + Wmax; // max y range of swing foot
    }
    else if(stanceLeg_index == 2){ //left Leg stance
        Lmin_global = pLF_m.x + Lmin;
        Lmax_global = pLF_m.x + Lmax;
        Wmin_global = pLF_m.y - lp + Wmin; //min y range of swing foot
        Wmax_global = pLF_m.y - lp + Wmax;
    }
    OC.setNums(5, 2, 6);
    OC.MakeEq_StepOptimizer(CP_m, ZMP_global, t_now, w);
    OC.MakeIneq_StepOptimizer(Lmin_global, Lmax_global, Wmin_global, Wmax_global, Tmin, Tmax, w);

    //Behavior 1 : Lnom tracking
    A0 = MatrixNd::Zero(5,5);
    B0 = MatrixNd::Zero(5,1);

    A0(0,0) = 1; //uTx
    B0(0,0) = Lnom_global;
    W0 = 1;

    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    //Behavior 2 : Wnom tracking
    A0 = MatrixNd::Zero(5,5);
    B0 = MatrixNd::Zero(5,1);

    A0(0,1) = 1; //uTy
    B0(0,0) = Wnom_global;
    W0 = 1;

    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    //Behavior 3 : Tnom tracking
    A0 = MatrixNd::Zero(5,5);
    B0 = MatrixNd::Zero(5,1);

    A0(0,2) = 1; //tau
    B0(0,0) = exp(w*Tnom);
    W0 = 1;

    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    //Behavior 4 : bx tracking
    A0 = MatrixNd::Zero(5,5);
    B0 = MatrixNd::Zero(5,1);

    A0(0,3) = 1; //bx
    B0(0,0) = bxnom;
    W0 = 1000;

    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    //Behavior 5 : by tracking
    A0 = MatrixNd::Zero(5,5);
    B0 = MatrixNd::Zero(5,1);

    A0(0,4) = 1; //by
    B0(0,0) = bynom;
    W0 = 1000;

    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    //Making the problem convex (property of QuadProg++ Library)
    A0  = MatrixNd::Identity(5,5);//make it smaller
    B0  = MatrixNd::Zero(5,1);
    W0 = 1e-7;
    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    OC.MakeHF2(As,Bs,Ws);

    X = VectorNd::Zero(OC.getNUMCOLS());

    X = OC.calcX2();

    return X;
}

VectorNd HB_DynamicWalk2::dIK_Solver_COM(REFERENCE _REF){

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

    //making jacobian
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

VectorNd HB_DynamicWalk2::ddIK_Solver_COM(REFERENCE _REF){
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

void HB_DynamicWalk2::save_onestep(int cnt)
{
    if(k<200000)
    {
        SAVE[0][cnt] = COM.x;
        SAVE[1][cnt] = CP.x;
        SAVE[2][cnt] = 0;
        SAVE[3][cnt] = 0;
        SAVE[4][cnt] = CP_m_filtered.x;
        SAVE[5][cnt] = COM.y;
        SAVE[6][cnt] = CP.y;
        SAVE[7][cnt] = 0;
        SAVE[8][cnt] = 0;
        SAVE[9][cnt] = CP_m_filtered.y;

        SAVE[10][cnt] = 0;
        SAVE[11][cnt] = 0;
        SAVE[12][cnt] = 0;
        SAVE[13][cnt] = 0;
        SAVE[14][cnt] = ZMP_global.x;
        SAVE[15][cnt] = ZMP_global.y;
        SAVE[16][cnt] = CP_eos.x;
        SAVE[17][cnt] = CP_eos.y;

        SAVE[18][cnt] = IMUangle.x;
        SAVE[19][cnt] = IMUangle.y;
        SAVE[20][cnt] = IMUangle.z;

        SAVE[21][cnt] = 0;
        SAVE[22][cnt] = 0;
        SAVE[23][cnt] = 0;
        SAVE[24][cnt] = 0;
        SAVE[25][cnt] = 0;
        SAVE[26][cnt] = 0;

        SAVE[27][cnt] = dT;//dT_est;//dT_buf[0];
        SAVE[28][cnt] = 0;
        SAVE[29][cnt] = 0;

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
        SAVE[52][cnt] = 0;
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
        SAVE[67][cnt] = 0;



        //SAVE[47][cnt] = realCOM.z;//
    }
}

void HB_DynamicWalk2::save_all()
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




#endif // HB_DYNAMIC_WALK2

