#ifndef HB_WALKING_H
#define HB_WALKING_H

#include "BasicMatrix.h"
#include <unistd.h>
#include "../../share/Headers/RBSharedMemory.h"
#include <QVector>
#include "HB_functions.h"

#define     RFoot              -1
#define     DSP                 0
#define     SSP                 2
#define     LFoot               1
//#define     dt                  0.005

#define     PI                  3.141592
#define     SIGN(x)            (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define     ABS(a)             (((a)<(0)) ? -(a):(a))

using namespace std;

typedef struct _STEP_DATA{
    double x;
    double y;
    int R_or_L; // -1 : Right, 1: Left, 0 : DSP
    double t;
}STEP_DATA;

class HB_WALKING
{
public:
    const double dt = 0.005;
    double w, zc;
    double t_step, t_start, t_end, real_t_step, t_total, t_elapsed, dT, dT_min,t_stable, t_foot_now, step_len_x, step_len_y;
    double c_ssp_x, k_ssp_x, c_ssp_y, k_ssp_y, c_dsp_x, k_dsp_x, c_dsp_y, k_dsp_y;
    int  R_or_L, k, real_time_adjust_count;
    char step_phase_change_flag, real_step_phase_change_flag, real_time_adjust_counter_start_flag;
    unsigned int step_phase,real_step_phase, N_step;
    double init_foot_width;
    char swingFoot;
    vec3 COM_est, dCOM_est, ddCOM_est, ZMP_est, COM_m, COM_m_old, CP_m_filtered, COM_m_filtered,COM_m_filtered1,COM_m_filtered2,dCOM_m_filtered,dCOM_m_filtered1,dCOM_m_filtered2, dCOM_m, ddCOM_m, COM_old, dCOM_old, COM, dCOM, ddCOM, ddCOM_con, dCOM_con, COM_con;
    vec3 COM_choreonoid, dCOM_choreonoid, dCOM_choreonoid_filtered, ddCOM_con_filetered;
    vec3 CP, CP_m, CP_eos, COM_p, dCOM_p, CP_p, ZMP_p, ZMP_ref, ZMP_ref_filtered, COM_cps, dCOM_cps, COM_cpt, dCOM_cpt, CP_cps, ZMP_cps, CP_cpt, ZMP_cpt, ZMP_ref_inSP, ZMP_ref_filtered_inSP, ZMP_ref_MAX_filtered, ZMP_ref_MIN_filtered;
    vec3 pPel_old, pRF, pRF_old, pLF, pLF_old, F_RF, F_LF, M_RF, M_LF, IMUangle, dIMUangle, ZMP_global, ZMP_global_filtered,ZMP_global_filtered_old,dZMP_global_filtered, ZMP_local;
    vec3 ZMP_ref_gap, total_CP_eos_gap, CP_eos_gap, CP_eos_gap_old, CP_eos_gap_RF, CP_eos_gap_LF, CP_eos_gap_RF_old, CP_eos_gap_LF_old;
    vec3 F_RF_filtered, F_LF_filtered, M_RF_filtered, M_LF_filtered;
    quat qRF, qRF_old, qLF, qLF_old, qPel, qPel_old;
    quat qPel_m;
    double dsp_ratio, step_time_gap, step_time_gap_old,FootUp_height;
    vec3 RF_z_dz_ddz_old, LF_z_dz_ddz_old, RF_z_dz_ddz_pattern, LF_z_dz_ddz_pattern, RF_x_dx_ddx_old, LF_x_dx_ddx_old, RF_y_dy_ddy_old, LF_y_dy_ddy_old;
    vec3 RF_x_dx_ddx_pattern, LF_x_dx_ddx_pattern, RF_y_dy_ddy_pattern, LF_y_dy_ddy_pattern;
    double dt_gain1, dt_gain2, dt_gain3;
    QVector<STEP_DATA> step_data_buf, step_data_buf_original;  // x, y, R or L, time

    // torque control
    vec4 R_Ank_angVel, L_Ank_angVel, R_Ank_ref, L_Ank_ref, input_torque_filtered;
    vec3 R_ank_torque, L_ank_torque;

    // torque monitoring
    double R_KNEE_torque, L_KNEE_torque, R_HIP_P_torque, L_HIP_P_torque, R_HIP_R_torque, L_HIP_R_torque;

    // Landing Control
    vec3 pRF_landing, pLF_landing;
    char support_phase;
    bool RF_landing_flag, LF_landing_flag, _isRF_swingFirst, _isLF_swingFirst;
    double z_ctrl, Fz_diff_error;

    // Damping Control
    vec3 COM_damp_con;

    // Torso Orientation Control
    double RHR_Comp_Angle, LHR_Comp_Angle;

    //Control Flag
    char LandingControl_flag, DSPControl_flag, FootStepPosition_ctrl_flag, FootStepTime_ctrl_flag;
    char Choreonoid_flag;
    double SAVE[80][200000];

    // Function--------------------------------------------------------------------
    void save_onestep(int cnt);
    void save_all();
    void save_sysID(double _freq);

    HB_WALKING(){
        LandingControl_flag = false;
        DSPControl_flag = false;
        FootStepPosition_ctrl_flag = false;
        FootStepTime_ctrl_flag = false;
        FootUp_height = 0.07;

        zc = 0.69;
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
        CP_eos_gap_RF = vec3(0,0,0);
        CP_eos_gap_LF = vec3(0,0,0);
        CP_eos_gap_RF_old = vec3(0,0,0);
        CP_eos_gap_LF_old = vec3(0,0,0);
        CP_eos_gap_old = vec3(0,0,0);
        step_time_gap = 0;
        step_time_gap_old = 0;
        t_foot_now = 0.0;
        z_ctrl = 0;
        Choreonoid_flag = true;
        Fz_diff_error = 0;
        ZMP_ref_gap = vec3(0,0,0);

        dCOM_con = vec3(0,0,0);
        ddCOM_con = vec3(0,0,0);

        //for DSP
        c_dsp_x = 2.5;//450;
        k_dsp_x = 30;//200;//

        c_dsp_y = 3.7;//400;
        k_dsp_y = 15;//200;//400;//

        //for SSP
        c_ssp_x = 3.5;//450;
        k_ssp_x = 15;//200;//

        c_ssp_y = 3.2;//700;
        k_ssp_y = 16;//500;//



//        c_dsp_x = 140;
//        k_dsp_x = 20;//

//        c_dsp_y = 140;
//        k_dsp_y = 20;//


//        c_ssp_x = 140;
//        k_ssp_x = 20;//

//        c_ssp_y = 140;
//        k_ssp_y = 20;//

    }
    void HB_set_step(vec3 _COM_ini, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF, double _t_step, double _N_step, double _step_len_x, double _step_len_y, int _RL_first){
        // Initialize variable-----------------------------------------------------------------
        t_step = _t_step;
        t_start = t_step/3;
        real_t_step = t_start;
        t_end = t_step*1.1;
        t_stable = t_step*3;
        N_step = _N_step;
        R_or_L = _RL_first;
        k = 0; t_elapsed = 0;
        dT_min = 2*dt;//t_step*0.02;
        dT = real_t_step;
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
        COM_p = _COM_ini;
        dCOM_p = vec3(0,0,0);
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
        _isRF_swingFirst = true; _isLF_swingFirst = true;
        pRF_landing = pRF; pLF_landing = pLF;
        RF_landing_flag = true, LF_landing_flag = true;
        z_ctrl = 0;
        Fz_diff_error = 0;

        CP_eos_gap = vec3(0,0,0);
        CP_eos_gap_RF = vec3(0,0,0);
        CP_eos_gap_LF = vec3(0,0,0);
        CP_eos_gap_RF_old = vec3(0,0,0);
        CP_eos_gap_LF_old = vec3(0,0,0);
        CP_eos_gap_old = vec3(0,0,0);
        step_time_gap_old = 0;

        step_time_gap = 0;

        COM_damp_con = vec3(0,0,0);

        dt_gain1 = 30;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
        dt_gain2 = 25;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
        dt_gain3 = 25;
        //--------------------------------------------------------------------------------------

        // foot step & timing buffer-----------------------------------------------------------
        for(int i=0;i<N_step+2;i++){
            if(i == 0){
                if(R_or_L == 1)
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
    int CPS_CPT_Walking(void){ // measurement fucntion should be performed previously
        // COM & foot step & timing planning
        if(step_phase_change_flag == 1){
            t_step = step_data_buf[step_phase].t;
            dT = t_step;

            step_phase_change_flag = 0;

            //cout<<"stpe_phase: "<<step_phase<<"R or L"<<step_data_buf[step_phase].R_or_L<<endl;
        }

        if(real_step_phase_change_flag == 1){
            t_foot_now = 0.0;
            real_t_step = t_step;

            step_time_gap = 0;
            step_time_gap_old = 0;
//            CP_eos_gap = vec3(0,0,0);
//            CP_eos_gap_old = vec3(0,0,0);


            if(step_data_buf[real_step_phase].R_or_L == 0) swingFoot = DSP;
            else if(step_data_buf[real_step_phase].R_or_L == -1) swingFoot = RFoot;
            else swingFoot = LFoot;
            real_step_phase_change_flag = 0;

            //dt_gain scheduling
            if(real_step_phase <= N_step+1){
                // ankle torque con off
//                dt_gain1 = 25;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
//                dt_gain2 = 40;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
//                dt_gain3 = 30;  // Ankle Torque control

             // Ankle torque con on
             dt_gain1 = 25;  //16 is good sway without ATC// for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
             dt_gain2 = 25;  //20 is good sway without ATC// for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
             dt_gain3 = 24;  // Ankle Torque control
            }
            else{
                // Ankle torque con off
//                dt_gain1 = 25;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
//                dt_gain2 = 50;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
//                dt_gain3 = 30;  // Ankle Torque control

                // Ankle torque con on
                dt_gain1 = 35;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
                dt_gain2 = 50;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
                dt_gain3 = 60;  // Ankle Torque control
            }

            if(Choreonoid_flag == true){
                if(real_step_phase <= N_step+1){
                    dt_gain1 = 10;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
                    dt_gain2 = 15;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
                    dt_gain3 = 15;  // Ankle Torque control
                }
                else{
                    dt_gain1 = 30;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
                    dt_gain2 = 30;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
                    dt_gain3 = 30;  // Ankle Torque control
                }
            }



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


        CP_m = COM_m_filtered + dCOM_m_filtered/w;              //for ankle torque generation
        CP_m_filtered = COM_m_filtered2 + dCOM_m_filtered2/w;   //for ddCOM_con generation
//        CP_m = COM_est + dCOM_est/w;              //for ankle torque generation
//        CP_m_filtered = COM_est + dCOM_est/w;   //for ddCOM_con generation
//        CP_m = COM_choreonoid + dCOM_choreonoid_filtered/w;              //for ankle torque generation
//        CP_m_filtered = COM_choreonoid + dCOM_choreonoid_filtered/w;   //for ddCOM_con generation
        CP_cps = COM_cps + dCOM_cps/w;  //for CPS pattern
        CP_cpt = COM_cpt + dCOM_cpt/w;  //fpr CPT pattern

        // CPS approach
        ZMP_cps = 1/(1 - exp(w*dT))*CP_eos - exp(w*dT)/(1 - exp(w*dT))*CP_cps;

        COM_cps = cosh(w*dt)*COM_cps + sinh(w*dt)/w*dCOM_cps + (1-cosh(w*dt))*ZMP_cps;
        dCOM_cps = w*sinh(w*dt)*COM_cps + cosh(w*dt)*dCOM_cps - w*sinh(w*dt)*ZMP_cps;

        // CPT approach
//        dt_gain1 = 18;  // for COM pattern generation --> sway tuning parameter ( large value cuase little sway)
//        dt_gain2 = 55;  // for CP feedback(COM control) -->> controller gain tuning parameter ( large value is low control gain)
        //dt_gain3 = 25;  // for AnkleTorqueControl

        ZMP_cpt = 1/(1 - exp(w*dt*dt_gain1))*CP_cps - exp(w*dt*dt_gain1)/(1 - exp(w*dt*dt_gain1))*CP_cpt;
        ZMP_ref = 1/(1 - exp(w*dt*dt_gain3))*CP_cps - exp(w*dt*dt_gain3)/(1 - exp(w*dt*dt_gain3))*CP_m; //for ankle torque generation
        ZMP_ref_filtered = 1/(1 - exp(w*dt*dt_gain2))*CP_cps - exp(w*dt*dt_gain2)/(1 - exp(w*dt*dt_gain2))*CP_m_filtered; //for ddCOM_con generation

        COM_cpt = cosh(w*dt)*COM_cpt + sinh(w*dt)/w*dCOM_cpt + (1-cosh(w*dt))*ZMP_cpt;
        dCOM_cpt = w*sinh(w*dt)*COM_cpt + cosh(w*dt)*dCOM_cpt - w*sinh(w*dt)*ZMP_cpt;

        ZMP_ref_inSP = zmpProjectionToSP_large(ZMP_ref, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered, -0.01);//for ankle torque generation
        ZMP_ref_filtered_inSP = zmpProjectionToSP_large(ZMP_ref_filtered, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered,+0.15); //for COM control

        ZMP_ref_MAX_filtered = zmpProjectionToSP_large(ZMP_ref, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered,10.0);
        ZMP_ref_MIN_filtered = zmpProjectionToSP_large(ZMP_ref_filtered, pRF_old, pLF_old, qRF_old, qLF_old, F_RF_filtered, F_LF_filtered,0.0);

        double k_gain_y;
        double k_gain_x;

        if(Choreonoid_flag == true){ // choreonoid
            k_gain_y = 0.007;//--> gain scheduling needed
            k_gain_x = 0.004;
        }
        else{ //real hubo experiment
            if(real_step_phase <= N_step+1){
                k_gain_y = 0.015;//0.01;//1.5;//0.1;//1.2; //--> gain scheduling needed
                k_gain_x = 0.01;//1;//1.0 ;
            }
            else{
                k_gain_y = 0.004;//1.5;//0.1;//1.2; //--> gain scheduling needed
                k_gain_x = 0.004;//1.5;//1.0 ;
            }

        }

        double Fz_both = F_RF.z + F_LF.z;

        vec3 ZMP_ref_filtered_local = Calc_local(pRF_old, qRF_old, pLF_old, qLF_old, ZMP_ref_filtered_inSP);
        vec3 ZMP_filtered_local = Calc_local(pRF_old, qRF_old, pLF_old, qLF_old, ZMP_global_filtered);

//        ddCOM_con.y = k_gain_y*Fz_both/zc*(ZMP_filtered_local.y - ZMP_ref_filtered_local.y);
//        ddCOM_con.x = k_gain_x*Fz_both/zc*(ZMP_filtered_local.x - ZMP_ref_filtered_local.x);
//        ddCOM_con.z = 0;
        ddCOM_con.y = k_gain_y*Fz_both/zc*(ZMP_global_filtered.y - ZMP_ref_filtered_inSP.y);
        ddCOM_con.x = k_gain_x*Fz_both/zc*(ZMP_global_filtered.x - ZMP_ref_filtered_inSP.x);
        ddCOM_con.z = 0;

        double alpha_ddCOM = 1/(1 + 2*PI*dt*2);
        ddCOM_con_filetered = alpha_ddCOM*ddCOM_con_filetered + (1 - alpha_ddCOM)*ddCOM_con;

        // COM Damping Controller
        COM_damp_con = vec3(0,0,0);//ComDampingController(COM_damp_con, COM_cpt, dCOM_cpt, COM_m_filtered, dCOM_m_filtered, F_RF_filtered, F_LF_filtered);
        if(k % 5 == 0 ){

           // cout<<"RF land : "<<RF_landing_flag<<"  LF land : "<<LF_landing_flag<<"  COM con : "<<COM_damp_con.y<<endl;
            //cout<<"ddcony : "<<ddCOM_con.y<<endl;
        }

        // COM input shaping + Controller input fusing
        vec3 u;

        if(F_LF.z > 100 && F_RF.z > 100){  //DSP
            u.y = ddCOM_con.y + c_dsp_y*dCOM_cpt.y + k_dsp_y*COM_cpt.y;
            u.x = ddCOM_con.x + c_dsp_x*dCOM_cpt.x + k_dsp_x*COM_cpt.x;
            COM.y = exp(-k_dsp_y/c_dsp_y*dt)*COM.y - 1/k_dsp_y*u.y*(exp(-k_dsp_y/c_dsp_y*dt) - 1) + COM_damp_con.y;
            COM.x = exp(-k_dsp_x/c_dsp_x*dt)*COM.x - 1/k_dsp_x*u.x*(exp(-k_dsp_x/c_dsp_x*dt) - 1);
        }
        else{
            u.y = ddCOM_con.y + c_ssp_y*dCOM_cpt.y + k_ssp_y*COM_cpt.y;
            u.x = ddCOM_con.x + c_ssp_x*dCOM_cpt.x + k_ssp_x*COM_cpt.x;
            COM.y = exp(-k_ssp_y/c_ssp_y*dt)*COM.y - 1/k_ssp_y*u.y*(exp(-k_ssp_y/c_ssp_y*dt) - 1) + COM_damp_con.y;
            COM.x = exp(-k_ssp_x/c_ssp_x*dt)*COM.x - 1/k_ssp_x*u.x*(exp(-k_ssp_x/c_ssp_x*dt) - 1);
        }
//        u.y = ddCOM_con.y;// - 50*dCOM_cpt.y;
//        u.x = ddCOM_con.x;
//        COM_con.y = exp(-k_dsp_y/c_dsp_y*dt)*COM_con.y - 1/k_dsp_y*u.y*(exp(-k_dsp_y/c_dsp_y*dt) - 1);
//        COM_con.x = exp(-k_dsp_x/c_dsp_x*dt)*COM_con.x - 1/k_dsp_x*u.x*(exp(-k_dsp_x/c_dsp_x*dt) - 1);

//        dCOM_con  = dCOM_con + ddCOM_con*dt;
//        COM_con = COM_con + dCOM_con*dt;

//        if(COM_con.x >= 0.06) COM_con.x = 0.06;
//        if(COM_con.x <= -0.06) COM_con.x = -0.06;
//        if(COM_con.y >= 0.06) COM_con.y = 0.06;
//        if(COM_con.y <= -0.06) COM_con.y = -0.06;

//        COM.y = COM_cpt.y + COM_con.y;
//        COM.x = COM_cpt.x + COM_con.x;


        // Foot placement control---------------------------------------------------------------------------------------------------
//        ZMP_ref_gap =  ZMP_ref_filtered_inSP - ZMP_ref_MAX_filtered;
        if(real_step_phase >= 0 && real_step_phase <= N_step){


             //Step timing adjustment--------------------------------------------------------------



            //Step Position Adjustment--------------------------------------------------------------------------------------
//            if(FootStepPosition_ctrl_flag == true){
//                vec3 dCP_eos = vec3(0,0,0);
//                if((ZMP_ref_gap.y > 0.05 || ZMP_ref_gap.y < -0.05) && dT < real_t_step*0.9){
//                    dCP_eos.y = (1 - exp(w*dT))*ZMP_ref_gap.y;// - CP_eos_gap.y/(30.0);//(1 - exp(w*dT))*ZMP_ref_gap.y - CP_eos_gap.y/(30.0);
//                }
//                else{
//                    dCP_eos.y = -CP_eos_gap.y/(2.0);
//                }

////                if((ZMP_ref_gap.x > 0.2 || ZMP_ref_gap.x < -0.2) && dT < real_t_step*0.9)
////                    dCP_eos.x = (1 - exp(w*dT))*ZMP_ref_gap.x;// - CP_eos_gap.x/(30.0);
////                else
////                    dCP_eos.x = -CP_eos_gap.x/(50.0);

//                if(dT > real_t_step/4){
//                    CP_eos_gap.y = CP_eos_gap.y + dCP_eos.y*dt;
//                    cout<<"CP_eos_gap.y: "<<CP_eos_gap.y<<endl;
//                }

//                vec3 del_CP_eos_gap = CP_eos_gap - CP_eos_gap_old;

//                vec3 pFoot_des(0,0,0);

//                pFoot_des.y = step_data_buf[real_step_phase].y + del_CP_eos_gap.y;
//                step_data_buf[real_step_phase].y = FootY_pos_limiter(swingFoot, pPel_old, pFoot_des, pRF_old, pLF_old);

////                pFoot_des.x = step_data_buf[real_step_phase].x + del_CP_eos_gap.x;
////                step_data_buf[real_step_phase].x = FootX_pos_limiter(pPel_old, pFoot_des);
//                //cout<<"step_data_buf[real_step_phase].x = "<<step_data_buf[real_step_phase].x<<endl;
//                //cout<<"ZMP_ref_gap.y : "<<ZMP_ref_gap.y<<" del_CP_eos_gap.y: "<<del_CP_eos_gap.y<<" step_data_buf[real_step_phase].y: "<<step_data_buf[real_step_phase].y<<endl;


//            }
        }

        ZMP_ref_gap =  ZMP_ref_inSP - ZMP_ref_MAX_filtered;

//        if(FootStepTime_ctrl_flag == true && real_step_phase <= N_step){
//            double dstep_time_gap = 0;

////            if(ZMP_ref_gap.y > 0.05 || ZMP_ref_gap.y < -0.05){
////                //dstep_time_gap = -(1 - exp(w*dT))/((CP_eos_prev.y - step_data_buf[real_step_phase - 1].y)*w*exp(w*dT))*ZMP_ref_gap.y;// - step_time_gap/(30);
////                dstep_time_gap_y = -(1 - exp(w*dT))/((CP_m_filtered.y - ZMP_global_filtered.y)*w*exp(w*dT))*ZMP_ref_gap.y;// - step_time_gap/(30);
////                //cout<<CP_in_out_flag<<" ZMP ref gap : "<<ZMP_ref_gap.y<<" step time gap : "<<step_time_gap<<endl;
////            }
////            if(ZMP_ref_gap.x > 0.03 || ZMP_ref_gap.x < -0.03){
////                dstep_time_gap_x = -(1 - exp(w*dT))/((CP_m_filtered.x - ZMP_global_filtered.x)*w*exp(w*dT))*ZMP_ref_gap.x;// - step_time_gap/(30);
////                //cout<<CP_in_out_flag<<" ZMP ref gap : "<<ZMP_ref_gap.y<<" step time gap : "<<step_time_gap<<endl;
////            }


//            double min_step_time = 0.5;
////            if(dT > min_step_time){
////                step_time_gap += dstep_time_gap_y*dt;
////            }
////            if(dT > min_step_time){
////                step_time_gap += dstep_time_gap_x*dt;
////                //cout<<"step_time_gap = "<<step_time_gap<<endl;
////            }

//            if(ZMP_ref_gap.x > 0.001 || ZMP_ref_gap.x < -0.001 || ZMP_ref_gap.y > 0.001 || ZMP_ref_gap.y < -0.001){
//                vec3 temp = CP_m_filtered - ZMP_ref_filtered_inSP;
//                dstep_time_gap = -(1 - exp(w*dT))/(w*exp(w*dT))*dot(temp,ZMP_ref_gap)/dot(temp);
//            }

//            if(dT > min_step_time){
//                step_time_gap += dstep_time_gap*dt;
//                cout<<"step_time_gap = "<<step_time_gap<<endl;
//            }


//            //if(dT > min_step_time && step_time_gap < -(dT - min_step_time)) step_time_gap = -(dT - min_step_time);
//            if(step_time_gap > 0.3) step_time_gap = step_time_gap_old;
//            if(step_time_gap < min_step_time - t_step) step_time_gap = step_time_gap_old;

//            double del_step_time_gap = step_time_gap - step_time_gap_old;

//            step_data_buf[real_step_phase].t += del_step_time_gap;
//            //step_data_buf[real_step_phase+1].t += del_step_time_gap/2;
//            dT += del_step_time_gap;
//            real_t_step += del_step_time_gap;

//            step_time_gap_old = step_time_gap;
//        }

//        if(FootStepPosition_ctrl_flag == true && real_step_phase <= N_step){
//            vec3 dCP_eos = vec3(0,0,0);
//            if(swingFoot == RFoot && RF_landing_flag == false){
//                //y direction
//                if((ZMP_ref_gap.y > 0.001 || ZMP_ref_gap.y < -0.001) && (dT < real_t_step*0.98) && (dT > real_t_step*0.07)){
//                    dCP_eos.y = 3.5*(1 - exp(w*dT))*ZMP_ref_gap.y - CP_eos_gap_RF.y/(10.0);
//                }
//                else{
//                    dCP_eos.y = -CP_eos_gap_RF.y/(5.0);
//                }

//                //x direction
//                if((ZMP_ref_gap.x > 0.001 || ZMP_ref_gap.x < -0.001) && (dT < real_t_step*0.98) && (dT > real_t_step*0.07)){
//                    dCP_eos.x = (1 - exp(w*dT))*ZMP_ref_gap.x - CP_eos_gap_RF.x/(10.0);
//                }
//                else{
//                    dCP_eos.x = -CP_eos_gap_RF.x/(5.0);
//                }

//                CP_eos_gap_RF.y = CP_eos_gap_RF.y + dCP_eos.y*dt;
//                CP_eos_gap_RF.x = CP_eos_gap_RF.x + dCP_eos.x*dt;

//                vec3 del_CP_eos_gap = vec3(0,0,0);
//                del_CP_eos_gap = CP_eos_gap_RF - CP_eos_gap_RF_old;

//                vec3 pRFoot_des = vec3(0,0,0);

//                pRFoot_des.y = step_data_buf[real_step_phase].y + del_CP_eos_gap.y;
//                step_data_buf[real_step_phase].y = FootY_pos_limiter(swingFoot, pPel_old, pRFoot_des, pRF_old, pLF_old);

//                pRFoot_des.x = step_data_buf[real_step_phase].x + del_CP_eos_gap.x;
//                step_data_buf[real_step_phase].x = FootX_pos_limiter(pPel_old, pRFoot_des);

//                CP_eos_gap_RF_old = CP_eos_gap_RF;

//                //LF adjustment value return
//                CP_eos_gap_LF = CP_eos_gap_LF - CP_eos_gap_LF/(5.0);
//                CP_eos_gap_LF_old = CP_eos_gap_LF;
//            }
//            dCP_eos = vec3(0,0,0);
//            if(swingFoot == LFoot && LF_landing_flag == false){
//                //y direction
//                if((ZMP_ref_gap.y > 0.001 || ZMP_ref_gap.y < -0.001) && (dT < real_t_step*0.98) && (dT > real_t_step*0.07)){
//                    dCP_eos.y = (1 - exp(w*dT))*ZMP_ref_gap.y - CP_eos_gap_LF.y/(10.0);
//                }
//                else{
//                    dCP_eos.y = -CP_eos_gap_LF.y/(10.0);
//                }

//                //x direction
//                if((ZMP_ref_gap.x > 0.001 || ZMP_ref_gap.x < -0.001) && dT < (real_t_step*0.98) && (dT > real_t_step*0.07)){
//                    dCP_eos.x = 3.5*(1 - exp(w*dT))*ZMP_ref_gap.x - CP_eos_gap_LF.x/(10.0);
//                }
//                else{
//                    dCP_eos.x = -CP_eos_gap_LF.x/(5.0);
//                }

//                CP_eos_gap_LF.y = CP_eos_gap_LF.y + dCP_eos.y*dt;
//                CP_eos_gap_LF.x = CP_eos_gap_LF.x + dCP_eos.x*dt;

//                vec3 del_CP_eos_gap = vec3(0,0,0);
//                del_CP_eos_gap = CP_eos_gap_LF - CP_eos_gap_LF_old;

//                vec3 pLFoot_des = vec3(0,0,0);

//                pLFoot_des.y = step_data_buf[real_step_phase].y + del_CP_eos_gap.y;
//                step_data_buf[real_step_phase].y = FootY_pos_limiter(swingFoot, pPel_old, pLFoot_des, pRF_old, pLF_old);

//                pLFoot_des.x = step_data_buf[real_step_phase].x + del_CP_eos_gap.x;
//                step_data_buf[real_step_phase].x = FootX_pos_limiter(pPel_old, pLFoot_des);

//                CP_eos_gap_LF_old = CP_eos_gap_LF;

//                //RF adjustment value return
//                CP_eos_gap_RF = CP_eos_gap_RF - CP_eos_gap_RF/(5.0);
//                CP_eos_gap_RF_old = CP_eos_gap_RF;
//            }
//            else{
//                CP_eos_gap_LF = CP_eos_gap_LF - CP_eos_gap_LF/(5.0);
//                CP_eos_gap_LF_old = CP_eos_gap_LF;
//                CP_eos_gap_RF = CP_eos_gap_RF - CP_eos_gap_RF/(5.0);
//                CP_eos_gap_RF_old = CP_eos_gap_RF;
//            }
//        }





        // Pel Orientation controller---------------------------------------------------------------------------------------------------------------------------------
//        double rpyGain = 0.01;
//        double RollError = -IMUangle[0]; double PitchError = -IMUangle[1];
//        if(RollError >= 5) RollError = 5;
//        if(RollError <= -5) RollError = -5;
//        if(PitchError >= 5) PitchError = 5;
//        if(PitchError <= -5) PitchError = -5;
//        rpy RP_Error = rpy(rpyGain*RollError*D2R, rpyGain*PitchError*D2R, 0);
//        quat Error_quat = quat(RP_Error);
//        qPel =  qPel_old*Error_quat;

        vec3 comp_angle = HipRoll_Compensation(swingFoot, real_t_step, t_foot_now, dsp_ratio);
        RHR_Comp_Angle = comp_angle[0];
        LHR_Comp_Angle = comp_angle[1];



        // Ankle Torque Control ***************************************************************************************************************************
        VectorNd FF_torque = AnkleFT_Calculator(ZMP_ref_inSP, qPel_old, pRF, pLF, qRF, qLF, F_RF_filtered, F_LF_filtered);

        vec4 ZMP_FB_torque = zmpFeedBackController(0, ZMP_ref_inSP, vec3(0,0,0), ZMP_global_filtered, dZMP_global_filtered, qRF, qLF, F_RF, F_LF);

        vec4 Angle_FB_torque = AnkleJointAngleContoller(2, F_RF, F_LF, R_Ank_angVel, L_Ank_angVel, R_Ank_ref, L_Ank_ref);

        double alpha1 = 1/(1 + 2*PI*dt*2.5); // roll torque filtering
        double alpha2 = 1/(1 + 2*PI*dt*3); // pitch torque filtering
        for(int i=0 ;i<4;i++){
            if(i == 0 || i == 2)
                input_torque_filtered[i] = alpha1*input_torque_filtered[i] + (1.0 - alpha1)*(FF_torque[i]);// + ZMP_FB_torque[i] + Angle_FB_torque[i]);
            if(i == 1 || i == 3)
                input_torque_filtered[i] = alpha2*input_torque_filtered[i] + (1.0 - alpha2)*(FF_torque[i]);// + ZMP_FB_torque[i] + Angle_FB_torque[i]);
        }

        R_ank_torque.x = input_torque_filtered[0] + ZMP_FB_torque[0] + Angle_FB_torque[0];
        R_ank_torque.y = input_torque_filtered[1] + ZMP_FB_torque[1] + Angle_FB_torque[1];
        L_ank_torque.x = input_torque_filtered[2] + ZMP_FB_torque[2] + Angle_FB_torque[2];
        L_ank_torque.y = input_torque_filtered[3] + ZMP_FB_torque[3] + Angle_FB_torque[3];

       // Foot trajectory generation*************************************************************************************************************************

        if(swingFoot == RFoot){
            // Z direction
            if(LandingControl_flag == true)
            {
                if(F_RF_filtered.z <= 40 && t_foot_now < real_t_step/2.0 && RF_landing_flag == true){
                    RF_landing_flag = false;
                }

                if(F_RF_filtered.z > 40 && t_foot_now > real_t_step/2.0 && RF_landing_flag == false){
                    RF_landing_flag = true;
                    // Z direction
                    pRF_landing.z = pRF_old.z - 0.01;//RF_z_dz_ddz_pattern[0]-0.01;//
                    RF_z_dz_ddz_old[1] = RF_z_dz_ddz_old[1]*0.4;//RF_z_dz_ddz_old[1]/2; // make foot z speed zero at landing time
                    RF_z_dz_ddz_old[2] = RF_z_dz_ddz_old[2]*1;
                    cout<<real_step_phase<<" landing height RF : "<<pRF_landing.z<<endl;

                    // X direction
                    pRF_landing.x = pRF_old.x;
                    RF_x_dx_ddx_old[1] = 0.0; // foot x speed zero
                    RF_x_dx_ddx_old[2] = 0.0;
                    step_data_buf[real_step_phase].x = pRF_landing.x;

                    // Y direction
                    pRF_landing.y = pRF_old.y;
                    RF_y_dy_ddy_old[1] = 0.0; // foot y speed zero
                    RF_y_dy_ddy_old[2] = 0.0;
                    if(real_step_phase == N_step ){
                        step_data_buf[real_step_phase].y = pRF_landing.y;
                        step_data_buf[real_step_phase+1].y = pRF_landing.y + step_len_y/2;
                        step_data_buf[real_step_phase+2].y = pRF_landing.y + step_len_y/2;
                        step_data_buf[real_step_phase+3].y = pRF_landing.y + step_len_y/2;
                    }
                    else if(real_step_phase < N_step)
                        step_data_buf[real_step_phase].y = pRF_landing.y;
                }
                if(RF_landing_flag == false){
                    // z
                    //RF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height, -0.001);

                    // x
                    RF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_x_dx_ddx_old, step_data_buf[real_step_phase].x);

                    // y
                    double pFoot_y;
                    if(real_step_phase == N_step + 1)
                        pFoot_y = step_data_buf[real_step_phase].y - step_len_y/2;
                    else
                        pFoot_y = step_data_buf[real_step_phase].y;

                    RF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_y_dy_ddy_old, pFoot_y);


                }
                else if(RF_landing_flag == true){
                    //RF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height, pRF_landing.z);

                    RF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_x_dx_ddx_old, pRF_landing.x);

                    RF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_y_dy_ddy_old, pRF_landing.y);
                }


            }
            else{
                // Original Z pattern
                //RF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height, 0.00);

                // Original X pattern
                RF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_x_dx_ddx_old, step_data_buf[real_step_phase].x);

                // Original Y pattern
                double pFoot_y;
                if(step_data_buf[real_step_phase+1].R_or_L == 0)
                    pFoot_y = step_data_buf[real_step_phase].y - step_len_y/2;
                else
                    pFoot_y = step_data_buf[real_step_phase].y;

                RF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, RF_y_dy_ddy_old, pFoot_y);
            }




            // pLF recovary to 0 ----------------------------------------------------------------
            //LF_del_z = 0;
            _isLF_swingFirst = true;
            if(_isRF_swingFirst == true){
                LF_z_dz_ddz_old = vec3(LF_z_dz_ddz_pattern[0],0,0);//vec3(pLF.z, 0, 0);
                _isRF_swingFirst = false;
            }
            LF_z_dz_ddz_pattern = FootZ_recovary(real_t_step, t_foot_now, dsp_ratio, LF_z_dz_ddz_old, pLF_landing.z);
            pLF_landing.z = 0.00;
            //------------------------------------------------------------------------------------




        }
        else if(swingFoot == LFoot){
            // Z direction
            if(LandingControl_flag == true)
            {
                if(F_LF_filtered.z <= 40 && t_foot_now < real_t_step/2.0 && LF_landing_flag == true){
                    LF_landing_flag = false;
                }
                if(F_LF_filtered.z > 40 && t_foot_now > real_t_step/2.0 && LF_landing_flag == false){
                    LF_landing_flag = true;
                    pLF_landing.z = pLF_old.z - 0.01;//LF_z_dz_ddz_pattern[0] - 0.01;//
                    LF_z_dz_ddz_old[1] = LF_z_dz_ddz_old[1]*0.4;
                    LF_z_dz_ddz_old[2] = LF_z_dz_ddz_old[2]*1;
                    cout<<real_step_phase<<" landing height LF : "<<pLF_landing.z<<endl;

                    // X direction
                    pLF_landing.x = pLF_old.x;
                    LF_x_dx_ddx_old[1] = 0.0; // foot x speed zero
                    LF_x_dx_ddx_old[2] = 0.0;
                    step_data_buf[real_step_phase].x = pLF_landing.x;

                    // Y direction
                    pLF_landing.y = pLF_old.y;
                    LF_y_dy_ddy_old[1] = 0.0; // foot y speed zero
                    LF_y_dy_ddy_old[2] = 0.0;
                    if(real_step_phase == N_step){
                        step_data_buf[real_step_phase].y = pLF_landing.y;
                        step_data_buf[real_step_phase+1].y = pLF_landing.y - step_len_y/2;
                        step_data_buf[real_step_phase+2].y = pLF_landing.y - step_len_y/2;
                        step_data_buf[real_step_phase+3].y = pLF_landing.y - step_len_y/2;
                    }
                    else if(real_step_phase < N_step)
                        step_data_buf[real_step_phase].y = pLF_landing.y;

                }

                if(LF_landing_flag == false){
                    //LF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height, -0.001);

                    // x
                    LF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_x_dx_ddx_old, step_data_buf[real_step_phase].x);

                    // y
                    double pFoot_y;
                    if(real_step_phase == N_step + 1){
                        pFoot_y = step_data_buf[real_step_phase].y + step_len_y/2;
                    }
                    else
                        pFoot_y = step_data_buf[real_step_phase].y;

                    LF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_y_dy_ddy_old, pFoot_y);


                }
                else if(LF_landing_flag == true){
                    //LF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height, pLF_landing.z);

                    LF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_x_dx_ddx_old, pLF_landing.x);

                    LF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_y_dy_ddy_old, pLF_landing.y);

                }


            }
            else{
                // Original Z pattern
                //LF_z_dz_ddz_pattern = FootZ_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height, -0.00);

                // Original X pattern
                LF_x_dx_ddx_pattern = FootX_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_x_dx_ddx_old, step_data_buf[real_step_phase].x);

                // Original Y pattern
                double pFoot_y;
                if(step_data_buf[real_step_phase+1].R_or_L == 0)
                    pFoot_y = step_data_buf[real_step_phase].y + step_len_y/2;
                else
                    pFoot_y = step_data_buf[real_step_phase].y;

                LF_y_dy_ddy_pattern = FootY_trajectory(real_t_step, t_foot_now, dsp_ratio, LF_y_dy_ddy_old, pFoot_y);
            }



            // pRF recovary to 0 ----------------------------------------------------------------
            //RF_del_z = 0;
            _isRF_swingFirst = true;
            if(_isLF_swingFirst == true){
                RF_z_dz_ddz_old = vec3(RF_z_dz_ddz_pattern[0],0,0);//vec3(pRF.z, 0, 0);
                _isLF_swingFirst = false;
            }
            RF_z_dz_ddz_pattern = FootZ_recovary(real_t_step, t_foot_now, dsp_ratio, RF_z_dz_ddz_old, pRF_landing.z);
            pRF_landing.z = 0.00;
            //----------------------------------------------------------------------------------------


        }
        else{
//            if(real_step_phase == 0){
                RF_landing_flag = true;
                LF_landing_flag = true;
                _isRF_swingFirst = true;
                _isLF_swingFirst = true;

                RF_z_dz_ddz_old = vec3(pRF.z, 0,0);
                RF_x_dx_ddx_old = vec3(pRF.x, 0,0);
                RF_y_dy_ddy_old = vec3(pRF.y, 0,0);

                LF_z_dz_ddz_old = vec3(pLF.z, 0,0);
                LF_x_dx_ddx_old = vec3(pLF.x, 0,0);
                LF_y_dy_ddy_old = vec3(pLF.y, 0,0);



        }
        double t_half_dsp = real_t_step*(dsp_ratio)/2.0;

        if(t_foot_now <= t_half_dsp) support_phase = DSP;
        else if(t_foot_now <= real_t_step - t_half_dsp) support_phase = SSP;
        else support_phase = DSP;

        double Fz_diff_ref = FF_torque[5] - FF_torque[4]; // LFz_ref - RFz_ref
        double Fz_diff = F_LF.z - F_RF.z;

        double alpha = 1/(1 + 2*PI*dt*2);
        Fz_diff_error = alpha*Fz_diff_error + (1 - alpha)*(Fz_diff_ref - Fz_diff);

        //if(RF_landing_flag == true && LF_landing_flag == true){ // DSP
        //if(F_RF.z > 100 && F_LF.z > 100){
        if(support_phase == DSP || (F_RF.z > 40 && F_LF.z > 40)){
        //if(support_phase == DSP || (RF_landing_flag == true && LF_landing_flag == true)){

            double z_ctrl_gain = 0.0005;
            if(real_step_phase >= N_step) z_ctrl_gain = 0.0005/5;
            double dz_ctrl = z_ctrl_gain*Fz_diff_error - z_ctrl/20.0;

            z_ctrl += dz_ctrl*dt;

        }
        else{ // SSP
            double dz_ctrl = - z_ctrl/(5);

            z_ctrl += dz_ctrl;
        }

        if(z_ctrl > 0.06) z_ctrl = 0.06;
        if(z_ctrl < -0.06) z_ctrl = -0.06;

        //cout<<"z ctrl : "<<z_ctrl<<endl;
        if(DSPControl_flag == true){
            pRF.z = RF_z_dz_ddz_pattern[0];// + 0.5*z_ctrl;
            pLF.z = LF_z_dz_ddz_pattern[0];// - 0.5*z_ctrl;

            pRF.x = RF_x_dx_ddx_pattern[0];
            pLF.x = LF_x_dx_ddx_pattern[0];

            pRF.y = RF_y_dy_ddy_pattern[0];
            pLF.y = LF_y_dy_ddy_pattern[0];
        }
        else{
            pRF.z = RF_z_dz_ddz_pattern[0];
            pLF.z = LF_z_dz_ddz_pattern[0];

            pRF.x = RF_x_dx_ddx_pattern[0];
            pLF.x = LF_x_dx_ddx_pattern[0];

            pRF.y = RF_y_dy_ddy_pattern[0];
            pLF.y = LF_y_dy_ddy_pattern[0];
        }

        RF_z_dz_ddz_old = RF_z_dz_ddz_pattern;
        RF_x_dx_ddx_old = RF_x_dx_ddx_pattern;
        RF_y_dy_ddy_old = RF_y_dy_ddy_pattern;

        LF_z_dz_ddz_old = LF_z_dz_ddz_pattern;
        LF_x_dx_ddx_old = LF_x_dx_ddx_pattern;
        LF_y_dy_ddy_old = LF_y_dy_ddy_pattern;


        rpy qPel_rpy(qPel_m);
        if(ABS(qPel_rpy.r*R2D) > 30.0)
            return -1;


        if(dT > dT_min)
            dT = dT - dt;
        else
            dT = dT_min;

        k++;
        t_total += dt;
        t_foot_now += dt;

        save_onestep(k);

        if(t_total - t_elapsed >= real_t_step){
            step_phase_change_flag = 1;
            real_time_adjust_counter_start_flag = 1;
            step_phase++;
            t_elapsed += real_t_step;
            if(step_phase > N_step + 2) return -1;
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

    }

    void MeasurementInput(vec3 _COM_kine, vec3 _COM_est, vec3 _dCOM_est, vec3 _pPel, quat _qPel_kine, vec3 _IMUangle, quat _IMUquat, vec3 _IMUvel, vec3 _dIMUangle, vec3 _F_RF, vec3 _F_LF,
                          vec3 _M_RF, vec3 _M_LF, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF,
                            vec4 _R_Ank_angVel, vec4 _L_Ank_angVel, vec4 _R_Ank_ref, vec4 _L_Ank_ref){
        if(_F_RF.z <= 0) _F_RF.z = 0;
        if(_F_LF.z <= 0) _F_LF.z = 0;

        F_RF=_F_RF; F_LF=_F_LF; M_RF=_M_RF; M_LF=_M_LF; pPel_old = _pPel;
        IMUangle=_IMUangle; dIMUangle=_dIMUangle;
        pRF_old=_pRF; pLF_old=_pLF; qRF_old=_qRF; qLF_old=_qLF;
        R_Ank_angVel = _R_Ank_angVel; L_Ank_angVel = _L_Ank_angVel; R_Ank_ref = _R_Ank_ref; L_Ank_ref = _L_Ank_ref;

        dCOM_old = (_COM_kine - COM_old)/dt;
        COM_old = _COM_kine;  qPel_old=_qPel_kine;

        // COM estimation
        COM_est = _COM_est;
        dCOM_est = _dCOM_est;

        // ZMP calculation
        ZMP_global = ZMP_calc_global(_pRF, _qRF, F_RF, M_RF, _pLF, _qLF, F_LF, M_LF);

        // ZMP Filtering
        double alpha = 1/(1 + 2*PI*dt*25);
        ZMP_global_filtered = alpha*ZMP_global_filtered + (1 - alpha)*ZMP_global;
        dZMP_global_filtered = (ZMP_global_filtered - ZMP_global_filtered_old)/dt;
        ZMP_global_filtered_old = ZMP_global_filtered;

        //FT Sensor Filtering
        double alpha_FT = 1/(1 + 2*PI*dt*60);
        F_RF_filtered = F_RF_filtered*alpha_FT + _F_RF*(1 - alpha_FT);
        F_LF_filtered = F_LF_filtered*alpha_FT + _F_LF*(1 - alpha_FT);
        M_RF_filtered = M_RF_filtered*alpha_FT + _M_RF*(1 - alpha_FT);
        M_LF_filtered = M_LF_filtered*alpha_FT + _M_LF*(1 - alpha_FT);

        // Measured Global COM
        //COM_m = COM_measurment(_COM_kine, ZMP_global, IMUangle, _qPel_kine);
        COM_m = COM_measurment_by_quat(_COM_kine, ZMP_global, _IMUquat, _qPel_kine);
        dCOM_m = (COM_m - COM_m_old)/dt;
        COM_m_old = COM_m;
        //dCOM_m = dCOM_measurment(COM_old, dCOM_old, ZMP_global, IMUangle, _IMUvel, _qPel_kine);

        // Measured Global qPel
        qPel_m = Global_qPel_measurment(IMUangle, _qPel_kine);


        double alpha1 = 1/(1 + 2*PI*dt*8);  // for ankle torque generation
        COM_m_filtered = alpha1*COM_m_filtered + (1 - alpha1)*COM_m;
        dCOM_m_filtered = alpha1*dCOM_m_filtered + (1 - alpha1)*dCOM_m;

        double alpha2 = 1/(1 + 2*PI*dt*3.0);  // for ddCOM_con generation
        COM_m_filtered2 = alpha2*COM_m_filtered2 + (1 - alpha2)*COM_m;
        dCOM_m_filtered2 = alpha2*dCOM_m_filtered2 + (1 - alpha2)*dCOM_m;


    }

    void MeasurementInput_for_Choreonoid(vec3 _COM_kine, vec3 _COM_est, vec3 _dCOM_est, vec3 _pPel, quat _qPel_kine, vec3 _IMUangle, vec3 _IMUvel, vec3 _dIMUangle, vec3 _F_RF, vec3 _F_LF,
                          vec3 _M_RF, vec3 _M_LF, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF,
                            vec4 _R_Ank_angVel, vec4 _L_Ank_angVel, vec4 _R_Ank_ref, vec4 _L_Ank_ref){
        if(_F_RF.z <= 0) _F_RF.z = 0;
        if(_F_LF.z <= 0) _F_LF.z = 0;

        F_RF=_F_RF; F_LF=_F_LF; M_RF=_M_RF; M_LF=_M_LF; pPel_old = _pPel;
        IMUangle=_IMUangle; dIMUangle=_dIMUangle;
        pRF_old=_pRF; pLF_old=_pLF; qRF_old=_qRF; qLF_old=_qLF;
        R_Ank_angVel = _R_Ank_angVel; L_Ank_angVel = _L_Ank_angVel; R_Ank_ref = _R_Ank_ref; L_Ank_ref = _L_Ank_ref;

        dCOM_old = (_COM_kine - COM_old)/dt;
        COM_old = _COM_kine;  qPel_old=_qPel_kine;

        // COM estimation
        COM_est = _COM_est;
        dCOM_est = _dCOM_est;

        // ZMP calculation
        ZMP_global = ZMP_calc_global(_pRF, _qRF, F_RF, M_RF, _pLF, _qLF, F_LF, M_LF);

        // ZMP Filtering
        double alpha = 1/(1 + 2*PI*dt*25);
        ZMP_global_filtered = alpha*ZMP_global_filtered + (1 - alpha)*ZMP_global;
        dZMP_global_filtered = (ZMP_global_filtered - ZMP_global_filtered_old)/dt;
        ZMP_global_filtered_old = ZMP_global_filtered;

        //FT Sensor Filtering
        double alpha_FT = 1/(1 + 2*PI*dt*60);
        F_RF_filtered = F_RF_filtered*alpha_FT + _F_RF*(1 - alpha_FT);
        F_LF_filtered = F_LF_filtered*alpha_FT + _F_LF*(1 - alpha_FT);
        M_RF_filtered = M_RF_filtered*alpha_FT + _M_RF*(1 - alpha_FT);
        M_LF_filtered = M_LF_filtered*alpha_FT + _M_LF*(1 - alpha_FT);

        // Measured Global COM
        COM_m = COM_measurment(_COM_kine, zc, ZMP_global, IMUangle, _qPel_kine);
        dCOM_m = (COM_m - COM_m_old)/dt;
        COM_m_old = COM_m;
        //dCOM_m = dCOM_measurment(COM_old, dCOM_old, ZMP_global, IMUangle, _IMUvel, _qPel_kine);

        // Measured Global qPel
        qPel_m = Global_qPel_measurment(IMUangle, _qPel_kine);


        double alpha1 = 1/(1 + 2*PI*dt*15);  // for ankle torque generation
        COM_m_filtered = alpha1*COM_m_filtered + (1 - alpha1)*COM_m;
        dCOM_m_filtered = alpha1*dCOM_m_filtered + (1 - alpha1)*dCOM_m;
//        dCOM_m_filtered = (COM_m_filtered - COM_m_filtered_old)/dt;
//        COM_m_filtered_old = COM_m_filtered;

        double alpha2 = 1/(1 + 2*PI*dt*15);  // for ddCOM_con generation
        COM_m_filtered2 = alpha2*COM_m_filtered2 + (1 - alpha2)*COM_m;
        dCOM_m_filtered2 = alpha2*dCOM_m_filtered2 + (1 - alpha2)*dCOM_m;
//        dCOM_m_filtered2 = (COM_m_filtered2 - COM_m_filtered2_old)/dt;
//        COM_m_filtered2_old = COM_m_filtered2;






    }

    void Measurement_Choreonoid(vec3 _pPel_global, quat _qPel_global, vec3 _pCOM_kine, vec3 _pPel_kine){
        quat G_R_g = _qPel_global;
        vec3 gPEL2COM = _pCOM_kine -_pPel_kine;

        vec3 COM_temp = _pPel_global + G_R_g*gPEL2COM;

        dCOM_choreonoid = (COM_temp - COM_choreonoid)/dt;

        double alpha = 1/(1 + 2*PI*dt*30);
        dCOM_choreonoid_filtered = alpha*dCOM_choreonoid_filtered + (1 - alpha)*dCOM_choreonoid;

        COM_choreonoid = COM_temp;
    }
};

void HB_WALKING::save_onestep(int cnt)
{
    if(k<200000)
    {
        SAVE[0][cnt] = COM.x;
        SAVE[1][cnt] = CP.x;
        SAVE[2][cnt] = CP_cps.x;
        SAVE[3][cnt] = ZMP_ref_filtered_inSP.x;
        SAVE[4][cnt] = CP_m_filtered.x;
        SAVE[5][cnt] = COM.y;
        SAVE[6][cnt] = CP.y;
        SAVE[7][cnt] = CP_cps.y;
        SAVE[8][cnt] = ZMP_ref_filtered_inSP.y;
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
        SAVE[28][cnt] = R_ank_torque.x;
        SAVE[29][cnt] = L_ank_torque.x;

        SAVE[30][cnt] = pRF.z;
        SAVE[31][cnt] = pLF.z;
        SAVE[32][cnt] = z_ctrl;

        SAVE[33][cnt] = COM_m.x; // COM_measure x
        SAVE[34][cnt] = COM_m.y; // COM_measure y

        SAVE[35][cnt] = R_ank_torque.y;
        SAVE[36][cnt] = L_ank_torque.y;

        SAVE[37][cnt] = R_HIP_P_torque; // temp x
        SAVE[38][cnt] = R_HIP_R_torque; // temp y

        SAVE[39][cnt] = R_KNEE_torque;//COM_est.x + dCOM_est.x/w; // CP estimation x
        SAVE[40][cnt] = swingFoot;// + dCOM_est.y/w; // CP estimation y

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
        SAVE[52][cnt] = pLF.x;
        SAVE[53][cnt] = dCOM.y;
        SAVE[54][cnt] = pLF.y;

        SAVE[55][cnt] = COM_old.x;
        SAVE[56][cnt] = COM_old.y;
        SAVE[57][cnt] = ZMP_est.y;

        SAVE[58][cnt] = CP_eos_gap_RF.x;
        SAVE[59][cnt] = COM_choreonoid.y;
        SAVE[60][cnt] = CP_eos_gap_LF.x;
        SAVE[61][cnt] = dCOM_choreonoid.y;

        SAVE[62][cnt] = ZMP_ref_gap.y;
        SAVE[63][cnt] = CP_eos_gap_RF.y;
        SAVE[64][cnt] = pLF_landing.y;

        SAVE[65][cnt] = CP_eos_gap_LF.y;
        SAVE[66][cnt] = ZMP_ref_gap.x;
        SAVE[67][cnt] = (double)LF_landing_flag;



        //SAVE[47][cnt] = realCOM.z;//
    }
}

void HB_WALKING::save_all()
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

void HB_WALKING::save_sysID(double _freq)
{
    printf("walk finished %d\n",k);
    char name[100];
    sprintf(name, "/home/rainbow/Desktop/%.2f.txt",_freq);
    FILE* ffp = fopen(name,"w");
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




#endif // HB_WALKING_H
