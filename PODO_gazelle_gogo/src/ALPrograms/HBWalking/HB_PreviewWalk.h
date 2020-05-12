#ifndef HB_PREVIEWWALK
#define HB_PREVIEWWALK

#include "BasicMatrix.h"
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include "../../share/Headers/RBSharedMemory.h"
#include "../../share/Headers/UserSharedMemory.h"
#include <QVector>
#include "HB_functions.h"
#include "../../Gazelle_Ankle_Kine/Gazelle_kine.h"
#include "HB_StepAdjustor.h"

#define     RFoot              -1
#define     DSP                 0
#define     SSP                 2
#define     LFoot               1
//#define     dt                  0.002

#define     PI                  3.141592
#define     SIGN(x)            (x >= 0) ? 1 : ((x < 0) ? -1 : 0)
#define     ABS(a)             (((a)<(0)) ? -(a):(a))

using namespace std;

typedef struct _STEP_INFO{
    vec3 Fpos;
    quat Fquat;
    double yaw_rad;
    int swingFoot;
    double t;
    int ros_step_phase;
}STEP_INFO;

typedef struct _WINDOW{
    vec3 ZMP_ref;
    vec3 ZMP_3m;
    vec3 COM;
    vec3 CP;

    vec3 RF_x_dx_ddx;
    vec3 RF_y_dy_ddy;
    vec3 RF_z_dz_ddz;
    quat qRF;

    vec3 LF_x_dx_ddx;
    vec3 LF_y_dy_ddy;
    vec3 LF_z_dz_ddz;
    quat qLF;

    double t_now;
    int step_phase;
}WINDOW;

class HB_PreviewWalk
{
public:
    pRBCORE_SHM_COMMAND     sharedCMD;
    pRBCORE_SHM_REFERENCE   sharedREF;
    pRBCORE_SHM_SENSOR      sharedSEN;
    pUSER_SHM               userData;

    // Robot parameter
    const double pelv_w = 0.2182;
    double zc, w;
    const double g = 9.81;
    const double dt = 0.002;

    // vector
    vec3 COM_ref, dCOM_ref, ddCOM_ref, CP_ref, pRF_ref, pLF_ref;
    vec3 RF_z_dz_ddz, RF_x_dx_ddx, RF_y_dy_ddy, LF_z_dz_ddz, LF_x_dx_ddx, LF_y_dy_ddy;
    vec3 RF_z_dz_ddz_old, RF_x_dx_ddx_old, RF_y_dy_ddy_old, LF_z_dz_ddz_old, LF_x_dx_ddx_old, LF_y_dy_ddy_old;
    vec3 RF_z_dz_ddz_w, RF_x_dx_ddx_w, RF_y_dy_ddy_w, LF_z_dz_ddz_w, LF_x_dx_ddx_w, LF_y_dy_ddy_w;
    vec3 RF_z_dz_ddz_old_w, RF_x_dx_ddx_old_w, RF_y_dy_ddy_old_w, LF_z_dz_ddz_old_w, LF_x_dx_ddx_old_w, LF_y_dy_ddy_old_w;
    vec3 COM_x_dx_ddx, COM_x_dx_ddx_old, COM_z_dz_ddz, COM_z_dz_ddz_old, COM_y_dy_ddy, COM_y_dy_ddy_old; 
    vec3 ZMP_global, p_out, CP_m, CP_m_comp;
    vec3 F_RF_filtered, F_LF_filtered, M_RF_filtered, M_LF_filtered, ACC_RF_Hfiltered, ACC_LF_Hfiltered, ACC_RF_old, ACC_LF_old;
    vec3 ACC_RF_filtered, ACC_LF_filtered;

    vec3 CP_m_filtered, COM_m, dCOM_m, COM_m_old, COM_m_filtered, dCOM_m_filtered, dCOM_m_diff_filtered, dCOM_m_imu_filtered;
    vec3 COM_m_comp, COM_m_comp_old, COM_m_comp_filtered, dCOM_m_comp, dCOM_m_comp_filtered;
    vec3 Ye_obs, dYe_obs, Xe_obs, dXe_obs;
    vec3 dCOM_m_diff, dCOM_m_imu;
    vec3 COM_y_dy_ddy_pre, COM_y_dy_ddy_old_pre, COM_x_dx_ddx_pre, COM_x_dx_ddx_old_pre;
    double Y_zmp_e_sum_pre, X_zmp_e_sum_pre;
    vec3 COM_e_imu_local, dCOM_e_imu_local, COM_e_imu_local_filtered, dCOM_e_imu_local_filtered;

    // data plot
    vec3 ZMP_local;
    // cZMPs
    vec3 cZMP, cZMP_proj, ddCOM_con,dCOM_con, COM_con, uCOM, duCOM, uCOM_old, cZMP_TC, cZMP_TC_local, cZMP_TC_global, cZMP_TC_proj, cZMP_dsp_local, cZMP_dsp_global, cZMP_dsp_local_proj, cZMP_dsp_global_proj;
    vec3 cZMP_proj_filtered;
    vec3 cZMP_local_scaled;

    // COM ref
    vec3 COM_ref_local, dCOM_ref_local, ddCOM_ref_local;
    vec3 dCOM_ref_local_filtered, ddCOM_ref_local_filtered;
    vec3 COM_LIPM, dCOM_LIPM, ddCOM_LIPM;

    // COM damping control
    vec3 COM_obs, dCOM_obs, Y_obs, dY_obs, X_obs, dX_obs;
    vec3 COM_damping_con, ZMP_error_local, COM_error_local, dCOM_error_local, ZMP_error_global;

    // ZMP_tracking_control
    vec3 COM_zmp_con, dCOM_zmp_con;

    // quat
    quat qRF_ref, qLF_ref, qPel_ref;

    // Orientation Trajectory
    vec3 RF_delta_yaw_d_dd, LF_delta_yaw_d_dd, RF_delta_yaw_d_dd_old, LF_delta_yaw_d_dd_old;
    quat RF_yaw_delta_quat, LF_yaw_delta_quat;
    quat RF_yaw_quat_ref, LF_yaw_quat_ref, PEL_yaw_quat_ref;
    quat RF_yaw_quat_first, LF_yaw_quat_first;
    double RF_delta_yaw_goal_rad, LF_delta_yaw_goal_rad;

    // int
    unsigned int step_phase, N_step, No_of_cycle, NL;

    unsigned int k;

    // double
    double t_total, t_elapsed, t_step, t_start, t_prev, t_window, t_now, t_now_w, t_stable, dT;
    double Y_zmp_e_sum, X_zmp_e_sum;
    double step_stride, dsp_ratio, dsp_ratio_com;
    double FootUp_height;
    double SAVE[80][50000];
    //double WDSAVE[3000][10][500];
    double dt_gain1;


    // char
    char R_or_L = RFoot;

    // bool
    bool step_phase_change_flag;
    bool walk_done_flag;

    // joystick Walking
    bool Joystick_walk_flag, Joystick_off_signal, Joystick_on_signal;

    vec3 des_Velocity; // (Vx, Vy, theta_dot) //from joystick input
    double des_step_t, des_dsp_ratio;
    double ros_step_t = 1.0;
    vec3 del_pos; // (delX, delY, delTheta_deg) // should be calculated
    void Calc_del_pos_from_Joystick(int _RJOG_RL, int _RJOG_UD, int _LJOG_RL, int _LJOG_UD, double _des_t_step);
    vec3 StancF_to_NextF_limitation(char _swingFoot, vec3 _stancF_to_NextF);
    int Joystick_off_count;

    // Preview system matrix
    mat3 A;
    vec3 B, C;

    // Robot states
    RobotStates RS;

    // Ankle Kinematic object
    Gazelle_Kine GK;
    

    // Preview Gain
    double PreviewGains[1000][5000];
    double Gi;
    double Gx[3];
    double Gp[5000];
    vec3 p_ref[5000];
    vec3 p_ref_for_COM[5000];

    QVector<STEP_INFO> SDB, SDB_original;
    WINDOW WD[5000];
    WINDOW WD_SA_ref[5000], WD_SA_ref_extended[5000];

    // Ankle torque Control
    vec3 RF_del_pos, LF_del_pos;
    quat RF_del_quat, LF_del_quat;
    vec3 RF_del_angle, LF_del_angle, RF_del_angle_old, LF_del_angle_old;

    // SwingPhase Vibration Control
    bool Swing_leg_vibration_control_flag;
    vec3 R_roll_obs, L_roll_obs, dR_roll_obs, dL_roll_obs;
    vec3 R_pitch_obs, L_pitch_obs, dR_pitch_obs, dL_pitch_obs;
    vec3 X_LTorsoRoll, dX_LTorsoRoll, X_RTorsoRoll, dX_RTorsoRoll;
    vec3 X_LTorsoYaw, dX_LTorsoYaw, X_RTorsoYaw, dX_RTorsoYaw;
    quat RF_quat_cur, LF_quat_cur;

    double RHR_con_deg, LHR_con_deg, RHY_con_deg, LHY_con_deg, LHP_con_deg, RHP_con_deg;
    double LSwingLeg_Vib_Control(vec3 _ACC_LF_filtered, bool return_flag);
    double RSwingLeg_Vib_Control(vec3 _ACC_RF_filtered, bool return_flag);
    double LTorso_Roll_Vib_Control(vec3 _IMULocalW, bool return_flag);
    double RTorso_Roll_Vib_Control(vec3 _IMULocalW, bool return_flag);
    double LTorso_Yaw_Vib_Control(vec3 _IMULocalW, bool return_flag);
    double RTorso_Yaw_Vib_Control(vec3 _IMULocalW, bool return_flag);
    double LSwingLeg_Pitch_Vib_Control(vec3 _ACC_LF_filtered, bool return_flag);
    double RSwingLeg_Pitch_Vib_Control(vec3 _ACC_RF_filtered, bool return_flag);
    void R_Vib_Control_init();
    void L_Vib_Control_init();

    // Ankle torque Control
    vec4 AnkleTorque_ref;
    vec4 InputTorque;  //init need
    LegJoints LJ_ref, LJ_ref_old;
    vec4 Ankle_Torque_from_cZMP(vec3 _global_cZMP, vec3 _global_ZMP, quat _global_qRF, quat _global_qLF,
                                                vec3 _global_pRF, vec3 _global_pLF, vec3 _F_RF, vec3 _F_LF);
    vec4 Ankle_Torque_from_PositionFB(int _ctrl_mode, vec3 _F_RF, vec3 _F_LF,
                          double _RAP_ref_deg, double _RAR_ref_deg, double _LAP_ref_deg, double _LAR_ref_deg,
                          double _RAP_cur_deg, double _RAR_cur_deg, double _LAP_cur_deg, double _LAR_cur_deg,
                          double _RAP_vel_ref_deg, double _RAR_vel_ref_deg, double _LAP_vel_ref_deg, double _LAR_vel_ref_deg,
                          double _RAP_vel_cur_deg, double _RAR_vel_cur_deg, double _LAP_vel_cur_deg, double _LAR_vel_cur_deg);
    void AnkleTorqueController(double _RAR_T_ref, double _RAP_T_ref, double _LAR_T_ref, double _LAP_T_ref,
                                               double _RA1_ang_deg, double _RA2_ang_deg, double _LA1_ang_deg, double _LA2_ang_deg,
                                               double _RAP_ang_deg, double _RAR_ang_deg, double _LAP_ang_deg, double _LAR_ang_deg,
                                               double _RA1_vel_deg, double _RA2_vel_deg, double _LA1_vel_deg, double _LA2_vel_deg,
                                               vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF);

    void AnkleQuatCalculaltor(double _RAP_ang_deg_cur, double _RAR_ang_deg_cur, double _LAP_ang_deg_cur, double _LAR_ang_del_deg);

    vec3 AnkleT_to_MotorT_left(vec3 M_ref, double _AP_ang_deg, double AR_ang_deg, double _A1_ang_deg, double _A2_ang_deg);
    vec3 AnkleT_to_MotorT_right(vec3 M_ref, double _AP_ang_deg, double AR_ang_deg, double _A1_ang_deg, double _A2_ang_deg);
    int Torque_to_Duty(double _Tmotor, double _Vel_degsec, int _mode);
    double CP_BEMF_comp(double Vel_degsec);
    double NCP_BEMF_comp(double Vel_degsec);

    // Landing controller
    bool RF_landing_flag, LF_landing_flag, LandingControl_flag, _isLF_swingFirst, _isRF_swingFirst;
    vec3 pRF_landing, pLF_landing;
    double RF_landing_time, LF_landing_time;
    double Max_recovary_speed, recovary_speed, del_t;
    double LandingZ_des, pRF_z_impact_new, pLF_z_impact_new;
    vec3 Landing_delXY;
    mat3 G_R_g_pitroll;
    rpy G_R_g_pitroll_rpy;
    double Calc_Landing_Threshold(char _swingFoot, vec3 _cZMP_dsp);

    // dsp controller
    bool DSP_time_flag, DSP_force_flag, DSP_FZ_Control_flag;
    double Fz_diff_ref, Fz_diff, Fz_diff_error;
    double dz_ctrl, dz_ctrl_filtered, z_ctrl, z_ctrl_filtered;
    double Alpha_dsp, Alpha_AT;
    void DSP_Fz_controller(char _swingFoot, double _F_RF_z, double _F_LF_z);
    void DSP_Fz_controller2(char _swingFoot);
    double dsp_ctrl_gain, dsp_tilt_gain;
    double RF_Fz_ref, LF_Fz_ref;

    // Step pos & timing adjustment
    vec3 COM_x_dx_ddx_SA, COM_x_dx_ddx_old_SA, COM_y_dy_ddy_SA, COM_y_dy_ddy_old_SA;
    vec3 COM_SA_ref, dCOM_SA_ref, ddCOM_SA_ref;
    vec3 p_ref_SA[5000];
    double Y_zmp_e_sum_SA, X_zmp_e_sum_SA;
    vec3 p_out_SA;
    int t_now_index;
    vec3 COM_SA_LIPM, dCOM_SA_LIPM, ddCOM_SA_LIPM;
    vec3 COM_SA_ref_local, dCOM_SA_ref_local, ddCOM_SA_ref_local;
    vec3 cZMP_SA_lf, cZMP_SA_rf;
    vec3 CP_error_lf, CP_error_rf, CP_SA_ref_local;
    vec3 del_u_f, del_b_f, del_u_g, del_b_g, del_u_f_modi;
    vec3 del_u_f_filtered, del_b_f_filtered;
    double new_T_filtered;
    double new_T, T_nom;
    bool SA_Enable_flag,  UD_flag, StepAdjustControl_flag;
    vec3 del_u_modification_by_foot_size(vec3 _del_u_f);
    vec3 CP_error_local_SA, CP_error_global_SA;


    // HipTorque Control
    double Omega_pitch, Omega_roll, Omega_pitch_filtered, Omega_roll_filtered;
    double Pelv_pitch_acc_ref, Pelv_roll_acc_ref;
    double Pelv_pitch_vel_ref, Pelv_roll_vel_ref, Pelv_pitch_ref, Pelv_roll_ref;
    bool HipTorqueControl_flag;

    vec3 del_b0_Nf, b0_Nf, del_u_Nf, del_b_Nf, del_u_Nf_filtered, del_b_Nf_filtered;
    vec3 del_b_Nf_g, del_u_Nf_g;
    double new_T_Nf, new_T_Nf_filtered;

    //del Z Controller
    bool delZcon_Enable_flag;
    double dz_com_ctrl, z_com_ctrl;


    // Ankle torque control by position control
    vec3 RF_angle_ctrl, LF_angle_ctrl, dRF_angle_ctrl, dLF_angle_ctrl;
    void AnkleTorqueController_pos(double _RAR_T_ref, double _RAP_T_ref, double _LAR_T_ref, double _LAP_T_ref,
                                   vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF);
    bool Pos_Ankle_torque_control_flag;
    quat RF_quat_ctrl, LF_quat_ctrl;
    bool Standing_mode_flag;

    // Foot landing angle Control
    bool Landing_angle_control_flag;
    rpy RF_landing_angle, LF_landing_angle, dRF_landing_angle, dLF_landing_angle;

    // Link deflection compensation
    bool Joint_deflection_compensation_flag;
    double L_roll_compen_deg, R_roll_compen_deg, L_knee_compen_deg, R_knee_compen_deg;

    // Rotational momentum compensation
    double Angular_momentum_about_COM;
    double dWST_des_rad, dWST_rad, WST_rad, WST_ref_deg;

    // p_ref offset control
    bool ZMP_offset_controller_flag;
    double p_ref_offset_y, p_ref_con_error_filtered;
    double sway_con;

    // Calc_ landing Heigh
    double Landing_Z_filetered;
    vec3 Landing_delXY_filtered;
    double Calc_LandingHeight(char _SwingFoot, vec3 _pLF_des, vec3 _pRF_des, quat _qLF_ref, quat _qRF_ref, mat3 _G_R_g_pitroll);
    vec3 Calc_Landing_delXY(char _SwingFoot, vec3 _pLF_des, vec3 _pRF_des, quat _qLF_ref, quat _qRF_ref, mat3 _G_R_g_pitroll);
    vec3 des_pLF_local, des_pRF_local;

    // MemberFunctions
    int Preveiw_walking();
    void MeasurementInput(RobotStates _RST);
    void WindowFill_3mass();
    void Fill_ZMPref_get_CPref(int _dt_gain);
    void WindowFill();
    void WindowFill_3rd();
    void WindowFill_3rd_heel2toe();
    void WindowFill_3rd_heel2toe_3rd();
    void WindowFill_SA_ref();
    void WindowFill_SA_ref2();
    void WindowFill_SA_ref_3rd();
    void save_onestep(int cnt);
    void save_all();
    void save_WD();
    vec3 DampingControl();
    void DampingControl2();
    void DampingControl3();
    void ZMP_Tracking_controller();
    void State_Observer();

    vec3 InputShaping();
    int Logging();
    int Test_init(vec3 _COM_ini, vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF);
    void R_SSP_ZMP_Control(vec3 _ZMP_des_local, vec3 _ZMP_measure_local, vec3 _FT_F, vec3 _FT_M);
    void L_SSP_ZMP_Control(vec3 _ZMP_des_local, vec3 _ZMP_measure_local, vec3 _FT_F, vec3 _FT_M);
    void DSP_GRF_Control(vec3 _ZMP_ref, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF,
                          vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF);

    vec3 COM_measurment(vec3 _COM, double _zc, vec3 _ZMP_global, vec3 _IMUangle, quat _qPel);

    // Constructor
    HB_PreviewWalk(){
        zc = 0.60;
        w = sqrt(g/zc);
        step_phase = 0;
        k = 0;
        t_total = 0;
        t_elapsed = 0;
        COM_ref = vec3();
        COM_LIPM = vec3();

        step_phase_change_flag = true;

        // Controller Flags       
        StepAdjustControl_flag = true;
        // --> variable filtering of dp

        HipTorqueControl_flag = true;

        Landing_angle_control_flag = false;


        ZMP_offset_controller_flag = false;






        ////---------Controller tuning order
        //1. Com Damping Controller
        //2. Swing leg vibration Control
        Swing_leg_vibration_control_flag = true;

        //3. Joint deflection compensation
        Joint_deflection_compensation_flag = true;

        //4. dsp_ratio &  dsp_ratio_com tuning
        //5. Landing Controller
        LandingControl_flag = true;
        // --> Landing Z control  enabled
        // --> Landing XY control  disabled
        // --> Calc Landing Theshold

        //6. DSP_FZ_control
        DSP_FZ_Control_flag = true;

        //7. Pos ankle torque control
        Pos_Ankle_torque_control_flag = true;

    }

    void HB_set_step(vec3 _COM_ini, quat _qPel, vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF, double _WST_ini_deg, double _t_step, double _N_step, double _step_stride, int _RL_first){
        t_step = _t_step;
        des_step_t = t_step;
        N_step = _N_step;
        step_stride = _step_stride;
        R_or_L = _RL_first;
        dsp_ratio = 0.04;
        dsp_ratio_com = 0.2;
        des_dsp_ratio = dsp_ratio;
        FootUp_height = 0.10;

        No_of_cycle = 0;

        //zc = _COM_ini.z;

        qPel_ref = _qPel;
        rpy Pelv_rpy = rpy(qPel_ref);
        Pelv_roll_ref = Pelv_rpy.r; //rad
        Pelv_pitch_ref = Pelv_rpy.p; //rad
        // pelvis flywhell
        Pelv_roll_vel_ref = 0;
        Pelv_pitch_vel_ref = 0;


        COM_ref = _COM_ini;
        uCOM = _COM_ini;
        duCOM = vec3(); uCOM_old = vec3();

        COM_LIPM = _COM_ini;
        dCOM_LIPM = vec3();
        ddCOM_LIPM = vec3();

        COM_SA_ref = _COM_ini;
        dCOM_SA_ref = vec3();
        ddCOM_SA_ref = vec3();

        COM_SA_LIPM = _COM_ini;
        dCOM_SA_LIPM = vec3();
        ddCOM_SA_LIPM = vec3();

        CP_m_filtered = _COM_ini;
        p_ref[0] = vec3();
        pRF_ref = _pRF;
        pLF_ref = _pLF;
        qRF_ref = _qRF;
        qLF_ref = _qLF;

        des_pRF_local = pRF_ref;
        des_pLF_local = pLF_ref;


        rpy RF_rpy = rpy(_qRF);
        rpy LF_rpy = rpy(_qLF);
        rpy PEL_rpy = rpy(_qPel);

        // Foot yaw trajectory
        RF_yaw_quat_ref = quat(vec3(0,0,1), RF_rpy.y);
        LF_yaw_quat_ref = quat(vec3(0,0,1), LF_rpy.y);
        RF_yaw_quat_first = RF_yaw_quat_ref;
        LF_yaw_quat_first = LF_yaw_quat_ref;

        PEL_yaw_quat_ref = quat(vec3(0,0,1), PEL_rpy.y);

        RF_delta_yaw_d_dd = vec3();
        LF_delta_yaw_d_dd = vec3();
        RF_delta_yaw_d_dd_old = vec3();
        LF_delta_yaw_d_dd_old = vec3();

        ZMP_global = vec3(_COM_ini.x, _COM_ini.y, 0);

        COM_m_old = _COM_ini;
        COM_m_filtered = _COM_ini;
        dCOM_m_filtered = vec3();
        dCOM_m_diff_filtered = vec3();
        dCOM_m_imu_filtered = vec3();

        COM_m_comp_old = _COM_ini;
        COM_m_comp_filtered = _COM_ini;
        dCOM_m_comp_filtered = vec3();


        dCOM_ref_local_filtered = vec3();
        ddCOM_ref_local_filtered = vec3();

        dCOM_e_imu_local_filtered = vec3();
        COM_e_imu_local_filtered = _COM_ini;

        t_start = 0.15;
        t_prev = 1.5;
        t_stable = t_step;
        t_window = t_prev;
        t_now = 0;
        t_now_w = 0;
        t_total = 0;
        t_elapsed = 0;

        dT = t_start;
        k = 0;
        step_phase = 0;
        step_phase_change_flag = true;

        NL = (int)(t_prev*freq);

        RF_z_dz_ddz = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy = vec3(_pRF.y, 0, 0);
        RF_z_dz_ddz_old = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx_old = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy_old = vec3(_pRF.y, 0, 0);

        LF_z_dz_ddz = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy = vec3(_pLF.y, 0, 0);
        LF_z_dz_ddz_old = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx_old = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy_old = vec3(_pLF.y, 0, 0);

        RF_z_dz_ddz_w = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx_w = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy_w = vec3(_pRF.y, 0, 0);
        RF_z_dz_ddz_old_w = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx_old_w = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy_old_w = vec3(_pRF.y, 0, 0);

        LF_z_dz_ddz_w = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx_w = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy_w = vec3(_pLF.y, 0, 0);
        LF_z_dz_ddz_old_w = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx_old_w = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy_old_w = vec3(_pLF.y, 0, 0);

        COM_x_dx_ddx = vec3(_COM_ini.x, 0, 0);
        COM_y_dy_ddy = vec3(_COM_ini.y, 0, 0);
        COM_z_dz_ddz = vec3(_COM_ini.z, 0, 0);
        COM_x_dx_ddx_old = vec3(_COM_ini.x, 0, 0);
        COM_y_dy_ddy_old = vec3(_COM_ini.y, 0, 0);
        COM_z_dz_ddz_old = vec3(_COM_ini.z, 0, 0);

        Y_zmp_e_sum = 0;
        X_zmp_e_sum = 0;

        F_RF_filtered = vec3();
        F_LF_filtered = vec3();
        M_RF_filtered = vec3();
        M_LF_filtered = vec3();
        
        //Preview System matrix
        A = mat3(1, dt, dt*dt/2,
                 0,  1, dt,
                 0,  0,  1);
        B = vec3(dt*dt*dt/6, dt*dt/2, dt);
        C = vec3(1, 0, -zc/g);
        
        SDB.clear();
        SDB_original.clear();
        
        cZMP = vec3();  cZMP_proj = vec3(); cZMP_proj_filtered = vec3();
        dt_gain1 = 10;



        //Damping Controller Initialization
        Y_obs = vec3(0, 0, 0);
        X_obs = vec3(0, 0, 0);
        dY_obs = vec3(); dX_obs = vec3();
        ZMP_error_global = vec3();

        //CP Controller
        COM_con = vec3();
        dCOM_con = vec3();

        // SwingPhase Vibration control
        RHR_con_deg = 0; LHR_con_deg = 0;
        RHY_con_deg = 0; LHY_con_deg = 0;
        RHP_con_deg = 0; LHP_con_deg = 0;

        ACC_RF_Hfiltered = vec3();
        ACC_LF_Hfiltered = vec3();
        ACC_RF_filtered = vec3();
        ACC_LF_filtered = vec3();
        ACC_RF_old = vec3();
        ACC_LF_old = vec3();

        L_roll_obs = vec3();
        R_roll_obs = vec3();

        R_pitch_obs = vec3();
        dR_pitch_obs = vec3();

        X_RTorsoRoll = vec3();
        dX_RTorsoRoll = vec3();

        X_RTorsoYaw = vec3();
        dX_RTorsoYaw = vec3();

        X_LTorsoRoll = vec3();
        dX_LTorsoRoll = vec3();

        X_LTorsoYaw = vec3();
        dX_LTorsoYaw = vec3();

        //State Observer
        Ye_obs = vec3(); dYe_obs = vec3();
        Xe_obs = vec3(); dXe_obs = vec3();

        // Ankle Torque Control
        RF_del_angle = vec3();
        LF_del_angle = vec3();
        RF_del_angle_old = vec3();
        LF_del_angle_old = vec3();
        RF_del_pos = vec3();
        LF_del_pos = vec3();
        RF_del_quat = quat();
        LF_del_quat = quat();

        // joystick Walking
        Joystick_walk_flag = false;
        Joystick_off_signal = false;
        Joystick_on_signal = false;
        des_Velocity = vec3(); // (Vx, Vy, theta_dot) //from joystick input
        del_pos = vec3(); // (delX, delY, delTheta_deg) // should be calculated
        Joystick_off_count = 0;

        // Landing Controller
        RF_landing_flag = LF_landing_flag = true;
        _isLF_swingFirst = true;
        _isRF_swingFirst = true;
        Max_recovary_speed = 0.08;
        pLF_landing = _pLF;
        pRF_landing = _pRF;
        recovary_speed = 0;
        del_t = 0.00;
        LandingZ_des = 0;


        // Step Adjustor
        Y_zmp_e_sum_SA = 0;
        X_zmp_e_sum_SA = 0;

        // DSP Fz control
        z_ctrl = 0;
        z_ctrl_filtered = 0;
        dz_ctrl_filtered = 0;
        dsp_ctrl_gain = 0;
        dsp_tilt_gain = 0;

        // CoM z control
        z_com_ctrl = 0;
        dz_com_ctrl = 0;

        // Positino Ankle torque control
        LF_angle_ctrl = vec3();
        RF_angle_ctrl = vec3();

        // Foot Landing Angle Control
        RF_landing_angle = rpy(0,0,0);
        LF_landing_angle = rpy(0,0,0);

        // hip roll deflection compensation
        L_roll_compen_deg = 0;
        R_roll_compen_deg = 0;

        // waste yaw momentum
        WST_ref_deg = _WST_ini_deg;
        WST_rad = _WST_ini_deg*D2R;
        dWST_rad = 0;
        dWST_des_rad = 0;




        // fill in the Step_DATA buffer ( this should be modifed for joystick walking)
        for(int i = 0; i < N_step + 3 ;i++)
        {
            if(i == 0)
            {
                STEP_INFO SD_temp;
                vec3 zmp_ini = COM_ref;// + vec3(0, R_or_L*pelv_w/50, 0);

                SD_temp.Fpos = zmp_ini + vec3(0,+0.12,0); //for fast start
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_start;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }
            else if(i == 1)
            {
                STEP_INFO SD_temp;
                if(R_or_L == 1)
                {
                    SD_temp.Fpos = pLF_ref;
                    SD_temp.Fquat = qLF_ref;
                    SD_temp.yaw_rad = 0;
                    SD_temp.swingFoot = -R_or_L;
                    SD_temp.t = t_step;

                    SDB.push_back(SD_temp);
                    cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                    cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
                }
                else{
                    SD_temp.Fpos = pRF_ref;
                    SD_temp.Fquat = qRF_ref;
                    SD_temp.yaw_rad = 0;
                    SD_temp.swingFoot = -R_or_L;
                    SD_temp.t = t_step;

                    SDB.push_back(SD_temp);
                    cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                    cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
                }
            }
            else if(i == N_step + 2){ // last step
                STEP_INFO SD_temp;

//                SD_temp.Fpos = vec3(SDB.back().Fpos.x, SDB.back().Fpos.y + R_or_L*pelv_w/2*1.5, 0);
//                SD_temp.Fquat = quat();
//                SD_temp.yaw_rad = 0;
//                SD_temp.swingFoot = DSP;
//                SD_temp.t = t_stable;

//                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_stable;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }
            else{
                STEP_INFO SD_temp;
                if(i == 2 || i == N_step + 1){
                    SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride/2, R_or_L*pelv_w/2, 0);
                }

                SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride, R_or_L*pelv_w/2, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = -R_or_L;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }

            R_or_L *= -1;
        }


    }

    void Singlelog_set_step(vec3 _COM_ini, quat _qPel, vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF, double _WST_ini_deg, double _t_step, double _N_step, double _step_stride, int _RL_first)
    {
        t_step = _t_step;
        des_step_t = t_step;
        N_step = _N_step;
        step_stride = _step_stride;
        R_or_L = _RL_first;

        dsp_ratio = 0.04;
        dsp_ratio_com = 0.2;
        des_dsp_ratio = dsp_ratio;
        FootUp_height = 0.10;
        No_of_cycle = 0;

        qPel_ref = _qPel;
        rpy Pelv_rpy = rpy(qPel_ref);
        Pelv_roll_ref = Pelv_rpy.r; //rad
        Pelv_pitch_ref = Pelv_rpy.p; //rad

        // pelvis flywhell
        Pelv_roll_vel_ref = 0;
        Pelv_pitch_vel_ref = 0;

        COM_ref = _COM_ini;
        uCOM = _COM_ini;
        duCOM = vec3(); uCOM_old = vec3();

        COM_LIPM = _COM_ini;
        dCOM_LIPM = vec3();
        ddCOM_LIPM = vec3();

        COM_SA_ref = _COM_ini;
        dCOM_SA_ref = vec3();
        ddCOM_SA_ref = vec3();

        COM_SA_LIPM = _COM_ini;
        dCOM_SA_LIPM = vec3();
        ddCOM_SA_LIPM = vec3();

        CP_m_filtered = _COM_ini;
        p_ref[0] = vec3();
        pRF_ref = _pRF;
        pLF_ref = _pLF;
        qRF_ref = _qRF;
        qLF_ref = _qLF;

        des_pRF_local = pRF_ref;
        des_pLF_local = pLF_ref;

        rpy RF_rpy = rpy(_qRF);
        rpy LF_rpy = rpy(_qLF);
        rpy PEL_rpy = rpy(_qPel);

        // Foot yaw trajectory
        RF_yaw_quat_ref = quat(vec3(0,0,1), RF_rpy.y);
        LF_yaw_quat_ref = quat(vec3(0,0,1), LF_rpy.y);
        RF_yaw_quat_first = RF_yaw_quat_ref;
        LF_yaw_quat_first = LF_yaw_quat_ref;

        PEL_yaw_quat_ref = quat(vec3(0,0,1), PEL_rpy.y);

        RF_delta_yaw_d_dd = vec3();
        LF_delta_yaw_d_dd = vec3();
        RF_delta_yaw_d_dd_old = vec3();
        LF_delta_yaw_d_dd_old = vec3();

        ZMP_global = vec3(_COM_ini.x, _COM_ini.y, 0);

        COM_m_old = _COM_ini;
        COM_m_filtered = _COM_ini;
        dCOM_m_filtered = vec3();
        dCOM_m_diff_filtered = vec3();
        dCOM_m_imu_filtered = vec3();

        COM_m_comp_old = _COM_ini;
        COM_m_comp_filtered = _COM_ini;
        dCOM_m_comp_filtered = vec3();


        dCOM_ref_local_filtered = vec3();
        ddCOM_ref_local_filtered = vec3();

        dCOM_e_imu_local_filtered = vec3();
        COM_e_imu_local_filtered = _COM_ini;

        t_start = 0.15;
        t_prev = 1.5;
        t_stable = t_step;
        t_window = t_prev;
        t_now = 0;
        t_now_w = 0;
        t_total = 0;
        t_elapsed = 0;

        dT = t_start;
        k = 0;
        step_phase = 0;
        step_phase_change_flag = true;

        NL = (int)(t_prev*freq);

        RF_z_dz_ddz = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy = vec3(_pRF.y, 0, 0);
        RF_z_dz_ddz_old = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx_old = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy_old = vec3(_pRF.y, 0, 0);

        LF_z_dz_ddz = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy = vec3(_pLF.y, 0, 0);
        LF_z_dz_ddz_old = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx_old = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy_old = vec3(_pLF.y, 0, 0);

        RF_z_dz_ddz_w = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx_w = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy_w = vec3(_pRF.y, 0, 0);
        RF_z_dz_ddz_old_w = vec3(_pRF.z, 0, 0);
        RF_x_dx_ddx_old_w = vec3(_pRF.x, 0, 0);
        RF_y_dy_ddy_old_w = vec3(_pRF.y, 0, 0);

        LF_z_dz_ddz_w = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx_w = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy_w = vec3(_pLF.y, 0, 0);
        LF_z_dz_ddz_old_w = vec3(_pLF.z, 0, 0);
        LF_x_dx_ddx_old_w = vec3(_pLF.x, 0, 0);
        LF_y_dy_ddy_old_w = vec3(_pLF.y, 0, 0);

        COM_x_dx_ddx = vec3(_COM_ini.x, 0, 0);
        COM_y_dy_ddy = vec3(_COM_ini.y, 0, 0);
        COM_z_dz_ddz = vec3(_COM_ini.z, 0, 0);
        COM_x_dx_ddx_old = vec3(_COM_ini.x, 0, 0);
        COM_y_dy_ddy_old = vec3(_COM_ini.y, 0, 0);
        COM_z_dz_ddz_old = vec3(_COM_ini.z, 0, 0);

        Y_zmp_e_sum = 0;
        X_zmp_e_sum = 0;

        F_RF_filtered = vec3();
        F_LF_filtered = vec3();
        M_RF_filtered = vec3();
        M_LF_filtered = vec3();

        //Preview System matrix
        A = mat3(1, dt, dt*dt/2,
                 0,  1, dt,
                 0,  0,  1);
        B = vec3(dt*dt*dt/6, dt*dt/2, dt);
        C = vec3(1, 0, -zc/g);

        SDB.clear();
        SDB_original.clear();

        cZMP = vec3();  cZMP_proj = vec3(); cZMP_proj_filtered = vec3();
        dt_gain1 = 10;



        //Damping Controller Initialization
        Y_obs = vec3(0, 0, 0);
        X_obs = vec3(0, 0, 0);
        dY_obs = vec3(); dX_obs = vec3();
        ZMP_error_global = vec3();

        //CP Controller
        COM_con = vec3();
        dCOM_con = vec3();

        // SwingPhase Vibration control
        RHR_con_deg = 0; LHR_con_deg = 0;
        RHY_con_deg = 0; LHY_con_deg = 0;
        RHP_con_deg = 0; LHP_con_deg = 0;

        ACC_RF_Hfiltered = vec3();
        ACC_LF_Hfiltered = vec3();
        ACC_RF_filtered = vec3();
        ACC_LF_filtered = vec3();
        ACC_RF_old = vec3();
        ACC_LF_old = vec3();

        L_roll_obs = vec3();
        R_roll_obs = vec3();

        R_pitch_obs = vec3();
        dR_pitch_obs = vec3();

        X_RTorsoRoll = vec3();
        dX_RTorsoRoll = vec3();

        X_RTorsoYaw = vec3();
        dX_RTorsoYaw = vec3();

        X_LTorsoRoll = vec3();
        dX_LTorsoRoll = vec3();

        X_LTorsoYaw = vec3();
        dX_LTorsoYaw = vec3();

        //State Observer
        Ye_obs = vec3(); dYe_obs = vec3();
        Xe_obs = vec3(); dXe_obs = vec3();

        // Ankle Torque Control
        RF_del_angle = vec3();
        LF_del_angle = vec3();
        RF_del_angle_old = vec3();
        LF_del_angle_old = vec3();
        RF_del_pos = vec3();
        LF_del_pos = vec3();
        RF_del_quat = quat();
        LF_del_quat = quat();

        // joystick Walking
        Joystick_walk_flag = false;
        Joystick_off_signal = false;
        Joystick_on_signal = false;
        des_Velocity = vec3(); // (Vx, Vy, theta_dot) //from joystick input
        del_pos = vec3(); // (delX, delY, delTheta_deg) // should be calculated
        Joystick_off_count = 0;

        // Landing Controller
        RF_landing_flag = LF_landing_flag = true;
        _isLF_swingFirst = true;
        _isRF_swingFirst = true;
        Max_recovary_speed = 0.08;
        pLF_landing = _pLF;
        pRF_landing = _pRF;
        recovary_speed = 0;
        del_t = 0.00;
        LandingZ_des = 0;


        // Step Adjustor
        Y_zmp_e_sum_SA = 0;
        X_zmp_e_sum_SA = 0;

        // DSP Fz control
        z_ctrl = 0;
        z_ctrl_filtered = 0;
        dz_ctrl_filtered = 0;
        dsp_ctrl_gain = 0;
        dsp_tilt_gain = 0;

        // CoM z control
        z_com_ctrl = 0;
        dz_com_ctrl = 0;

        // Positino Ankle torque control
        LF_angle_ctrl = vec3();
        RF_angle_ctrl = vec3();

        // Foot Landing Angle Control
        RF_landing_angle = rpy(0,0,0);
        LF_landing_angle = rpy(0,0,0);

        // hip roll deflection compensation
        L_roll_compen_deg = 0;
        R_roll_compen_deg = 0;

        // waste yaw momentum
        WST_ref_deg = _WST_ini_deg;
        WST_rad = _WST_ini_deg*D2R;
        dWST_rad = 0;
        dWST_des_rad = 0;




        // fill in the Step_DATA buffer ( this should be modifed for joystick walking)
        for(int i = 0; i < N_step + 3 ;i++){
            if(i == 0){
                STEP_INFO SD_temp;
                vec3 zmp_ini = COM_ref;// + vec3(0, R_or_L*pelv_w/50, 0);

                SD_temp.Fpos = zmp_ini + vec3(0,+0.12,0); //for fast start
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_start;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }
            else if(i == 1){
                STEP_INFO SD_temp;
                if(R_or_L == 1){
                    SD_temp.Fpos = pLF_ref;
                    SD_temp.Fquat = qLF_ref;
                    SD_temp.yaw_rad = 0;
                    SD_temp.swingFoot = -R_or_L;
                    SD_temp.t = t_step;

                    SDB.push_back(SD_temp);
                    cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                    cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
                }
                else{
                    SD_temp.Fpos = pRF_ref;
                    SD_temp.Fquat = qRF_ref;
                    SD_temp.yaw_rad = 0;
                    SD_temp.swingFoot = -R_or_L;
                    SD_temp.t = t_step;

                    SDB.push_back(SD_temp);
                    cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                    cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
                }
            }
            else if(i == N_step + 2){ // last step
                STEP_INFO SD_temp;

//                SD_temp.Fpos = vec3(SDB.back().Fpos.x, SDB.back().Fpos.y + R_or_L*pelv_w/2*1.5, 0);
//                SD_temp.Fquat = quat();
//                SD_temp.yaw_rad = 0;
//                SD_temp.swingFoot = DSP;
//                SD_temp.t = t_stable;

//                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_stable;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);

                SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = DSP;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }
            else{
                STEP_INFO SD_temp;
                if(i == 2 || i == N_step + 1){
                    SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride/2, R_or_L*pelv_w/2, 0);
                }else
                {
                    SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride, R_or_L*pelv_w/2, 0);

                }

                SD_temp.Fquat = quat();
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = -R_or_L;
                SD_temp.t = t_step;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }

            R_or_L *= -1;
        }


    }

    int PreviewGainLoad(double _zc){
        int No_of_col = 0;
        int No_of_row = 0;

        ifstream file;
        file.open("../PreviewGain_Qr_high.txt",ios::in);

        if(file.is_open()){
            int i = 0;
            while(1){
                char oneLine[20000];
                file.getline(oneLine,sizeof(oneLine));

                if(file.eof() == true) break;

                char *Data;

                Data = strtok(oneLine,"\t");

                int j = 0;
                //cout<<"gains["<<i<<"] : "<<Data;
                while(Data != NULL){
                    PreviewGains[i][j] = atof(Data);

                    Data = strtok(NULL,"\t");
                    //cout<<PreviewGains[i][j];
                    j++;
                    No_of_col = j;
                }
                //cout<<endl;
                i++;
                No_of_row = i;
            }
        }
        else{
            cout<<"Preview gain file Load Fail !!"<<endl;
            return -1;
        }
        file.close();

        int index = (int)(_zc*1000);
        cout<<"index: "<<index<<endl;
//        cout<<"no of col: "<<No_of_col<<endl;
//        cout<<"no of row: "<<No_of_row<<endl;
//        cout<<"asf: "<<PreviewGains[690][1]<<endl;

        Gi = PreviewGains[index][1];

        for(int i=0; i<3 ; i++){
            Gx[i] = PreviewGains[index][i+2];
            //cout<<"i: "<<i<<" , gain: "<<PreviewGains[index][i+2]<<endl;
        }

        for(int i=0; i<No_of_col-5; i++){
            Gp[i] = PreviewGains[index][i+5];
//            cout<<"i: "<<i<<" , gain: "<<PreviewGains[index][i+5]<<endl;
        }

        cout<<"Preview gain File load done"<<endl;
        return 1;
    }





    // Functoions -----------------------------------------------------------------------------

};




#endif // HB_PREVIEWWALK

