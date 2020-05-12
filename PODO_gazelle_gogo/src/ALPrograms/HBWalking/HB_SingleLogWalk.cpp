#include "HB_SingleLogWalk.h"

int HB_SingleLogWalk::Preveiw_walking(){

    //// ----------------- step phase change-----------------------------
    if(step_phase_change_flag == true){
       //// -------------------------------joystick mode-----------------------------------------

        if(Joystick_walk_flag == true && step_phase > 2){
            // Step replanning every step phase change
            // ahead Next Two Step or more...(how much step should be planned before..?)
            // del_pos is confirmed (from joystick input)
            // del_pos : (X moving amount, Y moving amount, Yaw angle_deg)

            if(Joystick_off_signal == true){

                Joystick_walk_flag = false;
                Joystick_off_signal == false;
                cout<<"================================"<<endl;
                cout<<"      joystick flag off"<<endl;
                cout<<"================================"<<endl;

                N_step = step_phase + 2;

                STEP_INFO SD_last;

//              SD_last.Fpos = (SDB[step_phase].Fpos + SDB[step_phase + 1].Fpos)/2; // middle Foot
                SD_last.Fpos.x = (pRF_ref.x + pLF_ref.x)/2; // middle Foot
                SD_last.Fpos.y = (pRF_ref.y + pLF_ref.y)/2; // middle Foot
                SD_last.Fpos.z = 0;
                SD_last.Fquat = SDB[step_phase].Fquat;
                SD_last.t = t_stable;
                SD_last.swingFoot = DSP;

                for(int i = 2 ; i <= 6 ; i++){
                    SDB[step_phase + i] = SD_last;
                    SDB.push_back(SD_last);
                }

                del_pos = vec3();

            }
            else{
                N_step = step_phase + 5;  //extend last step

                if(SDB.size() <= step_phase + 7){
                    STEP_INFO SD_defalt;

                    SD_defalt.Fpos = vec3();
                    SD_defalt.Fquat = quat();
                    SD_defalt.t = 0.7;
                    SD_defalt.swingFoot = DSP;

                    SDB.push_back(SD_defalt);
                }
            }

            //// (step phase + i + 1) to (step_phase + i + 2)    i= 0,1
            for(int i=0 ; i<2 ; i++){
                STEP_INFO SD_next_step; // step for (step phase + i + 3)

                SD_next_step.t = des_step_t;//SDB[step_phase + i + 2].t; // should be changed to re-calced t_step
                dsp_ratio = des_dsp_ratio;

                if(SDB[step_phase + i + 0].swingFoot == RFoot){
                    // get 'step phase i + 2' stance leg coordination from Global coordinate
                    // --> get 'Global_R_stanceFoot' : g_R_f
                    rpy Euler_foot = rpy(SDB[step_phase + i + 0].Fquat);
                    mat3 g_R_stanceF = mat3(vec3(0,0,1),Euler_foot.y);

                    // get stancF_to_Next foot step (in 'step_phase + i + 2' foot coordinate )
                    double half_pelv_w = pelv_w/2;
                    half_pelv_w = pelv_w/2;// + 0.02*(fabs(del_pos.z)/20.0);
                    vec3 stancF_to_nextF_local = vec3(del_pos.x, del_pos.y - pelv_w/2, 0) + mat3(vec3(0,0,1),del_pos.z*D2R)*vec3(0,-half_pelv_w,0);

                    //stancF_to_NextF limiter
                    vec3 stancF_to_nextF_local_limited = StancF_to_NextF_limitation(RFoot, stancF_to_nextF_local);

                    vec3 stancF_to_nextF_global = g_R_stanceF*stancF_to_nextF_local_limited;

                    quat del_Fquat_local = quat(vec3(0,0,1),del_pos.z*D2R);

                    SD_next_step.swingFoot = LFoot; // i+3 step_phase swing foot (one step ahead)
                    SD_next_step.Fquat = SDB[step_phase + i + 0].Fquat*del_Fquat_local;
                    SD_next_step.Fpos.x = SDB[step_phase + i + 0].Fpos.x + stancF_to_nextF_global.x;
                    SD_next_step.Fpos.y = SDB[step_phase + i + 0].Fpos.y + stancF_to_nextF_global.y;

//                    cout<<"------------------delx : "<<del_pos.x<<" dely: "<<del_pos.y<<endl;
//                    cout<<"------------------global(x,y): "<<stancF_to_nextF_global.x<<", "<<stancF_to_nextF_global.y<<endl;

//                    if(i == 1){
//                        cout<<"step_phase: "<<step_phase<<" SDB[step_phase + 2].t: "<<SD_next_step.t<<endl;
//                        cout<<"del_pos.x : "<<del_pos.x<<"   del_pos.y : "<<del_pos.y<<endl;
//                        cout<<"stancF_to_nextF_global.x : "<<stancF_to_nextF_global.x<<endl;
//                        cout<<"stancF_to_nextF_global.y : "<<stancF_to_nextF_global.y<<endl<<endl;
//                    }

                }
                else if(SDB[step_phase + i + 0].swingFoot == LFoot){
                    rpy Euler_foot = rpy(SDB[step_phase + i + 0].Fquat);
                    mat3 g_R_stanceF = mat3(vec3(0,0,1), Euler_foot.y);

                    // get stancF_to_Next foot step (in 'step_phase + i + 2' foot coordinate )
                    double half_pelv_w = pelv_w/2;
                    half_pelv_w = pelv_w/2;// + 0.02*(fabs(del_pos.z)/20.0);
                    vec3 stancF_to_nextF_local = vec3(del_pos.x, del_pos.y + pelv_w/2, 0) + mat3(vec3(0,0,1),del_pos.z*D2R)*vec3(0,+half_pelv_w,0);

                    //stancF_to_NextF limiter
                    vec3 stancF_to_nextF_local_limited = StancF_to_NextF_limitation(LFoot, stancF_to_nextF_local);

                    vec3 stancF_to_nextF_global = g_R_stanceF*stancF_to_nextF_local_limited;

                    quat del_Fquat_local = quat(vec3(0,0,1), del_pos.z*D2R);

                    SD_next_step.swingFoot = RFoot; // i+3 step_phase swing foot (one step ahead)
                    SD_next_step.Fquat = SDB[step_phase + i + 0].Fquat*del_Fquat_local;
                    SD_next_step.Fpos.x = SDB[step_phase + i + 0].Fpos.x + stancF_to_nextF_global.x;
                    SD_next_step.Fpos.y = SDB[step_phase + i + 0].Fpos.y + stancF_to_nextF_global.y;

//                    cout<<"----------------delx : "<<del_pos.x<<" dely: "<<del_pos.y<<endl;
//                    cout<<"----------------global(x,y): "<<stancF_to_nextF_global.x<<", "<<stancF_to_nextF_global.y<<endl;

//                    if(i == 1){
//                        cout<<"step_phase: "<<step_phase<<" SDB[step_phase + 2].t: "<<SD_next_step.t<<endl;
//                        cout<<"del_pos.x : "<<del_pos.x<<"   del_pos.y : "<<del_pos.y<<endl;
//                        cout<<"stancF_to_nextF_global.x : "<<stancF_to_nextF_global.x<<endl;
//                        cout<<"stancF_to_nextF_global.y : "<<stancF_to_nextF_global.y<<endl<<endl;
//                    }
                }

                //SDB[step_phase + i].t = SD_next_step.t;
                SDB[step_phase + i + 1].t = SD_next_step.t;
                SDB[step_phase + i + 1].swingFoot = SD_next_step.swingFoot;
                SDB[step_phase + i + 1].Fpos = SD_next_step.Fpos;
                SDB[step_phase + i + 1].Fquat = SD_next_step.Fquat;
            }

        }


        // Step phase initialize
        t_step = SDB[step_phase].t;
        T_nom = SDB[step_phase].t;
        cout<<"step phase: "<<step_phase<<"   t_step: "<<t_step<<endl;
        dT = t_step;
        t_now = 0;

        // step adjustor initialize
        new_T = t_step;
        new_T_filtered = t_step;
        del_u_f_filtered = vec3();
        del_b_f_filtered = vec3();
        del_u_f_modi = vec3();
        Omega_pitch_filtered = 0;
        Omega_roll_filtered = 0;

        new_T_Nf_filtered = SDB[step_phase + 1].t;
        del_u_Nf_filtered = vec3();
        del_b_Nf_filtered = vec3();

        CP_error_lf = vec3();
        CP_error_rf = vec3();

        Landing_Z_filetered = 0;
        Landing_delXY_filtered = vec3();


        SDB_original = SDB;

        //---------generate ZMP SA ref for Step Adjustment
        WindowFill_SA_ref_3rd();
        // syncronize with real ref to SA ref
        COM_y_dy_ddy_old_SA = COM_y_dy_ddy_old;
        COM_x_dx_ddx_old_SA = COM_x_dx_ddx_old;

//        for(int i=0; i<NL; i++){
//            WDSAVE[step_phase][0][i] =    WD_SA_ref[i].ZMP_ref.y;

//        }

        // Foot yaw Trajectory
        rpy RF_rpy_cur = rpy(qRF_ref);
        rpy LF_rpy_cur = rpy(qLF_ref);

        RF_yaw_quat_first = quat(vec3(0,0,1), RF_rpy_cur.y);
        LF_yaw_quat_first = quat(vec3(0,0,1), LF_rpy_cur.y);

        if(SDB[step_phase].swingFoot == RFoot){
            RF_yaw_delta_quat = inverse_HB(RF_yaw_quat_ref)*SDB[step_phase + 1].Fquat;

            rpy RF_delta_quat_rpy = rpy(RF_yaw_delta_quat);

            RF_delta_yaw_goal_rad = RF_delta_quat_rpy.y; // RF delta yaw for current step_phase
            LF_delta_yaw_goal_rad = 0;
        }
        else if(SDB[step_phase].swingFoot == LFoot){
            LF_yaw_delta_quat = inverse_HB(LF_yaw_quat_ref)*SDB[step_phase + 1].Fquat;

            rpy LF_delta_quat_rpy = rpy(LF_yaw_delta_quat);

            LF_delta_yaw_goal_rad = LF_delta_quat_rpy.y; // LF delta yaw for current step_phase
            RF_delta_yaw_goal_rad = 0;
        }
        else{
            RF_delta_yaw_goal_rad = 0;
            LF_delta_yaw_goal_rad = 0;
        }


        step_phase_change_flag = false;

        cout<<"current phase changed: "<<step_phase<<"  t_step: "<<t_step<<endl;
    }
    if(step_phase >= N_step + 4){
        cout<<"Walk done!"<<endl;
        return -1;
    }

    //// ------------------Walking Stop when fell down----------------------------
    if(fabs(G_R_g_pitroll_rpy.r) > 14*D2R || fabs(G_R_g_pitroll_rpy.p) > 14*D2R ){
        cout<<"fall down!"<<endl;
        return -1;
    }

    //// ------------------p_ref_offset Control ----------------------------------

    if(ZMP_offset_controller_flag == true){
        double Alpha_offset_con = 1./(1. + 2*3.14*dt*2.5);
    //    tilt_angle_y_filtered = Alpha_offset_con*tilt_angle_y_filtered + (1.0 - Alpha_offset_con)*RS.IMUangle.y*R2D;
        p_ref_con_error_filtered = Alpha_offset_con*p_ref_con_error_filtered + (1.0 - Alpha_offset_con)*(COM_m_filtered.y + dCOM_m_diff_filtered.y/w) - (COM_SA_LIPM.y + dCOM_SA_LIPM.y/w);


        p_ref_offset_y = -0.0004*p_ref_con_error_filtered;
    }
    else{
        p_ref_offset_y = 0.0;
    }

    //cout<<"p_ref_offset y : "<<p_ref_offset_y<<endl;


    ////------------------- Fill in the Window buffer----------------------
    WindowFill();
    //WindowFill_3rd();
    //WindowFill_3rd_heel2toe();
    //WindowFill_3rd_heel2toe_3rd();


    ////----------------- get COM SA ref -------------------------------
    // fill in zmp ref buffer
    t_now_index = (int)((t_now + dt/2)*freq);
    //cout<<"t_now_index : "<<t_now_index<<endl;

    // if t_step become longer than original T, WD_SA_ref should be extended
    int t_extention_index = 0;
    if(T_nom < t_step){
        t_extention_index = (int)((t_step - T_nom + dt/2)*freq);
    }

    // middle index : t_now_index for middle time
    int middle_index = (int)((T_nom/2 + dt/2)*freq);

    for(int i=0; i < middle_index ; i++){
        WD_SA_ref_extended[i].ZMP_ref.x = WD_SA_ref[i].ZMP_ref.x;
        WD_SA_ref_extended[i].ZMP_ref.y = WD_SA_ref[i].ZMP_ref.y;
    }

    for(int i=middle_index; i < middle_index + t_extention_index ; i++){
        WD_SA_ref_extended[i].ZMP_ref.x = WD_SA_ref[middle_index].ZMP_ref.x;
        WD_SA_ref_extended[i].ZMP_ref.y = WD_SA_ref[middle_index].ZMP_ref.y;
    }

    for(int i=middle_index + t_extention_index; i < NL + t_now_index + t_extention_index ; i++){
        WD_SA_ref_extended[i].ZMP_ref.x = WD_SA_ref[i - t_extention_index].ZMP_ref.x;
        WD_SA_ref_extended[i].ZMP_ref.y = WD_SA_ref[i - t_extention_index].ZMP_ref.y;
    }


    for(int i=0; i<NL+1 ; i++){
        p_ref_SA[i].x = WD_SA_ref_extended[i + t_now_index].ZMP_ref.x;
        p_ref_SA[i].y = WD_SA_ref_extended[i + t_now_index].ZMP_ref.y;
    }

    // calc COM_SA_ref
    double temp_x1_SA = 0;
    double temp_x2_SA = 0;
    double temp_y1_SA = 0;
    double temp_y2_SA = 0;

    for(int i=0; i<3; i++){
        temp_y1_SA += Gx[i]*COM_y_dy_ddy_old_SA[i];
        temp_x1_SA += Gx[i]*COM_x_dx_ddx_old_SA[i];
    }
    for(int i=0; i<NL; i++){
        temp_y2_SA += p_ref_SA[i+1].y*Gp[i];
        temp_x2_SA += p_ref_SA[i+1].x*Gp[i];
    }

    // Y direction
    double u_y_SA = -Gi*Y_zmp_e_sum_SA - temp_y1_SA - temp_y2_SA;

    COM_y_dy_ddy_SA = A*COM_y_dy_ddy_old_SA + u_y_SA*B;
    p_out_SA.y = dot(C,COM_y_dy_ddy_old_SA);

    Y_zmp_e_sum_SA += p_out_SA.y - p_ref_SA[0].y;

    // X direction
    double u_x_SA = -Gi*X_zmp_e_sum_SA - temp_x1_SA - temp_x2_SA;

    COM_x_dx_ddx_SA = A*COM_x_dx_ddx_old_SA + u_x_SA*B;
    p_out_SA.x = dot(C,COM_x_dx_ddx_old_SA);

    X_zmp_e_sum_SA += p_out_SA.x - p_ref_SA[0].x;

    COM_SA_LIPM = vec3(COM_x_dx_ddx_SA[0], COM_y_dy_ddy_SA[0], zc);
    dCOM_SA_LIPM = vec3(COM_x_dx_ddx_SA[1], COM_y_dy_ddy_SA[1], 0);
    ddCOM_SA_LIPM = vec3(COM_x_dx_ddx_SA[2], COM_y_dy_ddy_SA[2], 0);

    COM_y_dy_ddy_old_SA = COM_y_dy_ddy_SA;
    COM_x_dx_ddx_old_SA = COM_x_dx_ddx_SA;


    ////----------------- get COM ref ---------------------------------
    // fill in zmp ref buffer
    for(int i=0; i<NL+1 ; i++){
//        p_ref[i].x = WD[i].ZMP_3m.x;
//        p_ref[i].y = WD[i].ZMP_3m.y;
        p_ref[i].x = WD[i].ZMP_ref.x;
        p_ref[i].y = WD[i].ZMP_ref.y;

        vec3 p_ref_for_COM_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[i]);

        p_ref_for_COM_local.y = p_ref_for_COM_local.y - 0.000;//p_ref_offset_y;

        p_ref_for_COM[i] = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref,p_ref_for_COM_local);

    }


//    for(int i=0; i<NL; i++){
//        WDSAVE[k][0][i] =    WD[i].ZMP_ref.y;
//        WDSAVE[k][1][i] =    dT;
//        WDSAVE[k][2][i] =    step_phase;

//    }


    // calc COM ref
    double temp_x1 = 0;
    double temp_x2 = 0;
    double temp_y1 = 0;
    double temp_y2 = 0;

    for(int i=0; i<3; i++){
        temp_y1 += Gx[i]*COM_y_dy_ddy_old[i];
        temp_x1 += Gx[i]*COM_x_dx_ddx_old[i];
    }
    for(int i=0; i<NL; i++){
        temp_y2 += p_ref_for_COM[i+1].y*Gp[i];
        temp_x2 += p_ref_for_COM[i+1].x*Gp[i];
    }

    // Y direction
    double u_y = -Gi*Y_zmp_e_sum - temp_y1 - temp_y2;

    COM_y_dy_ddy = A*COM_y_dy_ddy_old + u_y*B;
    p_out.y = dot(C,COM_y_dy_ddy_old);

    Y_zmp_e_sum += p_out.y - p_ref_for_COM[0].y;

    // X direction
    double u_x = -Gi*X_zmp_e_sum - temp_x1 - temp_x2;

    COM_x_dx_ddx = A*COM_x_dx_ddx_old + u_x*B;
    p_out.x = dot(C,COM_x_dx_ddx_old);

    X_zmp_e_sum += p_out.x - p_ref_for_COM[0].x;

    COM_LIPM = vec3(COM_x_dx_ddx[0], COM_y_dy_ddy[0], zc);
    dCOM_LIPM = vec3(COM_x_dx_ddx[1], COM_y_dy_ddy[1], 0);
    ddCOM_LIPM = vec3(COM_x_dx_ddx[2], COM_y_dy_ddy[2], 0);
    //cout<<"ZMP_refy: "<<p_ref[0].y<<"  COM y: "<<COM_ref.y<<endl;
    //CP_ref = COM_ref + dCOM_ref/w;

    COM_y_dy_ddy_old = COM_y_dy_ddy;
    COM_x_dx_ddx_old = COM_x_dx_ddx;

    //// ---------------------Calc COM ref Local ----------------------------------------------
    COM_ref_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_LIPM);
    dCOM_ref_local = global2local_vec(qRF_ref, qLF_ref, dCOM_LIPM);
    ddCOM_ref_local = global2local_vec(qRF_ref, qLF_ref, ddCOM_LIPM);

//    COM_SA_ref_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_SA_LIPM);
//    dCOM_SA_ref_local = global2local_vec(qRF_ref, qLF_ref, dCOM_SA_LIPM);
//    ddCOM_SA_ref_local = global2local_vec(qRF_ref, qLF_ref, ddCOM_SA_LIPM);

    //// -------------------- Sway scaling ----------------------------------------------------

//    vec3 p_ref_local_sway_con = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]);

//    double Alpha_sway_con = 1./(1. + 2*3.14*dt*2.5);
//    sway_con = Alpha_sway_con*sway_con + (1.0 - Alpha_sway_con)*HB_sign(p_ref_local_sway_con.y)*RS.IMUangle.y*R2D;

//    //cout<<"p_ref_offset y huhuhu : "<<tilt_angle_y_filtered<<endl;



    double scaling_factor = 1.00;// + 0.1*sway_con;
    COM_ref_local.y = COM_ref_local.y*scaling_factor;
    dCOM_ref_local.y = dCOM_ref_local.y*scaling_factor;
    ddCOM_ref_local.y = ddCOM_ref_local.y*scaling_factor;

    COM_ref = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_ref_local);
    dCOM_ref = local2global_vec(qRF_ref, qLF_ref, dCOM_ref_local);
    ddCOM_ref = local2global_vec(qRF_ref, qLF_ref, ddCOM_ref_local);

    CP_ref = COM_ref + dCOM_ref/w;

//    COM_SA_ref_local.y = COM_SA_ref_local.y*scaling_factor;
//    dCOM_SA_ref_local.y = dCOM_SA_ref_local.y*scaling_factor;
//    ddCOM_SA_ref_local.y = ddCOM_SA_ref_local.y*scaling_factor;

//    COM_SA_ref = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_SA_ref_local);
//    dCOM_SA_ref = local2global_vec(qRF_ref, qLF_ref, dCOM_SA_ref_local);
//    ddCOM_SA_ref = local2global_vec(qRF_ref, qLF_ref, ddCOM_SA_ref_local);

    COM_SA_ref = COM_SA_LIPM;
    dCOM_SA_ref = dCOM_SA_LIPM;
    ddCOM_SA_ref = ddCOM_SA_LIPM;


    //// --------------------- Get cZMP----------------------------------------------
    // CPT Controller ( get cZMP )
//    cZMP = 1/(1 - exp(w*dt*dt_gain1))*CP_ref - exp(w*dt*dt_gain1)/(1 - exp(w*dt*dt_gain1))*CP_m;// - CP_ref + p_ref[0];

    //cZMP from DCM Tracking Controller
    double dT_cZMP = t_step - t_now;
    if(dT_cZMP < 0.3) dT_cZMP = 0.3;
//    cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*((COM_m_filtered + dCOM_m_diff_filtered/w) - (COM_LIPM + dCOM_LIPM/w));
//    cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*((COM_m_filtered + dCOM_m_diff_filtered/w) - (COM_ref + COM_ref/w));
//    cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*CP_e_imu;

    double k_gain = 2;


    
    int dt_gain = 30;
//    cZMP_TC = 1/(1 - exp(w*dt*dt_gain))*(COM_LIPM + dCOM_LIPM/w) - exp(w*dt*dt_gain)/(1 - exp(w*dt*dt_gain))*(COM_m_filtered + dCOM_m_diff_filtered/w);


    //cZMP = p_ref[0] + (1 + k_gain/w)*CP_error_global;


//    vec3 CP_error_local = COM_e_imu_local + 1.0*dCOM_e_imu_local/w;
//    vec3 p_ref_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]);

//    double k_gain_y = 2;
//    double k_gain_x = 3.2;
//    cZMP_local_scaled.y = (p_ref_local.y + (1 + k_gain_y/w)*CP_error_local.y)*0.9;
//    cZMP_local_scaled.x = (p_ref_local.x + (1 + k_gain_x/w)*CP_error_local.x)*1;


//    cZMP = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, cZMP_local_scaled);

//    //cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*CP_error_global;
//    cZMP_proj = zmpProjectionToSP_offset(cZMP, pRF_ref, pLF_ref, qRF_ref, qLF_ref, RS.F_RF, RS.F_LF, +0.00, 0.00);





    // --------cZMP for dsp fz control-------
    // Using only imu error
//    cZMP_dsp_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]) + (1 + 3.0/w)*(COM_e_imu_local + 1.0*dCOM_e_imu_local/w);// + 2.67*COM_e_imu_local + 0.34*dCOM_e_imu_local;
//    cZMP_dsp_global = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, cZMP_dsp_local);



//    Using damping con + imu error
    cZMP_dsp_global = p_ref[0] + (1 + 4.0/w)*((COM_m_filtered + dCOM_m_diff_filtered/w) - (COM_LIPM + dCOM_LIPM/w));

    cZMP_dsp_global_proj = zmpProjectionToSP_offset(cZMP_dsp_global, pRF_ref, pLF_ref, qRF_ref, qLF_ref, 100, 100, -0.002, -0.002);

    cZMP_dsp_local_proj = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, cZMP_dsp_global_proj);  //-> not using

    // -------cZNP for torque control--------
    // Using only imu error
    vec3 temp_cZMP_TC_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]) + (1 + 11.0/w)*(COM_e_imu_local + 0.4*dCOM_e_imu_local/w);

    cZMP_TC_local.x = temp_cZMP_TC_local.x - 0.000;
    cZMP_TC_local.y = temp_cZMP_TC_local.y;

    cZMP_TC_global = local2global_point(qRF_ref, qLF_ref,pRF_ref, pLF_ref, cZMP_TC_local);

    cZMP_TC_proj = zmpProjectionToSP_offset(cZMP_TC_global, pRF_ref, pLF_ref, qRF_ref, qLF_ref, RS.F_RF, RS.F_LF, +0.00, +0.00);


    // Using damping con + imu error
//    cZMP_TC_global = p_ref[0] + (1 +4.0/w)*((COM_m_filtered + dCOM_m_diff_filtered/w) - (COM_LIPM + dCOM_LIPM/w));

//    vec3 temp_cZMP_TC_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref,cZMP_TC_global);

//    cZMP_TC_local.x = temp_cZMP_TC_local.x + 0.0;
//    cZMP_TC_local.y = temp_cZMP_TC_local.y;

//    vec3 temp_cZMP_TC_global = local2global_point(qRF_ref, qLF_ref,pRF_ref, pLF_ref,cZMP_TC_local);

//    cZMP_TC_proj = zmpProjectionToSP_offset(temp_cZMP_TC_global, pRF_ref, pLF_ref, qRF_ref, qLF_ref, RS.F_RF, RS.F_LF, +0.00, +0.00);



    //// -----------------------ddCOM controller   &   Input-shaping    &     Damping Controller -----------------------------------------
    double k_gain1 = 0;//0.0002;

    double Fz_both = 340;//F_RF_filtered.z + F_LF_filtered.z;

    ddCOM_con = k_gain1*Fz_both/zc*(ZMP_global - cZMP_dsp_global);
    ddCOM_con.z = 0;

    dCOM_con = dCOM_con + ddCOM_con*dt;
    dCOM_con.z = 0;

    COM_con = COM_con + dCOM_con*dt;
    COM_con.z = 0;
    COM_con.x = 0;

    //cout<<"ddCOM_con : "<<ddCOM_con.y<<" COM_con : "<<COM_con.y<<endl;

    DampingControl3();


    //uCOM = COM_ref + COM_damping_con;// + InputShaping() ;
    uCOM = COM_ref + COM_damping_con;// + COM_con;


    
    //// ---------------------------------------Step Adjustment -------------------------------------------------
    double dT_SA = dT;  // dT for clamping
    if(dT_SA < 10*dt) dT_SA = 10*dt;

    // Step Adjustment Flag enable & disable
    if(StepAdjustControl_flag == true){
        if(LandingControl_flag == true){ // landing control on : if one of foot is landed, disable StepAdjust
            if((t_now > 0.005 && t_now < t_step - 0.08) && (RF_landing_flag == false || LF_landing_flag == false)){
                SA_Enable_flag = true;
            }
            else{
                SA_Enable_flag = false;
            }
        }
        else{
            if(t_now > 0.005 && t_now < t_step - 0.08){ // landing control off : SA enables depends on time
                SA_Enable_flag = true;
            }
            else{
                SA_Enable_flag = false;
            }
        }
    }
    else{
        SA_Enable_flag = false;
    }

    userData->M2G.valveMode = 41;
    
    if(SDB[step_phase].swingFoot == RFoot && SA_Enable_flag == true){
        HB_StepAdjustor HBSA;
        bool Second_step_optimization_flag = false;

        ////-------------------- First Step Optimization ------------------------------

        // get Stance foot rotation matrix
        rpy Euler_angle_StanceFoot_rad = rpy(qLF_ref);
        
        mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
        
        mat3 f_R_g = inverse_HB(g_R_f);
        
        // CP_error left foot frame
//        CP_error_local_SA = 1.4*(1.0*COM_e_imu_local + 1.5*dCOM_e_imu_local/w);
//        CP_error_global_SA = local2global_vec(qRF_ref, qLF_ref, CP_error_local_SA);
//        CP_error_lf = f_R_g*CP_error_global_SA;

//        double Alpha_CPelf = 1./(1. + 2*3.14*dt*8);
//        CP_error_lf = Alpha_CPelf*CP_error_lf + (1.-Alpha_CPelf)*f_R_g*((COM_m_filtered - COM_SA_ref) + (dCOM_m_diff_filtered/w - dCOM_SA_ref/w));

        vec3 CP_error_local_for_x = f_R_g*((COM_m_filtered - COM_SA_ref) + (dCOM_m_diff_filtered/w - dCOM_SA_ref/w));
        vec3 CP_error_local_for_y = f_R_g*((COM_m_filtered - COM_SA_ref) + 0.95*(dCOM_m_diff_filtered/w - dCOM_SA_ref/w));

        double Alpha_CPelf_x = 1./(1. + 2*3.14*dt*8);
        CP_error_lf.x = Alpha_CPelf_x*CP_error_lf.x + (1.0 - Alpha_CPelf_x)*CP_error_local_for_x.x;

        double Alpha_CPelf_y = 1./(1. + 2*3.14*dt*5);
        CP_error_lf.y = Alpha_CPelf_y*CP_error_lf.y + (1.0 - Alpha_CPelf_y)*CP_error_local_for_y.y;


        //scaling
//        CP_error_lf.x = CP_error_lf.x*1.1;
//        CP_error_lf.y = CP_error_lf.y*0.9;

        // CP error clampin
        if(CP_error_lf.y > 0.13) CP_error_lf.y = 0.13;
        if(CP_error_lf.y < -0.13) CP_error_lf.y = -0.13;

//        if(CP_error_lf.x > 0.13) CP_error_lf.x = 0.13;
//        if(CP_error_lf.x < -0.13) CP_error_lf.x = -0.13;

        if(CP_error_lf.x > 0.25) CP_error_lf.x = 0.25;
        if(CP_error_lf.x < -0.25) CP_error_lf.x = -0.25;

        // CP_ref left foot frame
        vec3 StanceFoot2CPref_global = (COM_SA_ref + dCOM_SA_ref/w) - pLF_ref;
        CP_SA_ref_local = f_R_g*StanceFoot2CPref_global;
        
        // cZMP left foot frame
        vec3 cZMP_SA_lf_before_saturate;

        double dT_cZMP_x = T_nom-0.2;// - t_now;//t_step - t_now;
        if(dT_cZMP_x < 0.1) dT_cZMP_x = 0.1;
        cZMP_SA_lf_before_saturate.x = exp(w*dT_cZMP_x)/(exp(w*dT_cZMP_x) - 1)*CP_error_lf.x;

        double dT_cZMP_y = 0.55;// - t_now;//t_step - t_now;
        if(dT_cZMP_y < 0.1) dT_cZMP_y = 0.1;
        cZMP_SA_lf_before_saturate.y = exp(w*dT_cZMP_y)/(exp(w*dT_cZMP_y) - 1)*CP_error_lf.y;

        
        // cZMP saturation into left foot
        cZMP_SA_lf = zmpProjectionTo_StanceFoot(RFoot, cZMP_SA_lf_before_saturate,-0.16, -0.08);

        // swing Foot destination in left foot frame
        vec3 pSwingFoot;
        if(SDB_original[step_phase+1].swingFoot == DSP){ // for last step
            vec3 last_foot_offset = g_R_f*vec3(0, -pelv_w/2, 0);

            pSwingFoot = SDB_original[step_phase+1].Fpos + last_foot_offset;

            Second_step_optimization_flag = false;
        }
        else{
            pSwingFoot = SDB_original[step_phase+1].Fpos;
        }

        vec3 StanceFoot_to_SwingFoot_destination_global = pSwingFoot - pLF_ref;
        vec3 Stance_to_Swing_footFrame = f_R_g*StanceFoot_to_SwingFoot_destination_global;

        vec3 Stance_to_curruent_SwingFoot_footFrame = f_R_g*(pRF_ref - pLF_ref);


        // put the informations into QP solver  & Slove!       
        if(HipTorqueControl_flag == false){
            HBSA.setNums(5,2,6);
            HBSA.MakeEq(CP_error_lf, cZMP_SA_lf, t_now, w, CP_SA_ref_local, T_nom);
            HBSA.MakeIneq(RFoot,Stance_to_Swing_footFrame, Stance_to_curruent_SwingFoot_footFrame,
                          del_u_f_filtered, t_now, t_step, w, T_nom, t_step);
            HBSA.SetBehavior(w,T_nom, t_step, del_u_f_filtered);
            HBSA.MakeHF();
            HBSA.calcX();

            del_u_f.x = HBSA.X_array[0]; // in lf frame
            del_u_f.y = HBSA.X_array[1]; // in lf frame
            del_b_f.x = HBSA.X_array[2]; // in lf frame
            del_b_f.y = HBSA.X_array[3]; // in lf frame
            new_T = log(HBSA.X_array[4])/w;

        }
        else{
            HBSA.setNums(7,2,10);
            HBSA.MakeEq2(CP_error_lf, cZMP_SA_lf, t_now, w, CP_SA_ref_local, T_nom, t_step);
            HBSA.MakeIneq2(RFoot,Stance_to_Swing_footFrame, Stance_to_curruent_SwingFoot_footFrame, del_u_f_filtered, t_now, t_step, w, T_nom,
                           Pelv_pitch_ref, Pelv_pitch_vel_ref, Pelv_roll_ref, Pelv_roll_vel_ref);
            HBSA.SetBehavior2(w, T_nom,t_step, del_u_f_filtered, t_now,
                             Pelv_pitch_ref, Pelv_pitch_vel_ref, Pelv_roll_ref, Pelv_roll_vel_ref);
            HBSA.MakeHF();
            HBSA.calcX();

            del_u_f.x = HBSA.X_array[0]; // in lf frame
            del_u_f.y = HBSA.X_array[1]; // in lf frame
            del_b_f.x = HBSA.X_array[2]; // in lf frame
            del_b_f.y = HBSA.X_array[3]; // in lf frame
            new_T = log(HBSA.X_array[4])/w;
            Pelv_pitch_acc_ref = HBSA.X_array[5];
            Pelv_roll_acc_ref = HBSA.X_array[6];

            if(new_T > T_nom){
                double del_T = new_T - T_nom;
                new_T = T_nom - del_T;
            }
        }

        // del_u modification by foot size
        del_u_f_modi = del_u_modification_by_foot_size(del_u_f);

        // filtering of output variables
//        double fc = (10 - 5)/0.4*dT + 5;
        double alpha1x = 1/(1 + 2*PI*dt*20);
        del_u_f_filtered.x = alpha1x*del_u_f_filtered.x + (1 - alpha1x)*del_u_f_modi.x;
        double alpha1y = 0;// 1/(1 + 2*PI*dt*20);
        del_u_f_filtered.y = alpha1y*del_u_f_filtered.y + (1 - alpha1y)*del_u_f_modi.y;

        double alpha2 = 1/(1 + 2*PI*dt*10.0);
        del_b_f_filtered.x = alpha2*del_b_f_filtered.x + (1 - alpha2)*del_b_f.x;
        del_b_f_filtered.y = alpha2*del_b_f_filtered.y + (1 - alpha2)*del_b_f.y;

        double alpha3 = 1/(1 + 2*PI*dt*20.0);
        new_T_filtered = alpha3*new_T_filtered + (1 - alpha3)*new_T;

//        double alpha4 = 1/(1 + 2*PI*dt*6.0);
//        Omega_pitch_filtered = alpha4*Omega_pitch_filtered + (1 - alpha4)*Omega_pitch;
//        Omega_roll_filtered = alpha4*Omega_roll_filtered + (1 - alpha4)*Omega_roll;

        // Change step time
        t_step = new_T_filtered;
        SDB[step_phase].t = new_T_filtered;
        //cout<<"t_step: "<<t_step<<endl;

        // change modification varialbe to global frame
        del_u_g = g_R_f*del_u_f_filtered;
        del_b_g = g_R_f*del_b_f_filtered;


        // update footstep modification to future step data buffer (update SDB)
        for(int i = step_phase + 1; i < SDB.size(); i++){
            SDB[i].Fpos = SDB_original[i].Fpos + del_u_g;
        }

        ////-------------------- Second Step Optimization ------------------------------

        if(Second_step_optimization_flag == true){
            // get Next Stance foot rotation matrix
            rpy Next_StanceFoot_rpy = rpy(SDB[step_phase+1].Fquat);

            mat3 g_R_Nf = mat3(vec3(0,0,1), Next_StanceFoot_rpy.y);

            mat3 Nf_R_g = inverse_HB(g_R_Nf);

            // del_b of Next foot frame
            del_b0_Nf = Nf_R_g*del_b_g;

            // 2nd swing Foot destination in right foot frame
            vec3 pSwingFoot_2nd_global;
            if(SDB[step_phase + 2].swingFoot == DSP){ // for last step
                vec3 last_foot_offset = g_R_Nf*vec3(0, pelv_w/2, 0);

                pSwingFoot_2nd_global = SDB[step_phase + 2].Fpos + last_foot_offset;
            }
            else{
                pSwingFoot_2nd_global = SDB[step_phase + 2].Fpos;
            }

            vec3 Nf_to_2nd_swingfoot_destination_global = pSwingFoot_2nd_global - SDB[step_phase + 1].Fpos;
            vec3 Nf_to_2nd_swingfoot_destination_Nf = Nf_R_g*Nf_to_2nd_swingfoot_destination_global;

            // Original b0 of Next Foot frame
            vec3 vec_Nf2NNf_g = SDB_original[step_phase + 2].Fpos - SDB_original[step_phase + 1].Fpos; // delta Next foot to NextNext Foot global

            vec3 vec_Nf2NNf_Nf = Nf_R_g*vec_Nf2NNf_g;

            b0_Nf.y = vec_Nf2NNf_Nf.y/(exp(w*SDB_original[step_phase + 1].t) + 1); // Calc b0 with original step time (y direction)
            b0_Nf.x = vec_Nf2NNf_Nf.x/(exp(w*SDB_original[step_phase + 1].t) - 1); // Calc b0 with original step time (x direction)

            // put the informations into QP solver  & Slove!
            HBSA.setNums(5,2,5); // Variable, Eq Constraints, inEq Constraints
            HBSA.MakeEq_2nd(del_b0_Nf, b0_Nf, w, SDB_original[step_phase + 1].t);
            HBSA.MakeIneq_2nd(RFoot, Nf_to_2nd_swingfoot_destination_Nf, w);
            HBSA.SetBehavior_2nd(w, SDB_original[step_phase + 1].t);
            HBSA.MakeHF2();
            HBSA.calcX2();

            del_u_Nf.x = HBSA.X_array[0]; // in Nf frame
            del_u_Nf.y = HBSA.X_array[1]; // in Nf frame
            del_b_Nf.x = HBSA.X_array[2]; // in Nf frame
            del_b_Nf.y = HBSA.X_array[3]; // in Nf frame
            new_T_Nf = log(HBSA.X_array[4])/w;

            // filtering of output variables
            double alpha1 = 1/(1 + 2*PI*dt*12.0);
            del_u_Nf_filtered.x = alpha1*del_u_Nf_filtered.x + (1 - alpha1)*del_u_Nf.x;
            del_u_Nf_filtered.y = alpha1*del_u_Nf_filtered.y + (1 - alpha1)*del_u_Nf.y;

            alpha2 = 1/(1 + 2*PI*dt*12.0);
            del_b_Nf_filtered.x = alpha2*del_b_Nf_filtered.x + (1 - alpha2)*del_b_Nf.x;
            del_b_Nf_filtered.y = alpha2*del_b_Nf_filtered.y + (1 - alpha2)*del_b_Nf.y;

            alpha3 = 1/(1 + 2*PI*dt*12.0);
            new_T_Nf_filtered = alpha3*new_T_Nf_filtered + (1 - alpha3)*new_T_Nf;

            //Apply step time for next step
            SDB[step_phase + 1].t = new_T_Nf_filtered;

            // change modification varialbe to global frame
            del_u_Nf_g = g_R_Nf*del_u_Nf_filtered;

            // update footstep modification to future step data buffer (update SDB)
            for(int i = step_phase + 2; i < SDB.size(); i++){
                SDB[i].Fpos = SDB_original[i].Fpos + del_u_g + del_u_Nf_g;
            }
        }


    }

    userData->M2G.valveMode = 42;

    if(SDB[step_phase].swingFoot == LFoot && SA_Enable_flag == true){
        HB_StepAdjustor HBSA;
        bool Second_step_optimization_flag = false;

        ////-------------------- First Step Optimization ------------------------------

        // get Stance foot rotation matrix
        rpy Euler_angle_StanceFoot_rad = rpy(qRF_ref);

        mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);

        mat3 f_R_g = inverse_HB(g_R_f);

        // CP_error right foot frame
//        CP_error_local_SA = 1.4*(1.0*COM_e_imu_local + 1.5*dCOM_e_imu_local/w);
//        CP_error_global_SA = local2global_vec(qRF_ref, qLF_ref, CP_error_local_SA);
//        CP_error_rf = f_R_g*CP_error_global_SA;

//        double Alpha_CPelf = 1./(1. + 2*3.14*dt*8);
//        CP_error_rf = Alpha_CPelf*CP_error_rf + (1 - Alpha_CPelf)*f_R_g*(1.0*(COM_m_filtered - COM_SA_ref) + (dCOM_m_diff_filtered/w - dCOM_SA_ref/w));

        vec3 CP_error_local_for_x = f_R_g*((COM_m_filtered - COM_SA_ref) + (dCOM_m_diff_filtered/w - dCOM_SA_ref/w));
        vec3 CP_error_local_for_y = f_R_g*((COM_m_filtered - COM_SA_ref) + 0.95*(dCOM_m_diff_filtered/w - dCOM_SA_ref/w));

        double Alpha_CPelf_x = 1./(1. + 2*3.14*dt*8);
        CP_error_rf.x = Alpha_CPelf_x*CP_error_rf.x + (1.0 - Alpha_CPelf_x)*CP_error_local_for_x.x;

        double Alpha_CPelf_y = 1./(1. + 2*3.14*dt*5);
        CP_error_rf.y = Alpha_CPelf_y*CP_error_rf.y + (1.0 - Alpha_CPelf_y)*CP_error_local_for_y.y;


//        if(k%5 ==0){
//            cout<<"CP_error_rf.x : "<<Pelv_roll_ref*R2D<<endl;
//        }

        //scaling
//        CP_error_rf.x = CP_error_rf.x*1.1;
//        CP_error_rf.y = CP_error_rf.y*0.9;

        //CP error clamping
        if(CP_error_rf.y > 0.13) CP_error_rf.y = 0.13;
        if(CP_error_rf.y < -0.13) CP_error_rf.y = -0.13;

        if(CP_error_rf.x > 0.25) CP_error_rf.x = 0.25;
        if(CP_error_rf.x < -0.25) CP_error_rf.x = -0.25;

        // CP_ref right foot frame
        vec3 StanceFoot2CPref_global = (COM_SA_ref + dCOM_SA_ref/w) - pRF_ref;
        CP_SA_ref_local = f_R_g*StanceFoot2CPref_global;

        // cZMP left foot frame
        vec3 cZMP_SA_rf_before_saturate;

        double dT_cZMP_x = T_nom-0.2;// - t_now;//t_step - t_now;
        if(dT_cZMP_x < 0.1) dT_cZMP_x = 0.1;
        cZMP_SA_rf_before_saturate.x = exp(w*dT_cZMP_x)/(exp(w*dT_cZMP_x) - 1)*CP_error_rf.x;

        double dT_cZMP_y = 0.55;// - t_now;//t_step - t_now;
        if(dT_cZMP_y < 0.1) dT_cZMP_y = 0.1;
        cZMP_SA_rf_before_saturate.y = exp(w*dT_cZMP_y)/(exp(w*dT_cZMP_y) - 1)*CP_error_rf.y;


        // cZMP saturation into left foot
        cZMP_SA_rf = zmpProjectionTo_StanceFoot(LFoot, cZMP_SA_rf_before_saturate,-0.16, -0.08);

        // swing Foot destination in left foot frame
        vec3 pSwingFoot;
        if(SDB_original[step_phase+1].swingFoot == DSP){
            vec3 last_foot_offset = g_R_f*vec3(0, pelv_w/2, 0);

            pSwingFoot = SDB_original[step_phase+1].Fpos + last_foot_offset;

            Second_step_optimization_flag = false;
        }
        else{
            pSwingFoot = SDB_original[step_phase+1].Fpos;
        }

        vec3 StanceFoot_to_SwingFoot_destination_global = pSwingFoot - pRF_ref;
        vec3 Stance_to_Swing_footFrame = f_R_g*StanceFoot_to_SwingFoot_destination_global;

        vec3 Stance_to_curruent_SwingFoot_footFrame = f_R_g*(pLF_ref - pRF_ref);

        // put the informations into QP solver  & Slove!
        if(HipTorqueControl_flag == false){
            HBSA.setNums(5,2,6);
            HBSA.MakeEq(CP_error_rf, cZMP_SA_rf, t_now, w, CP_SA_ref_local, T_nom);
            HBSA.MakeIneq(LFoot,Stance_to_Swing_footFrame, Stance_to_curruent_SwingFoot_footFrame,
                          del_u_f_filtered, t_now, t_step, w, T_nom, t_step);
            HBSA.SetBehavior(w, T_nom, t_step, del_u_f_filtered);

            HBSA.MakeHF();
            HBSA.calcX();

            del_u_f.x = HBSA.X_array[0]; // in rf frame
            del_u_f.y = HBSA.X_array[1]; // in rf frame
            del_b_f.x = HBSA.X_array[2]; // in rf frame
            del_b_f.y = HBSA.X_array[3]; // in rf frame
            new_T = log(HBSA.X_array[4])/w;
        }
        else{
            HBSA.setNums(7,2,10);
            HBSA.MakeEq2(CP_error_rf, cZMP_SA_rf, t_now, w, CP_SA_ref_local, T_nom, t_step);
            HBSA.MakeIneq2(LFoot,Stance_to_Swing_footFrame, Stance_to_curruent_SwingFoot_footFrame, del_u_f_filtered, t_now, t_step, w, T_nom,
                           Pelv_pitch_ref, Pelv_pitch_vel_ref, Pelv_roll_ref, Pelv_roll_vel_ref);
            HBSA.SetBehavior2(w, T_nom, t_step, del_u_f_filtered, t_now,
                              Pelv_pitch_ref, Pelv_pitch_vel_ref, Pelv_roll_ref, Pelv_roll_vel_ref);
            HBSA.MakeHF();
            HBSA.calcX();

            del_u_f.x = HBSA.X_array[0]; // in rf frame
            del_u_f.y = HBSA.X_array[1]; // in rf frame
            del_b_f.x = HBSA.X_array[2]; // in rf frame
            del_b_f.y = HBSA.X_array[3]; // in rf frame
            new_T = log(HBSA.X_array[4])/w;
            Pelv_pitch_acc_ref = HBSA.X_array[5];
            Pelv_roll_acc_ref = HBSA.X_array[6];

            if(new_T > T_nom){
                double del_T = new_T - T_nom;
                new_T = T_nom - del_T;
            }
        }


        // del_u modification by foot size
        del_u_f_modi = del_u_modification_by_foot_size(del_u_f);

        // filtering of output variables
        double alpha1x = 1/(1 + 2*PI*dt*20);
        del_u_f_filtered.x = alpha1x*del_u_f_filtered.x + (1 - alpha1x)*del_u_f_modi.x;
        double alpha1y = 0;//1/(1 + 2*PI*dt*20);
        del_u_f_filtered.y = alpha1y*del_u_f_filtered.y + (1 - alpha1y)*del_u_f_modi.y;

        double alpha2 = 1/(1 + 2*PI*dt*10.0);
        del_b_f_filtered.x = alpha2*del_b_f_filtered.x + (1 - alpha2)*del_b_f.x;
        del_b_f_filtered.y = alpha2*del_b_f_filtered.y + (1 - alpha2)*del_b_f.y;

        double alpha3 = 1/(1 + 2*PI*dt*20.0);
        new_T_filtered = alpha3*new_T_filtered + (1 - alpha3)*new_T;

        // Change step time
        t_step = new_T_filtered;
        SDB[step_phase].t = new_T_filtered;
        //cout<<"t_step: "<<t_step<<endl;

        // change modification varialbe to global frame
        del_u_g = g_R_f*del_u_f_filtered;
        del_b_g = g_R_f*del_b_f_filtered;

        // update footstep modification to future step data buffer (update SDB)
        for(int i = step_phase + 1; i < SDB.size(); i++){
            SDB[i].Fpos = SDB_original[i].Fpos + del_u_g;
        }

        ////-------------------- Second Step Optimization ------------------------------

        if(Second_step_optimization_flag == true){
            // get Next Stance foot rotation matrix
            rpy Next_StanceFoot_rpy = rpy(SDB[step_phase+1].Fquat);

            mat3 g_R_Nf = mat3(vec3(0,0,1), Next_StanceFoot_rpy.y);

            mat3 Nf_R_g = inverse_HB(g_R_Nf);

            // del_b of Next foot frame
            del_b0_Nf = Nf_R_g*del_b_g;

            // 2nd swing Foot destination in right foot frame
            vec3 pSwingFoot_2nd_global;
            if(SDB[step_phase + 2].swingFoot == DSP){ // for last step
                vec3 last_foot_offset = g_R_Nf*vec3(0, -pelv_w/2, 0);

                pSwingFoot_2nd_global = SDB[step_phase + 2].Fpos + last_foot_offset;
            }
            else{
                pSwingFoot_2nd_global = SDB[step_phase + 2].Fpos;
            }

            vec3 Nf_to_2nd_swingfoot_destination_global = pSwingFoot_2nd_global - SDB[step_phase + 1].Fpos;
            vec3 Nf_to_2nd_swingfoot_destination_Nf = Nf_R_g*Nf_to_2nd_swingfoot_destination_global;

            // Original b0 of Next Foot frame
            vec3 vec_Nf2NNf_g = SDB_original[step_phase + 2].Fpos - SDB_original[step_phase + 1].Fpos; // delta Next foot to NextNext Foot global

            vec3 vec_Nf2NNf_Nf = Nf_R_g*vec_Nf2NNf_g;

            b0_Nf.y = vec_Nf2NNf_Nf.y/(exp(w*SDB_original[step_phase + 1].t) + 1); // Calc b0 with original step time (y direction)
            b0_Nf.x = vec_Nf2NNf_Nf.x/(exp(w*SDB_original[step_phase + 1].t) - 1); // Calc b0 with original step time (x direction)

            // put the informations into QP solver  & Slove!
            HBSA.setNums(5,2,5); // Variable, Eq Constraints, inEq Constraints
            HBSA.MakeEq_2nd(del_b0_Nf, b0_Nf, w, SDB_original[step_phase + 1].t);
            HBSA.MakeIneq_2nd(LFoot, Nf_to_2nd_swingfoot_destination_Nf, w);
            HBSA.SetBehavior_2nd(w, SDB_original[step_phase + 1].t);
            HBSA.MakeHF2();
            HBSA.calcX2();

            del_u_Nf.x = HBSA.X_array[0]; // in Nf frame
            del_u_Nf.y = HBSA.X_array[1]; // in Nf frame
            del_b_Nf.x = HBSA.X_array[2]; // in Nf frame
            del_b_Nf.y = HBSA.X_array[3]; // in Nf frame
            new_T_Nf = log(HBSA.X_array[4])/w;

            // filtering of output variables
            double alpha1 = 1/(1 + 2*PI*dt*12.0);
            del_u_Nf_filtered.x = alpha1*del_u_Nf_filtered.x + (1 - alpha1)*del_u_Nf.x;
            del_u_Nf_filtered.y = alpha1*del_u_Nf_filtered.y + (1 - alpha1)*del_u_Nf.y;

            alpha2 = 1/(1 + 2*PI*dt*12.0);
            del_b_Nf_filtered.x = alpha2*del_b_Nf_filtered.x + (1 - alpha2)*del_b_Nf.x;
            del_b_Nf_filtered.y = alpha2*del_b_Nf_filtered.y + (1 - alpha2)*del_b_Nf.y;

            alpha3 = 1/(1 + 2*PI*dt*12.0);
            new_T_Nf_filtered = alpha3*new_T_Nf_filtered + (1 - alpha3)*new_T_Nf;

            //Apply step time for next step
            SDB[step_phase + 1].t = new_T_Nf_filtered;

            // change modification varialbe to global frame
            del_u_Nf_g = g_R_Nf*del_u_Nf_filtered;

            // update footstep modification to future step data buffer (update SDB)
            for(int i = step_phase + 2; i < SDB.size(); i++){
                SDB[i].Fpos = SDB_original[i].Fpos + del_u_g + del_u_Nf_g;
            }
        }

    }
    if(SA_Enable_flag == false){
        del_u_f = vec3();
        del_b_f = vec3();
        del_u_f_modi = vec3();


        del_u_g = vec3();
        del_b_g = vec3();

        del_u_Nf = vec3();
        del_b_Nf = vec3();

        del_u_Nf_g = vec3();
    }

    userData->M2G.valveMode = 43;

    //// ----------------qPel Generation --------------------------------------


    if(SA_Enable_flag == false){
        double kp = 60;
        double kd = 60;

        Pelv_pitch_acc_ref = -kd*Pelv_pitch_vel_ref - kp*Pelv_pitch_ref;
        Pelv_roll_acc_ref = -kd*Pelv_roll_vel_ref - kp*Pelv_roll_ref;
    }


//    if(k%5 ==0){
//        cout<<"pelv_roll : "<<Pelv_roll_ref*R2D<<endl;
//        cout<<"pelv_pitch : "<<Pelv_pitch_ref*R2D<<endl;
//    }

    // Hardware Acc limit
    double Acc_lim_pitch_rad = 400.0*D2R;
    double Acc_lim_roll_rad = 400.0*D2R;

    // Hardware Vel limit
    double Vel_Max_pitch_rad = 150*D2R;
    double Vel_Max_roll_rad = 150*D2R;

    // Hardware Ang limit
    double Max_pitch_rad = 15*D2R;
    double min_pitch_rad = -30*D2R;

    double Max_roll_rad = 20*D2R;
    double min_roll_rad = -20*D2R;



    // Acc Max1 : hardware limit
    double Acc_Max1_pitch_rad = Acc_lim_pitch_rad;
    double Acc_Max1_roll_rad = Acc_lim_roll_rad;

    // Acc Max2 : limit from Max vel
    double Acc_Max2_pitch_rad = (Vel_Max_pitch_rad - Pelv_pitch_vel_ref)/dt;
    double Acc_Max2_roll_rad = (Vel_Max_roll_rad - Pelv_roll_vel_ref)/dt;

    // Acc Max3 : limit form Max angle
    double temp1_pitch = Max_pitch_rad - Pelv_pitch_ref;
    double temp1_roll = Max_roll_rad - Pelv_roll_ref;


    if(temp1_pitch < 0) temp1_pitch = 0;
    if(temp1_roll < 0) temp1_roll = 0;

    double Acc_Max3_pitch_rad = (sqrt(2*Acc_lim_pitch_rad*temp1_pitch) - Pelv_pitch_vel_ref)/dt;
    double Acc_Max3_roll_rad = (sqrt(2*Acc_lim_roll_rad*temp1_roll) - Pelv_roll_vel_ref)/dt;


    // Acc min1 : hardware limit
    double Acc_min1_pitch_rad = -Acc_lim_pitch_rad;
    double Acc_min1_roll_rad = -Acc_lim_roll_rad;

    // Acc min2 : limit from Max vel
    double Acc_min2_pitch_rad = (-Vel_Max_pitch_rad - Pelv_pitch_vel_ref)/dt;
    double Acc_min2_roll_rad = (-Vel_Max_roll_rad - Pelv_roll_vel_ref)/dt;

    // Acc min3 : limit from Max angle
    double temp2_pitch = Pelv_pitch_ref - min_pitch_rad;
    double temp2_roll = Pelv_roll_ref - min_roll_rad;

    if(temp2_pitch < 0) temp2_pitch = 0;
    if(temp2_roll < 0) temp2_roll = 0;

    double Acc_min3_pitch_rad = (-sqrt(2*Acc_lim_pitch_rad*temp2_pitch) - Pelv_pitch_vel_ref)/dt;
    double Acc_min3_roll_rad = (-sqrt(2*Acc_lim_roll_rad*temp2_roll) - Pelv_roll_vel_ref)/dt;


    // get ACC Max = min(AccMax1, AccMax2, AccMax3)
    double Acc_Max_pitch = Acc_Max1_pitch_rad;
    if(Acc_Max_pitch >= Acc_Max2_pitch_rad) Acc_Max_pitch = Acc_Max2_pitch_rad;
    if(Acc_Max_pitch >= Acc_Max3_pitch_rad) Acc_Max_pitch = Acc_Max3_pitch_rad;

    double Acc_Max_roll = Acc_Max1_roll_rad;
    if(Acc_Max_roll >= Acc_Max2_roll_rad) Acc_Max_roll = Acc_Max2_roll_rad;
    if(Acc_Max_roll >= Acc_Max3_roll_rad) Acc_Max_roll = Acc_Max3_roll_rad;

    // get Acc Min = max(AccMin1, AccMin2, AccMin3)
    double Acc_min_pitch = Acc_min1_pitch_rad;
    if(Acc_min_pitch <= Acc_min2_pitch_rad) Acc_min_pitch = Acc_min2_pitch_rad;
    if(Acc_min_pitch <= Acc_min3_pitch_rad) Acc_min_pitch = Acc_min3_pitch_rad;

    double Acc_min_roll = Acc_min1_roll_rad;
    if(Acc_min_roll <= Acc_min2_roll_rad) Acc_min_roll = Acc_min2_roll_rad;
    if(Acc_min_roll <= Acc_min3_roll_rad) Acc_min_roll = Acc_min3_roll_rad;

    if(Pelv_pitch_acc_ref > Acc_Max1_pitch_rad) Pelv_pitch_acc_ref = Acc_Max1_pitch_rad;

    if(Pelv_pitch_acc_ref > Acc_Max_pitch) Pelv_pitch_acc_ref = Acc_Max_pitch;
    if(Pelv_pitch_acc_ref < Acc_min_pitch) Pelv_pitch_acc_ref = Acc_min_pitch;
    if(Pelv_roll_acc_ref > Acc_Max_roll) Pelv_roll_acc_ref = Acc_Max_roll;
    if(Pelv_roll_acc_ref < Acc_min_roll) Pelv_roll_acc_ref = Acc_min_roll;


    Pelv_roll_vel_ref = Pelv_roll_vel_ref + Pelv_roll_acc_ref*dt;  // in rad
    Pelv_pitch_vel_ref = Pelv_pitch_vel_ref + Pelv_pitch_acc_ref*dt;

    Pelv_roll_ref = Pelv_roll_ref + Pelv_roll_vel_ref *dt; // in rad - Pelv_roll_ref/0.1
    Pelv_pitch_ref = Pelv_pitch_ref + Pelv_pitch_vel_ref*dt; //- Pelv_pitch_ref/0.1

    if(Pelv_roll_ref >= 25*D2R)  Pelv_roll_ref = 25*D2R;
    if(Pelv_roll_ref <= -25*D2R) Pelv_roll_ref = -25*D2R;

    if(Pelv_pitch_ref >= 30*D2R)  Pelv_pitch_ref = 30*D2R;
    if(Pelv_pitch_ref <= -30*D2R) Pelv_pitch_ref = -30*D2R;

    if(StepAdjustControl_flag == false || HipTorqueControl_flag == false){
        Pelv_roll_ref = 0;//
        Pelv_pitch_ref = 0;//
    }

    rpy Pelv_rpy = rpy(Pelv_roll_ref, Pelv_pitch_ref, get_Pel_yaw_from_qRF_qLF(qRF_ref, qLF_ref));
    qPel_ref = quat(Pelv_rpy);
    

    //// -------------------------------------- Foot trajectory ----------------------------
    FootUp_height = 0.05/0.4*t_step;
    //for rigid foot
//    double Landing_Threshold = 40; // N
    //for damping foot
    double Landing_Threshold = 25; // N




    // RFoot position
    if(SDB[step_phase].swingFoot == RFoot){
        // Landing Height & delta XY decision
        LandingZ_des = Calc_LandingHeight(RFoot, pLF_ref, SDB[step_phase+1].Fpos, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_delXY = Calc_Landing_delXY(RFoot, pLF_ref, SDB[step_phase+1].Fpos, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_Threshold = Calc_Landing_Threshold(RFoot, vec3());

        //cout<<"Landing delx: "<<Landing_delXY.x<<"  Landing dely: "<<Landing_delXY.y<<endl;

        // get orientation of stance foot
        vec3 pFoot;
        if(SDB[step_phase+1].swingFoot == DSP){
            rpy Euler_angle_StanceFoot_rad = rpy(qLF_ref);

            mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
            vec3 last_foot_offset = g_R_f*vec3(0, -pelv_w/2, 0);

            pFoot = SDB[step_phase+1].Fpos + last_foot_offset;
        }
        else{
            pFoot = SDB[step_phase+1].Fpos;
        }

        // destination of Rfoot
        des_pRF_local  = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pFoot);

        double t_half_dsp = T_nom*dsp_ratio*0.5;

        if(LandingControl_flag == true){
//            if(RS.F_RF.z <= 70 && t_now < t_step*0.5 && RF_landing_flag == true){
            if(t_now > t_half_dsp && t_now < t_half_dsp + 3*dt && RF_landing_flag == true){
                RF_landing_flag = false;
                LF_landing_flag = true;
            }
            if(F_RF_filtered.z > Landing_Threshold && t_now > T_nom*0.6 && t_now < T_nom - t_half_dsp*2 && RF_landing_flag == false){
                RF_landing_flag = true;

                // Z
                pRF_landing.z = RF_z_dz_ddz_old[0]; // Landing height
                RF_z_dz_ddz_old[1] = RF_z_dz_ddz_old[1]*0.7;

                //for impact reduce
                pLF_z_impact_new = LF_z_dz_ddz_old[0];

                // X
                pRF_landing.x = RF_x_dx_ddx_old[0];// + HB_sign(RF_x_dx_ddx_old[1])*0.002; // Landing x
                RF_x_dx_ddx_old[1] = RF_x_dx_ddx_old[1]*0.7;

                // y
                pRF_landing.y = RF_y_dy_ddy_old[0];// + HB_sign(RF_y_dy_ddy_old[1])*0.002; // Landing y
                RF_y_dy_ddy_old[1] = RF_y_dy_ddy_old[1]*0.7;

                RF_landing_time = t_step;


            }


            if(RF_landing_flag == false || t_now < T_nom*0.5){
                // Z direction
               RF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height + LandingZ_des, LandingZ_des-0.001);
               // X direction
               RF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, RF_x_dx_ddx_old, pFoot.x + Landing_delXY.x);
               // Y direction
               RF_y_dy_ddy = FootX_trajectory(t_step, t_now, dsp_ratio, RF_y_dy_ddy_old, pFoot.y + Landing_delXY.y);

            }
            if(RF_landing_flag == true && t_now > T_nom*0.4){
                // Z direction
                //RF_z_dz_ddz = FootZ_Landing_trajectory(RF_landing_time, t_now, RF_z_dz_ddz_old, pRF_landing.z);
                RF_z_dz_ddz = vec3(RF_z_dz_ddz_old[0]+RF_z_dz_ddz_old[1]*dt, RF_z_dz_ddz_old[1]*0.5, 0);
                // X direction
                //RF_x_dx_ddx = FootXY_Landing_trajectory(RF_landing_time, t_now, RF_x_dx_ddx_old, pRF_landing.x);
                RF_x_dx_ddx = vec3(RF_x_dx_ddx_old[0]+RF_x_dx_ddx_old[1]*dt, RF_x_dx_ddx_old[1]*0.5, 0);
                // Y direction
                //RF_y_dy_ddy = FootXY_Landing_trajectory(RF_landing_time, t_now, RF_y_dy_ddy_old, pRF_landing.y);
                RF_y_dy_ddy = vec3(RF_y_dy_ddy_old[0]+RF_y_dy_ddy_old[1]*dt, RF_y_dy_ddy_old[1]*0.5, 0);

            }


        }
        else{ // Landing Control Off
            // Z direction
            RF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height + LandingZ_des/2.0, LandingZ_des);


            // get yaw orientation of stance foot
            vec3 pFoot;
            if(SDB[step_phase+1].swingFoot == DSP){
                rpy Euler_angle_StanceFoot_rad = rpy(qLF_ref);

                mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
                vec3 last_foot_offset = g_R_f*vec3(0, -pelv_w/2, 0);

                pFoot = SDB[step_phase+1].Fpos + last_foot_offset;
            }
            else{
                pFoot = SDB[step_phase+1].Fpos;
            }

            // X direction
            RF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, RF_x_dx_ddx_old, pFoot.x);
            // Y direction
            RF_y_dy_ddy = FootX_trajectory(t_step, t_now, dsp_ratio, RF_y_dy_ddy_old, pFoot.y);
        }

        // pLF recovary to 0----------------------------------------------------------------------------------------
        _isLF_swingFirst = true;
        if(_isRF_swingFirst == true){
            _isRF_swingFirst = false;
            LF_z_dz_ddz_old = vec3(LF_z_dz_ddz[0], 0, 0);

            // Decide recovary speed
            recovary_speed = -LF_z_dz_ddz_old[0]/(T_nom*(1 - dsp_ratio) - del_t);

            if(recovary_speed > Max_recovary_speed){
                recovary_speed = Max_recovary_speed;
            }
            else if(recovary_speed < -Max_recovary_speed){
                recovary_speed = -Max_recovary_speed;
            }
        }


        LF_z_dz_ddz = FootZ_recovary_trapezoidal(t_step, t_now, dsp_ratio, del_t,recovary_speed, LF_z_dz_ddz_old);

        // --------------------------------------------------------------------------------------------------------------


        // RFoot Orientation
        RF_delta_yaw_d_dd = Foot_yaw_trajectory(t_step, t_now, dsp_ratio, RF_delta_yaw_d_dd_old, RF_delta_yaw_goal_rad);
        LF_delta_yaw_d_dd = vec3();

        RF_yaw_delta_quat = quat(vec3(0,0,1), RF_delta_yaw_d_dd[0]);
        RF_yaw_quat_ref = RF_yaw_quat_first*RF_yaw_delta_quat;

        //Swing Phase Vib Control
        if(Swing_leg_vibration_control_flag == true){
            if(t_now > T_nom*dsp_ratio/2 && t_now < T_nom*0.7){// && fabs(RS.IMUangle.x) < 10*D2R){
                RSwingLeg_Vib_Control(ACC_RF_filtered, false);
    //            RSwingLeg_Pitch_Vib_Control(ACC_RF_filtered, false);
                //RTorso_Roll_Vib_Control(RS.IMULocalW, false);
                RTorso_Yaw_Vib_Control(RS.IMULocalW, false);
            }
            else{
                RSwingLeg_Vib_Control(ACC_RF_filtered, true);
    //            RSwingLeg_Pitch_Vib_Control(ACC_RF_filtered, true);
                //RTorso_Roll_Vib_Control(RS.IMULocalW, true);
                RTorso_Yaw_Vib_Control(RS.IMULocalW, true);
    //            L_Vib_Control_init();
            }
        }


    }
    else if(SDB[step_phase].swingFoot == LFoot){
        // Landing Height decision
        LandingZ_des = Calc_LandingHeight(LFoot, SDB[step_phase+1].Fpos, pRF_ref, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_delXY = Calc_Landing_delXY(LFoot, SDB[step_phase+1].Fpos, pRF_ref, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_Threshold = Calc_Landing_Threshold(LFoot, vec3());

        // get yaw orientation of stance foot
        vec3 pFoot;
        if(SDB[step_phase+1].swingFoot == DSP){
            rpy Euler_angle_StanceFoot_rad = rpy(qRF_ref);

            mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
            vec3 last_foot_offset = g_R_f*vec3(0, pelv_w/2, 0);

            pFoot = SDB[step_phase+1].Fpos + last_foot_offset;
        }
        else{
            pFoot = SDB[step_phase+1].Fpos;
        }


        // destination of Lfoot
        des_pLF_local  = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pFoot);

        double t_half_dsp = T_nom*dsp_ratio*0.5;

        if(LandingControl_flag == true){
//            if(RS.F_LF.z <= 70 && t_now < t_step*0.5 && LF_landing_flag == true){
            if( t_now > t_half_dsp && t_now < t_half_dsp + 3*dt && LF_landing_flag == true){
                LF_landing_flag = false;
                RF_landing_flag = true;
            }
            if(F_LF_filtered.z > Landing_Threshold && t_now > T_nom*0.6 && t_now < T_nom - t_half_dsp*2 && LF_landing_flag == false){
                LF_landing_flag = true;

                // Z
                pLF_landing.z = LF_z_dz_ddz_old[0];
                LF_z_dz_ddz_old[1] = LF_z_dz_ddz_old[1]*0.70;

                //for impact reduce
                pRF_z_impact_new = RF_z_dz_ddz_old[0];

                // X
                pLF_landing.x = LF_x_dx_ddx_old[0];// + HB_sign(LF_x_dx_ddx_old[1])*0.002; // Landing x
                LF_x_dx_ddx_old[1] = LF_x_dx_ddx_old[1]*0.7;

                // y
                pLF_landing.y = LF_y_dy_ddy_old[0];// + HB_sign(LF_y_dy_ddy_old[1])*0.002; // Landing y
                LF_y_dy_ddy_old[1] = LF_y_dy_ddy_old[1]*0.7;

                LF_landing_time = t_step;


            }


            if(LF_landing_flag == false || t_now < T_nom*0.5){
                // Z direction
                LF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height + LandingZ_des, LandingZ_des-0.001);
                // X direction
                LF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, LF_x_dx_ddx_old, pFoot.x + Landing_delXY.x);
                // Y direction
                LF_y_dy_ddy = FootX_trajectory(t_step, t_now, dsp_ratio, LF_y_dy_ddy_old, pFoot.y + Landing_delXY.y);
            }
            if(LF_landing_flag == true && t_now > T_nom*0.4){
//                cout<<"========================================================================"<<endl;
//                cout<<"Lf landing x: "<<pLF_landing.x<<"  pLF_landing.y: "<<pLF_landing.y<<endl;
                // Z direction
                //LF_z_dz_ddz = FootZ_Landing_trajectory(LF_landing_time, t_now, LF_z_dz_ddz_old, pLF_landing.z);
                LF_z_dz_ddz = vec3(LF_z_dz_ddz_old[0]+LF_z_dz_ddz_old[1]*dt, LF_z_dz_ddz_old[1]*0.5, 0);
                // X direction
                //LF_x_dx_ddx = FootXY_Landing_trajectory(LF_landing_time, t_now, LF_x_dx_ddx_old, pLF_landing.x);
                LF_x_dx_ddx = vec3(LF_x_dx_ddx_old[0]+LF_x_dx_ddx_old[1]*dt, LF_x_dx_ddx_old[1]*0.5, 0);
                // Y direction
                //LF_y_dy_ddy = FootXY_Landing_trajectory(LF_landing_time, t_now, LF_y_dy_ddy_old, pLF_landing.y);
                LF_y_dy_ddy = vec3(LF_y_dy_ddy_old[0]+LF_y_dy_ddy_old[1]*dt, LF_y_dy_ddy_old[1]*0.5, 0);
            }




        }
        else{ // Landing Control Off
            // Z direction
            LF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height + LandingZ_des/2.0, LandingZ_des);

            // get orientation of stance foot
            vec3 pFoot;
            if(SDB[step_phase+1].swingFoot == DSP){
                rpy Euler_angle_StanceFoot_rad = rpy(qRF_ref);

                mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
                vec3 last_foot_offset = g_R_f*vec3(0, pelv_w/2, 0);

                pFoot = SDB[step_phase+1].Fpos + last_foot_offset;
            }
            else{
                pFoot = SDB[step_phase+1].Fpos;
            }

            // X direction
            LF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, LF_x_dx_ddx_old, pFoot.x);
            // Y direction
            LF_y_dy_ddy = FootX_trajectory(t_step, t_now, dsp_ratio, LF_y_dy_ddy_old, pFoot.y);
        }

        // pRF recovary to 0----------------------------------------------------------------------------------------
        _isRF_swingFirst = true;
        if(_isLF_swingFirst == true){
            _isLF_swingFirst = false;
            RF_z_dz_ddz_old = vec3(RF_z_dz_ddz[0], 0, 0);

            // Decide recovary speed
            recovary_speed = -RF_z_dz_ddz_old[0]/(T_nom*(1 - dsp_ratio) - del_t);

            cout<<"recovary speed: "<<recovary_speed<<endl;

            if(recovary_speed > Max_recovary_speed){
                recovary_speed = Max_recovary_speed;
            }
            else if(recovary_speed < -Max_recovary_speed){
                recovary_speed = -Max_recovary_speed;
            }
        }

        RF_z_dz_ddz = FootZ_recovary_trapezoidal(t_step, t_now, dsp_ratio, del_t, recovary_speed, RF_z_dz_ddz_old);






        //----------------------------------------------------------------------------------------------------------

        // LFoot Orientation
        RF_delta_yaw_d_dd = vec3();
        LF_delta_yaw_d_dd = Foot_yaw_trajectory(t_step, t_now, dsp_ratio, LF_delta_yaw_d_dd_old, LF_delta_yaw_goal_rad);

        LF_yaw_delta_quat = quat(vec3(0,0,1), LF_delta_yaw_d_dd[0]);
        LF_yaw_quat_ref = LF_yaw_quat_first*LF_yaw_delta_quat;

        //Swing Phase Vib Control
        if(Swing_leg_vibration_control_flag == true){
            if(t_now > T_nom*dsp_ratio/2 && t_now < T_nom*0.7){// && fabs(RS.IMUangle.x) < 10*D2R){
                LSwingLeg_Vib_Control(ACC_LF_filtered, false);
    //            LSwingLeg_Pitch_Vib_Control(ACC_LF_filtered, false);
                //LTorso_Roll_Vib_Control(RS.IMULocalW, false);
                LTorso_Yaw_Vib_Control(RS.IMULocalW, false);
            }
            else{
                LSwingLeg_Vib_Control(ACC_LF_filtered, true);
    //            LSwingLeg_Pitch_Vib_Control(ACC_LF_filtered, true);
                //LTorso_Roll_Vib_Control(RS.IMULocalW, true);
                LTorso_Yaw_Vib_Control(RS.IMULocalW, true);
    //            R_Vib_Control_init();
            }
        }


    }
    else{
        RF_z_dz_ddz_old = vec3(pRF_ref.z, 0,0);
        LF_z_dz_ddz_old = vec3(pLF_ref.z, 0,0);

        RF_x_dx_ddx_old = vec3(pRF_ref.x, 0,0);
        LF_x_dx_ddx_old = vec3(pLF_ref.x, 0,0);

        RF_y_dy_ddy_old = vec3(pRF_ref.y, 0,0);
        LF_y_dy_ddy_old = vec3(pLF_ref.y, 0,0);

        RSwingLeg_Vib_Control(ACC_RF_filtered, true);
        RSwingLeg_Pitch_Vib_Control(ACC_RF_filtered, true);
        RTorso_Roll_Vib_Control(RS.IMULocalW, true);
        RTorso_Yaw_Vib_Control(RS.IMULocalW, true);

        LSwingLeg_Vib_Control(ACC_LF_filtered, true);
        LSwingLeg_Pitch_Vib_Control(ACC_LF_filtered, true);
        LTorso_Roll_Vib_Control(RS.IMULocalW, true);
        LTorso_Yaw_Vib_Control(RS.IMULocalW, true);
    }

    RF_z_dz_ddz_old = RF_z_dz_ddz;
    LF_z_dz_ddz_old = LF_z_dz_ddz;

    RF_x_dx_ddx_old = RF_x_dx_ddx;
    LF_x_dx_ddx_old = LF_x_dx_ddx;

    RF_y_dy_ddy_old = RF_y_dy_ddy;
    LF_y_dy_ddy_old = LF_y_dy_ddy;

    RF_delta_yaw_d_dd_old = RF_delta_yaw_d_dd;
    LF_delta_yaw_d_dd_old = LF_delta_yaw_d_dd;




    //// --------------- Waste momentum compensation ----------------------
    // Calc angular momentum from foot motion
    double m_foot = 2; // kg
    vec3 COM2Rfoot_xy_plane = vec3(RF_x_dx_ddx[0], RF_y_dy_ddy[0], 0) - COM_LIPM;
    COM2Rfoot_xy_plane.z = 0;
    vec3 v_RF_xy_plane_about_COM = vec3(RF_x_dx_ddx[1], RF_y_dy_ddy[1], 0) - dCOM_LIPM;
    v_RF_xy_plane_about_COM.z = 0;

    vec3 COM2Lfoot_xy_plane = vec3(LF_x_dx_ddx[0], LF_y_dy_ddy[0], 0) - COM_LIPM;
    COM2Lfoot_xy_plane.z = 0;
    vec3 v_LF_xy_plane_about_COM = vec3(LF_x_dx_ddx[1], LF_y_dy_ddy[1], 0) - dCOM_LIPM;
    v_LF_xy_plane_about_COM.z = 0;

    vec3 Angular_momentum_RF = m_foot*cross(COM2Rfoot_xy_plane, v_RF_xy_plane_about_COM);
    vec3 Angular_momentum_LF = m_foot*cross(COM2Lfoot_xy_plane, v_LF_xy_plane_about_COM);

    Angular_momentum_about_COM = Angular_momentum_RF.z + Angular_momentum_LF.z;

    double torso_inertia = 0.15;//0.5;//0.18;

    dWST_des_rad = -Angular_momentum_about_COM/torso_inertia;

    dWST_rad = dWST_des_rad - WST_rad/0.25;

    WST_rad += dWST_rad*dt;

    if(WST_rad > 45.0*D2R) WST_rad = 45.0*D2R;
    if(WST_rad < -45.0*D2R) WST_rad = -45.0*D2R;

    WST_ref_deg = WST_rad*R2D;

    //cout<<"wst angle ref deg : "<<WST_ref_deg<<endl;

    //// ---------------- com z control ------------------------
    vec3 L = pLF_ref - pRF_ref;

    vec3 L_local = global2local_vec(qRF_ref, qLF_ref, L);

    double foot_width ;
    if(fabs(L_local.y) > pelv_w){
       foot_width = fabs(L_local.y) - pelv_w;
    }
    else{
        foot_width = 0;
    }


    double alpha_z = 1/(1 + 2*PI*dt*3);
    z_com_ctrl = alpha_z*z_com_ctrl + (1.0 - alpha_z)*foot_width*0.15;

    if(z_com_ctrl > 0.03) z_com_ctrl = 0.03;
    if(z_com_ctrl < -0.03) z_com_ctrl = -0.03;


    //// ---------------- Position Ankle Torque Control ------------------------


    AnkleTorque_ref = Ankle_Torque_from_cZMP(cZMP_TC_proj, ZMP_global, qRF_ref, qLF_ref, pRF_ref, pLF_ref, RS.F_RF, RS.F_LF);

    // calc 'qaut_ctrl' according to torque reference
    AnkleTorqueController_pos(AnkleTorque_ref[0], AnkleTorque_ref[1], AnkleTorque_ref[2], AnkleTorque_ref[3],
                                RS.F_RF, RS.F_LF, RS.M_RF, RS.M_LF);


    //// ---------------- DSP Fz control & total foot position reference--------------------------------------

    //  "Alpha_dsp" is from  "Ankle_Torque_from_cZMP()" function   Alpha_dsp = 0 ~ 1

    DSP_Fz_controller(SDB[step_phase].swingFoot, RS.F_RF.z, RS.F_LF.z); // this functino gives z_ctrl;
//    DSP_Fz_controller2(SDB[step_phase].swingFoot);


    if(DSP_FZ_Control_flag == true){
        pRF_ref.z = RF_z_dz_ddz[0] + 0.5*z_ctrl + z_com_ctrl;
        pLF_ref.z = LF_z_dz_ddz[0] - 0.5*z_ctrl + z_com_ctrl;
    }
    else{
        pRF_ref.z = RF_z_dz_ddz[0];
        pLF_ref.z = LF_z_dz_ddz[0];
    }

    pRF_ref.x = RF_x_dx_ddx[0];
    pLF_ref.x = LF_x_dx_ddx[0];

    pRF_ref.y = RF_y_dy_ddy[0];
    pLF_ref.y = LF_y_dy_ddy[0];


    //// -----------------Foot landing Angle Control ---------------------------
    // RF
//    double RF_landing_angle_gain_roll, RF_landing_angle_gain_pitch;
//    RF_landing_angle_gain_roll = 10*40/(40 + RS.F_RF.z);
//    RF_landing_angle_gain_pitch = 10*40/(40 + RS.F_RF.z);

    if(RF_landing_flag == false){
        dRF_landing_angle.r = 0.005*(-G_R_g_pitroll_rpy.r) - RF_landing_angle.r/0.1;
        dRF_landing_angle.p = 0.005*(-G_R_g_pitroll_rpy.p) - RF_landing_angle.p/0.1;
    }
    else{
        dRF_landing_angle.r = - RF_landing_angle.r/0.1;
        dRF_landing_angle.p = - RF_landing_angle.p/0.1;
    }

    RF_landing_angle.r += dRF_landing_angle.r*dt;
    RF_landing_angle.p += dRF_landing_angle.p*dt;

    if(RF_landing_angle.r >= 15*D2R) RF_landing_angle.r = 15*D2R;
    if(RF_landing_angle.r <= -15*D2R) RF_landing_angle.r = -15*D2R;
    if(RF_landing_angle.p >= 15*D2R) RF_landing_angle.p = 15*D2R;
    if(RF_landing_angle.p <= -15*D2R) RF_landing_angle.p = -15*D2R;

//    // LF
//    double LF_landing_angle_gain_roll, LF_landing_angle_gain_pitch;
//    LF_landing_angle_gain_roll = 12*50/(50 + RS.F_LF.z);
//    LF_landing_angle_gain_pitch = 12*50/(50 + RS.F_LF.z);

    if(LF_landing_flag == false){
        dLF_landing_angle.r = 0.005*(-G_R_g_pitroll_rpy.r) - LF_landing_angle.r/0.1;
        dLF_landing_angle.p = 0.005*(-G_R_g_pitroll_rpy.p) - LF_landing_angle.p/0.1;
    }
    else{
        dLF_landing_angle.r = - LF_landing_angle.r/0.1;
        dLF_landing_angle.p = - LF_landing_angle.p/0.1;
    }

    LF_landing_angle.r += dLF_landing_angle.r*dt;
    LF_landing_angle.p += dLF_landing_angle.p*dt;

    if(LF_landing_angle.r >= 15*D2R) LF_landing_angle.r = 15*D2R;
    if(LF_landing_angle.r <= -15*D2R) LF_landing_angle.r = -15*D2R;
    if(LF_landing_angle.p >= 15*D2R) LF_landing_angle.p = 15*D2R;
    if(LF_landing_angle.p <= -15*D2R) LF_landing_angle.p = -15*D2R;


    ////-----------------Total Ankle orientation -------------------------------

    if(Pos_Ankle_torque_control_flag == true && Landing_angle_control_flag == false){
        qRF_ref = RF_yaw_quat_ref*RF_quat_ctrl;
        qLF_ref = LF_yaw_quat_ref*LF_quat_ctrl;

    }
    else if(Pos_Ankle_torque_control_flag == false && Landing_angle_control_flag == true){
        qRF_ref = RF_yaw_quat_ref*quat(RF_landing_angle);
        qLF_ref = LF_yaw_quat_ref*quat(LF_landing_angle);

    }
    else if(Pos_Ankle_torque_control_flag == true && Landing_angle_control_flag == true){
        qRF_ref = RF_yaw_quat_ref*quat(RF_landing_angle)*RF_quat_ctrl;
        qLF_ref = LF_yaw_quat_ref*quat(LF_landing_angle)*LF_quat_ctrl;
        //cout<<"RF landing: "<<RF_landing_angle.p<<"LF landing: "<<LF_landing_angle.p<<endl;
    }
    else{
        qRF_ref = RF_yaw_quat_ref;
        qLF_ref = LF_yaw_quat_ref;
    }

    ////------------------- hip roll deflection compensation----------------------------

    if(Joint_deflection_compensation_flag == true){
        if(SDB[step_phase].swingFoot == RFoot){
            vec3 ZMP_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]);
            //vec3 COM_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_ref);

            // hip roll
            double alpha = 1/(1 + 2*PI*dt*3);
            L_roll_compen_deg = alpha*L_roll_compen_deg + (1-alpha)*HB_sign(ZMP_local.y)*0.1*13;
            //L_roll_compen_deg = alpha*L_roll_compen_deg + (1-alpha)*COM_local.y*20;

            R_roll_compen_deg = alpha*R_roll_compen_deg + (1-alpha)*0;

            // knee
            double basic_comp_deg = -0.0;
            L_knee_compen_deg = alpha*L_knee_compen_deg + (1-alpha)*(basic_comp_deg);// - fabs(ZMP_local.y)*3);

            R_knee_compen_deg = alpha*R_knee_compen_deg + (1-alpha)*(basic_comp_deg);

        }
        if(SDB[step_phase].swingFoot == LFoot){
            vec3 ZMP_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]);
            //vec3 COM_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_ref);

            // hip roll
            double alpha = 1/(1 + 2*PI*dt*3);
            R_roll_compen_deg = alpha*R_roll_compen_deg + (1-alpha)*HB_sign(ZMP_local.y)*0.1*13;
            //R_roll_compen_deg = alpha*R_roll_compen_deg + (1-alpha)*COM_local.y*20;

            L_roll_compen_deg = alpha*L_roll_compen_deg + (1-alpha)*0;

            // knee
            double basic_comp_deg = -0.0;
            L_knee_compen_deg = alpha*L_knee_compen_deg + (1-alpha)*(basic_comp_deg);

            R_knee_compen_deg = alpha*R_knee_compen_deg + (1-alpha)*(basic_comp_deg);// - fabs(ZMP_local.y)*3);

        }
        if(SDB[step_phase].swingFoot == DSP || SDB[step_phase+1].swingFoot == DSP){

            // hip roll
            double alpha = 1/(1 + 2*PI*dt*2);
            R_roll_compen_deg = alpha*R_roll_compen_deg + (1-alpha)*0;

            L_roll_compen_deg = alpha*L_roll_compen_deg + (1-alpha)*0;

            // knee
            L_knee_compen_deg = alpha*L_knee_compen_deg + (1-alpha)*0;

            R_knee_compen_deg = alpha*R_knee_compen_deg + (1-alpha)*0;
        }

        if(R_roll_compen_deg > 2.0) R_roll_compen_deg = 2.0;
        if(R_roll_compen_deg < -2.0) R_roll_compen_deg = -2.0;
        if(L_roll_compen_deg > 2.0) L_roll_compen_deg = 2.0;
        if(L_roll_compen_deg < -2.0) L_roll_compen_deg = -2.0;
    }





    //// --------------- time handling --------------------------------------
    t_now += dt;
    t_total += dt;
    dT = t_step - t_now;
    k++;

    if(dT <= dt + dt/2){
        step_phase_change_flag = true;
        step_phase++;
    }



}




////******************************************************************************************************
/// *************************** Functions ****************************************************************
/// ******************************************************************************************************
///



void HB_SingleLogWalk::MeasurementInput(RobotStates _RST){
    RS = _RST;

    // ZMP calculation
    ZMP_global = ZMP_calc_global(pRF_ref, qRF_ref, RS.F_RF, RS.M_RF, pLF_ref, qLF_ref, RS.F_LF, RS.M_LF);


    //// COM measure
    //COM_m = COM_measurment_by_quat(uCOM, ZMP_global, RS.IMUquat, quat());
    COM_m = COM_measurment(uCOM, zc, ZMP_global, RS.IMUangle, qPel_ref);
    //COM_m_comp = COM_measurment(uCOM, zc, ZMP_global, RS.IMUangle_comp, qPel_ref);

    double alpha1 = 1.0/(1.0 + 2.0*PI*dt*12.0);
    COM_m_filtered = alpha1*COM_m_filtered + (1.0 - alpha1)*COM_m;

    double alpha_comp = 1.0/(1.0 + 2.0*PI*dt*12.0);
    COM_m_comp_filtered = alpha_comp*COM_m_comp_filtered + (1.0 - alpha_comp)*COM_m_comp;

    //// dCOM measure by imu
    duCOM = (uCOM - uCOM_old)/dt;
    uCOM_old = uCOM;

    dCOM_m_imu = dCOM_measurement_by_quat(COM_m, duCOM, ZMP_global, RS.IMUquat, RS.IMULocalW);

    double alpha2 = 1.0/(1.0 + 2.0*PI*dt*10.0);
    dCOM_m_imu_filtered = alpha2*dCOM_m_imu_filtered + (1 - alpha2)*dCOM_m_imu;
    //cout<<"dCOMimu: "<<dCOM_m_imu_filtered.y<<endl;
    //COM_m = RS.CSP.pCOM;

    //// dCOM measure by differentiation
    dCOM_m_diff = (COM_m - COM_m_old)/dt;
    COM_m_old = COM_m;

    dCOM_m_comp = (COM_m_comp - COM_m_comp_old)/dt;
    COM_m_comp_old = COM_m_comp;

    double alpha3 = 1.0/(1.0 + 2.0*PI*dt*6.0);
    dCOM_m_diff_filtered = alpha3*dCOM_m_diff_filtered + (1.0 - alpha3)*dCOM_m_diff;

    double alpha_dcomp = 1.0/(1.0 + 2.0*PI*dt*8.0);
    dCOM_m_comp_filtered = alpha_dcomp*dCOM_m_comp_filtered + (1.0 - alpha_dcomp)*dCOM_m_comp;
    //cout<<"dCOMdif: "<<dCOM_m_diff_filtered.y<<endl;

    // State Observer
    //State_Observer();
    //CP_m = COM_ref + vec3(Xe_obs[0], Ye_obs[0],0) + (dCOM_ref + vec3(Xe_obs[1], Ye_obs[1],0))/w;


//    CP_m = RS.CSP.pCOM + RS.CSV.dpCOM/w;  CP_m.z = 0;

    //// COM local error measure by imu tilt
    double alpha_COMe_imu = 1/(1 + 2*PI*dt*20);
    COM_e_imu_local.x = alpha_COMe_imu*COM_e_imu_local.x + (1-alpha_COMe_imu)*G_R_g_pitroll_rpy.p*0.72;
    COM_e_imu_local.y = alpha_COMe_imu*COM_e_imu_local.y + (1-alpha_COMe_imu)*(-G_R_g_pitroll_rpy.r*0.72);
//    COM_e_imu_local.x = alpha_COMe_imu*COM_e_imu_local.x + (1-alpha_COMe_imu)*RS.IMUangle.y*0.72;
//    COM_e_imu_local.y = alpha_COMe_imu*COM_e_imu_local.y + (1-alpha_COMe_imu)*(-RS.IMUangle.x*0.72);
    COM_e_imu_local.z = 0;

    double alpha_dCOMe_imu = 1/(1 + 2*PI*dt*6);
    dCOM_e_imu_local.x = alpha_dCOMe_imu*dCOM_e_imu_local.x + (1-alpha_dCOMe_imu)*RS.IMULocalW.y*0.72;
    dCOM_e_imu_local.y = alpha_dCOMe_imu*dCOM_e_imu_local.y + (1-alpha_dCOMe_imu)*(-RS.IMULocalW.x*0.72);
    dCOM_e_imu_local.z = 0;

    //// CP measured
    CP_m = COM_m_filtered + dCOM_m_diff_filtered/w;  CP_m.z = 0;
    CP_m_comp = COM_m_comp + dCOM_m_comp/w;         CP_m_comp.z = 0;

    //CPm filtering
    double alpha_CP = 1/(1 + 2*PI*dt*15);
    //CP_m_filtered = CP_m_filtered*alpha_CP + CP_m*(1 - alpha_CP);
    //cout<<"dCOM: "<<RS.CSV.dpCOM.y<<endl;

    //FT Sensor Filtering
    double alpha_FT = 1/(1 + 2*PI*dt*30);
    F_RF_filtered = F_RF_filtered*alpha_FT + RS.F_RF*(1 - alpha_FT);
    F_LF_filtered = F_LF_filtered*alpha_FT + RS.F_LF*(1 - alpha_FT);
    M_RF_filtered = M_RF_filtered*alpha_FT + RS.M_RF*(1 - alpha_FT);
    M_LF_filtered = M_LF_filtered*alpha_FT + RS.M_LF*(1 - alpha_FT);

    ////FT ACC Filtering
    // HPF
    double alpha_FT_ACC = 1.0/(1.0 + 2.0*PI*dt*1.0);
    ACC_RF_Hfiltered = alpha_FT_ACC*(ACC_RF_Hfiltered + (RS.ACC_RF - ACC_RF_old));
    ACC_LF_Hfiltered = alpha_FT_ACC*(ACC_LF_Hfiltered + (RS.ACC_LF - ACC_LF_old));
    ACC_RF_old = RS.ACC_RF;
    ACC_LF_old = RS.ACC_LF;

    // LPF
    double alpha_FT_ACC2 = 0;//1.0/(1.0 + 2.0*PI*dt*20.0);
    ACC_RF_filtered = ACC_RF_filtered*alpha_FT_ACC2 + ACC_RF_Hfiltered*(1 - alpha_FT_ACC2);
    ACC_LF_filtered = ACC_LF_filtered*alpha_FT_ACC2 + ACC_LF_Hfiltered*(1 - alpha_FT_ACC2);

}

vec3 HB_SingleLogWalk::COM_measurment(vec3 _COM, double _zc, vec3 _ZMP_global, vec3 _IMUangle, quat _qPel){

    // get Imu rot matrix w.r.t global without yaw (only pitch and roll)
    rpy rpy_Pel = rpy(_qPel);

    mat3 G_R_g = mat3(vec3(0,0,1), rpy_Pel.y);
    mat3 g_R_G = inverse_HB(G_R_g);

    mat3 a;

    rpy imu_pitroll_local = rpy(_IMUangle.x, _IMUangle.y, 0);

    ////Real robot tilt : Rotation matrix between Real global frame(G) and WBIK frame(g) (without yaw)
    // WBIK qPel with out yaw


    mat3 rot_Pel_pitroll_local = mat3(rpy(rpy_Pel.r, rpy_Pel.p, 0));

    // Real tilt
    G_R_g_pitroll = mat3(imu_pitroll_local)*inverse_HB(rot_Pel_pitroll_local);

    G_R_g_pitroll_rpy = rpy(quat(G_R_g_pitroll));

//    if(k%50 == 0){
//        cout<<"G_R_g_pitroll.r : "<<G_R_g_pitroll_rpy.r*R2D<<endl;
//        cout<<"G_R_g_pitroll.p : "<<G_R_g_pitroll_rpy.p*R2D<<endl;
//    }


    // Measure Global COM position using FK and ZMP and Gyro
    _COM.z = _zc;
    _ZMP_global.z = 0;

    vec3 ZMP2COM_global = _COM - _ZMP_global;

    vec3 ZMP2COM_local = g_R_G*ZMP2COM_global;

    vec3 ZMP2COM_real_local = G_R_g_pitroll*ZMP2COM_local;

    vec3 ZMP2COM_real_global = G_R_g*ZMP2COM_real_local;

    vec3 COM_measure = _ZMP_global + ZMP2COM_real_global;

    return COM_measure;
}
