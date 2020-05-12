#include "GG_SingleLogWalk.h"

GG_SingleLogWalk::GG_SingleLogWalk()
{

}

int GG_SingleLogWalk::Preveiw_walking()
{

    //// ----------------- step phase change-----------------------------
    if(step_phase_change_flag == true)
    {//once call(init) every time step changes

        userData->step_phase = step_phase;
        //// -------------------------------joystick mode-----------------------------------------

         if(ROSWalk_flag == true && step_phase > 2)
         {
             // Step replanning every step phase change
             // ahead Next Two Step or more...(how much step should be planned before..?)

             if(ROSWalk_off_flag == true)
             {
                 ROSWalk_flag = false;
                 ROSWalk_off_flag == false;

                 cout<<"================================"<<endl;
                 cout<<"      ROS flag off"<<endl;
                 cout<<"================================"<<endl;

                 N_step = step_phase + 2;

                 STEP_INFO SD_last;

                 printf("step_phase = %d\n",step_phase);
               SD_last.Fpos = (SDB[step_phase].Fpos + SDB[step_phase + 1].Fpos)/2; // middle Foot
               printf("step_phase y = %f, +1 y = %f\n",SDB[step_phase].Fpos.y,SDB[step_phase + 1].Fpos.y);
               printf("step_phase x = %f, +1 x = %f\n",SDB[step_phase].Fpos.x,SDB[step_phase + 1].Fpos.x);
//                 SD_last.Fpos.x = (pRF_ref.x + pLF_ref.x)/2; // middle Foot
//                 SD_last.Fpos.y = (pRF_ref.y + pLF_ref.y)/2; // middle Foot
//                 SD_last.Fpos.z = 0;
                 SD_last.Fquat = SDB[step_phase].Fquat;
                 SD_last.t = t_stable;
                 SD_last.swingFoot = DSP;
                 SD_last.ros_step_phase = -1;

                 for(int i = 2 ; i <= 6 ; i++)
                 {
                     printf("%d changed to SD_last\n",step_phase + i);
                     printf("origin = %f, %f\n",SDB[step_phase + i].Fpos.x,SDB[step_phase + i].Fpos.y);
                     SDB[step_phase + i] = SD_last;
                     SDB.push_back(SD_last);
                 }
                 del_pos = vec3();
             }
             else
             {
                 printf("-------------%d th new step replanning--------------\n",step_phase);

                 N_step = step_phase + userData->ros_step_num;  //extend last step

                 if(SDB.size() <= step_phase + 7)
                 {
                     printf("**SDB size = %d, so add 7 DSP buffers\n", SDB.size());
                     STEP_INFO SD_default;

                     SD_default.Fpos = vec3();
                     SD_default.Fquat = quat();
                     SD_default.t = 0.7;
                     SD_default.swingFoot = DSP;
                     SD_default.ros_step_phase = -1;

                     SDB.push_back(SD_default);
                 }

                 //// (step phase + i + 1) to (step_phase + i + 2)    i= 0,1
                 if(userData->FLAG_receivedROS == ROS_RX_TRUE)//only when you get data from ros
                 {
                     STEP_INFO SD_next_step;
                     int overwrite_flag = false;
                     rosstep_l2g();

                     FILE_LOG(logSUCCESS) << "ROS command received";

                     for(int i=0; i<4; i++)
                     {
                         if(rosstep_global[i].lr_state == RFoot)//if next swing foot is left
                         {
                             //change next footstep using ros data
                             SD_next_step.t = ros_step_t;
                             dsp_ratio = des_dsp_ratio;

                             //stancF_to_NextF limiter
                             quat del_Fquat_local = quat(vec3(0,0,1),del_pos.z*D2R);
                             quat des_yaw = quat(vec3(0,0,1), rosstep_global[i].r*D2R);

                             SD_next_step.swingFoot = LFoot;

                             //load ros footstep
//                             SD_next_step.Fquat  = SDB[step_phase + i + 0].Fquat*del_Fquat_local;
                             SD_next_step.Fquat = des_yaw*del_Fquat_local;

                             SD_next_step.Fpos.x = rosstep_global[i].x;
                             SD_next_step.Fpos.y = rosstep_global[i].y;
                             SD_next_step.Fpos.z = 0.0;


                             SD_next_step.ros_step_phase = roswalk_first_phase + rosstep_global[i].step_phase;


                             double x_stride = SD_next_step.Fpos.x-SDB[SD_next_step.ros_step_phase - 1].Fpos.x;
                             double y_stride = SD_next_step.Fpos.y-SDB[SD_next_step.ros_step_phase - 1].Fpos.y;
                             double len_stride = sqrt(x_stride*x_stride + y_stride*y_stride);
                             printf("stride : x(%f), y(%f) SO,len = %f\n",x_stride, y_stride, len_stride);


                             //check next step safety
                             if(SD_next_step.swingFoot == SDB[SD_next_step.ros_step_phase - 1].swingFoot) //(L->L)
                             {
                                 FILE_LOG(logERROR) << "L->L";
                                 overwrite_flag = false;
                             }else if(len_stride > 0.46) //step stride > 0.35m
                             {
                                 FILE_LOG(logERROR) << "step stride bigger than 0.46m";
                                 overwrite_flag = false;
                             }else
                             {
                                 overwrite_flag = true;
                                 userData->FLAG_receivedROS = ROS_RX_EMPTY;
                             }
                         }else      //if next swing foot is right
                         {
                             //change next footstep using ros data
                             SD_next_step.t = ros_step_t;
                             dsp_ratio = des_dsp_ratio;

                             //stancF_to_NextF limiter
                             quat del_Fquat_local = quat(vec3(0,0,1),del_pos.z*D2R);
                             quat des_yaw = quat(vec3(0,0,1), rosstep_global[i].r*D2R);

                             SD_next_step.swingFoot = RFoot;

                             //load ros footstep
                             SD_next_step.Fquat  = SDB[step_phase + i + 0].Fquat*del_Fquat_local;
                             SD_next_step.Fquat = des_yaw*del_Fquat_local;

                             SD_next_step.Fpos.x = rosstep_global[i].x;
                             SD_next_step.Fpos.y = rosstep_global[i].y;
                             SD_next_step.Fpos.z = 0.0;

                             SD_next_step.ros_step_phase = roswalk_first_phase + rosstep_global[i].step_phase;

                             double x_stride = SD_next_step.Fpos.x-SDB[SD_next_step.ros_step_phase - 1].Fpos.x;
                             double y_stride = SD_next_step.Fpos.y-SDB[SD_next_step.ros_step_phase - 1].Fpos.y;
                             double len_stride = sqrt(x_stride*x_stride + y_stride*y_stride);
                             printf("stride : x(%f), y(%f) SO,len = %f\n",x_stride, y_stride, len_stride);

                             //check next step safety
                             if(SD_next_step.swingFoot == SDB[SD_next_step.ros_step_phase - 1].swingFoot) //(R->R)
                             {
                                 FILE_LOG(logERROR) << "R->R";
                                 overwrite_flag = false;
                             }else if(len_stride > 0.46) //step stride > 0.35m
                             {
                                 FILE_LOG(logERROR) << "step stride bigger than 0.46m";
                                 overwrite_flag = false;
                             }else
                             {
                                 overwrite_flag = true;
                                 userData->FLAG_receivedROS = ROS_RX_EMPTY;
                                 printf("receive empty overwrite\n");
                             }
                         }


                         //overwrite SDB to SD_next_step
                         if(overwrite_flag == true && (SD_next_step.ros_step_phase > step_phase))
                         {
                             FILE_LOG(logSUCCESS) << "SDB overwrited";
                             printf("SDB[%d] %f, %f     ->      %f, %f\n", SD_next_step.ros_step_phase,
                                        SDB[SD_next_step.ros_step_phase].Fpos.x, SDB[SD_next_step.ros_step_phase].Fpos.y,
                                        SD_next_step.Fpos.x, SD_next_step.Fpos.y);

                             SDB[SD_next_step.ros_step_phase].t = SD_next_step.t;
                             SDB[SD_next_step.ros_step_phase].swingFoot = SD_next_step.swingFoot;
                             SDB[SD_next_step.ros_step_phase].Fpos = SD_next_step.Fpos;
                             SDB[SD_next_step.ros_step_phase].Fquat = SD_next_step.Fquat;
                             SDB[SD_next_step.ros_step_phase].ros_step_phase = SD_next_step.ros_step_phase;

                             if(SD_next_step.ros_step_phase - step_phase == 1)
                                SDB[step_phase].ros_step_phase = SD_next_step.ros_step_phase - 1;

                         }else
                         {
                             FILE_LOG(logERROR) << "SDB don't changed";
                             printf("SDB[%d] %f, %f         (%f, %f)\n", SD_next_step.ros_step_phase,
                                        SDB[SD_next_step.ros_step_phase].Fpos.x, SDB[SD_next_step.ros_step_phase].Fpos.y,
                                        SD_next_step.Fpos.x, SD_next_step.Fpos.y);
                         }

                         if(SDB[step_phase + 1 + i].ros_step_phase == -1)
                         {
                             FILE_LOG(logINFO) << "next SDB changed";
                             printf("SDB[%d] %f, %f     ->      %f, %f\n", step_phase + 1 + i,
                                        SDB[step_phase + 1 + i].Fpos.x, SDB[step_phase + 1 + i].Fpos.y,
                                     SDB[step_phase - 1 + i].Fpos.x, SDB[step_phase - 1 + i].Fpos.y);

                             SDB[step_phase + 1 + i].t = SDB[step_phase - 1 + i].t;
                             SDB[step_phase + 1 + i].swingFoot = SDB[step_phase - 1 + i].swingFoot;
                             SDB[step_phase + 1 + i].Fpos = SDB[step_phase - 1 + i].Fpos;
                             SDB[step_phase + 1 + i].Fquat = SDB[step_phase - 1 + i].Fquat;
                             SDB[step_phase + 1 + i].ros_step_phase = -1;
                         }
                     }
                     printf("SDB[0,1,2] = %f, %f(%d), %f, %f(%d), %f, %f(%d)\n",
                                SDB[step_phase].Fpos.x,     SDB[step_phase].Fpos.y, SDB[step_phase].ros_step_phase,
                                SDB[step_phase+1].Fpos.x,   SDB[step_phase+1].Fpos.y, SDB[step_phase+1].ros_step_phase,
                                SDB[step_phase+2].Fpos.x,   SDB[step_phase+2].Fpos.y, SDB[step_phase+2].ros_step_phase);

                 }else
                 {
                     STEP_INFO SD_next_step;
                     FILE_LOG(logERROR) << "can't received ros command";

                     for(int i=0;i<4;i++)
                     {
                         if(SDB[step_phase + 1 + i].ros_step_phase == -1)
                         {
                             FILE_LOG(logINFO) << "next SDB changed";
                             printf("SDB[%d] %f, %f     ->      %f, %f\n", step_phase + 1 + i,
                                        SDB[step_phase + 1 + i].Fpos.x, SDB[step_phase + 1 + i].Fpos.y,
                                     SDB[step_phase - 1 + i].Fpos.x, SDB[step_phase - 1 + i].Fpos.y);

                             SDB[step_phase + 1 + i].t = SDB[step_phase - 1 + i].t;
                             SDB[step_phase + 1 + i].swingFoot = SDB[step_phase - 1 + i].swingFoot;
                             SDB[step_phase + 1 + i].Fpos = SDB[step_phase - 1 + i].Fpos;
                             SDB[step_phase + 1 + i].Fquat = SDB[step_phase - 1 + i].Fquat;
                             SDB[step_phase + 1 + i].ros_step_phase = -1;
                         }else
                         {
                             FILE_LOG(logINFO) << "next SDB don't changed";
                             printf("SDB[%d] %f, %f          (%f, %f)\n", step_phase + 1 + i,
                                        SDB[step_phase + 1 + i].Fpos.x, SDB[step_phase + 1 + i].Fpos.y,
                                     SDB[step_phase - 1 + i].Fpos.x, SDB[step_phase - 1 + i].Fpos.y);
                         }
                     }
                     printf("SDB[0,1,2] = %f(%d), %f(%d), %f(%d)\n",
                                SDB[step_phase].Fpos.x,     SDB[step_phase].ros_step_phase,
                                SDB[step_phase+1].Fpos.x,   SDB[step_phase+1].ros_step_phase,
                                SDB[step_phase+2].Fpos.x,   SDB[step_phase+2].ros_step_phase);
                 }

                 printf("-----------------------------------------------\n");
             }

         }

        // Step phase initialize
        t_step = SDB[step_phase].t;
        T_nom = SDB[step_phase].t;
        cout<<"step phase: "<<step_phase<<"   t_step: "<<t_step<<endl;
        double yaw_deg = rpy(SDB[step_phase].Fquat).y*R2D;
        printf("Current Foot Step = %f, %f, %f (%d)\n",SDB[step_phase].Fpos.x,SDB[step_phase].Fpos.y,yaw_deg,SDB[step_phase].swingFoot);
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

        // Foot yaw Trajectory
        rpy RF_rpy_cur = rpy(qRF_ref);
        rpy LF_rpy_cur = rpy(qLF_ref);

        RF_yaw_quat_first = quat(vec3(0,0,1), RF_rpy_cur.y);
        LF_yaw_quat_first = quat(vec3(0,0,1), LF_rpy_cur.y);

        if(SDB[step_phase].swingFoot == RFoot)
        {
            RF_yaw_delta_quat = inverse_HB(RF_yaw_quat_ref)*SDB[step_phase + 1].Fquat;

            rpy RF_delta_quat_rpy = rpy(RF_yaw_delta_quat);

            RF_delta_yaw_goal_rad = RF_delta_quat_rpy.y; // RF delta yaw for current step_phase
            LF_delta_yaw_goal_rad = 0;
        }
        else if(SDB[step_phase].swingFoot == LFoot)
        {
            LF_yaw_delta_quat = inverse_HB(LF_yaw_quat_ref)*SDB[step_phase + 1].Fquat;

            rpy LF_delta_quat_rpy = rpy(LF_yaw_delta_quat);

            LF_delta_yaw_goal_rad = LF_delta_quat_rpy.y; // LF delta yaw for current step_phase
            RF_delta_yaw_goal_rad = 0;
        }
        else
        {
            RF_delta_yaw_goal_rad = 0;
            LF_delta_yaw_goal_rad = 0;
        }

        step_phase_change_flag = false;
        cout<<"current phase changed: "<<step_phase<<"  t_step: "<<t_step<<endl;
    }

    if(step_phase >= N_step + 4)
    {
        cout<<"Walk done!"<<endl;
        ROSWalk_status = ROSWALK_WALKING_DONE;
        step_phase_change_flag = true;
        return -1;
    }


    //// ------------------Walking Stop when fell down----------------------------
    if(fabs(G_R_g_pitroll_rpy.r) > 14*D2R || fabs(G_R_g_pitroll_rpy.p) > 14*D2R )
    {
        cout<<"fall down!"<<endl;
        ROSWalk_status = ROSWALK_FALL_DONE;
        return -1;
    }


    //// ------------------p_ref_offset Control ----------------------------------
    if(ZMP_offset_controller_flag == true)
    {
        double Alpha_offset_con = 1./(1. + 2*3.14*dt*2.5);
        p_ref_con_error_filtered = Alpha_offset_con*p_ref_con_error_filtered + (1.0 - Alpha_offset_con)*(COM_m_filtered.y + dCOM_m_diff_filtered.y/w) - (COM_SA_LIPM.y + dCOM_SA_LIPM.y/w);

        p_ref_offset_y = -0.0004*p_ref_con_error_filtered;
    }
    else
    {
        p_ref_offset_y = 0.0;
    }
    //cout<<"p_ref_offset y : "<<p_ref_offset_y<<endl;


    ////------------------- Fill in the Window buffer----------------------
    WindowFill();


    ////----------------- get COM SA ref -------------------------------
    // fill in zmp ref buffer
    t_now_index = (int)((t_now + dt/2)*freq);

    // if t_step become longer than original T, WD_SA_ref should be extended
    int t_extention_index = 0;
    if(T_nom < t_step)
    {
        t_extention_index = (int)((t_step - T_nom + dt/2)*freq);
    }

    // middle index : t_now_index for middle time
    int middle_index = (int)((T_nom/2 + dt/2)*freq);

    for(int i=0; i < middle_index ; i++)
    {
        WD_SA_ref_extended[i].ZMP_ref.x = WD_SA_ref[i].ZMP_ref.x;
        WD_SA_ref_extended[i].ZMP_ref.y = WD_SA_ref[i].ZMP_ref.y;
    }

    for(int i=middle_index; i < middle_index + t_extention_index ; i++)
    {
        WD_SA_ref_extended[i].ZMP_ref.x = WD_SA_ref[middle_index].ZMP_ref.x;
        WD_SA_ref_extended[i].ZMP_ref.y = WD_SA_ref[middle_index].ZMP_ref.y;
    }

    for(int i=middle_index + t_extention_index; i < NL + t_now_index + t_extention_index ; i++)
    {
        WD_SA_ref_extended[i].ZMP_ref.x = WD_SA_ref[i - t_extention_index].ZMP_ref.x;
        WD_SA_ref_extended[i].ZMP_ref.y = WD_SA_ref[i - t_extention_index].ZMP_ref.y;
    }


    for(int i=0; i<NL+1 ; i++)
    {
        p_ref_SA[i].x = WD_SA_ref_extended[i + t_now_index].ZMP_ref.x;
        p_ref_SA[i].y = WD_SA_ref_extended[i + t_now_index].ZMP_ref.y;
    }

    // calc COM_SA_ref
    double temp_x1_SA = 0;
    double temp_x2_SA = 0;
    double temp_y1_SA = 0;
    double temp_y2_SA = 0;

    for(int i=0; i<3; i++)
    {
        temp_y1_SA += Gx[i]*COM_y_dy_ddy_old_SA[i];
        temp_x1_SA += Gx[i]*COM_x_dx_ddx_old_SA[i];
    }
    for(int i=0; i<NL; i++)
    {
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
    for(int i=0; i<NL+1 ; i++)
    {
        p_ref[i].x = WD[i].ZMP_ref.x;
        p_ref[i].y = WD[i].ZMP_ref.y;

        vec3 p_ref_for_COM_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[i]);

        p_ref_for_COM_local.y = p_ref_for_COM_local.y - 0.000;

        p_ref_for_COM[i] = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref,p_ref_for_COM_local);
    }

    // calc COM ref
    double temp_x1 = 0;
    double temp_x2 = 0;
    double temp_y1 = 0;
    double temp_y2 = 0;

    for(int i=0; i<3; i++)
    {
        temp_y1 += Gx[i]*COM_y_dy_ddy_old[i];
        temp_x1 += Gx[i]*COM_x_dx_ddx_old[i];
    }

    for(int i=0; i<NL; i++)
    {
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

    COM_y_dy_ddy_old = COM_y_dy_ddy;
    COM_x_dx_ddx_old = COM_x_dx_ddx;


    //// ---------------------Calc COM ref Local ----------------------------------------------
    COM_ref_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_LIPM);
    dCOM_ref_local = global2local_vec(qRF_ref, qLF_ref, dCOM_LIPM);
    ddCOM_ref_local = global2local_vec(qRF_ref, qLF_ref, ddCOM_LIPM);


    //// -------------------- Sway scaling ----------------------------------------------------
    double scaling_factor = 1.00;// + 0.1*sway_con;
    COM_ref_local.y = COM_ref_local.y*scaling_factor;
    dCOM_ref_local.y = dCOM_ref_local.y*scaling_factor;
    ddCOM_ref_local.y = ddCOM_ref_local.y*scaling_factor;

    COM_ref = local2global_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, COM_ref_local);
    dCOM_ref = local2global_vec(qRF_ref, qLF_ref, dCOM_ref_local);
    ddCOM_ref = local2global_vec(qRF_ref, qLF_ref, ddCOM_ref_local);

    CP_ref = COM_ref + dCOM_ref/w;

    COM_SA_ref = COM_SA_LIPM;
    dCOM_SA_ref = dCOM_SA_LIPM;
    ddCOM_SA_ref = ddCOM_SA_LIPM;


    //// --------------------- Get cZMP----------------------------------------------
    // CPT Controller ( get cZMP )
//    cZMP = 1/(1 - exp(w*dt*dt_gain1))*CP_ref - exp(w*dt*dt_gain1)/(1 - exp(w*dt*dt_gain1))*CP_m;// - CP_ref + p_ref[0];

    //cZMP from DCM Tracking Controller
//    double dT_cZMP = t_step - t_now;
//    if(dT_cZMP < 0.3) dT_cZMP = 0.3;
//    cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*((COM_m_filtered + dCOM_m_diff_filtered/w) - (COM_LIPM + dCOM_LIPM/w));
//    cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*((COM_m_filtered + dCOM_m_diff_filtered/w) - (COM_ref + COM_ref/w));
//    cZMP = p_ref[0] + exp(w*dT_cZMP)/(exp(w*dT_cZMP) - 1)*CP_e_imu;


    // --------cZMP for dsp fz control-------
    // Using damping con + imu error
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

    uCOM = COM_ref + COM_damping_con;// + COM_con;



    //// ---------------------------------------Step Adjustment -------------------------------------------------
    double dT_SA = dT;  // dT for clamping
    if(dT_SA < 10*dt) dT_SA = 10*dt;

    // Step Adjustment Flag enable & disable
    if(StepAdjustControl_flag == true)
    {
        if(LandingControl_flag == true)
        { // landing control on : if one of foot is landed, disable StepAdjust
            if((t_now > 0.005 && t_now < t_step - 0.08) && (RF_landing_flag == false || LF_landing_flag == false))
            {
                SA_Enable_flag = true;
            }
            else
            {
                SA_Enable_flag = false;
            }
        }
        else
        {
            if(t_now > 0.005 && t_now < t_step - 0.08)
            { // landing control off : SA enables depends on time
                SA_Enable_flag = true;
            }
            else
            {
                SA_Enable_flag = false;
            }
        }
    }
    else
    {
        SA_Enable_flag = false;
    }

    if(SDB[step_phase].swingFoot == RFoot && SA_Enable_flag == true)
    {
        HB_StepAdjustor HBSA;
        bool Second_step_optimization_flag = false;

        ////-------------------- First Step Optimization ------------------------------
        // get Stance foot rotation matrix
        rpy Euler_angle_StanceFoot_rad = rpy(qLF_ref);

        mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);

        mat3 f_R_g = inverse_HB(g_R_f);

        // CP_error left foot frame
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
        if(SDB_original[step_phase+1].swingFoot == DSP)
        { // for last step
            vec3 last_foot_offset = g_R_f*vec3(0, -pelv_w/2, 0);

            pSwingFoot = SDB_original[step_phase+1].Fpos + last_foot_offset;
            Second_step_optimization_flag = false;
        }
        else
        {
            pSwingFoot = SDB_original[step_phase+1].Fpos;
        }

        vec3 StanceFoot_to_SwingFoot_destination_global = pSwingFoot - pLF_ref;
        vec3 Stance_to_Swing_footFrame = f_R_g*StanceFoot_to_SwingFoot_destination_global;
        vec3 Stance_to_curruent_SwingFoot_footFrame = f_R_g*(pRF_ref - pLF_ref);

        // put the informations into QP solver  & Slove!
        if(HipTorqueControl_flag == false)
        {
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
        else
        {
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
        double alpha1x = 1/(1 + 2*PI*dt*20);
        del_u_f_filtered.x = alpha1x*del_u_f_filtered.x + (1 - alpha1x)*del_u_f_modi.x;
        double alpha1y = 0;// 1/(1 + 2*PI*dt*20);
        del_u_f_filtered.y = alpha1y*del_u_f_filtered.y + (1 - alpha1y)*del_u_f_modi.y;

        double alpha2 = 1/(1 + 2*PI*dt*10.0);
        del_b_f_filtered.x = alpha2*del_b_f_filtered.x + (1 - alpha2)*del_b_f.x;
        del_b_f_filtered.y = alpha2*del_b_f_filtered.y + (1 - alpha2)*del_b_f.y;

        double alpha3 = 1/(1 + 2*PI*dt*20.0);
        new_T_filtered = alpha3*new_T_filtered + (1 - alpha3)*new_T;

        // Change step time
        t_step = new_T_filtered;
        SDB[step_phase].t = new_T_filtered;

        // change modification varialbe to global frame
        del_u_g = g_R_f*del_u_f_filtered;
        del_b_g = g_R_f*del_b_f_filtered;


        // update footstep modification to future step data buffer (update SDB)
        for(int i = step_phase + 1; i < SDB.size(); i++)
        {
            SDB[i].Fpos = SDB_original[i].Fpos + del_u_g;
        }


        ////-------------------- Second Step Optimization ------------------------------
        if(Second_step_optimization_flag == true)
        {
            // get Next Stance foot rotation matrix
            rpy Next_StanceFoot_rpy = rpy(SDB[step_phase+1].Fquat);

            mat3 g_R_Nf = mat3(vec3(0,0,1), Next_StanceFoot_rpy.y);

            mat3 Nf_R_g = inverse_HB(g_R_Nf);

            // del_b of Next foot frame
            del_b0_Nf = Nf_R_g*del_b_g;

            // 2nd swing Foot destination in right foot frame
            vec3 pSwingFoot_2nd_global;
            if(SDB[step_phase + 2].swingFoot == DSP)
            { // for last step
                vec3 last_foot_offset = g_R_Nf*vec3(0, pelv_w/2, 0);

                pSwingFoot_2nd_global = SDB[step_phase + 2].Fpos + last_foot_offset;
            }
            else
            {
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

    if(SDB[step_phase].swingFoot == LFoot && SA_Enable_flag == true)
    {
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
        if(HipTorqueControl_flag == false)
        {
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
        else
        {
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

            if(new_T > T_nom)
            {
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
        for(int i = step_phase + 1; i < SDB.size(); i++)
        {
            SDB[i].Fpos = SDB_original[i].Fpos + del_u_g;
        }

        ////-------------------- Second Step Optimization ------------------------------

        if(Second_step_optimization_flag == true)
        {
            // get Next Stance foot rotation matrix
            rpy Next_StanceFoot_rpy = rpy(SDB[step_phase+1].Fquat);

            mat3 g_R_Nf = mat3(vec3(0,0,1), Next_StanceFoot_rpy.y);

            mat3 Nf_R_g = inverse_HB(g_R_Nf);

            // del_b of Next foot frame
            del_b0_Nf = Nf_R_g*del_b_g;

            // 2nd swing Foot destination in right foot frame
            vec3 pSwingFoot_2nd_global;
            if(SDB[step_phase + 2].swingFoot == DSP)
            { // for last step
                vec3 last_foot_offset = g_R_Nf*vec3(0, -pelv_w/2, 0);

                pSwingFoot_2nd_global = SDB[step_phase + 2].Fpos + last_foot_offset;
            }
            else
            {
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

    if(SA_Enable_flag == false)
    {
        del_u_f = vec3();
        del_b_f = vec3();
        del_u_f_modi = vec3();

        del_u_g = vec3();
        del_b_g = vec3();

        del_u_Nf = vec3();
        del_b_Nf = vec3();

        del_u_Nf_g = vec3();
    }


    //// ----------------qPel Generation --------------------------------------
    if(SA_Enable_flag == false)
    {
        double kp = 60;
        double kd = 60;

        Pelv_pitch_acc_ref = -kd*Pelv_pitch_vel_ref - kp*Pelv_pitch_ref;
        Pelv_roll_acc_ref = -kd*Pelv_roll_vel_ref - kp*Pelv_roll_ref;
    }

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

    if(StepAdjustControl_flag == false || HipTorqueControl_flag == false)
    {
        Pelv_roll_ref = 0;//
        Pelv_pitch_ref = 0;//
    }

    rpy Pelv_rpy = rpy(Pelv_roll_ref, Pelv_pitch_ref, get_Pel_yaw_from_qRF_qLF(qRF_ref, qLF_ref));
    qPel_ref = quat(Pelv_rpy);


    //// -------------------------------------- Foot trajectory ----------------------------
    //FootUp_height = 0.05/0.4*t_step;
    //for rigid foot
//    double Landing_Threshold = 40; // N
    //for damping foot
    double Landing_Threshold = 25; // N




    // RFoot position
    if(SDB[step_phase].swingFoot == RFoot)
    {
        // Landing Height & delta XY decision
        LandingZ_des = Calc_LandingHeight(RFoot, pLF_ref, SDB[step_phase+1].Fpos, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_delXY = Calc_Landing_delXY(RFoot, pLF_ref, SDB[step_phase+1].Fpos, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_Threshold = Calc_Landing_Threshold(RFoot, vec3());
        MaxFoot_y_cur = -MaxFoot_y;

        //cout<<"Landing delx: "<<Landing_delXY.x<<"  Landing dely: "<<Landing_delXY.y<<endl;

        // get orientation of stance foot
        vec3 pFoot;
        if(SDB[step_phase+1].swingFoot == DSP)
        {
            rpy Euler_angle_StanceFoot_rad = rpy(qLF_ref);

            mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
            vec3 last_foot_offset = g_R_f*vec3(0, -pelv_w/2, 0);

            pFoot = SDB[step_phase+1].Fpos + last_foot_offset;
        }
        else
        {
            pFoot = SDB[step_phase+1].Fpos;
        }

        // destination of Rfoot
        des_pRF_local  = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pFoot);

        double t_half_dsp = T_nom*dsp_ratio*0.5;

        if(LandingControl_flag == true)
        {
            if(t_now > t_half_dsp && t_now < t_half_dsp + 3*dt && RF_landing_flag == true)
            {
                RF_landing_flag = false;
                LF_landing_flag = true;
                step_status = DSP;
            }
            if(F_RF_filtered.z > Landing_Threshold && t_now > T_nom*0.6 && t_now < T_nom - t_half_dsp*2 && RF_landing_flag == false)
            {
                RF_landing_flag = true;
                step_status = DSP;

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

            if(RF_landing_flag == false || t_now < T_nom*0.5)
            {
                step_status = SSP;
               // Z direction
               RF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height + LandingZ_des, LandingZ_des-0.001);
               // X direction
               RF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, RF_x_dx_ddx_old, pFoot.x + Landing_delXY.x);
               // Y direction
               RF_y_dy_ddy = FootY_trajectory(t_step, t_now, dsp_ratio, RF_y_dy_ddy_old, pFoot.y + Landing_delXY.y);
            }

            if(RF_landing_flag == true && t_now > T_nom*0.4)
            {
                step_status = DSP;
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
        else
        { // Landing Control Off
            // Z direction
            RF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, RF_z_dz_ddz_old, FootUp_height + LandingZ_des/2.0, LandingZ_des);

            // get yaw orientation of stance foot
            vec3 pFoot;
            if(SDB[step_phase+1].swingFoot == DSP)
            {
                rpy Euler_angle_StanceFoot_rad = rpy(qLF_ref);

                mat3 g_R_f = mat3(vec3(0,0,1), Euler_angle_StanceFoot_rad.y);
                vec3 last_foot_offset = g_R_f*vec3(0, -pelv_w/2, 0);

                pFoot = SDB[step_phase+1].Fpos + last_foot_offset;
            }
            else
            {
                pFoot = SDB[step_phase+1].Fpos;
            }

            // X direction
            RF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, RF_x_dx_ddx_old, pFoot.x);
            // Y direction
            RF_y_dy_ddy = FootY_trajectory(t_step, t_now, dsp_ratio, RF_y_dy_ddy_old, pFoot.y);
        }

        // pLF recovary to 0----------------------------------------------------------------------------------------
        _isLF_swingFirst = true;
        if(_isRF_swingFirst == true)
        {
            _isRF_swingFirst = false;
            LF_z_dz_ddz_old = vec3(LF_z_dz_ddz[0], 0, 0);

            // Decide recovary speed
            recovary_speed = -LF_z_dz_ddz_old[0]/(T_nom*(1 - dsp_ratio) - del_t);

            if(recovary_speed > Max_recovary_speed)
            {
                recovary_speed = Max_recovary_speed;
            }
            else if(recovary_speed < -Max_recovary_speed)
            {
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
        if(Swing_leg_vibration_control_flag == true)
        {
            if(t_now > T_nom*dsp_ratio/2 && t_now < T_nom*0.7){// && fabs(RS.IMUangle.x) < 10*D2R){
                RSwingLeg_Vib_Control(ACC_RF_filtered, false);
                RTorso_Yaw_Vib_Control(RS.IMULocalW, false);
            }
            else{
                RSwingLeg_Vib_Control(ACC_RF_filtered, true);
                RTorso_Yaw_Vib_Control(RS.IMULocalW, true);
            }
        }
    }
    else if(SDB[step_phase].swingFoot == LFoot){
        // Landing Height decision
        LandingZ_des = Calc_LandingHeight(LFoot, SDB[step_phase+1].Fpos, pRF_ref, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_delXY = Calc_Landing_delXY(LFoot, SDB[step_phase+1].Fpos, pRF_ref, qLF_ref, qRF_ref, G_R_g_pitroll);
        Landing_Threshold = Calc_Landing_Threshold(LFoot, vec3());
        MaxFoot_y_cur = MaxFoot_y;

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

        if(LandingControl_flag == true)
        {
//            if(RS.F_LF.z <= 70 && t_now < t_step*0.5 && LF_landing_flag == true){
            if( t_now > t_half_dsp && t_now < t_half_dsp + 3*dt && LF_landing_flag == true){
                LF_landing_flag = false;
                RF_landing_flag = true;
                step_status = DSP;
            }
            if(F_LF_filtered.z > Landing_Threshold && t_now > T_nom*0.6 && t_now < T_nom - t_half_dsp*2 && LF_landing_flag == false){
                LF_landing_flag = true;
                step_status = DSP;

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
                step_status = SSP;
                // Z direction
                LF_z_dz_ddz = FootZ_trajectory(t_step, t_now, dsp_ratio, LF_z_dz_ddz_old, FootUp_height + LandingZ_des, LandingZ_des-0.001);
                // X direction
                LF_x_dx_ddx = FootX_trajectory(t_step, t_now, dsp_ratio, LF_x_dx_ddx_old, pFoot.x + Landing_delXY.x);
                // Y direction
                LF_y_dy_ddy = FootY_trajectory(t_step, t_now, dsp_ratio, LF_y_dy_ddy_old, pFoot.y + Landing_delXY.y);
            }
            if(LF_landing_flag == true && t_now > T_nom*0.4){
                step_status = DSP;
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
            LF_y_dy_ddy = FootY_trajectory(t_step, t_now, dsp_ratio, LF_y_dy_ddy_old, pFoot.y);
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


    //// --------------- Waist momentum compensation ----------------------
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
    if(fabs(L_local.y) > pelv_w)
    {
       foot_width = fabs(L_local.y) - pelv_w;
    }
    else
    {
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
    DSP_Fz_controller(SDB[step_phase].swingFoot, RS.F_RF.z, RS.F_LF.z); // this function gives z_ctrl;

    if(DSP_FZ_Control_flag == true)
    {
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

    if(RF_landing_flag == false)
    {
        dRF_landing_angle.r = 0.005*(-G_R_g_pitroll_rpy.r) - RF_landing_angle.r/0.1;
        dRF_landing_angle.p = 0.005*(-G_R_g_pitroll_rpy.p) - RF_landing_angle.p/0.1;
    }
    else
    {
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

    if(LF_landing_flag == false)
    {
        dLF_landing_angle.r = 0.005*(-G_R_g_pitroll_rpy.r) - LF_landing_angle.r/0.1;
        dLF_landing_angle.p = 0.005*(-G_R_g_pitroll_rpy.p) - LF_landing_angle.p/0.1;
    }
    else
    {
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

    if(Pos_Ankle_torque_control_flag == true && Landing_angle_control_flag == false)
    {
        qRF_ref = RF_yaw_quat_ref*RF_quat_ctrl;
        qLF_ref = LF_yaw_quat_ref*LF_quat_ctrl;

    }
    else if(Pos_Ankle_torque_control_flag == false && Landing_angle_control_flag == true)
    {
        qRF_ref = RF_yaw_quat_ref*quat(RF_landing_angle);
        qLF_ref = LF_yaw_quat_ref*quat(LF_landing_angle);

    }
    else if(Pos_Ankle_torque_control_flag == true && Landing_angle_control_flag == true)
    {
        qRF_ref = RF_yaw_quat_ref*quat(RF_landing_angle)*RF_quat_ctrl;
        qLF_ref = LF_yaw_quat_ref*quat(LF_landing_angle)*LF_quat_ctrl;
        //cout<<"RF landing: "<<RF_landing_angle.p<<"LF landing: "<<LF_landing_angle.p<<endl;
    }
    else{
        qRF_ref = RF_yaw_quat_ref;
        qLF_ref = LF_yaw_quat_ref;
    }

    ////------------------- hip roll deflection compensation----------------------------

    if(Joint_deflection_compensation_flag == true)
    {
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

    if(dT < t_step/2)
    {
        if(SDB[step_phase].ros_step_phase != -1 && userData->FLAG_receivedROS == ROS_RX_EMPTY)
        {
            printf("ros step_done\n");
            printf("ucom pose = %f, %f\n", userData->pel_pose[0], userData->pel_pose[1]);
            printf("rcom pose = %f, %f\n", COM_m_filtered[0], COM_m_filtered[1]);

//            ROSWalk_status = ROSWALK_STEP_DONE;
            userData->FLAG_sendROS = 2;
        }
    }
    if(dT <= dt + dt/2)
    {
        step_phase_change_flag = true;
        printf("step done\n");
//        if(SDB[step_phase].ros_step_phase != -1)
//        {
//            printf("ros step_done\n");
//            ROSWalk_status = ROSWALK_STEP_DONE;
//        }else
//        {
//            printf("step_done\n");
//            ROSWalk_status = ROSWALK_STEP_PASS;
//        }

        step_phase++;
    }
}



void GG_SingleLogWalk::WindowFill()
{
    int last_tic;


    ////-------put ZMP trajectory to Window
    // Count how many step phase in preview time
    int temp_count = (int)((t_prev - dT + dt/2)*freq);
    No_of_cycle = 0;
    while(1){
        if(temp_count - SDB[step_phase + No_of_cycle + 1].t*freq <= 0){
            break;
        }
        else{
            temp_count -= SDB[step_phase + No_of_cycle + 1].t*freq;
        }

        No_of_cycle += 1;
    }

    // put ZMP tracjectory to Window
    double t_half_dsp = SDB[step_phase].t*dsp_ratio_com*0.5;


    //// -- for current step ZMP trajectory x,y--------------------------------------
    if(dT >= SDB[step_phase].t - t_half_dsp){
        double dsp_timer = 2*t_half_dsp - (dT - (SDB[step_phase].t - t_half_dsp)); // time for 5th trajectory interval

        // 5th parametor
        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;
        if(step_phase >= 1){
            calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase-1].Fpos.y,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
            calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase-1].Fpos.x,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);
        }
        else{
            calc_5th_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
            calc_5th_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);
        }

        for(int i=0 ; i<(int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = (int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq);

        for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD[i].ZMP_ref = SDB[step_phase].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq);

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);
        dsp_timer = 0;

        for(int i = last_tic; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

    }
    else if(dT > t_half_dsp){
        for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
            WD[i].ZMP_ref = SDB[step_phase].Fpos;
        }
        last_tic = (int)((dT - t_half_dsp + dt/2)*freq);

        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);
        double dsp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);
    }
    else{// if(dT >= 0){
        double dsp_timer = t_half_dsp - dT;

        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++)
        {
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    //// ----- for (2 ~ N-1) step ZMP trajectory x,y
    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio_com*0.5;

        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + j - 1].Fpos.y,0,0), vec3(SDB[step_phase + j].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + j - 1].Fpos.x,0,0), vec3(SDB[step_phase + j].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic ; i < last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD[i].ZMP_ref = SDB[step_phase + j].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq);

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + j].Fpos.y,0,0), vec3(SDB[step_phase + j + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + j].Fpos.x,0,0), vec3(SDB[step_phase + j + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        dsp_timer = 0;
        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp+ + dt/2)*freq);
    }

    ////---- for N's step (last step in preview time)
    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio_com*0.5;
    if((int)((t_prev + dt/2)*freq) + 1 - last_tic  < (int)((t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < (int)((t_prev + dt/2)*freq) + 1; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }

    }
    else if((int)((t_prev + dt/2)*freq) + 1 - last_tic < (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i<(int)((t_prev + dt/2)*freq) + 1; i++){
            WD[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
    }
    else{// if((int)(t_prev*freq) - 1 - last_tic <= (int)((SDB[step_phase + No_of_cycle + 1].t)*freq)){
        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        double dsp_timer = t_half_dsp;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i < last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq); i++){
            WD[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq);

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        dsp_timer = 0;

        for(int i = last_tic ; i < (int)((t_prev + dt/2)*freq) + 1 ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
    }
}

void GG_SingleLogWalk::HB_set_step(vec3 _COM_ini, quat _qPel, vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF, double _WST_ini_deg, double _t_step, double _N_step, double _step_stride, int _RL_first){
    t_step = _t_step;
    des_step_t = t_step;
    N_step = _N_step;
    step_stride = _step_stride;
    R_or_L = _RL_first;
    dsp_ratio = 0.04;
    dsp_ratio_com = 0.2;
    des_dsp_ratio = dsp_ratio;
    FootUp_height = 0.10;
    roswalk_first_phase = 0;

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
    ROSWalk_status = ROSWALK_BREAK;

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

    // Singlelog walking
    MaxFoot_y_cur = MaxFoot_y;

    // joystick Walking
    ROSWalk_flag = false;
    ROSWalk_off_flag = false;
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
            SD_temp.ros_step_phase = -1;

            SDB.push_back(SD_temp);
            cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot << "   t_step = " << SDB[i].t<<endl;
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
                SD_temp.ros_step_phase = -1;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot << "   t_step = " << SDB[i].t<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }
            else{
                SD_temp.Fpos = pRF_ref;
                SD_temp.Fquat = qRF_ref;
                SD_temp.yaw_rad = 0;
                SD_temp.swingFoot = -R_or_L;
                SD_temp.t = t_step;
                SD_temp.ros_step_phase = -1;

                SDB.push_back(SD_temp);
                cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot << "   t_step = " << SDB[i].t<<endl;
                cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
            }
        }
        else if(i == N_step + 2)
        { // last step
            STEP_INFO SD_temp;
            SD_temp.Fpos = vec3(SDB.back().Fpos.x, 0, 0);
            SD_temp.Fquat = quat();
            SD_temp.yaw_rad = 0;
            SD_temp.swingFoot = DSP;
            SD_temp.t = t_stable;
            SD_temp.ros_step_phase = -1;

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
            cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot << "   t_step = " << SDB[i].t<<endl;
            cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
        }
        else
        {
            STEP_INFO SD_temp;
//            if(Walking_mode == SINGLELOG)
//            {
//                if(i == 2)
//                {
//                    SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride, R_or_L*singlelog_w/2, 0);
//                }else if(i > N_step -1)
//                {
//                    SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride, R_or_L*pelv_w/2, 0);
//                }else
//                {
//                    SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride, R_or_L*singlelog_w/2, 0);
//                }
//            }else
//            {
            SD_temp.Fpos = vec3(SDB.back().Fpos.x + step_stride, R_or_L*pelv_w/2, 0);
//            }

            SD_temp.Fquat = quat();
            SD_temp.yaw_rad = 0;
            SD_temp.swingFoot = -R_or_L;
            SD_temp.t = t_step;
            SD_temp.ros_step_phase = -1;

            SDB.push_back(SD_temp);
            cout<<"i : "<<i<<"  swingFoot: "<<(int)SDB[i].swingFoot << "   t_step = " << SDB[i].t<<endl;
            cout << "Fpos = " << SD_temp.Fpos.x << "," << SD_temp.Fpos.y << endl;
        }

        R_or_L *= -1;
    }


}

vec3 GG_SingleLogWalk::FootY_trajectory(double _real_t_step, double _t_foot_now, double _dsp_ratio, vec3 _y_dy_ddy_Foot_old, double _Y_footStep)
{
    double t_moving = 0.9*_real_t_step*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;
    double t_half_ssp = t_moving*0.5;

//    if(Walking_mode == SINGLELOG)
//    {
//        if(_t_foot_now < t_half_dsp - 0.5*dt)
//        {
//        }
//        else if(_t_foot_now < t_half_ssp + t_half_dsp - 0.5*dt)
//        {
//            vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, t_half_ssp + t_half_dsp, _y_dy_ddy_Foot_old, vec3(MaxFoot_y_cur,0,0));
//            return p_dp_ddp;
//        }
//        else if(_t_foot_now < _real_t_step - t_half_dsp - 0.5*dt)
//        {
//            vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, _real_t_step - t_half_dsp + dt, _y_dy_ddy_Foot_old, vec3(_Y_footStep,0,0));
//            return p_dp_ddp;
//        }
//        else
//        {
//        }
//    }else
//    {
        if(_t_foot_now < t_half_dsp - 0.5*dt)
        {
        }
        else if(_t_foot_now < t_moving + t_half_dsp - 0.5*dt)
        {
            vec3 p_dp_ddp = calc_3rd(_t_foot_now - dt, t_moving + t_half_dsp, _y_dy_ddy_Foot_old, vec3(_Y_footStep,0,0));
            return p_dp_ddp;//vec3(xFoot, dxFoot, ddxFoot); //return z, dz, ddz
        }
        else
        {
        }
//    }

    return _y_dy_ddy_Foot_old;
}
void GG_SingleLogWalk::rosstep_l2g()
{
    printf("================rosstep l2g=================\n");
    double l2g_x, l2g_y, l2g_r = 0.;
    double l_x, l_y, l_r = 0.;

    for(int i = 0; i<4; i++)
    {
        rosstep_global[i].step_phase = userData->ros_footsteps[i].step_phase;
        rosstep_global[i].lr_state = userData->ros_footsteps[i].lr_state;

        if(i == 0)
        {
            if(rosstep_global[i].step_phase == 0)
            {
                roswalk_first_phase = step_phase + 1;
                printf("first step (origin:left foot)\n");

                l2g_x = SDB[step_phase].Fpos.x;
                l2g_y = pelv_w/2;
                l2g_r = 0.;
            }else
            {
                l2g_x = SDB[step_phase].Fpos.x;
                l2g_y = SDB[step_phase].Fpos.y;
                l2g_r = rpy(SDB[step_phase].Fquat).y*R2D;

            }
        }else
        {
//            l2g_x = rosstep_global[i-1].x;
//            l2g_y = rosstep_global[i-1].y;
//            l2g_r = rosstep_global[i-1].r;
        }

        printf("l2g = %f, %f, %f\n",l2g_x, l2g_y, l2g_r);

        vec3 localfoot_ros = vec3(userData->ros_footsteps[i].x, userData->ros_footsteps[i].y, 0.);
        printf("localfoot_ros = %f, %f, %f\n",localfoot_ros[0],localfoot_ros[1],userData->ros_footsteps[i].r);

        if(userData->ros_footsteps[i].x == 0.)
        {
            printf("no input\n");
            rosstep_global[i].x = 0.;
            rosstep_global[i].y = 0.;
            rosstep_global[i].r = 0.;

        }else
        {
            vec3 globalfoot_ros;
            globalfoot_ros[0] = localfoot_ros[0]*cos(l2g_r*D2R) - localfoot_ros[1]*sin(l2g_r*D2R);
            globalfoot_ros[1] = localfoot_ros[0]*sin(l2g_r*D2R) + localfoot_ros[1]*cos(l2g_r*D2R);
            printf("globalfoot_ros = %f, %f, %f\n",globalfoot_ros[0],globalfoot_ros[1],userData->ros_footsteps[i].r);

            rosstep_global[i].x = globalfoot_ros[0] + l2g_x;
            rosstep_global[i].y = globalfoot_ros[1] + l2g_y;
            rosstep_global[i].r = userData->ros_footsteps[i].r + l2g_r;
        }



        printf("[%d] global = %f, %f, %f    local = %f, %f, %f [%d]\n",
               rosstep_global[i].step_phase, rosstep_global[i].x, rosstep_global[i].y, rosstep_global[i].r,
               userData->ros_footsteps[i].x, userData->ros_footsteps[i].y, userData->ros_footsteps[i].r, rosstep_global[i].lr_state);
    }
    printf("=============================================\n");
}
