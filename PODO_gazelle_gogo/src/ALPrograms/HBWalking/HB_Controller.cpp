#include "HB_PreviewWalk.h"
//#define SIGN(x) (x>=0 ? 1:-1)

vec3 HB_PreviewWalk::DampingControl(){
    //// -------------------Damping Controller test---------------------------
    //Original System : Ks = 2430.9 C = 1.780  m = 32;
    double Ks = 2430.9;
    double C = 1.780;
    double m = 32;

    // Desired System
    double wn_des = 4;
    double zeta_des = 0.9994;
    double Ks_des = m*zc*zc*(wn_des*wn_des + g/zc);
    double C_des = 2*m*zc*zc*zeta_des*wn_des;

    double A_3 = g/zc - Ks/m/zc/zc;
    double A_4 = -C/m/zc/zc;
    double B_2 = Ks/m/zc/zc/zc;

    double Ades_3 = g/zc - Ks_des/m/zc/zc;
    double Ades_4 = -C_des/m/zc/zc;

    double k1 = (A_3 - Ades_3)/B_2;
    double k2 = (A_4 - Ades_4)/B_2;

    vec3 state_X = COM_m/zc;
    vec3 state_dX = dCOM_m_diff_filtered/zc;

    vec3 COM_damping_con = -(k1*state_X + k2*state_dX)*zc;
    COM_damping_con.x = 0;


    if(COM_damping_con.y > 0.05) COM_damping_con.y = 0.05;
    if(COM_damping_con.y < -0.05) COM_damping_con.y = -0.05;


    // observer




    cout<<"COMm.y : "<<COM_m.y<<endl;
//    cout<<"uCOM.y : "<<uCOM.y<<endl;
//    cout<<"A_3 A_4 B_2 zc: "<<A_3<<", "<<A_4<<", "<<B_2<<", "<<zc<<endl;
//    cout<<"k1 k2 : "<<k1<<", "<<k2<<endl;

    return COM_damping_con;

}

void HB_PreviewWalk::DampingControl2(){ //  Only FT used
    //// -------------------Damping Controller test---------------------------
    // Ks = 2488.4 C = 25.7819 (real system) m = 32;
    // y -direction
    double Ks = 4516.1;//2430.9;  //after new leg part and torso mass
    double C = 58.72;//1.780;    //after new leg part and torso mass
    double m = 42;

//    double wn_des = 4;
//    double zeta_des = 0.9994;
//    double Ks_des = m*zc*zc*(wn_des*wn_des + g/zc);
//    double C_des = 2*m*zc*zc*zeta_des*wn_des;

    double A_3 = g/zc - Ks/m/zc/zc;
    double A_4 = -C/m/zc/zc;
    double B_2 = Ks/m/zc/zc/zc;

//    double Ades_3 = g/zc - Ks_des/m/zc/zc;
//    double Ades_4 = -C_des/m/zc/zc;

//    double k1 = (A_3 - Ades_3)/B_2;
//    double k2 = (A_4 - Ades_4)/B_2;

//    vec3 Kc = vec3(k1, k2, 0);
    vec3 Kc = vec3(-0.6480,    0.0240, 0); //pole [-4+0.01i -4-0.01i];


    // x-direction

    double Ksx = 2000.0;//879.9401;
    double Cx = 40.7;//34.3596;

//    double wnx_des = 4;
//    double zetax_des = 0.9994;
//    double Ksx_des = m*zc*zc*(wnx_des*wnx_des + g/zc);
//    double Cx_des = 2*m*zc*zc*zetax_des*wnx_des;

    double Ax_3 = g/zc - Ksx/m/zc/zc;
    double Ax_4 = -Cx/m/zc/zc;
    double Bx_2 = Ksx/m/zc/zc/zc;

//    double Axdes_3 = g/zc - Ksx_des/m/zc/zc;
//    double Axdes_4 = -Cx_des/m/zc/zc;

//    double kx1 = (Ax_3 - Axdes_3)/Bx_2;
//    double kx2 = (Ax_4 - Axdes_4)/Bx_2;

//    vec3 Kcx = vec3(kx1, kx2, 0);
    vec3 Kcx = vec3(-0.4945,    0.0610, 0); //pole [-4+0.01i -4-0.01i];



    //// observer
    // y-direction
    mat3 A_obs = mat3(0,  1,  0,
                   A_3,A_4,0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, B_2, 0);
    vec3 C_obs = vec3(Ks/m/g, C/m/g, 0);

    double D_obs = -Ks/m/g/zc;

    vec3 Ke = vec3(1.3716, -9.7718, 0); // observer pole [-8+0.01i -8-0.01i];

    // x-direction
    mat3 Ax_obs = mat3(0,  1,  0,
                   Ax_3,Ax_4,0,
                   0,  0,  0);
    vec3 Bx_obs = vec3(0, Bx_2, 0);
    vec3 Cx_obs = vec3(Ksx/m/g, Cx/m/g, 0);

    double Dx_obs = -Ksx/m/g/zc;

    vec3 Kex = vec3( 2.9894, -1.5150, 0); // observer pole [-8+0.01i -8-0.01i];

    double alpha_zmp_e = 1/(1 + 2*PI*dt*8.0);
    ZMP_error_global = alpha_zmp_e*ZMP_error_global + (1 - alpha_zmp_e)*(ZMP_global - p_ref[0]);

    ZMP_error_local = global2local_vec(qRF_ref, qLF_ref, ZMP_error_global);

//    vec3 ZMP_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, ZMP_global);
//    vec3 cZMP_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, cZMP);
//    vec3 p_ref_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]);

//    ZMP_error_local.x =  ZMP_local.x - p_ref_local.x;
//    ZMP_error_local.y =  ZMP_local.y - cZMP_local.y;

    // observed state is local frame state
    dY_obs = A_obs*Y_obs + B_obs*(-dot(Kc,Y_obs)) + Ke*(ZMP_error_local.y - (dot(C_obs,Y_obs) + D_obs*(-dot(Kc,Y_obs))));
    Y_obs = Y_obs + dY_obs*dt;

    dX_obs = Ax_obs*X_obs + Bx_obs*(-dot(Kc,X_obs)) + Kex*(ZMP_error_local.x - (dot(Cx_obs,X_obs) + Dx_obs*(-dot(Kcx,X_obs))));
    X_obs = X_obs + dX_obs*dt;

    vec3 COM_damping_con_local = vec3(-dot(Kc,X_obs),-dot(Kc,Y_obs),0)*zc;

    if(COM_damping_con_local.y > 0.05) COM_damping_con_local.y = 0.05;
    if(COM_damping_con_local.y < -0.05) COM_damping_con_local.y = -0.05;
    if(COM_damping_con_local.x > 0.05) COM_damping_con_local.x = 0.05;
    if(COM_damping_con_local.x < -0.05) COM_damping_con_local.x = -0.05;

//    uCOM = COM_ref + COM_damping_con;

    //cout<<"COMy_obs : "<<Y_obs[0]*zc<<endl;

    COM_damping_con = local2global_vec(qRF_ref, qLF_ref, COM_damping_con_local);


}

void HB_PreviewWalk::DampingControl3(){  // FT and IMU are used
    //// -------------------Damping Controller test---------------------------
    // Ks = 2245.4.4 C = 38.71 (real system) m = 32;
    double Ks = 2430.9;//2245.4;//2430.9; //2245.4
    double C = 38.71;//1.780; // 38.71
    double m = 33;

    double Ksx = 879.9401;
    double Cx = 34.3596;

//    double wn_des = 4;
//    double zeta_des = 0.9994;
//    double Ks_des = m*zc*zc*(wn_des*wn_des + g/zc);
//    double C_des = 2*m*zc*zc*zeta_des*wn_des;

    double A_3 = g/zc - Ks/m/zc/zc;
    double A_4 = -C/m/zc/zc;
    double B_2 = Ks/m/zc/zc/zc;

    double Ax_3 = g/zc - Ksx/m/zc/zc;
    double Ax_4 = -Cx/m/zc/zc;
    double Bx_2 = Ksx/m/zc/zc/zc;

//    double Ades_3 = g/zc - Ks_des/m/zc/zc;
//    double Ades_4 = -C_des/m/zc/zc;

//    double k1 = (A_3 - Ades_3)/B_2;
//    double k2 = (A_4 - Ades_4)/B_2;

//    vec3 Kc = vec3(k1, k2, 0);

//    vec3 Kc = vec3(-0.4440,    0.0301, 0); // pole placement [-6+0.001i -6-0.001i];
    vec3 Kc = vec3(-0.4440,    0.0301, 0); // pole placement [-6+0.001i -6-0.001i];
    vec3 Kcx = vec3(-0.3442,    0.0451,0); //pole placement [-4+0.001i -4-0.001i];

    // observer
    mat3 A_obs = mat3(0,  1,  0,
                   A_3,A_4,0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, B_2, 0);
    mat3 C_obs = mat3(Ks/m/g, C/m/g, 0,
                      1,      0,     0,
                      0,      1,     0);
    vec3 D_obs = vec3(-Ks/m/g/zc,0,0);

    // x
    mat3 Ax_obs = mat3(0,  1,  0,
                   Ax_3,Ax_4,0,
                   0,  0,  0);
    vec3 Bx_obs = vec3(0, Bx_2, 0);
    mat3 Cx_obs = mat3(Ksx/m/g, Cx/m/g, 0,
                      1,      0,     0,
                      0,      1,     0);
    vec3 Dx_obs = vec3(-Ksx/m/g/zc,0,0);


    // steady state Kalman Filter
    // with Q = [1 0; 0 0.1];
    //      R = [0.1 0 0; 0 0.05 0; 0 0 100];
//    mat3 Ke = mat3(3.0898, 0.7980, -0.0001,
//                   -0.1255, -1.380, 0.0702,
//                   0,0,0);
    mat3 Ke = mat3(9.6030,    0.1408,   -0.0038,
                   -1.3041,   -0.3785,    0.2090,
                   0,0,0);

//    mat3 Kex = mat3(9.2710 ,   0.0714,   -0.0667,
//                    -5.6173,   -0.1335,    1.1961,
//                     0,0,0);
    mat3 Kex = mat3(3.5164  ,  0.0691 ,  -0.3244,
                    -4.9341 ,  -0.1622  ,  3.7990,
                    0,0,0);


    double alpha_zmp_e = 1/(1 + 2*PI*dt*4.0);
    ZMP_error_global = alpha_zmp_e*ZMP_error_global + (1 - alpha_zmp_e)*(ZMP_global - p_ref[0]);

    ZMP_error_local = global2local_vec(qRF_ref, qLF_ref, ZMP_error_global); // ZMP error local
//    ZMP_error_local = global2local_vec(qRF_ref, qLF_ref, ZMP_global - cZMP_proj_filtered); // ZMP error local


//    vec3 ZMP_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, ZMP_global);
//    vec3 cZMP_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, cZMP);
//    vec3 p_ref_local = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, p_ref[0]);

//    ZMP_error_local.x =  ZMP_local.x - p_ref_local.x;
//    ZMP_error_local.y =  ZMP_local.y - (p_ref_local.y);

//    COM_error_local = global2local_vec(qRF_ref, qLF_ref, vec3());//COM_m - COM_LIPM);     // COM error local
//    dCOM_error_local = global2local_vec(qRF_ref, qLF_ref, dCOM_m_diff - dCOM_LIPM);  // dCOM error local

    double alpha_e_imu = 1/(1 + 2*PI*dt*6.0);
    COM_e_imu_local_filtered = alpha_e_imu*COM_e_imu_local_filtered + (1- alpha_e_imu)*COM_e_imu_local;
    dCOM_e_imu_local_filtered = alpha_e_imu*dCOM_e_imu_local_filtered + (1- alpha_e_imu)*dCOM_e_imu_local;


    COM_error_local = global2local_vec(qRF_ref, qLF_ref, COM_e_imu_local_filtered);     // COM error local
    dCOM_error_local = global2local_vec(qRF_ref, qLF_ref, dCOM_e_imu_local_filtered);  // dCOM error local


    // observed state is local frame state
    dY_obs = A_obs*Y_obs + B_obs*(-dot(Kc,Y_obs)) + Ke*(vec3(ZMP_error_local.y,COM_error_local.y/zc,dCOM_error_local.y/zc) - (C_obs*Y_obs + D_obs*(-dot(Kc,Y_obs))));
    Y_obs = Y_obs + dY_obs*dt;


    dX_obs = Ax_obs*X_obs + Bx_obs*(-dot(Kcx,X_obs)) + Kex*(vec3(ZMP_error_local.x,COM_error_local.x/zc,dCOM_error_local.x/zc) - (Cx_obs*X_obs + Dx_obs*(-dot(Kcx,X_obs))));
    X_obs = X_obs + dX_obs*dt;

    vec3 COM_damping_con_local = vec3(-dot(Kcx,X_obs),-dot(Kc,Y_obs),0)*zc;

    if(COM_damping_con_local.y > 0.05) COM_damping_con_local.y = 0.05;
    if(COM_damping_con_local.y < -0.05) COM_damping_con_local.y = -0.05;
    if(COM_damping_con_local.x > 0.05) COM_damping_con_local.x = 0.05;
    if(COM_damping_con_local.x < -0.05) COM_damping_con_local.x = -0.05;

//    uCOM = COM_ref + COM_damping_con;

    //cout<<"COMy_obs : "<<Y_obs[0]*zc<<endl;

    COM_damping_con = local2global_vec(qRF_ref, qLF_ref, COM_damping_con_local);

}

void HB_PreviewWalk::ZMP_Tracking_controller(){
   double k_zmp_y = w*0.3;
   double k_com_y = 1*w;

   double k_zmp_x = w*0.3;
   double k_com_x = 0.8*w;

   vec3 zmp_ref;

   //zmp_ref.y = +2.67*(COM_ref.y - COM_m_filtered.y) + 0.34*(dCOM_ref.y - dCOM_m_filtered.y) - p_ref[0].y;

   dCOM_zmp_con.y = -k_zmp_y*(p_ref[0].y - ZMP_global.y) + k_com_y*(COM_ref.y - COM_m_filtered.y);
   dCOM_zmp_con.x = -k_zmp_x*(p_ref[0].x - ZMP_global.x) + k_com_x*(COM_ref.x - COM_m_filtered.x);
//   dCOM_zmp_con = -k_zmp*(p_ref[0] - ZMP_global) + k_com*(COM_ref - uCOM);
   COM_zmp_con.x += dCOM_zmp_con.x*dt;
   COM_zmp_con.y += dCOM_zmp_con.y*dt;

   if(COM_zmp_con.y > 0.05) COM_zmp_con.y = 0.05;
   if(COM_zmp_con.y < -0.05) COM_zmp_con.y = -0.05;
   if(COM_zmp_con.x > 0.05) COM_zmp_con.x = 0.05;
   if(COM_zmp_con.x < -0.05) COM_zmp_con.x = -0.05;

   cout<<"COM_zmp_con.y : "<<COM_zmp_con.y<<endl;
}

void HB_PreviewWalk::State_Observer(){
    //// -------------------Damping Controller test---------------------------
    // Ks = 2488.4 C = 25.7819 (real system) m = 32;
    double Ks = 2430.9;
    double C = 1.780;
    double m = 32;

    double wn_des = 4;
    double zeta_des = 0.9994;
    double Ks_des = m*zc*zc*(wn_des*wn_des + g/zc);
    double C_des = 2*m*zc*zc*zeta_des*wn_des;

    double A_3 = g/zc - Ks/m/zc/zc;
    double A_4 = -C/m/zc/zc;
    double B_2 = Ks/m/zc/zc/zc;

    double Ades_3 = g/zc - Ks_des/m/zc/zc;
    double Ades_4 = -C_des/m/zc/zc;

    double k1 = (A_3 - Ades_3)/B_2;
    double k2 = (A_4 - Ades_4)/B_2;

    vec3 Kc = vec3(k1, k2, 0);
//    vec3 Kc = vec3(0, 0, 0);

    // observer
    mat3 A_obs = mat3(0,  1,  0,
                   A_3,A_4,0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, B_2, 0);
//    vec3 C_obs = vec3(Ks/m/g, C/m/g, 0);
    mat3 C_obs = mat3(Ks/m/g, C/m/g, 0,
                      1,      0,     0,
                      0,      1,     0);
    vec3 D_obs = vec3(-Ks/m/g/zc,0,0);


    // steady state Kalman Filter
    // with Q = [1 0; 0 0.1];
    //      R = [0.1 0 0; 0 0.1 0; 0 0 10];

    mat3 Ke = mat3(2.8619,    0.7403,   -0.0740,
                   -5.4630,   -1.4805,    4.7484,
                   0,0,0);

    vec3 ZMP_local = global2local_vec(qRF_ref, qLF_ref, ZMP_global - p_ref[0]);

    vec3 COM_m_local = global2local_vec(qRF_ref, qLF_ref, COM_m - COM_ref);
    vec3 dCOM_m_local = global2local_vec(qRF_ref, qLF_ref, dCOM_m_imu - dCOM_ref);

    //vec3 uCOM_local = Calc_local(pRF_ref, qRF_ref, pLF_ref, qLF_ref, uCOM);

    // observed state is local frame state
    dYe_obs = A_obs*Ye_obs + B_obs*(-dot(Kc,Ye_obs)) + Ke*(vec3(ZMP_local.y,COM_m_local.y/zc,dCOM_m_local.y/zc) - (C_obs*Ye_obs + D_obs*(-dot(Kc,Ye_obs))));
//    dY_obs = A_obs*Y_obs + B_obs*(uCOM_local.y/zc) + Ke*(vec3(ZMP_local.y,COM_m_local.y/zc,dCOM_m_local.y/zc) - (C_obs*Y_obs + D_obs*(uCOM_local.y/zc)));
//    dY_obs = A_obs*Y_obs + B_obs*(0) + Ke*(ZMP_global.y - (dot(C_obs,Y_obs) + D_obs*(0)));
    Ye_obs = Ye_obs + dYe_obs*dt;

//    dX_obs = A_obs*X_obs + B_obs*(-dot(Kc,X_obs)) + Ke*(ZMP_local.x - (dot(C_obs,X_obs) + D_obs*(-dot(Kc,X_obs))));
    dXe_obs = A_obs*Xe_obs + B_obs*(-dot(Kc,Xe_obs)) + Ke*(vec3(ZMP_local.x,COM_m_local.x/zc,dCOM_m_local.x/zc) - (C_obs*Xe_obs + D_obs*(-dot(Kc,Xe_obs))));

    Xe_obs = Xe_obs + dXe_obs*dt;



}

vec3 HB_PreviewWalk::InputShaping(){
//    double _M = 80;
//    double _C = 10;
//    double _K = 5250;
    double _My = 32;
    double _Cy = 34.78;
    double _Ky = 2000;

    double _Mx = 32;
    double _Cx = 34.78;
    double _Kx = 1000;// 879

    vec3 COM_con_local;

    if(step_phase < 1){
        _My = 10;
        _Cy = 38.7;// - 80*1/(exp(t_now*freq*0.8));;
        _Ky = 4500 - 2000*1/(exp(t_now*freq*0.5));
    }
    else{
        _My = 32;
        _Cy = 38.7;  // low  --> sway down
        _Ky = 3500;
    }

//    //for Ankle position control
//    if(step_phase < 1){
//        _My = 32;
//        _Cy = 35.7;// - 80*1/(exp(t_now*freq*0.8));;
//        _Ky = 2500 - 100*1/(exp(t_now*freq*0.5));
//    }
//    else{
//        _My = 32;
//        _Cy = 38.7;  // low  --> sway down
//        _Ky = 2500;
//    }

    double alpha1 = 1/(1 + 2*PI*dt*4.0);
    dCOM_ref_local_filtered = alpha1*dCOM_ref_local_filtered + (1.0 - alpha1)*dCOM_ref_local;

    double alpha2 = 1/(1 + 2*PI*dt*4.0);
    ddCOM_ref_local_filtered = alpha2*ddCOM_ref_local_filtered + (1.0 - alpha2)*ddCOM_ref_local;


    COM_con_local.y = _Cy/_Ky*dCOM_ref_local_filtered.y + _My*zc*zc/_Ky*ddCOM_ref_local_filtered.y - _My*g*zc/_Ky*COM_ref_local.y;

    //COM_con_local.x = _Cx/_Kx*dCOM_ref_local.x + _Mx*zc*zc/_Kx*ddCOM_ref_local.x - _Mx*g*zc/_Kx*COM_ref_local.x;

    vec3 COM_con_global = local2global_vec(qRF_ref, qLF_ref, COM_con_local);


    return COM_con_global;
}


double HB_PreviewWalk::LTorso_Roll_Vib_Control(vec3 _IMULocalW, bool return_flag){
    ////Torso Vibration Control
    // state : [imu.roll Err, dimu.roll Err]

    vec3 Kc = vec3(-0.7375,    0.0204, 0);
    //vec3 Kc = vec3(-0.5660,    0.0204, 0);
    mat3 Ke = mat3(0.0337,   -0.7314,   0,
                   -0.3657,   37.8342,  0,
                   0,0,0);

    double Wn = 32.7768;
    double zeta = 0.0330;

    // Observer
    mat3 A_obs = mat3(0,  1,  0,
                   -Wn*Wn,-2*zeta*Wn, 0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);

    mat3 C_obs = mat3(1,  0,  0,
                      0,  1,  0,
                      0,  0,  0);


    vec3 y = vec3(0, _IMULocalW.x, 0);

    if(return_flag == false){
        dX_LTorsoRoll = A_obs*X_LTorsoRoll + B_obs*RHR_con_deg*D2R + Ke*(y - X_LTorsoRoll);
        
        X_LTorsoRoll = X_LTorsoRoll + dX_LTorsoRoll*dt;
    }
    else{
        X_LTorsoRoll = X_LTorsoRoll/6;
    }

    

    double alpha = 1/(1 + 2*PI*dt*6.0);
    RHR_con_deg = RHR_con_deg*alpha + dot(Kc,X_LTorsoRoll)*R2D*(1-alpha);

    if(RHR_con_deg > 0.3) RHR_con_deg = 0.3;
    if(RHR_con_deg < -0.3) RHR_con_deg = -0.3;


//    cout<<"R Roll con: "<<RHR_con_deg<<" dRoll measure: "<<RS.IMULocalW.x<<endl;

    return RHR_con_deg;

}

double HB_PreviewWalk::RTorso_Roll_Vib_Control(vec3 _IMULocalW, bool return_flag){
    ////Torso Vibration Control
    // state : [imu.roll Err, dimu.roll Err]

    vec3 Kc = vec3(-0.7375,    0.0204, 0);
    mat3 Ke = mat3(0.0337,   -0.7314,   0,
                   -0.3657,   37.8342,  0,
                   0,0,0);

    double Wn = 32.7768;
    double zeta = 0.0330;

    // Observer
    mat3 A_obs = mat3(0,  1,  0,
                   -Wn*Wn,-2*zeta*Wn, 0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);

    mat3 C_obs = mat3(1,  0,  0,
                      0,  1,  0,
                      0,  0,  0);


    vec3 y = vec3(0, _IMULocalW.x, 0);

    if(return_flag == false){
        dX_RTorsoRoll = A_obs*X_RTorsoRoll + B_obs*LHR_con_deg*D2R + Ke*(y - X_RTorsoRoll);
        
        X_RTorsoRoll = X_RTorsoRoll + dX_RTorsoRoll*dt;
    }
    else{
        X_RTorsoRoll = X_RTorsoRoll/6;
    }

    

    double alpha = 1/(1 + 2*PI*dt*6.0);
    LHR_con_deg = LHR_con_deg*alpha - dot(Kc,X_RTorsoRoll)*R2D*(1-alpha);

    if(LHR_con_deg > 0.3) LHR_con_deg = 0.3;
    if(LHR_con_deg < -0.3) LHR_con_deg = -0.3;


//    cout<<"R Roll con: "<<RHR_con_deg<<" dRoll measure: "<<RS.IMULocalW.x<<endl;

    return -LHR_con_deg;

}

double HB_PreviewWalk::LTorso_Yaw_Vib_Control(vec3 _IMULocalW, bool return_flag){
    ////Torso Yaw Vibration Control
    // state : [imu.Yaw Err, dimu.Yaw Err]

//    vec3 Kc = vec3(-0.9844,    0.0057, 0);
//    mat3 Ke = mat3(8.0000,    0.9950,    0,
//                   -982.8175,    5.8870,  0,
//                   0,0,0);

//    double Wn = 31.35;
//    double zeta = 0.0337;

    vec3 Kc = vec3(-0.9975,    0.0354, 0);
    mat3 Ke = mat3(0.0337,   -0.7314,   0,
                   -0.3657,   37.8342,  0,
                   0,0,0);

    double Wn = 32.7768;
    double zeta = 0.0330;

    // Observer
    mat3 A_obs = mat3(0,  1,  0,
                   -Wn*Wn,-2*zeta*Wn, 0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);

    mat3 C_obs = mat3(1,  0,  0,
                      0,  1,  0,
                      0,  0,  0);


    vec3 y = vec3(0, _IMULocalW.z, 0);

    if(return_flag == false){
        dX_LTorsoYaw = A_obs*X_LTorsoYaw + B_obs*RHY_con_deg*D2R + Ke*(y - X_LTorsoYaw);
        
        X_LTorsoYaw = X_LTorsoYaw + dX_LTorsoYaw*dt;
    }
    else{
        X_LTorsoYaw = X_LTorsoYaw/6;
    }

    

    double alpha = 1.0/(1.0 + 2*PI*dt*6.0);
    RHY_con_deg = RHY_con_deg*alpha - dot(Kc,X_LTorsoYaw)*R2D*(1.0-alpha);

    if(RHY_con_deg > 0.3) RHY_con_deg = 0.3;
    if(RHY_con_deg < -0.3) RHY_con_deg = -0.3;



//    cout<<"R Yaw con: "<<RHY_con_deg<<" dRoll measure: "<<RS.IMULocalW.z<<endl;


    return -RHY_con_deg;

}

double HB_PreviewWalk::RTorso_Yaw_Vib_Control(vec3 _IMULocalW, bool return_flag){
    ////Torso Yaw Vibration Control
    // state : [imu.Yaw Err, dimu.Yaw Err]

    vec3 Kc = vec3(-0.9975,    0.0354, 0);
    mat3 Ke = mat3(0.0337,   -0.7314,   0,
                   -0.3657,   37.8342,  0,
                   0,0,0);

    double Wn = 32.7768;
    double zeta = 0.0330;

    // Observer
    mat3 A_obs = mat3(0,  1,  0,
                   -Wn*Wn,-2*zeta*Wn, 0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);

    mat3 C_obs = mat3(1,  0,  0,
                      0,  1,  0,
                      0,  0,  0);


    vec3 y = vec3(0, _IMULocalW.z, 0);

    if(return_flag == false)
    {
        dX_RTorsoYaw = A_obs*X_RTorsoYaw + B_obs*LHY_con_deg*D2R + Ke*(y - X_RTorsoYaw);
        
        X_RTorsoYaw = X_RTorsoYaw + dX_RTorsoYaw*dt;
    }
    else{
        X_RTorsoYaw = X_RTorsoYaw/6;
    }

    

    double alpha = 1/(1 + 2*PI*dt*6.0);
    LHY_con_deg = LHY_con_deg*alpha - dot(Kc,X_RTorsoYaw)*R2D*(1-alpha);

    if(LHY_con_deg > 0.3) LHY_con_deg = 0.3;
    if(LHY_con_deg < -0.3) LHY_con_deg = -0.3;




    return -LHY_con_deg;

}

double HB_PreviewWalk::LSwingLeg_Vib_Control(vec3 _ACC_LF_filtered, bool return_flag){
    //// Modeling

    double Wn = 35;//31.35;
    double zeta = 0.02;//0.0304;
    double Lleg = 0.6;//0.7; //leg length is 0.7m (HR2foot at walk ready)

    // observer
    mat3 A_obs = mat3(0,  1,  0,
                  -Wn*Wn, -2*zeta*Wn,  0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);
    vec3 C_obs = vec3(-Wn*Wn, -2*zeta*Wn, 0);

    double D_obs = Wn*Wn;


    //// Conroller and Observer Gain
    /// Calc From Matlab (Gazelle_Control_design.m)
    //vec3 Kc = vec3(0,   0.0407, 0); // Controller gain (Pole Placement)
//    vec3 Kc = vec3(-0.9908,    0.0042, 0);
    vec3 Kc = vec3(-0.9908,    0.0022, 0);


    //vec3 Ke = vec3(-0.3147,   0.4929, 0); // Observer gain( Kalman Filter)
//    vec3 Ke = vec3(-0.0143, 0.9622, 0);  // Pole Placement (-8+0.001i , -8-0.001i)
    vec3 Ke = vec3(-0.0087, 0.9827, 0);//


    //// State Observer
    // State
    // L_roll_obs = [Foot y position error    Foot y position error dot]'

    // Measurment
    double LF_ACC = _ACC_LF_filtered.y;
    double u = LHR_con_deg*D2R*Lleg;// input unit is position(m)

    // Observer
    if(return_flag == false){
        dL_roll_obs = A_obs*L_roll_obs + B_obs*u + Ke*(LF_ACC - dot(C_obs,L_roll_obs) - D_obs*u);
        
        L_roll_obs = L_roll_obs + dL_roll_obs*dt;
    }
    else{
        L_roll_obs = L_roll_obs/6.0;
    }

    double input_to_HR_deg = dot(Kc,L_roll_obs)/Lleg*R2D;
    

    double alpha = 1.0/(1.0 + 2.0*PI*dt*15.0);
    LHR_con_deg = alpha*LHR_con_deg + (1.0 - alpha)*input_to_HR_deg;

    if(LHR_con_deg > 0.4) LHR_con_deg = 0.4;
    if(LHR_con_deg < -0.4) LHR_con_deg = -0.4;


//    LHR_con_deg = -dot(Kc,L_roll_obs)*R2D;

//    cout<<"L Roll con: "<<LHR_con_deg<<endl;

//    if(LHR_con_deg > 0.2) LHR_con_deg = 0.2;
//    if(LHR_con_deg < -0.2) LHR_con_deg = -0.2;


    return +LHR_con_deg;

}


double HB_PreviewWalk::RSwingLeg_Vib_Control(vec3 _ACC_RF_filtered, bool return_flag){
    //// Modeling

    double Wn = 29.6212;
    double zeta = 0.0352;
    double Lleg = 0.7; //leg length is 0.7m (HR2foot at walk ready)

    // observer
    mat3 A_obs = mat3(0,  1,  0,
                  -Wn*Wn, -2*zeta*Wn,  0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);
    vec3 C_obs = vec3(-Wn*Wn, -2*zeta*Wn, 0);

    double D_obs = Wn*Wn;


    //// Conroller and Observer Gain
    /// Calc From Matlab (Gazelle_Control_design.m)
    //vec3 Kc = vec3(0,   0.0407, 0); // Controller gain (Pole Placement)
    vec3 Kc = vec3(-0.9837,    0.0062, 0);


    //vec3 Ke = vec3(-0.3147,   0.4929, 0); // Observer gain( Kalman Filter)
    vec3 Ke = vec3(-0.0159, 0.9601, 0);  // Pole Placement (-8+0.001i , -8-0.001i)


    //// State Observer
    // State
    // L_roll_obs = [Foot y position error    Foot y position error dot]'

    // Measurment
    double RF_ACC = _ACC_RF_filtered.y*0.8;
    double u = RHR_con_deg*D2R*Lleg;// input unit is position(m)


    // Observer
    if(return_flag == false)
    {
        dR_roll_obs = A_obs*R_roll_obs + B_obs*u + Ke*(RF_ACC - dot(C_obs,R_roll_obs) - D_obs*u);

        R_roll_obs = R_roll_obs + dR_roll_obs*dt;
    }
    else{
        R_roll_obs = R_roll_obs/6.0;
    }

    double input_to_HR_deg = dot(Kc,R_roll_obs)/Lleg*R2D;


    double alpha = 1.0/(1.0 + 2.0*PI*dt*3.0);
    RHR_con_deg = alpha*RHR_con_deg + (1.0 - alpha)*input_to_HR_deg;

    if(RHR_con_deg > 0.2) RHR_con_deg = 0.2;
    if(RHR_con_deg < -0.2) RHR_con_deg = -0.2;


//    LHR_con_deg = -dot(Kc,L_roll_obs)*R2D;

//    cout<<"R Roll con: "<<RHR_con_deg<<endl;

//    if(LHR_con_deg > 0.2) LHR_con_deg = 0.2;
//    if(LHR_con_deg < -0.2) LHR_con_deg = -0.2;


    return RHR_con_deg;
}

double HB_PreviewWalk::LSwingLeg_Pitch_Vib_Control(vec3 _ACC_LF_filtered, bool return_flag){
    //// Modeling

    double Wn = 32.0354;
    double zeta = 0.0371;

    // observer
    mat3 A_obs = mat3(0,  1,  0,
                  -Wn*Wn, -2*zeta*Wn,  0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);
    vec3 C_obs = vec3(-Wn*Wn, 0, 0);

    double D_obs = Wn*Wn;

    mat3 A_return = mat3(-1/2, 0, 0,
                         0, -1/2, 0,
                         0,  0,   0);

    //// Conroller and Observer Gain
    /// Calc From Matlab (Gazelle_Control_design.m)
    //vec3 Kc = vec3(0,   0.0407, 0); // Controller gain (Pole Placement)
    vec3 Kc = vec3(-0.9961,    0.0066, 0);
    //vec3 Ke = vec3(-0.3147,   0.4929, 0); // Observer gain( Kalman Filter)
    vec3 Ke = vec3(-0.0154,   0.9405, 0);  // Pole Placement


    //// State Observer
    // State
    // L_roll_obs = [HR_Err    HR_vel_Err]'

    // Measurment
    double HR_ACC = _ACC_LF_filtered.x/0.8;

    // Observer
    if(return_flag == false){
        dL_pitch_obs = A_obs*L_pitch_obs + B_obs*LHP_con_deg*D2R + Ke*(HR_ACC - dot(C_obs,L_pitch_obs) - D_obs*LHP_con_deg*D2R);

        L_pitch_obs = L_pitch_obs + dL_pitch_obs*dt;
    }
    else{
        L_pitch_obs = L_pitch_obs/6;
    }

    double alpha = 1/(1 + 2*PI*dt*6.0);
    LHP_con_deg = LHP_con_deg*alpha - dot(Kc,L_pitch_obs)*R2D*(1-alpha);



    if(LHP_con_deg > 0.2) LHP_con_deg = 0.2;
    if(LHP_con_deg < -0.2) LHP_con_deg = -0.2;


//    LHR_con_deg = -dot(Kc,L_roll_obs)*R2D;

//    cout<<"L pitch con: "<<LHP_con_deg<<endl;

//    if(LHR_con_deg > 0.2) LHR_con_deg = 0.2;
//    if(LHR_con_deg < -0.2) LHR_con_deg = -0.2;


    return LHP_con_deg;

}

double HB_PreviewWalk::RSwingLeg_Pitch_Vib_Control(vec3 _ACC_RF_filtered, bool return_flag){
    //// Modeling

    double Wn = 32.0354;
    double zeta = 0.0371;

    // observer
    mat3 A_obs = mat3(0,  1,  0,
                  -Wn*Wn, -2*zeta*Wn,  0,
                   0,  0,  0);
    vec3 B_obs = vec3(0, Wn*Wn, 0);
    vec3 C_obs = vec3(-Wn*Wn, 0, 0);

    double D_obs = Wn*Wn;

    mat3 A_return = mat3(-1/10, 0, 0,
                         0, -1/10, 0,
                         0,  0,   0);

    //// Conroller and Observer Gain
    /// Calc From Matlab (Gazelle_Control_design.m)
    //vec3 Kc = vec3(0,   0.0407, 0); // Controller gain (Pole Placement)
    vec3 Kc = vec3(-0.9961,    0.0066, 0);
    //vec3 Ke = vec3(-0.3147,   0.4929, 0); // Observer gain( Kalman Filter)
    vec3 Ke = vec3(-0.0154,   0.9405, 0);  // Pole Placement


    //// State Observer
    // State
    // L_roll_obs = [HR_Err    HR_vel_Err]'

    // Measurment
    double HR_ACC = _ACC_RF_filtered.x/0.8;

    // Observer
    if(return_flag == false){
        dR_pitch_obs = A_obs*R_pitch_obs + B_obs*RHP_con_deg*D2R + Ke*(HR_ACC - dot(C_obs,R_pitch_obs) - D_obs*RHP_con_deg*D2R);
        
        R_pitch_obs = R_pitch_obs + dR_pitch_obs*dt;

    }
    else{
        R_pitch_obs = R_pitch_obs/6;
    }

    

    double alpha = 1/(1 + 2*PI*dt*6.0);
    RHP_con_deg = RHP_con_deg*alpha - dot(Kc,R_pitch_obs)*R2D*(1-alpha);

    if(RHP_con_deg > 0.2) RHP_con_deg = 0.2;
    if(RHP_con_deg < -0.2) RHP_con_deg = -0.2;


//    LHR_con_deg = -dot(Kc,L_roll_obs)*R2D;

//    cout<<"L pitch con: "<<RHP_con_deg<<endl;

//    if(LHR_con_deg > 0.2) LHR_con_deg = 0.2;
//    if(LHR_con_deg < -0.2) LHR_con_deg = -0.2;


    return RHP_con_deg;

}

void HB_PreviewWalk::R_Vib_Control_init(){
    R_roll_obs = vec3();
    X_RTorsoRoll = vec3();
    dX_RTorsoRoll = vec3();
    X_RTorsoYaw = vec3();
    dX_RTorsoYaw = vec3();
    R_pitch_obs = vec3();
    dR_pitch_obs = vec3();
}

void HB_PreviewWalk::L_Vib_Control_init(){
    L_roll_obs = vec3();
    X_LTorsoRoll = vec3();
    dX_LTorsoRoll = vec3();
    X_LTorsoYaw = vec3();
    dX_LTorsoYaw = vec3();
    L_pitch_obs = vec3();
    dL_pitch_obs = vec3();
}

void HB_PreviewWalk::R_SSP_ZMP_Control(vec3 _ZMP_des_local, vec3 _ZMP_measure_local, vec3 _FT_F, vec3 _FT_M){
    vec3 Torque_des;
    Torque_des.x = _ZMP_des_local.y*_FT_F.z;
    Torque_des.y = -_ZMP_des_local.x*_FT_F.z;

    vec3 Torque_error;
    Torque_error = Torque_des - _FT_M;

    double Px_gain = 0.001;
    double Py_gain = 0.001;

    vec3 wind_back = vec3();
    if(_FT_F.z < 5){
        wind_back = -RF_del_angle/50;
    }
    else{
        wind_back = vec3();
    }



    RF_del_angle.x = RF_del_angle.x - Px_gain*Torque_error.x; //rad
    RF_del_angle.y = RF_del_angle.y - Py_gain*Torque_error.y; //rad

    double alpha = 1/(1 + 2*3.14*dt*2);
    RF_del_angle = alpha*RF_del_angle_old +(1-alpha)*RF_del_angle + wind_back;
    RF_del_angle_old = RF_del_angle;


    double angle_limit = 10; //deg
    if(RF_del_angle.x > angle_limit*D2R) RF_del_angle.x = angle_limit*D2R;
    if(RF_del_angle.x < -angle_limit*D2R) RF_del_angle.x = -angle_limit*D2R;
    if(RF_del_angle.y > angle_limit*D2R) RF_del_angle.y = angle_limit*D2R;
    if(RF_del_angle.y < -angle_limit*D2R) RF_del_angle.y = -angle_limit*D2R;

    RF_del_quat = quat(vec3(0,1,0),RF_del_angle.y)*quat(vec3(1,0,0),RF_del_angle.x);

    RF_del_pos.x = SIGN(RF_del_angle.y)*0.0*(1 - cos(RF_del_angle.y));
    RF_del_pos.y = SIGN(RF_del_angle.x)*0.0*(1 - cos(RF_del_angle.x));
    RF_del_pos.z = SIGN((RF_del_angle.x + RF_del_angle.y)/2)*0.00*sin((RF_del_angle.x + RF_del_angle.y)/2);

}

void HB_PreviewWalk::L_SSP_ZMP_Control(vec3 _ZMP_des_local, vec3 _ZMP_measure_local, vec3 _FT_F, vec3 _FT_M){
    vec3 Torque_des;
    Torque_des.x = _ZMP_des_local.y*_FT_F.z;
    Torque_des.y = -_ZMP_des_local.x*_FT_F.z;

    vec3 Torque_error;
    Torque_error = Torque_des - _FT_M;

    double Px_gain = 0.001;
    double Py_gain = 0.001;

    vec3 wind_back = vec3();
    if(_FT_F.z < 5){
        wind_back = -LF_del_angle/50;
    }
    else{
        wind_back = vec3();
    }



    LF_del_angle.x = LF_del_angle.x - Px_gain*Torque_error.x; //rad
    LF_del_angle.y = LF_del_angle.y - Py_gain*Torque_error.y; //rad

    double alpha = 1/(1 + 2*3.14*dt*2);
    LF_del_angle = alpha*LF_del_angle_old +(1-alpha)*LF_del_angle + wind_back;
    LF_del_angle_old = LF_del_angle;


    double angle_limit = 10; //deg
    if(LF_del_angle.x > angle_limit*D2R) LF_del_angle.x = angle_limit*D2R;
    if(LF_del_angle.x < -angle_limit*D2R) LF_del_angle.x = -angle_limit*D2R;
    if(LF_del_angle.y > angle_limit*D2R) LF_del_angle.y = angle_limit*D2R;
    if(LF_del_angle.y < -angle_limit*D2R) LF_del_angle.y = -angle_limit*D2R;

    LF_del_quat = quat(vec3(0,1,0),LF_del_angle.y)*quat(vec3(1,0,0),LF_del_angle.x);

    LF_del_pos.x = SIGN(LF_del_angle.y)*0.0*(1 - cos(LF_del_angle.y));
    LF_del_pos.y = SIGN(LF_del_angle.x)*0.0*(1 - cos(LF_del_angle.x));
    LF_del_pos.z = SIGN((LF_del_angle.x + LF_del_angle.y)/2)*0.00*sin((LF_del_angle.x + LF_del_angle.y)/2);

}
void HB_PreviewWalk::DSP_GRF_Control(vec3 _ZMP_ref, vec3 _pRF, vec3 _pLF, quat _qRF, quat _qLF,
                                     vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF){
    if(_F_RF.z <= 0) _F_RF.z = 0;
    if(_F_LF.z <= 0) _F_LF.z = 0;

    // Calculate Alpha
    // get P_alpha : closest point on line P_LF & P_RF from _ZMP_ref
    vec3 n = vec3(_pLF.x - _pRF.x, _pLF.y - _pRF.y, 0);  //RF2LF
    vec3 p = vec3(_ZMP_ref.x - _pRF.x, _ZMP_ref.y - _pRF.y, 0);  // RF2ZMP

    vec3 p2n_proj = n*(dot(p,n)/dot(n)); // alpha spot


    // get alpha
    double Alpha = p2n_proj.norm()/n.norm();  // alpha 0  --> only RF

    double total_mass = 313;// newton



    // Calculate Sum of reference Torque Tr + Tl
    vec3 SumOfTorque_global = -cross((_pRF - _ZMP_ref), mat3(_qRF)*_F_RF) - cross((_pLF - _ZMP_ref), mat3(_qLF)*_F_LF);

    vec3 LF2RF = _pRF - _pLF;
    LF2RF.z = 0;

    double TF_angle = atan(LF2RF.x/(-LF2RF.y)); // return -pi/2 ~ pi/2  in radian

    vec3 SumOfTorque_local = vec3(0,0,0);
    SumOfTorque_local.x = SumOfTorque_global.x*cos(TF_angle) + SumOfTorque_global.y*sin(TF_angle);
    SumOfTorque_local.y = -SumOfTorque_global.x*sin(TF_angle) + SumOfTorque_global.y*cos(TF_angle);

    // Distribute SumOfTorque_local (local means ... T' in Kajita's paper)
    vec3 TorqueRF_local, TorqueLF_local, TorqueRF_global, TorqueLF_global;
    TorqueRF_local.y = (1.0 - Alpha)*SumOfTorque_local.y;
    TorqueLF_local.y = Alpha*SumOfTorque_local.y;

    TorqueRF_local.x = (1.0 - Alpha)*SumOfTorque_local.x;
    TorqueLF_local.x = Alpha*SumOfTorque_local.x;

    // calc each foot torque in global frame
    TorqueRF_global.x = TorqueRF_local.x*cos(-TF_angle) + TorqueRF_local.y*sin(-TF_angle);
    TorqueRF_global.y = -TorqueRF_local.x*sin(-TF_angle) + TorqueRF_local.y*cos(-TF_angle);

    TorqueLF_global.x = TorqueLF_local.x*cos(-TF_angle) + TorqueLF_local.y*sin(-TF_angle);
    TorqueLF_global.y = -TorqueLF_local.x*sin(-TF_angle) + TorqueLF_local.y*cos(-TF_angle);

    vec3 TorqueRF = _qRF*TorqueRF_global;
    vec3 TorqueLF = _qLF*TorqueLF_global;

    double RFz_des = total_mass*(1.0 - Alpha);
    double LFz_des = total_mass*Alpha;

    vec3 R_ZMP_des_local = vec3(-TorqueRF.y, TorqueRF.x ,0)/RFz_des;
    vec3 L_ZMP_des_local = vec3(-TorqueLF.y, TorqueLF.x ,0)/LFz_des;


    R_SSP_ZMP_Control(R_ZMP_des_local,vec3(), _F_RF, _M_RF);
    L_SSP_ZMP_Control(L_ZMP_des_local,vec3(), _F_LF, _M_LF);


}

vec4 HB_PreviewWalk::Ankle_Torque_from_cZMP(vec3 _global_cZMP, vec3 _global_ZMP, quat _global_qRF, quat _global_qLF,
                                            vec3 _global_pRF, vec3 _global_pLF, vec3 _F_RF, vec3 _F_LF){
    //// Torque Distribution (Kajita's Paper ref)
    // localization of parameters
    vec3 local_cZMP = global2local_point(_global_qRF, _global_qLF, _global_pRF, _global_pLF, _global_cZMP);

    vec3 local_ZMP = global2local_point(_global_qRF, _global_qLF,  _global_pRF, _global_pLF, _global_ZMP);
    vec3 local_pRF = global2local_point(_global_qRF, _global_qLF, _global_pRF, _global_pLF,_global_pRF);
    vec3 local_pLF = global2local_point(_global_qRF, _global_qLF, _global_pRF, _global_pLF,_global_pLF);

    quat local_qRF = global2local_quat(_global_qRF, _global_qLF, _global_qRF);
    quat local_qLF = global2local_quat(_global_qRF, _global_qLF, _global_qLF);

    vec3 local_F_RF = local_qRF*_F_RF;
    vec3 local_F_LF = local_qLF*_F_LF;

    vec3 local_SumOfTorque;


    if(_F_RF.z <= 15) _F_RF.z = 0;
    if(_F_LF.z <= 15) _F_LF.z = 0;
    Alpha_dsp = 0.5;
    double Alpha2 = 0.5;


    Alpha2 = 0.5;

    double _Xoffset = 0;
    double _Yoffset = +0.022;

    // get support polygon data for local frame
    vec3 LF_lt1 = vec3((FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);
    vec3 LF_rt2 = vec3((FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 LF_rb3 = vec3(-(FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 LF_lb4 = vec3(-(FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);

    vec3 RF_lt1 = vec3((FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);
    vec3 RF_rt2 = vec3((FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 RF_rb3 = vec3(-(FootLength+_Xoffset)/2, -(FootWidth+_Yoffset)/2, 0);
    vec3 RF_lb4 = vec3(-(FootLength+_Xoffset)/2, (FootWidth+_Yoffset)/2, 0);

    LF_lt1 = mat3(local_qLF)*LF_lt1;
    LF_rt2 = mat3(local_qLF)*LF_rt2;
    LF_rb3 = mat3(local_qLF)*LF_rb3;
    LF_lb4 = mat3(local_qLF)*LF_lb4;

    RF_lt1 = mat3(local_qRF)*RF_lt1;
    RF_rt2 = mat3(local_qRF)*RF_rt2;
    RF_rb3 = mat3(local_qRF)*RF_rb3;
    RF_lb4 = mat3(local_qRF)*RF_lb4;

    LF_lt1 = LF_lt1 + local_pLF;
    LF_rt2 = LF_rt2 + local_pLF;
    LF_rb3 = LF_rb3 + local_pLF;
    LF_lb4 = LF_lb4 + local_pLF;

    RF_lt1 = RF_lt1 + local_pRF;
    RF_rt2 = RF_rt2 + local_pRF;
    RF_rb3 = RF_rb3 + local_pRF;
    RF_lb4 = RF_lb4 + local_pRF;

    stack<Point> SP_LF;
    SP_LF.push({LF_lt1.x, LF_lt1.y});
    SP_LF.push({LF_rt2.x, LF_rt2.y});
    SP_LF.push({LF_rb3.x, LF_rb3.y});
    SP_LF.push({LF_lb4.x, LF_lb4.y});

    stack<Point> SP_RF;
    SP_RF.push({RF_lt1.x, RF_lt1.y});
    SP_RF.push({RF_rt2.x, RF_rt2.y});
    SP_RF.push({RF_rb3.x, RF_rb3.y});
    SP_RF.push({RF_lb4.x, RF_lb4.y});

    Point P_local_cZMP = {local_cZMP.x, local_cZMP.y};


    if(zmpInOutCheck(P_local_cZMP, SP_LF) == true){
        Alpha_AT = 0;        // cZMP is in LF
        local_SumOfTorque = - cross(local_pLF - local_cZMP, local_F_LF);

    }
    else if(zmpInOutCheck(P_local_cZMP, SP_RF) == true){
        Alpha_AT = 1;   // cZMP is in RF
        local_SumOfTorque = -cross(local_pRF - local_cZMP, local_F_RF);
    }
    else{                                                           // cZMP is between LF & RF

        local_SumOfTorque = -cross(local_pRF - local_cZMP, local_F_RF) - cross(local_pLF - local_cZMP, local_F_LF);

        // get Alpha
        vec3 n = vec3(local_pLF.x - local_pRF.x, local_pLF.y - local_pRF.y, 0);
        vec3 p = vec3(local_cZMP.x - local_pRF.x, local_cZMP.y - local_pRF.y, 0);

        vec3 p2n_proj = n*(dot(p,n)/dot(n));

        vec3 P_alpha = vec3(local_pRF.x + p2n_proj.x, local_pRF.y + p2n_proj.y,0);

        Alpha_AT = sqrt(distSq({local_pLF.x, local_pLF.y},{P_alpha.x, P_alpha.y}))/sqrt(distSq({local_pLF.x, local_pLF.y},{local_pRF.x, local_pRF.y}));
    }
    if(Alpha_AT >= 1.0) Alpha_AT = 1.0;
    if(Alpha_AT < 0) Alpha_AT = 0;


    vec3 TorqueRF, TorqueLF, local_TorqueRF, local_TorqueLF;



    //cout<<"Alpha_AT : "<<Alpha_AT<<endl;

    if(_F_RF.z > 20 && _F_LF.z > 20){  // When both feet are on the ground (DSP)
        if(local_SumOfTorque.x <= 0){
            local_TorqueRF.x = 0;
            local_TorqueLF.x = -local_SumOfTorque.x;
        }
        else{
            local_TorqueRF.x = -local_SumOfTorque.x;
            local_TorqueLF.x = 0;
        }
    }

    local_TorqueRF.y = Alpha_AT*local_SumOfTorque.y;
    local_TorqueLF.y = (1.0 - Alpha_AT)*local_SumOfTorque.y;

    local_TorqueRF.x = Alpha_AT*local_SumOfTorque.x;
    local_TorqueLF.x = (1.0 - Alpha_AT)*local_SumOfTorque.x;



    // Calculate torque of each foot joint

    TorqueRF = inverse_HB(mat3(local_qRF))*local_TorqueRF;
    TorqueLF = inverse_HB(mat3(local_qLF))*local_TorqueLF;


    // Input Torque Limitation (only inContact)
    // for rigid foot
//    double F_width_outer = 0.11;
//    double F_width_inner = 0.075;
//    double F_length = 0.23;

    // for damping foot
    double F_width_outer = 0.12;
    double F_width_inner = 0.15;
    double F_length = 0.35;

    if(_F_RF.z > -10)
    {
        //RAR torque
        if(TorqueRF.x > F_width_inner*_F_RF.z) TorqueRF.x = F_width_inner*_F_RF.z;
        if(TorqueRF.x < -F_width_outer*_F_RF.z) TorqueRF.x = -F_width_outer*_F_RF.z;

        //RAP torque
        if(TorqueRF.y > F_length*_F_RF.z/2) TorqueRF.y = F_length*_F_RF.z/2;
        if(TorqueRF.y < -F_length*_F_RF.z/2) TorqueRF.y = -F_length*_F_RF.z/2;

    }
    if(_F_LF.z > -10)
    {
        //LAR torque
        if(TorqueLF.x > F_width_outer*_F_LF.z) TorqueLF.x  = F_width_outer*_F_LF.z;
        if(TorqueLF.x  < -F_width_inner*_F_LF.z) TorqueLF.x  = -F_width_inner*_F_LF.z;


        //LAP torque
        if(TorqueLF.y > F_length*_F_LF.z/2) TorqueLF.y = F_length*_F_LF.z/2;
        if(TorqueLF.y < -F_length*_F_LF.z/2) TorqueLF.y = -F_length*_F_LF.z/2;

    }

    return vec4(TorqueRF.x, TorqueRF.y, TorqueLF.x, TorqueLF.y);

}

vec4 HB_PreviewWalk::Ankle_Torque_from_PositionFB(int _ctrl_mode, vec3 _F_RF, vec3 _F_LF,
                      double _RAP_ref_deg, double _RAR_ref_deg, double _LAP_ref_deg, double _LAR_ref_deg,
                      double _RAP_cur_deg, double _RAR_cur_deg, double _LAP_cur_deg, double _LAR_cur_deg,
                      double _RAP_vel_ref_deg, double _RAR_vel_ref_deg, double _LAP_vel_ref_deg, double _LAR_vel_ref_deg,
                      double _RAP_vel_cur_deg, double _RAR_vel_cur_deg, double _LAP_vel_cur_deg, double _LAR_vel_cur_deg){

    vec4 Feedback_Joint_Torque;

    if(_ctrl_mode == 0){
      Feedback_Joint_Torque[0] = 0;
      Feedback_Joint_Torque[1] = 0;
      Feedback_Joint_Torque[2] = 0;
      Feedback_Joint_Torque[3] = 0;
      return Feedback_Joint_Torque;
    }

    else if(_ctrl_mode == 1){  // Always Position Control Mode
      double Pgain_pos = 2.0;
      double Dgain_pos = 0.05;
      Feedback_Joint_Torque[0] = Pgain_pos*(_RAR_ref_deg - _RAR_cur_deg) + Dgain_pos*(_RAR_vel_ref_deg - _RAR_vel_cur_deg);
      Feedback_Joint_Torque[1] = 3/2*Pgain_pos*(_RAP_ref_deg - _RAP_cur_deg) + Dgain_pos*(_RAP_vel_ref_deg - _RAP_vel_cur_deg);
      Feedback_Joint_Torque[2] = Pgain_pos*(_LAR_ref_deg - _LAR_cur_deg) + Dgain_pos*(_LAR_vel_ref_deg - _LAR_vel_cur_deg);
      Feedback_Joint_Torque[3] = 3/2*Pgain_pos*(_LAP_ref_deg - _LAP_cur_deg) + Dgain_pos*(_LAP_vel_ref_deg - _LAP_vel_cur_deg);

      return Feedback_Joint_Torque;
    }

    else if(_ctrl_mode == 2){ // arial phase position control
      double low_R_Pgain_roll;
      double low_R_Dgain_roll;
      double low_R_Pgain_pitch;
      double low_R_Dgain_pitch;

      double low_L_Pgain_roll;
      double low_L_Dgain_roll;
      double low_L_Pgain_pitch;
      double low_L_Dgain_pitch;

      double FF_R_gain, FF_L_gain;

      if(_F_RF.z <= 20){ // LF SSP phase
          // Right ankle need position control to maintain its orientation in aireal phase
          low_R_Pgain_roll = 2;//1.5;//10;
          low_R_Dgain_roll = 0.1;  // low gain in arial phase
          low_R_Pgain_pitch = 2;//1.5;
          low_R_Dgain_pitch = 0.1;
          FF_R_gain = 0.0;

      }
      else{ // in contact, no position control
          low_R_Pgain_roll = 0.0;
          low_R_Dgain_roll = 0.01;
          low_R_Pgain_pitch =0.0;
          low_R_Dgain_pitch =0.01;
          FF_R_gain = 0.0;
      }
      double FF_RAR = _RAR_vel_ref_deg*FF_R_gain;
      double FF_RAP = _RAP_vel_ref_deg*FF_R_gain;

      Feedback_Joint_Torque[0] = FF_RAR + low_R_Pgain_roll*(_RAR_ref_deg - _RAR_cur_deg) + low_R_Dgain_roll*(_RAR_vel_ref_deg - _RAR_vel_cur_deg);
      Feedback_Joint_Torque[1] = FF_RAP + low_R_Pgain_pitch*(_RAP_ref_deg - _RAP_cur_deg) + low_R_Dgain_pitch*(_RAP_vel_ref_deg - _RAP_vel_cur_deg);

      if(_F_LF.z <= 20){ // RF SSP phase
          // Left ankle need position control to maintain its orientation in aireal phase
          low_L_Pgain_roll = 2;//1.5;
          low_L_Dgain_roll = 0.1;  // low gain in arial phase
          low_L_Pgain_pitch = 2;//1.5;
          low_L_Dgain_pitch = 0.1;  // low gain in arial phase
          FF_L_gain = 0.0;
      }
      else{
          low_L_Pgain_roll = 0.0;
          low_L_Dgain_roll = 0.01;
          low_L_Pgain_pitch =0.0;
          low_L_Dgain_pitch =0.01;
          FF_L_gain = 0.0;
      }
      double FF_LAR = _LAR_vel_ref_deg*FF_L_gain;
      double FF_LAP = _LAP_vel_ref_deg*FF_L_gain;

      Feedback_Joint_Torque[2] = FF_LAR + low_L_Pgain_roll*(_LAR_ref_deg - _LAR_cur_deg) + low_L_Dgain_roll*(_LAR_vel_ref_deg - _LAR_vel_cur_deg);
      Feedback_Joint_Torque[3] = FF_LAP + low_L_Pgain_pitch*(_LAP_ref_deg - _LAP_cur_deg) + low_L_Dgain_pitch*(_LAP_vel_ref_deg - _LAP_vel_cur_deg);
    }
    return Feedback_Joint_Torque;

}

void HB_PreviewWalk::AnkleTorqueController(double _RAR_T_ref, double _RAP_T_ref, double _LAR_T_ref, double _LAP_T_ref,
                                           double _RA1_ang_deg, double _RA2_ang_deg, double _LA1_ang_deg, double _LA2_ang_deg,
                                           double _RAP_ang_deg, double _RAR_ang_deg, double _LAP_ang_deg, double _LAR_ang_deg,
                                           double _RA1_vel_deg, double _RA2_vel_deg, double _LA1_vel_deg, double _LA2_vel_deg,
                                           vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF){
    //// Gains
    double Pgain_R_roll;
    double Dgain_R_roll;
    double Igain_R_roll;

    double Pgain_R_pitch;
    double Dgain_R_pitch;
    double Igain_R_pitch;

    double Pgain_L_roll;
    double Dgain_L_roll;
    double Igain_L_roll;

    double Pgain_L_pitch;
    double Dgain_L_pitch;
    double Igain_L_pitch;

    // static variable for integral control
    static double _E_RAR_old = 0;
    static double _E_RAP_old = 0;
    static double _E_LAR_old = 0;
    static double _E_LAP_old = 0;

    static double _E_RAR_sum = 0;
    static double _E_RAP_sum = 0;
    static double _E_LAR_sum = 0;
    static double _E_LAP_sum = 0;

    static double F_RF_z_filtered = 0;
    static double F_LF_z_filtered = 0;

    double FF_RAR_T_ref, FF_RAP_T_ref, FF_LAR_T_ref, FF_LAP_T_ref;

    double alpha_fz = 1/(1 + 2*PI*dt*4);
    F_RF_z_filtered = alpha_fz*F_RF_z_filtered + (1.0 - alpha_fz)*_F_RF.z;

    F_LF_z_filtered = alpha_fz*F_LF_z_filtered + (1.0 - alpha_fz)*_F_LF.z;


    //// FT sensor Feedback only in Contact
    if(F_RF_z_filtered > 50.0){
        double gain_vari = 330/(330 + F_RF_z_filtered);
        Pgain_R_roll = 16.4*gain_vari;
        Dgain_R_roll = 0.000*gain_vari;
        Igain_R_roll = 0.02*gain_vari;

        Pgain_R_pitch = 14.0*gain_vari;
        Dgain_R_pitch = 0.000*gain_vari;//
        Igain_R_pitch = 0.015*gain_vari;

        FF_RAR_T_ref = _RAR_T_ref/2.5;
        FF_RAP_T_ref = _RAP_T_ref/2.5;

    }
    else{
        Pgain_R_roll = 0;
        Dgain_R_roll = 0;
        Igain_R_roll = 0;

        Pgain_R_pitch = 0;
        Dgain_R_pitch = 0;
        Igain_R_pitch = 0;

        FF_RAR_T_ref = _RAR_T_ref;
        FF_RAP_T_ref = _RAP_T_ref;

        _E_RAR_sum -= _E_RAR_sum/30.0;
        _E_RAP_sum -= _E_RAP_sum/30.0;
    }

    if(F_LF_z_filtered > 50.0){
        double gain_vari = 330/(330 + F_LF_z_filtered);
        Pgain_L_roll = 16.4*gain_vari;
        Dgain_L_roll = 0.000*gain_vari;
        Igain_L_roll = 0.02*gain_vari;

        Pgain_L_pitch = 14.0*gain_vari;
        Dgain_L_pitch = 0.000*gain_vari;//
        Igain_L_pitch = 0.015*gain_vari;

        FF_LAR_T_ref = _LAR_T_ref/2.5;
        FF_LAP_T_ref = _LAP_T_ref/2.5;
    }
    else{
        Pgain_L_roll = 0;
        Dgain_L_roll = 0;
        Igain_L_roll = 0;

        Pgain_L_pitch = 0;
        Dgain_L_pitch = 0;
        Igain_L_pitch = 0;

        FF_LAR_T_ref = _LAR_T_ref;
        FF_LAP_T_ref = _LAP_T_ref;

        _E_LAR_sum -= _E_LAR_sum/30.0;
        _E_LAP_sum -= _E_LAP_sum/30.0;
    }

    double _E_RAR_cur = _RAR_T_ref - _M_RF.x;
    double _E_RAP_cur = _RAP_T_ref - _M_RF.y;
    double _E_LAR_cur = _LAR_T_ref - _M_LF.x;
    double _E_LAP_cur = _LAP_T_ref - _M_LF.y;
    double dE_RAR_cur = (_E_RAR_cur - _E_RAR_old)/dt;
    double dE_RAP_cur = (_E_RAP_cur - _E_RAP_old)/dt;
    double dE_LAR_cur = (_E_LAR_cur - _E_LAR_old)/dt;
    double dE_LAP_cur = (_E_LAP_cur - _E_LAP_old)/dt;

    _E_RAR_sum += _E_RAR_cur;
    _E_RAP_sum += _E_RAP_cur;
    _E_LAR_sum += _E_LAR_cur;
    _E_LAP_sum += _E_LAP_cur;

    _E_RAR_old = _E_RAR_cur;
    _E_RAP_old = _E_RAP_cur;
    _E_LAR_old = _E_LAR_cur;
    _E_LAP_old = _E_LAP_cur;

    double FB_RAR_T_ref, FB_RAP_T_ref, FB_LAR_T_ref, FB_LAP_T_ref;

    FB_RAR_T_ref = _E_RAR_cur*Pgain_R_roll + dE_RAR_cur*Dgain_R_roll + _E_RAR_sum*Igain_R_roll;
    FB_RAP_T_ref = _E_RAP_cur*Pgain_R_pitch + dE_RAP_cur*Dgain_R_pitch + _E_RAP_sum*Igain_R_pitch;
    FB_LAR_T_ref = _E_LAR_cur*Pgain_L_roll + dE_LAR_cur*Dgain_L_roll + _E_LAR_sum*Igain_L_roll;
    FB_LAP_T_ref = _E_LAP_cur*Pgain_L_pitch + dE_LAP_cur*Dgain_L_pitch + _E_LAP_sum*Igain_L_pitch;

    vec4 temp_input;
    temp_input[0] = (FF_RAR_T_ref + FB_RAR_T_ref); //RAR
    temp_input[1] = (FF_RAP_T_ref + FB_RAP_T_ref); //RAP
    temp_input[2] = (FF_LAR_T_ref + FB_LAR_T_ref); //LAR
    temp_input[3] = (FF_LAP_T_ref + FB_LAP_T_ref); //LAP


    // Input Torque Limitation (only inContact)
    double F_width_outer = 0.10;
    double F_width_inner = 0.075;
    double F_length = 0.23;
    double length = 0.05;
    if(F_RF_z_filtered > 20){
        //RAR torque
        if(temp_input[0] > F_width_inner*_F_RF.z) temp_input[0] = F_width_inner*_F_RF.z;
        if(temp_input[0] < -F_width_outer*_F_RF.z) temp_input[0] = -F_width_outer*_F_RF.z;

        //RAP torque
        if(temp_input[1] > F_length*_F_RF.z/2) temp_input[1] = F_length*_F_RF.z/2;
        if(temp_input[1] < -F_length*_F_RF.z/2) temp_input[1] = -F_length*_F_RF.z/2;

//        double T_abs = sqrt(temp_input[0]*temp_input[0] + temp_input[1]*temp_input[1]);
//        if( T_abs > length*_F_RF.z){
//            temp_input[0] = temp_input[0]*length*_F_RF.z/T_abs;
//            temp_input[1] = temp_input[1]*length*_F_RF.z/T_abs;
//            cout<<"cutcut!!   "<<T_abs<<endl;
//        }
    }
    if(F_LF_z_filtered > 20){
        //LAR torque
        if(temp_input[2] > F_width_outer*_F_LF.z) temp_input[2] = F_width_outer*_F_LF.z;
        if(temp_input[2] < -F_width_inner*_F_LF.z) temp_input[2] = -F_width_inner*_F_LF.z;


        //LAP torque
        if(temp_input[3] > F_length*_F_LF.z/2) temp_input[3] = F_length*_F_LF.z/2;
        if(temp_input[3] < -F_length*_F_LF.z/2) temp_input[3] = -F_length*_F_LF.z/2;

//        double T_abs = sqrt(temp_input[2]*temp_input[2] + temp_input[3]*temp_input[3]);
//        if( T_abs > length*_F_LF.z){
//            temp_input[2] = temp_input[2]*length*_F_LF.z/T_abs;
//            temp_input[3] = temp_input[3]*length*_F_LF.z/T_abs;
//            cout<<"cutcut!!"<<endl;
//        }
    }


    // Torque filtering
    double alpha_roll = 1/(1 + 2*PI*dt*3.0);
    double alpha_pitch = 1/(1 + 2*PI*dt*4.0);
    InputTorque[0] = InputTorque[0]*alpha_roll + temp_input[0]*(1-alpha_roll); //RAR
    InputTorque[1] = InputTorque[1]*alpha_pitch + temp_input[1]*(1-alpha_pitch); //RAP
    InputTorque[2] = InputTorque[2]*alpha_roll + temp_input[2]*(1-alpha_roll); //LAR
    InputTorque[3] = InputTorque[3]*alpha_pitch + temp_input[3]*(1-alpha_pitch); //LAP

    // Jacobian Transform
    vec3 T_RA1_RA2, T_LA1_LA2;
    T_RA1_RA2 = AnkleT_to_MotorT_right(vec3(InputTorque[1], InputTorque[0],0),_RAP_ang_deg, _RAR_ang_deg, _RA1_ang_deg, _RA2_ang_deg);
    T_LA1_LA2 = AnkleT_to_MotorT_left(vec3(InputTorque[3], InputTorque[2],0),_LAP_ang_deg, _LAR_ang_deg, _LA1_ang_deg, _LA2_ang_deg);

    // Torque to duty
    double pwm_RA1, pwm_RA2, pwm_LA1, pwm_LA2;
//    pwm_RA1 = Torque_to_Duty(T_RA1_RA2[0]/75, 0, 1);
//    pwm_RA2 = Torque_to_Duty(T_RA1_RA2[1]/75, 0, 1);
//    pwm_LA1 = Torque_to_Duty(T_LA1_LA2[0]/75, 0, 1);
//    pwm_LA2 = Torque_to_Duty(T_LA1_LA2[1]/75, 0, 1);  //75 : gear ratio

    pwm_RA1 = Torque_to_Duty(T_RA1_RA2[0]/75, _RA1_vel_deg*90, 1);
    pwm_RA2 = Torque_to_Duty(T_RA1_RA2[1]/75, _RA2_vel_deg*90, 1);
    pwm_LA1 = Torque_to_Duty(T_LA1_LA2[0]/75, _LA1_vel_deg*90, 1);
    pwm_LA2 = Torque_to_Duty(T_LA1_LA2[1]/75, _LA2_vel_deg*90, 1);

    double pwm_lim = 300;
    if(pwm_RA1 > +pwm_lim) pwm_RA1 = pwm_lim;
    if(pwm_RA1 < -pwm_lim) pwm_RA1 = -pwm_lim;
    if(pwm_RA2 > +pwm_lim) pwm_RA2 = pwm_lim;
    if(pwm_RA2 < -pwm_lim) pwm_RA2 = -pwm_lim;

    if(pwm_LA1 > +pwm_lim) pwm_LA1 = pwm_lim;
    if(pwm_LA1 < -pwm_lim) pwm_LA1 = -pwm_lim;
    if(pwm_LA2 > +pwm_lim) pwm_LA2 = pwm_lim;
    if(pwm_LA2 < -pwm_lim) pwm_LA2 = -pwm_lim;

//    cout<<"pwmRA1: "<<pwm_RA1<<" pwmRA2: "<<pwm_RA2<<"  pwmLA1: "<<pwm_LA1<<" pwmLA2: "<<pwm_LA2<<endl;

    MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno, 3, pwm_RA1, 0, 0); //RAP
    MCJointPWMCommand2chHR(JOINT_INFO[RAR].canch, JOINT_INFO[RAR].bno, 3, pwm_RA2, 0, 0); //RAR

    MCJointPWMCommand2chHR(JOINT_INFO[LAP].canch, JOINT_INFO[LAP].bno, 3, pwm_LA1, 0, 0); //LAP
    MCJointPWMCommand2chHR(JOINT_INFO[LAR].canch, JOINT_INFO[LAR].bno, 3, pwm_LA2, 0, 0); //LAR

}

void HB_PreviewWalk::AnkleQuatCalculaltor(double _RAP_ang_del_deg, double _RAR_ang_del_deg, double _LAP_ang_del_deg, double _LAR_ang_del_deg){
    //// limit of angle
    double angle_lim_deg = 10.0;
    if(_RAP_ang_del_deg >= angle_lim_deg) _RAP_ang_del_deg = angle_lim_deg;
    if(_RAP_ang_del_deg <= -angle_lim_deg) _RAP_ang_del_deg = -angle_lim_deg;
    if(_RAR_ang_del_deg >= angle_lim_deg) _RAR_ang_del_deg = angle_lim_deg;
    if(_RAR_ang_del_deg <= -angle_lim_deg) _RAR_ang_del_deg = -angle_lim_deg;

    if(_LAP_ang_del_deg >= angle_lim_deg) _LAP_ang_del_deg = angle_lim_deg;
    if(_LAP_ang_del_deg <= -angle_lim_deg) _LAP_ang_del_deg = -angle_lim_deg;
    if(_LAR_ang_del_deg >= angle_lim_deg) _LAR_ang_del_deg = angle_lim_deg;
    if(_LAR_ang_del_deg <= -angle_lim_deg) _LAR_ang_del_deg = -angle_lim_deg;

    double alpha = 1/(1 + 2*PI*dt*5.0);
    RF_del_angle.y = RF_del_angle.y*alpha + _RAP_ang_del_deg*(1-alpha);
    RF_del_angle.x = RF_del_angle.x*alpha + _RAR_ang_del_deg*(1-alpha);

    LF_del_angle.y = LF_del_angle.y*alpha + _LAP_ang_del_deg*(1-alpha);
    LF_del_angle.x = LF_del_angle.x*alpha + _LAR_ang_del_deg*(1-alpha);

    //// Calc Ankle Quaternion from measured Ankle Angle
    RF_del_quat = quat(vec3(0,1,0),RF_del_angle.y*D2R)*quat(vec3(1,0,0),RF_del_angle.x*D2R);
    LF_del_quat = quat(vec3(0,1,0),LF_del_angle.y*D2R)*quat(vec3(1,0,0),LF_del_angle.x*D2R);
}

vec3 HB_PreviewWalk::AnkleT_to_MotorT_left(vec3 M_ref, double _AP_ang_deg, double AR_ang_deg, double _A1_ang_deg, double _A2_ang_deg){
    double J00, J01, J10, J11;
    GK.gazelle_jacobian_numeric_left(_AP_ang_deg, AR_ang_deg, _A1_ang_deg, _A2_ang_deg, J00, J01, J10, J11);
    mat3 Jacobian_trans = mat3(J00, J01, 0,
                               J10, J11, 0,
                                0, 0, 0);

    vec3 Torque_A1_A2 = Jacobian_trans*M_ref;

    return Torque_A1_A2;
}

vec3 HB_PreviewWalk::AnkleT_to_MotorT_right(vec3 M_ref, double _AP_ang_deg, double AR_ang_deg, double _A1_ang_deg, double _A2_ang_deg){
    double J00, J01, J10, J11;
    GK.gazelle_jacobian_numeric_right(_AP_ang_deg, AR_ang_deg, _A1_ang_deg, _A2_ang_deg, J00, J01, J10, J11);
    mat3 Jacobian_trans = mat3(J00, J01, 0,
                               J10, J11, 0,
                                0, 0, 0);

    vec3 Torque_A1_A2 = Jacobian_trans*M_ref;

    return Torque_A1_A2;
}

int HB_PreviewWalk::Torque_to_Duty(double _Tmotor, double _Vel_degsec, int _mode){
    //mode 0: complementary mode(not backdrivable)
    //mode 1: non-complementary mode(backdrivable)
    const double _KtCP = 150./0.468*2.;
    const double _KtNCP = 300./0.468*2.;

    int _PWM_T;
    if (_mode==0) _PWM_T = int(_Tmotor*_KtCP + CP_BEMF_comp(_Vel_degsec));
    else _PWM_T = int(_Tmotor*_KtNCP + NCP_BEMF_comp(_Vel_degsec));

    return _PWM_T;
}

//#define SIGN(x) (x>=0 ? 1:-1)
double HB_PreviewWalk::CP_BEMF_comp(double Vel_degsec)
{
    double t = fabs(Vel_degsec);
    int i;
    int curr = 0;

    const double sign = 1.;
    const int n = 19;
    const double c_table[19] = {0,  50,     100,    150,    200,    250,    300,    350,    400,    450,    500,    550,    600,    650,    700,    750,    800,    850,    900};
    const double t_table[19] = {0,  3350,   9600,   15100,  20200,  25400,  30500,  35700,  40800,  46000,  51200,  56500,  61800,  67000,  72100,  77600,  82800,  88000,  93100};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(Vel_degsec));
}

double HB_PreviewWalk::NCP_BEMF_comp(double Vel_degsec)
{
    double t = fabs(Vel_degsec);
    int i;
    int curr = 0;

    const double sign = 1.;
    const int n = 19;
    const double c_table[19] = {0,  45,     100,    150,    200,    250,    300,    350,    400,    450,    500,    550,    600,    650,    700,    750,    800,    850,    900};
    const double t_table[19] = {0,  3280,   21100,  32900,  41600,  48600,  54000,  58500,  63000,  66500,  68900,  71200,  73700,  76000,  78800,  81000,  83500,  86000,  89500};

    for(i=1; i<n; i++)
        if(t < t_table[i])
            break;
    if(t >= t_table[n-1])
        curr = (t-t_table[n-1])/(t_table[n-1]-t_table[n-2])*(c_table[n-1]-c_table[n-2]) + c_table[n-1];
    else
        curr = (t-t_table[i-1])/(t_table[i]-t_table[i-1])*(c_table[i]-c_table[i-1]) + c_table[i-1] ;

    return (sign*curr*SIGN(Vel_degsec));
}

void HB_PreviewWalk::AnkleTorqueController_pos(double _RAR_T_ref, double _RAP_T_ref, double _LAR_T_ref, double _LAP_T_ref,
                                               vec3 _F_RF, vec3 _F_LF, vec3 _M_RF, vec3 _M_LF){
    ////gains
    //for rigid foot
    double gain_roll_air = -1.5;//-1.0;
    double decrease_factor_roll = 5;//4;

    double gain_pitch_air = -1.2;//-1.5;
    double decrease_factor_pitch = 10;

    //for uneven terrain
    //gain_roll_min = -0.06;
    //gain_pitch_min = -0.08;
    //double alpha_ref = 1/(1 + 2*PI*dt*4.5);

    //// variable gains
    double gain_roll_max = -0.5;//-0.5;
    double gain_roll_min = -0.035;//-0.045;//-0.045;

    double gain_pitch_max = -0.85;//-0.5;//-0.5;
    double gain_pitch_min = -0.045;//-0.06;//-0.045;

    if(step_phase <= 1 || step_phase > N_step - 2)
    {
        gain_roll_max = -0.1;//-0.5;
        gain_roll_min = -0.01;
        gain_pitch_max = -0.1;//-0.5;
        gain_pitch_min = -0.01;

    }
    if(Standing_mode_flag == true)
    {
        gain_roll_max = -0.01;//-0.5;
        gain_roll_min = -0.015;
        gain_pitch_max = -0.01;//-0.5;
        gain_pitch_min = -0.03;
    }



    //for damping foot
//    double gain_roll_air = -1.0;
//    double gain_pitch_air = -1.3;
//    double decrease_factor = 3.0;

    double RF_variable_gain_roll, RF_variable_gain_pitch;
    double LF_variable_gain_roll, LF_variable_gain_pitch;

    double Contact_threshold = 50; //50N

    double RF_Arial_return_factor = 0.5;
    double LF_Arial_return_factor = 0.5;

    if(_F_RF.z < 10) _F_RF.z = 0;
    if(_F_LF.z < 10) _F_LF.z = 0;

    //// static variables for filtering
    static vec3 _F_RF_filtered = vec3();
    static vec3 _F_LF_filtered = vec3();
    static vec3 _M_RF_filtered = vec3();
    static vec3 _M_LF_filtered = vec3();
    static double RAR_T_ref_filtered = 0;
    static double RAP_T_ref_filtered = 0;
    static double LAR_T_ref_filtered = 0;
    static double LAP_T_ref_filtered = 0;



    //// filterring
    double alpha_F = 1/(1 + 2*PI*dt*30);
    _F_RF_filtered = alpha_F*_F_RF_filtered + (1.0 - alpha_F)*_F_RF;
    _F_LF_filtered = alpha_F*_F_LF_filtered + (1.0 - alpha_F)*_F_LF;

    vec3 _F_RF_filtered_new, _F_LF_filtered_new;
    _F_RF_filtered_new.z = _F_RF_filtered.z - 10;
    _F_LF_filtered_new.z = _F_LF_filtered.z - 10;

    if(_F_RF_filtered_new.z <= 0) _F_RF_filtered_new.z = 0;
    if(_F_LF_filtered_new.z <= 0) _F_LF_filtered_new.z = 0;

    double alpha_M = 1/(1 + 2*PI*dt*90);
    _M_RF_filtered = alpha_M*_M_RF_filtered + (1.0 - alpha_M)*_M_RF;
    _M_LF_filtered = alpha_M*_M_LF_filtered + (1.0 - alpha_M)*_M_LF;


    double alpha_ref = 1/(1 + 2*PI*dt*3.5);
    RAR_T_ref_filtered = alpha_ref*RAR_T_ref_filtered + (1.0 - alpha_ref)*_RAR_T_ref;
    RAP_T_ref_filtered = alpha_ref*RAP_T_ref_filtered + (1.0 - alpha_ref)*_RAP_T_ref;

    LAR_T_ref_filtered = alpha_ref*LAR_T_ref_filtered + (1.0 - alpha_ref)*_LAR_T_ref;
    LAP_T_ref_filtered = alpha_ref*LAP_T_ref_filtered + (1.0 - alpha_ref)*_LAP_T_ref;

    //// Angle Control according to contact state

    //----------------------------RF control --------------------------------------------------------------
//    RF_varible_gain_roll = gain_roll_air*1/(1 + _F_RF_filtered_new.z/decrease_factor_roll);
//    RF_variable_gain_pitch = gain_pitch_air*1/(1 + _F_RF_filtered_new.z/decrease_factor_pitch);

    RF_variable_gain_roll = (gain_roll_min - gain_roll_max)/350*_F_RF_filtered_new.z + gain_roll_max;
    RF_variable_gain_pitch = (gain_pitch_min - gain_pitch_max)/350*_F_RF_filtered_new.z + gain_pitch_max;

    if(RF_variable_gain_roll < gain_roll_max) RF_variable_gain_roll = gain_roll_max;
    if(RF_variable_gain_pitch < gain_pitch_max) RF_variable_gain_pitch = gain_pitch_max;

    if(RF_variable_gain_roll > gain_roll_min) RF_variable_gain_roll = gain_roll_min;
    if(RF_variable_gain_pitch > gain_pitch_min) RF_variable_gain_pitch = gain_pitch_min;

    RF_Arial_return_factor = 0.2;//0.1*(10 + _F_RF_filtered.z)/10;

    if(_F_RF_filtered.z > Contact_threshold)
    {
        dRF_angle_ctrl.x = RF_variable_gain_roll*(RAR_T_ref_filtered - _M_RF_filtered.x);
        dRF_angle_ctrl.y = RF_variable_gain_pitch*(RAP_T_ref_filtered - _M_RF_filtered.y);
    }
    else
    {
        dRF_angle_ctrl.x = RF_variable_gain_roll*(RAR_T_ref_filtered - _M_RF_filtered.x)-  RF_angle_ctrl.x/RF_Arial_return_factor;
        dRF_angle_ctrl.y = RF_variable_gain_pitch*(RAP_T_ref_filtered - _M_RF_filtered.y) - RF_angle_ctrl.y/RF_Arial_return_factor;
    }


    //----------------------------LF control --------------------------------------------------------------
//    LF_varible_gain_roll = gain_roll_air*1/(1 + _F_LF_filtered_new.z/decrease_factor_roll);
//    LF_variable_gain_pitch = gain_pitch_air*1/(1 + _F_LF_filtered_new.z/decrease_factor_pitch);
//    //cout<<"roll gain : "<<LF_varible_gain_roll<<"  pitch gain : "<<LF_variable_gain_pitch<<endl;

    LF_variable_gain_roll = (gain_roll_min - gain_roll_max)/350*_F_LF_filtered_new.z + gain_roll_max;
    LF_variable_gain_pitch = (gain_pitch_min - gain_pitch_max)/350*_F_LF_filtered_new.z + gain_pitch_max;

    if(LF_variable_gain_roll < gain_roll_max) LF_variable_gain_roll = gain_roll_max;
    if(LF_variable_gain_pitch < gain_pitch_max) LF_variable_gain_pitch = gain_pitch_max;

    if(LF_variable_gain_roll > gain_roll_min) LF_variable_gain_roll = gain_roll_min;
    if(LF_variable_gain_pitch > gain_pitch_min) LF_variable_gain_pitch = gain_pitch_min;


    LF_Arial_return_factor = 0.2;//0.1*(10 + _F_LF_filtered.z)/10;

    if(_F_LF_filtered.z > Contact_threshold)
    {
        dLF_angle_ctrl.x = LF_variable_gain_roll*(LAR_T_ref_filtered - _M_LF_filtered.x);
        dLF_angle_ctrl.y = LF_variable_gain_pitch*(LAP_T_ref_filtered - _M_LF_filtered.y);
    }
    else
    {
        dLF_angle_ctrl.x = LF_variable_gain_roll*(LAR_T_ref_filtered - _M_LF_filtered.x) -  LF_angle_ctrl.x/LF_Arial_return_factor;
        dLF_angle_ctrl.y = LF_variable_gain_pitch*(LAP_T_ref_filtered - _M_LF_filtered.y) -  LF_angle_ctrl.y/LF_Arial_return_factor;
    }

    double Max_ankle_vel = 0.9;

    if(dRF_angle_ctrl.x > Max_ankle_vel) dRF_angle_ctrl.x = Max_ankle_vel;
    if(dRF_angle_ctrl.x < -Max_ankle_vel) dRF_angle_ctrl.x = -Max_ankle_vel;

    if(dRF_angle_ctrl.y > Max_ankle_vel) dRF_angle_ctrl.y = Max_ankle_vel;
    if(dRF_angle_ctrl.y < -Max_ankle_vel) dRF_angle_ctrl.y = -Max_ankle_vel;

    if(dLF_angle_ctrl.x > Max_ankle_vel) dLF_angle_ctrl.x = Max_ankle_vel;
    if(dLF_angle_ctrl.x < -Max_ankle_vel) dLF_angle_ctrl.x = -Max_ankle_vel;

    if(dLF_angle_ctrl.y > Max_ankle_vel) dLF_angle_ctrl.y = Max_ankle_vel;
    if(dLF_angle_ctrl.y < -Max_ankle_vel) dLF_angle_ctrl.y = -Max_ankle_vel;



    if(Pos_Ankle_torque_control_flag == true){
        RF_angle_ctrl.x += dRF_angle_ctrl.x*dt;
        RF_angle_ctrl.y += dRF_angle_ctrl.y*dt;

        LF_angle_ctrl.x += dLF_angle_ctrl.x*dt;
        LF_angle_ctrl.y += dLF_angle_ctrl.y*dt;

        double angle_limit = 20; //deg
        double angle_limit_rol = 25;
        if(RF_angle_ctrl.x > angle_limit_rol*D2R) RF_angle_ctrl.x = angle_limit_rol*D2R;
        if(RF_angle_ctrl.x < -angle_limit_rol*D2R) RF_angle_ctrl.x = -angle_limit_rol*D2R;
        if(RF_angle_ctrl.y > angle_limit*D2R) RF_angle_ctrl.y = angle_limit*D2R;
        if(RF_angle_ctrl.y < -angle_limit*D2R) RF_angle_ctrl.y = -angle_limit*D2R;

        if(LF_angle_ctrl.x > angle_limit_rol*D2R) LF_angle_ctrl.x = angle_limit_rol*D2R;
        if(LF_angle_ctrl.x < -angle_limit_rol*D2R) LF_angle_ctrl.x = -angle_limit_rol*D2R;
        if(LF_angle_ctrl.y > angle_limit*D2R) LF_angle_ctrl.y = angle_limit*D2R;
        if(LF_angle_ctrl.y < -angle_limit*D2R) LF_angle_ctrl.y = -angle_limit*D2R;
    }
    else{
        RF_angle_ctrl.x =0;
        RF_angle_ctrl.y =0;

        LF_angle_ctrl.x =0;
        LF_angle_ctrl.y =0;
    }

//    RF_quat_ctrl = quat(vec3(0,1,0),RF_angle_ctrl.y)*quat(vec3(1,0,0),RF_angle_ctrl.x);
//    LF_quat_ctrl = quat(vec3(0,1,0),LF_angle_ctrl.y)*quat(vec3(1,0,0),LF_angle_ctrl.x);
    RF_quat_ctrl = quat();//quat(vec3(0,1,0),RF_angle_ctrl.y)*quat(vec3(1,0,0),RF_angle_ctrl.x);
    LF_quat_ctrl = quat();//quat(vec3(0,1,0),LF_angle_ctrl.y)*quat(vec3(1,0,0),LF_angle_ctrl.x);


}

void HB_PreviewWalk::DSP_Fz_controller2(char _swingFoot){
    //// get alpha
    vec3 local_pRF = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pRF_ref);
    vec3 local_pLF = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pLF_ref);

    quat local_qRF = global2local_quat(qRF_ref, qLF_ref, qRF_ref);
    quat local_qLF = global2local_quat(qRF_ref, qLF_ref, qLF_ref);

    vec3 LF_lt1 = vec3(FootLength/2, FootWidth/2, 0);
    vec3 LF_rt2 = vec3(FootLength/2, -FootWidth/2, 0);
    vec3 LF_rb3 = vec3(-FootLength/2, -FootWidth/2, 0);
    vec3 LF_lb4 = vec3(-FootLength/2, FootWidth/2, 0);

    vec3 RF_lt1 = vec3(FootLength/2, FootWidth/2, 0);
    vec3 RF_rt2 = vec3(FootLength/2, -FootWidth/2, 0);
    vec3 RF_rb3 = vec3(-FootLength/2, -FootWidth/2, 0);
    vec3 RF_lb4 = vec3(-FootLength/2, FootWidth/2, 0);

    LF_lt1 = mat3(local_qLF)*LF_lt1;
    LF_rt2 = mat3(local_qLF)*LF_rt2;
    LF_rb3 = mat3(local_qLF)*LF_rb3;
    LF_lb4 = mat3(local_qLF)*LF_lb4;

    RF_lt1 = mat3(local_qRF)*RF_lt1;
    RF_rt2 = mat3(local_qRF)*RF_rt2;
    RF_rb3 = mat3(local_qRF)*RF_rb3;
    RF_lb4 = mat3(local_qRF)*RF_lb4;

    LF_lt1 = LF_lt1 + local_pLF;
    LF_rt2 = LF_rt2 + local_pLF;
    LF_rb3 = LF_rb3 + local_pLF;
    LF_lb4 = LF_lb4 + local_pLF;

    RF_lt1 = RF_lt1 + local_pRF;
    RF_rt2 = RF_rt2 + local_pRF;
    RF_rb3 = RF_rb3 + local_pRF;
    RF_lb4 = RF_lb4 + local_pRF;

    stack<Point> SP_LF;
    SP_LF.push({LF_lt1.x, LF_lt1.y});
    SP_LF.push({LF_rt2.x, LF_rt2.y});
    SP_LF.push({LF_rb3.x, LF_rb3.y});
    SP_LF.push({LF_lb4.x, LF_lb4.y});

    stack<Point> SP_RF;
    SP_RF.push({RF_lt1.x, RF_lt1.y});
    SP_RF.push({RF_rt2.x, RF_rt2.y});
    SP_RF.push({RF_rb3.x, RF_rb3.y});
    SP_RF.push({RF_lb4.x, RF_lb4.y});

    Point P_local_cZMP = {cZMP_dsp_local_proj.x, cZMP_dsp_local_proj.y};

    if(zmpInOutCheck(P_local_cZMP, SP_LF) == true) Alpha_dsp = 0;        // cZMP is in LF
    else if(zmpInOutCheck(P_local_cZMP, SP_RF) == true) Alpha_dsp = 1;   // cZMP is in RF
    else{                                                           // cZMP is between LF & RF
        // get Alpha
        vec3 n = vec3(local_pLF.x - local_pRF.x, (local_pLF.y - FootWidth/2) - (local_pRF.y + FootWidth/2), 0);
        vec3 p = vec3(cZMP_dsp_local_proj.x - local_pRF.x, cZMP_dsp_local_proj.y - (local_pRF.y + FootWidth/2), 0);

        vec3 p2n_proj = n*(dot(p,n)/dot(n));

        vec3 P_alpha = vec3(local_pRF.x + p2n_proj.x, (local_pRF.y + FootWidth/2) + p2n_proj.y,0);

        Alpha_dsp = sqrt(distSq({local_pLF.x, (local_pLF.y - FootWidth/2)},{P_alpha.x, P_alpha.y}))/sqrt(distSq({local_pLF.x, (local_pLF.y - FootWidth/2)},{local_pRF.x, (local_pRF.y + FootWidth/2)}));
    }
    if(Alpha_dsp > 1.0) Alpha_dsp = 1.0;
    if(Alpha_dsp < 0) Alpha_dsp = 0;


    double t_half_dsp = t_step*dsp_ratio/2;


    /// check control flag
    // check dsp by time
    if(t_now < t_half_dsp - 3*dt){
        DSP_time_flag = true;
//        DSP_time_flag = false;
    }
    else if(t_now < t_step - t_half_dsp - 1*dt){
        DSP_time_flag = false;
    }
    else{
        DSP_time_flag = true;
    }

    // check dsp by force
    if((RF_landing_flag == true && LF_landing_flag == true)){
       DSP_force_flag = true;
    }
    else{
        DSP_force_flag = false;
    }

    //// gain scheduling

    if(step_phase <= 1 || step_phase > N_step - 2){
        dsp_ctrl_gain = 0.00005;
    }
    else{
//        if(DSP_time_flag == true && DSP_force_flag == false){  //Late landing
            // In late landing situation, reference force of landing foot should be total FZ
            //--> if swingfoot is RF, Alpha = 1;
            //--> if swingfoot is LF, Alpha = 0;

            // torso tilt should not be considered.

//            if((_swingFoot == RFoot && t_now >= t_step - t_half_dsp) || (_swingFoot == LFoot && t_now <= t_half_dsp)){ // RFswing
//                Alpha_dsp = 1;
//                if(COM_e_imu_local.y > 0.02){   //just late landing
//                    dsp_ctrl_gain = 0.0;
//                    dsp_tilt_gain = 0;
//                }
//                else{                       // obstacle late landing
//                    dsp_ctrl_gain = 0.00022;
//                    dsp_tilt_gain = 0;
//                }

//                // if foot is on the ground, change to dsp landing con..
////                if(_F_RF_z > 15){
////                    //RF_landing_flag = true;
////                    dsp_tilt_gain = 2;
////                }
//            }
//            else if((_swingFoot == LFoot && t_now >= t_step - t_half_dsp) || (_swingFoot == RFoot && t_now <= t_half_dsp)){ // LFswing
//                Alpha_dsp = 0;
//                if(COM_e_imu_local.y < -0.02){   //just late landing
//                    dsp_ctrl_gain = 0.0;
//                    dsp_tilt_gain = 0;
//                }
//                else{                       // obstacle late landing
//                    dsp_ctrl_gain = 0.00022;
//                    dsp_tilt_gain = 0;
//                }

//                // if foot is on the ground, change to dsp landing con..
////                if(_F_LF_z > 15){
////                    //LF_landing_flag = true;
////                    dsp_tilt_gain = 2;
////                }
//            }

//            dsp_tilt_gain = 0;
////            dsp_ctrl_gain = 0.0001;

//        }
        if(DSP_force_flag == true || DSP_time_flag == true){  // Ealy landing

            if((_swingFoot == RFoot && t_now >= t_step*0.7) || (_swingFoot == LFoot && t_now <= t_step*0.3)){  // RFswing
                if(COM_e_imu_local.y <= -0.03){ // tilt early landing
                    dsp_ctrl_gain = 0.0004;
                    dsp_tilt_gain = 1;
                }
                else if(COM_e_imu_local.y > 0.03){  //obstacle early landing
                    dsp_ctrl_gain = 0.0005;
                    dsp_tilt_gain = 1;
                }
                else{                                       // just dsp
                    dsp_ctrl_gain = 0.0005;
                    dsp_tilt_gain = 1;
                }
            }
            else if((_swingFoot == LFoot && t_now >= t_step*0.7) || (_swingFoot == RFoot && t_now <= t_step*0.3)){ // LFswing
                if(COM_e_imu_local.y >= 0.03){ // tilt early landing
                    dsp_ctrl_gain = 0.0004;
                    dsp_tilt_gain = 1;
                }
                else if(COM_e_imu_local.y < -0.03){ //obstacle early landing
                    dsp_ctrl_gain = 0.0005;
                    dsp_tilt_gain = 1;
                }
                else{                               // just dsp
                    dsp_ctrl_gain = 0.0005;
                    dsp_tilt_gain = 1;
                }
            }
            else{
                dsp_ctrl_gain = 0.0002;
                dsp_tilt_gain = 0;
            }

        }
        else{ //pure ssp phase
            dsp_ctrl_gain = 0.0005;//0.00005;
            dsp_tilt_gain = 0;//4;
        }

    }

//    if(DSP_time_flag == true || DSP_force_flag == true){
//        dsp_ctrl_gain = 0.0001;//0.00005;
//        dsp_tilt_gain = 0.0;//4;
//    }
//    else{
//        dsp_ctrl_gain = 0.000015;//0.00005;
//        dsp_tilt_gain = 0;//4;
//    }


    dsp_ctrl_gain = 0.00015;//0.0002;//0.00005;
    dsp_tilt_gain = 0;//4;


    //// control
    double Fz_total = RS.F_RF.z + RS.F_LF.z;// 360; //32kg
    RF_Fz_ref = Alpha_dsp*Fz_total;
    LF_Fz_ref = (1.0 - Alpha_dsp)*Fz_total;

    Fz_diff_ref = LF_Fz_ref - RF_Fz_ref;

    //// get Fz difference Error
    double alpha_dFz = 1/(1 + 2*PI*dt*2);

    Fz_diff = (RS.F_LF.z - RS.F_RF.z);

    Fz_diff_error = Fz_diff_ref - Fz_diff;


//    if(DSP_time_flag == true || DSP_force_flag == true){
//        dz_ctrl = dsp_ctrl_gain*Fz_diff_error + dsp_tilt_gain*COM_e_imu_local.y;// - z_ctrl/1;
//    }
//    else{
//        dz_ctrl = -z_ctrl/0.1;
//    }

    dz_ctrl = dsp_ctrl_gain*Fz_diff_error + dsp_tilt_gain*COM_e_imu_local.y- z_ctrl/0.3;


//    double alpha_dz = 1/(1 + 2*PI*dt*10);
//    dz_ctrl_filtered = alpha_dz*dz_ctrl_filtered + (1.0 - alpha_dz)*dz_ctrl;

    z_ctrl += dz_ctrl*dt;

    if(z_ctrl > 0.05) z_ctrl = 0.05;
    if(z_ctrl < -0.05) z_ctrl = -0.05;

}

void HB_PreviewWalk::DSP_Fz_controller(char _swingFoot, double _F_RF_z, double _F_LF_z){
    double t_half_dsp = T_nom*dsp_ratio/2;


    /// check control flag
    // check dsp by time
    if(t_now < t_half_dsp - 3*dt){
        DSP_time_flag = true;
//        DSP_time_flag = false;
    }
    else if(t_now < t_step - t_half_dsp - 1*dt){
        DSP_time_flag = false;
    }
    else{
        DSP_time_flag = true;
    }

    // check dsp by force
    if((RF_landing_flag == true && LF_landing_flag == true)){
       DSP_force_flag = true;
    }
    else{
        DSP_force_flag = false;
    }


    //// get Alpha dsp
    //// get alpha
    vec3 local_pRF = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pRF_ref);
    vec3 local_pLF = global2local_point(qRF_ref, qLF_ref, pRF_ref, pLF_ref, pLF_ref);

    quat local_qRF = global2local_quat(qRF_ref, qLF_ref, qRF_ref);
    quat local_qLF = global2local_quat(qRF_ref, qLF_ref, qLF_ref);

    vec3 LF_lt1 = vec3(FootLength/2, FootWidth/2, 0);
    vec3 LF_rt2 = vec3(FootLength/2, -FootWidth/2, 0);
    vec3 LF_rb3 = vec3(-FootLength/2, -FootWidth/2, 0);
    vec3 LF_lb4 = vec3(-FootLength/2, FootWidth/2, 0);

    vec3 RF_lt1 = vec3(FootLength/2, FootWidth/2, 0);
    vec3 RF_rt2 = vec3(FootLength/2, -FootWidth/2, 0);
    vec3 RF_rb3 = vec3(-FootLength/2, -FootWidth/2, 0);
    vec3 RF_lb4 = vec3(-FootLength/2, FootWidth/2, 0);

    LF_lt1 = mat3(local_qLF)*LF_lt1;
    LF_rt2 = mat3(local_qLF)*LF_rt2;
    LF_rb3 = mat3(local_qLF)*LF_rb3;
    LF_lb4 = mat3(local_qLF)*LF_lb4;

    RF_lt1 = mat3(local_qRF)*RF_lt1;
    RF_rt2 = mat3(local_qRF)*RF_rt2;
    RF_rb3 = mat3(local_qRF)*RF_rb3;
    RF_lb4 = mat3(local_qRF)*RF_lb4;

    LF_lt1 = LF_lt1 + local_pLF;
    LF_rt2 = LF_rt2 + local_pLF;
    LF_rb3 = LF_rb3 + local_pLF;
    LF_lb4 = LF_lb4 + local_pLF;

    RF_lt1 = RF_lt1 + local_pRF;
    RF_rt2 = RF_rt2 + local_pRF;
    RF_rb3 = RF_rb3 + local_pRF;
    RF_lb4 = RF_lb4 + local_pRF;

    stack<Point> SP_LF;
    SP_LF.push({LF_lt1.x, LF_lt1.y});
    SP_LF.push({LF_rt2.x, LF_rt2.y});
    SP_LF.push({LF_rb3.x, LF_rb3.y});
    SP_LF.push({LF_lb4.x, LF_lb4.y});

    stack<Point> SP_RF;
    SP_RF.push({RF_lt1.x, RF_lt1.y});
    SP_RF.push({RF_rt2.x, RF_rt2.y});
    SP_RF.push({RF_rb3.x, RF_rb3.y});
    SP_RF.push({RF_lb4.x, RF_lb4.y});

    Point P_local_cZMP = {cZMP_dsp_local_proj.x, cZMP_dsp_local_proj.y};

    if(zmpInOutCheck(P_local_cZMP, SP_LF) == true) Alpha_dsp = 0;        // cZMP is in LF
    else if(zmpInOutCheck(P_local_cZMP, SP_RF) == true) Alpha_dsp = 1;   // cZMP is in RF
    else{                                                           // cZMP is between LF & RF
        // get Alpha
        vec3 n = vec3(local_pLF.x - local_pRF.x, (local_pLF.y - FootWidth/2) - (local_pRF.y + FootWidth/2), 0);
        vec3 p = vec3(cZMP_dsp_local_proj.x - local_pRF.x, cZMP_dsp_local_proj.y - (local_pRF.y + FootWidth/2), 0);

        vec3 p2n_proj = n*(dot(p,n)/dot(n));

        vec3 P_alpha = vec3(local_pRF.x + p2n_proj.x, (local_pRF.y + FootWidth/2) + p2n_proj.y,0);

        Alpha_dsp = sqrt(distSq({local_pLF.x, (local_pLF.y - FootWidth/2)},{P_alpha.x, P_alpha.y}))/sqrt(distSq({local_pLF.x, (local_pLF.y - FootWidth/2)},{local_pRF.x, (local_pRF.y + FootWidth/2)}));
    }
    if(Alpha_dsp > 1.0) Alpha_dsp = 1.0;
    if(Alpha_dsp < 0) Alpha_dsp = 0;


    /// get alpha dsp 2
//    Alpha_dsp = 0.5 - COM_e_imu_local.y*20 - dCOM_e_imu_local.y*15;
//    //Alpha_dsp = 0.5 - (COM_m_filtered.y - COM_ref.y)*70 - (dCOM_m_filtered.y - dCOM_ref.y)*5;

//    if(Alpha_dsp >= 1) Alpha_dsp = 1;
//    if(Alpha_dsp <= 0) Alpha_dsp = 0;

//    //cout<<"alpha dsp : "<<Alpha_dsp<<endl;


    /// get alpha 3
//    double A = -cZMP_local_scaled.y/0.03;
//    if(A >= 1.0) A = 1.0;
//    if(A <= -1.0) A = -1.0;

//    Alpha_dsp = (A + 1.0)/2.0;


    //// Control

    if(step_phase <= 1 || step_phase > N_step - 2)
    {
        dsp_ctrl_gain = 0.0001; //0.0008
        dsp_tilt_gain = 1.5;//0.9
    }
    else
    {
        dsp_ctrl_gain = 0.001; //0.0012
        dsp_tilt_gain = 12.0; //18
    }
//    else{
//        if(DSP_time_flag == true && DSP_force_flag == false){  //Late landing
//            // In late landing situation, reference force of landing foot should be total FZ
//            //--> if swingfoot is RF, Alpha = 1;
//            //--> if swingfoot is LF, Alpha = 0;

//            // torso tilt should not be considered.

//            if((_swingFoot == RFoot && t_now >= t_step - t_half_dsp) || (_swingFoot == LFoot && t_now <= t_half_dsp)){ // RFswing
//                //Alpha_dsp = 1;
//                if(COM_e_imu_local.y > 0.01){   //just late landing
//                    dsp_ctrl_gain = 0.0;
//                    dsp_tilt_gain = 0;
//                }
//                else{                       // obstacle late landing
//                    dsp_ctrl_gain = 0.0003;
//                    dsp_tilt_gain = 0;
//                    cout<<"R swing obstacle late landing"<<endl;
//                }

//            }
//            else if((_swingFoot == LFoot && t_now >= t_step - t_half_dsp) || (_swingFoot == RFoot && t_now <= t_half_dsp)){ // LFswing
//                //Alpha_dsp = 0;
//                if(COM_e_imu_local.y < -0.01){   //just late landing
//                    dsp_ctrl_gain = 0.0;
//                    dsp_tilt_gain = 0;
//                }
//                else{                       // obstacle late landing
//                    dsp_ctrl_gain = 0.0003;
//                    dsp_tilt_gain = 0;
//                    cout<<"L swing obstacle late landing"<<endl;
//                }

//                // if foot is on the ground, change to dsp landing con..
////                if(_F_LF_z > 15){
////                    //LF_landing_flag = true;
////                    dsp_tilt_gain = 2;
////                }
//            }

//            dsp_tilt_gain = 0;
////            dsp_ctrl_gain = 0.0001;

//        }
//        else if(DSP_force_flag == true){  // Ealy landing

//            if((_swingFoot == RFoot && t_now >= t_step*0.7) || (_swingFoot == LFoot && t_now <= t_step*0.3)){  // RFswing
//                if(COM_e_imu_local.y <= -0.02){ // tilt early landing
//                    dsp_ctrl_gain = 0.0001;
//                    dsp_tilt_gain = 3;
//                    cout<<"R swing tilt ealry landing"<<endl;

//                }
//                else if(COM_e_imu_local.y > 0.02){  //obstacle early landing
//                    dsp_ctrl_gain = 0.0002;
//                    dsp_tilt_gain = 0;
//                    cout<<"R swing obstacle ealry landing"<<endl;
//                }
//                else{                                       // just dsp
//                    dsp_ctrl_gain = 0.0002;
//                    dsp_tilt_gain = 0;
//                    cout<<"R swing just DSP"<<endl;
//                }
//            }
//            else if((_swingFoot == LFoot && t_now >= t_step*0.7) || (_swingFoot == RFoot && t_now <= t_step*0.3)){ // LFswing
//                if(COM_e_imu_local.y >= 0.02){ // tilt early landing
//                    dsp_ctrl_gain = 0.0001;
//                    dsp_tilt_gain = 3;

//                }
//                else if(COM_e_imu_local.y < -0.02){ //obstacle early landing
//                    dsp_ctrl_gain = 0.0002;
//                    dsp_tilt_gain = 0;
//                }
//                else{                               // just dsp
//                    dsp_ctrl_gain = 0.0002;
//                    dsp_tilt_gain = 0;
//                }
//            }
//        }
//        else{ //pure ssp phase
//            dsp_ctrl_gain = 0;//0.00005;
//            dsp_tilt_gain = 0;//4;
//        }

//    }

    //// Calc ref Fz difference
    //robot mass
    double Fz_total = _F_RF_z + _F_LF_z;// 360; //32kg

    // minimum Contact Force
    if(Standing_mode_flag == true){

    }
    else if(DSP_force_flag == true || DSP_time_flag == true){
        if(Alpha_dsp < 0.3)Alpha_dsp = 0.3;
        if(Alpha_dsp > 0.7)Alpha_dsp = 0.7;
    }


    RF_Fz_ref = Alpha_dsp*Fz_total;
    LF_Fz_ref = (1.0 - Alpha_dsp)*Fz_total;

    Fz_diff_ref = LF_Fz_ref - RF_Fz_ref;

    //// get Fz difference Error
    Fz_diff = _F_LF_z - _F_RF_z;



//    dsp_ctrl_gain = 0.0003;
//    dsp_tilt_gain = 0;


    double alpha = 1/(1 + 2*PI*dt*8);
    Fz_diff_error = Fz_diff_ref - Fz_diff;//alpha*Fz_diff_error + (1 - alpha)*(Fz_diff_ref - Fz_diff);

    double L;
    if(_swingFoot == LFoot){
        L = des_pLF_local.x - local_pRF.x;
    }
    else if(_swingFoot == RFoot){
        L = local_pLF.x - des_pRF_local.x;
    }
    else{
        L = 0;
    }


    if(DSP_time_flag == true || DSP_force_flag == true)
    {
        //dz_ctrl = dsp_ctrl_gain*Fz_diff_error + dsp_tilt_gain*COM_e_imu_local.y; // - z_ctrl/0.1;
//        dz_ctrl = dsp_ctrl_gain*Fz_diff_error + dsp_tilt_gain/3*COM_e_imu_local.y
//                    + L*dsp_tilt_gain*COM_e_imu_local.x;

        if((COM_e_imu_local.y > 0.02 && RS.F_RF.z < 30) || (COM_e_imu_local.y < -0.02 && RS.F_LF.z < 30))
        {
            dz_ctrl = +dsp_ctrl_gain/10*Fz_diff_error - dsp_tilt_gain/10*COM_e_imu_local.y
                        + L*dsp_tilt_gain/5*COM_e_imu_local.x;
//            cout<<"DSP tilting out!"<<endl;
        }
        else
        {
            dz_ctrl = dsp_ctrl_gain*Fz_diff_error + dsp_tilt_gain/2*COM_e_imu_local.y
                        + L*dsp_tilt_gain*COM_e_imu_local.x;
        }

        //Pelv_roll_vel_ref = 8*COM_e_imu_local.y - Pelv_roll_ref/0.5;
//        Pelv_roll_vel_ref = 0*COM_e_imu_local.y - Pelv_roll_ref/0.5;

        // torso control

        if(dz_ctrl > 0.35) dz_ctrl = 0.35;
        if(dz_ctrl < -0.35) dz_ctrl = -0.35;

    }
    else
    {
        //dz_ctrl = dsp_ctrl_gain/1.5*Fz_diff_error + dsp_tilt_gain/2*COM_e_imu_local.y  - z_ctrl/0.3;// +dsp_tilt_gain/50*dCOM_e_imu_local.y;
        dz_ctrl = dsp_ctrl_gain/2.0*Fz_diff_error + dsp_tilt_gain/3.0*COM_e_imu_local.y
                    +0*L*dsp_tilt_gain*COM_e_imu_local.x - z_ctrl/0.3;

        dz_ctrl = dsp_tilt_gain/3.0*COM_e_imu_local.y
                    - z_ctrl/0.3;




        if((COM_e_imu_local.y > 0.02 && RS.F_RF.z < 30) || (COM_e_imu_local.y < -0.02 && RS.F_LF.z < 30))
        {
            dz_ctrl = dsp_ctrl_gain/3.0*Fz_diff_error - dsp_tilt_gain/6*COM_e_imu_local.y
                    +0*L*dsp_tilt_gain*COM_e_imu_local.x - z_ctrl/0.3;
//            cout<<"SSP tilting out!"<<endl;

        }

        if(dz_ctrl > 0.25) dz_ctrl = 0.25;
        if(dz_ctrl < -0.25) dz_ctrl = -0.25;




        //cout<<"dz_ctrl : "<<dz_ctrl<<endl;
//        Pelv_roll_vel_ref = - Pelv_roll_ref/0.5;
//        if(t_now < t_step*(1 - dsp_ratio/2) - 0.1){
//            dz_ctrl = - z_ctrl/0.03;
//        }
//        else{
//            dz_ctrl = dsp_tilt_gain*COM_e_imu_local.y - z_ctrl/0.03;
//        }

    }

    if(Standing_mode_flag == true)
    {
        dsp_ctrl_gain = 0.0001;
        dsp_tilt_gain = 0.6;
        dz_ctrl = dsp_ctrl_gain*Fz_diff_error + dsp_tilt_gain/3*COM_e_imu_local.y;
    }



    double alpha_dz = 1/(1 + 2*PI*dt*100);
    dz_ctrl_filtered = dz_ctrl;// alpha_dz*dz_ctrl_filtered + (1.0 - alpha_dz)*dz_ctrl;


//    if(k%100 == 0)
//        cout<<dz_ctrl_filtered<<endl;

    z_ctrl += dz_ctrl_filtered*dt;

//    Pelv_roll_ref += Pelv_roll_vel_ref*dt; // in rad

    //cout<<"pelv roll ref deg"<<Pelv_roll_ref*R2D<<endl;

    if(z_ctrl > 0.09) z_ctrl = 0.09;
    if(z_ctrl < -0.09) z_ctrl = -0.09;




//    // torso control

//    if(Pelv_roll_ref >= 5*D2R)  Pelv_roll_ref = 5*D2R;
//    if(Pelv_roll_ref <= -5*D2R) Pelv_roll_ref = -5*D2R;

//    if(Pelv_pitch_ref >= 5*D2R)  Pelv_pitch_ref = 5*D2R;
//    if(Pelv_pitch_ref <= -5*D2R) Pelv_pitch_ref = -5*D2R;

//    rpy Pelv_rpy = rpy(Pelv_roll_ref, Pelv_pitch_ref, get_Pel_yaw_from_qRF_qLF(qRF_ref, qLF_ref));
//    qPel_ref = quat(Pelv_rpy);

}

vec3 HB_PreviewWalk::del_u_modification_by_foot_size(vec3 _del_u_f){
    double Foot_length = 0.18;
    double Foot_width = 0.15;

    // X- direction
    if(fabs(del_u_f_modi.x - _del_u_f.x) <= Foot_length/2.0){
        // do nothing
    }
    else if(_del_u_f.x - del_u_f_modi.x > Foot_length/2.0){
        del_u_f_modi.x = _del_u_f.x - Foot_length/2.0;
    }
    else if(_del_u_f.x - del_u_f_modi.x < -Foot_length/2.0){
        del_u_f_modi.x = _del_u_f.x + Foot_length/2.0;
    }

    // Y- direction
    if(fabs(del_u_f_modi.y - _del_u_f.y) <= Foot_width/2.0){
        // do nothing
    }
    else if(_del_u_f.y - del_u_f_modi.y > Foot_width/2.0){
        del_u_f_modi.y = _del_u_f.y - Foot_width/2.0;
    }
    else if(_del_u_f.y - del_u_f_modi.y < -Foot_width/2.0){
        del_u_f_modi.y = _del_u_f.y + Foot_width/2.0;
    }

    return del_u_f_modi;


//    vec3 del_u_f_modi_ = vec3();

//    if(fabs(_del_u_f.x) <= Foot_length/2){
//        del_u_f_modi_.x = 0;
//    }
//    else{

//        //del_u_f_modi_.x = HB_sign(_del_u_f.x)*(fabs(_del_u_f.x) - Foot_length/2);

//        if(_del_u_f.x > 0) del_u_f_modi_.x = _del_u_f.x - Foot_length/2;
//        if(_del_u_f.x < 0) del_u_f_modi_.x = _del_u_f.x + Foot_length/2;

//    }

//    if(fabs(_del_u_f.y) <= Foot_width/2){
//        del_u_f_modi_.y = 0;
//    }
//    else{

//        //del_u_f_modi_.y = HB_sign(_del_u_f.y)*(fabs(_del_u_f.y) - Foot_width/2);

//        if(_del_u_f.y > 0) del_u_f_modi_.y = _del_u_f.y - Foot_width/2;
//        if(_del_u_f.y < 0) del_u_f_modi_.y = _del_u_f.y + Foot_width/2;
//    }

//    return del_u_f_modi_;

}


double HB_PreviewWalk::Calc_LandingHeight(char _SwingFoot, vec3 _pLF_des, vec3 _pRF_des, quat _qLF_ref, quat _qRF_ref, mat3 _G_R_g_pitroll)
{
//    if(_SwingFoot == -1){ // RightFoot Swing
//        vec3 L = (_pRF_des - _pLF_des)/1.0;

//        vec3 L_local = global2local_vec(_qRF_ref, _qLF_ref, L);

//        L_local.z = 0;
//        vec3 L_G = _G_R_g_pitroll*L_local;

//        if(L_G.z >= 0.06) L_G.z = 0.06;
//        if(L_G.z <= -0.06) L_G.z = -0.06;

//        //if(L_G.z > 0.02) L_G.z*0.5; // when landing heigh is (-) , reduce it to late landing

//        double alpha = 1/(1 + 2*PI*dt*5.0);
//        Landing_Z_filetered = alpha*Landing_Z_filetered + (1 - alpha)*(-L_G.z);

//        return Landing_Z_filetered;
//    }

//    if(_SwingFoot == 1){ // Left Foot Swing
//        vec3 L = (_pLF_des - _pRF_des)/1.0;

//        vec3 L_local = global2local_vec(_qRF_ref, _qLF_ref, L);

//        L_local.z = 0;

//        vec3 L_G = _G_R_g_pitroll*L_local;

//        if(L_G.z >= 0.06) L_G.z = 0.06;
//        if(L_G.z <= -0.06) L_G.z = -0.06;

//        //if(L_G.z > 0) L_G.z*0.5;;

//        double alpha = 1/(1 + 2*PI*dt*5.0);
//        Landing_Z_filetered = alpha*Landing_Z_filetered + (1 - alpha)*(-L_G.z);

//        return Landing_Z_filetered;
//    }

//    else{
//        Landing_Z_filetered = 0;
//    }

    if(_SwingFoot == -1){ // RightFoot Swing
        vec3 L = (_pRF_des - _pLF_des)/1.0;

        vec3 L_local = global2local_vec(_qRF_ref, _qLF_ref, L);

        L_local.z = 0;


        double L_y = G_R_g_pitroll_rpy.r*L_local.y;
        double L_x = -G_R_g_pitroll_rpy.p*L_local.x;

        if(L_y > 0) L_y = L_y*1.7;
        else L_y = L_y*1.5;

        double L_G = L_x + L_y;

        if(L_G >= 0.07) L_G = 0.07;
        if(L_G <= -0.07) L_G = -0.07;

        //if(L_G > 0.00) L_G = L_G*1.5; // when landing heigh is (-) , reduce it to late landing

        //if(t_now < 0.8*t_step){
            double alpha = 1/(1 + 2*PI*dt*2.0);
            Landing_Z_filetered = alpha*Landing_Z_filetered + (1.0 - alpha)*(-L_G);
        //}

        return Landing_Z_filetered;
    }

    if(_SwingFoot == 1){ // Left Foot Swing
        vec3 L = (_pLF_des - _pRF_des)/1.0;

        vec3 L_local = global2local_vec(_qRF_ref, _qLF_ref, L);

        L_local.z = 0;

        double L_y = G_R_g_pitroll_rpy.r*L_local.y;
        double L_x = -G_R_g_pitroll_rpy.p*L_local.x;

        if(L_y > 0) L_y = L_y*1.7;
        else L_y = L_y*1.5;

        double L_G = L_x + L_y;

        if(L_G >= 0.07) L_G = 0.07;
        if(L_G <= -0.07) L_G = -0.07;

        //if(L_G > 0) L_G = L_G*1.5;

        //if(t_now < 0.8*t_step){
            double alpha = 1/(1 + 2*PI*dt*2.0);
            Landing_Z_filetered = alpha*Landing_Z_filetered + (1.0 - alpha)*(-L_G);
        //}

        return Landing_Z_filetered;
    }

    else{
        double alpha = 1/(1 + 2*PI*dt*2.0);
        Landing_Z_filetered = alpha*Landing_Z_filetered + (1.0 - alpha)*0;
    }

    return 0;
}

vec3 HB_PreviewWalk::Calc_Landing_delXY(char _SwingFoot, vec3 _pLF_des, vec3 _pRF_des, quat _qLF_ref, quat _qRF_ref, mat3 _G_R_g_pitroll){
    vec3 Landing_delXY = vec3();

    if(_SwingFoot == -1){ // RightFoot Swing
        vec3 L = _pRF_des - _pLF_des;

        vec3 L_local = global2local_vec(_qRF_ref, _qLF_ref, L);
        L_local.z = 0;

        vec3 L_G = _G_R_g_pitroll*L_local;

        if(L_G.z > 0){ // control only if when Foot destination is under the ground
            Landing_delXY = vec3();
        }
        else{
            L_G.z = 0;
            Landing_delXY = L_local - L_G;
        }

    }

    if(_SwingFoot == 1){ // RightFoot Swing
        vec3 L = _pLF_des - _pRF_des;

        vec3 L_local = global2local_vec(_qRF_ref, _qLF_ref, L);
        L_local.z = 0;

        vec3 L_G = _G_R_g_pitroll*L_local;
        L_G.z = 0;

        if(L_G.z > 0){ // control only if when Foot destination is under the ground
            Landing_delXY = vec3();
        }
        else{
            L_G.z = 0;
            Landing_delXY = L_local - L_G;
        }
    }

    double alpha = 1/(1 + 2*PI*dt*5.0);
    Landing_delXY_filtered = alpha*Landing_delXY_filtered + (1 - alpha)*Landing_delXY;

    if(fabs(Landing_delXY.x) < 0.01) Landing_delXY.x = 0;
    if(fabs(Landing_delXY.y) < 0.01) Landing_delXY.y = 0;

    if(Landing_delXY_filtered.x >= 0.02) Landing_delXY_filtered.x = 0.02;
    if(Landing_delXY_filtered.x <= -0.02) Landing_delXY_filtered.x = -0.02;

    if(Landing_delXY_filtered.y >= 0.02) Landing_delXY_filtered.y = 0.02;
    if(Landing_delXY_filtered.y <= -0.02) Landing_delXY_filtered.y = -0.02;

    return local2global_vec(_qRF_ref, _qLF_ref, Landing_delXY_filtered);
}

void HB_PreviewWalk::Calc_del_pos_from_Joystick(int _RJOG_RL, int _RJOG_UD, int _LJOG_RL, int _LJOG_UD, double _des_t_step){

    //version 1
    double del_pos_Max_x = 0.30;
    double del_pos_min_x = 0.25;

    double del_pos_Max_y = 0.15;
    double del_pos_Max_yaw = 15; //deg

    if(_LJOG_UD >= 0)
    {
        del_pos.x =  -(double)(_LJOG_UD)/32767.0*del_pos_Max_x;
    }
    else
    {
        del_pos.x =  -(double)(_LJOG_UD)/32767.0*del_pos_min_x;
    }
    del_pos.y = (double)(_LJOG_RL)/32767.0*del_pos_Max_y;
    del_pos.z = -(double)(_RJOG_RL)/32767.0*del_pos_Max_yaw;

    if(_des_t_step >= 0.9) _des_t_step = 0.9;
    if(_des_t_step <= 0.54) _des_t_step = 0.54;

    des_step_t = _des_t_step;

    //version 2
//    double Max_speed_x = 0.45;
//    double dsp_ratio_MAX = 0.3;
//    double dsp_ratio_min = 0.1;

//    double step_t_MAX = 0.9;
//    double step_t_min = 0.7;

//    des_Velocity.x = (double)(_LJOG_UD)/32767.0*Max_speed_x;
//    des_dsp_ratio = dsp_ratio_MAX - (dsp_ratio_MAX - dsp_ratio_min)/Max_speed_x*fabs(des_Velocity.x);
//    des_step_t = step_t_MAX - (step_t_MAX - step_t_m in)/Max_speed_x*fabs(des_Velocity.x);
//    del_pos.x = -des_step_t*des_Velocity.x;


//    double del_pos_Max_y = 0.15;
//    double del_pos_Max_yaw = 20; //deg

//    del_pos.y = (double)(_LJOG_RL)/32767.0*del_pos_Max_y;
//    del_pos.z = -(double)(_RJOG_RL)/32767.0*del_pos_Max_yaw;

}

vec3 HB_PreviewWalk::StancF_to_NextF_limitation(char _swingFoot, vec3 _stancF_to_NextF){

    if(_swingFoot == LFoot){
        if(_stancF_to_NextF.y <= 0.19) _stancF_to_NextF.y = 0.19;
        if(_stancF_to_NextF.y >= 0.4) _stancF_to_NextF.y = 0.4;
    }
    if(_swingFoot == RFoot){
        if(_stancF_to_NextF.y >= -0.19) _stancF_to_NextF.y = -0.19;
        if(_stancF_to_NextF.y <= -0.4) _stancF_to_NextF.y = -0.4;
    }


    return _stancF_to_NextF;
}

double HB_PreviewWalk::Calc_Landing_Threshold(char _swingFoot, vec3 _cZMP_dsp){

    // calc alpha
//    double Alpha_landing;
//    Alpha_landing = 0.5 - (COM_e_imu_local.y*60 + dCOM_e_imu_local.y*5);
//    //    Alpha_dsp = 0.5 - (COM_m_filtered.y - COM_ref.y)*70 - (dCOM_m_filtered.y - dCOM_ref.y)*5;

//    if(Alpha_landing >= 1.0) Alpha_landing = 1.0;
//    if(Alpha_landing <= 0.0) Alpha_landing = 0.0;

    double Fz_total = 430.0; //32kg
    double RF_Threshold = Alpha_dsp*Fz_total + 30;
    double LF_Threshold = (1.0 - Alpha_dsp)*Fz_total + 30;

    if(RF_Threshold < 30.0) RF_Threshold = 30.0;
    if(LF_Threshold < 30.0) LF_Threshold = 30.0;

    if(_swingFoot == RFoot)
    {
        //if(RF_Threshold > 25) cout<<"RF_Threshold : "<<RF_Threshold<<endl;
        return RF_Threshold;
    }
    if(_swingFoot == LFoot)
    {
        //if(LF_Threshold > 25) cout<<"LF_Threshold : "<<LF_Threshold<<endl;
        return LF_Threshold;
    }
    else{
        return 30.0;
    }
}




int HB_PreviewWalk::Logging(){
    //// ------------------Logging---------------------------


    save_onestep(k);
    k++;
}

int HB_PreviewWalk::Test_init(vec3 _COM_ini, vec3 _pRF, quat _qRF, vec3 _pLF, quat _qLF){

    uCOM = _COM_ini;
    duCOM = vec3(); uCOM_old = vec3();
    COM_ref  = _COM_ini;
    dCOM_ref = vec3();
    pRF_ref = _pRF;
    pLF_ref = _pLF;
    qRF_ref = _qRF;
    qLF_ref = _qLF;
    p_ref[0] = _COM_ini;

    ZMP_global = vec3(_COM_ini.x, _COM_ini.y, 0);

    COM_m_old = _COM_ini;
    COM_m_filtered = _COM_ini;
    dCOM_m_filtered = vec3();
    dCOM_m_diff_filtered = vec3();
    dCOM_m_imu_filtered = vec3();



    RF_del_angle = vec3();
    LF_del_angle = vec3();
    RF_del_angle_old = vec3();
    LF_del_angle_old = vec3();
    RF_del_pos = vec3();
    LF_del_pos = vec3();



    Y_obs = vec3(0, 0, 0);
    X_obs = vec3(0, 0, 0);
    dY_obs = vec3(); dX_obs = vec3();

    // SwingPhase Vibration control
    RHR_con_deg = 0;
    LHR_con_deg = 0;
    RHY_con_deg = 0;
    LHY_con_deg = 0;


    ACC_RF_Hfiltered = vec3();
    ACC_LF_Hfiltered = vec3();
    ACC_RF_filtered = vec3();
    ACC_LF_filtered = vec3();
    ACC_RF_old = vec3();
    ACC_LF_old = vec3();

    L_roll_obs = vec3();
    R_roll_obs = vec3();

    X_RTorsoRoll = vec3();
    dX_RTorsoRoll = vec3();

    X_RTorsoYaw = vec3();
    dX_RTorsoYaw = vec3();

    X_LTorsoRoll = vec3();
    dX_LTorsoRoll = vec3();

    X_LTorsoYaw = vec3();
    dX_LTorsoYaw = vec3();

    // Ankle Torque Control
    RF_del_angle = vec3();
    LF_del_angle = vec3();
    RF_del_angle_old = vec3();
    LF_del_angle_old = vec3();
    RF_del_pos = vec3();
    LF_del_pos = vec3();
    RF_del_quat = quat();
    LF_del_quat = quat();

    // dsp controller
    z_ctrl = 0;
    z_ctrl_filtered = 0;
    dz_ctrl_filtered = 0;


    k = 0;

}

void HB_PreviewWalk::WindowFill(){
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

    //// -- for current step --------------------------------------

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

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    //// ----- for (2 ~ N-1) step
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

void HB_PreviewWalk::WindowFill_3rd(){
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
    if(dT >= SDB[step_phase].t - t_half_dsp){
        double dsp_timer = 2*t_half_dsp - (dT - (SDB[step_phase].t - t_half_dsp)); // time for 5th trajectory interval

        // 3th parametor
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;
        if(step_phase >= 1){
            calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase-1].Fpos.y,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase-1].Fpos.x,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        }
        else{
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        }

        for(int i=0 ; i<(int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = (int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq);

        for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD[i].ZMP_ref = SDB[step_phase].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        dsp_timer = 0;

        for(int i = last_tic; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
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

        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        double dsp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);
    }
    else{// if(dT >= 0){
        double dsp_timer = t_half_dsp - dT;

        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio_com*0.5;

        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j - 1].Fpos.y,0,0), vec3(SDB[step_phase + j].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j - 1].Fpos.x,0,0), vec3(SDB[step_phase + j].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic ; i < last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD[i].ZMP_ref = SDB[step_phase + j].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j].Fpos.y,0,0), vec3(SDB[step_phase + j + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j].Fpos.x,0,0), vec3(SDB[step_phase + j + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;
        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp+ + dt/2)*freq);
    }

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio_com*0.5;

    if((int)((t_prev + dt/2)*freq) + 1 - last_tic  < (int)((t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < (int)((t_prev + dt/2)*freq) + 1; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }

    }
    else if((int)((t_prev + dt/2)*freq) + 1 - last_tic < (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
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
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i < last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq); i++){
            WD[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;

        for(int i = last_tic ; i < (int)((t_prev + dt/2)*freq) + 1 ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
    }
}

void HB_PreviewWalk::WindowFill_3rd_heel2toe(){
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


    //// first preview Interval --------------------------------------------------------------------------------------===================================

    // put ZMP tracjectory to Window
    double t_half_dsp = SDB[step_phase].t*dsp_ratio_com*0.5;

    //// case 1
    if(dT >= SDB[step_phase].t - t_half_dsp){

        //// Interval 1
        double dsp_timer = 2*t_half_dsp - (dT - (SDB[step_phase].t - t_half_dsp)); // time for 5th trajectory interval

        // 3th parametor  Last Toe to Next Heel
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;
        if(step_phase >= 1){
            vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase].swingFoot, SDB[step_phase-1].Fpos, SDB[step_phase-1].Fquat, SDB[step_phase].Fpos, SDB[step_phase].Fquat);

            vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
            vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);


            calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);
        }
        else{
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        }

        for(int i=0 ; i<(int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = (int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq);


        //// interval 2

        double ssp_timer = 0;
        if(step_phase >= 1){
            vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase].swingFoot, SDB[step_phase-1].Fpos, SDB[step_phase-1].Fquat, SDB[step_phase].Fpos, SDB[step_phase].Fquat);

            vec3 Cur_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);  // Next heel --> cur heel

            Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

            vec3 Cur_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);   // Last_toe  --> cur toe



            calc_1st_param(0, SDB[step_phase].t - t_half_dsp*2, Cur_heel.y, Cur_toe.y,Ay,By);
            calc_1st_param(0, SDB[step_phase].t - t_half_dsp*2, Cur_heel.x, Cur_toe.x,Ax,Bx);

            for(int i=last_tic ; i<last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
                double t1 = ssp_timer;

                double zmpY = Ay*t1 + By;
                double zmpX = Ax*t1 + Bx;
                ssp_timer += dt;

                WD[i].ZMP_ref.y = zmpY;
                WD[i].ZMP_ref.x = zmpX;
            }

        }
        else{

            for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
                WD[i].ZMP_ref = SDB[step_phase].Fpos;
            }

        }
        last_tic = last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq);



        //// Interval 3

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;

        for(int i = last_tic; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

    }


    //// case 2
    else if(dT > t_half_dsp){

        //// No Interval 1
        //
        //


        //// Interval 2
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        if(step_phase >= 1){
            vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase].swingFoot, SDB[step_phase-1].Fpos, SDB[step_phase-1].Fquat, SDB[step_phase].Fpos, SDB[step_phase].Fquat);

            vec3 Cur_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);  // Next heel --> cur heel

            Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

            vec3 Cur_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);   // Last_toe  --> cur toe



            calc_1st_param(0, SDB[step_phase].t - t_half_dsp*2, Cur_heel.y, Cur_toe.y,Ay,By);
            calc_1st_param(0, SDB[step_phase].t - t_half_dsp*2, Cur_heel.x, Cur_toe.x,Ax,Bx);

            double ssp_timer = SDB[step_phase].t - dT - t_half_dsp;

            for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
                double t1 = ssp_timer;

                double zmpY = Ay*t1 + By;
                double zmpX = Ax*t1 + Bx;
                ssp_timer += dt;

                WD[i].ZMP_ref.y = zmpY;
                WD[i].ZMP_ref.x = zmpX;
            }

        }
        else{
            for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
                WD[i].ZMP_ref = SDB[step_phase].Fpos;
            }
        }
        last_tic = (int)((dT - t_half_dsp + dt/2)*freq);


        //// Interval 3

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);
    }

    //// case 3
    else{// if(dT >= 0){

        //// Interval 3
        double dsp_timer = t_half_dsp - dT;

        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    //// Repeating preview Interval --------------------------------------------------------------------------------------===================================

    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio_com*0.5;

        //// Interval 1
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j].swingFoot, SDB[step_phase + j - 1].Fpos, SDB[step_phase + j - 1].Fquat, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);


        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);


        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);


        //// Interval 2

        Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j].swingFoot, SDB[step_phase + j - 1].Fpos, SDB[step_phase + j - 1].Fquat, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat);

        vec3 Cur_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);  // Next heel --> cur heel

        Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j + 1].swingFoot, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat, SDB[step_phase + j + 1].Fpos, SDB[step_phase + j + 1].Fquat);

        vec3 Cur_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);   // Last_toe  --> cur toe



        calc_1st_param(0, SDB[step_phase].t - t_half_dsp*2, Cur_heel.y, Cur_toe.y,Ay,By);
        calc_1st_param(0, SDB[step_phase].t - t_half_dsp*2, Cur_heel.x, Cur_toe.x,Ax,Bx);

        double ssp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            double t1 = ssp_timer;

            double zmpY = Ay*t1 + By;
            double zmpX = Ax*t1 + Bx;
            ssp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }

        last_tic = last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq);


        //// Interval 3

        Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j+1].swingFoot, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat, SDB[step_phase + j+1].Fpos, SDB[step_phase + j+1].Fquat);

        Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;
        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp+ + dt/2)*freq);
    }


    //// Last preview Interval --------------------------------------------------------------------------------------===================================


    //// case 1

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio_com*0.5;

    if((int)((t_prev + dt/2)*freq) + 1 - last_tic  < (int)((t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < (int)((t_prev + dt/2)*freq) + 1; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }

    }
    else if((int)((t_prev + dt/2)*freq) + 1 - last_tic < (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
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
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i < last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq); i++){
            WD[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;

        for(int i = last_tic ; i < (int)((t_prev + dt/2)*freq) + 1 ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
    }
}

void HB_PreviewWalk::WindowFill_3rd_heel2toe_3rd(){
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


    //// first preview Interval --------------------------------------------------------------------------------------===================================

    // put ZMP tracjectory to Window
    double t_half_dsp = SDB[step_phase].t*dsp_ratio_com*0.5;

    //// case 1
    if(dT >= SDB[step_phase].t - t_half_dsp){

        //// Interval 1
        double dsp_timer = 2*t_half_dsp - (dT - (SDB[step_phase].t - t_half_dsp)); // time for 5th trajectory interval

        // 3th parametor  Last Toe to Next Heel
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;
        if(step_phase >= 1){
            vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase].swingFoot, SDB[step_phase-1].Fpos, SDB[step_phase-1].Fquat, SDB[step_phase].Fpos, SDB[step_phase].Fquat);

            vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
            vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);


            calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);
        }
        else{
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        }

        for(int i=0 ; i<(int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = (int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq);


        //// interval 2

        double ssp_timer = 0;
        if(step_phase >= 1){
            vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase].swingFoot, SDB[step_phase-1].Fpos, SDB[step_phase-1].Fquat, SDB[step_phase].Fpos, SDB[step_phase].Fquat);

            vec3 Cur_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);  // Next heel --> cur heel

            Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

            vec3 Cur_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);   // Last_toe  --> cur toe



            calc_3rd_param(0, SDB[step_phase].t - t_half_dsp*2, vec3(Cur_heel.y,0,0), vec3(Cur_toe.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, SDB[step_phase].t - t_half_dsp*2, vec3(Cur_heel.x,0,0), vec3(Cur_toe.x,0,0),Ax,Bx,Cx,Dx);

            for(int i=last_tic ; i<last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
                double t1 = ssp_timer;
                double t2 = t1*t1;
                double t3 = t2*t1;

                double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
                double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
                ssp_timer += dt;

                WD[i].ZMP_ref.y = zmpY;
                WD[i].ZMP_ref.x = zmpX;
            }

        }
        else{

            for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
                WD[i].ZMP_ref = SDB[step_phase].Fpos;
            }

        }
        last_tic = last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq);



        //// Interval 3

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;

        for(int i = last_tic; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

    }


    //// case 2
    else if(dT > t_half_dsp){

        //// No Interval 1
        //
        //


        //// Interval 2
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        if(step_phase >= 1){
            vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase].swingFoot, SDB[step_phase-1].Fpos, SDB[step_phase-1].Fquat, SDB[step_phase].Fpos, SDB[step_phase].Fquat);

            vec3 Cur_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);  // Next heel --> cur heel

            Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

            vec3 Cur_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);   // Last_toe  --> cur toe



            calc_3rd_param(0, SDB[step_phase].t - t_half_dsp*2, vec3(Cur_heel.y,0,0), vec3(Cur_toe.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, SDB[step_phase].t - t_half_dsp*2, vec3(Cur_heel.x,0,0), vec3(Cur_toe.x,0,0),Ax,Bx,Cx,Dx);

            double ssp_timer = SDB[step_phase].t - dT - t_half_dsp;

            for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
                double t1 = ssp_timer;
                double t2 = t1*t1;
                double t3 = t2*t1;

                double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
                double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
                ssp_timer += dt;

                WD[i].ZMP_ref.y = zmpY;
                WD[i].ZMP_ref.x = zmpX;
            }

        }
        else{
            for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
                WD[i].ZMP_ref = SDB[step_phase].Fpos;
            }
        }
        last_tic = (int)((dT - t_half_dsp + dt/2)*freq);


        //// Interval 3

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);
    }

    //// case 3
    else{// if(dT >= 0){

        //// Interval 3
        double dsp_timer = t_half_dsp - dT;

        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase+1].swingFoot, SDB[step_phase].Fpos, SDB[step_phase].Fquat, SDB[step_phase+1].Fpos, SDB[step_phase+1].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    //// Repeating preview Interval --------------------------------------------------------------------------------------===================================

    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio_com*0.5;

        //// Interval 1
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        vec4 Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j].swingFoot, SDB[step_phase + j - 1].Fpos, SDB[step_phase + j - 1].Fquat, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat);

        vec3 Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        vec3 Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);


        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);


        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);


        //// Interval 2

        Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j].swingFoot, SDB[step_phase + j - 1].Fpos, SDB[step_phase + j - 1].Fquat, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat);

        vec3 Cur_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);  // Next heel --> cur heel

        Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j + 1].swingFoot, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat, SDB[step_phase + j + 1].Fpos, SDB[step_phase + j + 1].Fquat);

        vec3 Cur_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);   // Last_toe  --> cur toe



        calc_3rd_param(0, SDB[step_phase].t - t_half_dsp*2, vec3(Cur_heel.y,0,0), vec3(Cur_toe.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, SDB[step_phase].t - t_half_dsp*2, vec3(Cur_heel.x,0,0), vec3(Cur_toe.x,0,0),Ax,Bx,Cx,Dx);

        double ssp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            double t1 = ssp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            ssp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }

        last_tic = last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq);


        //// Interval 3

        Last_toe_and_Next_heel = Last_toe_Next_heel(SDB[step_phase + j+1].swingFoot, SDB[step_phase + j].Fpos, SDB[step_phase + j].Fquat, SDB[step_phase + j+1].Fpos, SDB[step_phase + j+1].Fquat);

        Last_toe = vec3(Last_toe_and_Next_heel[0], Last_toe_and_Next_heel[1], 0);
        Next_heel = vec3(Last_toe_and_Next_heel[2], Last_toe_and_Next_heel[3], 0);

        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.y,0,0), vec3(Next_heel.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(Last_toe.x,0,0), vec3(Next_heel.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;
        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp+ + dt/2)*freq);
    }


    //// Last preview Interval --------------------------------------------------------------------------------------===================================


    //// case 1

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio_com*0.5;

    if((int)((t_prev + dt/2)*freq) + 1 - last_tic  < (int)((t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < (int)((t_prev + dt/2)*freq) + 1; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }

    }
    else if((int)((t_prev + dt/2)*freq) + 1 - last_tic < (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
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
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i < last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq); i++){
            WD[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;

        for(int i = last_tic ; i < (int)((t_prev + dt/2)*freq) + 1 ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;
        }
    }
}

void HB_PreviewWalk::WindowFill_SA_ref(){
    int last_tic;
    double t_prev_plus_t_step = t_prev + t_step;

    ////-------put ZMP trajectory to Window
    // Count how many step phase in preview time
    int temp_count = (int)((t_prev_plus_t_step - dT + dt/2)*freq);
    No_of_cycle = 0;
    while(1){
        if(temp_count - (int)((SDB[step_phase + No_of_cycle + 1].t + dt/2)*freq) <= 0){
            break;
        }
        else{
            temp_count -= (int)((SDB[step_phase + No_of_cycle + 1].t + dt/2)*freq);
        }

        No_of_cycle += 1;
    }

    // put ZMP tracjectory to Window
    double t_half_dsp = SDB[step_phase].t*dsp_ratio_com*0.5;
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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = (int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq);

        for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase].Fpos;
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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

    }
    else if(dT > t_half_dsp){
        for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase].Fpos;
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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);
    }
    else{// if(dT >= 0){
        double dsp_timer = t_half_dsp - dT;

        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;


            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic ; i < last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase + j].Fpos;
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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp+ + dt/2)*freq);
    }

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio_com*0.5;

    if((int)((t_prev_plus_t_step + dt/2)*freq) + 1 - last_tic  < (int)((t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < (int)((t_prev_plus_t_step + dt/2)*freq) + 1; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }

    }
    else if((int)((t_prev_plus_t_step + dt/2)*freq) + 1 - last_tic < (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp + dt/2)*freq)){
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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i<(int)((t_prev_plus_t_step + dt/2)*freq) + 1; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
    }
    else{// if((int)(t_prev_plus_t_step*freq) - 1 - last_tic <= (int)((SDB[step_phase + No_of_cycle + 1].t)*freq)){
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

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i < last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq); i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq);

        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.y,0,0),Ay,By,Cy,Dy,Ey,Fy);
        calc_5th_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.x,0,0),Ax,Bx,Cx,Dx,Ex,Fx);

        dsp_timer = 0;

        for(int i = last_tic ; i < (int)((t_prev_plus_t_step + dt/2)*freq) + 1 ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
    }
}

void HB_PreviewWalk::WindowFill_SA_ref_3rd(){
    int last_tic;
    double t_prev_plus_t_step = t_prev + t_step;

    ////-------put ZMP trajectory to Window
    // Count how many step phase in preview time
    int temp_count = (int)((t_prev_plus_t_step - dT + dt/2)*freq);
    No_of_cycle = 0;
    while(1){
        if(temp_count - (int)((SDB[step_phase + No_of_cycle + 1].t + dt/2)*freq) <= 0){
            break;
        }
        else{
            temp_count -= (int)((SDB[step_phase + No_of_cycle + 1].t + dt/2)*freq);
        }

        No_of_cycle += 1;
    }

    // put ZMP tracjectory to Window
    double t_half_dsp = SDB[step_phase].t*dsp_ratio_com*0.5;
    if(dT >= SDB[step_phase].t - t_half_dsp){
        double dsp_timer = 2*t_half_dsp - (dT - (SDB[step_phase].t - t_half_dsp)); // time for 5th trajectory interval

        // 5th parametor
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;
        if(step_phase >= 1){
            calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase-1].Fpos.y,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase-1].Fpos.x,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        }
        else{
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.y,0,0),Ay,By,Cy,Dy);
            calc_3rd_param(0, t_half_dsp*2, vec3(0,0,0), vec3(SDB[step_phase].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        }

        for(int i=0 ; i<(int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = (int)((dT - (SDB[step_phase].t - t_half_dsp) + dt/2)*freq);

        for(int i=last_tic ; i< last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        dsp_timer = 0;

        for(int i = last_tic; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

    }
    else if(dT > t_half_dsp){
        for(int i=0 ; i<(int)((dT - t_half_dsp + dt/2)*freq) ; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase].Fpos;
        }
        last_tic = (int)((dT - t_half_dsp + dt/2)*freq);

        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);
        double dsp_timer = 0;

        for(int i=last_tic ; i< last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);
    }
    else{// if(dT >= 0){
        double dsp_timer = t_half_dsp - dT;

        double Ay,By,Cy,Dy,Ey,Fy, Ax,Bx,Cx,Dx,Ex,Fx;
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.y,0,0), vec3(SDB[step_phase + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase].Fpos.x,0,0), vec3(SDB[step_phase + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;


            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio_com*0.5;

        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j - 1].Fpos.y,0,0), vec3(SDB[step_phase + j].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j - 1].Fpos.x,0,0), vec3(SDB[step_phase + j].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic ; i < last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq) ; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase + j].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + j].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j].Fpos.y,0,0), vec3(SDB[step_phase + j + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + j].Fpos.x,0,0), vec3(SDB[step_phase + j + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;
        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp+ + dt/2)*freq);
    }

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio_com*0.5;

    if((int)((t_prev_plus_t_step + dt/2)*freq) + 1 - last_tic  < (int)((t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < (int)((t_prev_plus_t_step + dt/2)*freq) + 1; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }

    }
    else if((int)((t_prev_plus_t_step + dt/2)*freq) + 1 - last_tic < (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp + dt/2)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp ;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;


            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i<(int)((t_prev_plus_t_step + dt/2)*freq) + 1; i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
    }
    else{// if((int)(t_prev_plus_t_step*freq) - 1 - last_tic <= (int)((SDB[step_phase + No_of_cycle + 1].t)*freq)){
        double Ay,By,Cy,Dy, Ax,Bx,Cx,Dx;

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        double dsp_timer = t_half_dsp;

        for(int i=last_tic ; i < last_tic + (int)((t_half_dsp + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
        last_tic = last_tic + (int)((t_half_dsp + dt/2)*freq);

        for(int i=last_tic; i < last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq); i++){
            WD_SA_ref[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
        }
        last_tic = last_tic + (int)((SDB[step_phase + No_of_cycle + 1].t - t_half_dsp*2 + dt/2)*freq);

        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.y,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.y,0,0),Ay,By,Cy,Dy);
        calc_3rd_param(0, t_half_dsp*2, vec3(SDB[step_phase + No_of_cycle + 1].Fpos.x,0,0), vec3(SDB[step_phase + No_of_cycle + 2].Fpos.x,0,0),Ax,Bx,Cx,Dx);

        dsp_timer = 0;

        for(int i = last_tic ; i < (int)((t_prev_plus_t_step + dt/2)*freq) + 1 ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;

            double zmpY = Ay*t3 + By*t2 + Cy*t1 + Dy;
            double zmpX = Ax*t3 + Bx*t2 + Cx*t1 + Dx;
            dsp_timer += dt;

            WD_SA_ref[i].ZMP_ref.y = zmpY;
            WD_SA_ref[i].ZMP_ref.x = zmpX;
        }
    }
}

void HB_PreviewWalk::WindowFill_SA_ref2(){
    // this function perfomed only once at the start of step phase change
    int last_tic = 0;
    double t_prev_plus_t_step = t_prev + t_step;

    // Count how many step phase in preview time
    int temp_count = (int)((t_prev_plus_t_step + dt/2)*freq);
    int No_of_cycle = 0;
    while(1){
        if(temp_count - (int)((SDB[step_phase + No_of_cycle].t + dt/2)*freq) <= 0){
            break;
        }
        else{
            temp_count -= (int)((SDB[step_phase + No_of_cycle].t + dt/2)*freq);
        }

        No_of_cycle += 1;
    }

    // put ZMP trajectory to the window
    for(int j=0; j< No_of_cycle ; j++){

        for(int i=last_tic; i<last_tic + (int)((SDB[step_phase + j].t + dt/2)*freq) ; i++){

            WD_SA_ref[i].ZMP_ref = SDB[step_phase + j].Fpos;

        }
        last_tic = last_tic + (int)((SDB[step_phase + j].t + dt/2)*freq);
    }

    for(int i=last_tic; i < (int)((t_prev_plus_t_step + dt/2)*freq) + 1; i++){
        WD_SA_ref[i].ZMP_ref = SDB[step_phase + No_of_cycle].Fpos;

    }

}

void HB_PreviewWalk::save_onestep(int cnt)
{
    if(k<200000)
    {
        SAVE[0][cnt] = COM_ref.x;
        SAVE[1][cnt] = CP_ref.x;
        SAVE[2][cnt] = COM_ref.y;
        SAVE[3][cnt] = CP_ref.y;
        SAVE[4][cnt] = p_ref[0].x;
        SAVE[5][cnt] = p_ref[0].y;
        SAVE[6][cnt] = p_out.x;
        SAVE[7][cnt] = p_out.y;
        SAVE[8][cnt] = 0;
        SAVE[9][cnt] = 0;

        SAVE[10][cnt] = dCOM_ref.x;
        SAVE[11][cnt] = dCOM_ref.y;
        SAVE[12][cnt] = cZMP.x;
        SAVE[13][cnt] = cZMP.y;
        SAVE[14][cnt] = ZMP_global.x;
        SAVE[15][cnt] = ZMP_global.y;
        SAVE[16][cnt] = cZMP_proj.x;
        SAVE[17][cnt] = cZMP_proj.y;

        SAVE[18][cnt] = RS.IMUangle.x;
        SAVE[19][cnt] = RS.IMUangle.y;
        SAVE[20][cnt] = RS.IMUangle.z;

        SAVE[21][cnt] = X_obs[0];
        SAVE[22][cnt] = Y_obs[0];
        SAVE[23][cnt] = X_obs[1];
        SAVE[24][cnt] = Y_obs[1];
        SAVE[25][cnt] = RS.IMULocalW.x;
        SAVE[26][cnt] = RS.IMULocalW.y;

        SAVE[27][cnt] = dT;//dT_est;//dT_buf[0];
        SAVE[28][cnt] = pRF_ref.x;
        SAVE[29][cnt] = pLF_ref.x;

        SAVE[30][cnt] = pRF_ref.z;
        SAVE[31][cnt] = pLF_ref.z;
        SAVE[32][cnt] = 0;

        SAVE[33][cnt] = RS.CSP.pCOM.x; // COM_measure x
        SAVE[34][cnt] = RS.CSP.pCOM.y; // COM_measure y

        SAVE[35][cnt] = uCOM.x;
        SAVE[36][cnt] = uCOM.y;

        SAVE[37][cnt] = CP_m_filtered.x; // temp x
        SAVE[38][cnt] = CP_m_filtered.y; // temp y

        SAVE[39][cnt] = 0;
        SAVE[40][cnt] = SDB[step_phase].swingFoot;

        SAVE[41][cnt] = COM_m.x;
        SAVE[42][cnt] = COM_m.y;
        SAVE[43][cnt] = 0;
        SAVE[44][cnt] = 0;

        SAVE[45][cnt] = CP_m.y; // ZMP x estimation
        SAVE[46][cnt] = CP_m.x;
        SAVE[47][cnt] = 0;
        SAVE[48][cnt] = pRF_ref.x;
        SAVE[49][cnt] = pRF_ref.y;

        SAVE[50][cnt] = F_RF_filtered.z;
        SAVE[51][cnt] = F_LF_filtered.z;
        SAVE[52][cnt] = pLF_ref.x;
        SAVE[53][cnt] = pLF_ref.y;
        SAVE[54][cnt] = RS.CSV.dpCOM.y;

        SAVE[55][cnt] = RS.CSV.dpCOM.x;
        SAVE[56][cnt] = RS.CSV.dpCOM.y;
        SAVE[57][cnt] = 0;

        SAVE[58][cnt] = ACC_RF_filtered.x;
        SAVE[59][cnt] = ACC_RF_filtered.y;
        SAVE[60][cnt] = ACC_LF_filtered.x;
        SAVE[61][cnt] = ACC_LF_filtered.y;

        SAVE[62][cnt] = Ye_obs[0];
        SAVE[63][cnt] = Ye_obs[1];
        SAVE[64][cnt] = Xe_obs[0];
        SAVE[65][cnt] = Xe_obs[1];
        SAVE[66][cnt] = L_roll_obs[0];
        SAVE[67][cnt] = LHR_con_deg;

        SAVE[68][cnt] = dCOM_m_diff.x;
        SAVE[69][cnt] = dCOM_m_diff.y;
        SAVE[70][cnt] = dCOM_m_imu.x;
        SAVE[71][cnt] = dCOM_m_imu.y;
        SAVE[72][cnt] = 0;
        SAVE[73][cnt] = 0;

        SAVE[74][cnt] = RS.M_RF.x;
        SAVE[75][cnt] = RS.M_RF.y;
        SAVE[76][cnt] = RS.M_LF.x;
        SAVE[77][cnt] = RS.M_LF.y;
        SAVE[78][cnt] = 0;
        SAVE[79][cnt] = 0;



        //SAVE[47][cnt] = realCOM.z;//
    }
}

void HB_PreviewWalk::save_all()
{
    printf("walk finished %d\n",k);
    FILE* ffp = fopen("/home/rainbow/Desktop/HBtest_Walking_Data_prev1.txt","w");
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
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
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
                ,SAVE[68][i]
                ,SAVE[69][i]
                ,SAVE[70][i]
                ,SAVE[71][i]
                ,SAVE[72][i]
                ,SAVE[73][i]
                );
        fprintf(ffp,"%f\t%f\t%f\t%f\t%f\t%f\n"
                ,SAVE[74][i]
                ,SAVE[75][i]
                ,SAVE[76][i]
                ,SAVE[77][i]
                ,SAVE[78][i]
                ,SAVE[79][i]
                );
    }
    fclose(ffp);
}

//void HB_PreviewWalk::save_WD()
//{
//    for(int i = 0; i<k ; i++){
//        char name[100];
//        sprintf(name, "/home/rainbow/Desktop/Prev_walk_WindowData/Window_%d.txt",i);
//        FILE* ffp2 = fopen(name,"w");
//        for(int j=0; j<NL; j++){
//            fprintf(ffp2,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"
//                    ,WDSAVE[i][0][j]
//                    ,WDSAVE[i][1][j]
//                    ,WDSAVE[i][2][j]
//                    ,WDSAVE[i][3][j]
//                    ,WDSAVE[i][4][j]
//                    ,WDSAVE[i][5][j]
//                    ,WDSAVE[i][6][j]
//                    ,WDSAVE[i][7][j]
//                    ,WDSAVE[i][8][j]
//                    ,WDSAVE[i][9][j]
//                    );
//        }
//        fclose(ffp2);
//    }


//}

/*
void HB_PreviewWalk::WindowFill_3mass(){
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
    double t_half_dsp = SDB[step_phase].t*dsp_ratio*0.5;
    if(dT >= SDB[step_phase].t - t_half_dsp){
        double dsp_timer = 2*t_half_dsp - (dT - (SDB[step_phase].t - t_half_dsp));

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

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio*0.5;

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

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio*0.5;

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



//    for(int i=0; i<(int)(dT*freq); i++){
//        WD[i].ZMP_ref = SDB[step_phase].Fpos;
//    }
//    last_tic = (int)(dT*freq);

//    for(int j=1; j<= No_of_cycle; j++){
//        for(int i=last_tic; i < (int)(last_tic + SDB[step_phase + j].t*freq); i++){
//            WD[i].ZMP_ref = SDB[step_phase + j].Fpos;
//        }
//        last_tic = (int)(last_tic + SDB[step_phase + j].t*freq);
//    }

//    for(int i=last_tic; i<(int)(t_prev*freq) + 1; i++){
//        WD[i].ZMP_ref = SDB[step_phase + No_of_cycle + 1].Fpos;
//    }



    ////----------put Foot trajectory to Window-------------------------------
    RF_x_dx_ddx_old_w = RF_x_dx_ddx_old;
    RF_y_dy_ddy_old_w = RF_y_dy_ddy_old;
    RF_z_dz_ddz_old_w = RF_z_dz_ddz_old;

    LF_x_dx_ddx_old_w = LF_x_dx_ddx_old;
    LF_y_dy_ddy_old_w = LF_y_dy_ddy_old;
    LF_z_dz_ddz_old_w = LF_z_dz_ddz_old;

    t_now_w = t_now;

    for(int i=0; i<(int)(dT*freq); i++){
        FootUp_height = 0.03/0.4*t_step;

        if(SDB[step_phase].swingFoot == RFoot){
            // Z direction
            RF_z_dz_ddz_w = FootZ_trajectory(SDB[step_phase].t, t_now_w, dsp_ratio, RF_z_dz_ddz_old_w, FootUp_height, 0.0);
            LF_z_dz_ddz_w = LF_z_dz_ddz_old_w;

            // X direction
            RF_x_dx_ddx_w = FootX_trajectory(SDB[step_phase].t, t_now_w, dsp_ratio, RF_x_dx_ddx_old_w, SDB[step_phase+1].Fpos.x);
            LF_x_dx_ddx_w = LF_x_dx_ddx_old_w;
        }
        else if(SDB[step_phase].swingFoot == LFoot){
            // Z direction
            LF_z_dz_ddz_w = FootZ_trajectory(SDB[step_phase].t, t_now_w, dsp_ratio, LF_z_dz_ddz_old_w, FootUp_height, 0.0);
            RF_z_dz_ddz_w = RF_z_dz_ddz_old_w;

            // X direction
            LF_x_dx_ddx_w = FootX_trajectory(SDB[step_phase].t, t_now_w, dsp_ratio, LF_x_dx_ddx_old_w, SDB[step_phase+1].Fpos.x);
            RF_x_dx_ddx_w = RF_x_dx_ddx_old_w;
        }
        else{
            RF_z_dz_ddz_w = RF_z_dz_ddz_old_w;
            LF_z_dz_ddz_w = LF_z_dz_ddz_old_w;

            RF_x_dx_ddx_w = RF_x_dx_ddx_old_w;
            LF_x_dx_ddx_w = LF_x_dx_ddx_old_w;
        }

        WD[i].RF_z_dz_ddz = RF_z_dz_ddz_w;
        WD[i].RF_x_dx_ddx = RF_x_dx_ddx_w;

        WD[i].LF_z_dz_ddz = LF_z_dz_ddz_w;
        WD[i].LF_x_dx_ddx = LF_x_dx_ddx_w;

        RF_z_dz_ddz_old_w = RF_z_dz_ddz_w;
        LF_z_dz_ddz_old_w = LF_z_dz_ddz_w;
        RF_x_dx_ddx_old_w = RF_x_dx_ddx_w;
        LF_x_dx_ddx_old_w = LF_x_dx_ddx_w;

        t_now_w += dt;

    }
    last_tic = (int)(dT*freq);
    t_now_w = 0;

    for(int j=1; j<= No_of_cycle; j++){
        for(int i=last_tic; i < (int)(last_tic + SDB[step_phase + j].t*freq); i++){
            FootUp_height = 0.03/0.4*SDB[step_phase+j].t;

            if(SDB[step_phase+j].swingFoot == RFoot){ // RFoot
                // Z direction
                RF_z_dz_ddz_w = FootZ_trajectory(SDB[step_phase+j].t, t_now_w, dsp_ratio, RF_z_dz_ddz_old_w, FootUp_height, 0.0);

                // X direction
                RF_x_dx_ddx_w = FootX_trajectory(SDB[step_phase+j].t, t_now_w, dsp_ratio, RF_x_dx_ddx_old_w, SDB[step_phase+j+1].Fpos.x);
            }
            else if(SDB[step_phase+j].swingFoot == LFoot){ // LFoot
                // Z direction
                LF_z_dz_ddz_w = FootZ_trajectory(SDB[step_phase+j].t, t_now_w, dsp_ratio, LF_z_dz_ddz_old_w, FootUp_height, 0.0);

                // X direction
                LF_x_dx_ddx_w = FootX_trajectory(SDB[step_phase+j].t, t_now_w, dsp_ratio, LF_x_dx_ddx_old_w, SDB[step_phase+j+1].Fpos.x);
            }
            else{ //DSP
                RF_z_dz_ddz_w = RF_z_dz_ddz_old_w;
                LF_z_dz_ddz_w = LF_z_dz_ddz_old_w;

                RF_x_dx_ddx_w = RF_x_dx_ddx_old_w;
                LF_x_dx_ddx_w = LF_x_dx_ddx_old_w;
            }

            WD[i].RF_z_dz_ddz = RF_z_dz_ddz_w;
            WD[i].RF_x_dx_ddx = RF_x_dx_ddx_w;

            WD[i].LF_z_dz_ddz = LF_z_dz_ddz_w;
            WD[i].LF_x_dx_ddx = LF_x_dx_ddx_w;

            RF_z_dz_ddz_old_w = RF_z_dz_ddz_w;
            LF_z_dz_ddz_old_w = LF_z_dz_ddz_w;
            RF_x_dx_ddx_old_w = RF_x_dx_ddx_w;
            LF_x_dx_ddx_old_w = LF_x_dx_ddx_w;

            t_now_w += dt;
        }
        last_tic = (int)(last_tic + SDB[step_phase + j].t*freq);
        t_now_w = 0;
    }

    for(int i=last_tic; i<(int)(t_prev*freq); i++){
        FootUp_height = 0.03/0.4*SDB[step_phase+No_of_cycle+1].t;

        if(SDB[step_phase+No_of_cycle+1].swingFoot == RFoot){ // RFoot
            // Z direction
            RF_z_dz_ddz_w = FootZ_trajectory(SDB[step_phase+No_of_cycle+1].t, t_now_w, dsp_ratio, RF_z_dz_ddz_old_w, FootUp_height, 0.00);

            // X direction
            RF_x_dx_ddx_w = FootX_trajectory(SDB[step_phase+No_of_cycle+1].t, t_now_w, dsp_ratio, RF_x_dx_ddx_old_w, SDB[step_phase+No_of_cycle+2].Fpos.x);
        }
        else if(SDB[step_phase+No_of_cycle+1].swingFoot == LFoot){ // LFoot
            // Z direction
            LF_z_dz_ddz_w = FootZ_trajectory(SDB[step_phase+No_of_cycle+1].t, t_now_w, dsp_ratio, LF_z_dz_ddz_old_w, FootUp_height, 0.00);

            // X direction
            LF_x_dx_ddx_w = FootX_trajectory(SDB[step_phase+No_of_cycle+1].t, t_now_w, dsp_ratio, LF_x_dx_ddx_old_w, SDB[step_phase+No_of_cycle+2].Fpos.x);
        }
        else{    // DSP
            RF_z_dz_ddz_w = RF_z_dz_ddz_old_w;
            LF_z_dz_ddz_w = LF_z_dz_ddz_old_w;
            RF_x_dx_ddx_w = RF_x_dx_ddx_old_w;
            LF_x_dx_ddx_w = LF_x_dx_ddx_old_w;
        }


        WD[i].RF_z_dz_ddz = RF_z_dz_ddz_w;
        WD[i].LF_z_dz_ddz = LF_z_dz_ddz_w;
        WD[i].RF_x_dx_ddx = RF_x_dx_ddx_w;
        WD[i].LF_x_dx_ddx = LF_x_dx_ddx_w;

        RF_z_dz_ddz_old_w = RF_z_dz_ddz_w;
        LF_z_dz_ddz_old_w = LF_z_dz_ddz_w;
        RF_x_dx_ddx_old_w = RF_x_dx_ddx_w;
        LF_x_dx_ddx_old_w = LF_x_dx_ddx_w;

        t_now_w += dt;
    }

    //// 3 mass ZMP ref modification
    double m_com = 32;
    double m_sup = 1.;
    double m_swg = 1.;
    double m_pend = m_com - m_sup - m_swg;
    double m_rf, m_lf;

    for(int i=0; i<(int)(dT*freq); i++){
        if(SDB[step_phase].swingFoot == RFoot){
            m_rf = m_swg; m_lf = m_sup;
        }
        else if(SDB[step_phase].swingFoot == LFoot){
            m_rf = m_sup; m_lf = m_swg;
        }
        else{
            m_rf = m_sup; m_lf = m_sup;
        }

        // Lateral motion
        double Mfeet_x;

        Mfeet_x = m_rf*(WD[i].RF_y_dy_ddy[0] - WD[i].ZMP_ref.y)*(g + WD[i].RF_z_dz_ddz[2])
            - m_rf*WD[i].RF_y_dy_ddy[2]*(WD[i].RF_z_dz_ddz[0] - WD[i].ZMP_ref.z)
            + m_lf*(WD[i].LF_y_dy_ddy[0] - WD[i].ZMP_ref.y)*(g + WD[i].LF_z_dz_ddz[2])
            - m_rf*WD[i].LF_y_dy_ddy[2]*(WD[i].LF_z_dz_ddz[0] - WD[i].ZMP_ref.z);

        // Delta1 = ZMP_feet - ZMP_com
        double DeltaY1 = Mfeet_x/(m_sup + m_swg)/g;
        // Delta2 = ZMP_pend - ZMP_com
        double DeltaY2 = (m_swg + m_sup)/m_com*DeltaY1;

        WD[i].ZMP_3m.y = WD[i].ZMP_ref.y + DeltaY2;

        // Sagital motion
        double Mfeet_y;

        Mfeet_y = m_rf*(WD[i].RF_x_dx_ddx[0] - WD[i].ZMP_ref.x)*(g + WD[i].RF_z_dz_ddz[2])
            - m_rf*WD[i].RF_x_dx_ddx[2]*(WD[i].RF_z_dz_ddz[0] - WD[i].ZMP_ref.z)
            + m_lf*(WD[i].LF_x_dx_ddx[0] - WD[i].ZMP_ref.x)*(g + WD[i].LF_z_dz_ddz[2])
            - m_rf*WD[i].LF_x_dx_ddx[2]*(WD[i].LF_z_dz_ddz[0] - WD[i].ZMP_ref.z);

        double DeltaX1 = Mfeet_y/(m_sup + m_swg)/g;
        double DeltaX2 = (m_sup + m_swg)/m_com*DeltaX1;

        WD[i].ZMP_3m.x = WD[i].ZMP_ref.x + DeltaX2;

    }
    last_tic = (int)(dT*freq);

    for(int j=1; j<= No_of_cycle; j++){
        for(int i=last_tic; i < (int)(last_tic + SDB[step_phase + j].t*freq); i++){
            if(SDB[step_phase+j].swingFoot == RFoot){
                m_rf = m_swg; m_lf = m_sup;
            }
            else if(SDB[step_phase+j].swingFoot == LFoot){
                m_rf = m_sup; m_lf = m_swg;
            }
            else{
                m_rf = m_sup; m_lf = m_sup;
            }

            // Sagital motion
            double Mfeet_x;
            Mfeet_x = m_rf*(WD[i].RF_y_dy_ddy[0] - WD[i].ZMP_ref.y)*(g + WD[i].RF_z_dz_ddz[2])
                - m_rf*WD[i].RF_y_dy_ddy[2]*(WD[i].RF_z_dz_ddz[0] - WD[i].ZMP_ref.z)
                + m_lf*(WD[i].LF_y_dy_ddy[0] - WD[i].ZMP_ref.y)*(g + WD[i].LF_z_dz_ddz[2])
                - m_rf*WD[i].LF_y_dy_ddy[2]*(WD[i].LF_z_dz_ddz[0] - WD[i].ZMP_ref.z);

            // Delta1 = ZMP_feet - ZMP_com
            double DeltaY1 = Mfeet_x/(m_sup + m_swg)/g;
            // Delta2 = ZMP_pend - ZMP_com
            double DeltaY2 = (m_swg + m_sup)/m_com*DeltaY1;

            //Yzmp_com = m_pend/m_com*Window(i).zmp_ref(2) + (m_sup + m_swg)/m_com*Yzmp_feet;
            WD[i].ZMP_3m.y = WD[i].ZMP_ref.y + DeltaY2;

            // Sagital motion
            double Mfeet_y;
            Mfeet_y = m_rf*(WD[i].RF_x_dx_ddx[0] - WD[i].ZMP_ref.x)*(g + WD[i].RF_z_dz_ddz[2])
                - m_rf*WD[i].RF_x_dx_ddx[2]*(WD[i].RF_z_dz_ddz[0] - WD[i].ZMP_ref.z)
                + m_lf*(WD[i].LF_x_dx_ddx[0] - WD[i].ZMP_ref.x)*(g + WD[i].LF_z_dz_ddz[2])
                - m_rf*WD[i].LF_x_dx_ddx[2]*(WD[i].LF_z_dz_ddz[0] - WD[i].ZMP_ref.z);

            double DeltaX1 = Mfeet_y/(m_sup + m_swg)/g;
            double DeltaX2 = (m_sup + m_swg)/m_com*DeltaX1;

           WD[i].ZMP_3m.x = WD[i].ZMP_ref.x + DeltaX2;
        }
        last_tic = (int)(last_tic + SDB[step_phase + j].t*freq);
    }

    for(int i=last_tic; i<(int)(t_prev*freq); i++){
        if(SDB[step_phase+No_of_cycle+1].swingFoot == RFoot){
            m_rf = m_swg; m_lf = m_sup;
        }
        else if(SDB[step_phase+No_of_cycle+1].swingFoot == LFoot){
            m_rf = m_sup; m_lf = m_swg;
        }
        else{
            m_rf = m_sup; m_lf = m_sup;
        }

        // Sagital motion
        double Mfeet_x;
        Mfeet_x = m_rf*(WD[i].RF_y_dy_ddy[0] - WD[i].ZMP_ref.y)*(g + WD[i].RF_z_dz_ddz[2])
            - m_rf*WD[i].RF_y_dy_ddy[2]*(WD[i].RF_z_dz_ddz[0] - WD[i].ZMP_ref.z)
            + m_lf*(WD[i].LF_y_dy_ddy[0] - WD[i].ZMP_ref.y)*(g + WD[i].LF_z_dz_ddz[2])
            - m_rf*WD[i].LF_y_dy_ddy[2]*(WD[i].LF_z_dz_ddz[0] - WD[i].ZMP_ref.z);

        // Delta1 = ZMP_feet - ZMP_com
        double DeltaY1 = Mfeet_x/(m_sup + m_swg)/g;
        // Delta2 = ZMP_pend - ZMP_com
        double DeltaY2 = (m_swg + m_sup)/m_com*DeltaY1;

        WD[i].ZMP_3m.y = WD[i].ZMP_ref.y + DeltaY2;

        // Sagital motion
        double Mfeet_y;
        Mfeet_y = m_rf*(WD[i].RF_x_dx_ddx[0] - WD[i].ZMP_ref.x)*(g + WD[i].RF_z_dz_ddz[2])
            - m_rf*WD[i].RF_x_dx_ddx[2]*(WD[i].RF_z_dz_ddz[0] - WD[i].ZMP_ref.z)
            + m_lf*(WD[i].LF_x_dx_ddx[0] - WD[i].ZMP_ref.x)*(g + WD[i].LF_z_dz_ddz[2])
            - m_rf*WD[i].LF_x_dx_ddx[2]*(WD[i].LF_z_dz_ddz[0] - WD[i].ZMP_ref.z);

        double DeltaX1 = Mfeet_y/(m_sup + m_swg)/g;
        double DeltaX2 = (m_sup + m_swg)/m_com*DeltaX1;

        WD[i].ZMP_3m.x = WD[i].ZMP_ref.x+ DeltaX2;
    }
}

void HB_PreviewWalk::Fill_ZMPref_get_CPref(int _dt_gain){
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
    double t_half_dsp = SDB[step_phase].t*dsp_ratio*0.5;
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

        for(int i=0 ; i<(int)((dT + dt/2)*freq) ; i++){
            double t1 = dsp_timer;
            double t2 = t1*t1;
            double t3 = t2*t1;
            double t4 = t3*t1;
            double t5 = t4*t1;

            double zmpY = Ay*t5 + By*t4 + Cy*t3 + Dy*t2 + Ey*t1 + Fy;
            double zmpX = Ax*t5 + Bx*t4 + Cx*t3 + Dx*t2 + Ex*t1 + Fx;


            WD[i].ZMP_ref.y = zmpY;
            WD[i].ZMP_ref.x = zmpX;

//            if((int)((dT + dt/2)*freq) == 1){
//                cout<<"half dsp: "<<t_half_dsp<<endl;
//                cout<<"dsp timer"<<dsp_timer<<endl;
//                cout<<"dT: "<<dT<<endl;
//                cout<<"(int)dT*freq: "<<(int)((dT + dt/2)*freq)<<endl;
//            }
//            if((int)((dT + dt/2)*freq) == 2) cout<<"dsp timer2"<<dsp_timer<<endl;

            dsp_timer += dt;
        }
        last_tic = (int)((dT + dt/2)*freq);
    }

    for(int j=1; j<= No_of_cycle; j++){
        t_half_dsp = SDB[step_phase + j].t*dsp_ratio*0.5;

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

    t_half_dsp = SDB[step_phase + No_of_cycle + 1].t*dsp_ratio*0.5;

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

    //// get DCMref of _dt_gain ahead
    COM_y_dy_ddy_old_pre = COM_y_dy_ddy_old;
    COM_x_dx_ddx_old_pre = COM_x_dx_ddx_old;
    Y_zmp_e_sum_pre = Y_zmp_e_sum;
    X_zmp_e_sum_pre = X_zmp_e_sum;
    vec3 p_out_pre = vec3();

    for(int j=0 ; j<_dt_gain ; j++){
        // calc COM ref
        double temp_x1 = 0;
        double temp_x2 = 0;
        double temp_y1 = 0;
        double temp_y2 = 0;

        for(int i=0; i<3; i++){
            temp_y1 += Gx[i]*COM_y_dy_ddy_old_pre[i];
            temp_x1 += Gx[i]*COM_x_dx_ddx_old_pre[i];
        }
        for(int i=j; i<NL; i++){
            temp_y2 += WD[i].ZMP_ref.y*Gp[i-j];
            temp_x2 += WD[i].ZMP_ref.x*Gp[i-j];
        }

        // Y direction
        double u_y = -Gi*Y_zmp_e_sum_pre - temp_y1 - temp_y2;

        COM_y_dy_ddy_pre = A*COM_y_dy_ddy_old_pre + u_y*B;
        p_out_pre.y = dot(C,COM_y_dy_ddy_old_pre);

        Y_zmp_e_sum_pre += p_out_pre.y - WD[j].ZMP_ref.y;

        // X direction
        double u_x = -Gi*X_zmp_e_sum_pre - temp_x1 - temp_x2;

        COM_x_dx_ddx_pre = A*COM_x_dx_ddx_old_pre + u_x*B;
        p_out_pre.x = dot(C,COM_x_dx_ddx_old_pre);

        X_zmp_e_sum_pre += p_out_pre.x - WD[j].ZMP_ref.x;

        COM_y_dy_ddy_old_pre = COM_y_dy_ddy_pre;
        COM_x_dx_ddx_old_pre = COM_x_dx_ddx_pre;
    }
    vec3 COM_ref_pre = vec3(COM_x_dx_ddx_old_pre[0], COM_y_dy_ddy_old_pre[0], zc);
    vec3 dCOM_ref_pre = vec3(COM_x_dx_ddx_old_pre[1], COM_y_dy_ddy_old_pre[1], 0);

    //cout<<"ZMP_refy: "<<p_ref[0].y<<"  COM y: "<<COM_ref.y<<endl;
    CP_ref = COM_ref_pre + dCOM_ref_pre/w;
}
*/

