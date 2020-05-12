#ifndef HB_JUMPING
#define HB_JUMPING

#include "BasicMatrix.h"
#include <unistd.h>
#include "../../share/Headers/RBSharedMemory.h"
#include <QVector>
#include "HB_functions.h"

class HB_JUMP
{
public:
    //General Variables
    const double dt = 0.005;
    double SAVE[80][200000];
    double t_now, Mass, g, Tcycle;
    int k, N_jump, jump_phase;
    vec3 COM_ref, COM_p, dCOM_p, ddCOM_p;
    quat qPel_ref;

    //Jumping variables
    double Ts, Tf, Tl, lamda, t_elapsed, t_total, Tff;
    double fz_ref, F0;
    vec3 last_COM_d_dd, ini_COM_d_dd;


    //Functions
    void save_onestep(int cnt);
    void save_all();

    HB_JUMP(){
        Ts = 0.2;
        Tf = 0.07;
        Tl = 0.2;
        lamda = 0.9;
        k = 0;
        t_now = 0;
        Mass = 17;
        g = 9.81;
        t_total = 0;
        jump_phase = 0;

    }

    void HB_set_jump(vec3 _COM_ini){
        Ts = 0.2;  // jumping time
        Tf = 0.07; // airial time
        Tl = 0.15;  // landing time
        Tcycle = Ts + Tf + Tl;
        k = 0;
        t_now = 0;
        t_total = 0;
        t_elapsed = 0;
        jump_phase = 0;

        N_jump = 1;
        Tff = 0.11; // landing time tuning parameter


        COM_p = COM_ref = _COM_ini;
        cout<<"initial COM: ("<<COM_p.x<<", "<<COM_p.y<<", "<<COM_p.z<<")"<<endl;
        dCOM_p = ddCOM_p = vec3();
        ini_COM_d_dd = vec3(_COM_ini.z, 0, 0);

        //set F0
        F0 = 3/(2+lamda)*(1+Tf/Ts)*Mass*g;
    }

    int Jump_once(){
        //force profile
        if(t_now < lamda*Ts) fz_ref = F0;
        else if(t_now < Ts) fz_ref = F0*pow((1-((t_now-lamda*Ts)/((1-lamda)*Ts))),2);
        else if(t_now < Ts + Tf) fz_ref = 0;
        else fz_ref = F0;



//        if(t_now < Ts){
//            ddCOM_p.z = fz_ref/Mass - g;
//        }
//        else if(t_now < Ts + (Tf)/2){
//            //dCOM_p.z = (0.55 - COM_p.z)/3;
//        }
//        else{
//            ddCOM_p.z = 100*(-dCOM_p.z);
//        }
        if(t_now < Ts){
            ddCOM_p.z = fz_ref/Mass - g;

            dCOM_p.z = dCOM_p.z + ddCOM_p.z*dt;
            COM_p.z = COM_p.z + dCOM_p.z*dt;

            last_COM_d_dd = vec3(COM_p.z, dCOM_p.z, ddCOM_p.z);

            double percent = t_now/Ts;

            quat qPel_temp = quat(vec3(0,1,0), (20 - 20*percent)*D2R);

            qPel_ref = qPel_temp;

            cout<<"go up COM_p z : "<<COM_p.z<<endl;
            cout<<"up percent: "<<percent<<endl;
        }
        else if(t_now < Ts + Tf + Tff ){
            vec3 z_dz_ddz = Fifth_trajectory(t_now, Ts+Tf+Tff, vec3(COM_p.z, dCOM_p.z, ddCOM_p.z), vec3(last_COM_d_dd[0],-last_COM_d_dd[1], -last_COM_d_dd[2]));

            COM_p.z = z_dz_ddz[0];
            dCOM_p.z = z_dz_ddz[1];
            ddCOM_p.z = z_dz_ddz[2];


            cout<<"recover COM_p z : "<<COM_p.z<<endl;
        }
        else if(t_now < Ts + Tf + Tl + Tff){
            Tl = ((ini_COM_d_dd[0]-0.013) - last_COM_d_dd[0])/(-0.5*last_COM_d_dd[1] + 0);
            Tcycle = Ts + Tf + Tl + Tff;

            ddCOM_p.z = -(-last_COM_d_dd[1])/Tl; //2*(ini_COM_d_dd[0] - last_COM_d_dd[0] - last_COM_d_dd[1]*Tl)/(Tl*Tl);

            dCOM_p.z = dCOM_p.z + ddCOM_p.z*dt;
            COM_p.z = COM_p.z + dCOM_p.z*dt;

            double percent = (t_now - Tf - Tff - Ts)/Tl;

            quat qPel_temp = quat(vec3(0,1,0), (10*percent)*D2R);

            qPel_ref = qPel_temp;

            cout<<"percent: "<<percent<<endl;


            cout<<"landing COM_p z : "<<COM_p.z<<"   Tl : "<<Tl<<endl;
        }


        COM_ref.z = COM_p.z;// + 0.01*dCOM_p.z + 0.001*ddCOM_p.z;



        save_onestep(k);
        k++;
        t_now += dt;
        t_total += dt;

//        if(t_now > Ts + Tf){
//            return -1;
//        }
//        else
//            return 0;



        if(t_total - t_elapsed >= Tcycle){
            jump_phase += 1;
            t_elapsed += Tcycle;
            t_now = 0 ;

            cout<<"jump phase: "<<jump_phase<<endl;
            if(jump_phase >= N_jump) return -1;
            else return 0;
        }

    }


};

void HB_JUMP::save_onestep(int cnt)
{
    if(k<200000)
    {
        SAVE[0][cnt] = fz_ref;
        SAVE[1][cnt] = COM_p.z;
        SAVE[2][cnt] = dCOM_p.z;
        SAVE[3][cnt] = ddCOM_p.z;
        SAVE[4][cnt] = COM_ref.z;
//        SAVE[5][cnt] = COM.y;
//        SAVE[6][cnt] = CP.y;
//        SAVE[7][cnt] = CP_cps.y;
//        SAVE[8][cnt] = ZMP_ref_filtered_inSP.y;
//        SAVE[9][cnt] = CP_m_filtered.y;

//        SAVE[10][cnt] = COM_cps.x;
//        SAVE[11][cnt] = COM_cps.y;
//        SAVE[12][cnt] = ZMP_cps.x;
//        SAVE[13][cnt] = ZMP_cps.y;
//        SAVE[14][cnt] = ZMP_global.x;
//        SAVE[15][cnt] = ZMP_global.y;
//        SAVE[16][cnt] = CP_eos.x;
//        SAVE[17][cnt] = CP_eos.y;

//        SAVE[18][cnt] = IMUangle.x;
//        SAVE[19][cnt] = IMUangle.y;
//        SAVE[20][cnt] = IMUangle.z;

//        SAVE[21][cnt] = COM_cpt.x;
//        SAVE[22][cnt] = COM_cpt.y;
//        SAVE[23][cnt] = ZMP_cpt.x;
//        SAVE[24][cnt] = ZMP_cpt.y;
//        SAVE[25][cnt] = CP_cpt.x;
//        SAVE[26][cnt] = CP_cpt.y;

//        SAVE[27][cnt] = dT;//dT_est;//dT_buf[0];
//        SAVE[28][cnt] = R_ank_torque.x;
//        SAVE[29][cnt] = L_ank_torque.x;

//        SAVE[30][cnt] = pRF.z;
//        SAVE[31][cnt] = pLF.z;
//        SAVE[32][cnt] = z_ctrl;

//        SAVE[33][cnt] = COM_m.x; // COM_measure x
//        SAVE[34][cnt] = COM_m.y; // COM_measure y

//        SAVE[35][cnt] = R_ank_torque.y;
//        SAVE[36][cnt] = L_ank_torque.y;

//        SAVE[37][cnt] = R_HIP_P_torque; // temp x
//        SAVE[38][cnt] = R_HIP_R_torque; // temp y

//        SAVE[39][cnt] = R_KNEE_torque;//COM_est.x + dCOM_est.x/w; // CP estimation x
//        SAVE[40][cnt] = swingFoot;// + dCOM_est.y/w; // CP estimation y

//        SAVE[41][cnt] = COM_est.x; // COM x estimation
//        SAVE[42][cnt] = COM_est.y; // COM y estimation
//        SAVE[43][cnt] = dCOM_est.x; // dCOM x estimation
//        SAVE[44][cnt] = dCOM_est.y; // dCOM y estimation

//        SAVE[45][cnt] = CP_m.y; // ZMP x estimation
//        SAVE[46][cnt] = ZMP_global_filtered.x;
//        SAVE[47][cnt] = ZMP_global_filtered.y;
//        SAVE[48][cnt] = pRF.x;
//        SAVE[49][cnt] = pRF.y;

//        SAVE[50][cnt] = F_RF_filtered.z;
//        SAVE[51][cnt] = F_LF_filtered.z;
//        SAVE[52][cnt] = pLF.x;
//        SAVE[53][cnt] = dCOM.y;
//        SAVE[54][cnt] = pLF.y;

//        SAVE[55][cnt] = COM_old.x;
//        SAVE[56][cnt] = COM_old.y;
//        SAVE[57][cnt] = ZMP_est.y;

//        SAVE[58][cnt] = CP_eos_gap_RF.x;
//        SAVE[59][cnt] = COM_choreonoid.y;
//        SAVE[60][cnt] = CP_eos_gap_LF.x;
//        SAVE[61][cnt] = dCOM_choreonoid.y;

//        SAVE[62][cnt] = ZMP_ref_gap.y;
//        SAVE[63][cnt] = CP_eos_gap_RF.y;
//        SAVE[64][cnt] = pLF_landing.y;

//        SAVE[65][cnt] = CP_eos_gap_LF.y;
//        SAVE[66][cnt] = ZMP_ref_gap.x;
//        SAVE[67][cnt] = (double)LF_landing_flag;



        //SAVE[47][cnt] = realCOM.z;//
    }
}

void HB_JUMP::save_all()
{
    printf("Jump finished %d\n",k);
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


#endif // HB_JUMPING

