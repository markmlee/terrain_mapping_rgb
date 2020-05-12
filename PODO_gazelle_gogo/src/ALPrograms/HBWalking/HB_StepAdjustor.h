#ifndef HB_STEPADJUSTOR
#define HB_STEPADJUSTOR

#include "QuadProg++.hh"
#include "BasicMatrix.h"
#include <Eigen/Dense>
//#include <Eigen/Eigen>


#define     RFoot              -1
#define     DSP                 0
#define     SSP                 2
#define     LFoot               1

#define     D2R             	1.745329251994330e-2
#define     R2D             	5.729577951308232e1


using Eigen::MatrixXd;
using Eigen::VectorXd;


class HB_StepAdjustor
{
private:
    int numCols; // length of X
    int numEq;
    int numIneq;

    MatrixXd A_eq, B_eq;
    MatrixXd A_ineq, B_ineq;

    MatrixXd H,F, H2, F2;
    VectorXd X;

    const double dt = 0.002;


public:
    std::vector<MatrixXd> As, Bs;
    std::vector<double> Ws;
    std::vector<MatrixXd> As2, Bs2;
    std::vector<double> Ws2;
    double X_array[10];

public:
    void setNums(int _xlength, int _numEqConstraints, int _numIneqConstraints){
        numCols = _xlength;
        numEq = _numEqConstraints;
        numIneq = _numIneqConstraints;

        X = VectorXd::Zero(numCols);
        A_eq = MatrixXd::Zero(numEq,numCols);
        B_eq = MatrixXd::Zero(numEq,1);

        A_ineq = MatrixXd::Zero(numIneq, numCols);
        B_ineq = MatrixXd::Zero(numIneq, 1);

        H = MatrixXd::Zero(numCols, numCols);
        F = MatrixXd::Zero(numCols, 1);

        H2 = MatrixXd::Zero(numCols, numCols);
        F2 = MatrixXd::Zero(numCols, 1);

        Ws.clear();
        As.clear();

        Ws2.clear();
        As2.clear();

        for(int i=0; i<numCols ; i++){
            X_array[i] = 0;
        }
    }

    void MakeEq(vec3 _CP_error_local, vec3 _cZMP_local, double _t_now, double _w, vec3 _CP_ref_local, double _T_original){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot

        // A_eq * X = B_eq

        // X = [del_ux, del_uy, del_bx, del_by, tau]'

        //
        // A_eq = [1 0 1 0 -(CPerror_x + CPref_x - cZMPx)exp(-wt)]
        //        [0 1 0 1 -(CPerror_y + CPref_y - cZMPy)exp(-wt)]

        // B_eq = [cZMPx]
        //        [cZMPy]


        A_eq(0,0) = 1;
        A_eq(0,2) = 1;
        A_eq(0,4) = -(_CP_error_local.x + _CP_ref_local.x - _cZMP_local.x)*exp(-_w*_t_now);

        A_eq(1,1) = 1;
        A_eq(1,3) = 1;
        A_eq(1,4) = -(_CP_error_local.y + _CP_ref_local.y - _cZMP_local.y)*exp(-_w*_t_now);

        B_eq(0,0) = _cZMP_local.x - _CP_ref_local.x*exp(_w*(_T_original - _t_now));
        B_eq(1,0) = _cZMP_local.y - _CP_ref_local.y*exp(_w*(_T_original - _t_now));
    }

    void MakeIneq(int _swingFoot, vec3 _pSW, vec3 _pSW_cur, vec3 _del_u_f, double _t_now, double _T, double _W, double _T_original, double _T_last){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot
        // _pSW : original destination of swing Foot
        // _pSW_cur : stance foot to current swing foot in stance foot frame

        // A_ineq * X <= B_ineq


        double Lmax = 0.40;
        double Wmin = 0.18;
        double Wmax = 0.45;

        // velocity max of swing foot (absolute value)
        double Vmax_x = 0.7;
        double Vmax_y = 0.5;

        double Tmin = 0.45;




        //// del_u ineq constraints
        if(_swingFoot == RFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1.0;
            B_ineq(0,0) = Lmax - _pSW.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1.0;
            B_ineq(1,0) = Lmax + _pSW.x;

            //-del_uy < Wmax + pSW.y
            A_ineq(2,1) = -1.0;
            B_ineq(2,0) = Wmax + _pSW.y;

            //del_uy < -Wmin - pSW.y
            A_ineq(3,1) = 1.0;
            B_ineq(3,0) = -Wmin - _pSW.y;
        }

        if(_swingFoot == LFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1.0;
            B_ineq(0,0) = Lmax - _pSW.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1.0;
            B_ineq(1,0) = Lmax + _pSW.x;

            //del_uy < Wmax - pSW.y
            A_ineq(2,1) = 1.0;
            B_ineq(2,0) = Wmax - _pSW.y;

            //-del_uy < -Wmin + pSW.y
            A_ineq(3,1) = -1.0;
            B_ineq(3,0) = -Wmin + _pSW.y;
        }


        //---------------------Velocity Constraints -------------------------------
        // del_ux < pSW_cur.x - pSW.x + Vmax_x*(_T - _t_now)
//        A_ineq(4,0) = 1.0;
//        B_ineq(4,0) = _pSW_cur.x - _pSW.x + Vmax_x*(_T - _t_now);

//        // -del_ux < -(pSW_cur.x - pSW.x - Vmax_x*(_T - _t_now)
//        A_ineq(5,0) = -1.0;
//        B_ineq(5,0) = -(_pSW_cur.x - _pSW.x - Vmax_x*(_T - _t_now));

//        // del_uy < pSW_cur.x - pSW.x + Vmax_y*(_T - _t_now)
//        A_ineq(6,1) = 1.0;
//        B_ineq(6,0) = _pSW_cur.y - _pSW.y + Vmax_y*(_T - _t_now);

//        // -del_uy < -(pSW_cur.y - pSW.y - Vmax_y*(_T - _t_now)
//        A_ineq(7,1) = -1.0;
//        B_ineq(7,0) = -(_pSW_cur.y - _pSW.y - Vmax_y*(_T - _t_now));




        //// T constraints
        // max(Tmin, t_now, T-del_T) = T_ineq
//        double T_ineq;
//        if(Tmin > _t_now + 0.1){
//            T_ineq = Tmin;
//        }
//        else{
//            T_ineq = _t_now + 0.1;
//        }
//        if(T_ineq < _T - del_T){
//            T_ineq = _T - del_T;
//        }
//        else{}


        double T_min1 = _t_now + 0.08;
        double T_min2 = Tmin;



        double T_ineq = T_min1;
        if(T_ineq < T_min2) T_ineq = T_min2;    //T_ineq = max(T_min1, T_min2)

        // -tau < -tau_ineq
        A_ineq(4,4) = -1;
        B_ineq(4,0) = -exp(_W*T_ineq);

        // tau < tau_max
        double Tmax = _T_original;
//        if(_T_last < Tmax){
//            Tmax = _T_last;   // if step time reduced once, step time can not be greater than before
//        }

        A_ineq(5,4) = 1;
        B_ineq(5,0) = exp(_W*Tmax);

//        // -----------------Swing foot velocity constraits for T ---------------------
//        // T constraint is only depends on max velocity
//        // -tau < -(t_now - (pSW.x + del_u_f.x - pSW_cur.x)/Vmax_x   --> from x velocity constraints
//        A_ineq(10,4) = -1;
//        B_ineq(10,0) = -exp(_W*(_t_now - (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x));
//        // -tau < -(t_now + (pSW.x + del_u_f.x - pSW_cur.x)/Vmax_x
//        A_ineq(11,4) = -1;
//        B_ineq(11,0) = -exp(_W*(_t_now + (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x));

//        // -tau < -(t_now - (pSW.y + del_u_f.y - pSW_cur.y)/Vmax_y   --> from y velocity constraints
//        A_ineq(12,4) = -1;
//        B_ineq(12,0) = -exp(_W*(_t_now - (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y));
//        // -tau < -(t_now + (pSW.y + del_u_f.y - pSW_cur.y)/Vmax_y
//        A_ineq(13,4) = -1;
//        B_ineq(13,0) = -exp(_W*(_t_now + (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y));

        // 14 inequality constraints

    }

    void SetBehavior(double _w, double _T_nom, double _T_last, vec3 _del_u_f_last){
        // QP and Quadprog++ relation
        //
        // min 0.5*X'GX + g0*X
        //
        // set behavior as AX - B ~ 0
        //
        // (AX-B)'(AX-B) = X'A'AX - B'AX - X'A'B + B'B
        //
        // B'B is constant, so delete for minimize
        //
        // B'AX and X'A'B is scalar, so these are same value
        //
        // 0.5*X'A'AX + (-B'A)X

        // G = A'A  g0 = -B'A or -A'B


        MatrixXd A0, B0;
        double W0;


        //// step time weighting
        double tau_nom = exp(_w*_T_nom);

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,4) = 1;
        B0(0,0) = tau_nom;  // tau ~ tau_nom

        W0 = 0.01;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// step position
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,0) = 1;
        A0(1,1) = 1;
        B0(0,0) = 0;  // del u ~ 0
        B0(1,0) = 0;

        W0 = 0.05;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// DCM offset
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,2) = 1;
        A0(1,3) = 1;
        B0(0,0) = 0;  // del b ~ 0
        B0(1,0) = 0;

        W0 =3.2;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        //// Step time Filtering
        double tau_last = exp(_w*_T_last);

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,4) = 1;
        B0(0,0) = tau_last;  // tau ~ tau_last

        W0 = 0.1;//0.007;//0.011;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        //// Step position Filtering
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,0) = 1;
        A0(1,1) = 1;
        B0(0,0) = _del_u_f_last.x;  // del u ~ 0
        B0(1,0) = _del_u_f_last.y;

        W0 = 1.0;//0.05;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

//        //// Step position should not change(oscillate) much
//        A0 = MatrixXd::Zero(numCols, numCols);
//        B0 = MatrixXd::Zero(numCols,1);

//        A0(0,0) = 1;
//        A0(0,1) = 1;
//        B0(0,0) = _last_del_u.x;
//        B0(1,0) = _last_del_u.y;

//        W0 = 300;

//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

//        //// Step time should not change(oscillate) much
//        A0 = MatrixXd::Zero(numCols, numCols);
//        B0 = MatrixXd::Zero(numCols,1);

//        A0(0,4) = 1;
//        B0(0,0) = exp(_w*_last_T);


//        W0 = 0.1;

//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);


    }

    ////-----------------------------------------------------------------------------------------------------------------------
    /// ------------------------------------------ Pelvis Angular Acc add -------------------------------------------------------
    /// -----------------------------------------------------------------------------------------------------------------------

    void MakeEq2(vec3 _CP_error_local, vec3 _cZMP_local, double _t_now, double _w, vec3 _CP_ref_local, double _T_original, double _last_T){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot

        // A_eq * X = B_eq

        // X = [del_ux, del_uy, del_bx, del_by, tau, acc_pelv_pitch, acc_pelv_roll]'

        //
        // A_eq = [1 0 1 0 -(CPerror_x  + CPref_x - cZMPx)exp(-wt) -(1 - exp(w)(T_new-t_now))*Iy/mg  0]
        //        [0 1 0 1 -(CPerror_y  + CPref_y - cZMPy)exp(-wt)  0 -(1 - exp(w)(T_new-t_now))*Ix/mg]

        // B_eq = [cZMPx - CP_ref_x*exp(w)(T-t_now)]
        //        [cZMPy - CP_ref_y*exp(w)(T-t_now)]

        double Iy = 1.25;
        double Ix = 1.25;
        double mass = 43;
        double g = 9.81;


        A_eq(0,0) = 1;
        A_eq(0,2) = 1;
        A_eq(0,4) = -(_CP_error_local.x + _CP_ref_local.x - _cZMP_local.x)*exp(-_w*_t_now);
        A_eq(0,5) = -(1 - exp(_w*(_last_T - _t_now)))*Iy/mass/g;

        A_eq(1,1) = 1;
        A_eq(1,3) = 1;
        A_eq(1,4) = -(_CP_error_local.y + _CP_ref_local.y - _cZMP_local.y)*exp(-_w*_t_now);
        A_eq(1,6) = (1 - exp(_w*(_last_T - _t_now)))*Ix/mass/g;

        B_eq(0,0) = _cZMP_local.x - _CP_ref_local.x*exp(_w*(_T_original - _t_now));
        B_eq(1,0) = _cZMP_local.y - _CP_ref_local.y*exp(_w*(_T_original - _t_now));
    }

    void MakeIneq2(int _swingFoot, vec3 _pSW, vec3 _pSW_cur, vec3 _del_u_f, double _t_now, double _T, double _W, double _T_original,
                   double _q_pitch_rad, double _dq_pitch_rad, double _q_roll_rad, double _dq_roll_rad){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot
        // _pSW : position destination of swing Foot

        // A_ineq * X <= B_ineq


        double Lmax = 0.37;
        double Wmin = 0.13;
        double Wmax = 0.45;

        // velocity max of swing foot (absolute value)
        double Vmax_x = 0.7;
        double Vmax_y = 0.6;

        double Tmin = 0.5;



        //// del_u ineq constraints
        if(_swingFoot == RFoot){


            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1.0;
            B_ineq(0,0) = Lmax - _pSW.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1.0;
            B_ineq(1,0) = Lmax + _pSW.x;

            //-del_uy < Wmax + pSW.y
            A_ineq(2,1) = -1.0;
            B_ineq(2,0) = Wmax + _pSW.y;

            //del_uy < -Wmin - pSW.y
            A_ineq(3,1) = 1.0;
            B_ineq(3,0) = -Wmin - _pSW.y;
        }

        if(_swingFoot == LFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1.0;
            B_ineq(0,0) = Lmax - _pSW.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1.0;
            B_ineq(1,0) = Lmax + _pSW.x;

            //del_uy < Wmax - pSW.y
            A_ineq(2,1) = 1.0;
            B_ineq(2,0) = Wmax - _pSW.y;

            //-del_uy < -Wmin + pSW.y
            A_ineq(3,1) = -1.0;
            B_ineq(3,0) = -Wmin + _pSW.y;
        }


        //---------------------Velocity Constraints -------------------------------
        // del_ux < pSW_cur.x - pSW.x + Vmax_x*(_T - _t_now)
//        A_ineq(4,0) = 1;
//        B_ineq(4,0) = _pSW_cur.x - _pSW.x + Vmax_x*(_T - _t_now);

//        // -del_ux < -(pSW_cur.x - pSW.x - Vmax_x*(_T - _t_now)
//        A_ineq(5,0) = -1;
//        B_ineq(5,0) = -(_pSW_cur.x - _pSW.x - Vmax_x*(_T - _t_now));

//        // del_uy < pSW_cur.x - pSW.x + Vmax_y*(_T - _t_now)
//        A_ineq(6,1) = 1;
//        B_ineq(6,0) = _pSW_cur.y - _pSW.y + Vmax_y*(_T - _t_now);

//        // -del_uy < -(pSW_cur.y - pSW.y - Vmax_y*(_T - _t_now)
//        A_ineq(7,1) = -1;
//        B_ineq(7,0) = -(_pSW_cur.y - _pSW.y - Vmax_y*(_T - _t_now));




        //// T constraints
        // max(Tmin, t_now, T-del_T) = T_ineq
//        double T_ineq;
//        if(Tmin > _t_now + 0.1){
//            T_ineq = Tmin;
//        }
//        else{
//            T_ineq = _t_now + 0.1;
//        }
//        if(T_ineq < _T - del_T){
//            T_ineq = _T - del_T;
//        }
//        else{}


        double T_min1 = _t_now + 0.08;
        double T_min2 = Tmin;

//        double T_min3 = _t_now - (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x;
//        double T_min4 = _t_now + (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x;
//        double T_min5 = _t_now - (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y;
//        double T_min6 = _t_now - (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y;


        double T_ineq = T_min1;
        if(T_ineq < T_min2) T_ineq = T_min2;    //T_ineq = max(T_min1, T_min2)
//        if(T_ineq < T_min3) T_ineq = T_min3;
//        if(T_ineq < T_min4) T_ineq = T_min4;
//        if(T_ineq < T_min5) T_ineq = T_min5;
//        if(T_ineq < T_min6) T_ineq = T_min6;

        // -tau < -tau_ineq
        A_ineq(4,4) = -1;
        B_ineq(4,0) = -exp(_W*T_ineq);

        // tau < tau_max
        double Tmax = _T_original+0.2;
//        if(_T_last < Tmax){
//            Tmax = _T_last;   // if step time reduced once, step time can not be greater than before
//        }

        A_ineq(5,4) = 1;
        B_ineq(5,0) = exp(_W*Tmax);


        //--------------------Angular acc constraints -------------------------------

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
        double Acc_Max2_pitch_rad = (Vel_Max_pitch_rad - _dq_pitch_rad)/dt;
        double Acc_Max2_roll_rad = (Vel_Max_roll_rad - _dq_roll_rad)/dt;

        // Acc Max3 : limit form Max angle
        double temp1_pitch = Max_pitch_rad - _q_pitch_rad;
        double temp1_roll = Max_roll_rad - _q_roll_rad;


        if(temp1_pitch <= 0) temp1_pitch = 0;
        if(temp1_roll <= 0) temp1_roll = 0;

        double Acc_Max3_pitch_rad = (sqrt(2*Acc_lim_pitch_rad*temp1_pitch) - _dq_pitch_rad)/dt;
        double Acc_Max3_roll_rad = (sqrt(2*Acc_lim_roll_rad*temp1_roll) - _dq_roll_rad)/dt;


        // Acc min1 : hardware limit
        double Acc_min1_pitch_rad = -Acc_lim_pitch_rad;
        double Acc_min1_roll_rad = -Acc_lim_roll_rad;

        // Acc min2 : limit from Max vel
        double Acc_min2_pitch_rad = (-Vel_Max_pitch_rad - _dq_pitch_rad)/dt;
        double Acc_min2_roll_rad = (-Vel_Max_roll_rad - _dq_roll_rad)/dt;

        // Acc min3 : limit from Max angle
        double temp2_pitch = _q_pitch_rad - min_pitch_rad;
        double temp2_roll = _q_roll_rad - min_roll_rad;

        if(temp2_pitch <= 0) temp2_pitch = 0;
        if(temp2_roll <= 0) temp2_roll = 0;

        double Acc_min3_pitch_rad = (-sqrt(2*Acc_lim_pitch_rad*temp2_pitch) - _dq_pitch_rad)/dt;
        double Acc_min3_roll_rad = (-sqrt(2*Acc_lim_roll_rad*temp2_roll) - _dq_roll_rad)/dt;


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

        // safe condition
        if(Acc_Max_pitch <= Acc_min_pitch){
            Acc_min_pitch = Acc_Max_pitch - 0.01;
        }
        if(Acc_Max_roll <= Acc_min_roll){
            Acc_min_roll = Acc_Max_roll - 0.01;
        }

        // Acc_pitch < Acc_max_pitch
        A_ineq(6,5) = 1;
        B_ineq(6,0) = Acc_Max_pitch; //rad

        // -Acc_pitch < -Acc_min_pitch
        A_ineq(7,5) = -1;
        B_ineq(7,0) = -Acc_min_pitch; //rad

        // Acc_roll < Acc_max_roll
        A_ineq(8,6) = 1;
        B_ineq(8,0) = Acc_Max_roll; //rad

        // -Acc_roll < -Acc_min_roll
        A_ineq(9,6) = -1;
        B_ineq(9,0) = -Acc_min_roll; //rad



//        // -----------------Swing foot velocity constraits for T ---------------------
//        // T constraint is only depends on max velocity

//        // -tau < -(t_now - (pSW.x + del_u_f.x - pSW_cur.x)/Vmax_x   --> from x velocity constraints
//        A_ineq(14,4) = -1;
//        B_ineq(14,0) = -exp(_W*(_t_now - (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x));
//        // -tau < -(t_now + (pSW.x + del_u_f.x - pSW_cur.x)/Vmax_x
//        A_ineq(15,4) = -1;
//        B_ineq(15,0) = -exp(_W*(_t_now + (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x));

//        // -tau < -(t_now - (pSW.y + del_u_f.y - pSW_cur.y)/Vmax_y   --> from y velocity constraints
//        A_ineq(16,4) = -1;
//        B_ineq(16,0) = -exp(_W*(_t_now - (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y));
//        // -tau < -(t_now + (pSW.y + del_u_f.y - pSW_cur.y)/Vmax_y
//        A_ineq(17,4) = -1;
//        B_ineq(17,0) = -exp(_W*(_t_now + (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y));

        // 14 inequality constraints

    }

    void SetBehavior2(double _w, double _T_nom, double _T_last, vec3 _del_u_f_last, double _t_now,
                      double _q_pitch_rad, double _dq_pitch_rad, double _q_roll_rad, double _dq_roll_rad){
        // QP and Quadprog++ relation
        //
        // min 0.5*X'GX + g0*X
        //
        // set behavior as AX - B ~ 0
        //
        // (AX-B)'(AX-B) = X'A'AX - B'AX - X'A'B + B'B
        //
        // B'B is constant, so delete for minimize
        //
        // B'AX and X'A'B is scalar, so these are same value
        //
        // 0.5*X'A'AX + (-B'A)X

        // G = A'A  g0 = -B'A or -A'B


        MatrixXd A0, B0;
        double W0;


        //// step time weighting
        double tau_nom = exp(_w*_T_nom);

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,4) = 1;
        B0(0,0) = tau_nom;  // tau ~ tau_nom

        W0 = 0.01;//0.5;//0.015;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// step position
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,0) = 1;
        A0(1,1) = 1;
        B0(0,0) = 0;  // del u ~ 0
        B0(1,0) = 0;

        W0 = 0.05;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// DCM offset
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,2) = 1;
        A0(1,3) = 1;
        B0(0,0) = 0;  // del b ~ 0
        B0(1,0) = 0;

        W0 =3.2;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        //// Step time Filtering
        double tau_last = exp(_w*_T_last);

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,4) = 1;
        B0(0,0) = tau_last;  // tau ~ tau_last

        W0 = 0.06;//0.007;//0.011;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        //// Step position Filtering
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,0) = 1;
        A0(1,1) = 1;
        B0(0,0) = _del_u_f_last.x;  // del u ~ 0
        B0(1,0) = _del_u_f_last.y;

        W0 = 1.0;//0.05;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        //// pelvis return to upright : pitch
        double kp = 100;//150;//80;//20;
        double kd = 100;//140;//80;//200;
        double ddq_pitch_ref = -kd*_dq_pitch_rad - kp*_q_pitch_rad;

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,5) = 1;
        B0(0,0) = ddq_pitch_ref;  // del u ~ 0

        W0 = 0.01;//*(1.0 + _t_now/0.1); 0.0005

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// pelvis return to upright : roll

        kp = 80;//20;
        kd = 60;//200;
        double ddq_roll_ref = -kd*_dq_roll_rad - kp*_q_roll_rad;

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,6) = 1;
        B0(0,0) = ddq_roll_ref;  // del u ~ 0

        W0 = 0.01;//*(1.0 + _t_now/0.1);

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


//        //// Step position should not change(oscillate) much
//        A0 = MatrixXd::Zero(numCols, numCols);
//        B0 = MatrixXd::Zero(numCols,1);

//        A0(0,0) = 1;
//        A0(0,1) = 1;
//        B0(0,0) = _last_del_u.x;
//        B0(1,0) = _last_del_u.y;

//        W0 = 300;

//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

//        //// Step time should not change(oscillate) much
//        A0 = MatrixXd::Zero(numCols, numCols);
//        B0 = MatrixXd::Zero(numCols,1);

//        A0(0,4) = 1;
//        B0(0,0) = exp(_w*_last_T);


//        W0 = 0.1;

//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);


    }




    ////---------------------------------------------------------------------------------------------------------------------------------------
    ////-----------------------------------------------Pelvis Angular Velocity add------------------------------------------------------------
    ////---------------------------------------------------------------------------------------------------------------------------------------


    void MakeEq_Angular(vec3 _CP_error_local, vec3 _cZMP_local, double _t_now, double _w, vec3 _CP_ref_local, double _T_original, double _T_new){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot

        // A_eq * X = B_eq

        // X = [del_ux, del_uy, del_bx, del_by, tau, Omega_pitch, Omega_roll]'

        //
        // A_eq = [1 0 1 0 -(CPerror_x - cZMPx)exp(-wt)]
        //        [0 1 0 1 -(CPerror_y - cZMPy)exp(-wt)]

        // B_eq = [cZMPx - CP_ref_local.x*exp(_w*(_T_original - _t_now)]
        //        [cZMPy - CP_ref_local.y*exp(_w*(_T_original - _t_now)]

        double Jx = 0.2;
        double Jy = 0.2;
        double mass = 32;
        double zc = 0.62;


        A_eq(0,0) = 0.8;  //del_ux
        A_eq(0,2) = 1;  //del_bx
        A_eq(0,4) = -(_CP_error_local.x + _CP_ref_local.x - _cZMP_local.x)*exp(-_w*_t_now); //tau
        A_eq(0,5) = Jy/(mass*zc)*exp(_w*(_T_new - _t_now)); //Omega_pitch

        A_eq(1,1) = 0.8;  //del_uy
        A_eq(1,3) = 1;  //del_by
        A_eq(1,4) = -(_CP_error_local.y + _CP_ref_local.y - _cZMP_local.y)*exp(-_w*_t_now); //tau
        A_eq(1,6) = -Jx/(mass*zc)*exp(_w*(_T_new - _t_now)); //Omega_roll

        B_eq(0,0) = _cZMP_local.x - _CP_ref_local.x*exp(_w*(_T_original - _t_now));
        B_eq(1,0) = _cZMP_local.y - _CP_ref_local.y*exp(_w*(_T_original - _t_now));
    }

    void MakeIneq_Angular(int _swingFoot, vec3 _pSW, vec3 _pSW_cur, vec3 _del_u_f, double _t_now, double _T, double _W){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot
        // _pSW : position destination of swing Foot in stance foot frame
        // _pSW_cur : position of swing Foot now in stance foot frame (in the air)
        // _del_u_f : del_u_f optimization value (previous step)
        // _T : t_step optimization value (previous step)



        // A_ineq * X < B_ineq

        double Lmax = 0.4;
        double Wmax = 0.6;
        double Wmin = 0.19;

        // velocity max of swing foot (absolute value)
        double Vmax_x = 0.7;
        double Vmax_y = 0.5;

        double Omega_max = 3.14/2; // ~ 45 deg/s

        double Tmin = 0.45;


        //// del_u ineq constraints

        //---------------------Kinematic Constraints -------------------------------
        if(_swingFoot == RFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1;
            B_ineq(0,0) = Lmax - _pSW.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1;
            B_ineq(1,0) = Lmax + _pSW.x;

            //-del_uy < Wmax + pSW.y
            A_ineq(2,1) = -1;
            B_ineq(2,0) = Wmax + _pSW.y;

            //del_uy < -Wmin - pSW.y
            A_ineq(3,1) = 1;
            B_ineq(3,0) = -Wmin - _pSW.y;  

        }

        if(_swingFoot == LFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1;
            B_ineq(0,0) = Lmax - _pSW.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1;
            B_ineq(1,0) = Lmax + _pSW.x;

            //del_uy < Wmax - pSW.y
            A_ineq(2,1) = 1;
            B_ineq(2,0) = Wmax - _pSW.y;

            //-del_uy < -Wmin + pSW.y
            A_ineq(3,1) = -1;
            B_ineq(3,0) = -Wmin + _pSW.y;

        }

        //---------------------Velocity Constraints -------------------------------
        // del_ux < pSW_cur.x - pSW.x + Vmax_x*(_T - _t_now)
        A_ineq(4,0) = 1;
        B_ineq(4,0) = _pSW_cur.x - _pSW.x + Vmax_x*(_T - _t_now);

        // -del_ux < -(pSW_cur.x - pSW.x - Vmax_x*(_T - _t_now)
        A_ineq(5,0) = -1;
        B_ineq(5,0) = -(_pSW_cur.x - _pSW.x - Vmax_x*(_T - _t_now));

        // del_uy < pSW_cur.x - pSW.x + Vmax_y*(_T - _t_now)
        A_ineq(6,1) = 1;
        B_ineq(6,0) = _pSW_cur.y - _pSW.y + Vmax_y*(_T - _t_now);

        // -del_uy < -(pSW_cur.y - pSW.y - Vmax_y*(_T - _t_now)
        A_ineq(7,1) = -1;
        B_ineq(7,0) = -(_pSW_cur.y - _pSW.y - Vmax_y*(_T - _t_now));



        //// T constraints
        // max(Tmin, t_now, T-del_T) = T_ineq
        double T_ineq;
        if(Tmin > _t_now + 0.15){
            T_ineq = Tmin;
        }
        else{
            T_ineq = _t_now + 0.15;
        }
//        if(T_ineq < _T - del_T){
//            T_ineq = _T - del_T;
//        }
//        else{}

        // ------------------minimum step time constraints----------------------
        // -tau < -tau_ineq
        A_ineq(8,4) = -1;
        B_ineq(8,0) = -exp(_W*T_ineq);


        //// Omega constraints
        // Omega_pitch < Omega_max
        A_ineq(9,5) = 1;
        B_ineq(9,0) = Omega_max;

        // -Omega_pitch < Omeaga_max
        A_ineq(10,5) = -1;
        B_ineq(10,0) = Omega_max;

        // Omega_roll < Omega_max
        A_ineq(11,6) = 1;
        B_ineq(11,0) = Omega_max;

        // -Omega_roll < Omeaga_max
        A_ineq(12,6) = -1;
        B_ineq(12,0) = Omega_max;

        // -----------------Swing foot velocity constraits ---------------------
        // T constraint is only depends on max velocity
        // -tau < -(t_now - (pSW.x + del_u_f.x - pSW_cur.x)/Vmax_x   --> from x velocity constraints
        A_ineq(13,4) = -1;
        B_ineq(13,0) = -exp(_W*(_t_now - (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x));
        // -tau < -(t_now + (pSW.x + del_u_f.x - pSW_cur.x)/Vmax_x
        A_ineq(14,4) = -1;
        B_ineq(14,0) = -exp(_W*(_t_now + (_pSW.x + _del_u_f.x - _pSW_cur.x)/Vmax_x));

        // -tau < -(t_now - (pSW.y + del_u_f.y - pSW_cur.y)/Vmax_y   --> from y velocity constraints
        A_ineq(15,4) = -1;
        B_ineq(15,0) = -exp(_W*(_t_now - (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y));
        // -tau < -(t_now + (pSW.y + del_u_f.y - pSW_cur.y)/Vmax_y
        A_ineq(16,4) = -1;
        B_ineq(16,0) = -exp(_W*(_t_now + (_pSW.y + _del_u_f.y - _pSW_cur.y)/Vmax_y));

        // 8 Inequality Constraints
    }

    void SetBehavior_Angular(double _w, double _T_nom){
        // QP and Quadprog++ relation
        //
        // min 0.5*X'GX + g0*X
        //
        // set behavior as AX - B ~ 0
        //
        // (AX-B)'(AX-B) = X'A'AX - B'AX - X'A'B + B'B
        //
        // B'B is constant, so delete for minimize
        //
        // B'AX and X'A'B is scalar, so these are same value
        //
        // 0.5*X'A'AX + (-B'A)X

        // G = A'A  g0 = -B'A or -A'B


        MatrixXd A0, B0;
        double W0;


        //// step time weighting
        double tau_nom = exp(_w*_T_nom);

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,4) = 1;
        B0(0,0) = tau_nom;  // tau ~ tau_nom

        W0 = 0.007;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// step position
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,0) = 1;
        A0(1,1) = 1;
        B0(0,0) = 0;  // del u ~ 0
        B0(1,0) = 0;

        W0 = 0.2;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// DCM offset
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,2) = 1;
        A0(1,3) = 1;
        B0(0,0) = 0;  // del b ~ 0
        B0(1,0) = 0;

        W0 =2.5;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //// Pelvis Omega
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        // Omega ~ 0  or Omega ~ current Omega
        A0(0,5) = 1;
        A0(1,6) = 1;
        B0(0,0) = 0;//pitch
        B0(1,0) = 0;//roll

        W0 =0.03;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


//        //// Step position should not change(oscillate) much
//        A0 = MatrixXd::Zero(numCols, numCols);
//        B0 = MatrixXd::Zero(numCols,1);

//        A0(0,0) = 1;
//        A0(0,1) = 1;
//        B0(0,0) = _last_del_u.x;
//        B0(1,0) = _last_del_u.y;

//        W0 = 300;

//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

//        //// Step time should not change(oscillate) much
//        A0 = MatrixXd::Zero(numCols, numCols);
//        B0 = MatrixXd::Zero(numCols,1);

//        A0(0,4) = 1;
//        B0(0,0) = exp(_w*_last_T);


//        W0 = 0.1;

//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);


    }

    ////---------------------------------------------------------------------------------------------------------------------------------------
    ////-----------------------------------------------2nd Step Optimization-------------------------------------------------------------------
    ////---------------------------------------------------------------------------------------------------------------------------------------

    void MakeEq_2nd(vec3 _del_b0_Nf, vec3 _b0_Nf, double _w, double _T_original){
        //calc in Next stance Foot coordinate
        // origin : center of Next Stance foot
        // orientation : same as Next Stance foot

        // A_eq * X = B_eq

        // X = [del_ux, del_uy, del_bx, del_by, tau]'

        //
        // A_eq = [1 0 1 0 -(b0_x + del_b0_x)]
        //        [0 1 0 1 -(b0_y + del_b0_y)]

        // B_eq = [b0_x*exp(w*T_original)]
        //        [b0_y*exp(w*T_original)]


        A_eq(0,0) = 1;  //del_ux
        A_eq(0,2) = 1;  //del_bx
        A_eq(0,4) = -(_b0_Nf.x + _del_b0_Nf.x); //tau

        A_eq(1,1) = 1;  //del_uy
        A_eq(1,3) = 1;  //del_by
        A_eq(1,4) = -(_b0_Nf.y + _del_b0_Nf.y); //tau

        B_eq(0,0) = -_b0_Nf.x*exp(_w*_T_original);
        B_eq(1,0) = -_b0_Nf.y*exp(_w*_T_original);
    }

    void MakeIneq_2nd(int _swingFoot, vec3 _pSW_2nd, double _W){
        //calc in stance Foot coordinate
        // origin : center of Stance foot
        // orientation : same as Stance foot
        // _pSW : position destination of swing Foot

        // A_ineq * X <= B_ineq

        double Lmax = 0.4;
        double Wmax = 0.6;
        double Wmin = 0.19;

        double Tmin = 0.45;



        //// del_u ineq constraints
        if(_swingFoot == RFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1;
            B_ineq(0,0) = Lmax - _pSW_2nd.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1;
            B_ineq(1,0) = Lmax + _pSW_2nd.x;

            //del_uy < Wmax - pSW.y
            A_ineq(2,1) = 1;
            B_ineq(2,0) = Wmax - _pSW_2nd.y;

            //-del_uy < -Wmin + pSW.y
            A_ineq(3,1) = -1;
            B_ineq(3,0) = -Wmin + _pSW_2nd.y;
        }

        if(_swingFoot == LFoot){
            //del_ux < Lmax-pSW.x
            A_ineq(0,0) = 1;
            B_ineq(0,0) = Lmax - _pSW_2nd.x;

            //-del_ux < pSW.x + Lmax
            A_ineq(1,0) = -1;
            B_ineq(1,0) = Lmax + _pSW_2nd.x;

            //-del_uy < Wmax + pSW.y
            A_ineq(2,1) = -1;
            B_ineq(2,0) = Wmax + _pSW_2nd.y;

            //del_uy < -Wmin - pSW.y
            A_ineq(3,1) = 1;
            B_ineq(3,0) = -Wmin - _pSW_2nd.y;
        }

        //// T constraints
        // -tau < -tau_ineq
        A_ineq(4,4) = -1;
        B_ineq(4.0) = -exp(_W*Tmin);

    }

    void SetBehavior_2nd(double _w, double _T_nom){
        // QP and Quadprog++ relation
        //
        // min 0.5*X'GX + g0*X
        //
        // set behavior as AX - B ~ 0
        //
        // (AX-B)'(AX-B) = X'A'AX - B'AX - X'A'B + B'B
        //
        // B'B is constant, so delete for minimize
        //
        // B'AX and X'A'B is scalar, so these are same value
        //
        // 0.5*X'A'AX + (-B'A)X

        // G = A'A  g0 = -B'A or -A'B


        MatrixXd A0, B0;
        double W0;

        //// step time weighting
        double tau_nom = exp(_w*_T_nom);

        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,4) = 1;
        B0(0,0) = tau_nom;  // tau ~ tau_nom

        W0 = 0.2;

        As2.push_back(A0);
        Bs2.push_back(B0);
        Ws2.push_back(W0);

        //// step position
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,0) = 1;
        A0(1,1) = 1;
        B0(0,0) = 0;  // del u ~ 0
        B0(1,0) = 0;

        W0 = 6;

        As2.push_back(A0);
        Bs2.push_back(B0);
        Ws2.push_back(W0);

        //// DCM offset
        A0 = MatrixXd::Zero(numCols, numCols);
        B0 = MatrixXd::Zero(numCols,1);

        A0(0,2) = 1;
        A0(1,3) = 1;
        B0(0,0) = 0;  // del b ~ 0
        B0(1,0) = 0;

        W0 = 15;

        As2.push_back(A0);
        Bs2.push_back(B0);
        Ws2.push_back(W0);
    }


    void MakeHF(){

        int nn = As.size();

        H = MatrixXd::Zero(numCols,numCols);
        F = MatrixXd::Zero(numCols,1);

        for(int i=0;i<nn;i++)
        {
            MatrixXd As_trans = As[i].transpose();
            H = H + Ws[i]*Ws[i]*(As_trans*As[i]);
            F = F -Ws[i]*Ws[i]*(As_trans*Bs[i]);
        }

    }

    void MakeHF2(){

        int nn = As2.size();
        //cout<<"nn"<<nn<<endl;

        H2 = MatrixXd::Zero(numCols,numCols);
        F2 = MatrixXd::Zero(numCols,1);

        for(int i=0;i<nn;i++)
        {
            H2 = H2 + Ws2[i]*Ws2[i]*(As2[i].transpose()*As2[i]);
            F2 = F2 -Ws2[i]*Ws2[i]*(As2[i].transpose()*Bs2[i]);
        }

    }

    void calcX()
    {
        quadprogpp::Vector<double> outX;
        quadprogpp::Matrix<double> G;
        quadprogpp::Vector<double> g0;
        quadprogpp::Matrix<double> CE;
        quadprogpp::Vector<double> ce0;
        quadprogpp::Matrix<double> CI;
        quadprogpp::Vector<double> ci0;
        //min 0.5 * x G x + g0 x
        //CE^T x + ce0 = 0
        //CI^T x + ci0 >= 0

        G.resize(numCols,numCols);
        g0.resize(numCols);
        for(int i=0;i<numCols;i++)
        {
            for(int j=0;j<numCols;j++)
            {
                G[i][j] = H(i,j);
            }
            g0[i] = F(i,0);
        }
        CE.resize(numCols,numEq);
        ce0.resize(numEq);
        for(int i=0;i<numCols;i++)
        {
            for(int j=0;j<numEq;j++)
            {
                CE[i][j] = -A_eq(j,i);
            }
        }
        for(int j=0;j<numEq;j++)
        {
            ce0[j] = B_eq(j,0);
        }
        CI.resize(numCols,numIneq);
        ci0.resize(numIneq);
        for(int i=0;i<numCols;i++)
        {
            for(int j=0;j<numIneq;j++)
            {
                CI[i][j] = -A_ineq(j,i);
            }
        }
        for(int j=0;j<numIneq;j++)
        {
            ci0[j] = B_ineq(j,0);
        }
        outX.resize(numCols);

        solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
        for(int i=0;i<numCols;i++)
        {
            X[i] = outX[i];
            X_array[i] = outX[i];
        }




    }

    void calcX2()
    {
        quadprogpp::Vector<double> outX;
        quadprogpp::Matrix<double> G;
        quadprogpp::Vector<double> g0;
        quadprogpp::Matrix<double> CE;
        quadprogpp::Vector<double> ce0;
        quadprogpp::Matrix<double> CI;
        quadprogpp::Vector<double> ci0;
        //min 0.5 * x G x + g0 x
        //CE^T x + ce0 = 0
        //CI^T x + ci0 >= 0

        G.resize(numCols,numCols);
        g0.resize(numCols);
        for(int i=0;i<numCols;i++)
        {
            for(int j=0;j<numCols;j++)
            {
                G[i][j] = H2(i,j);
            }
            g0[i] = F2(i,0);
        }
        CE.resize(numCols,numEq);
        ce0.resize(numEq);
        for(int i=0;i<numCols;i++)
        {
            for(int j=0;j<numEq;j++)
            {
                CE[i][j] = -A_eq(j,i);
            }
        }
        for(int j=0;j<numEq;j++)
        {
            ce0[j] = B_eq(j,0);
        }
        CI.resize(numCols,numIneq);
        ci0.resize(numIneq);
        for(int i=0;i<numCols;i++)
        {
            for(int j=0;j<numIneq;j++)
            {
                CI[i][j] = -A_ineq(j,i);
            }
        }
        for(int j=0;j<numIneq;j++)
        {
            ci0[j] = B_ineq(j,0);
        }
        outX.resize(numCols);

        solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
        for(int i=0;i<numCols;i++)
        {
            X[i] = outX[i];
            X_array[i] = outX[i];
        }




    }

    void clear(){
        Ws.clear();
        As.clear();

        Ws2.clear();
        As2.clear();

    }



};

#endif // HB_STEPADJUSTOR

