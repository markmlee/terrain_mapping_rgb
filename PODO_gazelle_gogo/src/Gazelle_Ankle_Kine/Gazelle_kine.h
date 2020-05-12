#ifndef GAZELLE_KINE
#define GAZELLE_KINE

#include <iostream>
#include <unistd.h>

////-----------------Ankle Parameter------------------------------------------------------------
const double l1 = 0.052;  // drive arm length
//const double l2 = 0.32;  // link length
const double l2_L_1 = 0.334; // link length of left leg left link
const double l2_L_2 = 0.334; // link length of left leg right link

const double l2_R_1 = 0.334; // link length of left leg left link
const double l2_R_2 = 0.334;//0.328; // link length of left leg right link

const double l3 = 0.32249; // lower leg length
const double w = .0526;     // knee center to drive arm
const double cx = .0481;    // ankle center to universal (-x)
const double cy = .0455;    // ankle center to universal (abs(y))
const double cz = -0.0015;  // ankle center to universal (+z)

//const double DriveArm_offset = 0;
//angle offset in degree
const double offset_L1 = -10.999;//left leg left  //LAP
const double offset_L2 = -10.999;//left leg right //LAR
const double offset_R1 = -10.999; //right leg left //RAP
const double offset_R2 = -10.999; //right leg right //RAR
//const double DriveArm_offset = -0.405;

////------------------------------ BasicMath.h-----------------------------------------------------

#include <math.h>
#include <stdio.h>

const double _R2D = 57.2957802f;
const double _D2R = 0.0174533f;

inline double _abs(double a){return (a>=0 ? a : -a);}


////-----------------------------------------------------------------------------------------------


////------------------------------BasicMatrix.h ----------------------------------------------------


using namespace std;

class _vec3;
class _mat3;


class _vec3_{
public:
    union{
        struct{double x, y, z;};
        double v[3];
    };
          double& operator[] (int i)       {return v[i];}
    const double& operator[] (int i) const {return v[i];}
};

class _mat3_{
public:
    union{
        struct{double m00,m01,m02,m10,m11,m12,m20,m21,m22;};
        struct{_vec3_ x, y, z;};
        double m[3][3];
        double v[9];
    };

          double* operator[] (int i)       {return m[i];}
    const double* operator[] (int i) const {return m[i];}
          double& operator() (int i, int j)       {return m[i][j];}
    const double& operator() (int i, int j) const {return m[i][j];}
};

class _vec3 : public _vec3_{
public:
    _vec3(double x, double y, double z);
};

class _mat3 : public _mat3_{
public:
    _mat3(double a00, double a01, double a02,
         double a10, double a11, double a12,
         double a20, double a21, double a22);
};

// =======================================
//              constructors
// =======================================
inline _vec3::_vec3(double x, double y, double z)         {this->x=x;  this->y=y;  this->z=z;}

inline _mat3::_mat3(double a00, double a01, double a02,
           double a10, double a11, double a12,
           double a20, double a21, double a22){
    m00 = a00; m01 = a01; m02 = a02;
    m10 = a10; m11 = a11; m12 = a12;
    m20 = a20; m21 = a21; m22 = a22;
}

// =======================================
//   vector-vector_operators & functions
// =======================================
// vec3
inline _vec3  operator +(const _vec3_& v1, const _vec3_& v2){return _vec3(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);}
inline _vec3  operator -(const _vec3_& v1, const _vec3_& v2){return _vec3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z);}
inline _vec3  cross     (const _vec3_& v1, const _vec3_& v2){return _vec3(v1.y*v2.z - v1.z*v2.y,v1.z*v2.x - v1.x*v2.z,v1.x*v2.y - v1.y*v2.x);}

// =======================================
//   matrix-vector_operators & functions
// =======================================
inline _vec3   operator *(const _mat3_& m, const _vec3_& v)  {
    return _vec3(
        m.m00*v.x + m.m01*v.y + m.m02*v.z,
        m.m10*v.x + m.m11*v.y + m.m12*v.z,
        m.m20*v.x + m.m21*v.y + m.m22*v.z);
}

// =======================================
//   matrix-double_operators & functions
// =======================================
inline _mat3   operator *(double val, const _mat3_& m)     {
    return _mat3(m.m00*val,m.m01*val,m.m02*val,
                m.m10*val,m.m11*val,m.m12*val,
                m.m20*val,m.m21*val,m.m22*val);
}

////----------------------------------------------------------------------------------------------



////----------------------------------------------------------------------------------------------------

//Ankle kinematics

class Gazelle_Kine{
public:
    Gazelle_Kine(){

    }
    void Calc_Ankle_offset(){

        double _pitch_deg = 0;
        double _roll_deg = 0;

        double theta1, theta2;

        double phi = _pitch_deg*_D2R;
        double psi = _roll_deg*_D2R;

        double Ap = cx*cx + cy*cy + cz*cz + l1*l1 + w*w - 2*cy*w*cos(psi) + 2*cz*w*sin(psi) - l2_L_1*l2_L_1 + l3*l3 - 2*cz*l3*cos(phi)*cos(psi) - 2*cy*l3*cos(phi)*sin(psi) - 2*cx*l3*sin(phi);
        double B = -2*cx*l1*cos(phi) + 2*cz*l1*cos(psi)*sin(phi) + 2*cy*l1*sin(phi)*sin(psi);
        double C = -2*cx*l1*sin(phi) - 2*cz*l1*cos(phi)*cos(psi) - 2*cy*l1*cos(phi)*sin(psi) + 2*l1*l3;

        double t1[2] = {0,0};

        t1[0] = (-2*C + sqrt(4*C*C - 4*Ap*Ap + 4*B*B))/(2*Ap - 2*B);
        t1[1] = (-2*C - sqrt(4*C*C - 4*Ap*Ap + 4*B*B))/(2*Ap - 2*B);

        for (int i = 0;i<2;i++){
            if (_abs(int(2*atan(t1[i])*_R2D))<60){
                theta1 = 2*atan(t1[i])*_R2D;
            }
        }

        //if(fabs(theta1 - _theta1_deg_ini) > 5.0) theta1 = _theta1_deg_ini;

        double Ap2 = cx*cx + cy*cy + cz*cz + l1*l1 + w*w - 2*cy*w*cos(psi) - 2*cz*w*sin(psi) - l2_L_2*l2_L_2 + l3*l3 - 2*cz*l3*cos(phi)*cos(psi) + 2*cy*l3*cos(phi)*sin(psi) - 2*cx*l3*sin(phi);
        double B2 = -2*cx*l1*cos(phi) + 2*cz*l1*cos(psi)*sin(phi) - 2*cy*l1*sin(phi)*sin(psi);
        double C2 = -2*cx*l1*sin(phi) - 2*cz*l1*cos(phi)*cos(psi) + 2*cy*l1*cos(phi)*sin(psi) + 2*l1*l3;

        double t2[2] = {0, 0};

        t2[0] = (-2*C2 + sqrt(4*C2*C2 - 4*Ap2*Ap2 + 4*B2*B2))/(2*Ap2 - 2*B2);
        t2[1] = (-2*C2 - sqrt(4*C2*C2 - 4*Ap2*Ap2 + 4*B2*B2))/(2*Ap2 - 2*B2);

        for (int i = 0;i<2;i++){
            if (_abs(int(2*atan(t2[i])*_R2D))<60){
                theta2 = 2*atan(t2[i])*_R2D;
            }
        }

        //if(fabs(theta2 - _theta2_deg_ini) > 5.0) theta2 = _theta2_deg_ini;


        std::cout<<"Off1: "<<theta1<<", Off2: "<<theta2<<std::endl;

    }

    void IK_Ankle_left(double _pitch_deg, double _roll_deg, double &A1_deg, double &A2_deg){  //return theta1_deg, theta2_deg : vec3(theta1, theta2, 0)


        double theta1, theta2;

        double phi = _pitch_deg*_D2R;
        double psi = _roll_deg*_D2R;

        double Ap = cx*cx + cy*cy + cz*cz + l1*l1 + w*w - 2*cy*w*cos(psi) + 2*cz*w*sin(psi) - l2_L_1*l2_L_1 + l3*l3 - 2*cz*l3*cos(phi)*cos(psi) - 2*cy*l3*cos(phi)*sin(psi) - 2*cx*l3*sin(phi);
        double B = -2*cx*l1*cos(phi) + 2*cz*l1*cos(psi)*sin(phi) + 2*cy*l1*sin(phi)*sin(psi);
        double C = -2*cx*l1*sin(phi) - 2*cz*l1*cos(phi)*cos(psi) - 2*cy*l1*cos(phi)*sin(psi) + 2*l1*l3;

        double t1[2] = {0,0};

        t1[0] = (-2*C + sqrt(4*C*C - 4*Ap*Ap + 4*B*B))/(2*Ap - 2*B);
        t1[1] = (-2*C - sqrt(4*C*C - 4*Ap*Ap + 4*B*B))/(2*Ap - 2*B);

        for (int i = 0;i<2;i++){
            if (_abs(int(2*atan(t1[i])*_R2D))<60){
                theta1 = 2*atan(t1[i])*_R2D;
            }
        }

        //if(fabs(theta1 - _theta1_deg_ini) > 5.0) theta1 = _theta1_deg_ini;

        double Ap2 = cx*cx + cy*cy + cz*cz + l1*l1 + w*w - 2*cy*w*cos(psi) - 2*cz*w*sin(psi) - l2_L_2*l2_L_2 + l3*l3 - 2*cz*l3*cos(phi)*cos(psi) + 2*cy*l3*cos(phi)*sin(psi) - 2*cx*l3*sin(phi);
        double B2 = -2*cx*l1*cos(phi) + 2*cz*l1*cos(psi)*sin(phi) - 2*cy*l1*sin(phi)*sin(psi);
        double C2 = -2*cx*l1*sin(phi) - 2*cz*l1*cos(phi)*cos(psi) + 2*cy*l1*cos(phi)*sin(psi) + 2*l1*l3;

        double t2[2] = {0, 0};

        t2[0] = (-2*C2 + sqrt(4*C2*C2 - 4*Ap2*Ap2 + 4*B2*B2))/(2*Ap2 - 2*B2);
        t2[1] = (-2*C2 - sqrt(4*C2*C2 - 4*Ap2*Ap2 + 4*B2*B2))/(2*Ap2 - 2*B2);

        for (int i = 0;i<2;i++){
            if (_abs(int(2*atan(t2[i])*_R2D))<60){
                theta2 = 2*atan(t2[i])*_R2D;
            }
        }

        //if(fabs(theta2 - _theta2_deg_ini) > 5.0) theta2 = _theta2_deg_ini;

        A1_deg = theta1 + offset_L1;
        A2_deg = theta2 + offset_L2;
    }

    void IK_Ankle_right(double _pitch_deg, double _roll_deg, double &A1_deg, double &A2_deg){  //return theta1_deg, theta2_deg : vec3(theta1, theta2, 0)


        double theta1, theta2;

        double phi = _pitch_deg*_D2R;
        double psi = _roll_deg*_D2R;

        double Ap = cx*cx + cy*cy + cz*cz + l1*l1 + w*w - 2*cy*w*cos(psi) + 2*cz*w*sin(psi) - l2_R_1*l2_R_1 + l3*l3 - 2*cz*l3*cos(phi)*cos(psi) - 2*cy*l3*cos(phi)*sin(psi) - 2*cx*l3*sin(phi);
        double B = -2*cx*l1*cos(phi) + 2*cz*l1*cos(psi)*sin(phi) + 2*cy*l1*sin(phi)*sin(psi);
        double C = -2*cx*l1*sin(phi) - 2*cz*l1*cos(phi)*cos(psi) - 2*cy*l1*cos(phi)*sin(psi) + 2*l1*l3;

        double t1[2] = {0,0};

        t1[0] = (-2*C + sqrt(4*C*C - 4*Ap*Ap + 4*B*B))/(2*Ap - 2*B);
        t1[1] = (-2*C - sqrt(4*C*C - 4*Ap*Ap + 4*B*B))/(2*Ap - 2*B);

        for (int i = 0;i<2;i++){
            if (_abs(int(2*atan(t1[i])*_R2D))<60){
                theta1 = 2*atan(t1[i])*_R2D;
            }
        }

        //if(fabs(theta1 - _theta1_deg_ini) > 5.0) theta1 = _theta1_deg_ini;

        double Ap2 = cx*cx + cy*cy + cz*cz + l1*l1 + w*w - 2*cy*w*cos(psi) - 2*cz*w*sin(psi) - l2_R_2*l2_R_2 + l3*l3 - 2*cz*l3*cos(phi)*cos(psi) + 2*cy*l3*cos(phi)*sin(psi) - 2*cx*l3*sin(phi);
        double B2 = -2*cx*l1*cos(phi) + 2*cz*l1*cos(psi)*sin(phi) - 2*cy*l1*sin(phi)*sin(psi);
        double C2 = -2*cx*l1*sin(phi) - 2*cz*l1*cos(phi)*cos(psi) + 2*cy*l1*cos(phi)*sin(psi) + 2*l1*l3;

        double t2[2] = {0, 0};

        t2[0] = (-2*C2 + sqrt(4*C2*C2 - 4*Ap2*Ap2 + 4*B2*B2))/(2*Ap2 - 2*B2);
        t2[1] = (-2*C2 - sqrt(4*C2*C2 - 4*Ap2*Ap2 + 4*B2*B2))/(2*Ap2 - 2*B2);

        for (int i = 0;i<2;i++){
            if (_abs(int(2*atan(t2[i])*_R2D))<60){
                theta2 = 2*atan(t2[i])*_R2D;
            }
        }

        //if(fabs(theta2 - _theta2_deg_ini) > 5.0) theta2 = _theta2_deg_ini;

        A1_deg = theta1 + offset_R1;
        A2_deg = theta2 + offset_R2;
    }



    _mat3 gazelle_rotation_mtx(double _pitch_deg, double _roll_deg){  //)

        double p = _pitch_deg*_D2R;
        double r = _roll_deg*_D2R;

        return _mat3(cos(p), sin(p)*sin(r), sin(p)*cos(r),
                    0,      cos(r),        -sin(r),
                    -sin(p),cos(p)*sin(r), cos(p)*cos(r));

    }

    _mat3 gazelle_jacobian(double _pitch_deg, double _roll_deg, double _th1_deg, double _th2_deg){

        //q = [motor1 motor2]' ( motor1 : left,  motor2 : right)
        //x = [pitch roll]'

        // x' = Jq'


        double th1 = _th1_deg*_D2R; double th2 = _th2_deg*_D2R;

        _mat3 R_fo = gazelle_rotation_mtx(_pitch_deg,_roll_deg);
        //mat3 R_fo = mat3(vec3(0,1,0),_pitch_deg*D2R)*mat3(vec3(1,0,0), _roll_deg*D2R);

        _vec3 A1B1 = _vec3(-l1*cos(th1), 0, l1*sin(th1));
        _vec3 A2B2 = _vec3(-l1*cos(th2), 0, l1*sin(th2));
        _vec3 OF = _vec3(0, 0, -l3);

        _vec3 FC1 = _vec3(-cx, cy, cz);
        _vec3 FC2 = _vec3(-cx, -cy, cz);

        _vec3 OB1 = _vec3(-l1*cos(th1), w, l1*sin(th1));
        _vec3 OB2 = _vec3(-l1*cos(th2), -w, l1*sin(th2));

        _vec3 B1C1 = OF + R_fo*FC1 - OB1;
        _vec3 B2C2 = OF + R_fo*FC2 - OB2;

        _vec3 J = cross(A1B1,B1C1);
        _vec3 K = cross(A2B2,B2C2);
        _vec3 L = cross(FC1,B1C1);
        _vec3 M = cross(FC2,B2C2);



        _mat3 J_inv = _mat3(L.y/J.y, L.x/J.y, 0,
                          M.y/K.y, M.x/K.y, 0,
                          0,       0,       0);

        double det = J_inv.m00*J_inv.m11 - J_inv.m01*J_inv.m10;

        _mat3 jacobian = 1/det*_mat3(J_inv.m11, -J_inv.m01, 0,
                                   -J_inv.m10, J_inv.m00, 0,
                                   0,          0,         0);
        return jacobian;
    }

    void gazelle_jacobian_double(double _pitch_deg, double _roll_deg, double _th1_deg, double _th2_deg,
                          double &J00, double &J01, double &J10, double &J11){

        //q = [motor1 motor2]' ( motor1 : left,  motor2 : right)
        //x = [pitch roll]'

        // x' = Jq'


        double th1 = _th1_deg*_D2R; double th2 = _th2_deg*_D2R;

        _mat3 R_fo = gazelle_rotation_mtx(_pitch_deg,_roll_deg);
        //mat3 R_fo = mat3(vec3(0,1,0),_pitch_deg*D2R)*mat3(vec3(1,0,0), _roll_deg*D2R);

        _vec3 A1B1 = _vec3(-l1*cos(th1), 0, l1*sin(th1));
        _vec3 A2B2 = _vec3(-l1*cos(th2), 0, l1*sin(th2));
        _vec3 OF = _vec3(0, 0, -l3);

        _vec3 FC1 = _vec3(-cx, cy, cz);
        _vec3 FC2 = _vec3(-cx, -cy, cz);

        _vec3 OB1 = _vec3(-l1*cos(th1), w, l1*sin(th1));
        _vec3 OB2 = _vec3(-l1*cos(th2), -w, l1*sin(th2));

        _vec3 B1C1 = OF + R_fo*FC1 - OB1;
        _vec3 B2C2 = OF + R_fo*FC2 - OB2;

        _vec3 J = cross(A1B1,B1C1);
        _vec3 K = cross(A2B2,B2C2);
        _vec3 L = cross(FC1,B1C1);
        _vec3 M = cross(FC2,B2C2);



        _mat3 J_inv = _mat3(L.y/J.y, L.x/J.y, 0,
                          M.y/K.y, M.x/K.y, 0,
                          0,       0,       0);

        double det = J_inv.m00*J_inv.m11 - J_inv.m01*J_inv.m10;

        _mat3 jacobian = 1/det*_mat3(J_inv.m11, -J_inv.m01, 0,
                                   -J_inv.m10, J_inv.m00, 0,
                                   0,          0,         0);

        J00 = jacobian.m00;
        J01 = jacobian.m01;
        J10 = jacobian.m10;
        J11 = jacobian.m11;
    }
    
    void gazelle_jacobian_numeric_left(double _pitch_deg, double _roll_deg, double _th1_deg, double _th2_deg,
                          double &J00, double &J01, double &J10, double &J11){

        //q = [motor1 motor2]' ( motor1 : left,  motor2 : right)
        //x = [pitch roll]'

        // x' = Jq'
        
        //J00 = dx/dq1
        //J01 = dx/dq2
        //J10 = dy/dq1
        //J11 = dy/dq2

        double margin = 0.05;  //deg
        double Ji00, Ji01, Ji10, Ji11;
        
        //stage1 (J00, J01)
        double A1_new_deg, A2_new_deg;
        IK_Ankle_left(_pitch_deg + margin, _roll_deg, A1_new_deg, A2_new_deg);
        
        double dA1_deg = A1_new_deg - _th1_deg;
        double dA2_deg = A2_new_deg - _th2_deg;

        Ji00 = dA1_deg/margin;
        Ji10 = dA2_deg/margin;

        //stage2 (J10, J11)
        IK_Ankle_left(_pitch_deg, _roll_deg + margin, A1_new_deg, A2_new_deg);

        dA1_deg = A1_new_deg - _th1_deg;
        dA2_deg = A2_new_deg - _th2_deg;

        Ji01 = dA1_deg/margin;
        Ji11 = dA2_deg/margin;

        double det = Ji00*Ji11 - Ji01*Ji10;

        J00 = Ji11/det;
        J01 = -Ji01/det;
        J10 = -Ji10/det;
        J11 = Ji00/det;

    }

    void gazelle_jacobian_numeric_right(double _pitch_deg, double _roll_deg, double _th1_deg, double _th2_deg,
                          double &J00, double &J01, double &J10, double &J11){

        //q = [motor1 motor2]' ( motor1 : left,  motor2 : right)
        //x = [pitch roll]'

        // x' = Jq'

        //J00 = dx/dq1
        //J01 = dx/dq2
        //J10 = dy/dq1
        //J11 = dy/dq2

        double margin = 0.05;  //deg
        double Ji00, Ji01, Ji10, Ji11;

        //stage1 (J00, J01)
        double A1_new_deg, A2_new_deg;
        IK_Ankle_right(_pitch_deg + margin, _roll_deg, A1_new_deg, A2_new_deg);

        double dA1_deg = A1_new_deg - _th1_deg;
        double dA2_deg = A2_new_deg - _th2_deg;

        Ji00 = dA1_deg/margin;
        Ji10 = dA2_deg/margin;

        //stage2 (J10, J11)
        IK_Ankle_right(_pitch_deg, _roll_deg + margin, A1_new_deg, A2_new_deg);

        dA1_deg = A1_new_deg - _th1_deg;
        dA2_deg = A2_new_deg - _th2_deg;

        Ji01 = dA1_deg/margin;
        Ji11 = dA2_deg/margin;

        double det = Ji00*Ji11 - Ji01*Ji10;

        J00 = Ji11/det;
        J01 = -Ji01/det;
        J10 = -Ji10/det;
        J11 = Ji00/det;

    }

    void FK_diff_Ankle_left(double _th1_deg, double _th2_deg, double _old_pitch_deg, double _old_roll_deg, double &AP_deg, double &AR_deg)
    //differential forward kinematics : input th1 degree th2 degree, and last reference of roll and pitch degree
    //note that you can input old_pitch_deg and _old_roll_deg as 0,0 because the FK_differential will converge super fast
    {

    //    double th1_meas = _th1_deg*D2R; double th2_meas = _th2_deg*D2R;
    //    double p   = _old_pitch_deg*D2R; double r = _old_roll_deg*D2R;

        _th1_deg -= offset_L1;
        _th2_deg -= offset_L2;

        double K = 1;

        _vec3 pitch_roll = _vec3(_old_pitch_deg,_old_roll_deg,0);

        int i = 0;
        while (i <= 10)
        {

            double A1_deg, A2_deg;
            IK_Ankle_left(pitch_roll[0], pitch_roll[1], A1_deg, A2_deg);
            _vec3 th_est = _vec3(A1_deg - offset_L1, A2_deg - offset_L2, 0);



            _vec3 e = _vec3(_th1_deg - th_est[0], _th2_deg - th_est[1], 0);
            //cout<<" error : "<<e[0]<<" "<<e[1]<<endl;
            _mat3 J = gazelle_jacobian(pitch_roll[0], pitch_roll[1], th_est[0], th_est[1]);
            pitch_roll = K*J*e + pitch_roll;  //vector addition and multiplication is this okay?
            if(sqrt(e[0]*e[0] + e[1]*e[1]) < 0.0001){
                break;
            }
            //cout<<"pitch : "<<pitch_roll[0]<<" roll: "<<pitch_roll[1]<<endl;
            i++;
        }

        AP_deg = pitch_roll[0];
        AR_deg = pitch_roll[1];

    }

    void FK_diff_Ankle_right(double _th1_deg, double _th2_deg, double _old_pitch_deg, double _old_roll_deg, double &AP_deg, double &AR_deg)
    //differential forward kinematics : input th1 degree th2 degree, and last reference of roll and pitch degree
    //note that you can input old_pitch_deg and _old_roll_deg as 0,0 because the FK_differential will converge super fast
    {

    //    double th1_meas = _th1_deg*D2R; double th2_meas = _th2_deg*D2R;
    //    double p   = _old_pitch_deg*D2R; double r = _old_roll_deg*D2R;

        _th1_deg -= offset_R1;
        _th2_deg -= offset_R2;

        double K = 1;

        _vec3 pitch_roll = _vec3(_old_pitch_deg,_old_roll_deg,0);

        int i = 0;
        while (i <= 10)
        {

            double A1_deg, A2_deg;
            IK_Ankle_right(pitch_roll[0], pitch_roll[1], A1_deg, A2_deg);
            _vec3 th_est = _vec3(A1_deg - offset_R1, A2_deg - offset_R2, 0);



            _vec3 e = _vec3(_th1_deg - th_est[0], _th2_deg - th_est[1], 0);
//            cout<<" error : "<<e[0]<<" "<<e[1]<<endl;
            _mat3 J = gazelle_jacobian(pitch_roll[0], pitch_roll[1], th_est[0], th_est[1]);
            pitch_roll = K*J*e + pitch_roll;  //vector addition and multiplication is this okay?
            if(sqrt(e[0]*e[0] + e[1]*e[1]) < 0.0001){
                break;
            }
//            cout<<"pitch : "<<pitch_roll[0]<<" roll: "<<pitch_roll[1]<<endl;
            i++;
        }

        AP_deg = pitch_roll[0];
        AR_deg = pitch_roll[1];

    }

    void FK_diff_Ankle_vel(double _th1_vel_deg, double _th2_vel_deg, double _th1_deg, double _th2_deg, double _pitch_deg, double _roll_deg, double &_pitch_vel_deg, double &_roll_vel_deg){
        _mat3 J = gazelle_jacobian(_pitch_deg, _roll_deg, _th1_deg, _th2_deg);

        _vec3 Theta_vel_deg = _vec3(_th1_vel_deg, _th2_vel_deg, 0);

        _vec3 RP_vel_deg = J*Theta_vel_deg;

        _pitch_vel_deg = RP_vel_deg[0];
        _roll_vel_deg = RP_vel_deg[1];
    }


};


#endif // GAZELLE_KINE




