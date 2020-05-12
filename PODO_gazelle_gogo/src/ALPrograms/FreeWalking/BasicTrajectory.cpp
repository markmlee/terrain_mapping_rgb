


#include <iostream>
#include "BasicTrajectory.h"

using namespace std;
namespace rainbow{

//===================================================================================
//===================================================================================
// 1st Order Trajectory
//===================================================================================
//===================================================================================
int TRInfoOrder1::GetCurrentValue(TRInfo _info){
    currentAcceleration = ((TRInfoOrder1*)_info)->getCurrentAcceleration();
    currentVelocity = ((TRInfoOrder1*)_info)->getCurrentVelocity();
    currentPosition = ((TRInfoOrder1*)_info)->getCurrentPosition();
    return RB_SUCCESS;
}
int TRInfoOrder1::GetCurrentValue(doubles _retVal){	//this function will be called in special cases
                                                            //when there is no preceeding trajectory information
    currentPosition = _retVal[0];
    currentVelocity = 0.0;
    currentAcceleration = 0.0;
//    currentVelocity = _retVal[1];
//    currentAcceleration = _retVal[2];
    return RB_SUCCESS;
}

//===================================================================================
//===================================================================================
// 6st Order Trajectory
//===================================================================================
//===================================================================================
int TRInfoOrder6::GetCurrentValue(TRInfo _info){
    currentPosition_x = ((TRInfoOrder6*)_info)->getCurrentPosition_x();
    currentPosition_y = ((TRInfoOrder6*)_info)->getCurrentPosition_y();
    currentPosition_z = ((TRInfoOrder6*)_info)->getCurrentPosition_z();

    currentOrientation_yaw = ((TRInfoOrder6*)_info)->getCurrentOrientation_yaw();
    currentOrientation_roll = ((TRInfoOrder6*)_info)->getCurrentOrientation_roll();
    currentOrientation_pitch = ((TRInfoOrder6*)_info)->getCurrentOrientation_pitch();

    return RB_SUCCESS;
}
int TRInfoOrder6::GetCurrentValue(doubles _retVal){	//this function will be called in special cases
                                                            //when there is no preceeding trajectory information
    currentPosition_x = _retVal[0];
    currentPosition_y = _retVal[1];
    currentPosition_z = _retVal[2];
    currentOrientation_yaw = _retVal[3];
    currentOrientation_roll = _retVal[4];
    currentOrientation_pitch = _retVal[5];


//    currentVelocity = _retVal[1];
//    currentAcceleration = _retVal[2];
    return RB_SUCCESS;
}


// Constant Trajectory Method------------------------------
// Heel Pitching Trajectory Method------------------------------
int	TrajectoryHeelPitching_x::CalculateParameter(){
//    double polyTemp1, polyTemp2;
//    polyTemp1 = goalRotation;

//    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
//    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
//    trajParam[2] = currentVelocity;
//    trajParam[3] = currentPosition;

    double  t1 = Hyaw*D2Rf, t2 = Hroll*D2Rf, t3 = Hpitch*D2Rf;

//    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    GH[0][1] = sin(t1)*cos(t2);
//    GH[0][2] = cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2);
//    GH[0][3] = Hx;

//    GH[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    GH[1][1] = cos(t1)*cos(t2);
//    GH[1][2] = -sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3);
//    GH[1][3] = Hy;

//    GH[2][0] = -cos(t2)*sin(t3);
//    GH[2][1] = -sin(t2);
//    GH[2][2] = cos(t2)*cos(t3);
//    GH[2][3] = Hz;

//    GH[3][0] = 0.0f;
//    GH[3][1] = 0.0f;
//    GH[3][2] = 0.0f;
//    GH[3][3] = 1.0f;
    L = 0.12f;

    cout<<"CAL:"<<"t1:"<<t1<<","<<"t2:"<<t2<<","<<"t3:"<<t3<<endl;
    cout<<"Pos:"<<"Hx:"<<Hx<<","<<"Hy:"<<Hy<<","<<"Hz:"<<Hz<<endl;

    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
    GH[0][1] = -sin(t1)*cos(t2);
    GH[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);
    GH[0][3] = Hx;

    GH[1][0] = sin(t1)*cos(t3)+sin(t3)*cos(t1)*sin(t2);
    GH[1][1] = cos(t1)*cos(t2);
    GH[1][2] = sin(t1)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
    GH[1][3] = Hy;

    GH[2][0] = -cos(t2)*sin(t3);
    GH[2][1] = sin(t2);
    GH[2][2] = cos(t2)*cos(t3);
    GH[2][3] = Hz;

    GH[3][0] = 0.0f;
    GH[3][1] = 0.0f;
    GH[3][2] = 0.0f;
    GH[3][3] = 1.0f;



    F[0][0] = 1.0f;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[0][3] = L;

    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[1][3] = 0.0f;

    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;
    F[2][3] = 0.0f;

    F[3][0] = 0.0f;
    F[3][1] = 0.0f;
    F[3][2] = 0.0f;
    F[3][3] = 1.0f;


    kx = GH[0][1]; ky = GH[1][1]; kz = GH[2][1];

    return RB_SUCCESS;
}



// Heel Pitching Trajectory Method------------------------------
int	TrajectoryHeelPitching_y::CalculateParameter(){
//    double polyTemp1, polyTemp2;
//    polyTemp1 = goalRotation;

//    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
//    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
//    trajParam[2] = currentVelocity;
//    trajParam[3] = currentPosition;

    double  t1 = Hyaw*D2Rf, t2 = Hroll*D2Rf, t3 = Hpitch*D2Rf;

//    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    GH[0][1] = sin(t1)*cos(t2);
//    GH[0][2] = cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2);
//    GH[0][3] = Hx;

//    GH[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    GH[1][1] = cos(t1)*cos(t2);
//    GH[1][2] = -sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3);
//    GH[1][3] = Hy;

//    GH[2][0] = -cos(t2)*sin(t3);
//    GH[2][1] = -sin(t2);
//    GH[2][2] = cos(t2)*cos(t3);
//    GH[2][3] = Hz;

//    GH[3][0] = 0.0f;
//    GH[3][1] = 0.0f;
//    GH[3][2] = 0.0f;
//    GH[3][3] = 1.0f;

    L = 0.12f;

    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
    GH[0][1] = -sin(t1)*cos(t2);
    GH[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);
    GH[0][3] = Hx;

    GH[1][0] = sin(t1)*cos(t3)+sin(t3)*cos(t1)*sin(t2);
    GH[1][1] = cos(t1)*cos(t2);
    GH[1][2] = sin(t1)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
    GH[1][3] = Hy;

    GH[2][0] = -cos(t2)*sin(t3);
    GH[2][1] = sin(t2);
    GH[2][2] = cos(t2)*cos(t3);
    GH[2][3] = Hz;

    GH[3][0] = 0.0f;
    GH[3][1] = 0.0f;
    GH[3][2] = 0.0f;
    GH[3][3] = 1.0f;



    F[0][0] = 1.0f;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[0][3] = L;

    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[1][3] = 0.0f;

    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;
    F[2][3] = 0.0f;

    F[3][0] = 0.0f;
    F[3][1] = 0.0f;
    F[3][2] = 0.0f;
    F[3][3] = 1.0f;


    kx = GH[0][1]; ky = GH[1][1]; kz = GH[2][1];

    return RB_SUCCESS;
}



// Heel Pitching Trajectory Method------------------------------
int	TrajectoryHeelPitching_z::CalculateParameter(){
//    double polyTemp1, polyTemp2;
//    polyTemp1 = goalRotation;

//    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
//    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
//    trajParam[2] = currentVelocity;
//    trajParam[3] = currentPosition;

    double  t1 = Hyaw*D2Rf, t2 = Hroll*D2Rf, t3 = Hpitch*D2Rf;
L = 0.12f;
//    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    GH[0][1] = sin(t1)*cos(t2);
//    GH[0][2] = cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2);
//    GH[0][3] = Hx;

//    GH[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    GH[1][1] = cos(t1)*cos(t2);
//    GH[1][2] = -sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3);
//    GH[1][3] = Hy;

//    GH[2][0] = -cos(t2)*sin(t3);
//    GH[2][1] = -sin(t2);
//    GH[2][2] = cos(t2)*cos(t3);
//    GH[2][3] = Hz;

//    GH[3][0] = 0.0f;
//    GH[3][1] = 0.0f;
//    GH[3][2] = 0.0f;
//    GH[3][3] = 1.0f;

    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
    GH[0][1] = -sin(t1)*cos(t2);
    GH[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);
    GH[0][3] = Hx;

    GH[1][0] = sin(t1)*cos(t3)+sin(t3)*cos(t1)*sin(t2);
    GH[1][1] = cos(t1)*cos(t2);
    GH[1][2] = sin(t1)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
    GH[1][3] = Hy;

    GH[2][0] = -cos(t2)*sin(t3);
    GH[2][1] = sin(t2);
    GH[2][2] = cos(t2)*cos(t3);
    GH[2][3] = Hz;

    GH[3][0] = 0.0f;
    GH[3][1] = 0.0f;
    GH[3][2] = 0.0f;
    GH[3][3] = 1.0f;



    F[0][0] = 1.0f;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[0][3] = L;

    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[1][3] = 0.0f;

    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;
    F[2][3] = 0.0f;

    F[3][0] = 0.0f;
    F[3][1] = 0.0f;
    F[3][2] = 0.0f;
    F[3][3] = 1.0f;


    kx = GH[0][1]; ky = GH[1][1]; kz = GH[2][1];

    return RB_SUCCESS;
}



// Heel Pitching Trajectory Method------------------------------
int	TrajectoryHeelPitching_roll::CalculateParameter(){
//    double polyTemp1, polyTemp2;
//    polyTemp1 = goalRotation;

//    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
//    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
//    trajParam[2] = currentVelocity;
//    trajParam[3] = currentPosition;

    double  t1 = Hyaw*D2Rf, t2 = Hroll*D2Rf, t3 = Hpitch*D2Rf;
L = 0.12f;
//    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    GH[0][1] = sin(t1)*cos(t2);
//    GH[0][2] = cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2);
//    GH[0][3] = Hx;

//    GH[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    GH[1][1] = cos(t1)*cos(t2);
//    GH[1][2] = -sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3);
//    GH[1][3] = Hy;

//    GH[2][0] = -cos(t2)*sin(t3);
//    GH[2][1] = -sin(t2);
//    GH[2][2] = cos(t2)*cos(t3);
//    GH[2][3] = Hz;

//    GH[3][0] = 0.0f;
//    GH[3][1] = 0.0f;
//    GH[3][2] = 0.0f;
//    GH[3][3] = 1.0f;

    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
    GH[0][1] = -sin(t1)*cos(t2);
    GH[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);
    GH[0][3] = Hx;

    GH[1][0] = sin(t1)*cos(t3)+sin(t3)*cos(t1)*sin(t2);
    GH[1][1] = cos(t1)*cos(t2);
    GH[1][2] = sin(t1)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
    GH[1][3] = Hy;

    GH[2][0] = -cos(t2)*sin(t3);
    GH[2][1] = sin(t2);
    GH[2][2] = cos(t2)*cos(t3);
    GH[2][3] = Hz;

    GH[3][0] = 0.0f;
    GH[3][1] = 0.0f;
    GH[3][2] = 0.0f;
    GH[3][3] = 1.0f;


    F[0][0] = 1.0f;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[0][3] = L;

    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[1][3] = 0.0f;

    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;
    F[2][3] = 0.0f;

    F[3][0] = 0.0f;
    F[3][1] = 0.0f;
    F[3][2] = 0.0f;
    F[3][3] = 1.0f;


    kx = GH[0][1]; ky = GH[1][1]; kz = GH[2][1];

    return RB_SUCCESS;
}


// Heel Pitching Trajectory Method------------------------------
int	TrajectoryHeelPitching_yaw::CalculateParameter(){
//    double polyTemp1, polyTemp2;
//    polyTemp1 = goalRotation;

//    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
//    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
//    trajParam[2] = currentVelocity;
//    trajParam[3] = currentPosition;

    double  t1 = Hyaw*D2Rf, t2 = Hroll*D2Rf, t3 = Hpitch*D2Rf;
L = 0.12f;
//    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    GH[0][1] = sin(t1)*cos(t2);
//    GH[0][2] = cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2);
//    GH[0][3] = Hx;

//    GH[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    GH[1][1] = cos(t1)*cos(t2);
//    GH[1][2] = -sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3);
//    GH[1][3] = Hy;

//    GH[2][0] = -cos(t2)*sin(t3);
//    GH[2][1] = -sin(t2);
//    GH[2][2] = cos(t2)*cos(t3);
//    GH[2][3] = Hz;

//    GH[3][0] = 0.0f;
//    GH[3][1] = 0.0f;
//    GH[3][2] = 0.0f;
//    GH[3][3] = 1.0f;
    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
    GH[0][1] = -sin(t1)*cos(t2);
    GH[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);
    GH[0][3] = Hx;

    GH[1][0] = sin(t1)*cos(t3)+sin(t3)*cos(t1)*sin(t2);
    GH[1][1] = cos(t1)*cos(t2);
    GH[1][2] = sin(t1)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
    GH[1][3] = Hy;

    GH[2][0] = -cos(t2)*sin(t3);
    GH[2][1] = sin(t2);
    GH[2][2] = cos(t2)*cos(t3);
    GH[2][3] = Hz;

    GH[3][0] = 0.0f;
    GH[3][1] = 0.0f;
    GH[3][2] = 0.0f;
    GH[3][3] = 1.0f;
    F[0][0] = 1.0f;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[0][3] = L;

    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[1][3] = 0.0f;

    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;
    F[2][3] = 0.0f;

    F[3][0] = 0.0f;
    F[3][1] = 0.0f;
    F[3][2] = 0.0f;
    F[3][3] = 1.0f;


    kx = GH[0][1]; ky = GH[1][1]; kz = GH[2][1];

    return RB_SUCCESS;
}


// Heel Pitching Trajectory Method------------------------------
int	TrajectoryHeelPitching_pitch::CalculateParameter(){
//    double polyTemp1, polyTemp2;
//    polyTemp1 = goalRotation;

//    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
//    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
//    trajParam[2] = currentVelocity;
//    trajParam[3] = currentPosition;


    double  t1 = Hyaw*D2Rf, t2 = Hroll*D2Rf, t3 = Hpitch*D2Rf;
L = 0.12f;
//    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
//    GH[0][1] = sin(t1)*cos(t2);
//    GH[0][2] = cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2);
//    GH[0][3] = Hx;

//    GH[1][0] = -sin(t1)*cos(t3)-sin(t3)*cos(t1)*sin(t2);
//    GH[1][1] = cos(t1)*cos(t2);
//    GH[1][2] = -sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3);
//    GH[1][3] = Hy;

//    GH[2][0] = -cos(t2)*sin(t3);
//    GH[2][1] = -sin(t2);
//    GH[2][2] = cos(t2)*cos(t3);
//    GH[2][3] = Hz;

//    GH[3][0] = 0.0f;
//    GH[3][1] = 0.0f;
//    GH[3][2] = 0.0f;
//    GH[3][3] = 1.0f;



    GH[0][0] = cos(t1)*cos(t3)-sin(t3)*sin(t1)*sin(t2);
    GH[0][1] = -sin(t1)*cos(t2);
    GH[0][2] = cos(t1)*sin(t3)+cos(t3)*sin(t1);
    GH[0][3] = Hx;

    GH[1][0] = sin(t1)*cos(t3)+sin(t3)*cos(t1)*sin(t2);
    GH[1][1] = cos(t1)*cos(t2);
    GH[1][2] = sin(t1)*sin(t3)-cos(t1)*sin(t2)*cos(t3);
    GH[1][3] = Hy;

    GH[2][0] = -cos(t2)*sin(t3);
    GH[2][1] = sin(t2);
    GH[2][2] = cos(t2)*cos(t3);
    GH[2][3] = Hz;

    GH[3][0] = 0.0f;
    GH[3][1] = 0.0f;
    GH[3][2] = 0.0f;
    GH[3][3] = 1.0f;




    F[0][0] = 1.0f;
    F[0][1] = 0.0f;
    F[0][2] = 0.0f;
    F[0][3] = L;

    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = 0.0f;
    F[1][3] = 0.0f;

    F[2][0] = 0.0f;
    F[2][1] = 0.0f;
    F[2][2] = 1.0f;
    F[2][3] = 0.0f;

    F[3][0] = 0.0f;
    F[3][1] = 0.0f;
    F[3][2] = 0.0f;
    F[3][3] = 1.0f;


    kx = GH[0][1]; ky = GH[1][1]; kz = GH[2][1];

    return RB_SUCCESS;
}


doubles TrajectoryHeelPitching_x::CalculateTrajectory(double _nTime){
//    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
//                        + trajParam[1]*_nTime*_nTime
//                        + trajParam[2]*_nTime
//                        + trajParam[3];
//    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
//                        + 2.0*trajParam[1]*_nTime
//                        + trajParam[2];
//    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];

//Heel to Footfrime(origin same) frame for equivalant rotation about heel y axis

    theta = (goalRotation*0.5*(1.0f-cos(PIf*_nTime)))*D2Rf;

    v= 1.0f-cos(theta);

    HF[0][0] = kx*kx*v+cos(theta);
    HF[0][1] = kx*ky*v-kz*sin(theta);
    HF[0][2] = kx*kz*v+ky*sin(theta);
    HF[0][3] = 0.0f;

    HF[1][0] = kx*ky*v+kz*sin(theta);
    HF[1][1] = ky*ky*v+cos(theta);
    HF[1][2] = ky*kz*v-kx*sin(theta);
    HF[1][3] = 0.0f;

    HF[2][0] = kx*kz*v-ky*sin(theta);
    HF[2][1] = ky*kz*v+kx*sin(theta);
    HF[2][2] = kz*kz*v+cos(theta);
    HF[2][3] = 0.0f;

    HF[3][0] = 0.0f;
    HF[3][1] = 0.0f;
    HF[3][2] = 0.0f;
    HF[3][3] = 1.0f;

    double sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + GH[i][j]*HF[j][k];
            }

            Temp[i][k] = sum ;
            sum = 0.0f;
        }

    }

    sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + Temp[i][j]*F[j][k];
            }

            GF[i][k] = sum ;
            sum = 0.0f;
        }

    }

    retData[0] = currentPosition = GF[0][3];
    retData[1] = currentVelocity = 0.0f;
    retData[2] = currentAcceleration = 0.0f;

//    retData[1] = currentPosition_y = GF[1][3];
//    retData[2] = currentPosition_z = GF[2][3];
//    retData[3] = currentOrientation_yaw = atan2(GF[0][1],GF[1][1])*R2Df;
//    retData[4] = currentOrientation_roll = asin(-GF[2][1])*R2Df;
//    retData[5] = currentOrientation_pitch = atan2(-GF[2][0],GF[2][2])*R2Df;

    return retData;
}


doubles TrajectoryHeelPitching_y::CalculateTrajectory(double _nTime){
//    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
//                        + trajParam[1]*_nTime*_nTime
//                        + trajParam[2]*_nTime
//                        + trajParam[3];
//    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
//                        + 2.0*trajParam[1]*_nTime
//                        + trajParam[2];
//    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];

//Heel to Footfrime(origin same) frame for equivalant rotation about heel y axis

    theta = (goalRotation*0.5*(1.0-cos(PIf*_nTime)))*D2Rf;



    v= 1.0f-cos(theta);

    HF[0][0] = kx*kx*v+cos(theta);
    HF[0][1] = kx*ky*v-kz*sin(theta);
    HF[0][2] = kx*kz*v+ky*sin(theta);
    HF[0][3] = 0.0f;

    HF[1][0] = kx*ky*v+kz*sin(theta);
    HF[1][1] = ky*ky*v+cos(theta);
    HF[1][2] = ky*kz*v-kx*sin(theta);
    HF[1][3] = 0.0f;

    HF[2][0] = kx*kz*v-ky*sin(theta);
    HF[2][1] = ky*kz*v+kx*sin(theta);
    HF[2][2] = kz*kz*v+cos(theta);
    HF[2][3] = 0.0f;

    HF[3][0] = 0.0f;
    HF[3][1] = 0.0f;
    HF[3][2] = 0.0f;
    HF[3][3] = 1.0f;

    double sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + GH[i][j]*HF[j][k];
            }

            Temp[i][k] = sum ;
            sum = 0.0f;
        }

    }

    sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + Temp[i][j]*F[j][k];
            }

            GF[i][k] = sum ;
            sum = 0.0f;
        }

    }

    retData[0] = currentPosition = GF[1][3];
    retData[1] = currentVelocity = 0.0f;
    retData[2] = currentAcceleration = 0.0f;

//    retData[1] = currentPosition_y = GF[1][3];
//    retData[2] = currentPosition_z = GF[2][3];
//    retData[3] = currentOrientation_yaw = atan2(GF[0][1],GF[1][1])*R2Df;
//    retData[4] = currentOrientation_roll = asin(-GF[2][1])*R2Df;
//    retData[5] = currentOrientation_pitch = atan2(-GF[2][0],GF[2][2])*R2Df;

    return retData;
}


doubles TrajectoryHeelPitching_z::CalculateTrajectory(double _nTime){
//    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
//                        + trajParam[1]*_nTime*_nTime
//                        + trajParam[2]*_nTime
//                        + trajParam[3];
//    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
//                        + 2.0*trajParam[1]*_nTime
//                        + trajParam[2];
//    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];

//Heel to Footfrime(origin same) frame for equivalant rotation about heel y axis

    theta = (goalRotation*0.5*(1.0-cos(PIf*_nTime)))*D2Rf;


    v= 1.0f-cos(theta);

    HF[0][0] = kx*kx*v+cos(theta);
    HF[0][1] = kx*ky*v-kz*sin(theta);
    HF[0][2] = kx*kz*v+ky*sin(theta);
    HF[0][3] = 0.0f;

    HF[1][0] = kx*ky*v+kz*sin(theta);
    HF[1][1] = ky*ky*v+cos(theta);
    HF[1][2] = ky*kz*v-kx*sin(theta);
    HF[1][3] = 0.0f;

    HF[2][0] = kx*kz*v-ky*sin(theta);
    HF[2][1] = ky*kz*v+kx*sin(theta);
    HF[2][2] = kz*kz*v+cos(theta);
    HF[2][3] = 0.0f;

    HF[3][0] = 0.0f;
    HF[3][1] = 0.0f;
    HF[3][2] = 0.0f;
    HF[3][3] = 1.0f;

    double sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + GH[i][j]*HF[j][k];
            }

            Temp[i][k] = sum ;
            sum = 0.0f;
        }

    }

    sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + Temp[i][j]*F[j][k];
            }

            GF[i][k] = sum ;
            sum = 0.0f;
        }

    }

    retData[0] = currentPosition = GF[2][3];
    retData[1] = currentVelocity = 0.0f;
    retData[2] = currentAcceleration = 0.0f;

//    retData[1] = currentPosition_y = GF[1][3];
//    retData[2] = currentPosition_z = GF[2][3];
//    retData[3] = currentOrientation_yaw = atan2(GF[0][1],GF[1][1])*R2Df;
//    retData[4] = currentOrientation_roll = asin(-GF[2][1])*R2Df;
//    retData[5] = currentOrientation_pitch = atan2(-GF[2][0],GF[2][2])*R2Df;

    return retData;
}



doubles TrajectoryHeelPitching_yaw::CalculateTrajectory(double _nTime){
//    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
//                        + trajParam[1]*_nTime*_nTime
//                        + trajParam[2]*_nTime
//                        + trajParam[3];
//    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
//                        + 2.0*trajParam[1]*_nTime
//                        + trajParam[2];
//    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];

//Heel to Footfrime(origin same) frame for equivalant rotation about heel y axis

    theta = (goalRotation*0.5*(1.0-cos(PIf*_nTime)))*D2Rf;

    v= 1.0f-cos(theta);

    HF[0][0] = kx*kx*v+cos(theta);
    HF[0][1] = kx*ky*v-kz*sin(theta);
    HF[0][2] = kx*kz*v+ky*sin(theta);
    HF[0][3] = 0.0f;

    HF[1][0] = kx*ky*v+kz*sin(theta);
    HF[1][1] = ky*ky*v+cos(theta);
    HF[1][2] = ky*kz*v-kx*sin(theta);
    HF[1][3] = 0.0f;

    HF[2][0] = kx*kz*v-ky*sin(theta);
    HF[2][1] = ky*kz*v+kx*sin(theta);
    HF[2][2] = kz*kz*v+cos(theta);
    HF[2][3] = 0.0f;

    HF[3][0] = 0.0f;
    HF[3][1] = 0.0f;
    HF[3][2] = 0.0f;
    HF[3][3] = 1.0f;

    double sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + GH[i][j]*HF[j][k];
            }

            Temp[i][k] = sum ;
            sum = 0.0f;
        }

    }

    sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + Temp[i][j]*F[j][k];
            }

            GF[i][k] = sum ;
            sum = 0.0f;
        }

    }

    retData[0] = currentPosition = atan2(-GF[0][1],GF[1][1])*R2Df;
    retData[1] = currentVelocity = 0.0f;
    retData[2] = currentAcceleration = 0.0f;

//    retData[1] = currentPosition_y = GF[1][3];
//    retData[2] = currentPosition_z = GF[2][3];
//    retData[3] = currentOrientation_yaw = atan2(GF[0][1],GF[1][1])*R2Df;
//    retData[4] = currentOrientation_roll = asin(-GF[2][1])*R2Df;
//    retData[5] = currentOrientation_pitch = atan2(-GF[2][0],GF[2][2])*R2Df;

    return retData;
}



doubles TrajectoryHeelPitching_roll::CalculateTrajectory(double _nTime){
//    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
//                        + trajParam[1]*_nTime*_nTime
//                        + trajParam[2]*_nTime
//                        + trajParam[3];
//    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
//                        + 2.0*trajParam[1]*_nTime
//                        + trajParam[2];
//    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];

//Heel to Footfrime(origin same) frame for equivalant rotation about heel y axis

    theta = (goalRotation*0.5*(1.0-cos(PIf*_nTime)))*D2Rf;


    v= 1.0f-cos(theta);

    HF[0][0] = kx*kx*v+cos(theta);
    HF[0][1] = kx*ky*v-kz*sin(theta);
    HF[0][2] = kx*kz*v+ky*sin(theta);
    HF[0][3] = 0.0f;

    HF[1][0] = kx*ky*v+kz*sin(theta);
    HF[1][1] = ky*ky*v+cos(theta);
    HF[1][2] = ky*kz*v-kx*sin(theta);
    HF[1][3] = 0.0f;

    HF[2][0] = kx*kz*v-ky*sin(theta);
    HF[2][1] = ky*kz*v+kx*sin(theta);
    HF[2][2] = kz*kz*v+cos(theta);
    HF[2][3] = 0.0f;

    HF[3][0] = 0.0f;
    HF[3][1] = 0.0f;
    HF[3][2] = 0.0f;
    HF[3][3] = 1.0f;

    double sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + GH[i][j]*HF[j][k];
            }

            Temp[i][k] = sum ;
            sum = 0.0f;
        }

    }

    sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + Temp[i][j]*F[j][k];
            }

            GF[i][k] = sum ;
            sum = 0.0f;
        }

    }

    retData[0] = currentPosition = asin(GF[2][1])*R2Df;
    retData[1] = currentVelocity = 0.0f;
    retData[2] = currentAcceleration = 0.0f;

//    retData[1] = currentPosition_y = GF[1][3];
//    retData[2] = currentPosition_z = GF[2][3];
//    retData[3] = currentOrientation_yaw = atan2(GF[0][1],GF[1][1])*R2Df;
//    retData[4] = currentOrientation_roll = asin(-GF[2][1])*R2Df;
//    retData[5] = currentOrientation_pitch = atan2(-GF[2][0],GF[2][2])*R2Df;

    return retData;
}


doubles TrajectoryHeelPitching_pitch::CalculateTrajectory(double _nTime){
//    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
//                        + trajParam[1]*_nTime*_nTime
//                        + trajParam[2]*_nTime
//                        + trajParam[3];
//    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
//                        + 2.0*trajParam[1]*_nTime
//                        + trajParam[2];
//    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];

//Heel to Footfrime(origin same) frame for equivalant rotation about heel y axis

    theta = (goalRotation*0.5*(1.0-cos(PIf*_nTime)))*D2Rf;


    v= 1.0f-cos(theta);

    HF[0][0] = kx*kx*v+cos(theta);
    HF[0][1] = kx*ky*v-kz*sin(theta);
    HF[0][2] = kx*kz*v+ky*sin(theta);
    HF[0][3] = 0.0f;

    HF[1][0] = kx*ky*v+kz*sin(theta);
    HF[1][1] = ky*ky*v+cos(theta);
    HF[1][2] = ky*kz*v-kx*sin(theta);
    HF[1][3] = 0.0f;

    HF[2][0] = kx*kz*v-ky*sin(theta);
    HF[2][1] = ky*kz*v+kx*sin(theta);
    HF[2][2] = kz*kz*v+cos(theta);
    HF[2][3] = 0.0f;

    HF[3][0] = 0.0f;
    HF[3][1] = 0.0f;
    HF[3][2] = 0.0f;
    HF[3][3] = 1.0f;

    double sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + GH[i][j]*HF[j][k];
            }

            Temp[i][k] = sum ;
            sum = 0.0f;
        }

    }

    sum = 0.0f;

    for(int i=0;i<=3;i++)
    {
        for(int k = 0;k<=3;k++)
        {
            for(int j=0;j<=3;j++)
            {
                sum = sum + Temp[i][j]*F[j][k];
            }

            GF[i][k] = sum ;
            sum = 0.0f;
        }

    }

    retData[0] = currentPosition = atan2(-GF[2][0],GF[2][2])*R2Df;
    retData[1] = currentVelocity = 0.0f;
    retData[2] = currentAcceleration = 0.0f;

//    retData[1] = currentPosition_y = GF[1][3];
//    retData[2] = currentPosition_z = GF[2][3];
//    retData[3] = currentOrientation_yaw = atan2(GF[0][1],GF[1][1])*R2Df;
//    retData[4] = currentOrientation_roll = asin(-GF[2][1])*R2Df;
//    retData[5] = currentOrientation_pitch = atan2(-GF[2][0],GF[2][2])*R2Df;

    return retData;
}




int	TrajectoryConst::CalculateParameter(){
    return RB_SUCCESS;
}
doubles TrajectoryConst::CalculateTrajectory(double _nTime){
    currentVelocity = 0.0;
    currentAcceleration = 0.0;
    retData[0] = currentPosition;
    retData[1] = currentVelocity;
    retData[2] = currentAcceleration;
    return retData;
}

// 3rd Polynomial Trajectory Method------------------------------
int	TrajectoryPoly3rd::CalculateParameter(){
    double polyTemp1, polyTemp2;
    polyTemp1 = goalPosition - currentVelocity - currentPosition;
    polyTemp2 = goalVelocity - currentVelocity;

    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
    trajParam[2] = currentVelocity;
    trajParam[3] = currentPosition;
    return RB_SUCCESS;
}
doubles TrajectoryPoly3rd::CalculateTrajectory(double _nTime){
    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
                        + trajParam[1]*_nTime*_nTime
                        + trajParam[2]*_nTime
                        + trajParam[3];
    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
                        + 2.0*trajParam[1]*_nTime
                        + trajParam[2];
    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];
    retData[0] = currentPosition;
    retData[1] = currentVelocity;
    retData[2] = currentAcceleration;
    return retData;
}

// 5th Polynomial Trajectory Method------------------------------
int TrajectoryPoly5th::CalculateParameter(){
    double polyTemp1, polyTemp2, polyTemp3;
    polyTemp1 = goalPosition - 0.5*currentAcceleration - currentVelocity - currentPosition;
    polyTemp2 = goalVelocity - currentAcceleration - currentVelocity;
    polyTemp3 = goalAcceleration - currentAcceleration;

    trajParam[0] = 0.5*(polyTemp3 - 6.0*polyTemp2 + 12.0*polyTemp1);
    trajParam[1] = polyTemp2 - 3.0*polyTemp1 - 2.0*trajParam[0];
    trajParam[2] = polyTemp1 - trajParam[0] - trajParam[1];
    trajParam[3] = 0.5*currentAcceleration;
    trajParam[4] = currentVelocity;
    trajParam[5] = currentPosition;
    return RB_SUCCESS;
}
doubles TrajectoryPoly5th::CalculateTrajectory(double _nTime){
    currentPosition = trajParam[0]*_nTime*_nTime*_nTime*_nTime*_nTime
                        + trajParam[1]*_nTime*_nTime*_nTime*_nTime
                        + trajParam[2]*_nTime*_nTime*_nTime
                        + trajParam[3]*_nTime*_nTime
                        + trajParam[4]*_nTime
                        + trajParam[5];
    currentVelocity = 5.0*trajParam[0]*_nTime*_nTime*_nTime*_nTime
                        + 4.0*trajParam[1]*_nTime*_nTime*_nTime
                        + 3.0*trajParam[2]*_nTime*_nTime
                        + 2.0*trajParam[3]*_nTime
                        + trajParam[4];
    currentAcceleration = 20.0*trajParam[0]*_nTime*_nTime*_nTime
                        + 12.0*trajParam[1]*_nTime*_nTime
                        + 6.0*trajParam[2]*_nTime
                        + 2.0*trajParam[3];

    retData[0] = currentPosition;
    retData[1] = currentVelocity;
    retData[2] = currentAcceleration;
    return retData;
}

// Cosine Trajectory Method---------------------------------------
int TrajectoryCosine::CalculateParameter(){
    trajParam[0] = currentPosition;
    trajParam[1] = goalPosition - currentPosition;
    return RB_SUCCESS;
}
doubles TrajectoryCosine::CalculateTrajectory(double _nTime){
    currentPosition = trajParam[0] + trajParam[1]*0.5*(1.0-cos(PIf*_nTime));
    currentVelocity = 0.5*trajParam[1]*PIf*sin(PIf*_nTime);
    currentAcceleration = 0.5*trajParam[1]*PIf*PIf*cos(PIf*_nTime);
    retData[0] = currentPosition;
    retData[1] = currentVelocity;
    retData[2] = currentAcceleration;
    return retData;
}



//===================================================================================
//===================================================================================
// Quat Order Trajectory
//===================================================================================
//===================================================================================
int TRInfoOrderQuat::GetCurrentValue(TRInfo _info){
    currentQuat = ((TRInfoOrderQuat*)_info)->getCurrentQuat();
    return RB_SUCCESS;
}
int TRInfoOrderQuat::GetCurrentValue(doubles _retVal){	//this function will be called in special cases
                                                        //when there is no preceeding trajectory information
    currentQuat = _retVal;
    return RB_SUCCESS;
}

// Slerp with Exponential---------------------------------------
int TrajectorySlerpExp::CalculateParameter(){
    quat q0,q1,qtemp;
    startQuat = currentQuat;
    q0.w=currentQuat[0];	q0.x=currentQuat[1];	q0.y=currentQuat[2];	q0.z=currentQuat[3];
    q1.w=goalQuat[0];		q1.x=goalQuat[1];		q1.y=goalQuat[2];		q1.z=goalQuat[3];
    q0 = unit(q0);
    q1 = unit(q1);
    inverse(q0);
    qtemp = q0*q1;

    trajParam[0] = acos(qtemp.w);
    trajParam[1] = qtemp.x/sin(trajParam[0]);
    trajParam[2] = qtemp.y/sin(trajParam[0]);
    trajParam[3] = qtemp.z/sin(trajParam[0]);
    return RB_SUCCESS;
}
doubles TrajectorySlerpExp::CalculateTrajectory(double _nTime){
    quat q0,q1,qtemp;
    q0.w=startQuat[0];	q0.x=startQuat[1];	q0.y=startQuat[2];	q0.z=startQuat[3];
    q1.w = cos(_nTime*trajParam[0]);
    q1.x = trajParam[1]*sin(_nTime*trajParam[0]);
    q1.y = trajParam[2]*sin(_nTime*trajParam[0]);
    q1.z = trajParam[3]*sin(_nTime*trajParam[0]);
    qtemp = q0*q1;
    retData[0] = qtemp.w;
    retData[1] = qtemp.x;
    retData[2] = qtemp.y;
    retData[3] = qtemp.z;
    return retData;
}

// Slerp without Exponential-------------------------------------
int TrajectorySlerpNoExp::CalculateParameter(){
    quat q0,q1;
    double ftemp;
    startQuat = currentQuat;
    q0.w=currentQuat[0];	q0.x=currentQuat[1];	q0.y=currentQuat[2];	q0.z=currentQuat[3];
    q1.w=goalQuat[0];		q1.x=goalQuat[1];		q1.y=goalQuat[2];		q1.z=goalQuat[3];
    q0 = unit(q0);
    q1 = unit(q1);
    ftemp = dot(q0,q1);

    trajParam = acos(ftemp);
    return RB_SUCCESS;
}
doubles TrajectorySlerpNoExp::CalculateTrajectory(double _nTime){
    quat q0,q1,qtemp;
    double ftemp1,ftemp2;
    q0.w=startQuat[0];	q0.x=startQuat[1];	q0.y=startQuat[2];	q0.z=startQuat[3];
    q1.w=goalQuat[0];	q1.x=goalQuat[1];	q1.y=goalQuat[2];	q1.z=goalQuat[3];

    ftemp1 = sin((1.0-_nTime)*trajParam);
    ftemp2 = sin(_nTime*trajParam);

    qtemp.w = (q0.w*ftemp1 + q1.w*ftemp2)/sin(trajParam);
    qtemp.x = (q0.x*ftemp1 + q1.x*ftemp2)/sin(trajParam);
    qtemp.y = (q0.y*ftemp1 + q1.y*ftemp2)/sin(trajParam);
    qtemp.z = (q0.z*ftemp1 + q1.z*ftemp2)/sin(trajParam);

    retData[0] = qtemp.w;
    retData[1] = qtemp.x;
    retData[2] = qtemp.y;
    retData[3] = qtemp.z;
    return retData;
}



//===================================================================================
//===================================================================================
// Trajectory Controlling
//===================================================================================
//===================================================================================

// Constructor------------------------------------
TrajectoryHandler::TrajectoryHandler(int _order, double _tick){
    trajectoryOrder = _order;
    tickTime = _tick;
    trInfos.clear();
    currentInfo = NULL;
    AllocateRetValue();
}

void TrajectoryHandler::AllocateRetValue(){
    switch(trajectoryOrder){
    case ORDER_1:
        AllocateData(retValue, 3);
        break;
    case ORDER_2:
        AllocateData(retValue, 2);
        break;
    case ORDER_QUAT:
        AllocateData(retValue, 4);
        break;
    case ORDER_COM:
        AllocateData(retValue, 2);
        break;
    case ORDER_6:
        AllocateData(retValue, 6);
        break;

    }
}

// Control Method for Trajectory Information-------------------
int TrajectoryHandler::AddTrajInfo(TRInfo _info){
    if(_info->trajectoryOrder != trajectoryOrder)
        return RB_FAIL;
    trInfos.push_back(_info);
    return RB_SUCCESS;
}
int TrajectoryHandler::InsertTrajInfo(TRInfo _info, int n){
    if(_info->trajectoryOrder != trajectoryOrder)
        return RB_FAIL;
    trInfos.insert(n, _info);
    return RB_SUCCESS;
}
int TrajectoryHandler::DeleteTrajInfo(int n){
    TRInfo tempInfo = trInfos.at(n);
    trInfos.remove(n);
    delete tempInfo;
    return RB_SUCCESS;
}
int TrajectoryHandler::OverwriteTrajInfo(TRInfo _info){
    if(_info->trajectoryOrder != trajectoryOrder)
        return RB_FAIL;
    if(currentInfo == NULL){
        currentInfo = _info;
        currentInfo->GetCurrentValue(retValue);
        currentInfo->CalculateParameter();
        TimeReset();
    }else{
        TRInfo tempInfo = currentInfo;
        currentInfo = _info;
        currentInfo->GetCurrentValue(tempInfo);
        currentInfo->CalculateParameter();
        TimeReset();
        delete tempInfo;
    }
    return RB_SUCCESS;
}

// Update the Trajectory-----------------------------------------
doubles TrajectoryHandler::UpdateTrajectory(){
    if(currentInfo == NULL){
        if(!trInfos.empty()){
            currentInfo = trInfos.first();
            currentInfo->GetCurrentValue(retValue);
            trInfos.pop_front();
            currentInfo->CalculateParameter();
            TimeReset();
            return UpdateTrajectory();
        }
    }else if(normalizedTime < 1.0){
        currentTime += tickTime;
        normalizedTime = currentTime/currentInfo->goalTime;
        if(normalizedTime > 1.0){
            normalizedTime = 1.0;
            retValue = currentInfo->CalculateTrajectory(normalizedTime);
            return UpdateTrajectory();
        }
        retValue = currentInfo->CalculateTrajectory(normalizedTime);
    }else{
        retValue = currentInfo->retData;
        if(!trInfos.empty()){
            TRInfo tempInfo = currentInfo;
            currentInfo = trInfos.first();
            currentInfo->GetCurrentValue(tempInfo);
            trInfos.pop_front();
            currentInfo->CalculateParameter();
            TimeReset();
            delete tempInfo;
            return UpdateTrajectory();
        }else currentInfo = NULL;
    }
    return retValue;
}

void    TrajectoryHandler::StopAndEraseAll(){
    trInfos.clear();
    currentInfo = NULL;
}

}//end namespace rainbow


