


//#ifndef BASICTRAJECTORY_H
//#define BASICTRAJECTORY_H

//#include "BasicMatrix.h"

//namespace rainbow{
//const int RB_SUCCESS = true;
//const int RB_FAIL = false;

//const int ORDER_1		= 1;		// first order trajectory
//const int ORDER_2		= 2;		// second order trajectory
//const int ORDER_3		= 3;		// third order trajectory
//const int ORDER_QUAT	= 4;		// quaternion trajectory
//const int ORDER_COM		= 5;		// COM trajectory


//class TrajectoryInfo;
//class TrajectoryHandler;

//typedef TrajectoryInfo*				TRInfo;
//typedef TrajectoryHandler*			TRHandler;

//typedef QVector<TrajectoryInfo*>	TRInfos;
//typedef QVector<double>				doubles;
//typedef QVector<doubles>            doubless;
//typedef QVector<int>				ints;
//typedef QVector<ints>               intss;



//inline void AllocateData(doubles &vec, const int n){
//    vec.clear();
//    for(int i=0; i<n; i++) vec.push_back(0.0);
//}

//// Base Abstract Class for Various Trajecoties============================================================
//class TrajectoryInfo{
//    friend class TrajectoryHandler;
//protected:
//    int		trajectoryOrder;	// the order should be same within the sequence of trajectories
//    double	goalTime;
//    doubles	retData;

//    void	AllocateRetData(int n)				{AllocateData(retData, n);}
//public:
//    virtual int		CalculateParameter() = 0;					//pure virtual
//    virtual doubles	CalculateTrajectory(double _nTime) = 0;		//pure virtual
//    virtual int		GetCurrentValue(TRInfo _info) = 0;			//pure virtual
//    virtual int		GetCurrentValue(doubles _retVal) = 0;		//pure virtual
//};
////========================================================================================================


////========================================================================================================
//// 1st Order Trajectory Information---------------------------------------------------
//// now it demands current position, velocity, acceleration for the next trajectory
//class TRInfoOrder1 : public TrajectoryInfo
//{
//protected:
//    double currentPosition;
//    double currentVelocity;
//    double currentAcceleration;
//public:
//    TRInfoOrder1()								{trajectoryOrder = ORDER_1;	AllocateRetData(3); currentPosition = 0; currentVelocity = 0; currentAcceleration = 0;}
//    double			getCurrentPosition()		{return currentPosition;}
//    double			getCurrentVelocity()		{return currentVelocity;}
//    double			getCurrentAcceleration()	{return currentAcceleration;}

//    virtual int		GetCurrentValue(TRInfo _info);
//    virtual int		GetCurrentValue(doubles _retVal);
//};
//// Quat Order Trajectory Information---------------------------------------------------
//// now it demands current quaternion for the next trajectory
//class TRInfoOrderQuat : public TrajectoryInfo
//{
//protected:
//    doubles		currentQuat;
//public:
//    TRInfoOrderQuat()							{trajectoryOrder = ORDER_QUAT;	AllocateRetData(4);}
//    doubles			getCurrentQuat()			{return currentQuat;}

//    virtual int		GetCurrentValue(TRInfo _info);
//    virtual int		GetCurrentValue(doubles _retVal);
//};
//// COM Trajectory Information----------------------------------------------------------
//// now it demands current ZMP and COM for the next trajectory
//class COMTime{
//public:
//    doubles times;
//public:
//    COMTime()										{AllocateData(times, 3); times[0]=0.0; times[1]=0.0; times[2]=0.0;}
//    COMTime(double _t0, double _t1, double _t2)		{AllocateData(times, 3); times[0]=_t0; times[1]=_t1; times[2]=_t2;}
//    COMTime(doubles _time)							{times = _time;}
//    //using default copy constructor
//};
//class TRInfoCOM : public TrajectoryInfo
//{
//protected:
//    double	currentZMP;
//    double	currentCOM;

//    double	comHeight;
//    COMTime	comTime;
//public:
//    TRInfoCOM()								{trajectoryOrder = ORDER_COM;	AllocateRetData(2);}
//    double			getCurrentZMP()			{return currentZMP;}
//    double			getCurrentCOM()			{return currentCOM;}

//    virtual int		GetCurrentValue(TRInfo _info);
//    virtual int		GetCurrentValue(doubles _retVal);
//};
////========================================================================================================


////========================================================================================================
//// Constant Trajectory Info------------------------------
//class TrajectoryConst : public TRInfoOrder1{
//private:

//public:
//    TrajectoryConst(double _time)				{goalTime = _time;}

//    virtual int		CalculateParameter();
//    virtual doubles	CalculateTrajectory(double _nTime);
//};
//// 3rd Polynomial Trajectory Info------------------------------
//class TrajectoryPoly3rd : public TRInfoOrder1{
//private:
//    double	goalPosition;
//    double	goalVelocity;

//    double	trajParam[4];
//public:
//    TrajectoryPoly3rd(double _time, double _pos, double _vel)
//                                                {goalTime = _time;	goalPosition = _pos;	goalVelocity = _vel;}

//    virtual int		CalculateParameter();
//    virtual doubles	CalculateTrajectory(double _nTime);
//};
//// 5th Polynomial Trajectory Info------------------------------
//class TrajectoryPoly5th : public TRInfoOrder1{
//private:
//    double	goalPosition;
//    double	goalVelocity;
//    double	goalAcceleration;

//    double	trajParam[6];
//public:
//    TrajectoryPoly5th(double _time, double _pos, double _vel, double _acc)
//                                                {goalTime = _time;	goalPosition = _pos;	goalVelocity = _vel;	goalAcceleration = _acc;}

//    virtual int		CalculateParameter();
//    virtual doubles	CalculateTrajectory(double _nTime);
//};
//// Cosine Trajectory Info---------------------------------------
//class TrajectoryCosine : public TRInfoOrder1{
//private:
//    double	goalPosition;

//    double	trajParam[2];
//public:
//    TrajectoryCosine(double _time, double _pos)	{goalTime = _time;	goalPosition = _pos;}

//    virtual int		CalculateParameter();
//    virtual doubles	CalculateTrajectory(double _nTime);
//};
////========================================================================================================


////========================================================================================================
//// Slerp with Exponential Trajectory Info------------------------------
//class TrajectorySlerpExp : public TRInfoOrderQuat{
//private:
//    doubles	goalQuat;
//    doubles	startQuat;

//    double	trajParam[4];
//public:
//    TrajectorySlerpExp(double _time, doubles _quat)	{goalTime = _time;	goalQuat = _quat;	AllocateData(startQuat,4);}

//    virtual int		CalculateParameter();
//    virtual doubles	CalculateTrajectory(double _nTime);
//};
//// Slerp without Exponential Trajectory Info----------------------------
//class TrajectorySlerpNoExp : public TRInfoOrderQuat{
//private:
//    doubles	goalQuat;
//    doubles	startQuat;

//    double	trajParam;
//public:
//    TrajectorySlerpNoExp(double _time, doubles _quat)	{goalTime = _time;		goalQuat = _quat;}

//    virtual int		CalculateParameter();
//    virtual doubles	CalculateTrajectory(double _nTime);
//};
////========================================================================================================



////========================================================================================================
//// Handling Class for controlling TRInfos..
//class TrajectoryHandler
//{
//private:
//    int			trajectoryOrder;
//    double		tickTime;

//    double		currentTime;
//    double		normalizedTime;

//    TRInfos		trInfos;
//    TRInfo		currentInfo;
//    doubles		retValue;

//    void	AllocateRetValue();
//    void	TimeReset()				{currentTime = normalizedTime = 0.0;}

//public:
//    explicit TrajectoryHandler(int _order = ORDER_1, double _tick = 0.005);

//    // Control Method for Trajectory Information-------------
//    int		AddTrajInfo(TRInfo _info);
//    int		InsertTrajInfo(TRInfo _info, int n);
//    int		DeleteTrajInfo(int n);
//    int		OverwriteTrajInfo(TRInfo _info);

//    // Update the Trajectory---------------------------------
//    doubles	UpdateTrajectory();
//    void    StopAndEraseAll();

//    int		SetRetValue(doubles _ret)	{if(currentInfo != NULL){return RB_FAIL;}
//                                        retValue = _ret; return RB_SUCCESS;}
//    doubles	GetRetValue()				{return retValue;}
//};
////========================================================================================================



//}//end namespace rainbow

//#endif // BASICTRAJECTORY_H

















#ifndef BASICTRAJECTORY_H
#define BASICTRAJECTORY_H

#include "BasicMatrix.h"

namespace rainbow{
const int RB_SUCCESS = true;
const int RB_FAIL = false;

const int ORDER_1		= 1;		// first order trajectory
const int ORDER_2		= 2;		// second order trajectory
const int ORDER_3		= 3;		// third order trajectory
const int ORDER_QUAT	= 4;		// quaternion trajectory
const int ORDER_COM		= 5;		// COM trajectory
const int ORDER_6       = 6;


class TrajectoryInfo;
class TrajectoryHandler;

typedef TrajectoryInfo*				TRInfo;
typedef TrajectoryHandler*			TRHandler;

typedef QVector<TrajectoryInfo*>	TRInfos;
typedef QVector<double>				doubles;
typedef QVector<doubles>            doubless;
typedef QVector<int>				ints;
typedef QVector<ints>               intss;



inline void AllocateData(doubles &vec, const int n){
    vec.clear();
    for(int i=0; i<n; i++) vec.push_back(0.0);
}

// Base Abstract Class for Various Trajecoties============================================================
class TrajectoryInfo{
    friend class TrajectoryHandler;
protected:
    int		trajectoryOrder;	// the order should be same within the sequence of trajectories
    double	goalTime;
    doubles	retData;

    void	AllocateRetData(int n)				{AllocateData(retData, n);}
public:
    virtual int		CalculateParameter() = 0;					//pure virtual
    virtual doubles	CalculateTrajectory(double _nTime) = 0;		//pure virtual
    virtual int		GetCurrentValue(TRInfo _info) = 0;			//pure virtual
    virtual int		GetCurrentValue(doubles _retVal) = 0;		//pure virtual
};
//========================================================================================================


//========================================================================================================
// 1st Order Trajectory Information---------------------------------------------------
// now it demands current position, velocity, acceleration for the next trajectory
class TRInfoOrder1 : public TrajectoryInfo
{
protected:
    double currentPosition;
    double currentVelocity;
    double currentAcceleration;
public:
    TRInfoOrder1()								{trajectoryOrder = ORDER_1;	AllocateRetData(3); currentPosition = 0; currentVelocity = 0; currentAcceleration = 0;}
    double			getCurrentPosition()		{return currentPosition;}
    double			getCurrentVelocity()		{return currentVelocity;}
    double			getCurrentAcceleration()	{return currentAcceleration;}

    virtual int		GetCurrentValue(TRInfo _info);
    virtual int		GetCurrentValue(doubles _retVal);
};


//========================================================================================================
// 6st Order Trajectory Information---------------------------------------------------
// now it demands current position, velocity, acceleration for the next trajectory
class TRInfoOrder6 : public TrajectoryInfo
{
protected:
    double currentPosition_x;
    double currentPosition_y;
    double currentPosition_z;
    double currentOrientation_yaw;
    double currentOrientation_roll;
    double currentOrientation_pitch;
public:
//    TRInfoOrder6()								{trajectoryOrder = ORDER_6;	AllocateRetData(6); currentPosition[0] = 0; currentPosition[1] = 0; currentPosition[2] = 0; currentVelocity[0] = 0; currentVelocity[1] = 0; currentVelocity[2] = 0; currentAcceleration[0] = 0; currentAcceleration[1] = 0; currentAcceleration[2] = 0; currentOrientation[0] = 0; currentOrientation[1] = 0; currentOrientation[2] = 0;}
    TRInfoOrder6()								{trajectoryOrder = ORDER_6;	AllocateRetData(6); currentPosition_x = 0; currentPosition_y = 0; currentPosition_z = 0;  currentOrientation_yaw = 0; currentOrientation_roll = 0; currentOrientation_pitch = 0;}

    double			getCurrentPosition_x()		{return currentPosition_x;}
    double			getCurrentPosition_y()		{return currentPosition_y;}
    double			getCurrentPosition_z()      {return currentPosition_z;}

    double			getCurrentOrientation_yaw()		{return currentOrientation_yaw;}
    double			getCurrentOrientation_roll()		{return currentOrientation_roll;}
    double			getCurrentOrientation_pitch()      {return currentOrientation_pitch;}

    virtual int		GetCurrentValue(TRInfo _info);
    virtual int		GetCurrentValue(doubles _retVal);
};



// Quat Order Trajectory Information---------------------------------------------------
// now it demands current quaternion for the next trajectory
class TRInfoOrderQuat : public TrajectoryInfo
{
protected:
    doubles		currentQuat;
public:
    TRInfoOrderQuat()							{trajectoryOrder = ORDER_QUAT;	AllocateRetData(4);}
    doubles			getCurrentQuat()			{return currentQuat;}

    virtual int		GetCurrentValue(TRInfo _info);
    virtual int		GetCurrentValue(doubles _retVal);
};
// COM Trajectory Information----------------------------------------------------------
// now it demands current ZMP and COM for the next trajectory
class COMTime{
public:
    doubles times;
public:
    COMTime()										{AllocateData(times, 3); times[0]=0.0; times[1]=0.0; times[2]=0.0;}
    COMTime(double _t0, double _t1, double _t2)		{AllocateData(times, 3); times[0]=_t0; times[1]=_t1; times[2]=_t2;}
    COMTime(doubles _time)							{times = _time;}
    //using default copy constructor
};
class TRInfoCOM : public TrajectoryInfo
{
protected:
    double	currentZMP;
    double	currentCOM;

    double	comHeight;
    COMTime	comTime;
public:
    TRInfoCOM()								{trajectoryOrder = ORDER_COM;	AllocateRetData(2);}
    double			getCurrentZMP()			{return currentZMP;}
    double			getCurrentCOM()			{return currentCOM;}

    virtual int		GetCurrentValue(TRInfo _info);
    virtual int		GetCurrentValue(doubles _retVal);
};
//========================================================================================================


//========================================================================================================


// 6st Heel pitching Polynomial Trajectory Info------------------------------
class TrajectoryHeelPitching_x : public TRInfoOrder1{
private:
    double	goalRotation;
    double  Hx,Hy,Hz,Hyaw,Hroll,Hpitch;
    double	trajParam[6];
    double  kx,ky,kz,v;
    double  F[4][4],HF[4][4],GH[4][4],GF[4][4],Temp[4][4];
    double  L;
    double  theta;

public:
    TrajectoryHeelPitching_x(double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch)
                                                {goalTime = _time;	goalRotation = _rotang; Hx = _posx; Hy = _posy; Hz = _posz; Hyaw = _yaw; Hroll = _roll; Hpitch = _pitch;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};


// 6st Heel pitching Polynomial Trajectory Info------------------------------
class TrajectoryHeelPitching_y : public TRInfoOrder1{
private:
    double	goalRotation;
    double  Hx,Hy,Hz,Hyaw,Hroll,Hpitch;
    double	trajParam[6];
    double  kx,ky,kz,v;
    double  F[4][4],HF[4][4],GH[4][4],GF[4][4],Temp[4][4];
    double  L;
    double  theta;

public:
    TrajectoryHeelPitching_y(double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch)
                                                {goalTime = _time;	goalRotation = _rotang; Hx = _posx; Hy = _posy; Hz = _posz; Hyaw = _yaw; Hroll = _roll; Hpitch = _pitch;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};

// 6st Heel pitching Polynomial Trajectory Info------------------------------
class TrajectoryHeelPitching_z : public TRInfoOrder1{
private:
    double	goalRotation;
    double  Hx,Hy,Hz,Hyaw,Hroll,Hpitch;
    double	trajParam[6];
    double  kx,ky,kz,v;
    double  F[4][4],HF[4][4],GH[4][4],GF[4][4],Temp[4][4];
    double  L;
    double  theta;

public:
    TrajectoryHeelPitching_z(double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch)
                                                {goalTime = _time;	goalRotation = _rotang; Hx = _posx; Hy = _posy; Hz = _posz; Hyaw = _yaw; Hroll = _roll; Hpitch = _pitch;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};









// 6st Heel pitching Polynomial Trajectory Info------------------------------
class TrajectoryHeelPitching_yaw : public TRInfoOrder1{
private:
    double	goalRotation;
    double  Hx,Hy,Hz,Hyaw,Hroll,Hpitch;
    double	trajParam[6];
    double  kx,ky,kz,v;
    double  F[4][4],HF[4][4],GH[4][4],GF[4][4],Temp[4][4];
    double  L;
    double  theta;

public:
    TrajectoryHeelPitching_yaw(double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch)
                                                {goalTime = _time;	goalRotation = _rotang; Hx = _posx; Hy = _posy; Hz = _posz; Hyaw = _yaw; Hroll = _roll; Hpitch = _pitch;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};



// 6st Heel pitching Polynomial Trajectory Info------------------------------
class TrajectoryHeelPitching_pitch : public TRInfoOrder1{
private:
    double	goalRotation;
    double  Hx,Hy,Hz,Hyaw,Hroll,Hpitch;
    double	trajParam[6];
    double  kx,ky,kz,v;
    double  F[4][4],HF[4][4],GH[4][4],GF[4][4],Temp[4][4];
    double  L;
    double  theta;

public:
    TrajectoryHeelPitching_pitch(double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch)
                                                {goalTime = _time;	goalRotation = _rotang; Hx = _posx; Hy = _posy; Hz = _posz; Hyaw = _yaw; Hroll = _roll; Hpitch = _pitch;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};


// 6st Heel pitching Polynomial Trajectory Info------------------------------
class TrajectoryHeelPitching_roll : public TRInfoOrder1{
private:
    double	goalRotation;
    double  Hx,Hy,Hz,Hyaw,Hroll,Hpitch;
    double	trajParam[6];
    double  kx,ky,kz,v;
    double  F[4][4],HF[4][4],GH[4][4],GF[4][4],Temp[4][4];
    double  L;
    double  theta;

public:
    TrajectoryHeelPitching_roll(double _time, double _rotang,double _posx, double _posy, double _posz, double _yaw, double _roll, double _pitch)
                                                {goalTime = _time;	goalRotation = _rotang; Hx = _posx; Hy = _posy; Hz = _posz; Hyaw = _yaw; Hroll = _roll; Hpitch = _pitch;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
















// Constant Trajectory Info------------------------------
class TrajectoryConst : public TRInfoOrder1{
private:

public:
    TrajectoryConst(double _time)				{goalTime = _time;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// 3rd Polynomial Trajectory Info------------------------------
class TrajectoryPoly3rd : public TRInfoOrder1{
private:
    double	goalPosition;
    double	goalVelocity;

    double	trajParam[4];
public:
    TrajectoryPoly3rd(double _time, double _pos, double _vel)
                                                {goalTime = _time;	goalPosition = _pos;	goalVelocity = _vel;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// 5th Polynomial Trajectory Info------------------------------
class TrajectoryPoly5th : public TRInfoOrder1{
private:
    double	goalPosition;
    double	goalVelocity;
    double	goalAcceleration;
    double	trajParam[6];
public:
    TrajectoryPoly5th(double _time, double _pos, double _vel, double _acc)
                                                {goalTime = _time;	goalPosition = _pos;	goalVelocity = _vel;	goalAcceleration = _acc;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// Cosine Trajectory Info---------------------------------------
class TrajectoryCosine : public TRInfoOrder1{
private:
    double	goalPosition;

    double	trajParam[2];
public:
    TrajectoryCosine(double _time, double _pos)	{goalTime = _time;	goalPosition = _pos;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
//========================================================================================================


//========================================================================================================
// Slerp with Exponential Trajectory Info------------------------------
class TrajectorySlerpExp : public TRInfoOrderQuat{
private:
    doubles	goalQuat;
    doubles	startQuat;

    double	trajParam[4];
public:
    TrajectorySlerpExp(double _time, doubles _quat)	{goalTime = _time;	goalQuat = _quat;	AllocateData(startQuat,4);}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// Slerp without Exponential Trajectory Info----------------------------
class TrajectorySlerpNoExp : public TRInfoOrderQuat{
private:
    doubles	goalQuat;
    doubles	startQuat;

    double	trajParam;
public:
    TrajectorySlerpNoExp(double _time, doubles _quat)	{goalTime = _time;		goalQuat = _quat;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
//========================================================================================================



//========================================================================================================
// Handling Class for controlling TRInfos..
class TrajectoryHandler
{
private:
    int			trajectoryOrder;
    double		tickTime;

    double		currentTime;
    double		normalizedTime;

    TRInfos		trInfos;
    TRInfo		currentInfo;
    doubles		retValue;

    void	AllocateRetValue();
    void	TimeReset()				{currentTime = normalizedTime = 0.0;}

public:
    explicit TrajectoryHandler(int _order = ORDER_1, double _tick = 0.005);

    // Control Method for Trajectory Information-------------
    int		AddTrajInfo(TRInfo _info);
    int		InsertTrajInfo(TRInfo _info, int n);
    int		DeleteTrajInfo(int n);
    int		OverwriteTrajInfo(TRInfo _info);

    // Update the Trajectory---------------------------------
    doubles	UpdateTrajectory();
    void    StopAndEraseAll();

    int		SetRetValue(doubles _ret)	{if(currentInfo != NULL){return RB_FAIL;}
                                        retValue = _ret; return RB_SUCCESS;}
    doubles	GetRetValue()				{return retValue;}
};
//========================================================================================================



}//end namespace rainbow

#endif // BASICTRAJECTORY_H
