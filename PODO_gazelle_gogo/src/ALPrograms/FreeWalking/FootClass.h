

//#ifndef FOOTCLASS_H
//#define FOOTCLASS_H

//#include "BasicTrajectory.h"

//using namespace rainbow;

//typedef QVector<TRHandler>	TRHandlers;

//typedef enum{
//    JRHY = 0, JRHR, JRHP, JRKN, JRAP, JRAR,
//    JLHY, JLHR, JLHP, JLKN, JLAP, JLAR,
//    JRSP, JRSR, JRSY, JREB, JRWY, JRWP,
//    JLSP, JLSR, JLSY, JLEB, JLWY, JLWP,
//    JLKY, JNK1, JNK2, JWST,
//    JRF1, JRF2, JRF3, JLF1, JLF2, JLF3,
//    JNUM
//} JOINTNAME;

//class FootClass{
//public:
//    FootClass(double _tick = 0.005);

//    doubles		Pos;

//    //---foot velocity
//    doubles     Vel;

//    double		Yaw;

//    //---
//    double      Pitch;
//    double      Roll;


//    TRHandlers	PosHandler;
//    TRHandler	YawHandler;


//    //---foot velocity handler
//    TRHandlers   VelHandler;

//    //---
//    TRHandler	RollHandler;
//    TRHandler	PitchHandler;


//public:
//    void	RefreshData();
//    void	UpdateData();
//    void	UpdateData2();

//    int		SetInitialPosition(const double x, const double y, const double z  = 0.0, const double yaw = 0.0, const double roll = 0.0, const double pitch = 0.0);
//    int		SetInitialOrientation(const double qw, const double qx, const double qy, const double qz);

//    // Add trajectory information (fifth polynomial, const)
//    void	AddXFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddXFifth(const double _t);	//const
//    void	AddXCos(const double _pos, const double _t);
//    void	AddXCos(const double _t);	//const
//    void	AddYFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddYFifth(const double _t);	//const
//    void	AddYCos(const double _pos, const double _t);
//    void	AddYCos(const double _t);	//const
//    void	AddYThird(const double _pos, const double _t, const double _vel = 0.0);
//    void	AddYThird(const double _t);	//const
//    void	AddZFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddZFifth(const double _t);	//const
//    void	AddZCos(const double _pos, const double _t);
//    void	AddZCos(const double _t);	//const
//    void	AddYawFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddYawFifth(const double _t);	//const

//    void	AddPitchFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddPitchFifth(const double _t);	//const
//    void	AddPitchCos(const double _pos, const double _t);
//    void	AddPitchCos(const double _t);	//const


//    void	AddRollFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddRollFifth(const double _t);	//const
//    void	AddRollCos(const double _pos, const double _t);
//    void	AddRollCos(const double _t);	//const


//};

//class ZMPClass{
//public:
//    ZMPClass(double _tick = 0.005);

//    doubles		ZMP;
//    TRHandlers	ZMPHandler;
//public:
//    void	RefreshData();
//    void	UpdateData();

//    int		SetInitialZMP(const double xzmp, const double yzmp);

//    // Add trajectory information (fifth polynomial, const)
//    void	AddXZMPFifth(const double _xzmp, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddXZMPFifth(const double _t);	//const

//    void	AddYZMPFifth(const double _xzmp, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void	AddYZMPFifth(const double _t);	//const
//};



//class AddCompClass{
//public:
//    AddCompClass(double _tick = 0.005);

//    doubles     AddJoint;
//    doubles     AddCOM;
//    doubles     AddLeftFoot;
//    doubles     AddRightFoot;

//    TRHandlers  JointHandler;
//    TRHandlers  COMHandler;
//    TRHandlers  LFHandler;
//    TRHandlers  RFHandler;

//    //-------------add vel and acc
//    doubles     AddCOMVel;
//    doubles     AddCOMAcc;

//    TRHandlers  COMVelHandler;
//    TRHandlers  COMAccHandler;

//public:
//    void	RefreshData();
//    void	UpdateData();
//    int		SetInitialCOM(const double com1);

//    int		SetInitialAddJoint();
//    int     SetInitialAddCOM(doubles _com);
//    int     SetInitialLeftFoot(doubles _left);
//    int     SetInitialRightFoot(doubles _right);

//    void    AddJointsFifth(const int jointNum, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void    AddJointsFifth(const int jointNum, const double _t);
//    void    AddCOMFifth(const int xyz, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void    AddCOMFifth(const int xyz, const double _t);
//    void    AddLeftFootFifth(const int xyz, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void    AddLeftFootFifth(const int xyz, const double _t);
//    void    AddRightFootFifth(const int xyz, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
//    void    AddRightFootFifth(const int xyz, const double _t);

//    void    AddLeftFootCos(const int xyz, const double _des, const double _t);
//    void    AddLeftFootCos(const int xyz, const double _t);
//    void    AddRightFootCos(const int xyz, const double _des, const double _t);
//    void    AddRightFootCos(const int xyz, const double _t);

//    void    AddCOMCos(const int xyz, const double _des, const double _t);
//    void    AddCOMCos(const int xyz, const double _t);
//};


//#endif // FOOTCLASS_H





#ifndef FOOTCLASS_H
#define FOOTCLASS_H

#include "BasicTrajectory.h"

using namespace rainbow;

typedef QVector<TRHandler>	TRHandlers;

typedef enum{
    JRHY = 0, JRHR, JRHP, JRKN, JRAP, JRAR,
    JLHY, JLHR, JLHP, JLKN, JLAP, JLAR,
    JRSP, JRSR, JRSY, JREB, JRWY, JRWP,
    JLSP, JLSR, JLSY, JLEB, JLWY, JLWP,
    JLKY, JNK1, JNK2, JWST,
    JRF1, JRF2, JRF3, JLF1, JLF2, JLF3,
    JNUM
} JOINTNAME;

class FootClass{
public:
    FootClass(double _tick = 0.005);

    doubles		Pos;

    //---foot velocity
    doubles     Vel;

    double		Yaw;

    //---
    double      Pitch;
    double      Roll;


    TRHandlers	PosHandler;
    TRHandler	YawHandler;


    //---foot velocity handler
    TRHandlers   VelHandler;

    //---
    TRHandler	RollHandler;
    TRHandler	PitchHandler;


public:
    void	RefreshData();
    void	UpdateData();
    void	UpdateData2();

    int		SetInitialPosition(const double x, const double y, const double z  = 0.0, const double yaw = 0.0, const double roll = 0.0, const double pitch = 0.0);
    int		SetInitialOrientation(const double qw, const double qx, const double qy, const double qz);

    // Add trajectory information (fifth polynomial, const)
    void	AddXFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddXFifth(const double _t);	//const
    void	AddXCos(const double _pos, const double _t);
    void	AddXCos(const double _t);	//const
    void	AddYFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddYFifth(const double _t);	//const
    void	AddYCos(const double _pos, const double _t);
    void	AddYCos(const double _t);	//const
    void	AddYThird(const double _pos, const double _t, const double _vel = 0.0);
    void	AddYThird(const double _t);	//const
    void	AddZFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddZFifth(const double _t);	//const
    void	AddZCos(const double _pos, const double _t);
    void	AddZCos(const double _t);	//const
    void	AddYawFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddYawFifth(const double _t);	//const
    void    AddYawCos(const double _pos, const double _t);
    void    AddYawCos(const double _t);

    void	AddPitchFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddPitchFifth(const double _t);	//const
    void	AddPitchCos(const double _pos, const double _t);
    void	AddPitchCos(const double _t);	//const


    void	AddRollFifth(const double _pos, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddRollFifth(const double _t);	//const
    void	AddRollCos(const double _pos, const double _t);
    void	AddRollCos(const double _t);	//const

    void    AddHeelPitching_x(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch);

    void    AddHeelPitching_y(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch);

    void    AddHeelPitching_z(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch);

    void    AddHeelPitching_yaw(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch);

    void    AddHeelPitching_roll(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch);

    void    AddHeelPitching_pitch(const double _t,const double _rotang,const double _posx,const double _posy,const double _posz,const double _yaw,const double _roll,const double _pitch);



};

class ZMPClass{
public:
    ZMPClass(double _tick = 0.005);

    doubles		ZMP;
    TRHandlers	ZMPHandler;
public:
    void	RefreshData();
    void	UpdateData();

    int		SetInitialZMP(const double xzmp, const double yzmp);

    // Add trajectory information (fifth polynomial, const)
    void	AddXZMPFifth(const double _xzmp, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddXZMPFifth(const double _t);	//const

    void	AddYZMPFifth(const double _xzmp, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void	AddYZMPFifth(const double _t);	//const
};



class AddCompClass{
public:
    AddCompClass(double _tick = 0.005);

    doubles     AddJoint;
    doubles     AddCOM;
    doubles     AddLeftFoot;
    doubles     AddRightFoot;

    TRHandlers  JointHandler;
    TRHandlers  COMHandler;
    TRHandlers  LFHandler;
    TRHandlers  RFHandler;

    //-------------add vel and acc
    doubles     AddCOMVel;
    doubles     AddCOMAcc;

    TRHandlers  COMVelHandler;
    TRHandlers  COMAccHandler;

public:
    void	RefreshData();
    void	UpdateData();
    int		SetInitialCOM(const double com1);

    int		SetInitialAddJoint();
    int     SetInitialAddCOM(doubles _com);
    int     SetInitialLeftFoot(doubles _left);
    int     SetInitialRightFoot(doubles _right);

    void    AddJointsFifth(const int jointNum, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void    AddJointsFifth(const int jointNum, const double _t);
    void    AddCOMFifth(const int xyz, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void    AddCOMFifth(const int xyz, const double _t);
    void    AddLeftFootFifth(const int xyz, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void    AddLeftFootFifth(const int xyz, const double _t);
    void    AddRightFootFifth(const int xyz, const double _des, const double _t, const double _vel = 0.0, const double _acc = 0.0);
    void    AddRightFootFifth(const int xyz, const double _t);

    void    AddLeftFootCos(const int xyz, const double _des, const double _t);
    void    AddLeftFootCos(const int xyz, const double _t);
    void    AddRightFootCos(const int xyz, const double _des, const double _t);
    void    AddRightFootCos(const int xyz, const double _t);

    void    AddCOMCos(const int xyz, const double _des, const double _t);
    void    AddCOMCos(const int xyz, const double _t);
};


#endif // FOOTCLASS_H
