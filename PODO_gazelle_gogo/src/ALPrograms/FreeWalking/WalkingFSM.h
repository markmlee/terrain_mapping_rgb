//#ifndef WALKINGFSM_H
//#define WALKINGFSM_H

//#include <QVector>
//#include "BasicFSM.h"
//#include "FootClass.h"
//#include "../../SHARE/Headers/RBSharedMemory.h"
//#include "../../SHARE/Headers/UserSharedMemory.h"

//#define CAST_WFSM(x)		((pWFSM)(x))
//#define CAST_WSTATE(x)		((pRBWState)(x))
//#define CAST_DSPINIT(x)		((pDSPStateInit)(x))
//#define CAST_DSP(x)			((pDSPState)(x))
//#define CAST_DSPFINAL(x)	((pDSPStateFinal)(x))


//class WalkingFSM;
//class RBWalkingState;

//typedef WalkingFSM*			pWFSM;
//typedef RBWalkingState*		pRBWState;


//class DSPStateInit;
//class DSPState;
//class DSPStateFinal;
//class SSPStateForward;

//typedef DSPStateInit*		pDSPStateInit;
//typedef	DSPState*			pDSPState;
//typedef DSPStateFinal*		pDSPStateFinal;
//typedef SSPStateForward*	pSSPForward;



//typedef struct DSPTask_t{
//    double Left[3];
//    double Right[3];
//    double LYaw;
//    double RYaw;
//    double HTime;
//    double COMz[3];

//    double LPitch;
//    double RPitch;
//    double LRoll;
//    double RRoll;
//    double ZMP[2];


//}DSPTask;
//typedef QVector<DSPTask>	DSPTasks;

//typedef enum{
//    STATE_DSP_INIT = 0,
//    STATE_DSP,
//    STATE_DSP_FINAL,
//    STATE_SSP_FORWARD,
//    NUM_OF_STATE
//} WALKING_STATE;

//typedef enum{
//    STATE_DSP_INIT_RF = 0,
//    STATE_DSP_RF,//1
//    STATE_SSP_RF,//2

//    STATE_DSP_INIT_LF,//3
//    STATE_DSP_LF,//4
//    STATE_SSP_LF,//5

//    STATE_DSP_SAME,//6

//    STATE_DSP_FINAL_RFLF,//7
//    STATE_FINISHED//8

//} WALKING_DEATAIL_STATE;

//typedef enum{
//    RECENT_CURRENT = 0,
//    RECENT_POST1,
//    RECENT_POST2
//} RECENT_TASK;

//enum{
//    NORMAL_WALKING =0,
//    TERRAIN_WALKING,
//    TERRAIN_WALKING_ONE_STEP,
//    LADDER_WALKING,
//    GOAL_WALKING
//};
//enum Inside_OutsideMode
//{
//    INSIDE_WALKING = 0,
//    OUTSIDE_WALKING
//};
//const int FOOT_LEFT = 0;
//const int FOOT_RIGHT = 1;
//const int FOOT_ERROR = -1;
//const int FOOT_SAME = 10;


////double TIME_DSP = 0.25;//0.05;//0.2;
////double TIME_SSP = 0.75;//0.65;//0.65;





//class RBWalkingState : public RBAbstractState{
//public:
//    RBWalkingState(pRBAFSM parent = 0)
//        : RBAbstractState(parent) {


//                                   }


//    WALKING_STATE	GetState()		{return state;}
//    bool			IsFinished()	{return finishFlag;}

//    // Shared memory
//    pRBCORE_SHM walkingFSM_shared_data;
//    //using namespace std;







//protected:
//    WALKING_STATE	state;
//    bool			finishFlag;
//};



//class DSPStateInit : public RBWalkingState{
//public:
//    DSPStateInit(pRBAFSM parent = 0)
//        : RBWalkingState(parent) {state = STATE_DSP_INIT;}

//    virtual void	ExecuteBeginState();
//    virtual void	ExecuteDoingState();
//    virtual void	ExecuteEndState();

//    DSPTask		DSPInfo;
//    double		DSPTime;
//    double		ITimer;
//};

//class DSPState : public RBWalkingState{
//public:
//    DSPState(pRBAFSM parent = 0)
//        : RBWalkingState(parent) {state = STATE_DSP;}

//    virtual void	ExecuteBeginState();
//    virtual void	ExecuteDoingState();
//    virtual void	ExecuteEndState();

//    DSPTask		DSPInfo;
//    double		DSPTime;
//    double      HoldTime;
//    double		ITimer;
//    double      HTimer;
//};

//class DSPStateFinal : public RBWalkingState{
//public:
//    DSPStateFinal(pRBAFSM parent = 0)
//        : RBWalkingState(parent) {state = STATE_DSP_FINAL;}

//    virtual void	ExecuteBeginState();
//    virtual void	ExecuteDoingState();
//    virtual void	ExecuteEndState();

//    DSPTask		DSPInfo;
//    double		DSPTime;
//    double		ITimer;
//};

//class SSPStateForward : public RBWalkingState{
//public:
//    SSPStateForward(pRBAFSM parent = 0)
//        : RBWalkingState(parent) {state = STATE_SSP_FORWARD;}

//    virtual void	ExecuteBeginState();
//    virtual void	ExecuteDoingState();
//    virtual void	ExecuteEndState();

//    double		SSPTime;
//    double		ITimer;
//};


//class WalkingFSM : RBAbstractFSM{
//public:
//    WalkingFSM();
//    double          DEL_T;

//    DSPTasks		DSPScheduler;
//    DSPTasks		DSPRecentTask;

//    int             PrevFoot;
//    int             IsZeroStep;
//    int             IsUpward;
//    double          StepLength;
//    double          StepHeight;
//    double          SideStepLength;
//    double          HoldTime;

//    FootClass		*LeftFoot;
//    FootClass		*RightFoot;

//    ZMPClass		*ZMPPos;

//    int             WindowSize;
//    doubless        LeftInfos;
//    doubless        RightInfos;
//    doubless        ZMPInfos;

//    //------------foot velocity trajectory
//    doubless        LeftInfosVel;
//    doubless        RightInfosVel;

//    AddCompClass     *CompData;

//    doubless        AddJointInfos;
//    doubless        AddRightFootInfos;
//    doubless        AddLeftFootInfos;

//    //---------- add com
//    doubless        AddComInfos;

//    //---------- virtual com
//    doubless        VirtualComInfos;
//    doubless        VirtualComVelInfos;
//    doubless        VirtualComAccInfos;

//    doubles         curDSP;
//    doubles         postDSP;

//    intss            StateInfos;
//    doubless        FootUpInfos;
//    int             CurrentState;
//    int             IsLastSSP;

//    doubles         ITimeInfos;
//    doubles         STimeInfos;
//    double          CurrentITime;
//    double          CurrentSTime;

//    double FootUpHeight;//0.08;
//    double FootUpTime;

//    double FootDownTarget_RF;
//    double FootDownTarget_LF;


//    double TIME_DSP ,TIME_SSP;
//    double TimeRatio,TIME_SSP_Ratio;


//    double _com_offset,_temp_lz,_temp_rz, _temp_lx,_temp_rx,_alpha ,_block_height,_time_diff;
//    int _foot_height_diff,_dsp_foot_height_diff;
//    int SUPPORT_STATE;
//    int isTerrain;


//    double _temp_add_x ;
//    bool _normal_walking_flag;
//    int walking_mode;

//    pRBAState	targetStates[NUM_OF_STATE];

//    virtual void	Update();
//    virtual void	CheckStateTransition();

//    void    DataUpdate();

//    void	UpdateRecentTask(DSPTask &task);
//    void	MoveSchedulerToRecent(int n = 1);

//    void	SetInitialFootPlace(DSPTask task);

//    void	SetInitialFootPlace2(DSPTask task);

//    int		FootSelector();
//    int		FootSelectorNext();
//    void    ClearAll();
//};

//#endif // WALKINGFSM_H



#ifndef WALKINGFSM_H
#define WALKINGFSM_H

#include <QVector>
#include "BasicFSM.h"
#include "FootClass.h"
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/UserSharedMemory.h"

#define CAST_WFSM(x)		((pWFSM)(x))
#define CAST_WSTATE(x)		((pRBWState)(x))
#define CAST_DSPINIT(x)		((pDSPStateInit)(x))
#define CAST_DSP(x)			((pDSPState)(x))
#define CAST_DSPFINAL(x)	((pDSPStateFinal)(x))


class WalkingFSM;
class RBWalkingState;

typedef WalkingFSM*			pWFSM;
typedef RBWalkingState*		pRBWState;


class DSPStateInit;
class DSPState;
class DSPStateFinal;
class SSPStateForward;

typedef DSPStateInit*		pDSPStateInit;
typedef	DSPState*			pDSPState;
typedef DSPStateFinal*		pDSPStateFinal;
typedef SSPStateForward*	pSSPForward;



typedef struct DSPTask_t{
    double Left[3];
    double Right[3];
    double LYaw;
    double RYaw;
    double HTime;
    double COMz[3];

    double LPitch;
    double RPitch;
    double LRoll;
    double RRoll;
    double ZMP[2];


}DSPTask;
typedef QVector<DSPTask>	DSPTasks;

typedef enum{
    STATE_DSP_INIT = 0,
    STATE_DSP,
    STATE_DSP_FINAL,
    STATE_SSP_FORWARD,
    NUM_OF_STATE
} WALKING_STATE;

typedef enum{
    STATE_DSP_INIT_RF = 0,
    STATE_DSP_RF,//1
    STATE_SSP_RF,//2

    STATE_DSP_INIT_LF,//3
    STATE_DSP_LF,//4
    STATE_SSP_LF,//5

    STATE_DSP_SAME,//6

    STATE_DSP_FINAL_RFLF,//7
    STATE_FINISHED//8

} WALKING_DEATAIL_STATE;

typedef enum{
    RECENT_CURRENT = 0,
    RECENT_POST1,
    RECENT_POST2
} RECENT_TASK;

enum{
    NORMAL_WALKING =0,
    TERRAIN_WALKING,
    TERRAIN_WALKING_ONE_STEP,
    LADDER_WALKING,
    GOAL_WALKING
};
enum Inside_OutsideMode
{
    INSIDE_WALKING = 0,
    OUTSIDE_WALKING
};
const int FOOT_LEFT = -1;
const int FOOT_RIGHT = 1;
const int FOOT_ERROR = 0;
const int FOOT_SAME = 10;


//double TIME_DSP = 0.25;//0.05;//0.2;
//double TIME_SSP = 0.75;//0.65;//0.65;





class RBWalkingState : public RBAbstractState{
public:
    RBWalkingState(pRBAFSM parent = 0)
        : RBAbstractState(parent) {


                                   }


    WALKING_STATE	GetState()		{return state;}
    bool			IsFinished()	{return finishFlag;}







protected:
    WALKING_STATE	state;
    bool			finishFlag;
};



class DSPStateInit : public RBWalkingState{
public:
    DSPStateInit(pRBAFSM parent = 0)
        : RBWalkingState(parent) {state = STATE_DSP_INIT;}

    virtual void	ExecuteBeginState();
    virtual void	ExecuteDoingState();
    virtual void	ExecuteEndState();

    DSPTask		DSPInfo;
    double		DSPTime;
    double		ITimer;
};

class DSPState : public RBWalkingState{
public:
    DSPState(pRBAFSM parent = 0)
        : RBWalkingState(parent) {state = STATE_DSP;}

    virtual void	ExecuteBeginState();
    virtual void	ExecuteDoingState();
    virtual void	ExecuteEndState();

    DSPTask		DSPInfo;
    double		DSPTime;
    double      HoldTime;
    double		ITimer;
    double      HTimer;
};

class DSPStateFinal : public RBWalkingState{
public:
    DSPStateFinal(pRBAFSM parent = 0)
        : RBWalkingState(parent) {state = STATE_DSP_FINAL;}

    virtual void	ExecuteBeginState();
    virtual void	ExecuteDoingState();
    virtual void	ExecuteEndState();

    DSPTask		DSPInfo;
    double		DSPTime;
    double		ITimer;
};

class SSPStateForward : public RBWalkingState{
public:
    SSPStateForward(pRBAFSM parent = 0)
        : RBWalkingState(parent) {state = STATE_SSP_FORWARD;}

    virtual void	ExecuteBeginState();
    virtual void	ExecuteDoingState();
    virtual void	ExecuteEndState();

    double		SSPTime;
    double		ITimer;
};


class WalkingFSM : RBAbstractFSM{
public:
    WalkingFSM();
    double          DEL_T;

    DSPTasks		DSPScheduler;
    DSPTasks		DSPRecentTask;

    int             PrevFoot;
    int             IsZeroStep;
    int             IsUpward;
    double          StepLength;
    double          StepHeight;
    double          SideStepLength;
    double          HoldTime;

    FootClass		*LeftFoot;
    FootClass		*RightFoot;

    ZMPClass		*ZMPPos;

    int             WindowSize;
    doubless        LeftInfos;
    doubless        RightInfos;
    doubless        ZMPInfos;

    //------------foot velocity trajectory
    doubless        LeftInfosVel;
    doubless        RightInfosVel;

    AddCompClass     *CompData;

    doubless        AddJointInfos;
    doubless        AddRightFootInfos;
    doubless        AddLeftFootInfos;

    //---------- add com
    doubless        AddComInfos;

    //---------- virtual com
    doubless        VirtualComInfos;
    doubless        VirtualComVelInfos;
    doubless        VirtualComAccInfos;

    doubles         curDSP;
    doubles         postDSP;

    intss            StateInfos;
    doubless        FootUpInfos;
    int             CurrentState;
    int             IsLastSSP;

    doubles         ITimeInfos;
    doubles         STimeInfos;
    double          CurrentITime;
    double          CurrentSTime;

    double FootUpHeight;//0.08;
    double FootUpTime;

    double FootDownTarget_RF;
    double FootDownTarget_LF;


    double TIME_DSP ,TIME_SSP;
    double TimeRatio,TIME_SSP_Ratio;

    double _com_offset,_temp_lz,_temp_rz, _temp_lx,_temp_rx,_alpha ,_block_height,_time_diff;
    int _foot_height_diff,_dsp_foot_height_diff;
    int SUPPORT_STATE,isTerrain;
    double _RHpos[3],_LHpos[3];


    double _temp_add_x ;
    bool _normal_walking_flag;
    int walking_mode;
    int heel_pitching_mode;

    pRBAState	targetStates[NUM_OF_STATE];

    virtual void	Update();
    virtual void	CheckStateTransition();

    void    DataUpdate();

    void	UpdateRecentTask(DSPTask &task);
    void	MoveSchedulerToRecent(int n = 1);

    void	SetInitialFootPlace(DSPTask task);

    void	SetInitialFootPlace2(DSPTask task);

    void LL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3]);
    void Forward_LL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3]);

    void RL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3]);
    void Forward_RL_Heel_Configurtion(double Fpos_x,double Fpos_y,double Fpos_z,double t1,double t2, double t3,double Heel_pos[3]);


    int		FootSelector();
    int		FootSelectorNext();
    void    ClearAll();
};

#endif // WALKINGFSM_H
