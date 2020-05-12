#ifndef WMUPPERBODY_H
#define WMUPPERBODY_H
//-----------------------------------------------------
// Basic Constant
//-----------------------------------------------------
#ifndef PI
#define PI			3.141592653589793
#endif
#ifndef D2R
#define D2R			1.745329251994330e-2
#endif
#ifndef R2D
#define R2D			5.729577951308232e1
#endif

#define RIGHT_HAND                  0
#define LEFT_HAND                   1
//-----------------------------------------------------
// Basic Constant
//-----------------------------------------------------
enum CarTask_ALCOMMAND
{
    CarTask_AL_NO_ACT = 100,
    CarTask_AL_UPPER_TASK_POSE,
    CarTask_AL_CAR_TASK_POSE,
    CarTask_AL_HAND,
    CarTask_AL_GAIN,
    CarTask_AL_CT_GAIN_TUNING,
    CarTask_AL_CT_GO_POS,
    CarTask_AL_SAVE_ENC,
    CarTask_AL_CT_START,
    CarTask_AL_CT_GRAVITY_COMP,
    CarTask_AL_CT_ZERO_GAIN,
    CarTask_AL_CT_GO_POSITION_MODE,
    CarTask_AL_CT_FORCE_CONTROL,
    CarTask_AL_CT_HYBRID_CONTROL,
    CarTask_AL_CT_REFRESH,
    CarTask_AL_CAR_TURNING,
    CarTask_AL_CAR_TURNING_POSE,
    CarTask_AL_CAR_RELEASE,
    CarTask_AL_GRAB_RH,
    CarTask_AL_GRAB_RH_VISION_DATA,
    CarTask_AL_HAND_GRAB,
    CarTask_AL_GO_DESCENDING_POS,
    CarTask_AL_START_DESCENDING,
    CarTask_AL_PRINT_STATE,
    CarTask_AL_CT_REFRESH_POSITION,
    CarTask_AL_CT_POSITION_CONTROL,
    CarTask_AL_STATIC_WALK,
    CarTask_AL_STABILIZING,
    CarTask_AL_STABILIZING_LEFT,
    CarTask_AL_WALKREADY_POS,
    CarTask_AL_RH_GRAB_POS,
    CarTask_AL_HOME_POS,
    CarTask_AL_JOINT_FORCE_CTRL_TEST,
    CarTask_AL_WRIST_FT_NULLING,
    CarTask_AL_RH_RE_GRAB_POS,
    CarTask_Al_AFTER_STATIC_WALK_POS,
    EgressTask_AL_EGRESS_POS,
    CarTask_AL_SEND_ROI,
    CarTask_AL_CHECK_RH_GRAB_POS_VISION,
    CarTask_AL_MODIFY_VISION_DATA,
    CarTask_AL_SAVE_DATA,
};

//-----------------------------------------------------
// WBIK variables/functions
//-----------------------------------------------------
void StartWBIKmotion(int _mode);
void PrintWBIKinfo(void);

//-----------------------------------------------------
// CarTask Functions and Variables
//-----------------------------------------------------
void Change_Pos_CAR1(void);
void Change_Pos_CAR2(void);
void GotoHomePos(void);
void After_StaticWalk_pos(void);
void DescendingSequence(void);
void RHApproachControl(void);
void StaticWalkingSequence(void);
void RHGrabPos(void);
void Stabilizing_control_RH(void);
void Stabilizing_control_LH(void);
void GoWalkReady_sequence_new(void);

double RH_Fx;
double RH_Fy;
double RH_Fz;
double LH_Fx;
double LH_Fy;
double LH_Fz;
double RH_x;
double RH_y;
double RH_z;
double LH_x;
double LH_y;
double LH_z;

double qRH_4x1[4];
double qLH_4x1[4];

double RSP_angle;
double RSR_angle;
double RSY_angle;
double REB_angle;
double RWY_angle;
double RWP_angle;
double RF1_angle;

double LSP_angle;
double LSR_angle;
double LSY_angle;
double LEB_angle;
double LWY_angle;
double LWP_angle;
double LF1_angle;

double RH_grab_pos_non_vision[3]={0,0,0};
double RH_grab_pos_vision[3]={0,0,0};

// Flags -----------------------------
char WBIK_mode = 0;

int Descending_SQ_No = 0;
int StaticWalk_SQ_No = 0;
int GoWalkReady_SQ_No = 0;
int Stabilizing_RH_SQ_No = 0;
int Stabilizing_LH_SQ_No = 0;
int RH_Grab_pos_SQ_No = 0;

int pos_status = 0;

bool Descending_flag = false;
bool Static_walk_flag = false;
bool Stabilizing_RH_flag = false;
bool Stabilizing_LH_flag = false;

bool WalkReady_pos_flag = false;
bool RH_grab_pos_flag = false;

bool GravityComp_Left_Flag = false;
bool GravityComp_Right_Flag = false;

// Debug
int Iteration = 0;
void PrintJointRef(void);

//for JFR Experiment
float RWFT_Z_Data[100000];
unsigned int Data_index;

//-------------------------------------------------------------------------------------------
//--------------------------------------Egress Mode---------------------------------------
//Functions
void Change_Pos_Egress1(void);
void Change_Pos_Egress2(void);
void Egress_Driving_Pos(void);


#endif // WMUPPERBODY_H
