
#include "BasicFiles/BasicSetting.h"

#include <alchemy/task.h>
#include "rtdm/rtdm.h"

#include "myfunctions.h"
#include <cmatrix>

typedef techsoft::matrix<double> Matrix;
// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *jCon;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

unsigned int saveFlag=0;
unsigned int saveIndex=0;
float DataBuf[16][100000];
//float DataBuf[57][10000];
void SaveFile(void);


RTIME tickTime1, tickTime2;
void setStart(){
    tickTime1 = rt_timer_read();
}
void setEnd(){
    tickTime2 = rt_timer_read();
}
double checkDiff(){
    return (tickTime2 -tickTime1)/1000000.0;
}
unsigned int tickCount = 0;
double tick_time_sum =0;
/************************************************************
 * Hyoin Global Variables
 * *********************************************************/
unsigned char TARGET_DIR = 1;

bool    FLAG_PELVIS_ESTIMATION = false;
bool    FLAG_COM_ESTIMATION = false;
bool    FLAG_PELVIS_ESTIMATION_ISFIRST = true;

static double dt = 0.005;
static double D2R = 0.017453292519943;
static double R2D = 57.295779513082323;
static double PI = 3.141592653589793;

double unit = 1000;

double g = 9.81*unit;
double L = 0.79*unit;

double cutoff = 20;//Hz
double alpha = 1/(1+cutoff*2*PI*dt);
double beta = 1.0 - alpha;

unsigned int nn = 3;//<---------------# of state
//unsigned int mm = 2;//<---------------# of meas
unsigned int mm = 3;//<---------------# of meas    /Hyobin modi

// Matrix
Matrix A(nn,nn, 0.0);
Matrix trA(nn,nn, 0.0);
Matrix H(mm,nn, 0.0);
Matrix trH(nn,mm, 0.0);

Matrix Q(nn,nn, 0.0);
Matrix R(mm,mm, 0.0);
Matrix Qp(nn,nn, 0.0);
Matrix Rr(nn,nn, 0.0);
Matrix iM(nn,nn, 0.0);

Matrix P(nn,nn, 0.0);
Matrix Pinit(nn,nn, 0.0);
Matrix Pp(nn,nn, 0.0);

Matrix W(nn,nn, 0.0);
Matrix wPinit(nn,nn, 0.0);
Matrix Wp(nn,nn, 0.0);

Matrix C(nn,nn, 0.0);
Matrix Cinit(nn,nn, 0.0);
Matrix Cp(nn,nn, 0.0);

Matrix K(nn,mm, 0.0);
Matrix tempK(mm,mm, 0.0);
Matrix trK(mm,nn, 0.0);

Matrix G(nn,nn, 0.0);
Matrix tempG(nn,nn, 0.0);
Matrix trG(nn,nn, 0.0);

Matrix tempP(nn,nn, 0.0);
Matrix tempW(nn,nn, 0.0);

// Matrix 2
Matrix PY(nn,nn, 0.0);
Matrix PYinit(nn,nn, 0.0);
Matrix PYp(nn,nn, 0.0);

Matrix WY(nn,nn, 0.0);
Matrix wYPinit(nn,nn, 0.0);
Matrix WYp(nn,nn, 0.0);

Matrix CY(nn,nn, 0.0);
Matrix CYinit(nn,nn, 0.0);
Matrix CYp(nn,nn, 0.0);

Matrix KY(nn,mm, 0.0);
Matrix tempKY(mm,mm, 0.0);
Matrix trKY(mm,nn, 0.0);

Matrix GY(nn,nn, 0.0);
Matrix tempGY(nn,nn, 0.0);
Matrix trGY(nn,nn, 0.0);

Matrix tempPY(nn,nn, 0.0);
Matrix tempWY(nn,nn, 0.0);

// Vector
Matrix x(nn,1, 0.0);
Matrix xinit(nn,1, 0.0);
Matrix xp(nn,1, 0.0);
Matrix tempF(nn,1, 0.0);
Matrix cur_meas(mm,1, 0.0);
Matrix tempLPF_x(nn,1, 0.0);

Matrix w(nn,1, 0.0);
Matrix winit(nn,1, 0.0);
Matrix wp(nn,1, 0.0);
Matrix weq(nn,1, 0.0);

// Vector 2
Matrix y(nn,1, 0.0);
Matrix yinit(nn,1, 0.0);
Matrix yp(nn,1, 0.0);
Matrix tempFY(nn,1, 0.0);
Matrix cur_measY(mm,1, 0.0);
Matrix tempLPF_y(nn,1, 0.0);

Matrix wY(nn,1, 0.0);
Matrix wYinit(nn,1, 0.0);
Matrix wYp(nn,1, 0.0);
Matrix wYeq(nn,1, 0.0);


/************************************************************
 * Hyoin Global Variables
 * *********************************************************/

/************************************************************
 * Hyoin Global Variables - Kinematics
 * *********************************************************/
double  P2H = 0.0885;//0.105;//
double  ULEG = 0.28;//0.4;//
double  LLEG = 0.28;//0.38;//
double  AP2AR = 0.001;//0.025;//
double  A2F = 0.1;//0.113;//

double  P2SC = 0.3687;//
double  SC2S = 0.2145;//0.249;//
double  UARM = 0.17914;//
double  LARM = 0.16261;//
double  OFFELB = 0.022;//

double t_offset1[] = {0,0,P2SC};
Matrix offset_pel2sc(3,1, t_offset1);
double t_offset2[] = {0,-SC2S,0};
Matrix offset_sc2rs(3,1, t_offset2);
double t_offset3[] = {0,+SC2S,0};
Matrix offset_sc2ls(3,1, t_offset3);
double t_offset4[] = {OFFELB,0,-UARM};
Matrix offset_s2eb(3,1, t_offset4);
double t_offset5[] = {-OFFELB,0,-LARM};
Matrix offset_eb2w(3,1, t_offset5);

double t_offset6[] = {0,-P2H,0};
Matrix offset_pel2rh(3,1, t_offset6);
double t_offset7[] = {0,+P2H,0};
Matrix offset_pel2lh(3,1, t_offset7);
double t_offset8[] = {0,0,-ULEG};
Matrix offset_h2kn(3,1, t_offset8);
double t_offset9[] = {0,0,-LLEG};
Matrix offset_kn2an1(3,1, t_offset9);
double t_offset10[] = {0,0,-AP2AR};
Matrix offset_an12an2(3,1, t_offset10);
double t_offset11[] = {0,0,-A2F};
Matrix offset_an22ft(3,1, t_offset11);

double  m_rhy = 0.01;//0.7912;
double  m_rhr = 0.01;//1.3556;
double  m_rhp = 6.3512;//5.7294;
double  m_rkn = 1.9592;//3.70132;
double  m_rap = 0.01;//1.5781;
double  m_rar = 2.6626;//1.7955;

double  m_lhy = 0.01;//0.7912;
double  m_lhr = 0.01;//1.3556;
double  m_lhp = 6.3512;//5.7294;
double  m_lkn = 1.9592;//3.70132;
double  m_lap = 0.01;//1.5781;
double  m_lar = 2.6626;//1.7955;

double  m_lb = m_rhy + m_rhr + m_rhp + m_rkn + m_rap + m_rar + m_lhy + m_lhr + m_lhp + m_lkn + m_lap + m_lar;

double  m_rsp = 0.01;//0.;
double  m_rsr = 0.01;//0.;
double  m_rsy = 0.01;//0.;
double  m_reb = 0.01;//0.;
double  m_rwy = 0.01;//3.67881;
double  m_rwp = 0.001;//0.;
double  m_rf1 = 0.001;//0.;

double  m_lsp = 0.01;//0.;
double  m_lsr = 0.01;//0.;
double  m_lsy = 0.01;//0.;
double  m_leb = 0.01;//0.;
double  m_lwy = 0.01;//3.67881;
double  m_lwp = 0.001;//0.;
double  m_lf1 = 0.001;//0.;

double  m_torso = 17.2025;//24.9872;//
double  m_pel = 3.876;//11.8679;//

double  m_ub = m_pel+m_torso + m_rsp + m_rsr + m_rsy + m_reb + m_rwy + m_rwp + m_rf1 + m_lsp + m_lsr + m_lsy + m_leb + m_lwy + m_lwp + m_lf1;

double  m_tot = m_lb + m_ub;

double c_offset1[] = {0.0,0.0,0};
Matrix c_rsp(3,1, c_offset1);
double c_offset2[] = {0.0,0,0.0};
Matrix c_rsr(3,1, c_offset2);
double c_offset3[] = {0.0062,0.0178,-0.0451};
Matrix c_rsy(3,1, c_offset3);
double c_offset4[] = {0.0003,-0.0006,-0.047};
Matrix c_reb(3,1, c_offset4);
double c_offset5[] = {0.0, 0.0, 0.0};
Matrix c_rwy(3,1, c_offset5);
double c_offset6[] = {0.0, 0.0, 0.0};
Matrix c_rwp(3,1, c_offset6);
double c_offset7[] = {0.0, 0.0, 0.0};
Matrix c_rf1(3,1, c_offset7);

double c_offset8[] = {0.0, 0.0, 0.0};
Matrix c_lsp(3,1, c_offset8);
double c_offset9[] = {0.0, 0.0, 0.0};
Matrix c_lsr(3,1, c_offset9);
double c_offset10[] = {0.0062,-0.0178,-0.0451};
Matrix c_lsy(3,1, c_offset10);
double c_offset11[] = {0.0003,0.0006,-0.047};
Matrix c_leb(3,1, c_offset11);
double c_offset12[] = {0.0, 0.0, 0.0};
Matrix c_lwy(3,1, c_offset12);
double c_offset13[] = {0.0, 0.0, 0.0};
Matrix c_lwp(3,1, c_offset13);
double c_offset14[] = {0.0, 0.0, 0.0};
Matrix c_lf1(3,1, c_offset14);

double c_offset15[] = {-0.04, 0.0, 0.476};
Matrix c_torso(3,1, c_offset15);
double c_offset16[] = {-0.0119, 0.0, 0.1323};
Matrix c_pel(3,1, c_offset16);

double c_offset17[] = {0.0, 0.0, 0.0};
Matrix c_rhy(3,1, c_offset17);
double c_offset18[] = {0.0, 0.0, 0.0};
Matrix c_rhr(3,1, c_offset18);
double c_offset19[] = {0.0175,-0.0099,-0.0995};
Matrix c_rhp(3,1, c_offset19);
double c_offset20[] = {0.0146,-0.0146,-0.1845};
Matrix c_rkn(3,1, c_offset20);
double c_offset21[] = {0.0, 0.0, 0.0};
Matrix c_rap(3,1, c_offset21);
double c_offset22[] = {0.0216, -0.0014, -0.016};
Matrix c_rar(3,1, c_offset22);

double c_offset23[] = {0.0, 0.0, 0.0};
Matrix c_lhy(3,1, c_offset23);
double c_offset24[] = {0.0, 0.0, 0.0};
Matrix c_lhr(3,1, c_offset24);
double c_offset25[] = {0.0175,0.0099,-0.0995};
Matrix c_lhp(3,1, c_offset25);
double c_offset26[] = {0.0146,0.0146,-0.1845};
Matrix c_lkn(3,1, c_offset26);
double c_offset27[] = {0.0, 0.0, 0.0};
Matrix c_lap(3,1, c_offset27);
double c_offset28[] = {0.0216, 0.0014, -0.016};
Matrix c_lar(3,1, c_offset28);

Matrix R_IMU_X(3,3, 0.0);
Matrix R_IMU_Y(3,3, 0.0);
Matrix R_IMU_Z(3,3, 0.0);

Matrix R_PEL(3,3, 0.0);

Matrix R_RHY(3,3, 0.0);
Matrix R_RHR(3,3, 0.0);
Matrix R_RHP(3,3, 0.0);
Matrix R_RKN(3,3, 0.0);
Matrix R_RAP(3,3, 0.0);
Matrix R_RAR(3,3, 0.0);

Matrix R_LHY(3,3, 0.0);
Matrix R_LHR(3,3, 0.0);
Matrix R_LHP(3,3, 0.0);
Matrix R_LKN(3,3, 0.0);
Matrix R_LAP(3,3, 0.0);
Matrix R_LAR(3,3, 0.0);

Matrix R_WST(3,3, 0.0);

Matrix R_RSP(3,3, 0.0);
Matrix R_RSR(3,3, 0.0);
Matrix R_RSY(3,3, 0.0);
Matrix R_REB(3,3, 0.0);
Matrix R_RWY(3,3, 0.0);
Matrix R_RWP(3,3, 0.0);
Matrix R_RF1(3,3, 0.0);

Matrix R_LSP(3,3, 0.0);
Matrix R_LSR(3,3, 0.0);
Matrix R_LSY(3,3, 0.0);
Matrix R_LEB(3,3, 0.0);
Matrix R_LWY(3,3, 0.0);
Matrix R_LWP(3,3, 0.0);
Matrix R_LF1(3,3, 0.0);

Matrix R_RHIP(3,3, 0.0);
Matrix R_RKNEE(3,3, 0.0);
Matrix R_RFOOT(3,3, 0.0);

Matrix R_LHIP(3,3, 0.0);
Matrix R_LKNEE(3,3, 0.0);
Matrix R_LFOOT(3,3, 0.0);

Matrix R_TORSO(3,3, 0.0);

Matrix R_RSHD(3,3, 0.0);
Matrix R_RELB(3,3, 0.0);
Matrix R_RHAND(3,3, 0.0);

Matrix R_LSHD(3,3, 0.0);
Matrix R_LELB(3,3, 0.0);
Matrix R_LHAND(3,3, 0.0);

Matrix p_pel(3,1, 0.0);

Matrix p_rh(3,1, 0.0);
Matrix p_rkn(3,1, 0.0);
Matrix p_ran1(3,1, 0.0);
Matrix p_ran2(3,1, 0.0);
Matrix p_rf(3,1, 0.0);
Matrix prev_p_rf(3,1, 0.0);

Matrix p_lh(3,1, 0.0);
Matrix p_lkn(3,1, 0.0);
Matrix p_lan1(3,1, 0.0);
Matrix p_lan2(3,1, 0.0);
Matrix p_lf(3,1, 0.0);
Matrix prev_p_lf(3,1, 0.0);

Matrix p_sc(3,1, 0.0);

Matrix p_rs(3,1, 0.0);
Matrix p_reb(3,1, 0.0);
Matrix p_rw(3,1, 0.0);

Matrix p_ls(3,1, 0.0);
Matrix p_leb(3,1, 0.0);
Matrix p_lw(3,1, 0.0);

Matrix pm_PEL(3,1, 0.0);

Matrix pm_RHY(3,1, 0.0);
Matrix pm_RHR(3,1, 0.0);
Matrix pm_RHP(3,1, 0.0);
Matrix pm_RKN(3,1, 0.0);
Matrix pm_RAP(3,1, 0.0);
Matrix pm_RAR(3,1, 0.0);

Matrix pm_LHY(3,1, 0.0);
Matrix pm_LHR(3,1, 0.0);
Matrix pm_LHP(3,1, 0.0);
Matrix pm_LKN(3,1, 0.0);
Matrix pm_LAP(3,1, 0.0);
Matrix pm_LAR(3,1, 0.0);

Matrix pm_TORSO(3,1, 0.0);

Matrix pm_RSP(3,1, 0.0);
Matrix pm_RSR(3,1, 0.0);
Matrix pm_RSY(3,1, 0.0);
Matrix pm_REB(3,1, 0.0);
Matrix pm_RWY(3,1, 0.0);
Matrix pm_RWP(3,1, 0.0);
Matrix pm_RF1(3,1, 0.0);

Matrix pm_LSP(3,1, 0.0);
Matrix pm_LSR(3,1, 0.0);
Matrix pm_LSY(3,1, 0.0);
Matrix pm_LEB(3,1, 0.0);
Matrix pm_LWY(3,1, 0.0);
Matrix pm_LWP(3,1, 0.0);
Matrix pm_LF1(3,1, 0.0);

Matrix pCOM_LB(3,1, 0.0);
Matrix pCOM_UB(3,1, 0.0);
Matrix pCOM_TOT(3,1, 0.0);

Matrix temp_1(3,1, 0.0);
Matrix temp_2(3,1, 0.0);
Matrix temp_3(3,1, 0.0);

Matrix FT_R(6,1, 0.0);
Matrix FT_L(6,1, 0.0);
Matrix FT_CUR_R(6,1, 0.0);
Matrix FT_CUR_L(6,1, 0.0);

Matrix ref_point(3,1, 0.0);
Matrix cur_pelvis(3,1, 0.0);
Matrix cur_rf(3,1, 0.0);
Matrix cur_lf(3,1, 0.0);
Matrix CALC_COM(3,1, 0.0);
Matrix CALC_ZMP(2,1, 0.0);

int PHASE_MODE = 0;
int prev_PHASE_MODE = 0;
int foot_z_th = 50;
double FT_cutoff = 60;//hyobin
double FT_alpha = 1 / (2*PI*dt*FT_cutoff + 1);
/************************************************************
 * Hyoin Global Variables - Kinematics
 * *********************************************************/

void DLKF_INIT_PARAMETER(void){
    // Model
    A(0,0) = 1;
    A(0,1) = dt;
    A(1,0) = g/L*dt;
    A(1,1) = 1;
    A(1,2) = -g/L*dt;
    A(2,2) = 1;

    trA = ~A;

    H(0,0) = H(1,2) = 1;

    trH = ~H;

    // Init cov
    Pinit(0,0) = 0.2;
    Pinit(1,1) = 0.5;
    Pinit(2,2) = 1.0;

    wPinit(0,0) = wPinit(1,1) = wPinit(2,2) = 0.2;
    Cinit(0,0) = Cinit(1,1) = Cinit(2,2) = 0.1;

    PYinit(0,0) = 0.2;
    PYinit(1,1) = 0.5;
    PYinit(2,2) = 1.0;

    wYPinit(0,0) = wYPinit(1,1) = wYPinit(2,2) = 0.2;
    CYinit(0,0) = CYinit(1,1) = CYinit(2,2) = 0.1;

    // Tune
    Q(0,0) = 2.;
    Q(1,1) = 2.;
    Q(2,2) = 5.;

    R(0,0) = 500;//500.;  // COM measurement noise
    R(1,1) = 500;          // dCOM measurment noise
    R(2,2) = 20;//500.;  // ZMP measurment noise           Hyobin modi

    Qp(0,0) = 10.;
    Qp(1,1) = 50.;
    Qp(2,2) = 10.;

    Rr(0,0) = Rr(1,1) = Rr(2,2) = 10;

    iM(0,0) = iM(1,1) = iM(2,2) = 1.0;

    xinit(0,0) = 0;//<---------------COM pos
    xinit(1,0) = 0;//<---------------COM vel
    xinit(2,0) = 0;//<---------------ZMP pos

    yinit(0,0) = 0;//<---------------COM pos
    yinit(1,0) = 0;//<---------------COM vel
    yinit(2,0) = 0;//<---------------ZMP pos
}

void DLKF_INITIALIZE(void){
    x = xinit;
    w = winit;

    P = Pinit;
    W = wPinit;
    C = Cinit;

    tempLPF_x = xinit;


    y = yinit;
    wY = wYinit;

    PY = PYinit;
    WY = wYPinit;
    CY = CYinit;

    tempLPF_y = yinit;
}

void DLKF_ESTIMATION(void){
//    cur_meas(0,0) = CALC_COM(0, 0);//<---------------Forward Kinematic COMx
//    cur_meas(1,0) = CALC_ZMP(0, 0);//<---------------ZMPx from F/T
    cur_meas(0,0) = sharedCMD->StateEstimate.uX;
    cur_meas(1,0) = sharedCMD->StateEstimate.udX;
    cur_meas(2,0) = sharedCMD->StateEstimate.uZMPx; //<---------------ZMPx from F/T                  Hyobin modi

    tempF = A*x;
    xp = tempF + w;
    Pp = A*P*trA + A*C + ~C*trA + W;

//    xp = tempF;
//    Pp = A*P*trA + Q;

    tempK = H*Pp*trH + R;
    tempK = !tempK;
    K = Pp*trH*tempK;
    trK = ~K;
    x = xp + K*(cur_meas - H*xp);
    tempP = iM - K*H;
    P = tempP*Pp*~tempP + K*R*trK;
    P = 0.5*(P + ~P);

    weq = x - tempF;
    wp = w;
    Wp = W + Qp;
    Cp = A*C + W;
    tempG = K*H*Pp*trH*trK + K*R*trK + Rr;
    tempG = !tempG;
    G = ~Cp*trH*trK*tempG;
    trG = ~G;
    w = wp + G*(weq - wp);
    tempW = G*K*H*Cp;
    W = Wp - tempW - ~tempW + G*K*(H*Pp*trH + R)*trK*trG + G*Rr*trG;
    W = 0.5*(W + ~W);
    C = tempP*Cp - tempP*Pp*trH*trK*trG + K*R*trK*trG;

    tempLPF_x = alpha*tempLPF_x + beta*x;



//    cur_measY(0,0) = CALC_COM(1, 0);//<---------------Forward Kinematic COMy
//    cur_measY(1,0) = CALC_ZMP(1, 0);//<---------------ZMPy from F/T
    cur_measY(0,0) = sharedCMD->StateEstimate.uY;//<---------------Forward Kinematic COMy
    cur_measY(1,0) = sharedCMD->StateEstimate.udY;
    cur_measY(2,0) = sharedCMD->StateEstimate.uZMPy;//<---------------ZMPy from F/T                     Hyobin modi

    tempFY = A*y;
    yp = tempFY + wY;
    PYp = A*PY*trA + A*CY + ~CY*trA + WY;

//    yp = tempFY;
//    PYp = A*PY*trA + Q;

    tempKY = H*PYp*trH + R;
    tempKY = !tempKY;
    KY = PYp*trH*tempKY;
    trKY = ~KY;
    y = yp + KY*(cur_measY - H*yp);
    tempPY = iM - KY*H;
    PY = tempPY*PYp*~tempPY + KY*R*trKY;
    PY = 0.5*(PY + ~PY);

    wYeq = y - tempFY;
    wYp = wY;
    WYp = WY + Qp;
    CYp = A*CY + WY;
    tempGY = KY*H*PYp*trH*trKY + KY*R*trKY + Rr;
    tempGY = !tempGY;
    GY = ~CYp*trH*trKY*tempGY;
    trGY = ~GY;
    wY = wYp + GY*(wYeq - wYp);
    tempWY = GY*KY*H*CYp;
    WY = WYp - tempWY - ~tempWY + GY*KY*(H*PYp*trH + R)*trKY*trGY + GY*Rr*trGY;
    WY = 0.5*(WY + ~WY);
    CY = tempPY*CYp - tempPY*PYp*trH*trKY*trGY + KY*R*trKY*trGY;

    tempLPF_y = alpha*tempLPF_y + beta*y;
}

inline Matrix Rotation_Matrix_X(double angle){
    double rangle = angle*D2R;
    double rot_arr[] = {1, 0, 0, 0, cos(rangle), -sin(rangle), 0, sin(rangle), cos(rangle)};
    Matrix returnMatrix(3,3, rot_arr);
    return returnMatrix;
}

inline Matrix Rotation_Matrix_Y(double angle){
    double rangle = angle*D2R;
    double rot_arr[] = {cos(rangle), 0, sin(rangle), 0, 1, 0, -sin(rangle), 0, cos(rangle)};
    Matrix returnMatrix(3,3, rot_arr);
    return returnMatrix;
}

inline Matrix Rotation_Matrix_Z(double angle){
    double rangle = angle*D2R;
    double rot_arr[] = {cos(rangle), -sin(rangle), 0, sin(rangle), cos(rangle), 0, 0, 0, 1};
    Matrix returnMatrix(3,3, rot_arr);
    return returnMatrix;
}

void PELVIS_ESTIMATION(void){
    if(FLAG_PELVIS_ESTIMATION == true){
        R_IMU_X = Rotation_Matrix_X(sharedSEN->FOG.Roll);
        R_IMU_Y = Rotation_Matrix_Y(sharedSEN->FOG.Pitch);
        //R_IMU_Z = Rotation_Matrix_Z(sharedSEN->FOG.Yaw);

        R_PEL = R_IMU_Y*R_IMU_X;

        R_RHY = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[RHY]->MCId][jCon->Joints[RHY]->MCCh].CurrentPosition);
        R_RHR = Rotation_Matrix_X(sharedSEN->ENCODER[jCon->Joints[RHR]->MCId][jCon->Joints[RHR]->MCCh].CurrentPosition);
        R_RHP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[RHP]->MCId][jCon->Joints[RHP]->MCCh].CurrentPosition);
        R_RKN = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[RKN]->MCId][jCon->Joints[RKN]->MCCh].CurrentPosition);
        R_RAP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[RAP]->MCId][jCon->Joints[RAP]->MCCh].CurrentPosition);
        R_RAR = Rotation_Matrix_X(sharedSEN->ENCODER[jCon->Joints[RAR]->MCId][jCon->Joints[RAR]->MCCh].CurrentPosition);

        R_LHY = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[LHY]->MCId][jCon->Joints[LHY]->MCCh].CurrentPosition);
        R_LHR = Rotation_Matrix_X(sharedSEN->ENCODER[jCon->Joints[LHR]->MCId][jCon->Joints[LHR]->MCCh].CurrentPosition);
        R_LHP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[LHP]->MCId][jCon->Joints[LHP]->MCCh].CurrentPosition);
        R_LKN = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[LKN]->MCId][jCon->Joints[LKN]->MCCh].CurrentPosition);
        R_LAP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[LAP]->MCId][jCon->Joints[LAP]->MCCh].CurrentPosition);
        R_LAR = Rotation_Matrix_X(sharedSEN->ENCODER[jCon->Joints[LAR]->MCId][jCon->Joints[LAR]->MCCh].CurrentPosition);

        R_WST = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[WST]->MCId][jCon->Joints[WST]->MCCh].CurrentPosition);

        R_RSP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[RSP]->MCId][jCon->Joints[RSP]->MCCh].CurrentPosition);
        R_RSR = Rotation_Matrix_X(sharedSEN->ENCODER[jCon->Joints[RSR]->MCId][jCon->Joints[RSR]->MCCh].CurrentPosition - 15.);
        R_RSY = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[RSY]->MCId][jCon->Joints[RSY]->MCCh].CurrentPosition);
        R_REB = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[REB]->MCId][jCon->Joints[REB]->MCCh].CurrentPosition - 20.);
        R_RWY = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[RWY]->MCId][jCon->Joints[RWY]->MCCh].CurrentPosition);
        R_RWP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[RWP]->MCId][jCon->Joints[RWP]->MCCh].CurrentPosition);
        R_RF1 = Rotation_Matrix_Z(0.);//Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[RF1]->MCId][jCon->Joints[RF1]->MCCh].CurrentPosition);

        R_LSP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[LSP]->MCId][jCon->Joints[LSP]->MCCh].CurrentPosition);
        R_LSR = Rotation_Matrix_X(sharedSEN->ENCODER[jCon->Joints[LSR]->MCId][jCon->Joints[LSR]->MCCh].CurrentPosition + 15.);
        R_LSY = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[LSY]->MCId][jCon->Joints[LSY]->MCCh].CurrentPosition);
        R_LEB = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[LEB]->MCId][jCon->Joints[LEB]->MCCh].CurrentPosition - 20.);
        R_LWY = Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[LWY]->MCId][jCon->Joints[LWY]->MCCh].CurrentPosition);
        R_LWP = Rotation_Matrix_Y(sharedSEN->ENCODER[jCon->Joints[LWP]->MCId][jCon->Joints[LWP]->MCCh].CurrentPosition);
        R_LF1 = Rotation_Matrix_Z(0.);//Rotation_Matrix_Z(sharedSEN->ENCODER[jCon->Joints[LF1]->MCId][jCon->Joints[LF1]->MCCh].CurrentPosition);

        // right leg -------------------------------------

        p_rh = p_pel + R_PEL*offset_pel2rh;
        R_RHIP = R_PEL*R_RHY*R_RHR*R_RHP;
        p_rkn = p_rh + R_RHIP*offset_h2kn;
        R_RKNEE = R_RHIP*R_RKN;
        p_ran1 = p_rkn + R_RKNEE*offset_kn2an1;
        p_ran2 = p_ran1 + R_RKNEE*R_RAP*offset_an12an2;
        R_RFOOT = R_RKNEE*R_RAP*R_RAR;
        p_rf = p_ran2 + R_RFOOT*offset_an22ft;

        pm_RHY = p_rh + R_PEL*R_RHY*c_rhy;
        pm_RHR = p_rh + R_PEL*R_RHY*R_RHR*c_rhr;
        pm_RHP = p_rh + R_RHIP*c_rhp;
        pm_RKN = p_rkn + R_RKNEE*c_rkn;
        pm_RAP = p_ran1 + R_RKNEE*R_RAP*c_rap;
        pm_RAR = p_ran2 + R_RFOOT*c_rar;

        // left leg -------------------------------------

        p_lh = p_pel + R_PEL*offset_pel2lh;
        R_LHIP = R_PEL*R_LHY*R_LHR*R_LHP;
        p_lkn = p_lh + R_LHIP*offset_h2kn;
        R_LKNEE = R_LHIP*R_LKN;
        p_lan1 = p_lkn + R_LKNEE*offset_kn2an1;
        p_lan2 = p_lan1 + R_LKNEE*R_LAP*offset_an12an2;
        R_LFOOT = R_LKNEE*R_LAP*R_LAR;
        p_lf = p_lan2 + R_LFOOT*offset_an22ft;

        pm_LHY = p_lh + R_PEL*R_LHY*c_lhy;
        pm_LHR = p_lh + R_PEL*R_LHY*R_LHR*c_lhr;
        pm_LHP = p_lh + R_LHIP*c_lhp;
        pm_LKN = p_lkn + R_LKNEE*c_lkn;
        pm_LAP = p_lan1 + R_LKNEE*R_LAP*c_lap;
        pm_LAR = p_lan2 + R_LFOOT*c_lar;

        // upper body -------------------------------------
        R_TORSO = R_PEL*R_WST;
        p_sc = p_pel + R_TORSO*offset_pel2sc;

        p_rs = p_sc + R_TORSO*offset_sc2rs;
        R_RSHD = R_TORSO*R_RSP*R_RSR*R_RSY;
        p_reb = p_rs + R_RSHD*offset_s2eb;
        R_RELB = R_RSHD*R_REB;
        p_rw = p_reb + R_RELB*offset_eb2w;
        R_RHAND = R_RELB*R_RWY*R_RWP*R_RF1;

        p_ls = p_sc + R_TORSO*offset_sc2ls;
        R_LSHD = R_TORSO*R_LSP*R_LSR*R_LSY;
        p_leb = p_ls + R_LSHD*offset_s2eb;
        R_LELB = R_LSHD*R_LEB;
        p_lw = p_leb + R_LELB*offset_eb2w;
        R_LHAND = R_LELB*R_LWY*R_LWP*R_LF1;

        pm_PEL = p_pel + R_PEL*c_pel;
        pm_TORSO = p_pel + R_TORSO*c_torso;

        pm_RSP = p_rs + R_TORSO*R_RSP*c_rsp;
        pm_RSR = p_rs + R_TORSO*R_RSP*R_RSR*c_rsr;
        pm_RSY = p_rs + R_RSHD*c_rsy;
        pm_REB = p_reb + R_RELB*c_reb;
        pm_RWY = p_rw + R_RELB*R_RWY*c_rwy;
        pm_RWP = p_rw + R_RELB*R_RWY*R_RWP*c_rwp;
        pm_RF1 = p_rw + R_RHAND*c_rf1;

        pm_LSP = p_ls + R_TORSO*R_LSP*c_lsp;
        pm_LSR = p_ls + R_TORSO*R_LSP*R_LSR*c_lsr;
        pm_LSY = p_ls + R_LSHD*c_lsy;
        pm_LEB = p_leb + R_LELB*c_leb;
        pm_LWY = p_lw + R_LELB*R_LWY*c_lwy;
        pm_LWP = p_lw + R_LELB*R_LWY*R_LWP*c_lwp;
        pm_LF1 = p_lw + R_LHAND*c_lf1;

        // calculate
        temp_1 = pm_RHY*m_rhy + pm_RHR*m_rhr + pm_RHP*m_rhp + pm_RKN*m_rkn + pm_RAP*m_rap + pm_RAR*m_rar;
        temp_2 = pm_LHY*m_lhy + pm_LHR*m_lhr + pm_LHP*m_lhp + pm_LKN*m_lkn + pm_LAP*m_lap + pm_LAR*m_lar;
        pCOM_LB = (temp_1 + temp_2)/m_lb;

        temp_1 = pm_RSP*m_rsp + pm_RSR*m_rsr + pm_RSY*m_rsy + pm_REB*m_reb + pm_RWY*m_rwy + pm_RWP*m_rwp + pm_RF1*m_rf1;
        temp_2 = pm_LSP*m_lsp + pm_LSR*m_lsr + pm_LSY*m_lsy + pm_LEB*m_leb + pm_LWY*m_lwy + pm_LWP*m_lwp + pm_LF1*m_lf1;
        temp_3 = pm_PEL*m_pel + pm_TORSO*m_torso;
        pCOM_UB = (temp_1 + temp_2 + temp_3)/m_ub;

        // COM wrt PELV
        pCOM_TOT = (pCOM_LB*m_lb + pCOM_UB*m_ub)/m_tot;

        // Init
        if(FLAG_PELVIS_ESTIMATION_ISFIRST == true){
            FT_R(0,0) = sharedSEN->FT[0].Fx;
            FT_R(1,0) = sharedSEN->FT[0].Fy;
            FT_R(2,0) = sharedSEN->FT[0].Fz;
            FT_R(3,0) = sharedSEN->FT[0].Mx;
            FT_R(4,0) = sharedSEN->FT[0].My;
            FT_R(5,0) = sharedSEN->FT[0].Mz;

            FT_L(0,0) = sharedSEN->FT[1].Fx;
            FT_L(1,0) = sharedSEN->FT[1].Fy;
            FT_L(2,0) = sharedSEN->FT[1].Fz;
            FT_L(3,0) = sharedSEN->FT[1].Mx;
            FT_L(4,0) = sharedSEN->FT[1].My;
            FT_L(5,0) = sharedSEN->FT[1].Mz;

            prev_p_rf = p_rf;
            prev_p_lf = p_lf;
            prev_PHASE_MODE = 0;

            FLAG_PELVIS_ESTIMATION_ISFIRST = false;
        }

        // FT sensor value
        FT_CUR_R(0,0) = sharedSEN->FT[0].Fx;
        FT_CUR_R(1,0) = sharedSEN->FT[0].Fy;
        FT_CUR_R(2,0) = sharedSEN->FT[0].Fz;
        FT_CUR_R(3,0) = sharedSEN->FT[0].Mx;
        FT_CUR_R(4,0) = sharedSEN->FT[0].My;
        FT_CUR_R(5,0) = sharedSEN->FT[0].Mz;

        FT_CUR_L(0,0) = sharedSEN->FT[1].Fx;
        FT_CUR_L(1,0) = sharedSEN->FT[1].Fy;
        FT_CUR_L(2,0) = sharedSEN->FT[1].Fz;
        FT_CUR_L(3,0) = sharedSEN->FT[1].Mx;
        FT_CUR_L(4,0) = sharedSEN->FT[1].My;
        FT_CUR_L(5,0) = sharedSEN->FT[1].Mz;

        FT_R = FT_alpha*FT_R + (1.-FT_alpha)*FT_CUR_R;
        FT_L = FT_alpha*FT_L + (1.-FT_alpha)*FT_CUR_L;

        if(FT_R(2,0) < 0.)
            FT_R(2,0) = 0.01;
        if(FT_L(2,0) < 0.)
            FT_L(2,0) = 0.01;

        double FT_th = 50;//hyobin
        if(FT_R(2,0) > FT_th){
            if(FT_L(2,0) > FT_th){
                PHASE_MODE = 0;//DSP
            }else{
                PHASE_MODE = -500;//RSP
            }
        }else{
            if(FT_L(2,0) > FT_th){
                PHASE_MODE = 500;//LSP
            }else{
                PHASE_MODE = 0;
                cout<<"...Floating Status...!!!"<<endl;
                ;//FLOATING
            }
        }

        if(prev_PHASE_MODE == PHASE_MODE){
            ref_point = ref_point;
        }else{
            if(prev_PHASE_MODE == 500){
                if(PHASE_MODE == 0)
                    ref_point = ref_point + (p_rf - p_lf)/2.;
                else if(PHASE_MODE == -500)
                    ref_point = ref_point + (p_rf - p_lf);
            }else if(prev_PHASE_MODE == 0){
                if(PHASE_MODE == 500)
                    ref_point = ref_point + (p_lf - p_rf)/2.;
                else if(PHASE_MODE == -500)
                    ref_point = ref_point + (p_rf - p_lf)/2.;
            }else if(prev_PHASE_MODE == -500){
                if(PHASE_MODE == 500)
                    ref_point = ref_point + (p_lf - p_rf);
                else if(PHASE_MODE == 0)
                    ref_point = ref_point + (p_lf - p_rf)/2.;
            }
        }

        if(PHASE_MODE == 500)    //left foot support
            cur_pelvis = ref_point - p_lf;
        else if(PHASE_MODE == 0) //dsp
            cur_pelvis = ref_point - (p_lf + p_rf)/2.;
        else                    //right foot support
            cur_pelvis = ref_point - p_rf;

        CALC_COM = cur_pelvis + pCOM_TOT;

        cur_rf = cur_pelvis + p_rf;
        cur_lf = cur_pelvis + p_lf;

        CALC_ZMP(0,0) = (-FT_L(4,0) - FT_R(4,0) + cur_lf(0,0)*FT_L(2,0) + cur_rf(0,0)*FT_R(2,0))/(FT_L(2,0) + FT_R(2,0));
        CALC_ZMP(1,0) = (+FT_L(3,0) + FT_R(3,0) + cur_lf(1,0)*FT_L(2,0) + cur_rf(1,0)*FT_R(2,0))/(FT_L(2,0) + FT_R(2,0));

        prev_PHASE_MODE = PHASE_MODE;
        prev_p_rf = p_rf;
        prev_p_lf = p_lf;
    }
}

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "StateEstimation");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    jCon->SetMotionOwner(0);
    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case SE_CALC_TEST:
        {
            FILE_LOG(logSUCCESS) << "Command SE_CALC_TEST received..";
            //jCon->RefreshToCurrentReference();
            //jCon->SetAllMotionOwner();
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SE_NO_ACT;
            break;
        }
        case SE_EST_AXIS:
        {
            char axis_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            if(axis_mode == 0){
                TARGET_DIR = 0;
                cout<<"Estimation axis : X-axis...!!!"<<endl;
            }else if(axis_mode == 1){
                TARGET_DIR = 1;
                cout<<"Estimation axis : Y-axis...!!!"<<endl;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SE_NO_ACT;
            break;
        }
        case SE_EST_START://hyobin
        {
            FILE_LOG(logSUCCESS) << "Command SE_EST_START received..";

            //saveFlag = 1;

            DLKF_INIT_PARAMETER();
            DLKF_INITIALIZE();

            FILE_LOG(logSUCCESS) << "Estimation Initialization finished";

            FLAG_PELVIS_ESTIMATION_ISFIRST = true;
            FLAG_PELVIS_ESTIMATION = true;
            FLAG_COM_ESTIMATION = true;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SE_NO_ACT;
            break;
        }
        case SE_EST_FINISH://hyobin
        {
            FILE_LOG(logSUCCESS) << "Command SE_EST_FINISH received..";
            FLAG_COM_ESTIMATION = false;
            FLAG_PELVIS_ESTIMATION = false;

            FLAG_PELVIS_ESTIMATION_ISFIRST = true;

            //double avg_time = tick_time_sum / ((double)(tickCount));
            //tick_time_sum = 0;
            //tickCount = 0;
            //cout<<"AVG time : "<<avg_time<<endl;

            //SaveFile();
            FILE_LOG(logSUCCESS) << "Save done...!!!";
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = SE_NO_ACT;
        }
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}

//==============================//
// Task Thread
//==============================//
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        if(FLAG_PELVIS_ESTIMATION == true && FLAG_COM_ESTIMATION == true){
            //setStart();
            //PELVIS_ESTIMATION();  //hyobin modi
            DLKF_ESTIMATION();
            //setEnd();

//            tick_time_sum +=checkDiff();
//            tickCount++;

        }

        // send to SHM

        // hyobin
        sharedCMD->StateEstimate.X = x(0,0);
        sharedCMD->StateEstimate.dX = x(1,0);
        sharedCMD->StateEstimate.Px = x(2,0);
        sharedCMD->StateEstimate.Y = y(0,0);
        sharedCMD->StateEstimate.dY = y(1,0);
        sharedCMD->StateEstimate.Py = y(2,0);

        // x(0,0) : COM x pos
        // x(1,0) : COM x vel
        // x(2,0) : ZMP x
        // y same as x

        // cur_pelvis(0,0)~(2,0) : pelvis position x y z
        // PHASE_MODE : 0= DSP 500:LSP -500:RSP

        if(saveFlag == 1)
        {
            DataBuf[0][saveIndex]=saveIndex;

            DataBuf[1][saveIndex]=PHASE_MODE;

            DataBuf[2][saveIndex]=CALC_ZMP(0,0);
            DataBuf[3][saveIndex]=CALC_ZMP(1,0);

            DataBuf[4][saveIndex]=cur_pelvis(0,0);
            DataBuf[5][saveIndex]=cur_pelvis(1,0);
            DataBuf[6][saveIndex]=cur_pelvis(2,0);

            DataBuf[7][saveIndex]=CALC_COM(0,0);
            DataBuf[8][saveIndex]=CALC_COM(1,0);
            DataBuf[9][saveIndex]=CALC_COM(2,0);

            DataBuf[10][saveIndex]=x(0,0);
            DataBuf[11][saveIndex]=x(1,0);
            DataBuf[12][saveIndex]=x(2,0);

            DataBuf[13][saveIndex]=y(0,0);
            DataBuf[14][saveIndex]=y(1,0);
            DataBuf[15][saveIndex]=y(2,0);

//            // For raw data grabing
//            DataBuf[0][saveIndex] = saveIndex;

//            DataBuf[1][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RHY]->MCId][jCon->Joints[RHY]->MCCh].CurrentPosition;
//            DataBuf[2][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RHR]->MCId][jCon->Joints[RHR]->MCCh].CurrentPosition;
//            DataBuf[3][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RHP]->MCId][jCon->Joints[RHP]->MCCh].CurrentPosition;
//            DataBuf[4][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RKN]->MCId][jCon->Joints[RKN]->MCCh].CurrentPosition;
//            DataBuf[5][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RAP]->MCId][jCon->Joints[RAP]->MCCh].CurrentPosition;
//            DataBuf[6][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RAR]->MCId][jCon->Joints[RAR]->MCCh].CurrentPosition;

//            DataBuf[7][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LHY]->MCId][jCon->Joints[LHY]->MCCh].CurrentPosition;
//            DataBuf[8][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LHR]->MCId][jCon->Joints[LHR]->MCCh].CurrentPosition;
//            DataBuf[9][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LHP]->MCId][jCon->Joints[LHP]->MCCh].CurrentPosition;
//            DataBuf[10][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LKN]->MCId][jCon->Joints[LKN]->MCCh].CurrentPosition;
//            DataBuf[11][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LAP]->MCId][jCon->Joints[LAP]->MCCh].CurrentPosition;
//            DataBuf[12][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LAR]->MCId][jCon->Joints[LAR]->MCCh].CurrentPosition;

//            DataBuf[13][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RSP]->MCId][jCon->Joints[RSP]->MCCh].CurrentPosition;
//            DataBuf[14][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RSR]->MCId][jCon->Joints[RSR]->MCCh].CurrentPosition;
//            DataBuf[15][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RSY]->MCId][jCon->Joints[RSY]->MCCh].CurrentPosition;
//            DataBuf[16][saveIndex] = sharedSEN->ENCODER[jCon->Joints[REB]->MCId][jCon->Joints[REB]->MCCh].CurrentPosition;
//            DataBuf[17][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RWY]->MCId][jCon->Joints[RWY]->MCCh].CurrentPosition;
//            DataBuf[18][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RWP]->MCId][jCon->Joints[RWP]->MCCh].CurrentPosition;
//            DataBuf[19][saveIndex] = sharedSEN->ENCODER[jCon->Joints[RF1]->MCId][jCon->Joints[RF1]->MCCh].CurrentPosition;

//            DataBuf[20][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LSP]->MCId][jCon->Joints[LSP]->MCCh].CurrentPosition;
//            DataBuf[21][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LSR]->MCId][jCon->Joints[LSR]->MCCh].CurrentPosition;
//            DataBuf[22][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LSY]->MCId][jCon->Joints[LSY]->MCCh].CurrentPosition;
//            DataBuf[23][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LEB]->MCId][jCon->Joints[LEB]->MCCh].CurrentPosition;
//            DataBuf[24][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LWY]->MCId][jCon->Joints[LWY]->MCCh].CurrentPosition;
//            DataBuf[25][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LWP]->MCId][jCon->Joints[LWP]->MCCh].CurrentPosition;
//            DataBuf[26][saveIndex] = sharedSEN->ENCODER[jCon->Joints[LF1]->MCId][jCon->Joints[LF1]->MCCh].CurrentPosition;

//            DataBuf[27][saveIndex] = sharedSEN->ENCODER[jCon->Joints[WST]->MCId][jCon->Joints[WST]->MCCh].CurrentPosition;

//            DataBuf[28][saveIndex] = sharedSEN->FT[0].Fx;
//            DataBuf[29][saveIndex] = sharedSEN->FT[0].Fy;
//            DataBuf[30][saveIndex] = sharedSEN->FT[0].Fz;
//            DataBuf[31][saveIndex] = sharedSEN->FT[0].Mx;
//            DataBuf[32][saveIndex] = sharedSEN->FT[0].My;
//            DataBuf[33][saveIndex] = sharedSEN->FT[0].Mz;

//            DataBuf[34][saveIndex] = sharedSEN->FT[1].Fx;
//            DataBuf[35][saveIndex] = sharedSEN->FT[1].Fy;
//            DataBuf[36][saveIndex] = sharedSEN->FT[1].Fz;
//            DataBuf[37][saveIndex] = sharedSEN->FT[1].Mx;
//            DataBuf[38][saveIndex] = sharedSEN->FT[1].My;
//            DataBuf[39][saveIndex] = sharedSEN->FT[1].Mz;

//            DataBuf[40][saveIndex] = sharedSEN->FOG.Roll;
//            DataBuf[41][saveIndex] = sharedSEN->FOG.Pitch;
//            DataBuf[42][saveIndex] = sharedSEN->FOG.Yaw;
//            DataBuf[43][saveIndex] = sharedSEN->FOG.RollVel;
//            DataBuf[44][saveIndex] = sharedSEN->FOG.PitchVel;
//            DataBuf[45][saveIndex] = sharedSEN->FOG.YawVel;

//            DataBuf[46][saveIndex]=PHASE_MODE;
//            DataBuf[47][saveIndex]=CALC_ZMP(TARGET_DIR,0);
//            DataBuf[48][saveIndex]=cur_pelvis(0,0);
//            DataBuf[49][saveIndex]=cur_pelvis(1,0);
//            DataBuf[50][saveIndex]=cur_pelvis(2,0);

//            DataBuf[51][saveIndex]=CALC_COM(0,0);
//            DataBuf[52][saveIndex]=CALC_COM(1,0);
//            DataBuf[53][saveIndex]=CALC_COM(2,0);

//            DataBuf[54][saveIndex]=x(0,0);
//            DataBuf[55][saveIndex]=x(1,0);
//            DataBuf[56][saveIndex]=x(2,0);


            saveIndex++;
        }

        //jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()||1){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}
//==============================//


//==============================//
// File save
//==============================//
void SaveFile(void)
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("StateEstimation.txt", "w");

    for(i=0 ; i<saveIndex ; i++)
    {
        for(j=0 ; j<16 ; j++)//16 vs 57
            fprintf(fp, "%f\t", DataBuf[j][i]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    saveIndex=0;
    saveFlag=0;
}
//==============================//
