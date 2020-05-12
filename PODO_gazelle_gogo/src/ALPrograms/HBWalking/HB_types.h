#ifndef HB_TYPES
#define HB_TYPES

#include "BasicMatrix.h"
#include "rbdl/rbdl.h"

#define qDOF        18
#define dqDOF       19
// made by GG
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


struct LJointSpacePOS
{
    union {
        struct{
            double RHY,RHR,RHP,RKN,RAP,RAR;
            double LHY,LHR,LHP,LKN,LAP,LAR;
        };
        double JSP_Array[12];
    };
    vec3 pPel;
    quat qPel;
    double RA1, RA2, LA1, LA2;

    //Member Function
    VectorNd getQnow(int sizeof_q){
        VectorNd _Qnow = VectorNd::Zero(sizeof_q);

        for(int i=6;i<sizeof_q - 1;i++){
            _Qnow[i] = JSP_Array[i-6];
//            cout<<"i : "<<i<<" q: "<<_Qnow[i]<<endl;
//            cout<<"i : "<<i<<" JSP.JSP_Array[i]: "<<JSP.JSP_Array[i]<<endl;
        }

        _Qnow[0] = pPel.x;
        _Qnow[1] = pPel.y;
        _Qnow[2] = pPel.z;

        _Qnow[3] = qPel.x;
        _Qnow[4] = qPel.y;
        _Qnow[5] = qPel.z;
        _Qnow[sizeof_q-1] = qPel.w;

        return _Qnow;
    }

};
struct LJointSpaceVEL
{

    union{
        struct{
            double dRHY,dRHR,dRHP,dRKN,dRAP,dRAR;
            double dLHY,dLHR,dLHP,dLKN,dLAP,dLAR;
        };
        double JSV_Array[12];
    };

    vec3 dpPel;
    vec3 dqPel;

    double dRA1, dRA2, dLA1, dLA2;

    //Member Function
    VectorNd getdQnow(int sizeof_dq){
        VectorNd _dQnow = VectorNd::Zero(sizeof_dq);
        _dQnow[0] = dpPel.x;
        _dQnow[1] = dpPel.y;
        _dQnow[2] = dpPel.z;

        _dQnow[3] = dqPel.x;
        _dQnow[4] = dqPel.y;
        _dQnow[5] = dqPel.z;

        for(int i=6;i<sizeof_dq;i++){
            _dQnow[i] = JSV_Array[i-6];
        }

        return _dQnow;
    }
};
struct LJointSpaceACC
{
    union{
        struct{
            double ddRHY,ddRHR,ddRHP,ddRKN,ddRAP,ddRAR;
            double ddLHY,ddLHR,ddLHP,ddLKN,ddLAP,ddLAR;
        };
        double JSA_Array[12];
    };
    vec3 ddpPel;
    vec3 ddqPel;
};



struct LCartesianSpacePOS
{
    vec3 pRF;
    quat qRF;
    vec3 pLF;
    quat qLF;
    vec3 pPel;
    quat qPel;

    vec3 pCOM;
};
struct LCartesianSpaceVEL
{
    vec3 dpRF;
    vec3 dqRF;
    vec3 dpLF;
    vec3 dqLF;
    vec3 dpPel;
    vec3 dqPel;

    vec3 dpCOM;
};
struct LCartesianSpaceACC
{
    vec3 ddpRF;
    vec3 ddqRF;
    vec3 ddpLF;
    vec3 ddqLF;
    vec3 ddpPel;
    vec3 ddqPel;

    vec3 ddpCOM;
};

typedef union _LJointTorque{
    struct{
        double RHY, RHR, RHP, RKN, RAP, RAR,
                LHY, LHR, LHP, LKN, LAP, LAR;
    };
    double Torque[12];
}LJointTorque;

typedef struct _RobotSensor{
    //IMU
    vec3 IMUangle;
    vec3 IMUangle_comp;
    vec3 IMUomega;
    quat IMUquat;
    vec3 IMULocalW;
    //Encoder
    LJointSpacePOS JSP;
    LJointSpaceVEL JSV;

    //Force Torque Sensors
    vec3 F_RF,F_LF;
    vec3 M_RF,M_LF;
    vec3 ACC_RF, ACC_LF;
}RobotSensor;

struct RobotStates
{
    //IMU
    vec3 IMUangle;
    vec3 IMUangle_comp;
    vec3 IMUomega;
    quat IMUquat;
    vec3 IMULocalW;
    //Encoder
    LJointSpacePOS JSP;
    LJointSpaceVEL JSV;
    //Position
    LCartesianSpacePOS CSP;
    LCartesianSpaceVEL CSV;

    VectorNd Qnow;
    VectorNd dQnow;

    VectorNd Xnow;
    VectorNd dXnow;

    //Force Torque Sensors
    vec3 F_RF,F_LF;
    vec3 M_RF,M_LF;

    vec3 ACC_RF, ACC_LF;

    bool cRF, cLF;

    //Member Functions
    VectorNd getQnow(int sizeof_q)
    {
        VectorNd _Qnow = VectorNd::Zero(sizeof_q);

        for(int i=6;i<sizeof_q - 1;i++)
        {
            _Qnow[i] = JSP.JSP_Array[i-6];
//            cout<<"i : "<<i<<" q: "<<_Qnow[i]<<endl;
//            cout<<"i : "<<i<<" JSP.JSP_Array[i]: "<<JSP.JSP_Array[i]<<endl;
        }

        _Qnow[0] = JSP.pPel.x;
        _Qnow[1] = JSP.pPel.y;
        _Qnow[2] = JSP.pPel.z;

        _Qnow[3] = JSP.qPel.x;
        _Qnow[4] = JSP.qPel.y;
        _Qnow[5] = JSP.qPel.z;
        _Qnow[sizeof_q-1] = JSP.qPel.w;

        return _Qnow;
    }

    VectorNd getdQnow(int sizeof_dq){
        VectorNd _dQnow = VectorNd::Zero(sizeof_dq);
        _dQnow[0] = JSV.dpPel.x;
        _dQnow[1] = JSV.dpPel.y;
        _dQnow[2] = JSV.dpPel.z;

        _dQnow[3] = JSV.dqPel.x;
        _dQnow[4] = JSV.dqPel.y;
        _dQnow[5] = JSV.dqPel.z;

        for(int i=6;i<sizeof_dq;i++){
            _dQnow[i] = JSV.JSV_Array[i-6];
        }

        return _dQnow;
    }

};

struct REFERENCE
{
    LJointSpacePOS JSP;
    LJointSpaceVEL JSV;
    LJointSpaceACC JSA;

    LCartesianSpacePOS CSP;
    LCartesianSpaceVEL CSV;
    LCartesianSpaceACC CSA;

    VectorNd Qref;
    VectorNd dQref;
    VectorNd ddQref;

    bool cRF, cLF;
    bool RFup, RFdn, LFup, LFdn;

    //Member Functions
    VectorNd getQref(int sizeof_q){
        VectorNd _Qref = VectorNd::Zero(sizeof_q);
        _Qref[0] = JSP.pPel.x;
        _Qref[1] = JSP.pPel.y;
        _Qref[2] = JSP.pPel.z;

        _Qref[3] = JSP.qPel.x;
        _Qref[4] = JSP.qPel.y;
        _Qref[5] = JSP.qPel.z;
        _Qref[sizeof_q-1] = JSP.qPel.w;

        for(int i=6;i<sizeof_q - 1;i++){
            _Qref[i] = JSP.JSP_Array[i-6];
        }

        return _Qref;
    }
    VectorNd getdQref(int sizeof_dq){
        VectorNd _dQref = VectorNd::Zero(sizeof_dq);
        _dQref[0] = JSV.dpPel.x;
        _dQref[1] = JSV.dpPel.y;
        _dQref[2] = JSV.dpPel.z;

        _dQref[3] = JSV.dqPel.x;
        _dQref[4] = JSV.dqPel.y;
        _dQref[5] = JSV.dqPel.z;

        for(int i=6;i<sizeof_dq;i++){
            _dQref[i] = JSV.JSV_Array[i-6];
        }

        return _dQref;
    }
    VectorNd getddQref(int sizeof_dq){
        VectorNd _ddQref = VectorNd::Zero(sizeof_dq);
        _ddQref[0] = JSA.ddpPel.x;
        _ddQref[1] = JSA.ddpPel.y;
        _ddQref[2] = JSA.ddpPel.z;

        _ddQref[3] = JSA.ddqPel.x;
        _ddQref[4] = JSA.ddqPel.y;
        _ddQref[5] = JSA.ddqPel.z;

        for(int i=6;i<sizeof_dq;i++){
            _ddQref[i] = JSA.JSA_Array[i-6];
        }

        return _ddQref;
    }

};


struct DesiredStates
{
    VectorNd Xdes;

    VectorNd ddQdes;
    VectorNd Tdes;
    VectorNd Fdes;
};


#endif // HB_TYPES

