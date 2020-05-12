#ifndef BP_RBDL_H
#define BP_RBDL_H
#include "HB_inverse.h"
#include "rbdl/rbdl.h"
#include "Eigen/Geometry"
#include <vector>
#include "HB_types.h"
#include "ow_cplex.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;
class BP_RBDL
{
    OW_CPLEX OC;
public:
    Model* Robot;
    Body b_torso, b_pel;
    Body b_rhy, b_rhr, b_rhp, b_rkn, b_rap, b_rar;
    Body b_lhy, b_lhr, b_lhp, b_lkn, b_lap, b_lar;
    Body b_rsp, b_rsr, b_rsy, b_reb, b_rwy, b_rwp, b_rf1;
    Body b_lsp, b_lsr, b_lsy, b_leb, b_lwy, b_lwp, b_lf1;

    Joint j_world;
    Joint j_wst;
    Joint j_rhy, j_rhr, j_rhp, j_rkn, j_rap, j_rar;
    Joint j_lhy, j_lhr, j_lhp, j_lkn, j_lap, j_lar;
    Joint j_rsp, j_rsr, j_rsy, j_reb, j_rwy, j_rwp, j_rf1;
    Joint j_lsp, j_lsr, j_lsy, j_leb, j_lwy, j_lwp, j_lf1;

    int n_torso,n_pel;
    int n_rhy, n_rhr, n_rhp, n_rkn, n_rap, n_rar;
    int n_lhy, n_lhr, n_lhp, n_lkn, n_lap, n_lar;
    int n_rsp, n_rsr, n_rsy, n_reb, n_rwy, n_rwp, n_rf1;
    int n_lsp, n_lsr, n_lsy, n_leb, n_lwy, n_lwp, n_lf1;

    //Endeffector Jacobian
    MatrixNd JacobianRF6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianLF6D = MatrixNd::Zero(6,18);

    MatrixNd JacobianRF3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianLF3D = MatrixNd::Zero(3,18);

    MatrixNd JacobianRF3D_pos = MatrixNd::Zero(3,18);
    MatrixNd JacobianLF3D_pos = MatrixNd::Zero(3,18);
    MatrixNd JacobianRF3D_ori = MatrixNd::Zero(3,18);
    MatrixNd JacobianLF3D_ori = MatrixNd::Zero(3,18);

    //COM Jacobian
    MatrixNd JacobianCOMpel6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMrhy6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMrhr6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMrhp6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMrkn6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMrap6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMrar6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMlhy6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMlhr6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMlhp6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMlkn6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMlap6D = MatrixNd::Zero(6,18);
    MatrixNd JacobianCOMlar6D = MatrixNd::Zero(6,18);
//    MatrixNd JacobianCOMall6D;

    MatrixNd JacobianCOMpel3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMrhy3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMrhr3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMrhp3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMrkn3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMrap3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMrar3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMlhy3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMlhr3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMlhp3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMlkn3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMlap3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMlar3D = MatrixNd::Zero(3,18);
    MatrixNd JacobianCOMall3D = MatrixNd::Zero(3,18);

    MatrixNd ContactJacobian, ContactJacobian_pos, ContactJacobian_ori;

    MatrixNd JQref, JQref_inv;

    VectorNd Tau = VectorNd::Zero(18);
    VectorNd X; //[ddQ tau F]'
    int n_con_old;
    bool cRF_old, cLF_old;




    HB_inverse Hi;

    Vector3d v2V(vec3 in)
    {
        return Vector3d(in.x,in.y,in.z);
    }

    VectorNd q2V(quat in)
    {

        VectorNd _V = VectorNd::Zero(4);
        _V[0] = in[0];
        _V[1] = in[1];
        _V[2] = in[2];
        _V[3] = in[3];
        return _V;
    }

    Matrix3d m2M(mat3 in)
    {
        Matrix3d out;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                out(i,j) = in[i][j];
            }
        }
        return out;
    }

public:
    BP_RBDL()
    {
        Robot = new Model();
        Robot->gravity = Vector3d(0,0,-9.81);

        //BODY_body
        b_pel = Body(Hi.m_pel,v2V(Hi.c_pel),m2M(Hi.I_pel));//<-------------------------------only for lumped mass model
        //         b_torso = Body(Hi.m_torso,v2V(Hi.c_torso),m2M(Hi.I_torso));

        //BODY_legs
        b_rhy = Body(Hi.m_rhy,v2V(Hi.c_rhy),m2M(Hi.I_rhy));
        b_rhr = Body(Hi.m_rhr,v2V(Hi.c_rhr),m2M(Hi.I_rhr));
        b_rhp = Body(Hi.m_rhp,v2V(Hi.c_rhp),m2M(Hi.I_rhp));
        b_rkn = Body(Hi.m_rkn,v2V(Hi.c_rkn),m2M(Hi.I_rkn));
        b_rap = Body(Hi.m_rap,v2V(Hi.c_rap),m2M(Hi.I_rap));
        b_rar = Body(Hi.m_rar,v2V(Hi.c_rar),m2M(Hi.I_rar));

        b_lhy = Body(Hi.m_lhy,v2V(Hi.c_lhy),m2M(Hi.I_lhy));
        b_lhr = Body(Hi.m_lhr,v2V(Hi.c_lhr),m2M(Hi.I_lhr));
        b_lhp = Body(Hi.m_lhp,v2V(Hi.c_lhp),m2M(Hi.I_lhp));
        b_lkn = Body(Hi.m_lkn,v2V(Hi.c_lkn),m2M(Hi.I_lkn));
        b_lap = Body(Hi.m_lap,v2V(Hi.c_lap),m2M(Hi.I_lap));
        b_lar = Body(Hi.m_lar,v2V(Hi.c_lar),m2M(Hi.I_lar));

        //JOINT world
        j_world = Joint(JointTypeFloatingBase);

        // j_waists = Joint(JointTypeRevoluteZ);
        //         j_wst = Joint(JointTypeRevoluteZ);

        //JOINT_legs
        j_rhy = Joint(JointTypeRevoluteZ);
        j_rhr = Joint(JointTypeRevoluteX);
        j_rhp = Joint(JointTypeRevoluteY);
        j_rkn = Joint(JointTypeRevoluteY);
        j_rap = Joint(JointTypeRevoluteY);
        j_rar = Joint(JointTypeRevoluteX);

        j_lhy = Joint(JointTypeRevoluteZ);
        j_lhr = Joint(JointTypeRevoluteX);
        j_lhp = Joint(JointTypeRevoluteY);
        j_lkn = Joint(JointTypeRevoluteY);
        j_lap = Joint(JointTypeRevoluteY);
        j_lar = Joint(JointTypeRevoluteX);


        Vector3d zv = Vector3d(0.0,0.0,0.0);
        n_pel = Robot->AddBody(0,Xtrans(zv),j_world,b_pel,"PEL");

        //         n_torso = Robot->AddBody(n_pel,Xtrans(v2V(Hi.offset_p2p)),j_wst,b_torso,"TOR");
        //robot_rl
        n_rhy = Robot->AddBody(n_pel,Xtrans(v2V(Hi.offset_p2rHR)),j_rhy,b_rhy,"RHY");
        n_rhr = Robot->AddBody(n_rhy,Xtrans(zv),j_rhr,b_rhr,"RHR");
        n_rhp = Robot->AddBody(n_rhr,Xtrans(v2V(Hi.offset_rHR2rHP)),j_rhp,b_rhp,"RHP");
        n_rkn = Robot->AddBody(n_rhp,Xtrans(v2V(Hi.offset_uleg)),j_rkn,b_rkn,"RKN");
        n_rap = Robot->AddBody(n_rkn,Xtrans(v2V(Hi.offset_lleg)),j_rap,b_rap,"RAP");//NAN here... why????
        n_rar = Robot->AddBody(n_rap,Xtrans(zv),j_rar,b_rar,"RAR");
        //robot_ll
        n_lhy = Robot->AddBody(n_pel,Xtrans(v2V(Hi.offset_p2lHR)),j_lhy,b_lhy,"LHY");
        n_lhr = Robot->AddBody(n_lhy,Xtrans(zv),j_lhr,b_lhr,"LHR");
        n_lhp = Robot->AddBody(n_lhr,Xtrans(v2V(Hi.offset_lHR2lHP)),j_lhp,b_lhp,"LHP");
        n_lkn = Robot->AddBody(n_lhp,Xtrans(v2V(Hi.offset_uleg)),j_lkn,b_lkn,"LKN");
        n_lap = Robot->AddBody(n_lkn,Xtrans(v2V(Hi.offset_lleg)),j_lap,b_lap,"LAP");
        n_lar = Robot->AddBody(n_lap,Xtrans(zv),j_lar,b_lar,"LAR");

        n_con_old = 2;
        X = VectorNd::Zero(18+12+n_con_old*6);
    }




    void CalcInverseDynamics(VectorNd _Q,VectorNd _Qdot,VectorNd _Qddot)
    {
        VectorNd _Tau = VectorNd::Zero(Robot->dof_count);
        InverseDynamics(*Robot,_Q,_Qdot,_Qddot,_Tau);
        Tau = _Tau;
    }

    Vector3d COM;
    Vector3d dCOM;
    Vector3d Momentum;
    double Mass;

    void CalcCOMMomentum(VectorNd _Q,VectorNd _Qdot)
    {
//        Utils::CalcCenterOfMass(*Robot,_Q,_Qdot,Mass,COM,&dCOM,&Momentum,true);
    }

    void CalcEndeffectorJacobian6D(VectorNd _Q)
    {
        CalcPointJacobian6D(*Robot,_Q,n_rar,v2V(Hi.offset_foot),JacobianRF6D); // first 3 ori, last 3 position
        CalcPointJacobian6D(*Robot,_Q,n_lar,v2V(Hi.offset_foot),JacobianLF6D);

        JacobianRF3D_pos.block(0,0, 3,18) = JacobianRF6D.block(3,0, 3,18);
        JacobianLF3D_pos.block(0,0, 3,18) = JacobianLF6D.block(3,0, 3,18);

        JacobianRF3D_ori.block(0,0, 3,18) = JacobianRF6D.block(0,0, 3,18);
        JacobianLF3D_ori.block(0,0, 3,18) = JacobianLF6D.block(0,0, 3,18);
        //        std::cout << "JacobianRF6D \n" << JacobianRF6D << std::endl;
    }

    void CalcEndeffectorJacobian3D(VectorNd _Q)
    {
        CalcPointJacobian(*Robot,_Q,n_rar,v2V(Hi.offset_foot),JacobianRF3D);

        CalcPointJacobian(*Robot,_Q,n_lar,v2V(Hi.offset_foot),JacobianLF3D);
        //        std::cout << "JacobianRF3D \n" << JacobianRF3D << std::endl;
    }

    void CalcCOMJacobian3D(VectorNd _Q)
    {
        //get COM 3D jacobian
        CalcPointJacobian(*Robot,_Q,n_pel,v2V(Hi.c_pel),JacobianCOMpel3D);

        CalcPointJacobian(*Robot,_Q,n_rhy,v2V(Hi.c_rhy),JacobianCOMrhy3D);
        CalcPointJacobian(*Robot,_Q,n_rhr,v2V(Hi.c_rhr),JacobianCOMrhr3D);
        CalcPointJacobian(*Robot,_Q,n_rhp,v2V(Hi.c_rhp),JacobianCOMrhp3D);
        CalcPointJacobian(*Robot,_Q,n_rkn,v2V(Hi.c_rkn),JacobianCOMrkn3D);
        CalcPointJacobian(*Robot,_Q,n_rap,v2V(Hi.c_rap),JacobianCOMrap3D);
        CalcPointJacobian(*Robot,_Q,n_rar,v2V(Hi.c_rar),JacobianCOMrar3D);

        CalcPointJacobian(*Robot,_Q,n_lhy,v2V(Hi.c_lhy),JacobianCOMlhy3D);
        CalcPointJacobian(*Robot,_Q,n_lhr,v2V(Hi.c_lhr),JacobianCOMlhr3D);
        CalcPointJacobian(*Robot,_Q,n_lhp,v2V(Hi.c_lhp),JacobianCOMlhp3D);
        CalcPointJacobian(*Robot,_Q,n_lkn,v2V(Hi.c_lkn),JacobianCOMlkn3D);
        CalcPointJacobian(*Robot,_Q,n_lap,v2V(Hi.c_lap),JacobianCOMlap3D);
        CalcPointJacobian(*Robot,_Q,n_lar,v2V(Hi.c_lar),JacobianCOMlar3D);

        JacobianCOMall3D = (Hi.m_pel*JacobianCOMpel3D + Hi.m_rhy*JacobianCOMrhy3D + Hi.m_rhr*JacobianCOMrhr3D + Hi.m_rhp*JacobianCOMrhp3D + Hi.m_rkn*JacobianCOMrkn3D + Hi.m_rap*JacobianCOMrap3D + Hi.m_rar*JacobianCOMrar3D + Hi.m_lhy*JacobianCOMlhy3D + Hi.m_lhr*JacobianCOMlhr3D + Hi.m_lhp*JacobianCOMlhp3D + Hi.m_lkn*JacobianCOMlkn3D + Hi.m_lap*JacobianCOMlap3D +  Hi.m_lar*JacobianCOMlar3D)/(Hi.m_pel + Hi.m_rleg + Hi.m_lleg);
    }

    Vector3d CalcComAcceleration3D(VectorNd _Q, VectorNd _dQ, VectorNd _ddQ){
        Vector3d pel_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_pel, v2V(Hi.c_pel));

        Vector3d rhy_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_rhy, v2V(Hi.c_rhy));
        Vector3d rhr_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_rhr, v2V(Hi.c_rhr));
        Vector3d rhp_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_rhp, v2V(Hi.c_rhp));
        Vector3d rkn_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_rkn, v2V(Hi.c_rkn));
        Vector3d rap_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_rap, v2V(Hi.c_rap));
        Vector3d rar_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_rar, v2V(Hi.c_rar));

        Vector3d lhy_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_lhy, v2V(Hi.c_lhy));
        Vector3d lhr_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_lhr, v2V(Hi.c_lhr));
        Vector3d lhp_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_lhp, v2V(Hi.c_lhp));
        Vector3d lkn_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_lkn, v2V(Hi.c_lkn));
        Vector3d lap_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_lap, v2V(Hi.c_lap));
        Vector3d lar_Acc = CalcPointAcceleration(*Robot,_Q,_dQ, _ddQ ,n_lar, v2V(Hi.c_lar));

        Vector3d com_Acc = (Hi.m_pel*pel_Acc + Hi.m_rhy*rhy_Acc + Hi.m_rhr*rhr_Acc + Hi.m_rhp*rhp_Acc + Hi.m_rkn*rkn_Acc + Hi.m_rap*rap_Acc + Hi.m_rar*rar_Acc
                             + Hi.m_lhy*lhy_Acc + Hi.m_lhr*lhr_Acc + Hi.m_lhp*lhp_Acc + Hi.m_lkn*lkn_Acc + Hi.m_lap*lap_Acc + Hi.m_lar*lar_Acc)/(Hi.m_pel + Hi.m_rleg + Hi.m_lleg);
        return com_Acc;
    }

    void MakeContactJacobian_pos(bool _cRF, bool _cLF){
        if(_cRF == true && _cLF == false){
            ContactJacobian_pos = MatrixNd::Zero(3, Robot->dof_count);
            ContactJacobian_pos.block(0,0, 3,Robot->dof_count) = JacobianRF3D_pos;
        }
        else if(_cRF == false && _cLF == true){
            ContactJacobian_pos = MatrixNd::Zero(3, Robot->dof_count);
            ContactJacobian_pos.block(0,0, 3,Robot->dof_count) = JacobianLF3D_pos;
        }
        else if(_cRF == true && _cLF == true){
            ContactJacobian_pos = MatrixNd::Zero(6, Robot->dof_count);
            ContactJacobian_pos.block(0,0, 3,Robot->dof_count) = JacobianRF3D_pos;
            ContactJacobian_pos.block(3,0, 3,Robot->dof_count) = JacobianLF3D_pos;
        }
    }

    void MakeContactJacobian_ori(bool _cRF, bool _cLF){
        if(_cRF == true && _cLF == false){
            ContactJacobian_ori = MatrixNd::Zero(3, Robot->dof_count);
            ContactJacobian_ori.block(0,0, 3,Robot->dof_count) = JacobianRF3D_ori;
        }
        else if(_cRF == false && _cLF == true){
            ContactJacobian_ori = MatrixNd::Zero(3, Robot->dof_count);
            ContactJacobian_ori.block(0,0, 3,Robot->dof_count) = JacobianLF3D_ori;
        }
        else if(_cRF == true && _cLF == true){
            ContactJacobian_ori = MatrixNd::Zero(6, Robot->dof_count);
            ContactJacobian_ori.block(0,0, 3,Robot->dof_count) = JacobianRF3D_ori;
            ContactJacobian_ori.block(3,0, 3,Robot->dof_count) = JacobianLF3D_ori;
        }
    }

    int MakeContactJacobian(bool _cRF, bool _cLF){
        int n_con = 1;
        if(_cRF == true && _cLF == false){
            ContactJacobian = MatrixNd::Zero(6, Robot->dof_count);
            ContactJacobian.block(0,0, 3,Robot->dof_count) = JacobianRF3D_pos;
            ContactJacobian.block(3,0, 3,Robot->dof_count) = JacobianRF3D_ori;
            n_con = 1;
        }
        else if(_cRF == false && _cLF == true){
            ContactJacobian = MatrixNd::Zero(6, Robot->dof_count);
            ContactJacobian.block(0,0, 3,Robot->dof_count) = JacobianLF3D_pos;
            ContactJacobian.block(3,0, 3,Robot->dof_count) = JacobianLF3D_ori;
            n_con = 1;
        }
        else if(_cRF == true && _cLF == true){
            ContactJacobian = MatrixNd::Zero(12, Robot->dof_count);
            ContactJacobian.block(0,0, 3,Robot->dof_count) = JacobianRF3D_pos;
            ContactJacobian.block(3,0, 3,Robot->dof_count) = JacobianRF3D_ori;

            ContactJacobian.block(6,0, 3,Robot->dof_count) = JacobianLF3D_pos;
            ContactJacobian.block(9,0, 3,Robot->dof_count) = JacobianLF3D_ori;
            n_con = 2;
        }

        return n_con; // Nomber of Contact
    }
    
    VectorNd calc_QP(RobotStates _RS, REFERENCE _REF){
        VectorNd _Q = _RS.Qnow;
        VectorNd _dQ = _RS.dQnow;

        VectorNd _Qref = _REF.Qref;
        VectorNd _dQref = _REF.dQref;
        VectorNd _ddQref = _REF.ddQref;

        bool cRF = true;
        bool cLF = true;

        if(_REF.RFup == true ) cRF = false;
        if(_REF.RFdn == true && (_REF.cRF == true || _RS.cRF == true)) cRF = true;
        if(_REF.RFdn == true && (_REF.cRF == false && _RS.cRF == false)) cRF = false;

        if(_REF.LFup == true ) cLF = false;
        if(_REF.LFdn == true && (_REF.cLF == true || _RS.cLF == true)) cLF = true;
        if(_REF.LFdn == true && (_REF.cLF == false && _RS.cLF == false)) cLF = false;


        // Calc M matrix
        MatrixNd _M = MatrixNd::Zero (Robot->dof_count, Robot->dof_count);
        CompositeRigidBodyAlgorithm(*Robot,_Q,_M,true);

        // Calc b matrix
        VectorNd Nonlin = VectorNd::Zero (Robot->dof_count);
        NonlinearEffects(*Robot,_Q,_dQ,Nonlin);

        // Contact Jacobian & set ow cplex
        CalcEndeffectorJacobian6D(_Q); //JacobianRF3D_pos,ori  & JacobianLF3D_pos,ori
        int n_con = MakeContactJacobian(cRF, cLF); // calc ContactJacobian [pRF qRF pLF qLF]' return No of Contact (1 or 2)
        std::vector<MatrixNd> As, Bs;
        std::vector<double> Ws;

        MatrixNd A0, B0;
        double W0;

        OC.setNums(18 + 12 + 6*n_con, 18, 5*n_con + 4*n_con);
        OC.MakeEq(_M,Nonlin, ContactJacobian); //Dynamics
        OC.MakeIneq();  // Friction cone

        //cout<<"QP init setting done"<<endl;
        //cout<<"n_con: "<<n_con<<endl;

        ////----------------------------- Contact Constraints---------------------------------------
        // calc dJdQ
        VectorNd ddqZero = VectorNd::Zero(_dQ.size());
        SpatialVector RF_dJdQ_6d = CalcPointAcceleration6D(*Robot,_Q,_dQ,ddqZero,n_rar, v2V(Hi.offset_foot));
        Vector3d RF_dJdQ = Vector3d(RF_dJdQ_6d(3), RF_dJdQ_6d(4), RF_dJdQ_6d(5));
        Vector3d RF_dJRdQ = Vector3d(RF_dJdQ_6d(0), RF_dJdQ_6d(1), RF_dJdQ_6d(2));

        SpatialVector LF_dJdQ_6d = CalcPointAcceleration6D(*Robot,_Q,_dQ,ddqZero,n_lar, v2V(Hi.offset_foot));
        Vector3d LF_dJdQ = Vector3d(LF_dJdQ_6d(3), LF_dJdQ_6d(4), LF_dJdQ_6d(5));
        Vector3d LF_dJRdQ = Vector3d(LF_dJdQ_6d(0), LF_dJdQ_6d(1), LF_dJdQ_6d(2));

        // pos Constraints
        //ddx - dJdQ
        MatrixNd ddx_dJdQ = MatrixNd::Zero(3*n_con,1);

        if(cRF == true && n_con == 1){
            ddx_dJdQ.block(0,0, 3,1) = -RF_dJdQ;
        }
        else if(cLF == true && n_con == 1){
            ddx_dJdQ.block(0,0, 3,1) = -LF_dJdQ;
        }

        else if(n_con == 2){
            ddx_dJdQ.block(0,0, 3,1) = -RF_dJdQ;
            ddx_dJdQ.block(3,0,3,1) = -LF_dJdQ;
        }

        MakeContactJacobian_pos(cRF, cLF); // calc ContactJacobian_pos

        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        A0.block(0,0, ContactJacobian_pos.rows(), ContactJacobian_pos.cols()) = ContactJacobian_pos;
        B0.block(0,0, ddx_dJdQ.rows(), ddx_dJdQ.cols()) = ddx_dJdQ;
        W0 = 10;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //cout<<"1"<<endl;

        // ori Constraints (only roll & pitch)
//        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
//        B0 = MatrixNd::Zero(18+12+6*n_con,1);

//        MakeContactJacobian_ori(cRF, cLF); // calc ContactJacobian_ori

//        ddx_dJdQ = MatrixNd::Zero(3*n_con,1);
//        if(cRF == true && n_con == 1){
//            ddx_dJdQ.block(0,0, 3,1) = -RF_dJRdQ;

//            A0.block(0,0, 2, ContactJacobian_ori.cols()) = ContactJacobian_ori.block(0,0, 2,ContactJacobian_ori.cols());
//            B0.block(0,0, 2, 1) = ddx_dJdQ.block(0,0, 2,1);

//        }
//        else if(cLF == true && n_con == 1){
//            ddx_dJdQ.block(0,0, 3,1) = -LF_dJRdQ;

//            A0.block(0,0, 2, ContactJacobian_ori.cols()) = ContactJacobian_ori.block(0,0, 2,ContactJacobian_ori.cols());
//            B0.block(0,0, 2, 1) = ddx_dJdQ.block(0,0, 2,1);
//        }

//        else if(n_con == 2){
//            ddx_dJdQ(0,0) = -RF_dJRdQ(0);
//            ddx_dJdQ(1,0) = -RF_dJRdQ(1);
//            ddx_dJdQ(2,0) = -LF_dJRdQ(0);
//            ddx_dJdQ(3,0) = -LF_dJRdQ(1);

//            A0.block(0,0, 2, ContactJacobian_ori.cols()) = ContactJacobian_ori.block(0,0, 2,ContactJacobian_ori.cols());
//            A0.block(2,0, 2, ContactJacobian_ori.cols()) = ContactJacobian_ori.block(3,0, 2,ContactJacobian_ori.cols());

//            B0.block(0,0, 4,1) = ddx_dJdQ.block(0,0, 4,1);
//        }

//        W0 = 10;
//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

        ddx_dJdQ = MatrixNd::Zero(3*n_con,1);
        if(cRF == true && n_con == 1){
            ddx_dJdQ.block(0,0, 3,1) = -RF_dJRdQ;
        }
        else if(cLF == true && n_con == 1){
            ddx_dJdQ.block(0,0, 3,1) = -LF_dJRdQ;
        }

        else if(n_con == 2){
            ddx_dJdQ.block(0,0, 3,1) = -RF_dJRdQ;
            ddx_dJdQ.block(3,0, 3,1) = -LF_dJRdQ;
        }

        MakeContactJacobian_ori(cRF, cLF); // calc ContactJacobian_ori

        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        A0.block(0,0, ContactJacobian_ori.rows(), ContactJacobian_ori.cols()) = ContactJacobian_ori;
        B0.block(0,0, ddx_dJdQ.rows(), ddx_dJdQ.cols()) = ddx_dJdQ;
        W0 = 15;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        double wn;
        double z;

        //cout<<"2"<<endl;
        ////-------Swing Leg : Tracking the Joint trajectory of Swing leg-----------------------
//        z =1.0;
//        wn = 20;
//        int cnt4SL = 0;
//        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
//        B0 = MatrixNd::Zero(18+12+6*n_con,1);

//        double ddQfb[12];
//        for(int i=0;i<12;i++){
//            ddQfb[i] = 2*z*wn*(_dQref[i+6] - _dQ[i+6]) + wn*wn*(_Qref[i+6] - _Q[i+6]);
//            //joint Reference angle based control
//        }

//        if(!_cRF){ // RF swing phase
//            A0(0,6) = 1; B0(0,0) = ddQfb[0] + _ddQref[6];
//            A0(1,7) = 1; B0(1,0) = ddQfb[1] + _ddQref[7];
//            A0(2,8) = 1; B0(2,0) = ddQfb[2] + _ddQref[8];
//            A0(3,9) = 1; B0(3,0) = ddQfb[3] + _ddQref[9];
//            A0(4,10) = 1; B0(4,0) = ddQfb[4] + _ddQref[10];
//            A0(5,11) = 1; B0(5,0) = ddQfb[5] + _ddQref[11];
//            cnt4SL += 6;
//        }
//        if(!_cLF){ // Lf swing phase
//            A0(cnt4SL+0,12) = 1; B0(cnt4SL+0,0) = ddQfb[6] + _ddQref[12];
//            A0(cnt4SL+1,13) = 1; B0(cnt4SL+1,0) = ddQfb[7] + _ddQref[13];
//            A0(cnt4SL+2,14) = 1; B0(cnt4SL+2,0) = ddQfb[8] + _ddQref[14];
//            A0(cnt4SL+3,15) = 1; B0(cnt4SL+3,0) = ddQfb[9] + _ddQref[15];
//            A0(cnt4SL+4,16) = 1; B0(cnt4SL+4,0) = ddQfb[10] + _ddQref[16];
//            A0(cnt4SL+5,17) = 1; B0(cnt4SL+5,0) = ddQfb[11] + _ddQref[17];
//        }

//        W0 = 1e-2*100;
//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

        ////----------------------------------Tracking the Cartesian Space Trajectory of leg----------------------------footdd
        /// swing leg position tracking : right leg
        wn = 15;
        z = 1;
        A0 = MatrixNd::Zero(18 + 12 + 6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18 + 12 + 6*n_con,1);

        A0.block(0,0, JacobianRF3D_pos.rows(), JacobianRF3D_pos.cols()) = JacobianRF3D_pos;
        B0(0,0) = wn*wn*(_REF.CSP.pRF.x - _RS.CSP.pRF.x) + 2*z*wn*(_REF.CSV.dpRF.x - _RS.CSV.dpRF.x) + _REF.CSA.ddpRF.x - RF_dJdQ(0);
        B0(1,0) = wn*wn*(_REF.CSP.pRF.y - _RS.CSP.pRF.y) + 2*z*wn*(_REF.CSV.dpRF.y - _RS.CSV.dpRF.y) + _REF.CSA.ddpRF.y - RF_dJdQ(1);
        B0(2,0) = wn*wn*(_REF.CSP.pRF.z - _RS.CSP.pRF.z) + 2*z*wn*(_REF.CSV.dpRF.z - _RS.CSV.dpRF.z) + _REF.CSA.ddpRF.z - RF_dJdQ(2);

        //different weight depends on contact state
        if(cRF == false && _REF.RFup == true) W0 = 2; // RF swing  up  phase
        else if(cRF == false && _REF.RFup == false) W0 = 20; // RF swing down phase
        else W0 = 0.1;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        /// swing leg position tracking : left leg
        A0 = MatrixNd::Zero(18 + 12 + 6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18 + 12 + 6*n_con,1);

        A0.block(0,0, JacobianLF3D_pos.rows(), JacobianLF3D_pos.cols()) = JacobianLF3D_pos;
        B0(0,0) = wn*wn*(_REF.CSP.pLF.x - _RS.CSP.pLF.x) + 2*z*wn*(_REF.CSV.dpLF.x - _RS.CSV.dpLF.x) + _REF.CSA.ddpLF.x - LF_dJdQ(0);
        B0(1,0) = wn*wn*(_REF.CSP.pLF.y - _RS.CSP.pLF.y) + 2*z*wn*(_REF.CSV.dpLF.y - _RS.CSV.dpLF.y) + _REF.CSA.ddpLF.y - LF_dJdQ(1);
        B0(2,0) = wn*wn*(_REF.CSP.pLF.z - _RS.CSP.pLF.z) + 2*z*wn*(_REF.CSV.dpLF.z - _RS.CSV.dpLF.z) + _REF.CSA.ddpLF.z - LF_dJdQ(2);

        //different weight depends on contact state
        if(cLF == false && _REF.LFup == true) W0 = 2; // LF swing  up  phase
        else if(cLF == false && _REF.LFup == false) W0 = 20; // LF swing down phase
        else W0 = 0.1;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        /// swing leg orientation tracking
        ///right & left leg yaw (Always Tracking)
        wn = 20;
        z = 0.7;
        A0 = MatrixNd::Zero(18 + 12 + 6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18 + 12 + 6*n_con,1);

        A0.block(0,0, 1,JacobianRF3D_ori.cols()) = JacobianRF3D_ori.block(2,0,1,JacobianRF3D_ori.cols()); // yaw jacobian RF
        A0.block(1,0, 1,JacobianLF3D_ori.cols()) = JacobianLF3D_ori.block(2,0,1,JacobianLF3D_ori.cols()); // yaw jacobian LF

        quat deltaQ_RF = quat(mat3(_REF.CSP.qRF)*(mat3(_RS.CSP.qRF).inverse()));
        double angle_RF = 2*acos(deltaQ_RF[0]);
        double sinhalf_RF = sin(angle_RF/2);
        vec3 rotxyz_RF = vec3();
        if(angle_RF>1e-6){
            rotxyz_RF = vec3(deltaQ_RF[1],deltaQ_RF[2],deltaQ_RF[3])/sinhalf_RF * angle_RF;  // Angle Axis representation of relative Rotation
        }

        quat deltaQ_LF = quat(mat3(_REF.CSP.qLF)*(mat3(_RS.CSP.qLF).inverse()));
        double angle_LF = 2*acos(deltaQ_LF[0]);
        double sinhalf_LF = sin(angle_LF/2);
        vec3 rotxyz_LF = vec3();
        if(angle_LF>1e-6){
            rotxyz_LF = vec3(deltaQ_LF[1],deltaQ_LF[2],deltaQ_LF[3])/sinhalf_LF * angle_LF;  // Angle Axis representation of relative Rotation
        }

        B0(0,0) = wn*wn*(rotxyz_RF.z) + 2*z*wn*(_REF.CSV.dqRF.z-_RS.CSV.dqRF.z)  + _REF.CSA.ddqRF.z - RF_dJRdQ(2);
        B0(1,0) = wn*wn*(rotxyz_LF.z) + 2*z*wn*(_REF.CSV.dqLF.z-_RS.CSV.dqLF.z)  + _REF.CSA.ddqLF.z - LF_dJRdQ(2);

        W0 = 10;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        /// right leg roll, pitch
        wn = 15;
        z = 0.707;
        A0 = MatrixNd::Zero(18 + 12 + 6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18 + 12 + 6*n_con,1);

        A0.block(0,0, 2, JacobianRF3D_ori.cols()) = JacobianRF3D_ori.block(0,0,2,JacobianRF3D_ori.cols());

        B0(0,0) = wn*wn*(rotxyz_RF.x) + 2*z*wn*(_REF.CSV.dqRF.x-_RS.CSV.dqRF.x)  + _REF.CSA.ddqRF.x - RF_dJRdQ(0);
        B0(1,0) = wn*wn*(rotxyz_RF.y) + 2*z*wn*(_REF.CSV.dqRF.y-_RS.CSV.dqRF.y)  + _REF.CSA.ddqRF.y - RF_dJRdQ(1);
        //B0(2,0) = wn*wn*(rotxyz_RF.z) + 2*z*wn*(_REF.CSV.dqRF.z-_RS.CSV.dqRF.z)  + _REF.CSA.ddqRF.z - RF_dJRdQ(2);

        if(cRF == false) W0 = 20; // RF swing phase
        else W0 = 0.1;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        ///left leg roll, pitch
        A0 = MatrixNd::Zero(18 + 12 + 6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18 + 12 + 6*n_con,1);

        A0.block(0,0, 2, JacobianLF3D_ori.cols()) = JacobianLF3D_ori.block(0,0,2,JacobianLF3D_ori.cols());

        B0(0,0) = wn*wn*(rotxyz_LF.x) + 2*z*wn*(_REF.CSV.dqLF.x-_RS.CSV.dqLF.x)  + _REF.CSA.ddqLF.x - LF_dJRdQ(0);
        B0(1,0) = wn*wn*(rotxyz_LF.y) + 2*z*wn*(_REF.CSV.dqLF.y-_RS.CSV.dqLF.y)  + _REF.CSA.ddqLF.y - LF_dJRdQ(1);
        //B0(2,0) = wn*wn*(rotxyz_LF.z) + 2*z*wn*(_REF.CSV.dqLF.z-_RS.CSV.dqLF.z)  + _REF.CSA.ddqLF.z - LF_dJRdQ(2);

        if(cLF == false) W0 = 20; // LF swing phase
        else W0 = 0.1;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        ////----------------------------Direct Tracking of joint q -------------------------------------------------qdd
//        z = 1.0;
//        wn = 10;
//        A0 = MatrixNd::Zero(18+12+6*n_con,18 + 12 + 6*n_con);
//        B0 = MatrixNd::Zero(18+12+6*n_con,1);

//        A0.block(6,6, 12, 12) = MatrixNd::Identity(12,12);

//        for(int i=0; i<18 ; i++){
//            B0(i,0) = wn*wn*(_Qref[i]-_Q[i]) + 2*z*wn*(_dQref[i]-_dQ[i]) +_ddQref[i];
//        }

//        W0 = 1;
//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

        //cout<<"3"<<endl;

        ////-------------------------------------- COM x,y tracking----------------------------------------------------- comdd(x,y)
        CalcCOMJacobian3D(_Q); // calc JacobianCOMall3D
        Vector3d COM_dJdQ = CalcComAcceleration3D(_Q,_dQ,ddqZero);

        wn = 10;
        z = 1;
        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        A0.block(0,0, 2, 18) = JacobianCOMall3D.block(0,0, 2,18);

        B0(0,0) = wn*wn*(_REF.CSP.pCOM.x - _RS.CSP.pCOM.x) + 2*z*wn*(_REF.CSV.dpCOM.x - _RS.CSV.dpCOM.x) + _REF.CSA.ddpCOM.x - COM_dJdQ(0);
        B0(1,0) = wn*wn*(_REF.CSP.pCOM.y - _RS.CSP.pCOM.y) + 2*z*wn*(_REF.CSV.dpCOM.y - _RS.CSV.dpCOM.y) + _REF.CSA.ddpCOM.y - COM_dJdQ(1);
        //B0(2,0) = wn*wn*(_REF.CSP.pCOM.z - _RS.CSP.pCOM.z) + 2*z*wn*(_REF.CSV.dpCOM.z - _RS.CSV.dpCOM.z) + _REF.CSA.ddpCOM.z - COM_dJdQ(2);

        // no COM Pattern version ( only tracks ddCOM) trytry...
//        B0(0,0) = _REF.CSA.ddpCOM.x - COM_dJdQ(0);
//        B0(1,0) = _REF.CSA.ddpCOM.y - COM_dJdQ(1);
        W0 = 10;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);


        ////--------------------------------- pelvis x, y control------------------------------------------------------------peldd(x,y)
        SpatialVector Base_dJdQ_6d = CalcPointAcceleration6D(*Robot,_Q,_dQ,ddqZero,n_pel, Vector3d(0.0,0.0,0.0));
        Vector3d Base_dJdQ = Vector3d(Base_dJdQ_6d(3), Base_dJdQ_6d(4), Base_dJdQ_6d(5));
        Vector3d Base_dJRdQ = Vector3d(Base_dJdQ_6d(0), Base_dJdQ_6d(1), Base_dJdQ_6d(2));

        wn = 5;
        z = 1;
        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        // pelx pely Jacobian
        A0(0,0) = 1;
        A0(1,1) = 1;
        // pelxy ddX - dJdQ
        B0(0,0) = wn*wn*(_Qref[0]-_Q[0]) + 2*z*wn*(_dQref[0]-_dQ[0]) +_ddQref[0] - Base_dJdQ(0);
        B0(1,0) = wn*wn*(_Qref[1]-_Q[1]) + 2*z*wn*(_dQref[1]-_dQ[1]) +_ddQref[1] - Base_dJdQ(1);

        W0 = 0.1;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //cout<<"4"<<endl;
        ////-------------------- ddpelz = k(pel_ref-pel) + kd(dpel_ref - dpel)----------------------------------peldd(z)
        wn = 15;
        z = 1;
        A0 = MatrixNd::Zero(18+12+6*n_con, 18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        //pelz Jacobian : identity
        A0(0,2) = 1;
        // pelz ddX - dJdQ
        B0(0,0) = wn*wn*(_Qref[2]-_Q[2]) + 2*z*wn*(_dQref[2]-_dQ[2]) +_ddQref[2] - Base_dJdQ(2);

        W0 = 10;

        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //cout<<"3"<<endl;

        ////------------------------------- Pelvis Orientation Control roll ---------------------------------------------pelwd
        z = 1.0;
        wn = 15;
        A0 = MatrixNd::Zero(18+12+6*n_con,18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        quat qPelnow = quat(_Q[18],_Q[3],_Q[4],_Q[5]);
        quat qPelref = quat(_Qref[18],_Qref[3],_Qref[4],_Qref[5]);
        quat deltaQ = quat(mat3(qPelref)*(mat3(qPelnow).inverse()));

        double angle = 2*acos(deltaQ[0]);
        double sinhalf = sin(angle/2);
        vec3 rotxyz = vec3();
        if(angle>1e-6){
            rotxyz = vec3(deltaQ[1],deltaQ[2],deltaQ[3])/sinhalf * angle;  // Angle Axis representation of relative Rotation
        }

        // Jacobian of pelvis orientation (identity)
        A0(0,3) = 1;
        A0(1,4) = 0;
        A0(2,5) = 0;

        B0(0,0) = wn*wn*(rotxyz.x) + 2*z*wn*(_dQref[3]-_dQ[3])  + _ddQref[3] - Base_dJRdQ(0);//xrot to be zero  -dJR*dQ??
        B0(1,0) = 0;//wn*wn*(rotxyz.y) + 2*z*wn*(_dQref[4]-_dQ[4])  + _ddQref[4] - Base_dJRdQ(1);//yrot to be zero -dJR*dQ??
        B0(2,0) = 0;//wn*wn*(rotxyz.z) + 2*z*wn*(_dQref[5]-_dQ[5])  + _ddQref[5] - Base_dJRdQ(2);//yrot to be zero -dJR*dQ??

        W0 = 5;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        //cout<<"5"<<endl;

        ////------------------------------- Pelvis Orientation Control pitch & yaw ---------------------------------------------pelwd
        z = 1.0;
        wn = 15;
        A0 = MatrixNd::Zero(18+12+6*n_con,18 + 12 + 6*n_con);
        B0 = MatrixNd::Zero(18+12+6*n_con,1);

        A0(0,3) = 0;
        A0(1,4) = 1;
        A0(2,5) = 1;

        B0(0,0) = 0;//wn*wn*(rotxyz.x) + 2*z*wn*(_dQref[3]-_dQ[3])  + _ddQref[3] - Base_dJRdQ(0);//xrot to be zero  -dJR*dQ??
        B0(1,0) = wn*wn*(rotxyz.y) + 2*z*wn*(_dQref[4]-_dQ[4])  + _ddQref[4] - Base_dJRdQ(1);//yrot to be zero -dJR*dQ??
        B0(2,0) = wn*wn*(rotxyz.z) + 2*z*wn*(_dQref[5]-_dQ[5])  + _ddQref[5] - Base_dJRdQ(2);//yrot to be zero -dJR*dQ??

        W0 = 0.8
                ;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        ////------------------------------Minimization of Change in End Effector Force ---------------------------dF
        /// only perform when contact state is same as previous one
//        if(n_con_old == n_con){
//            A0  = MatrixNd::Zero(18+12+6*n_con,18+12+6*n_con);
//            B0  = MatrixNd::Zero(18+12+6*n_con,1);
//            W0 = 0.001;

//            A0.block(0,18+12, 6*n_con,6*n_con) = MatrixNd::Identity(6*n_con,6*n_con);
//            for(int i=0 ; i<6*n_con ; i++){
//                B0(i,0) = X(i+18+12); // previous Contact force
//            }
//            As.push_back(A0);
//            Bs.push_back(B0);
//            Ws.push_back(W0);
//        }
//        n_con_old = n_con;

        //cout<<"6"<<endl;

        ////---------------------------Minimization of change in torques ------------------------------------------dtau
//        A0 = MatrixNd::Zero(18+12+6*n_con,18 + 12 + 6*n_con);
//        B0 = MatrixNd::Zero(18+12+6*n_con,1);

//        //A0 = [M 0 -J']
//        A0.block(0,0, _M.rows(), _M.cols()) = _M;
//        A0.block(0,18+12, ContactJacobian.cols(), ContactJacobian.rows()) = -ContactJacobian.transpose();

//        //B0 = -h + tau_prev
//        for(int i=0; i<12 ; i++){
//            B0(i,0) = -Nonlin(i) + X(18+i); // previous tau
//        }

//        W0 = 3*1e-2;
//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

        ////-------------------------Minimization of change in contact force ----------------------------
//        A0  = MatrixNd::Zero(18+12+6*n_con,18+12+6*n_con);
//        B0  = MatrixNd::Zero(18+12+6*n_con,1);
//        W0 = 0.001;

//        A0.block(0,18, 12,12) = MatrixNd::Identity(12,12);
//        for(int i=0 ; i<12 ; i++){
//            B0(i,0) = X(i+18); // previous Contact force
//        }
//        As.push_back(A0);
//        Bs.push_back(B0);
//        Ws.push_back(W0);

//        cout<<"7"<<endl;

        ////----------------------------- Making the problem convex-------------------------------------
        A0  = MatrixNd::Identity(18+12+6*n_con,18+12+6*n_con);//make it smaller
        B0  = MatrixNd::Zero(18+12+6*n_con,1);
        W0 = 1e-7;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

        cRF_old = cRF; cLF_old = cLF;
        OC.MakeHF2(As,Bs,Ws);

        X = VectorNd::Zero(OC.getNUMCOLS());

        X = OC.calcX2();

        return X;
    }



};

#endif // OW_RBDL_H
