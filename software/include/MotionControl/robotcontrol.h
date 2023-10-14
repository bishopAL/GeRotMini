#ifndef RBOBOTCONTROL_H
#define RBOBOTCONTROL_H
#include "head.h"
#include "gebot.h"

class RobotControl : public Gebot{
public:
    RobotControl(__controlMode mode);
    Matrix<float, 4, 3> target_pos; // LF RF LH RH ; x y z  in CoM cordinate 
    Matrix<float, 4, 3> target_vel;
    Matrix<float, 4, 3> target_acc; // Force in target position
    Matrix<float, 4, 3> target_force;
    Matrix<float, 4, 3> xc_dotdot;
    Matrix<float, 4, 3> xc_dot;
    Matrix<float, 4, 3> xc;
    Matrix<float, 3, 4> force, force_last;              // force feedback   x y z ; LF RF LH RH
    Matrix<float, 3, 4> target_torque;
    Matrix<float, 4, 3> K_swing, K_stance, K_detach, K_attach;                     //LF RF LH RH
    Matrix<float, 4, 3> B_swing, B_stance, B_detach, B_attach;
    Matrix<float, 4, 3> M_swing, M_stance, M_detach, M_attach;
    float impCtlRate;
    enum __controlMode{ADMITTANCE,IMPEDANCE} _controlMode;  
    //vmc
    
    Matrix<float,3,3> tempASM;  // ASM: 0, -c, b, c, 0, -a, -b, a, 0
    vector<Matrix<float,3,3> > ftsPosASM;
    Matrix<float,3,3> tempEye33;
    Matrix<float,3,3> tempZero33;
    Matrix<float,Dynamic,Dynamic> A; //6*3c
    Matrix<float,Dynamic,Dynamic> W; //3c*3c
    Matrix<float,Dynamic,Dynamic> H; //3c*3c
    Matrix<float,3,1> gravity;
    Matrix<float,6,1> b61;
    Matrix<float,6,6> S66;
    Matrix<float,6,1> g61;
    Matrix<float,3,3> Kpcom;
    Matrix<float,3,3> Kdcom;
    Matrix<float,3,3> Kpbase;
    Matrix<float,3,3> Kdbase;
    Matrix<float,3,1> acc_DCOM;
    Matrix<float,3,1> angelAcc_DBase;
    Matrix<float,3,1> omegaDBase;
    Matrix<float,3,1> omegaBase;
    void calVmcCom();
    void updateImuData();
    void updateFtsPresForce(vector<float> torque);
    void updateTargTor(Matrix<float, 3, 4> force);
    void ParaDeliver();
    void control();
    void changePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM);


}
#endif