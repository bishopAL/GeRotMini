#ifndef leg_H
#define leg_H
#include "controlhelper.h"

class CLeg{

private:
    string m_sName;
    float m_fL1,m_fL2,m_fL3;   //length of three linkages;
    float m_ftheta0,m_ftheta1,m_ftheta2; //theta of three linkages;theta1--Z,theta2--X,theta3--Y; //represent present
    Matrix<float,3,3> m_mfJacobian;//present jacobian
    enum enum_LEGSTATUS{swing=0,attach,stance,detach} m_eLegStatus;
public:
    Leg();
    Leg(string name,float L1,float L2,float L3);
    Matrix<float,3,3> GetJacobian();
    void SetJointPos(vector<float> jointPos);
    void UpdateJacobian();
    Matrix<float,3,1> ForwardKinematics();
    Matrix<float,3,1> InverseKinematics(Matrix<float, 4, 3> cmdpos);   // standing state
    void ChangeStatus(__legStatus legStatus);
    enum_LEGSTATUS GetLegStatus();
    ~Leg();
};

#endif