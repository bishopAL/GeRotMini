#ifndef leg_H
#define leg_H
#include "head.h"

class Leg{

private:
    string _name;
    float _L1,_L2,_L3;   //length of three linkages;
    float _theta0,_theta1,_theta2; //theta of three linkages;theta1--Z,theta2--X,theta3--Y; //represent present
    Matrix<flaot,3,3> _jacobian;//present jacobian
    enum __legStatus{swing=0,attach,stance,detach}_legStatus;
public:
    Leg();
    Leg(string name,float L1,float L2,float L3);
    void setJointPos(vector<float> jointPos);
    void updateJacobians();
    Matrix<float,3,1> forwardKinematics();
    Matrix<float,3,1> inverseKinematics(Matrix<float, 4, 3> cmdpos);   // standing state
    void changeStatus(__legStatus legStatus);
    __legStatus getLegStatus();
    ~Leg();
};

#endif