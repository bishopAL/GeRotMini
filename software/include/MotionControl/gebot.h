#include "leg.h"
using namespace Eigen;
using namespace std;
class CGebot{

private:
    float m_fLength,m_fWidth,m_fHeight,m_fMass,m_fIxx,m_fIyy,m_fIzz; //2*(distance from com to shoulder);
    CLeg m_glLeg[4];


public:
    vector<vector<float>> vvfTimeForSwingPhase; 
    float fTimeForSwing[4];
    bool bInitFlag;
    float fTimeForGaitPeriod;  // The time of the whole period
    float fTimePeriod;  // The time of one period
    float fTimePresent;
    Matrix<float,4,3> mfShoulderPos;
    Matrix<float, 4,1> mfTimeForSwing;   // The swing time for legs
    Matrix<float, 4, 2> mfTimeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
    Matrix<float, 6,1> mfTargetCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 6,1> mfPresentCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 4, 3> mfTargetCoMPosition;  // X, Y , alpha in world cordinate
    Matrix<float, 4, 3> mfTargetCoMPosture;
    Matrix<float, 4> mfTimePresentForSwing;
    Matrix<float, 4, 2> mfShoulderPos;  // X-Y: LF, RF, LH, RH
    Matrix<float, 4, 3> mfStancePhaseStartPos;
    Matrix<float, 4, 3> mfStancePhaseEndPos;
    Matrix<float, 4, 3> mfInitFootPos;    // initial position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegCmdPos;  // command position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegPresPos;  // present position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegLastPos;
    Matrix<float, 4, 3> mfLegPresVel;  // present velocity of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegLastVel;   //last velocity of foot to shoulder,for filter
    Matrix<float, 4, 3> mfJointCmdPos;  // command joint angle 0-11
    Matrix<float, 4, 3>  mfJointPresPos;  // present motor 0-11
    Matrix<float, 4, 3>  mfJointPresVel;  // present motor 0-11
    Matrix<float, 4, 3>  mfJointCmdVel; 
   
    Gebot();
    Gebot(float length,float width,float height,float mass);
    Gebot(float length,float width,float height,float mass,float Ixx,float Iyy,float Izz);
    CLeg GetLeg(int legNum);
    void SetInitPos(Matrix<float, 4, 3> initPosition);
    void SetCoMVel(Matrix<float, 6,1> tCV);   
    void SetPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP);
    void NextStep();
    void UpdatejointPresPos();
    void UpdatejointPresVel();
    void UpdateJacobians();
    void ForwardKinematics();
    void InverseKinematics();   // standing state
    void UpdateFtsPresVel(); 
    //robot control
    //pump control
    uint8_t svStatus=0b00000000;
    API api;
    void AirControl();
    void PumpNegtive(int legNum);
    void PumpPositive(int legNum);
    void PumpAllNegtive();
    void PumpAllClose();
    //motor control
    vector<int> ID = {  
    0,1,2,
    3, 4, 5,
    6,7,8
    ,9,10,11
    };
    DxlAPI dxlMotors("/dev/ttyUSB0", 1000000, ID, 1);  //3000000  cannot hold 6 legs ttyUSB0 ttyAMA0
    void SetPos(Matrix<float,4,3> jointCmdPos);
    void SetTor(vector<float> setTor(12));




};