#include "leg.h"
#include "head.h"
using namespace Eigen;
using namespace std;
class Gebot{

private:
    float _length,_width,_height,_mass,_Ixx,_Iyy,_Izz; //2*(distance from com to shoulder);
    Leg leg[4];


public:
    vector<vector<float>> timeForSwingPhase; 
    float timeForSwing[4];
    bool initFlag;
    float timeForGaitPeriod;  // The time of the whole period
    float timePeriod;  // The time of one period
    float timePresent;
    Matrix<float,4,3> shoulderPos;
    Matrix<float, 4,1> timeForSwing;   // The swing time for legs
    Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
    Matrix<float, 6,1> targetCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 6,1> presentCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
    Matrix<float, 4, 3> targetCoMPosture;
    Matrix<float, 4> timePresentForSwing;
    Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
    Matrix<float, 4, 3> stancePhaseStartPos;
    Matrix<float, 4, 3> stancePhaseEndPos;
    Matrix<float, 4, 3> initFootPos;    // initial position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> legCmdPos;  // command position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> legPresPos;  // present position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> legLastPos;
    Matrix<float, 4, 3> legPresVel;  // present velocity of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> legLastVel;   //last velocity of foot to shoulder,for filter
    Matrix<float, 4, 3> jointCmdPos;  // command joint angle 0-11
    Matrix<float, 4, 3>  jointPresPos;  // present motor 0-11
    Matrix<float, 4, 3>  jointPresVel;  // present motor 0-11
    Matrix<float, 4, 3>  jointCmdVel; 
   
    Gebot();
    Gebot(float length,float width,float height,float mass);
    Gebot(float length,float width,float height,float mass,float Ixx,float Iyy,float Izz);
    void setInitPos(Matrix<float, 4, 3> initPosition);
    void setCoMVel(Matrix<float, 6,1> tCV);   
    void setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP);
    void nextStep();
    void updatejointPresPos();
    void updatejointPresVel();
    void updateJacobians();
    void forwardKinematics();
    void inverseKinematics();   // standing state
    void updateFtsPresVel(); 
    //robot control
    //pump control
    uint8_t svStatus=0b00000000;
    API api;
    void air_control();
    void pumpNegtive(int legNum);
    void pumpPositive(int legNum);
    void pumpAllNegtive();
    void pumpAllClose();
    //motor control
    vector<int> ID = {  
    0,1,2,
    3, 4, 5,
    6,7,8
    ,9,10,11
    };
    DxlAPI motors("/dev/ttyUSB0", 1000000, ID, 1);  //3000000  cannot hold 6 legs ttyUSB0 ttyAMA0
    void setPos(Matrix<float,4,3> jointCmdPos);
    void setTor(vector<float> setTor(12));




};