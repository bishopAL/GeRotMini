#include "gebot.h"

Gebot::Gebot(float length,float width,float height,float mass)
{
    leg[0]=new leg("LF",45,45,45);
    leg[1]=new leg("RF",45,45,45);
    leg[2]=new leg("LH",45,45,45);
    leg[3]=new leg("RH",45,45,45);
    _length=length;
    _width=width;
    _height=height;
    _mass=mass;
    shoulderPos<<width/2, length/2, 0,width/2, -length/2,0, -width/2, length/2,0,-width/2, -length/2,0;  // X-Y: LF, RF, LH, RH
    
    timePresent=0.0;
    timePresentForSwing.setZero();
    targetCoMVelocity.setZero();
}

Gebot::Gebot(float length,float width,float height,float mass,float Ixx,float Iyy,float Izz)
{
    leg[0]=new leg("LF",45,45,45);
    leg[1]=new leg("RF",45,45,45);
    leg[2]=new leg("LH",45,45,45);
    leg[3]=new leg("RH",45,45,45);
    _length=length;
    _width=width;
    _height=height;
    _mass=mass;
    _Ixx=Ixx;
    _Iyy=Iyy;
    _Izz=Izz;
    shoulderPos<<width/2, length/2, 0,width/2, -length/2,0, -width/2, length/2,0,-width/2, -length/2,0;  // X-Y: LF, RF, LH, RH
    
    timePresent=0.0;
    timePresentForSwing.setZero();
    targetCoMVelocity.setZero();
}

/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
void Gebot::setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
    timePeriod = tP;
    timeForGaitPeriod = tFGP;
    timeForStancePhase = tFSP;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
            timeForSwing(legNum) = _timeForSwingPhase[legNum][1]-_timeForSwingPhase[legNum][0];
    }
}



/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */

void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresPos = initPosition;
    legCmdPos = initPosition;
    initFootPos = initPosition;
    targetCoMPosition.setZero();
}

/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void MotionControl::setCoMVel(Vector<float, 6> tCV)
{
    targetCoMVelocity = tCV;
}


/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void MotionControl::updatejointPresPos()
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPresPos(i,j) = motors.present_positon[i*3 + j];
    }
}

/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPresVel(4,3)
 */
void MotionControl::updatejointPresVel()
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPresVel(i,j) = motors.present_velocity[i*3+j];
    }
}

/**
 * @brief 
 * update Vel of feet in shoulder coordinate
 */
void MotionControl::updateFtsPresVel()
{
    legLastVel=legPresVel;
    Matrix <float, 3, 1> temp_vel;
    for(int i=0; i<4; i++)
    {
        temp_vel = leg[i]._jacobian * jointPresVel.row(i).transpose();
        legPresVel.row(i) = temp_vel.transpose();
    }
    
}


void MotionControl::nextStep()
{

    if (abs(timePresent - timeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
        // legCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(timeForSwingPhase(legNum,0) < timeForSwingPhase(legNum,1))
        {
            if(timePresent > timeForSwingPhase(legNum,0) - timePeriod/2 && timePresent < timeForSwingPhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                leg[legNum].changStatus(swing);
            else    //stance phase 
                leg[legNum].changStatus(stance);           
        }
        if(leg[legNum].getLegStatus() == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
            {
                targetCoMPosition(legNum, pos) += targetCoMVelocity(pos) * timePeriod;
                targetCoMPosture(legNum,pos) +=targetCoMVelocity(pos+3)*timePeriod;
            }
            if(abs(timePresent - timeForSwingPhase(legNum,1)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                for(uint8_t pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosture(legNum,2)), -sin(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,0),
                sin(targetCoMPosture(legNum,2)), cos(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,1),
                0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<shoulderPos(legNum,0), shoulderPos(legNum,1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(timePresent - timeForSwingPhase(legNum,1)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                // shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                // shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForSwingPhase(legNum,0)) < timePeriod + 1e-4)  // if on the end pos   
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
        }
        else    //swing phase 
        {
            Matrix<float, 1, 3> swingPhaseVelocity = -(stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / timeForSwing(legNum) ;
            float x, xh, m, n, k;
            //cout<<"legNum_"<<(int)legNum<<":"<<swingPhaseVelocity.array()<<"  ";
            if( ( timePresentForSwing(legNum) - timeForSwing(legNum) * TimeHeight ) < 1e-4 && timePresentForSwing(legNum) > -1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    legCmdPos(legNum, pos) += swingPhaseVelocity(pos) * timePeriod * swingVelFactor;     // for vertical down
                x = legCmdPos(legNum, 0) - stancePhaseEndPos(legNum, 0);
                xh = -(stancePhaseEndPos(legNum, 0) - stancePhaseStartPos(legNum, 0)) * TimeHeight * swingVelFactor;

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = StepHeight / xh / xh;
                legCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + stancePhaseEndPos(legNum, 2);
            }
            else if( timePresentForSwing(legNum) - timeForSwing(legNum) < 1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {
                for(uint8_t pos=0; pos<2; pos++)
                    legCmdPos(legNum, pos) += swingPhaseVelocity(pos) * timePeriod * (1 - swingVelFactor * TimeHeight) / (1 - TimeHeight); // targetCoMVelocity
                legCmdPos(legNum, 2) -= StepHeight / timeForSwing(legNum) / (1 - TimeHeight) * timePeriod;
            }  

            if(swingPhaseVelocity( 0, 0) == 0)      //first step
            {
                if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) < 1e-4 && timePresentForSwing(legNum) > -1e-4)
                    legCmdPos(legNum, 2) += StepHeight / timeForSwing(legNum) * 2 * timePeriod;
                if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) > -1e-4)
                    legCmdPos(legNum, 2) -=  StepHeight / timeForSwing(legNum) * 2 * timePeriod;
            }
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
    }

    air_control();
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(stepFlag[legNum] != stance) timePresentForSwing(legNum) += timePeriod;
        else timePresentForSwing(legNum) = 0;   //stance phase
    }
    timePresent += timePeriod;
}


void Gebot::forwardKinematics(int mode)
{
    if(mode=0)
    for(int legNum=0;legNum<4;legNum++)
        legCmdPos.row(i)=leg[i].forwardKinematics().transpose();
    else{
        legLastPos=legPrePos;
        legPrePos.row(i)=leg[i].forwardKinematics().transpose();  
    }
}

void Gebot::inverseKinematics()
{
    for(int legNum=0;legNum<4;legNum++)
        jointCmdPos.row(i)=leg[i].inverseKinematics().transpose();
}

//motor control;
void Gebot::setPos(Matrix<float,4,3> jointCmdPos)
{
    vector<float> setPos(12);
    for(int i=0; i<4; i++)  
            for(int j=0;j<3;j++)
            {
                if( isnanf(imp.jointCmdPos(i,j)) )            
                {
                    imp.jointCmdPos(i,j) = SetPos[i*3+j];   // last
                    cout<<"-------------motor_angle_"<<i*3+j<<" NAN-----------"<<endl;
                    // cout<<"target_pos: \n"<<imp.target_pos<<"; \nxc: \n"<<imp.xc<<endl;
                    exit(0);
                }
                else
                {
                    if(imp.jointCmdPos(i,j) - SetPos[i*3+j] > MORTOR_ANGLE_AMP)
                    {
                        SetPos[i*3+j] += MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" +MAX-----------"<<endl;
                    }
                    else if(imp.jointCmdPos(i,j) - SetPos[i*3+j] < -MORTOR_ANGLE_AMP)
                    {
                        SetPos[i*3+j] -= MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" -MAX-----------"<<endl;
                    }
                    else 
                        SetPos[i*3+j] = imp.jointCmdPos(i,j);   // now
                }
                //cout<<"motor_angle_"<<i*3+j<<": "<<SetPos[i*3+j]<<"  ";
            }   
    motors.setPosition(SetPos); 
 
}




//robot's air control & imu update
void MotionControl::pumpAllNegtive(uint8_t legNum)
{
    svStatus=0b00000000;
    api.setSV(svStatus);
}
void MotionControl::pumpAllPositve(uint8_t legNum)
{
    svStatus=0b11111111;
    api.setSV(svStatus);
}
void MotionControl::pumpPositive(uint8_t legNum)
{
    svStatus=svStatus|(0b00000011<<((3-legNum)<<1));
    api.setSV(svStatus);
}
void MotionControl::pumpNegtive(uint8_t legNum)
{   
    svStatus=svStatus&!(0b00000011<<((3-legNum)<<1))
    api.setSV(svStatus);
}
/**
 * @brief control SV to ensure that robot has a suitable positive and negative pressure state to adhere to the wall
 * 
 */
void MotionControl::air_control()
{
    for(int legNum=0;legNum<4;legNum++)
    {
        if(leg[legNum].getLegStatus()==attach)
        {
            pumpNegative(legNum);
        }
        if(leg[legNum].getLegStatus()==detach)
        {
            pumpPositive(legNum);
        }

    }

}