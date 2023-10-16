#include "gebot.h"


#define ForceLPF  0.9
#define StepHeight  30.0
#define TimeHeight (2.0/4.0)  // time for trajectory within vertical part
#define swingVelFactor 3      

CGebot::CGebot(float length,float width,float height,float mass)
{
    m_glleg[0]=new CLeg("LF",45,45,45);
    m_glleg[1]=new CLeg("RF",45,45,45);
    m_glleg[2]=new CLeg("LH",45,45,45);
    m_glleg[3]=new CLeg("RH",45,45,45);
    m_fLength=length;
    m_fWidth=width;
    m_fHeight=height;
    m_fMass=mass;
    fShoulderPos<<width/2, length/2, 0,width/2, -length/2,0, -width/2, length/2,0,-width/2, -length/2,0;  // X-Y: LF, RF, LH, RH
    
    mfTimePresent=0.0;
    mfTimePresentForSwing.setZero();
    mfTargetCoMVelocity.setZero();
    dxlMotors.setOperatingMode(3);  //3 position control; 0 current control
    dxlMotors.torqueEnable();
    dxlMotors.getPosition();
    api.setPump(1, LOW);//LF
    api.setPump(24, LOW);//RF
    api.setPump(28, LOW);//LH
    api.setPump(29, LOW);//RH
    usleep(1e6);
}

CGebot::CGebot(float length,float width,float height,float mass,float Ixx,float Iyy,float Izz)
{
    m_glleg[0]=new Cleg("LF",45,45,45);
    m_glleg[1]=new Cleg("RF",45,45,45);
    m_glleg[2]=new Cleg("LH",45,45,45);
    m_glleg[3]=new Cleg("RH",45,45,45);
    m_fLength=length;
    m_fWidth=width;
    m_fHeight=height;
    m_fMass=mass;
    m_fIxx=Ixx;
    m_fIyy=Iyy;
    m_fIzz=Izz;
    shoulderPos<<width/2, length/2, 0,width/2, -length/2,0, -width/2, length/2,0,-width/2, -length/2,0;  // X-Y: LF, RF, LH, RH
    
    mfTimePresent=0.0;
    mfTimePresentForSwing.setZero();
    mfTargetCoMVelocity.setZero();

    dxlMotors.setOperatingMode(3);  //3 position control; 0 current control
    dxlMotors.torqueEnable();
    dxlMotors.getPosition();
    api.setPump(1, LOW);//LF
    api.setPump(24, LOW);//RF
    api.setPump(28, LOW);//LH
    api.setPump(29, LOW);//RH
    usleep(1e6);
}

CLeg CGebot::GetLeg(int legNum)
{
    return m_glLeg[legNum];
}

/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
void CGebot::setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
    mfTimePeriod = tP;
    mfTimeForGaitPeriod = tFGP;
    mfTimeForStancePhase = tFSP;
    mfTimePresent = 0.0;
    mfTimePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
            mfTimeForSwing(legNum) = vvfTimeForSwingPhase[legNum][1]-vvfTimeForSwingPhase[legNum][0];
    }
}



/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */

void CGebot::setInitPos(Matrix<float, 4, 3> initPosition)
{
    mfStancePhaseStartPos = initPosition;
    mfStancePhaseEndPos = initPosition;
    mfLegPresPos = initPosition;
    mfLegCmdPos = initPosition;
    mfInitFootPos = initPosition;
    mfTargetCoMPosition.setZero();
}

/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void CGebot::SetCoMVel(Vector<float, 6> tCV)
{
    mfTargetCoMVelocity = tCV;
}


/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void CGebot::UpdatejointPresPos()
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            mfJointPresPos(i,j) = dxlMotors.present_positon[i*3 + j];
    }
    for(int legNum=0;legNum<4;legNum++)
    {   
        vector<float> temp(3)=dxlMotors.present_positon(legNum*3,legNum*3+3);
        GetLeg[legNum].SetJointPos(temp);
    }
  
}

/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPresVel(4,3)
 */
void CGebot::UpdatejointPresVel()
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            mfJointPresVel(i,j) = dxlMotors.present_velocity[i*3+j];
    }
}

void CGebot::UpdateJacobians()
{
    for(int legNum=0;legNum<4;legNum++)
        GetLeg[legNum].UpdateJacobian();
}

/**
 * @brief 
 * update Vel of feet in shoulder coordinate
 */
void CGebot::UpdateFtsPresVel()
{
    mfLegLastVel=mfLegPresVel;
    Matrix <float, 3, 1> temp_vel;
    for(int i=0; i<4; i++)
    {
        temp_vel = GetLeg[i].GetJacobian() * mfJointPresVel.row(i).transpose();
        mfLegPresVel.row(i) = temp_vel.transpose();
    }
    
}


void CGebot::NextStep()
{

    if (abs(mfTimePresent - mfTimeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        mfTimePresent = 0.0;
        // mfLegCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(vvfTimeForSwingPhase[legNum][0] < vvfTimeForSwingPhase[legNum][1])
        {
            if(mfTimePresent > vvfTimeForSwingPhase[legNum][0] - fTimePeriod/2 && mfTimePresent < vvfTimeForSwingPhase[legNum][1] - fTimePeriod/2 )
                // check mfTimePresent is in stance phase or swing phase, -fTimePeriod/2 is make sure the equation is suitable
                GetLeg(legNum).ChangeStatus(swing);
            else    //stance phase 
                GetLeg(legNum).ChangeStatus(stance);           
        }
        if(GetLeg(legNum).GetLegStatus() == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
            {
                mfTargetCoMPosition(legNum, pos) += mfTargetCoMVelocity(pos) * fTimePeriod;
                mfTargetCoMPosture(legNum,pos) +=mfTargetCoMVelocity(pos+3)*fTimePeriod;
            }
            if(abs(mfTimePresent - vvfTimeForSwingPhase([legNum][1]) < 1e-4)  // if on the start pos 
            {
                mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
                for(uint8_t pos=0; pos<3; pos++)
                    mfTargetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(mfTargetCoMPosture(legNum,2)), -sin(mfTargetCoMPosture(legNum,2)), mfTargetCoMPosition(legNum,0),
                sin(mfTargetCoMPosture(legNum,2)), cos(mfTargetCoMPosture(legNum,2)), mfTargetCoMPosition(legNum,1),
                0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<shoulderPos(legNum,0), shoulderPos(legNum,1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(mfTimePresent - vvfTimeForSwingPhase[legNum][1]) < 1e-4)  // if on the start pos 
            {
                mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
                // shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                // shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(mfTimePresent - vvfTimeForSwingPhase[legNum][0]) < fTimePeriod + 1e-4)  // if on the end pos   
                mfStancePhaseEndPos(legNum) = mfLegCmdPos(legNum);

            mfLegCmdPos(legNum, 0) = mfStancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            mfLegCmdPos(legNum, 1) = mfStancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
        }
        else    //swing phase 
        {
            Matrix<float, 1, 3> swingPhaseVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / fTimeForSwing(legNum) ;
            float x, xh, m, n, k;
            //cout<<"legNum_"<<(int)legNum<<":"<<swingPhaseVelocity.array()<<"  ";
            if( ( mfTimePresentForSwing(legNum) - fTimeForSwing(legNum) * TimeHeight ) < 1e-4 && mfTimePresentForSwing(legNum) > -1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += swingPhaseVelocity(pos) * fTimePeriod * swingVelFactor;     // for vertical down
                x = mfLegCmdPos(legNum, 0) - mfStancePhaseEndPos(legNum, 0);
                xh = -(mfStancePhaseEndPos(legNum, 0) - mfStancePhaseStartPos(legNum, 0)) * TimeHeight * swingVelFactor;

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // mfLegCmdPos(legNum, 2) = -(x - m) * (x - m) + n + mfStancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = StepHeight / xh / xh;
                mfLegCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + mfStancePhaseEndPos(legNum, 2);
            }
            else if( mfTimePresentForSwing(legNum) - fTimeForSwing(legNum) < 1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += swingPhaseVelocity(pos) * fTimePeriod * (1 - swingVelFactor * TimeHeight) / (1 - TimeHeight); // mfTargetCoMVelocity
                mfLegCmdPos(legNum, 2) -= StepHeight / fTimeForSwing(legNum) / (1 - TimeHeight) * fTimePeriod;
            }  

            if(swingPhaseVelocity( 0, 0) == 0)      //first step
            {
                if( ( mfTimePresentForSwing(legNum) - fTimeForSwing(legNum)/2 ) < 1e-4 && mfTimePresentForSwing(legNum) > -1e-4)
                    mfLegCmdPos(legNum, 2) += StepHeight / fTimeForSwing(legNum) * 2 * fTimePeriod;
                if( ( mfTimePresentForSwing(legNum) - fTimeForSwing(legNum)/2 ) > -1e-4)
                    mfLegCmdPos(legNum, 2) -=  StepHeight / fTimeForSwing(legNum) * 2 * fTimePeriod;
            }
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
    }

    AirControl();
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(GetLeg(legNum).GetLegStatus() != stance) mfTimePresentForSwing(legNum) += fTimePeriod;
        else mfTimePresentForSwing(legNum) = 0;   //stance phase
    }
    mfTimePresent += fTimePeriod;
}


void CGebot::ForwardKinematics(int mode)
{
    if(mode=0)
    for(int legNum=0;legNum<4;legNum++)
        mfLegCmdPos.row(i)=GetLeg[legNum].ForwardKinematics().transpose();
    else{
        mfLegLastPos=mfLegPrePos;
        for(int legNum=0;legNum<4;legNum++)
        mfLegPrePos.row(legNum)=GetLeg[legNum].ForwardKinematics().transpose();  
    }
}

void CGebot::InverseKinematics()
{
    for(int legNum=0;legNum<4;legNum++)
        mfJointCmdPos.row(i)=GetLeg[legNum].InverseKinematics().transpose();
}

//motor control;
void CGebot::SetPos(Matrix<float,4,3> jointCmdPos)
{
    vector<float> SetPos(12);
    for(int i=0; i<4; i++)  
            for(int j=0;j<3;j++)
            {
                if( isnanf(jointCmdPos(i,j)) )            
                {
                    jointCmdPos(i,j) = SetPos[i*3+j];   // last
                    cout<<"-------------motor_angle_"<<i*3+j<<" NAN-----------"<<endl;
                    // cout<<"target_pos: \n"<<imp.target_pos<<"; \nxc: \n"<<imp.xc<<endl;
                    exit(0);
                }
                else
                {
                    if(jointCmdPos(i,j) - SetPos[i*3+j] > MORTOR_ANGLE_AMP)
                    {
                        SetPos[i*3+j] += MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" +MAX-----------"<<endl;
                    }
                    else if(jointCmdPos(i,j) - SetPos[i*3+j] < -MORTOR_ANGLE_AMP)
                    {
                        SetPos[i*3+j] -= MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" -MAX-----------"<<endl;
                    }
                    else 
                        SetPos[i*3+j] = jointCmdPos(i,j);   // now
                }
                //cout<<"motor_angle_"<<i*3+j<<": "<<SetPos[i*3+j]<<"  ";
            }   
    motors.setPosition(SetPos); 
 
}




//robot's air control & imu update
void CGebot::PumpAllNegtive(uint8_t legNum)
{
    svStatus=0b01010101;// 01-N 10-P;
    api.setSV(svStatus);
}
void CGebot::PumpAllPositve(uint8_t legNum)
{
    svStatus=0b10101010;
    api.setSV(svStatus);
}
void CGebot::PumpPositive(uint8_t legNum)
{
    svStatus=svStatus&(0b11111111<<((4-legNum)<<1))+svStatus&(0b11111111>>((legNum+1)<<1))+0b00000010<<((3-legNum)<<1);
    api.setSV(svStatus);
}
void CGebot::PumpNegtive(uint8_t legNum)
{   
    svStatus=svStatus&(0b11111111<<((4-legNum)<<1))+svStatus&(0b11111111>>((legNum+1)<<1))+0b00000001<<((3-legNum)<<1);
    api.setSV(svStatus);
}
/**
 * @brief control SV to ensure that robot has a suitable positive and negative pressure state to adhere to the wall
 * 
 */
void CGebot::AirControl()
{
    for(int legNum=0;legNum<4;legNum++)
    {
        if(GetLeg[legNum].GetLegStatus()==attach)
        {
            PumpNegative(legNum);
        }
        if(GetLeg[legNum].GetLegStatus()==detach)
        {
            PumpPositive(legNum);
        }
        cout<<"svStatus:"<<std::setw(2)<<svStatus<<endl;
    }

}

