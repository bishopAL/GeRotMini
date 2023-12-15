#include "gebot.h"





CGebot::CGebot(float length,float width,float height,float mass)
{
    dxlMotors.init("/dev/ttyAMA0", 3000000, ID, 2);  // CAN NOT 4M.   ttyUSB0 ttyAMA0
    m_glLeg[0] = new CLeg(LF,60.0,60.0,30.0);
    m_glLeg[1] = new CLeg(RF,60.0,60.0,30.0);
    m_glLeg[2] = new CLeg(LH,60.0,60.0,30.0);
    m_glLeg[3] = new CLeg(RH,60.0,60.0,30.0);
    m_fLength=length;
    m_fWidth=width;
    m_fHeight=height;
    m_fMass=mass;
    mfShoulderPos<<width/2, length/2, 0,width/2, -length/2,0, -width/2, length/2,0,-width/2, -length/2,0;  // X-Y: LF, RF, LH, RH
    for(int i=0;i<4;i++)
    {
        fSwingStatus[i][0]=0.125;
        fSwingStatus[i][1]=0.75;
        fSwingStatus[i][2]=0.125;
    }
    for(int i=0;i<12;++i)
        vLastSetPos.push_back(0);
    fTimePresent=0.0;
    mfTimePresentForSwing.setZero();
    vfTargetCoMVelocity.setZero();
    dxlMotors.setOperatingMode(3);  //3 position control; 0 current control
    dxlMotors.torqueEnable();
   // dxlMotors.getPosition();
    api.setPump(1, LOW);//LF
    api.setPump(24, LOW);//RF
    api.setPump(28, LOW);//LH
    api.setPump(29, LOW);//RH
    usleep(1e6);
}



/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
void CGebot::SetPhase(float tP, float tFGP, Matrix<float,4,2> tFSP)
{
    fTimePeriod = tP;
    fTimeForGaitPeriod = tFGP;
    mfTimeForSwingPhase = tFSP;
    fTimePresent = 0.0;
     mfTimeForStancePhase<< TimeForGaitPeriod/4.0 *3,          TimeForGaitPeriod/4.0 *2,   // tripod
                         TimeForGaitPeriod/4.0,             TimeForGaitPeriod,
                         TimeForGaitPeriod - TimePeriod,    TimeForGaitPeriod/4.0 *3,
                         TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0;
    mfTimePresentForSwing.setZero();
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
            fTimeForSwing[legNum] = mfTimeForSwingPhase(legNum,1)-mfTimeForSwingPhase(legNum,0);
            iStatusRunTimes[legNum][1]=fTimeForSwing[legNum]/fTimePeriod;
            iStatusRunTimes[legNum][0]=(fTimeForGaitPeriod-fTimeForSwing[legNum])/fTimePeriod;
    }
    for(uint8_t legNum=0; legNum<4; legNum++) 
    {
        if(mfTimeForSwingPhase(legNum,0)==0)
            m_glLeg[legNum]->ChangeStatus(swing);
        else
            m_glLeg[legNum]->ChangeStatus(stance);
    }
  
    
}



/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */

void CGebot::SetInitPos(Matrix<float, 4, 3> initPosition)
{
    mfStancePhaseStartPos = initPosition;
    mfStancePhaseEndPos = initPosition;
    mfLegPresPos = initPosition;
    mfLegCmdPos = initPosition;
    mfInitFootPos = initPosition;
    targetCoMPosition.setZero();
}

/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void CGebot::SetCoMVel(Matrix<float, 6,1> tCV)
{
    vfTargetCoMVelocity = tCV;
}


/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void CGebot::UpdatejointPresPos()
{
    mfJointPresPos=inverseMotorMapping(dxlMotors.present_position);
    for(int legNum=0;legNum<4;legNum++)
    {   
        Matrix<float,3,1> temp=mfJointCmdPos.block(legNum,0,1,3).transpose();
        m_glLeg[legNum]->SetJointPos(temp);
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
    mfJointPresVel=inverseMotorMapping(dxlMotors.present_velocity);       
}

void CGebot::UpdateJacobians()
{
    for(int legNum=0;legNum<4;legNum++)
        m_glLeg[legNum]->UpdateJacobian();
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
        temp_vel = m_glLeg[i]->GetJacobian() * mfJointPresVel.row(i).transpose();
        mfLegPresVel.row(i) = temp_vel.transpose();
    }
    
}
void CGebot::NextStep()
{
    if (abs(fTimePresent - fTimeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        fTimePresent = 0.0;
        // legCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(mfTimeForStancePhase(legNum,0) < mfTimeForStancePhase(legNum,1))
        {
            if(fTimePresent > mfTimeForStancePhase(legNum,0) - fTimePeriod/2 && fTimePresent < mfTimeForStancePhase(legNum,1) - fTimePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                m_glLeg[legNum]->ChangeStatus(stance);
            else    //swing phase 
                m_glLeg[legNum]->ChangeStatus(swing);           
        }
        else
        {
            if(fTimePresent > mfTimeForStancePhase(legNum,0) - fTimePeriod/2 || fTimePresent < mfTimeForStancePhase(legNum,1) - fTimePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                   m_glLeg[legNum]->ChangeStatus(stance);
            else    //swing phase 
                  m_glLeg[legNum]->ChangeStatus(swing);  
        }
        if(m_glLeg[legNum]->GetLegStatus() == swing)
        {
            if( ( mfTimePresentForSwing(legNum) - fTimeForSwing[legNum] * 0.5 ) < 1e-4 && mfTimePresentForSwing(legNum) > -1e-4)
                m_glLeg[legNum]->ChangeStatus(detach);
            else if( ( mfTimePresentForSwing(legNum) - fTimeForSwing[legNum] * 0.7 ) > -1e-4 )
                m_glLeg[legNum]->ChangeStatus(attach);  
        }

        if( m_glLeg[legNum]->GetLegStatus() == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
            {
                targetCoMPosition(legNum, pos) += vfTargetCoMVelocity(pos) * fTimePeriod;
                targetCoMPosture(legNum,pos) +=vfTargetCoMVelocity(pos+3)*fTimePeriod;
            }
            if(abs(fTimePresent - mfTimeForStancePhase(legNum,0)) < 1e-4)  // if on the start pos 
            {
                mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
                for(int pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosture(legNum,2)), -sin(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,0),
                sin(targetCoMPosture(legNum,2)), cos(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,1),
                0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<mfShoulderPos(legNum,0), mfShoulderPos(legNum,1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;
            // cout<<"1-"<<oneShoulderPos_3x1<<endl;
            if(abs(fTimePresent - mfTimeForStancePhase(legNum,0)) < 1e-4)  // if on the start pos 
            {
                mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
                // shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                // shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(fTimePresent - mfTimeForStancePhase(legNum,1)) < fTimePeriod + 1e-4)  // if on the end pos   
                mfStancePhaseEndPos(legNum) = mfLegCmdPos(legNum);

            mfLegCmdPos(legNum, 0) = mfStancePhaseStartPos(legNum, 0) + (mfShoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            mfLegCmdPos(legNum, 1) = mfStancePhaseEndPos(legNum, 1) + (mfShoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
            // cout<<"stance-"<<(unsigned)legNum<<"="<<mfLegCmdPos(legNum, 0)<<endl;
        }
        else    //swing phase 
        {
            // cout<<"swing-"<<(unsigned)legNum<<endl;
            Matrix<float, 1, 3> swingPhaseVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / fTimeForSwing[legNum] ;
            float x, xh, m, n, k;
            // cout<<"swingPhaseVelocity="<<swingPhaseVelocity.array()<<endl;
            if( ( mfTimePresentForSwing(legNum) - fTimeForSwing[legNum] * TimeHeight ) < 1e-4 && mfTimePresentForSwing(legNum) > -1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += swingPhaseVelocity(pos) * fTimePeriod * swingVelFactor;     // for vertical down
                x = mfLegCmdPos(legNum, 0) - mfStancePhaseEndPos(legNum, 0);
                xh = -(mfStancePhaseEndPos(legNum, 0) - mfStancePhaseStartPos(legNum, 0)) * TimeHeight * swingVelFactor;

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = StepHeight / xh / xh;
                mfLegCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + mfStancePhaseEndPos(legNum, 2);
            }
            else if( mfTimePresentForSwing(legNum) - fTimeForSwing[legNum] < 1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += swingPhaseVelocity(pos) * fTimePeriod * (1 - swingVelFactor * TimeHeight) / (1 - TimeHeight); // targetCoMVelocity
                mfLegCmdPos(legNum, 2) -= StepHeight / fTimeForSwing[legNum] / (1 - TimeHeight) * fTimePeriod;
            }  

            if(swingPhaseVelocity( 0, 0) == 0)      //first step
            {
                static int i=1;
                i++;
                // cout<<"i = "<<i<<endl;
                // cout<<"time="<<fTimePresent<<endl;
                if( ( mfTimePresentForSwing(legNum) - fTimeForSwing[legNum]/2 ) < 1e-4 && mfTimePresentForSwing(legNum) > -1e-4)
                    mfLegCmdPos(legNum, 2) += StepHeight / fTimeForSwing[legNum] * 2 * fTimePeriod;
                if( ( mfTimePresentForSwing(legNum) - fTimeForSwing[legNum]/2 ) > -1e-4)
                    mfLegCmdPos(legNum, 2) -=  StepHeight / fTimeForSwing[legNum] * 2 * fTimePeriod;
            }
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
        }

    AirControl();
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()!= stance) mfTimePresentForSwing(legNum) += fTimePeriod;
        else mfTimePresentForSwing(legNum) = 0;   //stance phase
    }
    fTimePresent += fTimePeriod;
}
/*
void CGebot::NextStep()
{
    for(int i=0;i<3;i++)
    {
        vfTargetCoMPosition(i) += vfTargetCoMVelocity(i) * fTimePeriod;
        vfTargetCoMPosture(i) += vfTargetCoMVelocity(i+3)*fTimePeriod;
    }
    for(int legNum=0;legNum<4;legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()==swing)
        {
            iStatusRunTimes[legNum][1]--;
            if(iStatusRunTimes[legNum][1]<0)
            {
                iStatusRunTimes[legNum][1]=fTimeForSwing[legNum]/fTimePeriod;
                m_glLeg[legNum]->ChangeStatus(stance);
                goto nextFlag;
            }
        Matrix<float, 1, 3> swingPhaseVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / iStatusRunTimes[legNum][1] ;
        float upStepX = fabsf(p->stancePhaseEndPos.element[legNum][0] - p->stancePhaseStartPos.element[legNum][0]) * iStatusRunTimes[legNum][1] / iStatusRunTimes[legNum][1];   
		float upStepY = fabsf(p->stancePhaseEndPos.element[legNum][1] - p->stancePhaseStartPos.element[legNum][1]) * iStatusRunTimes[legNum][1] / iStatusRunTimes[legNum][1];



        }
    }

    nextFlag:

//     if (abs(fTimePresent - fTimeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
//     {                                                            // if so, set it to 0.0
//         fTimePresent = 0.0;
//         // mfLegCmdPos = initFootPos;
//     }

//     for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
//     {   
//         if(mfTimeForSwingPhase(legNum,0) < mfTimeForSwingPhase(legNum,1))
//         {
//             if(fTimePresent > mfTimeForSwingPhase(legNum,0) - fTimePeriod/2 && fTimePresent < mfTimeForSwingPhase(legNum,1) - fTimePeriod/2 )
//                 // check fTimePresent is in stance phase or swing phase, -fTimePeriod/2 is make sure the equation is suitable
//                 m_glLeg[legNum]->ChangeStatus(swing);
//             else    //stance phase 
//                 m_glLeg[legNum]->ChangeStatus(stance);           
//         }
//         if(m_glLeg[legNum]->GetLegStatus() == stance ) //stance phase
//         {     
//             for(uint8_t pos=0; pos<3; pos++)
//             {
//                 mfTargetCoMPosition(legNum, pos) += vfTargetCoMVelocity(pos,0) * fTimePeriod;
//                 mfTargetCoMPosture(legNum,pos) +=vfTargetCoMVelocity(pos+3,0)*fTimePeriod;
//             }
//             if(abs(fTimePresent - mfTimeForSwingPhase(legNum,1)) < 1e-4)  // if on the start pos 
//             {
//                 mfStancePhaseStartPos.row(legNum) = mfLegCmdPos.row(legNum);
//                 cout<<"StartPos-"<<(unsigned)legNum<<":   "<<mfStancePhaseStartPos.row(legNum)<<endl;
//                 for(uint8_t pos=0; pos<3; pos++)
//                     mfTargetCoMPosition(legNum, pos) = 0.0;
//             }
//             Matrix<float, 3, 3> trans;
//             trans<<cos(mfTargetCoMPosture(legNum,2)), -sin(mfTargetCoMPosture(legNum,2)), mfTargetCoMPosition(legNum,0),
//                 sin(mfTargetCoMPosture(legNum,2)), cos(mfTargetCoMPosture(legNum,2)), mfTargetCoMPosition(legNum,1),
//                 0, 0, 1;
//             Matrix<float, 3, 1> oneShoulderPos_3x1;
//             oneShoulderPos_3x1<<mfShoulderPos(legNum,0), mfShoulderPos(legNum,1), 1;
//             oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

//             if(abs(fTimePresent - mfTimeForSwingPhase(legNum,1)) < 1e-4)  // if on the start pos 
//             {
//                 mfStancePhaseStartPos.row(legNum) = mfLegCmdPos.row(legNum);
//                 // shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
//                 // shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
//             }
//             if(fTimePresent<0.1 || fTimePresent>7.9)
//                 cout<<(unsigned)legNum<<"time: "<<fTimePresent<<"\n dshasdjh="<<abs(fTimePresent - mfTimeForSwingPhase(legNum,0))<<endl;
//             if(abs(fTimePresent - mfTimeForSwingPhase(legNum,0)) < fTimePeriod + 1e-4)  // if on the end pos   
//                 {
//                     mfStancePhaseEndPos.row(legNum) = mfLegCmdPos.row(legNum);
//                     cout<<"EndPos-"<<(unsigned)legNum<<":   "<<mfStancePhaseEndPos.row(legNum)<<endl;
//                     // cout<<"legNum"<<bitset<8>(legNum)<<endl;
//                 }

//             mfLegCmdPos(legNum, 0) = mfStancePhaseStartPos(legNum, 0) + (mfShoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
//             mfLegCmdPos(legNum, 1) = mfStancePhaseStartPos(legNum, 1) + (mfShoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
//         }
//         else    //swing phase 
//         {
//             Matrix<float, 1, 3> swingPhaseVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / fTimeForSwing[legNum] ;
//             float x, xh, m, n, k;
//             //cout<<"legNum_"<<(int)legNum<<":"<<swingPhaseVelocity.array()<<"  ";
//             if( ( mfTimePresentForSwing(legNum,0) - fTimeForSwing[legNum] * TimeHeight ) < 1e-4 && mfTimePresentForSwing(legNum,0) > -1e-4 && swingPhaseVelocity( 0, 0) != 0)
//             {            
//                 for(uint8_t pos=0; pos<2; pos++)
//                     mfLegCmdPos(legNum, pos) += swingPhaseVelocity(0,pos) * fTimePeriod * swingVelFactor;     // for vertical down
//                 x = mfLegCmdPos(legNum, 0) - mfStancePhaseEndPos(legNum, 0);
//                 xh = -(mfStancePhaseEndPos(legNum, 0) - mfStancePhaseStartPos(legNum, 0)) * TimeHeight * swingVelFactor;

//                 /* z = -( x - m )^2 + n + z0    */
//                 // m = (StepHeight + xh * xh) /2 /xh;
//                 // n = m * m;
//                 // mfLegCmdPos(legNum, 2) = -(x - m) * (x - m) + n + mfStancePhaseEndPos(legNum, 2);

//                 /* z = -k * ( x - xh )^2 + H + z0 */
//                 k = StepHeight / xh / xh;
//                 mfLegCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + mfStancePhaseEndPos(legNum, 2);
//             }
//             else if( mfTimePresentForSwing(legNum,0) - fTimeForSwing[legNum] < 1e-4 && swingPhaseVelocity( 0, 0) != 0)
//             {
//                 for(uint8_t pos=0; pos<2; pos++)
//                     mfLegCmdPos(legNum, pos) += swingPhaseVelocity(0,pos) * fTimePeriod * (1 - swingVelFactor * TimeHeight) / (1 - TimeHeight); // vfTargetCoMVelocity
//                 mfLegCmdPos(legNum, 2) -= StepHeight / fTimeForSwing[legNum] / (1 - TimeHeight) * fTimePeriod;
//             }  

//             if(swingPhaseVelocity( 0, 0) == 0)      //first step
//             {
//                 if( ( mfTimePresentForSwing(legNum,0) - fTimeForSwing[legNum]/2 ) < 1e-4 && mfTimePresentForSwing(legNum,0) > -1e-4)
//                     mfLegCmdPos(legNum, 2) += StepHeight / fTimeForSwing[legNum] * 2 * fTimePeriod;
//                 if( ( mfTimePresentForSwing(legNum,0) - fTimeForSwing[legNum]/2 ) > -1e-4)
//                     mfLegCmdPos(legNum, 2) -=  StepHeight / fTimeForSwing[legNum] * 2 * fTimePeriod;
//             }
//         }
//         //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
//     }

//     AirControl();
//     for(uint8_t legNum=0; legNum<4; legNum++)
//     {
//         if(m_glLeg[legNum]->GetLegStatus() != stance) mfTimePresentForSwing(legNum,0) += fTimePeriod;
//         else mfTimePresentForSwing(legNum,0) = 0;   //stance phase
//     }
//     fTimePresent += fTimePeriod;
// }
//*/


void CGebot::ForwardKinematics(int mode)
{
    if(mode=0)
    for(int legNum=0;legNum<4;legNum++)
        mfLegCmdPos.row(legNum)=m_glLeg[legNum]->ForwardKinematic().transpose();
    else{
        mfLegLastPos=mfLegPresPos;
        for(int legNum=0;legNum<4;legNum++)
        mfLegPresPos.row(legNum)=m_glLeg[legNum]->ForwardKinematic().transpose();  
    }
}

void CGebot::InverseKinematics(Matrix<float, 4, 3> cmdpos)
{
    for(int legNum=0;legNum<4;legNum++)
        mfJointCmdPos.row(legNum)=m_glLeg[legNum]->InverseKinematic(cmdpos.block(legNum,0,1,3)).transpose();
}

//motor control;
 void CGebot::SetPos(Matrix<float,4,3> jointCmdPos)
{
    vector<float> setPos;
    setPos=motorMapping(jointCmdPos);
    for(int i=0; i<4; i++)  
            for(int j=0;j<3;j++)
            {
                if( isnanf(setPos[i*3+j]))         
                {
                    setPos[i*3+j] = vLastSetPos[i*3+j];   // last
                    cout<<"-------------motor_angle_"<<i*3+j<<" NAN-----------"<<endl;
                    // cout<<"target_pos: \n"<<imp.target_pos<<"; \nxc: \n"<<imp.xc<<endl;
                    exit(0);
                }
                else
                {
                    if(setPos[i*3+j] - vLastSetPos[i*3+j] > MORTOR_ANGLE_AMP)
                    {
                        vLastSetPos[i*3+j] += MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" +MAX-----------"<<endl;
                    }
                    else if(setPos[i*3+j] - vLastSetPos[i*3+j] < -MORTOR_ANGLE_AMP)
                    {
                        vLastSetPos[i*3+j] -= MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" -MAX-----------"<<endl;
                    }
                    else 
                        vLastSetPos[i*3+j] = setPos[i*3+j];   // now
                }
                //cout<<"motor_angle_"<<i*3+j<<": "<<SetPos[i*3+j]<<"  ";
            }   
    dxlMotors.setPosition(vLastSetPos); 
 
}




//robot's air control & imu update
//RF-RH-LH-LF
void CGebot::PumpAllNegtive()
{
    svStatus=0b01010101;// 01-N 10-P;
    api.setSV(svStatus);
}
void CGebot::PumpAllPositve()
{
    svStatus=0b10101010;
    api.setSV(svStatus);
}
void CGebot::PumpPositive(int legNum)
{
    if(legNum==0) legNum=3;
    else if(legNum==1) legNum=0;
    else if(legNum==2) legNum=2;
    else if(legNum==3) legNum=1;
    svStatus|=1<<((3-legNum)*2+1);
    //cout<<"svStatus1="<<bitset<8>(svStatus)<<"\n";
    svStatus&=~(1<<((3-legNum)*2));
    //cout<<"svStatus2="<<bitset<8>(svStatus)<<"\n";
    api.setSV(svStatus);
}
void CGebot::PumpNegtive(int legNum)
{   
    if(legNum==0) legNum=3;
    else if(legNum==1) legNum=0;
    else if(legNum==2) legNum=2;
    else if(legNum==3) legNum=1;
    svStatus|=1<<((3-legNum)*2);
   //  cout<<"svStatus1="<<bitset<8>(svStatus)<<"\n";
    svStatus&=~(1<<((3-legNum)*2+1));
    //cout<<"svStatus2="<<bitset<8>(svStatus)<<"\n";
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
        if(m_glLeg[legNum]->GetLegStatus()==attach)
        {
            PumpNegtive(legNum);
        }
        else if(m_glLeg[legNum]->GetLegStatus()==detach)
        {
           PumpPositive(legNum);
        }

        //cout<<"svStatus:"<<std::setw(2)<<svStatus<<endl;
    }

}

