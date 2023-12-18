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

    fSwingPhaseStatusPart[0]=0.3; //detach
    fSwingPhaseStatusPart[1]=0.2; //swingUp
    fSwingPhaseStatusPart[2]=0.2; //swingDown
    fSwingPhaseStatusPart[3]=0.3; //attach
    fStancePhaseStatusPart[0]=0.1;//recover
    fStancePhaseStatusPart[1]=0.9;//stance

    BSwingPhaseStartFlag = true;
    BSwingPhaseEndFlag = 0;     //
    mfCompensation.setZero();

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
    float fSwPSFactor[4], fStPSFactor[2];
    fTimePeriod = tP;
    fTimeForGaitPeriod = tFGP;
    mfTimeForSwingPhase = tFSP;
    fTimePresent = 0.0;
    // mfTimeForStancePhase<< TimeForGaitPeriod/4.0 *3,          TimeForGaitPeriod/4.0 *2,   // tripod
    //                     TimeForGaitPeriod/4.0,             TimeForGaitPeriod,
    //                     TimeForGaitPeriod - TimePeriod,    TimeForGaitPeriod/4.0 *3,
    //                     TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0;
    mfTimeForStancePhase<<mfTimeForSwingPhase(0,1), mfTimeForSwingPhase(0,0),
                        mfTimeForSwingPhase(1,1), mfTimeForSwingPhase(1,0),
                        mfTimeForSwingPhase(2,1), mfTimeForSwingPhase(2,0),
                        mfTimeForSwingPhase(3,1), mfTimeForSwingPhase(3,0);
    mfTimePresentForSwing.setZero();

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        fTimeForSwing[legNum] = mfTimeForSwingPhase(legNum,1)-mfTimeForSwingPhase(legNum,0);
        iStatusCounterBuffer[legNum][int(detach)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[0]);
        iStatusCounterBuffer[legNum][int(swingUp)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[1]);
        iStatusCounterBuffer[legNum][int(swingDown)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[2]);
        iStatusCounterBuffer[legNum][int(attach)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[3]);
        iStatusCounterBuffer[legNum][int(recover)] = floor( (fTimeForGaitPeriod - fTimeForSwing[legNum]) / fTimePeriod * fStancePhaseStatusPart[0]);
        iStatusCounterBuffer[legNum][int(stance)] = floor( (fTimeForGaitPeriod - fTimeForSwing[legNum]) / fTimePeriod * fStancePhaseStatusPart[1]);
    }
    
    fSwPSFactor[0]=fSwingPhaseStatusPart[0];
    fSwPSFactor[1]=fSwPSFactor[0]+fSwingPhaseStatusPart[1];
    fSwPSFactor[2]=fSwPSFactor[1]+fSwingPhaseStatusPart[2];
    fSwPSFactor[3]=fSwPSFactor[2]+fSwingPhaseStatusPart[3];
    fStPSFactor[0]=fStancePhaseStatusPart[0];
    fStPSFactor[1]=fStPSFactor[0]+fStancePhaseStatusPart[1];
    for(uint8_t legNum=0; legNum<4; legNum++) 
    {
        if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  0 &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[0] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[0] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(detach);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[0] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[1] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[1] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(swingUp);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[1] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[2] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[2] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(swingDown);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[2] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[3] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[3] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(attach);
        }
        else //stance phase
		{
            if( fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0) < (fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[0] )
            {
                iStatusCounter[legNum] = floor(((fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[0] - (fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0)))/ fTimePeriod);
                m_glLeg[legNum]->ChangeStatus(recover);
            }
            else 
            {
                iStatusCounter[legNum] = floor(((fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[1] - (fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0))) / fTimePeriod);
                m_glLeg[legNum]->ChangeStatus(stance);
            }
        }
    }  
}

/**
 * @brief Update the status of leg, record the start Pos and the end Pos of stance phase.
 * 
 * @param legNum the leg number
 */
void CGebot::UpdateLegStatus(int legNum)
{
    iStatusCounter[legNum]--;
    // BSwingPhaseStartFlag = 0;
    // BSwingPhaseEndFlag = 0;
    if(iStatusCounter[legNum] <= 0)
        switch(m_glLeg[legNum]->GetLegStatus())
        {
        case detach:
            m_glLeg[legNum]->ChangeStatus(swingUp);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(swingUp)];
            break;
        case swingUp:
            m_glLeg[legNum]->ChangeStatus(swingDown);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(swingDown)];
            break;
        case swingDown:
            m_glLeg[legNum]->ChangeStatus(attach);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(attach)];
            break;
        case attach:
            m_glLeg[legNum]->ChangeStatus(recover);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
            mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
            for(int pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            BSwingPhaseEndFlag = true;
            break;
        case recover:
            m_glLeg[legNum]->ChangeStatus(stance);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(stance)];
            break;
        case stance:
            m_glLeg[legNum]->ChangeStatus(detach);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(detach)];
            mfStancePhaseEndPos(legNum) = mfLegCmdPos(legNum);
            mfSwingVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp] + iStatusCounterBuffer[legNum][(int)swingDown]) ;
            BSwingPhaseStartFlag = true;
            break;
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
        UpdateLegStatus(legNum);
        enum_LEGSTATUS ls=m_glLeg[legNum]->GetLegStatus();
        
        // cout<<"leg_"<<(int)legNum<<"_status: "<<(int)ls<<endl;
        if( ls == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
            {
                targetCoMPosition(legNum, pos) += vfTargetCoMVelocity(pos) * fTimePeriod;
                targetCoMPosture(legNum,pos) +=vfTargetCoMVelocity(pos+3)*fTimePeriod;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosture(legNum,2)), -sin(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,0),
                sin(targetCoMPosture(legNum,2)), cos(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,1),
                0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<mfShoulderPos(legNum,0), mfShoulderPos(legNum,1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;
            for (size_t i = 0; i < 3; i++)
                mfLegCmdPos(legNum, i) = mfStancePhaseStartPos(legNum, i) + (mfShoulderPos(legNum, i) - oneShoulderPos_3x1(i));
        }
        else if( ls == detach || ls == swingUp )   //swing phase 
        {
            // cout<<"swing-"<<(unsigned)legNum<<endl;
            float x, xh, m, n, k;
            if(mfSwingVelocity( 0, 0) == 0)      //first step
            {
                mfLegCmdPos(legNum, 2) += StepHeight / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp]);
            }
            else 
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += mfSwingVelocity(pos);     
                x = mfLegCmdPos(legNum, 0) - mfStancePhaseEndPos(legNum, 0);
                xh = -(mfStancePhaseEndPos(legNum, 0) - mfStancePhaseStartPos(legNum, 0)) * (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp])  / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp] + iStatusCounterBuffer[legNum][(int)swingDown]);

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = StepHeight / xh / xh;
                mfLegCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + mfStancePhaseEndPos(legNum, 2);
            }
        }
        else if( ls == swingDown )    //swing phase
        {
            if(mfSwingVelocity( 0, 0) != 0)     
            {
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += mfSwingVelocity(pos) ;
            }   
            mfLegCmdPos(legNum, 2) -= StepHeight / iStatusCounterBuffer[legNum][(int)ls];
        }
        else if( ls == attach )    //swing phase
        {
            mfLegCmdPos(legNum, 2) -=  Press / iStatusCounterBuffer[legNum][(int)ls];
        }
        else if( ls == recover )   //stance phase
        {   // mfLegCmdPos(legNum, 2)  is not related to mfStancePhaseEndPos(legNum, 2) and mfStartPhaseEndPos(legNum, 2)
            mfLegCmdPos(legNum, 2) +=  Press / iStatusCounterBuffer[legNum][(int)ls];
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

/**
 * @brief forwardKinematics
 * 
 * @param mode 
 * if mode=0    calculcate foot position(legCmdPos) with jointCmdPos(target),
 * if mode=1    calculcate foot position(legPresPos) with jointPresPos(present) and update legPos_last
 */
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




//robot's air control 
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
        if(m_glLeg[legNum]->GetLegStatus()==swingDown)  //attach
        {
            PumpNegtive(legNum);
        }
        else if(m_glLeg[legNum]->GetLegStatus()==stance)// Apply negative pressure in advance to solve gas path delay.
        {
            if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * 0.04) )   
                PumpPositive(legNum);
        }
        //cout<<"svStatus:"<<std::setw(2)<<svStatus<<endl;
    }
}
/**
 * @brief Correct body tilt.
 * Notice: Applicable to amble gait
 */
void CGebot::AttitudeCorrection()
{
    int legNum;
    if(BSwingPhaseStartFlag == true )
    {
        BSwingPhaseStartFlag = 0;
        for(legNum=0;legNum<4;legNum++)
        {
            switch(m_glLeg[legNum]->GetLegStatus()) // select swing leg
            {
            case detach:
            case swingUp:
            case swingDown:
            case attach:
                switch (legNum) //the number of swing leg is only 1 in amble gait   
                {
                case 0:
                    mfCompensation<< 0, CompensationDistanceA3, CompensationDistanceA2, CompensationDistanceA1;
                    break;
                case 1:
                    mfCompensation<< CompensationDistanceA3, 0, CompensationDistanceA1, CompensationDistanceA2;
                    break;
                case 2:
                    mfCompensation<< CompensationDistanceA2, CompensationDistanceA1, 0, CompensationDistanceA3;
                    break;
                case 3:
                    mfCompensation<< CompensationDistanceA1, CompensationDistanceA2, CompensationDistanceA3, 0;
                    break;
                }
                break;
            }
        }
    }
    if(BSwingPhaseEndFlag == true )
    {
        BSwingPhaseEndFlag = 0;
        mfCompensation<< CompensationDistanceALL, CompensationDistanceALL, CompensationDistanceALL, CompensationDistanceALL;
    }

}

