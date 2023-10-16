#include "robotcontrol.h"
using namespace Eigen;
CRobotControl::CRobotControl(enum_CONTROLMODE mode,float length,float width,float height,float mass):CGebot(length,width,height,mass)
{
    float impdata[200];
    string2float("../include/adm_parameter.csv",impdata);   // 0-swing,1-attach,2-stance,3-detach
    for(int i=0; i<4; i++)
    {
        Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata + 36 * i), mapB(impdata + 12 + 36 * i), mapM(impdata  + 24 + 36 * i);
        ChangePara(mapK,mapB,mapM, i);
    } 
    mfXcDotDot.setZero();
    mfXcDot.setZero();
    mfXc.setZero();
    mfTargetPos.setZero();
    mfTargetVel.setZero();
    mfTargetAcc.setZero();
    mfTargetForce.setZero();
    mfLastForce.setZero();
    fCtlRate = 100;
    // cal inertial
    m_fIxx=((_length*_length)+(_height*_height))*mass/12;
    m_eControlMode=mode;
}

void CRobotControl::UpdateImuData()
{
    api.updateIMU();
    vfVmcOmegaBase<<api.fAcc;
    vfGravity<<api.fGyro;
}

void CRobotControl::UpdateFtsPresForce()
{
    Matrix<float, 3, 4> temp;
    if(mfForce(2,3) - mfLastForce(2,3) > 0.3 || mfForce(2,3) - mfLastForce(2,3) < -0.3)
        temp.setZero();
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = dxlMotors.present_torque[i+j*3];
    for (int i=0; i<4; i++)
        mfForce.col(i) = ForceLPF * mfLastForce.col(i) + (1-ForceLPF) * leg[i]._jacobian.transpose().inverse() * temp.col(i);
    mfLastForce = mfForce;
}

void CRobotControl::UpdateTargTor(Matrix<float, 3, 4> force)
{
    for (int legNum=0; legNum<4; legNum++)
        mfTargetTor.col(legNum) = GetLeg[legNum].GetJacobian() * force.col(i); 
}

void CRobotControl::ParaDeliver()
{
    #ifdef  VMCCONTROL
    CalVmcCom();
    #else
    mfTargetPos = legCmdPos;
    for(uint8_t legNum=0; legNum<4; legNum++)
    {   
        if(GetLeg[legNum].GetLegStatus() == swing) //swing
        {
            if( ( timePresentForSwing(legNum) - (timeForSwing(legNum) - timePeriod *8) ) > -1e-4 )
            {
                GetLeg[legNum].GetLegStatus() = attach;   //attach
                mfTargetForce.row(legNum) << 0, 0, -1.6;  // x, y, z
            }
            else if( ( timePresentForSwing(legNum) - timePeriod *8 ) < 1e-4 && timePresentForSwing(legNum) > 1e-4)
            {
                GetLeg[legNum].GetLegStatus() = detach;   //detach
                mfTargetForce.row(legNum) << 0, 0, 1.5;
            }
            else    //swing
            {
                GetLeg[legNum].GetLegStatus() = swing;
                mfTargetForce.row(legNum) << 0, 0, 0;
            }
        }
        else        //stance
        {
            GetLeg[legNum].GetLegStatus() = stance;
            mfTargetForce.row(legNum) << -0.6, 0, -1.6;    
        }
    }
    #endif
}

void CRobotControl::ChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM)
{
        switch(mode) 
    {
        case 0:
            mfKstance = mK; mfBstance = mB; mfMstance = mM;
            break;
        case 1:
            mfKswing = mK; mfBwing = mB; mfMswing = mM;
            break;
        case 2:
            mfKdetach = mK; mfBdetach = mB; mfMdetach = mM;
            break;
        case 3:
            mfKattach = mK; mfBattach = mB; mfMattach = mM;   
            break; 
    }
}

void CRobotControl::Control()
{
    if(m_eControlMode == IMPEDANCE)  // Impedance control
    {
        mfXcDot = (mfLegPresPos - mfLegLastPos) * fCtlRate;
        mfXcDotDot = (mfLegPresVel - mfXcDot) * fCtlRate;
        for(uint8_t legNum=0; legNum<4; legNum++)
        {
            mfForce.transpose().row(legNum) = mfTargetForce.row(legNum)  
            + mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
            + mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) ;
            // + mfMstance.row(legNum).cwiseProduct(mfTargetAcc.row(legNum) - mfXcDotDot.row(legNum));
            cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<endl;
            cout<<"K__stance_"<<(int)legNum<<"  "<<mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
            cout<<"B__stance_"<<(int)legNum<<"  "<<mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
            // cout<<"mfForce"<<mfForce.transpose().row(legNum)<<endl;
        }
        UpdateTargTor(mfForce);
        // cout<<"mfTargetPos:"<<endl<<mfTargetPos<<endl;
        // cout<<"mfLegPresPos:"<<endl<<mfLegPresPos<<endl;
        cout<<"mfForce:"<<endl<<mfForce<<endl;
        cout<<"mfTargetTor:"<<endl<<mfTargetTor<<endl<<endl;
    }
    else if(m_eControlMode == ADMITTANCE)  // Admittance control    mfXcDotDot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    {
        for(uint8_t legNum=0; legNum<4; legNum++)
        {   
            switch(GetLeg[legNum].GetLegStatus())
            {
                case Leg::__legStatus::stance: //stance
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
                    + mfMstance.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
                    // cout<<"K__stance_"<<(int)legNum<<"  "<<mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
                    // cout<<"B__stance_"<<(int)legNum<<"  "<<mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
                    // cout<<"M__stance_"<<(int)legNum<<"  "<<mfMstance.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) )<<endl;
                    break;

                case Leg::__legStatus::swing: //swing
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKswing.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBwing.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
                    + mfMswing.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );

                    break;

                case Leg::__legStatus::detach: //detach
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKdetach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBdetach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))   
                    + mfMdetach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
                    cout<<"K__detach_"<<(int)legNum<<"  "<<mfKdetach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
                    cout<<"B__detach_"<<(int)legNum<<"  "<<mfBdetach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
                    cout<<"M__detach_"<<(int)legNum<<"  "<<mfMdetach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) )<<endl;
                    break;

                case Leg::__legStatus::attach: //attach
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKattach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBattach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
                    + mfMattach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
                    cout<<"K__attach_"<<(int)legNum<<"  "<<mfKattach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
                    cout<<"B__attach_"<<(int)legNum<<"  "<<mfBattach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
                    cout<<"M__attach_"<<(int)legNum<<"  "<<mfMattach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) )<<endl<<endl;                 
                    break;
            }
            // cout<<stepFlag[legNum]<<endl;
             mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
            + mfKattach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
            + mfBattach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
            + mfMattach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
        }
        mfXcDot =  mfLegPresVel + mfXcDotDot * (1/fCtlRate);
        mfXc =  mfLegPresPos + (mfXcDot * (1/fCtlRate));
        
        // cout<<"mfForce_\n"<<mfForce.transpose()<<endl;
    }

}

void CRobotControl::CalVmcCom()
{
    Matrix<float,3,3> mfTempASM; 
    bool isOk=1;
    int stanceCount=0;
    int stanceUsefulCount=0;
    std::vector<int> stanceLegNum;
    Matrix<float,3,1> stanceAccumlate;
    stanceAccumlate.setZero();
    for(int legNum=0;legNum<4;legNum++)
    {
        if(GetLeg[legNum].GetLegStatus()==stance)
        { 
        stanceLegNum.push_back(legNum);
        Matrix<float,3,1> temp_vel = jacobian_vector[legNum] * jointPresVel.row(legNum).transpose();
        mfLegPresVel.row(legNum) = temp_vel.transpose();
            for(int i=0;i<3;i++)
            {
                if((mfLegPresVel.row(legNum)[i]-legLastVel.row(legNum)[i])/legLastVel.row(legNum)[i]>0.3||(mfLegPresVel.row(legNum)[i]-legLastVel.row(legNum)[i])/legLastVel.row(legNum)[i]<-0.3)
                isOk=0;
            
                if(i==2)
                    {
                        if(isOk)
                        {
                        stanceAccumlate+=mfLegPresVel.row(legNum).transpose();
                        stanceUsefulCount++;
                        }
                        isOk=1;
                    }
                    
            }
        }

    }
    mfVmcKdcom<<200,0,0,0,200,0,0,0,0;
    mfVmcKdbase<<200,0,0,0,200,0,0,0,200;
    Matrix<float,3,1> targetComVelxyz;
    targetComVelxyz=targetCoMVelocity.block(0,0,3,1);
    presentCoMVelocity=-stanceAccumlate/stanceUsefulCount;
    acc_DCOM=mfVmcKdcom*(targetComVelxyz-presentCoMVelocity);
   
    vector<int>::size_type tempSize=stanceLegNum.size();
    int stanceCount=(int)tempSize;
    omegaBase=targetCoMVelocity.block(3,0,3,1);
    updateImuData();
    vfVmcAngelAccDBase=mfVmcKdbase(omegaDBase-omegaBase);//cal angelAcc


    vfVmcb61<<mass*(acc_DCOM+vfGravity),Ixx*vfVmcAngelAccDBase[0],Iyy*vfVmcAngelAccDBase[1],Izz*vfVmcAngelAccDBase[2];
    int k=3*stanceCount;
    mfTempASM.setZero();
    mfVmcS66.setIdentity(6,6);
    mfVmcS66=mfVmcS66*100;
    mfVmcW.setIdentity(k,k);
    for(int i=0; i<4; i++)
    {
        ftsPosASM.push_back(mfTempASM);
    }
    for(int i=0; i<4; i++)
    {
    mfTempASM << 0, -legCmdPos(i, 2), legCmdPos(i, 1), legCmdPos(i, 2), 0, -legCmdPos(i, 0), -legCmdPos(i, 1), legCmdPos(i, 0), 0;
    ftsPosASM[i] = mfTempASM;
    }
    mfVmcA.resize(6,k);
    for(int i=0;i<stanceCount;i++)
    {
        mfVmcA.block<3,3>(0,i*3)<<MatrixXf::Identity(3, 3);
        mfVmcA.block<3,3>(3,i*3)<<ftsPosASM[stanceLegNum[i]];
    }
    mfVmcH.resize(k,k);
    mfVmcH=2*mfVmcA.transpose()*vfVmcS66*mfVmcA+2*mfVmcW;
    vfVmcg61=-2*mfVmcA.transpose()*mfVmcS66*vfVmcb61;

    //qp
    qpOASES::Options options;
    options.initialStatusBounds = qpOASES::ST_INACTIVE;
	options.numRefinementSteps = 3;
	options.enableCholeskyRefactorisation = 1;
    qpOASES::QProblemB qpPrograme(k);
    qpPrograme.setOptions(options);
    qpOASES::real_t qp_H[k*k]={};
    qpOASES::real_t qp_g[6]={};
    qpOASES::real_t lb[6]={-200,-200,-200,-200,-200,-200};
    qpOASES::real_t ub[6]={200,200,200,200,200,200};
        for(int i=0; i<k; i++)
    {
        qp_g[i] = vfVmcg61(i,0);
        for(int j=0; j<k; j++)
        {
        qp_H[i*6+j] = mfVmcH(i,j);
        }
    }
    qpOASES::int_t nWSR = 10;
    qpPrograme.init(qp_H,qp_g,lb,ub,nWSR,0);
    qpOASES::real_t xOpt[k];
	qpPrograme.getPrimalSolution( xOpt );
    for(int i=0;i<3*stanceCount;i=i+3)
    {
        mfTargetForce.row(stanceLegNum[i/3])<<xOpt[i],xOpt[i+1];xOpt[i+2];
    }
}

