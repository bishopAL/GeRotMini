#include "robotcontrol.h"
using namespace Eigen;
RobotControl::RobotControl(__controlMode mode)
{
    float impdata[200];
    string2float("../include/adm_parameter.csv",impdata);   // 0-swing,1-attach,2-stance,3-detach
    for(int i=0; i<4; i++)
    {
        Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata + 36 * i), mapB(impdata + 12 + 36 * i), mapM(impdata  + 24 + 36 * i);
        changePara(mapK,mapB,mapM, i);
    } 
    xc_dotdot.setZero();
    xc_dot.setZero();
    xc.setZero();
    target_pos.setZero();
    target_vel.setZero();
    target_acc.setZero();
    target_force.setZero();
    force_last.setZero();
    impCtlRate = 100;
    // cal inertial
    Ixx=((_length*_length)+(_height*_height))*mass/12;
    _controlMode=mode;
}

void RobotControl::updateImuData()
{
    api.updateIMU();
    omegaBase<<api.fAcc;
    gravity<<api.fGyro;
}

void RobotControl::updateFtsPresForce()
{
    Matrix<float, 3, 4> temp;
    if(force(2,3) - force_last(2,3) > 0.3 || force(2,3) - force_last(2,3) < -0.3)
        temp.setZero();
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = motors.present_torque[i+j*3];
    for (int i=0; i<4; i++)
        force.col(i) = ForceLPF * force_last.col(i) + (1-ForceLPF) * leg[i]._jacobian.transpose().inverse() * temp.col(i);
    force_last = force;
}

void RobotControl::updateTargTor(Matrix<float, 3, 4> force)
{
    for (int legNum=0; legNum<4; legNum++)
        target_torque.col(legNum) = leg[legNum]._jacobian * force.col(i); 
}

void RobotControl::ParaDeliver()
{
    #ifdef  VMCCONTROL
    calVmcCom();
    #else
    target_pos = legCmdPos;
    for(uint8_t legNum=0; legNum<4; legNum++)
    {   
        if(leg[legNum].getLegStatus() == swing) //swing
        {
            if( ( timePresentForSwing(legNum) - (timeForSwing(legNum) - timePeriod *8) ) > -1e-4 )
            {
                leg[legNum].getLegStatus() = attach;   //attach
                target_force.row(legNum) << 0, 0, -1.6;  // x, y, z
            }
            else if( ( timePresentForSwing(legNum) - timePeriod *8 ) < 1e-4 && timePresentForSwing(legNum) > 1e-4)
            {
                leg[legNum].getLegStatus() = detach;   //detach
                target_force.row(legNum) << 0, 0, 1.5;
            }
            else    //swing
            {
                leg[legNum].getLegStatus() = swing;
                target_force.row(legNum) << 0, 0, 0;
            }
        }
        else        //stance
        {
            leg[legNum].getLegStatus() = stance;
            target_force.row(legNum) << -0.6, 0, -1.6;    
        }
    }
    #endif
}

void RobotControl::ChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM)
{
        switch(mode) 
    {
        case 0:
            K_stance = mK; B_stance = mB; M_stance = mM;
            break;
        case 1:
            K_swing = mK; B_swing = mB; M_swing = mM;
            break;
        case 2:
            K_detach = mK; B_detach = mB; M_detach = mM;
            break;
        case 3:
            K_attach = mK; B_attach = mB; M_attach = mM;   
            break; 
    }
}

void RobotControl::control()
{
    if(_controlMode == IMPEDANCE)  // Impedance control
    {
        xc_dot = (legPresPos - legPos_last) * impCtlRate;
        xc_dotdot = (legPresVel - xc_dot) * impCtlRate;
        for(uint8_t legNum=0; legNum<4; legNum++)
        {
            force.transpose().row(legNum) = target_force.row(legNum)  
            + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
            + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) ;
            // + M_stance.row(legNum).cwiseProduct(target_acc.row(legNum) - xc_dotdot.row(legNum));
            cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<endl;
            cout<<"K__stance_"<<(int)legNum<<"  "<<K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
            cout<<"B__stance_"<<(int)legNum<<"  "<<B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
            // cout<<"force"<<force.transpose().row(legNum)<<endl;
        }
        updateTargTor(force);
        // cout<<"target_pos:"<<endl<<target_pos<<endl;
        // cout<<"legPresPos:"<<endl<<legPresPos<<endl;
        cout<<"force:"<<endl<<force<<endl;
        cout<<"target_torque:"<<endl<<target_torque<<endl<<endl;
    }
    else if(_controlMode == ADMITTANCE)  // Admittance control    xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    {
        for(uint8_t legNum=0; legNum<4; legNum++)
        {   
            switch(leg[legNum].getLegStatus())
            {
                case Leg::__legStatus::stance: //stance
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    // cout<<"K__stance_"<<(int)legNum<<"  "<<K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    // cout<<"B__stance_"<<(int)legNum<<"  "<<B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    // cout<<"M__stance_"<<(int)legNum<<"  "<<M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                    break;

                case Leg::__legStatus::swing: //swing
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_swing.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_swing.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_swing.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );

                    break;

                case Leg::__legStatus::detach: //detach
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))   
                    + M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    cout<<"K__detach_"<<(int)legNum<<"  "<<K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    cout<<"B__detach_"<<(int)legNum<<"  "<<B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    cout<<"M__detach_"<<(int)legNum<<"  "<<M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                    break;

                case Leg::__legStatus::attach: //attach
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    cout<<"K__attach_"<<(int)legNum<<"  "<<K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    cout<<"B__attach_"<<(int)legNum<<"  "<<B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    cout<<"M__attach_"<<(int)legNum<<"  "<<M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl<<endl;                 
                    break;
            }
            // cout<<stepFlag[legNum]<<endl;
             xc_dotdot.row(legNum) =  target_acc.row(legNum) 
            + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
            + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
            + M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
        }
        xc_dot =  legPresVel + xc_dotdot * (1/impCtlRate);
        xc =  legPresPos + (xc_dot * (1/impCtlRate));
        
        // cout<<"force_\n"<<force.transpose()<<endl;
    }

}

void RobotControl::calVmcCom()
{
    bool isOk=1;
    int stanceCount=0;
    int stanceUsefulCount=0;
    std::vector<int> stanceLegNum;
    Matrix<float,3,1> stanceAccumlate;
    stanceAccumlate.setZero();
    for(int legNum=0;legNum<4;legNum++)
    {
        if(leg[legNum].getLegStatus()==stance)
        { 
        stanceLegNum.push_back(legNum);
        Matrix<float,3,1> temp_vel = jacobian_vector[legNum] * jointPresVel.row(legNum).transpose();
        legPresVel.row(legNum) = temp_vel.transpose();
            for(int i=0;i<3;i++)
            {
                if((legPresVel.row(legNum)[i]-legLastVel.row(legNum)[i])/legLastVel.row(legNum)[i]>0.3||(legPresVel.row(legNum)[i]-legLastVel.row(legNum)[i])/legLastVel.row(legNum)[i]<-0.3)
                isOk=0;
            
                if(i==2)
                    {
                        if(isOk)
                        {
                        stanceAccumlate+=legPresVel.row(legNum).transpose();
                        stanceUsefulCount++;
                        }
                        isOk=1;
                    }
                    
            }
        }

    }
    Kdcom<<200,0,0,0,200,0,0,0,0;
    Kdbase<<200,0,0,0,200,0,0,0,200;
    Matrix<float,3,1> targetComVelxyz;
    targetComVelxyz=targetCoMVelocity.block(0,0,3,1);
    presentCoMVelocity=-stanceAccumlate/stanceUsefulCount;
    acc_DCOM=Kdcom*(targetComVelxyz-presentCoMVelocity);
   
    vector<int>::size_type tempSize=stanceLegNum.size();
    int stanceCount=(int)tempSize;
    omegaBase=targetCoMVelocity.block(3,0,3,1);
    updateImuData();
    angelAcc_DBase=Kdbase(omegaDBase-omegaBase);//cal angelAcc


    b61<<mass*(acc_DCOM+gravity),Ixx*angelAcc_DBase[0],Iyy*angelAcc_DBase[1],Izz*angelAcc_DBase[2];
    int k=3*stanceCount;
    tempASM.setZero();
    S66.setIdentity(6,6);
    S66=S66*100;
    W.setIdentity(k,k);
    for(int i=0; i<4; i++)
    {
        ftsPosASM.push_back(tempASM);
    }
    for(int i=0; i<4; i++)
    {
    tempASM << 0, -legCmdPos(i, 2), legCmdPos(i, 1), legCmdPos(i, 2), 0, -legCmdPos(i, 0), -legCmdPos(i, 1), legCmdPos(i, 0), 0;
    ftsPosASM[i] = tempASM;
    }
    A.resize(6,k);
    for(int i=0;i<stanceCount;i++)
    {
        A.block<3,3>(0,i*3)<<MatrixXf::Identity(3, 3);
        A.block<3,3>(3,i*3)<<ftsPosASM[stanceLegNum[i]];
    }
    H.resize(k,k);
    H=2*A.transpose()*S66*A+2*W;
    g61=-2*A.transpose()*S66*b61;

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
        qp_g[i] = g61(i,0);
        for(int j=0; j<k; j++)
        {
        qp_H[i*6+j] = H(i,j);
        }
    }
    qpOASES::int_t nWSR = 10;
    qpPrograme.init(qp_H,qp_g,lb,ub,nWSR,0);
    qpOASES::real_t xOpt[k];
	qpPrograme.getPrimalSolution( xOpt );
    for(int i=0;i<3*stanceCount;i=i+3)
    {
        target_force.row(stanceLegNum[i/3])<<xOpt[i],xOpt[i+1];xOpt[i+2];
    }
}

