#include"robotcontrol.h"

#define CHECK_RET(q) if((q)==false){return 0;}
CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
bool runFlag=1;
void *udpConnect(void *data)
{

	string ip = "127.0.0.1";
	uint16_t port = 8888;

	CUdpSocket srv_sock;
	//创建套接字
	CHECK_RET(srv_sock.Socket());
	//绑定地址信息
	CHECK_RET(srv_sock.Bind(ip, port));
	while(1)
	{
		//接收数据
		string buf;
		string peer_ip;
		uint16_t peer_port;
		CHECK_RET(srv_sock.Recv(&buf, &peer_ip, &peer_port));
		cout << "UpperComputer["<<peer_ip<<":"<<peer_port<<"] Command: " << buf << endl;
        //buf match command 
        int ret=commandJudge((char*)string("start").c_str(),(char *)buf.c_str());
        if(ret) {runFlag=1; goto END;}
        ret=commandJudge((char*)string("stop").c_str(),(char *)buf.c_str());
        if(ret) {runFlag=0; goto END;}
        // int ret=match((char*)string("start").c_str(),(char*)string("startsada").c_str());
        // cout<<(char*)string("start").c_str()<<endl;
        // cout<<ret<<endl;
		//发送数据
        END:
		buf.clear();
		// cout << "server say: ";
		// cin >> buf;
		// CHECK_RET(srv_sock.Send(buf, peer_ip, peer_port));
	}
	//关闭套接字
	srv_sock.Close();
	return 0;
}
void *robotStateUpdateSend(void *data)
{
    Matrix<float,4,2> TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Matrix<float, 6,1> TCV;
    TCV << VELX, 0, 0,0,0,0 ;// X, Y , alpha 
    
    
    //motors initial

#if(INIMODE==1)
    vector<float> init_Motor_angle(12);
    float float_init_Motor_angle[12];
    string2float("../include/init_Motor_angle.csv", float_init_Motor_angle);//Motor angle     d
    //cout<<"____________"<<endl;
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            float_init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j] * 3.1416/180; //to rad
            init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j];      //vector
            rbt.mfJointCmdPos(i,j) = float_init_Motor_angle[i*3+j];            //rbt.forwardKinematics
            //cout<<init_Motor_angle[i*3+j]<<endl;
        }
    rbt.forwardKinematics(0);
    rbt.setInitPos(rbt.mfLegCmdPos);        //legCmdPos
    cout<<"legCmdPos:\n"<<rbt.legCmdPos<<endl ;

    motors.setPosition(init_Motor_angle);
#endif    
 
    //      rbt initial
    // TimeForStancePhase<< 0,                       TimeForGaitPeriod/2.0,     // diagonal
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      0,                       TimeForGaitPeriod/2.0;
    // TimeForSwingPhase<< TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0 *3,   // tripod
    //                      0,             TimeForGaitPeriod/4.0,
    //                      TimeForGaitPeriod/4.0 *3,    TimeForGaitPeriod,
    //                      TimeForGaitPeriod/4.0  ,          TimeForGaitPeriod/4.0 *2;
    TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
                        0,		 		 					3*TimeForGaitPeriod/16,		
                        12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
                        4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);
   
//    for (size_t i = 0; i < 4; i++)
//    {
//     cout<<rbt.iStatusCounter[i]<<endl;
//     cout<<"legstatus_"<<i<<": "<<(rbt.m_glLeg[i]->GetLegStatus())<<endl;
//    }
// for (size_t i = 0; i < 4; i++)
// {
//     for (size_t j = 0; j < 5; j++)
//     {
//         cout<<rbt.iStatusCounterBuffer[i][j]<<",";
//     }
//     cout<<endl;
// }

   
   
//    cout<<rbt.mfTimeForSwingPhase<<endl;

#if(INIMODE==2)
//    float  float_initPos[12]={    60, 60, -30,
//                                  60,-60, -30,
//                                 -60, 60, -30,
//                                 -60,-60, -30};
    float  float_initPos[12];
    string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            InitPos(i, j) = float_initPos[i*3+j];
            //cout<<InitPos(i, j)<<endl;
        }
    rbt.SetInitPos(InitPos);
#endif
  
   
    rbt.SetCoMVel(TCV);
    rbt.InverseKinematics(rbt.mfLegCmdPos);
  
  
#if(INIMODE==2)  
    rbt.SetPos(rbt.mfJointCmdPos);
#endif

    rbt.mfTargetPos = rbt.mfLegCmdPos;
    rbt.bInitFlag = 1;
    usleep(1e5);
    while(1)
    {
        if(runFlag)
        {
            struct timeval startTime,endTime;
            double timeUse;
            gettimeofday(&startTime,NULL);
            //If stay static, annotate below one line.
            rbt.NextStep();//
            rbt.ParaDeliver();
            cout<<"LegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 10); 
            //else
            // cout<<"TimeRobotStateUpdateSend: "<<timeUse<<endl;
        }
    }
 
}

void *runImpCtller(void *data)
{
    
    
    struct timeval startTime,endTime;
    double timeUse;
    int run_times=0;    // for debugging

    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);
    usleep(1e5);
    while (1)
    {
        if(runFlag)
        {
            gettimeofday(&startTime,NULL);
            /* get motors data  */
            rbt.dxlMotors.getTorque();
            rbt.dxlMotors.getPosition();
            rbt.dxlMotors.getVelocity();

            /* update the data IMP need */
            rbt.UpdatejointPresPos();         
            rbt.UpdatejointPresVel();
            rbt.ForwardKinematics(1);
            rbt.UpdateJacobians();
            rbt.UpdateFtsPresVel();
            rbt.UpdateFtsPresForce();  

            //rbt.mfTargetPos<<rbt.mfInitFootPos;
            rbt.Control();   
            // rbt.InverseKinematics(rbt.mfXc);   //    Admittance control
            rbt.InverseKinematics(rbt.mfLegCmdPos); //    Postion control

            // cout<<"mfJointCmdPos:"<<rbt.mfJointCmdPos;
            // cout<<"target_pos: \n"<<rbt.mfTargetPos<<endl;
            // cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
            // cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
            // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // cout<<endl;

            /*      Admittance control      */
            rbt.SetPos(rbt.mfJointCmdPos);

            /*      Impedance control      */
            // for(int i=0; i<4; i++)  
            //     for(int j=0;j<3;j++)
            //         SetTorque[i*3+j] = rbt.target_torque(j,i);
            // motors.setTorque(SetTorque); 

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateImpCtller*1e6 - (double)(timeUse) - 10); 
            else
                cout<<"timeImpCtller: "<<timeUse<<endl;
        }
    }
  
}

int main(int argc, char ** argv)
{   
    
    
    pthread_t th1, th2, th3;
	int ret;
    ret = pthread_create(&th1,NULL,udpConnect,NULL);
    if(ret != 0)
	{
		printf("create pthread1 error!\n");
		exit(1);
	}
    ret = pthread_create(&th2,NULL,robotStateUpdateSend,NULL);
    if(ret != 0)
	{
		printf("create pthread2 error!\n");
		exit(1);
	}
    ret = pthread_create(&th3,NULL,runImpCtller,NULL);
    if(ret != 0)
	{
		printf("create pthread3 error!\n");
		exit(1);
	}
    
    pthread_join(th2, NULL);
    pthread_join(th3, NULL);
    while(1);

    
    
    return 0;
}