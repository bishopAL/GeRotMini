#include"robotcontrol.h"

using namespace std;
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2
#define _JOYSTICK 1
#define MORTOR_ANGLE_AMP 40*3.14/180.0
#define loopRateCommandUpdate 100.0   //hz
#define loopRateStateUpdateSend 20.0   //hz
#define loopRateImpCtller 100.0   //hz
#define VELX 6.0    // mm  step length = VELX * timeForStancePhase        
#define TimePeriod 0.05
#define TimeForGaitPeriod 6

RobotControl rbt(ADMITTANCE,120,60,20,500);
void *robotStateUpdateSend(void *data)
{
    Matrix<float, 4, 2>TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Vector<float, 3> TCV={ VELX, 0, 0 };// X, Y , alpha 
    

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
    TimeForSwingPhase<< TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0 *3,   // tripod
                         0,             TimeForGaitPeriod/4.0,
                         TimeForGaitPeriod/4.0 *3,    TimeForGaitPeriod-TimePeriod,
                         TimeForGaitPeriod/4.0  ,          TimeForGaitPeriod/4.0 *2;
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);

#if(INIMODE==2)
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
        struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);

        //If stay static, annotate below one line.
        rbt.NextStep();//
        rbt.ParaDeliver();
        // cout<<"legCmdPos:\n"<<rbt.legCmdPos<<endl;

        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1e4)
            usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 10); 
        else
            cout<<"TimeRobotStateUpdateSend: "<<timeUse<<endl;

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
        gettimeofday(&startTime,NULL);
        /* get motors data  */
        while( motors.getTorque()==false || motors.getPosition()==false || motors.getVelocity()==false );
        /* update the data IMP need */
        rbt.UpdatejointPresPos();         
        rbt.UpdatejointPresVel();
        rbt.ForwardKinematics(1);
        rbt.UpdateJacobians();
        rbt.UpdateFtsPresVel();

        rbt.UpdateFtsPresForce();  

        // rbt.inverseKinematics(rbt.target_pos); //    within rbtCtller
        rbt.mfTargetPos<<rbt.mfInitFootPos;
        rbt.Control();   
        rbt.InverseKinematics(rbt.mfXc);   //    Admittance control
        
        // cout<<"target_pos: \n"<<rbt.target_pos<<endl;
        cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
        cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
        // cout<<"xc_dotdot: \n"<<rbt.xc_dotdot<<"; \nxc_dot: \n"<<rbt.xc_dot<<"; \nxc: \n"<<rbt.xc<<endl;
        // cout<<"legPresPos: \n"<<rbt.legPresPos<<endl;
        cout<<endl;

        /*      Admittance control      */
        rbt.setPos(rbt.mfJointCmdPos);

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

int main(int argc, char ** argv)
{
    // Lcm.subscribe("ROBOTCOMMAND", &RobotStateHandler::handleMessage, &rsHandle);

    pthread_t th1, th2, th3;
	int ret;

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