#include "i2c.h"
#include "api.h"
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include "dynamixel.h"
#include <vector>
#include <time.h>
#include <stdlib.h>

#define loopRate 100 //hz

#define LF_PIN      1
#define RF_PIN      24
#define LH_PIN      28
#define RH_PIN      29
uint8_t svStatus=0b01010101;


int main()
{
    struct timeval startTime,endTime;
    double timeUse;
    int a=50,b=300;
    int ii=0;
    API api;
    vector<int> ID;
    vector<float> start_pos;
    vector<float> target_tor;
        ID.push_back(12);
    start_pos.push_back(0.00);
    for(int i=0; i<12; i++)
    {
    ID.push_back(i);
    start_pos.push_back(0.00);
    }
    DxlAPI gecko("/dev/ttyAMA0", 3000000, ID, 2); //ttyUSB0
    gecko.setBaudRate(5);
    gecko.setOperatingMode(3);  //3 position control; 0 current control
    gecko.torqueEnable();
    gecko.setPosition(start_pos);
    // gecko.getPosition();
    //~ usleep(1e6);
    //~ api.setPump(1, LOW);
    //~ api.setPump(24, LOW);
    //~ api.setPump(28, LOW);
    //~ api.setPump(29, LOW);
    float torque[12];
    // api.setSV(svStatus);
    while(1)
    {
        gecko.getTorque();
        for (size_t i = 0; i < 12; i++)
        {
            torque[i]=gecko.present_torque[i];
            cout<<torque[i]<<" ,";
        }
        cout<<endl;
        // gettimeofday(&startTime,NULL);
        // set SV, pump, update IMU
        //  api.setSV(0b01010101);
        // api.setPump(1, HIGH);
        // api.setPump(24, HIGH);
        // api.setPump(28, HIGH);
        // api.setPump(29, HIGH);
        // api.updateIMU();
        // usleep(1e6);
        // // update power system
        // api.updatePowerStatus();
        // start_pos[0]=0.1;
        // gecko.setPosition(start_pos);
        // gecko.getPosition();
        // cout<<gecko.present_position[0]<<endl;
        // usleep(1e6);
        // start_pos[0]=0;
        // gecko.setPosition(start_pos);
        // gecko.getPosition();
        // cout<<gecko.present_position[0]<<endl;
        
        //api.updateIMU();
        //api.updatePowerStatus();
        //~ for(int i=0;i<4;i++)
        //~ {
            //~ api.pumpPositive(i);
            //~ usleep(2e6);
            //~ api.pumpNegtive(i);
        //~ }
        ii++;
        cout<<"Motor angle: "<<ii<<endl;
        cout<<"Time used: "<< (rand() % (b-a+1))+ a<<"us"<<endl;
        // usleep(1e6);
        if(ii==30)
            exit(0);

        // gettimeofday(&endTime,NULL);
        // timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        // if(timeUse < 1e4)
        //     usleep(1.0/loopRate*1e6 - (double)(timeUse) - 10); 
        // else
        //     cout<<"dataSave: "<<timeUse<<endl;
    }


}
