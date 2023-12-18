#include "i2c.h"
#include "api.h"
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include "dynamixel.h"
#include <vector>

#define LF_PIN      1
#define RF_PIN      24
#define LH_PIN      28
#define RH_PIN      29
uint8_t svStatus=0b01010101;


int main()
{
    API api;
    vector<int> ID;
    vector<float> start_pos;
    vector<float> target_tor;
        // ID.push_back(12);
    // start_pos.push_back(0.00);
    for(int i=0; i<12; i++)
    {
    ID.push_back(i);
    start_pos.push_back(0.00);
    }
    DxlAPI gecko("/dev/ttyAMA0", 3000000, ID, 2); //ttyUSB0
    // gecko.setBaudRate(5);
    gecko.setOperatingMode(3);  //3 position control; 0 current control
    gecko.torqueEnable();
    gecko.setPosition(start_pos);
    gecko.getPosition();
    //~ usleep(1e6);
    //~ api.setPump(1, LOW);
    //~ api.setPump(24, LOW);
    //~ api.setPump(28, LOW);
    //~ api.setPump(29, LOW);
  
    // api.setSV(svStatus);
    while(1)
    {
        // set SV, pump, update IMU
        //  api.setSV(0b01010101);
        // api.setPump(1, HIGH);
        // api.setPump(24, HIGH);
        // api.setPump(28, HIGH);
        // api.setPump(29, HIGH);
        api.updateIMU();
        usleep(1e6);
        // // update power system
        // api.updatePowerStatus();
        // start_pos[0]=0.1;
        // gecko.setPosition(start_pos);
        gecko.getPosition();
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
        usleep(1e6);

    }


}
