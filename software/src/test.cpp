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
void PumpPositive(uint8_t legNum)
{
    svStatus|=1<<((3-legNum)<<1+1);
    svStatus&=0<<((3-legNum)<<1);
    api.setSV(svStatus);
    
}
void PumpNegtive(uint8_t legNum)
{   
    svStatus|=1<<((3-legNum)<<1);
    svStatus&=0<<((3-legNum)<<1+1);
    api.setSV(svStatus);
}

int main()
{
    API api;
    vector<int> ID;
    vector<float> start_pos;
    vector<float> target_tor;
    for(int i=1; i<=12; i++)
    {
    ID.push_back(i);
    start_pos.push_back(0.00);
    }
    DxlAPI gecko("/dev/ttyAMA0", 3000000, ID, 2);
    // gecko.setBaudRate(5);
    gecko.setOperatingMode(3);  //3 position control; 0 current control
    gecko.torqueEnable();
    gecko.setPosition(start_pos);
    usleep(1e6);
    api.setPump(1, LOW);
    api.setPump(24, LOW);
    api.setPump(28, LOW);
    api.setPump(29, LOW);
  
    api.setSV(svStatus);
    while(1)
    {
        // set SV, pump, update IMU
        // api.setSV(0b01010101);
        // api.setPump(1, HIGH);
        // api.setPump(24, HIGH);
        // api.setPump(28, HIGH);
        // api.setPump(29, HIGH);
        // api.updateIMU();
        // // update power system
        // api.updatePowerStatus();
        // usleep(1e6);


  
        //api.updateIMU();
        //api.updatePowerStatus();
        for(int i=0;i<4;i++)
        {
            PumpPositive(i);
            usleep(2e6);
            PumpNegtive(i);
        }
        usleep(1e6);

    }


}
