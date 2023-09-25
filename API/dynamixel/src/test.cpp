#include <dynamixel.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#define TC 0

using namespace std;
int main()
{
    vector<int> ID;
    vector<float> start_pos;
    vector<float> target_tor;
    float K = 0.1;
    float D = 0.005;
    // for(int i=4; i<=1; i++)
    // {
    ID.push_back(4);
    // }
    start_pos.push_back(0.0);
    // for(int i=1; i<=1; i++)
    // {
    // target_tor.push_back(0.0);
    // }
    DxlAPI gecko("/dev/ttyAMA0", 3000000, ID, 2);

    gecko.setOperatingMode(3);  //3 position control; 0 current control
    gecko.torqueEnable();
    gecko.setPosition(start_pos);
    usleep(1e6);
    if(TC)
    {
        gecko.torqueDisable();
        gecko.setOperatingMode(0);
        gecko.torqueEnable();
    }
    
    for(int times=0; times<10000; times++)
    {
        if(TC)
        {
	struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);
            gecko.getPosition();
            gecko.getVelocity();
            //target_tor[0] = K*(0-gecko.present_position[0]) + D*(0-gecko.present_velocity[0]);
	for(int nums=0; nums<12; nums++)
	{
	target_tor[nums] = 0.005;
	}
            gecko.setTorque(target_tor);
	gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            cout<<"Time: "<< times<<" , TimeUse: "<<timeUse<<" , Pos: "<<gecko.present_position[0]<<" , tor: "<< target_tor[0]<<endl;
        }
        else{
            gecko.getTorque();
            cout<<"Time: "<< times<<" , present torque: "<<gecko.present_torque[0]<<endl;
        }
    }
    gecko.torqueDisable();
}
