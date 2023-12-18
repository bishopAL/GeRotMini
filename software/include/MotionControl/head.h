#ifndef head_H
#define head_H

// #define VMCCONTROL
#define ForceLPF  0.9
#define StepHeight_F  19.0  //swingUp
#define StepHeight_H  14.0  //swingUp
#define Press  16.0       //attach press
#define CompensationDistanceA1 -2     // AttitudeCorrection() Amble gait
#define CompensationDistanceA2 12  
#define CompensationDistanceA3 14  
#define CompensationDistanceALL 8     // AttitudeCorrection() All stace phase
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2
#define MORTOR_ANGLE_AMP 40*3.14/180.0
#define loopRateCommandUpdate 100.0   //hz
#define loopRateStateUpdateSend 20.0   //hz
#define loopRateImpCtller 100.0   //hz
#define loopRateDataSave 10 //hz
#define VELX 4.0    // mm  step length = VELX * timeForStancePhase        
#define TimePeriod 0.05
#define TimeForGaitPeriod 8
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <time.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <map>
#include <string>
#include <string.h>
#include "i2c.h"
#include "dynamixel.h"
#include "api.h"
#include <stdio.h>
#include <wiringPi.h> 
#include<fstream>
#include<sstream>
#include<bitset>
#ifdef  VMCCONTROL
  #include <qpOASES.hpp>
#endif
using namespace Eigen;
using namespace std;
#endif