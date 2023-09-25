#include "api.h"
#include <iostream>
static int fd;
u8 sendValue[2];
#define I2C_ADD_PCF 0x20

using namespace std;


/**
 * @brief a function to set solid valves' status
 * 
 * @param value a value int number, sequencing at LF0-LF1-RF0-RF1-LH0-LH1-RH0-RH1, like 01100110
 */
void setSV(u8 value)
{
    
    sendValue[0] = value;
    i2c_write_data(I2C_ADD_PCF, sendValue[0], sendValue, 1);
}