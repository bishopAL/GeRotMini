#include "i2c.h"
#include "api.h"
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h> 



#define LF	1
#define RF	24
#define LH	28
#define RH	29

int main()
{
    API api;
    

    while(1)
    {
        // set SV, pump, update IMU
        api.setSV(0b00000000);
        // api.setPump(1, HIGH);
        // api.setPump(24, HIGH);
        // api.setPump(28, HIGH);
        // api.setPump(29, HIGH);
        api.updateIMU();
        // update power system
        api.updatePowerStatus();
        usleep(1e6);

        api.setSV(0b11111111);
        // api.setPump(1, LOW);
        // api.setPump(24, LOW);
        // api.setPump(28, LOW);
        // api.setPump(29, LOW);
        api.updateIMU();
        api.updatePowerStatus();
        usleep(1e6);

    }
    

}