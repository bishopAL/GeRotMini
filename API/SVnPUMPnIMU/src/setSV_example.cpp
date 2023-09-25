#include "i2c.h"
#include "api.h"
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h> 

#define TIMEOUT	3
#define RETRY	3

static int fd;
u8 testing[2];
unsigned char *i2c_dev = (unsigned char *)"/dev/i2c-1";


int main()
{
    /**
     * @brief Turn on i2c device
     * 
     */
    fd = i2c_open(i2c_dev, 3, 3);

    /**
     * @brief Construct a new wiring Pi Setup object, set the pump pin as OUTPUT mode
     * 
     */
    wiringPiSetup();
    pinMode(1, OUTPUT); 
    pinMode(24, OUTPUT); 
    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);

    while(1)
    {
        setSV(0b00000000);
        digitalWrite(1, HIGH);
        digitalWrite(24, HIGH);
        digitalWrite(28, HIGH);
        digitalWrite(29, HIGH);
        usleep(1e6);

        setSV(0b01100110);
        digitalWrite(1, LOW);
        digitalWrite(24, LOW);
        digitalWrite(28, LOW);
        digitalWrite(29, LOW);
        usleep(1e6);
    }
    

}