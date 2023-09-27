#include "i2c.h"
#include "api.h"
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h> 
#include "GestionINA219.h"


#define LF	1
#define RF	24
#define LH	28
#define RH	29

int main()
{
    API api;
    GestionINA219 ina219;
    ina219.init(ADDR_40);
    ina219.reset();
    ina219.setCalibration_0_4A(_16V, B_12Bits_128S_69MS, S_12Bits_128S_69MS, ShuntAndBusVoltageContinuous);

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
        bool conversion = false;
		while(!conversion){
			conversion  = ina219.isConversionOk();
		}
        float busVoltage = ina219.getBusVoltage_V();
		float current = ina219.getCurrent_mA();
		float power = ina219.getPower_W();
        usleep(1e6);

        api.setSV(0b11111111);
        // api.setPump(1, LOW);
        // api.setPump(24, LOW);
        // api.setPump(28, LOW);
        // api.setPump(29, LOW);
        api.updateIMU();
        conversion = false;
		while(!conversion){
			conversion  = ina219.isConversionOk();
		}
        busVoltage = ina219.getBusVoltage_V();
		current = ina219.getCurrent_mA();
		power = ina219.getPower_W();
        usleep(1e6);
    }
    

}