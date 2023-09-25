#include <iostream>
#include <wiringPi.h>

#include "GestionINA219.h"

using namespace std;

GestionINA219 ina219;

int main() {
	cout << "init :" << ina219.init(ADDR_40) << endl;
	cout << "reset:" << ina219.reset() << endl;
	cout << "cal  :" << ina219.setCalibration_0_4A(_16V, B_12Bits_128S_69MS, S_12Bits_128S_69MS, ShuntAndBusVoltageContinuous) << endl;
	for(int compteur = 0; ; compteur++) {
		bool conversion = false;
		while(!conversion){
			conversion  = ina219.isConversionOk();
		}
		// Mesure
		float busVoltage = ina219.getBusVoltage_V();
		float current = ina219.getCurrent_mA();
		float power = ina219.getPower_W();

		cout << "Bus voltage V    :" << busVoltage << endl;
		cout << "Current mA       :" << current << endl;
		cout << "Power W          :" << power << endl << endl;
		delay(1000);
	}
}
