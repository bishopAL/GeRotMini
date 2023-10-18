#ifndef CONTROLHELPER_H
#define CONTROLHELPER_H
#include "head.h"
enum enum_LEGSTATUS{swing=0,attach,stance,detach}; 
enum enum_LEGNAME{LF,RF,LH,RH};
enum enum_CONTROLMODE{ADMITTANCE,IMPEDANCE};
void string2float(std::string add, float* dest);
void printSvStatus(unsigned char svStatus);
#endif