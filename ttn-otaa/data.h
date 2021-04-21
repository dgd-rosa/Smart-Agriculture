#ifndef DATA_H 
#define DATA_H
#include "Arduino.h"

uint8_t* uplinkMessageFormat(int moisture, byte humidity, byte temperature, bool pumpState);

#endif 
