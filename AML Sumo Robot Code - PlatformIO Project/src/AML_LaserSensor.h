#ifndef AML_LaserSensor_h
#define AML_LaserSensor_h

#include <Arduino.h>
#include <VL53L0X.h>

void AML_LaserSensor_setup();
int AML_LaserSensor_readSingle(int laserNumber);
void AML_LaserSensor_readAll(int *userArrayAddress);
void AML_LaserSensor_readAllTest(int *userArrayAddress);


#endif