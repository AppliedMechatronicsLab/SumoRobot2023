/*
  Library for 7 VL53L0x sensor

  Last updated: 11PM 13/06/2023 UTC+7
  Authors: anmh1205

*/

#include <AML_LaserSensor.h>


// Khởi tạo theo thư viện
VL53L0X FL;
VL53L0X FF;
VL53L0X FR;
VL53L0X RR;
VL53L0X BR;
VL53L0X BL;
VL53L0X RL;


// set the pins to shutdown for all 7 sensors

#define SHT_L0X_FL 22 // XSHUT Pin laser 1
#define SHT_L0X_FF 24 // XSHUT Pin laser 2
#define SHT_L0X_FR 26 // XSHUT Pin laser 3
#define SHT_L0X_RR 28 // XSHUT Pin laser 4
#define SHT_L0X_BR 30 // XSHUT Pin laser 5
#define SHT_L0X_BL 32 // XSHUT Pin laser 6
#define SHT_L0X_RL 34 // XSHUT Pin laser 7


// address we will assign for all 7 sensor
#define L0X_FL_ADDRESS 0x28
#define L0X_FF_ADDRESS 0x30
#define L0X_FR_ADDRESS 0x31
#define L0X_RR_ADDRESS 0x32
#define L0X_BR_ADDRESS 0x33
#define L0X_BL_ADDRESS 0x34
#define L0X_RL_ADDRESS 0x35


// Mảng giá trị cảm biến
int AML_laserSensorValue[8]; 


// bật từng laser lên để cài địa chỉ bằng chân XSHUT
void AML_LaserSensor_setup()
{
  int index = 0;
  pinMode(SHT_L0X_FL, OUTPUT);
  pinMode(SHT_L0X_FF, OUTPUT);
  pinMode(SHT_L0X_FR, OUTPUT);
  pinMode(SHT_L0X_RR, OUTPUT);
  pinMode(SHT_L0X_BR, OUTPUT);
  pinMode(SHT_L0X_BL, OUTPUT);
  pinMode(SHT_L0X_RL, OUTPUT);

  Serial.println(index++);
  // Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_L0X_FL, LOW);
  digitalWrite(SHT_L0X_FF, LOW);
  digitalWrite(SHT_L0X_FR, LOW);
  digitalWrite(SHT_L0X_RR, LOW);
  digitalWrite(SHT_L0X_BR, LOW);
  digitalWrite(SHT_L0X_BL, LOW);
  digitalWrite(SHT_L0X_RL, LOW);

  Serial.println(index++);

  // all unreset
  digitalWrite(SHT_L0X_FL, HIGH);
  digitalWrite(SHT_L0X_FF, HIGH);
  digitalWrite(SHT_L0X_FR, HIGH);
  digitalWrite(SHT_L0X_RR, HIGH);
  digitalWrite(SHT_L0X_BR, HIGH);
  digitalWrite(SHT_L0X_BL, HIGH);
  digitalWrite(SHT_L0X_RL, HIGH);
  delay(10);

  Serial.println(index++);

  // activating L0X_L and reseting L0X_F
  digitalWrite(SHT_L0X_FL, HIGH);
  digitalWrite(SHT_L0X_FF, LOW);
  digitalWrite(SHT_L0X_FR, LOW);
  digitalWrite(SHT_L0X_RR, LOW);
  digitalWrite(SHT_L0X_BR, LOW);
  digitalWrite(SHT_L0X_BL, LOW);
  digitalWrite(SHT_L0X_RL, LOW);

  Serial.println(index++);

  delay(10);
  FL.setAddress(L0X_FL_ADDRESS);

  Serial.println(index++);

  digitalWrite(SHT_L0X_FF, HIGH);
  delay(10);
  FF.setAddress(L0X_FF_ADDRESS);

  Serial.println(index++);

  digitalWrite(SHT_L0X_FR, HIGH);
  delay(10);
  FR.setAddress(L0X_FR_ADDRESS);

  Serial.println(index++);

  digitalWrite(SHT_L0X_RR, HIGH);
  delay(10);
  RR.setAddress(L0X_RR_ADDRESS);

  Serial.println(index++);

  digitalWrite(SHT_L0X_BR, HIGH);
  delay(10);
  BR.setAddress(L0X_BR_ADDRESS);

  Serial.println(index++);

  digitalWrite(SHT_L0X_BL, HIGH);
  delay(10);
  BL.setAddress(L0X_BL_ADDRESS);

  Serial.println(index++);

  digitalWrite(SHT_L0X_RL, HIGH);
  delay(10);
  RL.setAddress(L0X_RL_ADDRESS);

  Serial.println(index++);

  delay(10);

  FL.init();
  FF.init();
  FR.init();
  RR.init();
  BR.init();
  BL.init();
  RL.init();

  Serial.println(index++);

  // Gắn cờ lỗi
  FL.setTimeout(500);
  FF.setTimeout(500);
  FR.setTimeout(500);
  RR.setTimeout(500);
  BR.setTimeout(500);
  BL.setTimeout(500);
  RL.setTimeout(500);

  Serial.println(index++);

  // // reduce timing budget to 20 ms (default is about 33 ms)
  // FL.setMeasurementTimingBudget(20);
  // FF.setMeasurementTimingBudget(20);
  // FR.setMeasurementTimingBudget(20);
  // RR.setMeasurementTimingBudget(20);
  // BR.setMeasurementTimingBudget(20);
  // BL.setMeasurementTimingBudget(20);
  // RL.setMeasurementTimingBudget(20);

  FL.startContinuous(10);
  FF.startContinuous(10);
  FR.startContinuous(10);
  RR.startContinuous(10);
  BR.startContinuous(10);
  BL.startContinuous(10);
  RL.startContinuous(10);

  Serial.println(index++);
}


int AML_LaserSensor_readSingle(int laserNumber)
{
    AML_LaserSensor_readAll(AML_laserSensorValue);
    return AML_laserSensorValue[laserNumber];
}

// Đọc laser (không ghi ra serial)
void AML_LaserSensor_readAll(int *userArrayAddress)
{
  int index = 0;

  // dùng ++index khù khoằm này để copy patse cho nhanh, đỡ phải gõ
  AML_laserSensorValue[++index] = FL.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = FL.readRangeContinuousMillimeters();

  AML_laserSensorValue[++index] = FF.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = FF.readRangeContinuousMillimeters();

  AML_laserSensorValue[++index] = FR.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = FR.readRangeContinuousMillimeters();

  AML_laserSensorValue[++index] = RR.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = RR.readRangeContinuousMillimeters();

  AML_laserSensorValue[++index] = BR.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = BR.readRangeContinuousMillimeters();

  AML_laserSensorValue[++index] = BL.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = BL.readRangeContinuousMillimeters();

  AML_laserSensorValue[++index] = RL.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = RL.readRangeContinuousMillimeters();
}


// Hàm này giống hệt readLaserSensor(), khác mỗi cái có ghi ra serial để xem giá trị
void AML_LaserSensor_readAllTest(int *userArrayAddress)
{
  int index = 0;

  AML_laserSensorValue[++index] = FL.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = FL.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.print("   ");

  AML_laserSensorValue[++index] = FF.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = FF.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.print("   ");

  AML_laserSensorValue[++index] = FR.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = FR.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.print("   ");

  AML_laserSensorValue[++index] = RR.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = RR.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.print("   ");

  AML_laserSensorValue[++index] = BR.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = BR.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.print("   ");

  AML_laserSensorValue[++index] = BL.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = BL.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.print("   ");

  AML_laserSensorValue[++index] = RL.readRangeContinuousMillimeters();
  *(userArrayAddress + index) = RL.readRangeContinuousMillimeters();
  Serial.print(AML_laserSensorValue[index]);

  Serial.println("   ");
}