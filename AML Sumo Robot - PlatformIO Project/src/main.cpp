/*
  SumoRobot UTC 2023
  Stated: 30/03/2023
  LastUpdate: 3AM 17/04/2023

  Authors: An, Trung, Bình
*/

#include <Arduino.h>
#include <math.h>


#include <AML_LaserSensor.h>
#include <AML_Keyboard.h>
#include <AML_MotorControl.h>
#include <AML_IRSensor.h>
#include <AML_MPUSensor.h>



#include <MPU6050_light.h>

#include <SPI.h>
#include <Wire.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define logMode_pin 53

int count = 0;

int sensorValue[8]; // Mảng giá trị cảm biến
int mark[8];        // Mảng đánh dấu số thứ tự của laser khi sắp xếp



void (*func)(int);  // con trỏ trỏ đến hàm tương ứng với nút đã bấm
void (*plan)(void); // con trỏ trỏ đến hàm chiến thuật tương ứng với nút đã bấm


void scanI2CAddress()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(500); // wait 5 seconds for next scan
}





void setupLCD()
{
  lcd.init();
  lcd.backlight();
}

// truyền xâu vào đây để ghi lên LCD
void printLCD(String s)
{
  lcd.setCursor(0, 0);
  lcd.print(s);
}




// đổi giá trị 2 biến cho nhau sử dụng con trỏ
void swap(int *pointer1, int *pointer2)
{
  int x;
  x = *pointer1;
  *pointer1 = *pointer2;
  *pointer2 = x;
}


// sắp xếp mảng giá trị đo từ 7 laser
void sort()
{
  // int array[8];
  boolean haveSwap = false; // khởi tạo biến kiểm tra sự đổi chỗ

  for (int i = 1; i < 8; i++)
  {
    mark[i] = i;
  }

  for (int i = 1; i < 7; i++) // sắp xếp sủi bọt
  {
    haveSwap = false;
    for (int j = 1; j < 8 - i; j++)
    {
      if (sensorValue[j] > sensorValue[j + 1])
      {
        swap(&sensorValue[j], &sensorValue[j + 1]);
        swap(&mark[j], &mark[j + 1]);
        haveSwap = true; // đánh dấu đã đổi trong lần lặp này
      }
    }

    if (!haveSwap) // đã đổi, thoát vòng lặp lần này (vòng lặp biến i)
    {
      break;
    }
  }

  // đoạn từ đây đến hết hàm dùng để test thuật toán sắp xếp. Dữ liệu serial in ra hàng 1 là giá trị sensor, hàng 2 là số thứ tự của sensor

  // for (int i = 1; i < 8; i++)
  // {
  //   Serial.print(sensorValue[i]);
  //   Serial.print("    ");
  // }

  // Serial.println();

  // for (int i = 1; i < 8; i++)
  // {
  //   Serial.print(mark[i]);
  //   Serial.print("    ");
  // }

  // Serial.println();
  // Serial.println();
  // delay(300);
}

int minSensorValue()
{
  // delay(100);
  int minValue = 20000;
  int upperBlock = 500; // Chặn trên khoảng cách đo được của laser
  int markedSensor = 0;

  AML_LaserSensor_readAll(sensorValue);

  for (uint8_t i = 1; i <= 7; i++)
  {
    if (sensorValue[i] < minValue)
    {
      minValue = sensorValue[i];
      markedSensor = i;
    }
  }

  if (minValue < upperBlock)
  {
    return markedSensor;
  }
  else
  {
    return 0;
  }
}

// trả về số thứ tự của sensor phát hiện gần nhất
int searchNearest()
{
  const int upperBlock = 600;

  AML_LaserSensor_readAll(sensorValue);
  sort();

  // check chặn trên
  if (sensorValue[1] < upperBlock) //<- thay đổi chặn trên thay vào số ở đây
  {
    return mark[1];
  }
  else
  {
    return 0;
  }
}

void search1_An(int target)
{
  int t = 5;

  if (target == 0) // Không tìm thấy
  {
    // PWM(255, -255); // Quay tại chỗ để tìm
    Serial.println("None");
    // printLCD("None        ");
    delay(t);
  }
  if (target == 2) // Laser FF
  {
    AML_MotorControl_PWM(255, 255); // Đâm bằng đầu xe
    Serial.println("FF");
    // printLCD("FF              ");
    delay(t);
  }
  if (target == 1) // Laser FL
  {
    AML_MotorControl_PWM(-50, 50); // Quay trái
    Serial.println("FL");
    // printLCD("FL                ");
    delay(t);
  }
  if (target == 3) // Laser FR
  {
    AML_MotorControl_PWM(50, -50); // Quay phải
    Serial.println("FR");
    // printLCD("FR                    ");
    delay(t);
  }
  if (target == 4) // Laser RR
  {
    AML_MotorControl_PWM(-50, 50); // Quay trai
    Serial.println("RR");
    // printLCD("RR                        ");
    delay(t);
  }
  if (target == 7) // Laser RL
  {
    AML_MotorControl_PWM(70, -70); // Quay phải
    Serial.println("RL");
    // printLCD("RL                        ");
    delay(t);
  }
  if (target == 5 || target == 6) // Laser BR or BL
  {
    AML_MotorControl_PWM(-255, -255); // Đâm bằng đuôi xe
    Serial.println("BR or BL");
    // printLCD("BR or BL            ");
    delay(t);
  }
}

void search2_An(int target)
{
  int t = 5;

  if (target == 0) // Không tìm thấy
  {
    AML_MotorControl_PWM(-50, 50);
    // PWM(255, -255); // Quay tại chỗ để tìm
    Serial.println("None");
    // printLCD("None        ");
    delay(t);
  }
  if (target == 2) // Laser FF
  {
    // PWM(255, 255); // Đâm bằng đầu xe
    Serial.println("FF");
    // printLCD("FF              ");
    if (sensorValue[target] < 100)
    {
      AML_MotorControl_PWM(255, 255);
    }
    else
    {
      AML_MotorControl_PWM(120, 120);
    }
    delay(t);
  }
  if (target == 1) // Laser FL
  {
    // PWM(-50, 50); // Quay trái
    Serial.println("FL");
    // printLCD("FL                ");
    if (sensorValue[target] < 30)
    {
      AML_MotorControl_PWM(-255, 255);
    }
    else
    {
      AML_MotorControl_PWM(-50, 50);
    }
    delay(t);
  }
  if (target == 3) // Laser FR
  {
    // PWM(50, -50); // Quay phải
    Serial.println("FR");
    // printLCD("FR                    ");
    if (sensorValue[target] < 30)
    {
      AML_MotorControl_PWM(255, -255);
    }
    else
    {
      AML_MotorControl_PWM(50, -50);
    }
    delay(t);
  }
  if (target == 4) // Laser RR
  {
    // PWM(-50, 50); // Quay trai
    Serial.println("RR");
    // printLCD("RR                        ");
    if (sensorValue[target] < 35)
    {
      AML_MotorControl_PWM(-255, 255);
    }
    else
    {
      AML_MotorControl_PWM(-60, 60);
    }
    delay(t);
  }
  if (target == 7) // Laser RL
  {
    // PWM(70, -70); // Quay phải
    Serial.println("RL");
    // printLCD("RL                        ");
    if (sensorValue[target] < 35)
    {
      AML_MotorControl_PWM(255, -255);
    }
    else
    {
      AML_MotorControl_PWM(60, -60);
    }
    delay(t);
  }
  if (target == 5 || target == 6) // Laser BR or BL
  {
    // PWM(-255, -255); // Đâm bằng đuôi xe
    Serial.println("BR or BL");
    // printLCD("BR or BL            ");
    if (sensorValue[target] < 100)
    {
      AML_MotorControl_PWM(-255, -255); // Đâm bằng đầu xe
    }
    else
    {
      AML_MotorControl_PWM(-120, -120);
    }
    delay(t);
  }
}

void search_Trung(uint8_t target)
{

  switch (target)
  {
  case 0:
    AML_MotorControl_PWM(-255, 255);
    printLCD("0");
    break;
    ///////////////
  case 1:
    AML_MotorControl_PWM(-200, 200);
    printLCD("1");
    break;
    ///////////////
  case 2:
    AML_MotorControl_PWM(255, 255);
    printLCD("2");
    break;
    ///////////////
  case 3:
    AML_MotorControl_PWM(200, -200);
    printLCD("3");
    break;
    ///////////////
  case 4:
    AML_MotorControl_PWM(255, -255);
    printLCD("4");
    break;
    ///////////////
  case 5:
    AML_MotorControl_PWM(255, -255);
    printLCD("5");
    break;
    ///////////////
  case 6:
    AML_MotorControl_PWM(-255, 255);
    printLCD("6");
    break;
    ///////////////
  case 7:
    AML_MotorControl_PWM(-255, 255);
    printLCD("7");
    break;
  }
}

// nút C
void plan1()
{
}

// nút D
void plan2()
{
  AML_MotorControl_PWM(80, 160);
  delay(500);
}

// nút E
void plan3()
{
  AML_MotorControl_PWM(160, 120);
  delay(500);
}

void plan4()
{

 AML_MotorControl_PWM(200, 100);
 delay(200);
 AML_MotorControl_PWM(80, 200);
 delay(300);
 AML_MotorControl_PWM(0,0);
}

////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  AML_MotorControl_setupL298();
  AML_MotorControl_PWM(0, 0);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  pinMode(logMode_pin, OUTPUT);

  // AML_MPUSensor_setup();

  // scanI2CAddress();
  AML_LaserSensor_setup();
  // scanI2CAddress();

  setupLCD();
  printLCD("Standby");

  switch (AML_Keyboard_readKeyLoop())
  {
  case 0:
    func = &search2_An;
    plan = &plan4;
    break;
  case 2: // nút C
    func = &search2_An;
    plan = &plan1;
    // printLCD("Select AN");
    break;
  case 3: // nút D
    func = &search2_An;
    plan = &plan2;
    break;
  case 4:
    func = &search2_An;
    plan = &plan3;
    break;
  }

  digitalWrite(13, 1);
  delay(2500); // Theo luật thi đấu, cơ mà giờ đang test nên không cần
  printLCD("              ");
  // AML_IRSensor_setup();

  digitalWrite(logMode_pin, HIGH);
  plan();
  digitalWrite(logMode_pin, LOW);
  
  AML_MotorControl_PWM(0, 0);
}

void loop()
{
  AML_IRSensor_standby();

  func(minSensorValue()); // Gọi hàm được trỏ

  AML_Keyboard_readResetKey(); // Quét nút
}