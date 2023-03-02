#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define switch_pin = 4  //开关所接引脚

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz; //六轴数值，a为加速度计，g为角速度

void setup() {

  Wire.begin(); //初始化陀螺仪
  Serial.begin(38400);
  accelgyro.initialize();

  pinMode(switch_pin, INPUT)  //初始化开关
  while(!digitalRead(switch_pin)) //摁下开关后程序运行
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz)  //获取当前姿态
}
