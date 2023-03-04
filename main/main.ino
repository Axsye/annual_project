#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

#define SWITCH_PIN 4  //开关所接引脚
#define MPU_ADDRESS 0x68
#define DECLINATION -11.32

MPU9250 mpu;

void setup() {
  Serial.begin(115200)
  Wire.begin(); //初始化陀螺仪
  mpu.setup(MPU_ADDRESS);
  mpu.setMagneticDeclination(DECLINATION);
  pinMode(SWITCH_PIN, INPUT)  //初始化开关
  while(!digitalRead(SWITCH_PIN)){} //摁下开关后程序运行
  Serial.println("陀螺仪将在5秒内校准\n请将其放置于水平面");
  mpu.verbose(ture);
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.Println("指南针将在5秒内校准\n请在桌面上画\"8\"字形");
  delay(5000);
  mpu.calibrateMag();
  mpu.verbose(false);
}

void loop() {
  if(mpu.update()){
    static uint32_t prev_ms millis();
    if(millis() > prev_ms + 25){
      Serial.print(mpu.getYaw(), 2)
      prev_ms = millis();
    }
  }
}
