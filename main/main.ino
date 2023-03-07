#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "PID_v1.h"
#include "math.h"

#define SWITCH_PIN 4  //开关所接引脚
#define RELAY_PIN 6
#define PIN_INPUT 0
#define PIN_OUTPUT 3
#define MPU_ADDRESS 0x68
#define DECLINATION -11.32
#define DESTINATION 300
#define PI_DEGREE 180

double input, setpoint, output;
double raw_input, target;
double kp = 2, ki = 5, kd = 1;
double opposite;

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

MPU9250 mpu;

//定义顺时针为正转
bool if_reverse(){
  if(raw_input < PI_DEGREE){
    opposite = raw_input + PI_DEGREE;
    if(target > raw_input && target < opposite)
      return true;
    return false;
  }
  opposite = raw_input - PI_DEGREE;
  if (target > opposite && target < input)
    return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); //初始化陀螺仪
  mpu.setup(MPU_ADDRESS);
  //mpu.setMagneticDeclination(DECLINATION);
  pinMode(SWITCH_PIN, INPUT);  //初始化开关
  pinMode(RELAY_PIN, OUTPUT);
  //while(!digitalRead(SWITCH_PIN)){} //摁下开关后程序运行
  //Serial.println("陀螺仪将在5秒内校准\n请将其放置于水平面");
  mpu.verbose(true);
  //delay(5000);
  mpu.calibrateAccelGyro();
  //Serial.println("指南针将在5秒内校准\n请在桌面上画\"8\"字形");
  //delay(5000);
  //mpu.calibrateMag();
  mpu.verbose(false);
  target = DESTINATION;
  pid.SetMode(AUTOMATIC);
  raw_input = mpu.getYaw();
  setpoint = abs(target - raw_input);
}

void loop() {
  input = 0;
  pid.Compute();
  digitalWrite(RELAY_PIN,if_reverse());
  analogWrite(PIN_OUTPUT,output);
  if(mpu.update()){
    static uint32_t prev_ms = millis();
    if(millis() > prev_ms + 25){
      Serial.print(mpu.getYaw(), 2);
      prev_ms = millis();
      raw_input = mpu.getYaw();
    }
  }
  setpoint = abs(target - raw_input);
}
