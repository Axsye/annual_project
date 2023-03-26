#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <MPU6500_WE.h>
#include <PID_v1.h>
#include <math.h>

//hmcl5883设置
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//mpu设置
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);




//
#define DESTINATION 0     //外部设定角度(设定参数位置)
#define SWITCH_PIN 4  //开关所接引脚
#define RELAY_PIN 6 //继电器
#define PIN_INPUT 0 
#define PIN_OUTPUT 3  //mos
//
#define MPU_ADDRESS 0x68
#define DECLINATION -11.32  //地磁偏量
#define PI_DEGREE 180
#define MPUsetSampleRateDivider 40  //采样率控制值，Sample rate = Internal sample rate / (1 + divider)


double input, setpoint, output;
double raw_input, target;
double kp = 2, ki = 5, kd = 1;
double opposite;
float Imu_Yaw;

//PID设置
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

float Compass_Heading();
bool if_reverse();


void setup() {
  Serial.begin(115200); //设置串口

  pinMode(SWITCH_PIN, INPUT);  //初始化引脚
  pinMode(RELAY_PIN, OUTPUT);

  Wire.begin(); 

  //PID初始化
  pid.SetMode(AUTOMATIC);

  //5883初始化
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  //mpu初始化
   if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  //digital low pass filter (DLPF)
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(MPUsetSampleRateDivider);  //Sample rate = Internal sample rate / (1 + divider) **********************
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  //Range setting
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  delay(1000);
}

void loop() {
  
  //mpu数据获取
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);
  //对角加速度积分,25ms
  static uint32_t prev_ms = millis();
  Imu_Yaw += -gyr.z / MPUsetSampleRateDivider *1.5;
  //限制范围
  if(Imu_Yaw < 0)
    Imu_Yaw += 360;
  if(Imu_Yaw > 360)
    Imu_Yaw -= 360;
/*****************************************************************/
//信息获取部分结束

  raw_input = Imu_Yaw;
  pid.Compute();

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Compass_Heading (degrees): "); Serial.println(Compass_Heading());
  Serial.print("Imu_Heading (degrees): "); Serial.println(Imu_Yaw);
  

  Serial.println("********************************************");

  //执行输出
  digitalWrite(RELAY_PIN,if_reverse());
  analogWrite(PIN_OUTPUT,output);

  delay(1000 / MPUsetSampleRateDivider);
}



float Compass_Heading(){
  //电子指南针设置
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event); 
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);  //计算方位角
  float declinationAngle = DECLINATION / 180;
  //heading += declinationAngle;  //修正地磁偏角
  //限定角度输出范围在0-360
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}

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
