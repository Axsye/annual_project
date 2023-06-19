#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <MPU6500_WE.h>
#include <PID_v1.h>
#include <math.h>

// hmcl5883设置
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// mpu设置
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

//
#define DESTINATION 0 // 外部设定角度(设定参数位置)
#define SWITCH_PIN 4  // 开关所接引脚
#define RELAY_PIN 6   // 继电器
#define PIN_INPUT 0
#define PIN_OUTPUT 3 // mos
//
#define MPU_ADDRESS 0x68
#define DECLINATION -11.32 // 地磁偏量
#define PI_DEGREE 180
#define MPUsetSampleRateDivider 40 // 采样率控制值，Sample rate = Internal sample rate / (1 + divider)

float input, target; // 用于计算角度差绝对值
float setpoint;      // 当前角度与目标插值
float output;
float raw_input;              // 原始数据输入
float kp = 1, ki = 0, kd = 10; // pid参数(0-1)
float opposite;
float Imu_Yaw, unsigned_Imu_Yaw;
static unsigned long prev_ms;
static unsigned long delta_time;

float Compass_Heading(int processed_output);
bool if_reverse(float);
int Output_Process(float raw_output);
float myPID(float rt, float yt, float kp, float ki, float kd, float range_positive);

void setup()
{
  Serial.begin(115200); // 设置串口

  pinMode(SWITCH_PIN, INPUT); // 初始化引脚
  pinMode(RELAY_PIN, OUTPUT);

  Wire.begin();

  // 5883初始化
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

  // mpu初始化
  if (!myMPU6500.init())
  {
    Serial.println("MPU6500 does not respond");
  }
  else
  {
    Serial.println("MPU6500 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  // digital low pass filter (DLPF)
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(MPUsetSampleRateDivider + 1); // Sample rate = Internal sample rate / (1 + divider) ********+1补偿了漂
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  // Range setting
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  delay(1000);
}

void loop()
{

  // mpu数据获取
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);
  // 对角加速度积分,25ms，得到imu角度
  delta_time = millis() - prev_ms; // 获取delta_time(ms)
  prev_ms = millis();
  Imu_Yaw += -gyr.z * ((float)delta_time / 1000); // 积分（gyr.z单位是°/s）
  // Imu_Yaw += -gyr.z / MPUsetSampleRateDivider *1.5;
  // 限制范围,得到正数角度
  unsigned_Imu_Yaw = Imu_Yaw;
  while (unsigned_Imu_Yaw < 0)
  {
    unsigned_Imu_Yaw += 360;
  }
  while (unsigned_Imu_Yaw > 360)
  {
    unsigned_Imu_Yaw -= 360;
  }
  /*****************************************************************/
  // 信息获取部分结束

  // 计算偏差，调用pid
  raw_input = Imu_Yaw;
  float myPID_out = myPID(DESTINATION, raw_input, kp, ki, kd, 180);
  int processed_output = Output_Process(myPID_out);

  /*串口输出调试信息*/
  /*
  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  */
  // Serial.print("Compass_Heading (degrees): "); Serial.println(Compass_Heading());
  Serial.print("Imu_Heading (degrees): ");
  Serial.print(Imu_Yaw);
  Serial.print(" unsigned: ");
  Serial.println(unsigned_Imu_Yaw);

  Serial.print("PID rawoutput is: ");
  Serial.print(myPID_out);
  Serial.print(" Mos output is: ");
  Serial.println(processed_output);

  Serial.print("delta_time is: ");
  Serial.println(delta_time);
  Serial.println("********************************************");

  // 执行输出
  digitalWrite(RELAY_PIN, if_reverse(processed_output));
  // analogWrite(PIN_OUTPUT,output);
  analogWrite(PIN_OUTPUT, abs(processed_output));

  delay(1000 / MPUsetSampleRateDivider);
}

int Output_Process(float raw_output)
{
  int output = 0;
  if (raw_output < 0)
  {
    output = -100;
  }
  else
  {
    output = 100;
  }

  int pid_add = (int)(raw_output * 155);
  output += pid_add;

  return output;
}

float Compass_Heading()
{
  // 电子指南针设置
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x); // 计算方位角
  float declinationAngle = DECLINATION / 180;
  // heading += declinationAngle;  //修正地磁偏角
  // 限定角度输出范围在0-360
  //  Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}

bool if_reverse(int processed_output)
{
  /*
  if (raw_input < PI_DEGREE)
  {
    opposite = raw_input + PI_DEGREE;
    if (target > raw_input && target < opposite)
      return true;
    return false;
  }
  opposite = raw_input - PI_DEGREE;
  if (target > opposite && target < input)
    return false;
  return true;
  */
  if (processed_output >= 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

float myPID(float rt, float yt, float kp, float ki, float kd, float range_positive)
{
  // rt et输入 range为输入的范围（只要正范围）
  // kp ki kd输入必须归一化
  // 返回ut 为归一化数
  float et = rt - yt;
  float ut;
  static float intergration;
  static float differential;
  static float last_et;

  intergration += -et * ((float)delta_time / 1000); // 对et积分(使用loop里计算的时间增量)

  // differential = -gyr.z ; //等于et的导数（外部直接输入）

  // 不依赖额外输入的微分运算方法
  differential = et - last_et;
  last_et = et;

  ut = ((kp * et) + (ki * intergration) + (kd * differential)); // 得到系数归一化输出ut
  ut = ut / range_positive;                                     // 归一化ut(范围在(-1,1))

  return ut;
}