#include <Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
#define HMC_addr 0x1E //I2C Address for The HMC5883
#define PI 3.14159265
#define threshold 2
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 int x,y,z; //triple axis data
 #define G_SENS 131
 #define target_angle -30
#define EN1 A0
#define IN1 6
#define IN2 7
#define EN2 A1
#define IN3 5
#define IN4 4
#define Kp 1
#define Kd 0
#define Ki 0
float error=0;
float errSum=0;
float dErr=0;
float lastErr=0;
float pid_error =0;
int PWM =0;
 float gyro_reading=0;
 float ang_MPU=0;
 float initial_MPU=0;
 float ang_HMC=0;
 float initial_HMC=0;
 float new_time=0;
 float prev_time=0;
 int count=0;
 float time_change=0;
 float kalman_angle=0;
 // KasBot V1 - Kalman filter module

float Q_angle  =  0.01; //0.001
float Q_gyro   =  0.0003;  //0.003
float R_angle  =  0.01;  //0.03
float x_angle =0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float  Y, S;
float K_0, K_1;
 void HMC_setup();
 void MPU_setup();
 float HMC_data();
 float MPU_data();
 // newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float kalmanCalculate(float newAngle, float newRate,int looptime)
{
float dt = float(looptime)/1000;
x_angle += dt * (newRate - x_bias);
P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
P_01 +=  - dt * P_11;
P_10 +=  - dt * P_11;
P_11 +=  + Q_gyro * dt;

Y = newAngle - x_angle;
S = P_00 + R_angle;
K_0 = P_00 / S;
K_1 = P_10 / S;

x_angle +=  K_0 * Y;
x_bias  +=  K_1 * Y;
P_00 -= K_0 * P_00;
P_01 -= K_0 * P_01;
P_10 -= K_1 * P_00;
P_11 -= K_1 * P_01;

return x_angle;
}
 void HMC_setup()
 {
  Wire.beginTransmission(HMC_addr);
  Wire.write(0x02);//Set the register
   Wire.write(0x00);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission();
 }
 void MPU_setup()
 {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);
  Wire.endTransmission();
 }
 float HMC_data()
 {
   Wire.beginTransmission(HMC_addr);
  
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission(false);
  Wire.requestFrom(HMC_addr, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //MSB  x 
    x |= Wire.read(); //LSB  x
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
  }
   Wire.endTransmission();
     // Show Values
  /*Serial.print("X Value: ");
  Serial.print(x);
  Serial.print("Y Value: ");
  Serial.print(y);
  Serial.print("Z Value: ");
  Serial.print(z);*/
  if(count<10)
  {
  ang_HMC = atan2((double)z, (double)x )* (180 / PI) ; // angle in degrees
  initial_HMC = ang_HMC;
  /*if( ang_HMC < -180)
  {
    ang_HMC = 360+ang_HMC;
  }*/
  Serial.print("| Deg_HMC:");
  Serial.println(ang_HMC, 2);
  Serial.println();
  return (ang_HMC);
   }
   else
  {
  ang_HMC = atan2((double)z, (double)x )* (180 / PI) ; // angle in degrees
  ang_HMC = ang_HMC - initial_HMC;
  /*if(ang_HMC<(-180))
  {
    ang_HMC =360 + ang_HMC;
  }*/
 /* Serial.print("| Deg_HMC:");
  Serial.println(ang_HMC, 2);
  Serial.println();*/
  return (ang_HMC);
  }
  }
 float MPU_data()
  {
     Wire.beginTransmission(MPU_addr);
  
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 Wire.endTransmission();
 /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);*/
  new_time=micros();
  gyro_reading =( (double)GyX / G_SENS );
  Serial.print("Gyro-reading:");
  Serial.println(gyro_reading);
  if(abs(gyro_reading) > threshold)
  {
  time_change = (new_time-prev_time)/1000000;//time change in seconds
  ang_MPU += gyro_reading * time_change ; // angle in degrees
  prev_time=new_time;
 // ang_MPU=ang_MPU-initial_MPU;
  Serial.print("| Deg_MPU:");
   Serial.println(ang_MPU);
 // Serial.println(ang_MPU, 2);
  }
  else
  {
  time_change = (new_time-prev_time)/1000000;//time change in seconds
  prev_time=new_time;
  /*Serial.print("| Deg_MPU:");
   Serial.println(ang_MPU);*/
 // Serial.println(ang_MPU, 2);
  }
  return (gyro_reading);
  }
void turn_zero_radius()
{
  
    if (pid_error< (-2))   //right
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      if(abs(pid_error) > 2*target_angle)
      {
        PWM = 2*target_angle;
      }
      PWM = abs(pid_error);
      PWM = (double)PWM * 255/(2*target_angle);
      //PWM = map(PWM,0,2*target_angle,0,255);
      Serial.println("turning right with PWM=");
      Serial.print(PWM);
      analogWrite(EN1,PWM );
      analogWrite(EN2,PWM );
    }
    else if (pid_error > 2)  //left
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
     if(abs(pid_error) > 2*target_angle)
      {
        PWM = 2*target_angle;
      }
      PWM = abs(pid_error);
      PWM = (double)PWM * 255/(2*target_angle);
      //PWM = map(PWM,0,2*target_angle,0,255);
      Serial.println("turning left with PWM=");
      Serial.print(PWM);
      analogWrite(EN1, PWM);
      analogWrite(EN2, PWM);
    }
    else
    {
      Serial.println("break");
      digitalWrite(EN1, 0);
      digitalWrite(EN2, 0);
    }
}
void pid()
{
   error = target_angle - kalman_angle;
  // Serial.print("error:");
   //Serial.println(error);
    errSum += (error )*time_change;
    dErr = (error - lastErr)/time_change;
   pid_error = Kp * error + Ki * errSum + Kd * dErr;
   /*Remember some variables for next time*/
   lastErr = error; 
   turn_zero_radius();
} 
void setup(){
  Wire.begin(); //start talking
  MPU_setup();
  HMC_setup();
 
  Serial.begin(9600);
}
void loop(){
kalman_angle=kalmanCalculate( HMC_data(), MPU_data(), 20);
Serial.println("kalman angle:");
Serial.println(kalman_angle);
pid();
count++;
 // delay(1000);
}
