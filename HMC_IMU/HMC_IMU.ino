#include <Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
#define HMC_addr 0x1E //I2C Address for The HMC5883
#define PI 3.14159265
#define threshold 2
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 int x,y,z; //triple axis data
 #define G_SENS 131
 float gyro_reading=0;
 float ang_MPU=0;
 float initial_MPU=0;
 float ang_HMC=0;
 float initial_HMC=0;
 float new_time=0;
 float prev_time=0;
 int count=0;
 float time_change=0;
 void HMC_setup();
 void MPU_setup();
 void HMC_data();
 void MPU_data();
 
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
 void HMC_data()
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
   }
   else
  {
  ang_HMC = atan2((double)z, (double)x )* (180 / PI) ; // angle in degrees
  ang_HMC = ang_HMC - initial_HMC;
  if(ang_HMC >( 180))
  {
    ang_HMC =360 - ang_HMC;
  }
  if(ang_HMC < ( -180))
  {
    ang_HMC =360 + ang_HMC;
  }
  Serial.print("| Deg_HMC:");
  Serial.println(ang_HMC, 2);
  Serial.println();
  }
  }
 void MPU_data()
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
  Serial.print("| Deg_MPU:");
   Serial.println(ang_MPU);
 // Serial.println(ang_MPU, 2);
  
  }
  }

void setup(){
  Wire.begin(); //start talking
  MPU_setup();
  HMC_setup();
 
  Serial.begin(9600);
}
void loop(){
MPU_data();
HMC_data();
count++;
 // delay(1000);
}
