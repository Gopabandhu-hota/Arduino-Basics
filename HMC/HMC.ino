#include <Wire.h> //I2C Arduino Library

#define addr 0x1E //I2C Address for The HMC5883
float ang_HMC =0;
float old_x = 0;
float old_z = 0;
float change_x=0;
float change_z=0;
int count=0;
void setup(){
  
  Serial.begin(9600);
  Wire.begin();
  
  
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
}


void loop(){
  
  int x,y,z; //triple axis data
  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  
 
 //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //MSB  x 
    x |= Wire.read(); //LSB  x
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
    /*angle = atan2(y, x) * (180 / 3.14159265) ; // angle in degrees

  Serial.print("| Deg:");
  Serial.println(angle, 2);
  }*/
  }
  // Show Values
  Serial.print("X Value: ");
  Serial.print(x);
  Serial.print("Y Value: ");
  Serial.print(y);
  Serial.print("Z Value: ");
  Serial.print(z);
  if(count < 1)
  {
    old_x = x;
  }
  else
  {
  change_x = x - old_x;
  change_z =z - old_z;
  Serial.println();
  ang_HMC = atan(change_z/change_x) * (180 / 3.14159265) ; // angle in degrees
  Serial.print("| Deg_HMC:");
  Serial.println(ang_HMC, 2);
  Serial.println();
  }
  count ++;
  delay(500);
}


