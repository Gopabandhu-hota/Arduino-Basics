/*#include <Wire.h>
void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  
}

void loop() {
  Wire.beginTransmission(8);    // request 6 bytes from slave device #8
  Wire.write("hello ");
  Wire.endTransmission();
  delay(500);
}*/


#include <Wire.h>
void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
}

byte x = 0;

void loop() {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write("x is ");        // sends five bytes
  Wire.write(x);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  x++;
  //delay(500);
}
