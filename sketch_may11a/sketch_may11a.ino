#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#define HMC5883_WriteAddress 0x1E     //  i.e 0x3C >> 1
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand 0x00
#define HMC5883_DataOutputXMSBAddress  0x03

int regb = 0x01;
int regbdata = 0x40;
float heading=0;
void setup_HMC()
{
  //    Wire.beginTransmission(HMC5883_WriteAddress);
  //    Wire.write(regb);
  //    Wire.write(regbdata);
  //    Wire.endTransmission();
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");



}
float HMC_ldata()
{
  sensors_event_t event;
  mag.getEvent(&event);
  heading = atan2(event.magnetic.y, event.magnetic.x);
  heading = heading * 180 / M_PI + 180;
  Serial.print("Bot_deg"); Serial.println(heading);
  return heading;
}
void setup() {
  // put your setup code here, to run once:
   Wire.begin();
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
 setup_HMC();
}

void loop() {
  // put your main code here, to run repeatedly:
HMC_ldata();
delay(333);
}
