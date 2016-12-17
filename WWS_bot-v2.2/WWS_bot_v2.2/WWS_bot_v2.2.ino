#include <Wire.h>
#include <SoftwareSerial.h>
#define M_PI 3.14
#define EN1 9
#define IN1 6
#define IN2 7
#define EN2 8
#define IN3 5
#define IN4 4
SoftwareSerial BT(10, 11);
float head_target;
float head_init;
float walk_dist = 0;
/****************************************************************************************/
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float heading; float headingsum = 0;
float headingDegrees;
float declinationangle;


/************************************************************************************************/
int j, x, y, z;
double angle;
#define HMC5883_WriteAddress 0x1E     //  i.e 0x3C >> 1
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand 0x00
#define HMC5883_DataOutputXMSBAddress  0x03

int regb = 0x01;
int regbdata = 0x40;
int outputData[6];
/**************************************************************************************************/

/***************************************************************************************************/
void setup_HMC()
{
  Wire.beginTransmission(HMC5883_WriteAddress);
  Wire.write(regb);
  Wire.write(regbdata);
  Wire.endTransmission();
  //  sensor_t sensor;
  //  mag.getSensor(&sensor);
  //  Serial.println("------------------------------------");
  //  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  //  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  //  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  //  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  //  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  //  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  //  Serial.println("------------------------------------");
  // Serial.println("");
 

}
/*****************************************************************************************************/
float HMC_data()
{
  Wire.beginTransmission(HMC5883_WriteAddress); //Initiate a transmission with HMC5883 (Write address).
  Wire.write(HMC5883_ModeRegisterAddress);       //Place the Mode Register Address in send-buffer.
  Wire.write(HMC5883_ContinuousModeCommand);     //Place the command for Continuous operation Mode in send-buffer.
  Wire.endTransmission();                       //Send the send-buffer to HMC5883 and end the I2C transmission.
  delay(10);

  Wire.beginTransmission(HMC5883_WriteAddress);  //Initiate a transmission with HMC5883 (Write address).
  Wire.requestFrom(HMC5883_WriteAddress, 6);     //Request 6 bytes of data from the address specified.
  delay(10);
  //Read the value of magnetic components X,Y and Z
  if (6 <= Wire.available()) // If the number of bytes available for reading be <=6.
  {
    for (j = 0; j < 6; j++)
    {
      outputData[j] = Wire.read(); //Store the data in outputData buffer
    }
  }

  x = outputData[0] << 8 | outputData[1]; //Combine MSB and LSB of X
  z = outputData[2] << 8 | outputData[3]; //Combine MSB and LSB of Z
  y = outputData[4] << 8 | outputData[5]; //Combine MSB and LSB of Y
  angle = atan2((double)y, (double)x) * (180 / 3.14159265) + 180; // angle in degrees

  Serial.print("| Deg:");
  Serial.println(angle, 2);
  return angle;
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
void parse_string()
{
  char a[40], l[10], t[10];
  int i, j, k;
  i = j = k = 0;

  while (1)
  {

    a[i++] = BT.read();
    while (!BT.available());
    //delayMicroseconds(250);
    if (a[i - 1] == '\n')
    {
      break;
    }
  }

  a[i] = '\0';
 // BT.flush();
  // Serial.print("A : ");
  // Serial.println(a);

  for (j = 0; a[j + 2] != 'T'; j++)
  {
    l[j] = a[j + 2];
  }
  l[j] = '\0';
  for (k = 0; a[j + 4] != '\0'; j++, k++)
  {
    t[k] = a[j + 4];
  }
  t[k] = '\0';
  walk_dist = atof(l);
  head_target = atof(t);
  Serial.print(" WALK_DIST =");
  //Serial.print(l);
  Serial.print(walk_dist);
  Serial.print(" HEAD_TARGET = ");
  //Serial.println(t);
  Serial.println(head_target);
}
void turn_zero_radius()
{
  while (fabs(head_target - head_init) >= 12)
  {
    head_init = HMC_data();
    if (head_target - head_init <= 0)   //right
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(EN1, 140);
      analogWrite(EN2, 140);
    }
    else if (head_target - head_init >= 0)  //left
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(EN1, 140);
      analogWrite(EN2, 140);
    }
    Serial.print(head_target);
    Serial.print("   |  ");
    Serial.println("Zero_radius");
  }
}
void turn_walk()
{
  while (fabs(head_target - head_init) >= 12)
  {
    head_init = HMC_data();
    if (head_target - head_init <= 0)   //right
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      analogWrite(EN1, 140);
      analogWrite(EN2, 140);
    }
    else if (head_target - head_init >= 0)  //left
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(EN1, 140);
      analogWrite(EN2, 140);
    }
  }
}
void go_forward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, 140);
  analogWrite(EN2, 140);
  Serial.println("Go_forward");
  //delay(25);

  //stall();
}
void stall()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, 175);
  analogWrite(EN2, 175);
  Serial.println("stall");
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void setup(void)
{
  Serial.begin(9600);
  BT.begin(9600);

  /* Initialise the sensor */

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  Serial.print("here");
//  if (!mag.begin())
//  {
//    /* There was a problem detecting the HMC5883 ... check your connections */
//    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
//   
//  }


  setup_HMC();
  /* Display some basic information on this sensor */
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop(void)
{

  while (BT.available() > 0)
  {
    head_init = HMC_data();

    parse_string();
  
    if (fabs(head_target - head_init) >= 12 && walk_dist == 0)
    {
      turn_zero_radius();
    }
//    else if (fabs(head_target - head_init) >= 2 && walk_dist == 1)
//    {
//      turn_walk();
//    }
    else if (fabs(head_target - head_init) <= 12 && walk_dist == 1)
    {
      go_forward();
    }
    else if (fabs(head_target - head_init) <= 12 && walk_dist == 0)
    {
      stall();
    }
    else
    {
    }
    //BT.flush();
    // delay(50);
  }
}
