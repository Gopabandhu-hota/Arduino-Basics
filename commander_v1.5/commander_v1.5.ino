
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);
#include <Wire.h>
/*******************************************************/
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float heading; float headingsum = 0;
float headingDegrees;
float declinationangle;

/**************************************************/
int a; int b;
float foot_length = 90;
float acc_z;
float acc_y;
/**************************************************************************************************/
#define PI 3.143
const int G_SENS = 131; //a division factor that directly gives omega calculated
const int MPU = 0x68;                //address of the IMU sensor
const float dt = 0.001;            //derivative time interval
float y_old_theta = 0, y_theta = 0;
float y_sum_theta = 0; float y_diff;
float theta_old;
float theta_older;
float step_theta_1;
float step_theta_2;
int avgy;
int avgz;
int theta_1_flag = 0;
int theta_2_flag = 0;
float step_length = 0 , new_step_length = 0 , prev_step_length = 0;
float distance = 0;
float base;
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

/**********************************************************************************************/
float theta_calc()
{
  acc_z = analogRead(A0);
  acc_y = analogRead(A1);
  int normal_z=acc_z - 333;
  int normal_y=acc_y - 333;
 // Serial.print("acc_z="); Serial.print(normal_z);
 // Serial.print("acc_y="); Serial.println(normal_y);
//  if(normal_y<=2 && normal_y>=0 && normal_z>0)
//    y_theta=90;
//  else if(normal_y<=2 && normal_y>=0 && normal_z<0)
//    y_theta=-90;
//  else if(normal_y>=-2 && normal_y<=0 && normal_z>0)
//    y_theta=90;
//  else if(normal_y<=2 && normal_y>=0 && normal_z>0)
//    y_theta=-90;
//  else {
    y_theta = atan2(normal_z, normal_y);
    y_theta = y_theta * 180 / M_PI;
//  }
  Serial.print("deg :"); Serial.print(y_theta);
  return y_theta;
}
/*****************************************************************************************************/
void avgvalue()
{
  a = 0;
  b = 0;
  for (int i = 0; i < 10; i++)
  {
    a += analogRead(A1);
    b += analogRead(A0);
  }
  avgy = a / 10;
  avgz = b / 10;
}
/***************************************************************************************************/
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

  Serial.print("|| Deg:");
  Serial.print(angle, 2);
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
void setup(void)
{
  //avgvalue();
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Serial.begin(38400);                 //mistake probability...set baud rate properly
  BT.begin(38400);

  Wire.begin();
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  setup_HMC();
  Serial.print("here");
}

void loop(void)
{
   delay(10);
   theta_calc();
   move_status();
   Serial.print("Step=");Serial.print(step_length);
   angle = HMC_ldata();
  //  monitor_theta(y_theta);
   
   data_transmitter();
  //    Serial.print("||Step=");
  //   Serial.println(step_length);

}
/****************************************************************************************************/

void data_transmitter()
{
  BT.print("l="); BT.print(step_length);
  BT.print("T="); BT.println(angle);
  delay(100);
}
/**************************************************************************************************/
int move_status()
{
  if ((y_theta >= -15) && (y_theta <= 15))
  {
    step_length=0;
  }
  else 
  {
    step_length=1;
  }
}
/*************************************************************************************************/
float step_counter(float theta_1, float theta_2)
{
  float delta_theta;
  delta_theta = theta_2 - theta_1;

  if ( abs(delta_theta) < 8)
  {

    return step_length = 0;
  }
  else
  {
    
    //prev_step_length = new_step_length;
   // step_length = 4 * foot_length * sin(abs(delta_theta) * PI / 180); //check formula
    /*if (prev_step_length == new_step_length)
    {
      step_length=0;
    }
    else
    {
      step_length = new_step_length ;
    }*/
    return step_length=1 ;
  }
}
void monitor_theta(float theta)
{
  if (theta <= theta_old && theta_old >= theta_older)
  {
    step_theta_2 = theta_old;
    theta_2_flag = 1;
  }
  else if ( theta >= theta_old && theta_old <= theta_older)
  {
    step_theta_1 = theta_old;
    theta_1_flag = 1;
  }
  else
  {
    theta_1_flag = theta_2_flag = 0;
  }
  theta_older = theta_old;
  theta_old = theta;
  if (theta_2_flag == 1)
  {
    step_counter(step_theta_1, step_theta_2);
  }
}


