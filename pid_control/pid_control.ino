volatile double c = 0;
volatile double pos = 0;
volatile double encoder;
volatile double lastencoder=0;
unsigned long lastTime=0;
 unsigned long now;
double  Output;
double value=0;
double rpm,count;
double errSum, lastErr;
double kp, ki, kd;
//pid hasnt been tuned exactly
#define kp 2
#define ki 0.01
#define kd 0
#define A 2
#define B 3
#define  Setpoint 350

#define  vmax 457
int motor_left = 5;
int motor_right=6;
int EN1=11;
void setup(){
pinMode(A, INPUT);
//digitalWrite(A,HIGH);
pinMode(B, INPUT);
//digitalWrite(B,HIGH);
  pinMode(motor_left, OUTPUT);
  pinMode(motor_right, OUTPUT);
  pinMode(EN1,OUTPUT);; 
   digitalWrite(motor_right,LOW);
  digitalWrite(motor_left,HIGH);
attachInterrupt(digitalPinToInterrupt(A),WORKING,CHANGE);
Serial.begin(9600);
}
void loop(){
 /*How long since we last calculated*/
  now = millis();
 // Serial.print("time:");
//Serial.println(now);
 // delay(1000);
  if(now-lastTime >200)
  {
  pid();
  lastencoder=encoder;
  value=rpm+Output;
  //Serial.print("value;");
  //Serial.println(value);
  if(value>vmax)
  {
    value=vmax;
  }
  value=map(value,0,vmax,0,255);
 // Serial.print("value;");
 // Serial.println(value);
  analogWrite(EN1, value); 
  }
//Serial.print("encoder:");
//Serial.println(encoder);
}
void WORKING(){
    
    if (digitalRead(A) == digitalRead(B) ) {
         encoder += 1;
    }
    else encoder -= 1;
    /*if(encoder>64 || encoder<-64)
    {
      encoder=0;
    }*/
}
void pid()
{
   
   double timeChange = (double)(now - lastTime);
   count=(encoder-lastencoder)*1000/timeChange;
   rpm=double(count*60/(19*32));//gear ratio 19 nd interrupt on both rising & falling edge
   Serial.print("rpm:");
   Serial.println(abs(rpm));
   double error = Setpoint -abs( rpm);
  // Serial.print("error:");
   //Serial.println(error);
   errSum += (error );//not include time change as its discrete
   double dErr = (error - lastErr);
   Output = kp * error + ki * errSum + kd * dErr;
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now; 
} 

