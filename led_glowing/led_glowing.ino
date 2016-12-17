char inchar;
int LED = 13;
void setup()
{
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
}
void loop()
{
  if(Serial.available()>0)
  {
    inchar=Serial.read();
    if(inchar=='n')
    {
      digitalWrite(LED,HIGH);
      Serial.println("LED is on");
    }
     if(inchar=='m') 
    {
      digitalWrite(LED,LOW);
      Serial.println("LED is off");
    }
  }
}
    
