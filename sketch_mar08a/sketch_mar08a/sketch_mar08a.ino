#include <Wire.h>

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  delay(5000);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMuch) {
  //Serial.println(Wire.available());
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}




/*#include <Wire.h>
int i=0;
int j=0;
char data[10];

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(requestEvent);// register event
  Serial.begin(9600);  // start serial for output
}

void loop() {
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent(int bytes) {
  bytes=10;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read();// receive a byte as character
    data[i++]=c;
    if(i==6){
      i=0;
      for(j=0;j<6;j++)
      {
         Serial.print(data[j]);
      }// print the character
      Serial.println();
    }
  }
}*/
