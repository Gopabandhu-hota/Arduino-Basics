char inChar; 
int motor_delay = 100;

int LED = 13;
// ---------------------- Motors
int motor_left[] = {2, 3};
int motor_right[] = {6, 7};

// ---------------------- Setup
void setup() {
Serial.begin(9600);
int i;
for(i = 0; i < 2; i++){
  pinMode(motor_left[i], OUTPUT);
  pinMode(motor_right[i], OUTPUT);
  pinMode(LED,OUTPUT);
}

}

// ---------------------- Loop
void loop() {
   if (Serial.available() > 0) {
     inChar = Serial.read();      // get incoming byte:
    switch (inChar) {
      
      case 'w': 
        drive_forward();
        break;
      
      case 's': 
        drive_backward();
        break;
        
      case 'a': 
        turn_left();
        break;
      
      case 'd': 
        turn_right();
        break;
        
       case 'q': 
        full_left();;
        break;
        
       case 'b': 
        motor_stop();
        break;
        
       case 'e': 
        full_right();
        break;
        
       case 'l': 
        digitalWrite(LED,HIGH);
        delay(2000);
        digitalWrite(LED,LOW);
        break;
    
    }
  }
    
    
}

// ---------------------- Drive

void motor_stop(){
  digitalWrite(motor_left[0], LOW); 
  digitalWrite(motor_left[1], LOW); 
  Serial.println("Stop");
  digitalWrite(motor_right[0], LOW); 
  digitalWrite(motor_right[1], LOW);
  delay(25);
}

void drive_forward(){
  digitalWrite(motor_left[0], HIGH); 
  digitalWrite(motor_left[1], LOW); 
  Serial.println("Forward");
  digitalWrite(motor_right[0], HIGH); 
  digitalWrite(motor_right[1], LOW); 
}

void drive_backward(){
  digitalWrite(motor_left[0], LOW); 
  digitalWrite(motor_left[1], HIGH); 
  Serial.println("Backward");
  digitalWrite(motor_right[0], LOW); 
  digitalWrite(motor_right[1], HIGH); 
}

void turn_left(){
  digitalWrite(motor_left[0], LOW); 
  digitalWrite(motor_left[1], LOW); 
  Serial.println("Left");
  digitalWrite(motor_right[0], HIGH); 
  digitalWrite(motor_right[1], LOW);
}

void turn_right(){
  digitalWrite(motor_left[0], HIGH); 
  digitalWrite(motor_left[1], LOW); 
  Serial.println("Right");
  digitalWrite(motor_right[0], LOW); 
  digitalWrite(motor_right[1], LOW); 
}

void full_left(){
  digitalWrite(motor_left[0], LOW); 
  digitalWrite(motor_left[1], HIGH); 
  Serial.println("Full Left");
  digitalWrite(motor_right[0], HIGH); 
  digitalWrite(motor_right[1], LOW);
}

void full_right(){
  digitalWrite(motor_left[0], HIGH); 
  digitalWrite(motor_left[1], LOW); 
  Serial.println("Full Right");
  digitalWrite(motor_right[0], LOW); 
  digitalWrite(motor_right[1], HIGH); 
}
