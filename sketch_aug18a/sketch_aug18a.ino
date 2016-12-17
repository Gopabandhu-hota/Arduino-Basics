void setup() {
  pinMode(DAC0,OUTPUT);
  analogReadResolution(12);
  // put your setup code here, to run once:

}

void loop() {
  analogWrite(DAC0,1400);
  // put your main code here, to run repeatedly:

}
