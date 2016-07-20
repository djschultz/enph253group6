#include <Servo.h>
//THIS CODE IS FOR AN ARDUINO UNO, IT WILL OPEN THE CLAW

const int inputPin = 4;

Servo clawServo;

void setup() {
  // put your setup code here, to run once:
clawServo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(inputPin)){
    clawServo.write(60);
  } else{
    clawServo.write(117);
  }
}
