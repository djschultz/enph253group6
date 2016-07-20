#include <phys253.h>
#include <LiquidCrystal.h>


//Start the Phi motor turned all the way CW when the phi arm is vertical
//Start the Theta motor turned all the way CCW when the arm tip hits the plastic base
//and the Phi motor is all the way CW.
//Theta Servo is connected to RCServo0
//Phi Servo is connected to RCServo1
//The values of servoTheta and servoPhi must always be between
//0 and 180 degrees.
double theta;
double phi;
int count = 0;
void setup() {
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {

  double servoTheta = 180.0*knob(6)/1024.0;
  double servoPhi = 180.0*knob(7)/1024.0;
  
  if(count%5000 == 0){
  Serial.println("Servo Theta");
  Serial.println(servoTheta);
  Serial.println("Servo Phi");
  Serial.println(servoPhi);
  double jointTheta = servoTheta/2.0 + 120.0;
  double jointPhi = -servoPhi/2.0 + 90;
  Serial.println("Joint Theta");
  Serial.println(jointTheta);
  Serial.println("Joint Phi");
  Serial.println(jointPhi);
  Serial.println("Knob 6");
  Serial.println(knob(6));
  Serial.println("Knob 7");
  Serial.println(knob(7));
  }

  
  RCServo2.write(servoTheta);
  RCServo1.write(servoPhi);
  //RCServo2.write(90);
  count++; 
}
int jointToServoTheta(int angle) {
  int result = 2 * (angle - 120);
  return result;
}

int jointToServoPhi(int angle) {
  int result = 2 * (90 - angle);
  return result;
}

void moveServoTheta(int initAngle, int finalAngle, int increment, int servoDelay) {
  if (initAngle < finalAngle) {
    for (int i = initAngle; i < finalAngle; i = i + increment) {
      RCServo0.write(i);
      delay(servoDelay);
    }
  } else {
    for (int i = initAngle; i > finalAngle; i = i - increment) {
      RCServo0.write(i);
      delay(servoDelay);
    }
  }
}

void moveServoPhi(int initAngle, int finalAngle, int increment, int servoDelay) {
  if (initAngle < finalAngle) {
    for (int i = initAngle; i < finalAngle; i = i + increment) {
      RCServo1.write(i);
      delay(servoDelay);
    }
  } else {
    for (int i = initAngle; i > finalAngle; i = i - increment) {
      RCServo1.write(i);
      delay(servoDelay);
    }
  }
}

