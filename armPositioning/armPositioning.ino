#include <phys253.h>
#include <LiquidCrystal.h>

//Start the Phi motor turned all the way CW when the phi arm is vertical
//Start the Theta motor turned all the way CCW when the arm tip hits the plastic base
//and the Phi motor is all the way CW.
//Theta Servo is connected to RCServo0
//Phi Servo is connected to RCServo1
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
  
  if(count%1000 == 0){
  Serial.println("Servo Theta");
  Serial.println(servoTheta);
  Serial.println("Servo Phi");
  Serial.println(servoPhi);
  double jointTheta = servoTheta/3.0 + 90.0;
  double jointPhi = (servoPhi)/2.0 + 120;
  Serial.println("Joint Theta");
  Serial.println(jointTheta);
  Serial.println("Joint Phi");
  Serial.println(jointPhi);
  }
  
  RCServo0.write(servoTheta);
  RCServo1.write(servoPhi);
  count++; 
}


