#include <phys253.h>
#include <LiquidCrystal.h>

//Start the Phi motor turned all the way CW when the phi arm is vertical
//Start the Theta motor turned all the way CCW when the arm tip hits the plastic base
//and the Phi motor is all the way CW.
//Theta Servo is connected to RCServo0
//Phi Servo is connected to RCServo1
//The values of servoTheta and servoPhi must always be between
//0 and 180 degrees.
double servoTheta;
double servoPhi;


//Home Angles correspond to position of x = 2.5 in., y = 0.5 in., 
// and joint angles of thetaJoint = 131, and phiJoint = 90
const int homeAngleThetaJoint = 130;
const int homeAnglePhiJoint = 90;
//Extended angles correspond to position of x = 6.5 in., y = 0.5 in., 
// and joint angles of thetaJoint = 131, and phiJoint = 61
const int extendAngleThetaJoint = 132;
const int extendAnglePhiJoint = 61;
//Raise angles correspond to position of x = 6.5 in., and y = 6 in., 
// and joint angles of thetaJoint = 178, and phiJoint = 83
const int raiseAngleThetaJoint = 195;
const int raiseAnglePhiJoint = 80;

//Home Angles correspond to position of x = 2.5 in., y = 0.5 in., 
// and joint angles of thetaJoint = 131, and phiJoint = 90
const int homeAngleTheta = jointToServoTheta(homeAngleThetaJoint);
const int homeAnglePhi = jointToServoPhi(homeAnglePhiJoint);
//Extended angles correspond to position of x = 6.5 in., y = 0.5 in., 
// and joint angles of thetaJoint = 131, and phiJoint = 61
const int extendAngleTheta = jointToServoTheta(extendAngleThetaJoint);
const int extendAnglePhi = jointToServoPhi(extendAnglePhiJoint);
//Raise angles correspond to position of x = 6.5 in., and y = 6 in., 
//and joint angles of thetaJoint = 178, and phiJoint = 83
const int raiseAngleTheta = jointToServoTheta(raiseAngleThetaJoint);
const int raiseAnglePhi = jointToServoPhi(raiseAnglePhiJoint);



const int increment = 2;
//Time for motors to open and close in milliseconds
const int motorTimeOpen = 400;
const int motorTimeClose = 100;
//Value in milliseconds between incrementing the servo
const int servoDelay = 25;
int count = 0;

void setup() {
#include <phys253setup.txt>
Serial.begin(9600);
}


void loop() {

Serial.println(homeAngleTheta);
Serial.println(homeAnglePhi);
Serial.println(extendAngleTheta);
Serial.println(extendAnglePhi);
Serial.println(raiseAngleTheta);
Serial.println(raiseAnglePhi);

  if(count%6 == 0){
  servoTheta = homeAngleTheta;
  servoPhi = homeAnglePhi;
  RCServo0.write(servoTheta);
  RCServo1.write(servoPhi); 
  LCD.print("Moving to home");
  } 
  else if(count%6 == 1){
    LCD.print("Opening claw");
    motor.speed(1,200);
    delay(motorTimeOpen);
    motor.speed(1,0);
  } 
  else if(count%6 == 2){
    LCD.print("moving to pickup position");
    servoTheta = 22;
    //slowly move servoPhi to extendAnglePhi
    for(int i = homeAnglePhi; i <= extendAnglePhi; i = i + increment){
      RCServo1.write(i);
      delay(servoDelay);
    }
  } 
  else if(count%6 == 3){
    LCD.print("Closing claw");
    motor.speed(1,-200);
    delay(motorTimeClose);
    motor.speed(1,0);
  } 
  else if(count%6 == 4){
    LCD.print("Raising claw");
    //SLowly move to servoTheta = 86, servoPhi = 16
    for(int i = extendAngleTheta; i <= raiseAngleTheta; i = i + increment){
      RCServo0.write(i);
      delay(servoDelay);
    }
    for(int i = extendAnglePhi; i >=raiseAnglePhi; i = i - increment){
      RCServo1.write(i);
      delay(servoDelay);
    }
      LCD.print("moving back to home");
  } 
  else if(count%6 == 5){
    LCD.print("cycle complete!");
    //SLowly move to servoTheta = 22, servoPhi = 0
    for(int i = raiseAngleTheta; i >= homeAngleTheta; i = i - increment){
      RCServo0.write(i);
      delay(servoDelay);
    }
    for(int i = raiseAnglePhi; i >= homeAnglePhi; i = i - increment){
      RCServo1.write(i);
      delay(servoDelay);
    }
  }
  
  Serial.println("Servo Theta");
  Serial.println(servoTheta);
  Serial.println("Servo Phi");
  Serial.println(servoPhi);
  
  count++;
  delay(2000);
  LCD.clear();
  
}

int jointToServoTheta(int angle){
  int result = 2*(angle - 120);
  return result;
}

int jointToServoPhi(int angle){
  int result = 2*(90 - angle);
  return result;
}
