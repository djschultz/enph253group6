#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  dropoffAnimal(1);
}

void dropoffAnimal(bool dir) {

  //Initialize the angles of the theta and phi joints. These can be obtained from the MATLAB code.
  //Home Angles correspond to position of x = 2.5 in., y = 0.5 in.,
  // and joint angles of thetaJoint = 131, and phiJoint = 90
  //Home Angles correspond to position of x = 2.5 in., y = 0.5 in.,
  // and joint angles of thetaJoint = 131, and phiJoint = 90
  //Extended angles correspond to position of x = 6.5 in. to x = 9.5 in., and y = 0.5 in.
  //Raise angles correspond to position of x = 6.5 in. to x = 9.5 in., and y = 6 in.,
  const int clawQRDPin = 5;
  const int clawQRDThreshold = 200;

  const int numStopsClaw = 7;
  const int homeAngleThetaJoint = 131;
  const int homeAnglePhiJoint = 90;
  const int retractAngleThetaJoint = 210;
  const int retractAnglePhiJoint = 121;
  const int extendAngleThetaJoint = 135;
  const int extendAnglePhiJoint = 53;
  const int raiseAngleThetaJoint = 195;
  const int raiseAnglePhiJoint = 68;

  const int homeAngleTheta = jointToServoTheta(homeAngleThetaJoint);
  const int homeAnglePhi = jointToServoPhi(homeAnglePhiJoint);
  const int retractAngleTheta = jointToServoTheta(retractAngleThetaJoint);
  const int retractAnglePhi = jointToServoPhi(retractAnglePhiJoint);
  const int extendAngleTheta = jointToServoTheta(extendAngleThetaJoint);
  const int extendAnglePhi = jointToServoPhi(extendAnglePhiJoint);
  const int raiseAngleTheta = jointToServoTheta(raiseAngleThetaJoint);
  const int raiseAnglePhi = jointToServoPhi(raiseAnglePhiJoint);

  //Increment in degrees for the servo motors to step by as they move over a larger arc.
  const int increment = 2;
  //Value in milliseconds between incrementing the servo so that it moves smoothly, and the value in milliseconds
  //between doing the different steps in the pickup of animal procedure.
  const int servoDelay = 25;
  const int stepDelay = 2000;
  const int arduinoOutputPin = 10;

  //STEP 1: Ensure that the claw is closed around the animal.
  LCD.print("Closing claw");
  digitalWrite(arduinoOutputPin, LOW);
  delay(2*stepDelay);
  LCD.clear();
  
  //STEP 2: Ensure the servos are at home near the base of the arm (claw near base of the arm)
  LCD.print("Moving to home");
  RCServo0.write(homeAngleTheta);
  RCServo1.write(homeAnglePhi);
  RCServo2.write(90);
  delay(stepDelay);
  LCD.clear();

  //STEP 3: Raise the animal high into the air, just above home
  LCD.print("Raising Animal");
  moveServoTheta(homeAngleTheta, retractAngleTheta, increment, servoDelay);
  moveServoPhi(homeAnglePhi, retractAnglePhi, increment, servoDelay);
  delay(stepDelay);
  LCD.clear();
  
  //STEP 4: Rotate the arm until it faces in the direction of the drop off zone.
  LCD.print("Rotating Arm");
  if (dir) {
    RCServo2.write(179);
  } else {
    RCServo2.write(0);
  }
  delay(stepDelay);
  LCD.clear();

  //STEP 5: Extend the arm over the dropoff zone.
  LCD.print("Moving passenger to dropoff");
  moveServoTheta(retractAngleTheta, raiseAngleTheta, increment, servoDelay);
  moveServoPhi(retractAnglePhi, raiseAnglePhi, increment, servoDelay);
  delay(stepDelay);
  LCD.clear();
  
  //STEP 6: Open the claw so that it releases the animal.
  //This sends a signal to the arduino uno, which will open the claw.
  //This is why there is a longer delay, to allow the arduino to open the claw.
  LCD.print("Opening claw");
  digitalWrite(arduinoOutputPin, HIGH);
  delay(2*stepDelay);
  LCD.clear();

  //STEP 7: Retract claw at high height.
  LCD.print("Retracting claw");
  moveServoPhi(raiseAnglePhi, retractAnglePhi, increment, servoDelay);
  moveServoTheta(raiseAngleTheta, retractAngleTheta, increment, servoDelay);
  delay(stepDelay);
  LCD.clear(); 
  
  //STEP 8: Move the claw back to home.
  LCD.print("moving back to home");
  moveServoPhi(retractAnglePhi, homeAnglePhi, increment, servoDelay);
  moveServoTheta(retractAngleTheta, homeAngleTheta, increment, servoDelay);
  delay(stepDelay);
  LCD.clear();

  //STEP 9: Rotate the claw back to the middle.
  RCServo2.write(90);
  LCD.print("cycle complete!");
  delay(stepDelay);
  LCD.clear();
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

