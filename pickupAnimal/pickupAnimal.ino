#include <phys253.h>
#include <LiquidCrystal.h>

//Calibration:
//Start the Phi motor turned all the way CW when the phi arm is vertical
//Start the Theta motor turned all the way CCW when the arm tip hits the plastic base
//and the Phi motor is all the way CW.
//Theta Servo is connected to RCServo0
//Phi Servo is connected to RCServo1
//The values of servoTheta and servoPhi must always be between
//0 and 180 degrees.

//Initialize the QSD pin and the threshold of QSD signal for the animal to be beside the animal.
const int QSDPin = 0;
const int QSDThreshold = 700;

//Increment in degrees for the servo motors to step by as they move over a larger arc.
const int increment = 2;
//Value in milliseconds between incrementing the servo so that it moves smoothly, and the value in milliseconds
//between doing the different teps in the pickup of animal procedure.
const int servoDelay = 25;
const int stepDelay = 2000;
int count = 0;

void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);
}

void loop() {
  Serial.print("QSD VOLTAGE: ");
  Serial.println(analogRead(QSDPin));
  Serial.print("QRD VOLTAGE: ");
  Serial.println(analogRead(5));

  if (analogRead(QSDPin) >= QSDThreshold) {
    LCD.print("Passenger Detected!");
    LCD.print("Pickup animal!");
    pickupAnimal(1);
  }
}

//This function will pick up an animal in a certain direction, where dir = 0 is left and dir = 1 is right.
//The animal must be adjacent to the robot arm -- this is determined by the
//QSD sensors, which align the robot arm to the animal.
void pickupAnimal(bool dir) {

  //Initialize the angles of the theta and phi joints. These can be obtained from the MATLAB code.
  //Home Angles correspond to position of x = 2.5 in., y = 0.5 in.,
  // and joint angles of thetaJoint = 131, and phiJoint = 90
  //Home Angles correspond to position of x = 2.5 in., y = 0.5 in.,
  // and joint angles of thetaJoint = 131, and phiJoint = 90
  //Extended angles correspond to position of x = 6.5 in. to x = 9.5 in., and y = 0.5 in.
  //Raise angles correspond to position of x = 6.5 in. to x = 9.5 in., and y = 6 in.,
  const int clawQRDPin = 9;
  const int clawQRDThreshold = 990;

  const int numStopsClaw = 7;
  const int homeAngleThetaJoint = 131;
  const int homeAnglePhiJoint = 90;
  const int extendAngleThetaJoint[numStopsClaw] = {132, 133, 135, 137, 139, 141, 144};
  const int extendAnglePhiJoint[numStopsClaw] = {60, 57, 53, 50, 46, 43, 39};
  const int raiseAngleThetaJoint[numStopsClaw] = {180, 179, 180, 180, 180, 181, 181};
  const int raiseAnglePhiJoint[numStopsClaw] = {83, 79, 73, 68, 64, 57, 49};

  const int homeAngleTheta = jointToServoTheta(homeAngleThetaJoint);
  const int homeAnglePhi = jointToServoPhi(homeAnglePhiJoint);
  int extendAngleTheta[numStopsClaw];
  int extendAnglePhi[numStopsClaw];
  int raiseAngleTheta[numStopsClaw];
  int raiseAnglePhi[numStopsClaw];

  //Populate the arrays with the appropriate values, i.e.
  //calculate the corresponding servo angles based on the angles of the joints.
  for (int K = 0; K < numStopsClaw; K++) {
    extendAngleTheta[K] = jointToServoTheta(extendAngleThetaJoint[K]);
    extendAnglePhi[K] = jointToServoPhi(extendAnglePhiJoint[K]);
    raiseAngleTheta[K] = jointToServoTheta(raiseAngleThetaJoint[K]);
    raiseAnglePhi[K] = jointToServoPhi(raiseAnglePhiJoint[K]);
  }



  //Increment in degrees for the servo motors to step by as they move over a larger arc.
  const int increment = 2;
  //Value in milliseconds between incrementing the servo so that it moves smoothly, and the value in milliseconds
  //between doing the different steps in the pickup of animal procedure.
  const int servoDelay = 25;
  const int stepDelay = 2000;
  const int arduinoOutputPin = 10;

  //STEP 1: Set the servo motors to the home position (claw near base of the arm)
  RCServo0.write(homeAngleTheta);
  RCServo1.write(homeAnglePhi);
  RCServo2.write(90);
  LCD.print("Moving to home");
  delay(stepDelay);
  LCD.clear();

  //STEP 2: Rotate the arm until it faces in the direction of the passenger.
  LCD.print("Rotating Arm");
  if (dir) {
    RCServo2.write(179);
  } else {
    RCServo2.write(0);
  }
  delay(stepDelay);
  LCD.clear();

  //STEP 3: Open the claw so that it is open when it approaches the animal.
  //This sends a signal to the arduino uno, which will open the claw.
  //This is why there is a longer delay, to allow the arduino to open the claw.
  LCD.print("Opening claw");
  digitalWrite(arduinoOutputPin, HIGH);
  delay(2 * stepDelay);
  LCD.clear();

  //STEP 4: Extend the claw towards the animal's torso.
  LCD.print("moving to pickup position");
  double QRDVal = analogRead(clawQRDPin);
  moveServoTheta(homeAngleTheta, extendAngleTheta[0], increment, servoDelay);
  moveServoPhi(homeAnglePhi, extendAnglePhi[0], increment, servoDelay);
  int count;
  for (count = 0; count < numStopsClaw - 1; count++) {
    moveServoTheta(extendAngleTheta[count], extendAngleTheta[count + 1], increment, servoDelay);
    moveServoPhi(extendAnglePhi[count], extendAnglePhi[count + 1], increment, servoDelay);
    if (QRDVal > clawQRDThreshold) {
      break;
    } else {
      //QRDVal = analogRead(clawQRDPin);
      QRDVal = knob(clawQRDPin);
    }
  }
  delay(stepDelay);
  LCD.clear();

  //STEP 5: Close the claw around the animal's torso.
  //This sends a signal to the arduino uno, which will close the claw.
  //This is why there is a longer delay, to allow the arduino to close the claw.
  LCD.print("Closing claw");
  digitalWrite(arduinoOutputPin, LOW);
  delay(2 * stepDelay);
  LCD.clear();

  //STEP 6: Raise the claw, with animal in hand.
  LCD.print("Raising claw");
  moveServoTheta(extendAngleTheta[count], raiseAngleTheta[count], increment, servoDelay);
  moveServoPhi(extendAnglePhi[count], raiseAnglePhi[count], increment, servoDelay);
  delay(stepDelay);
  LCD.clear();

  //STEP 7: Move the claw back to home.
  LCD.print("moving back to home");
  moveServoTheta(raiseAngleTheta[count], homeAngleTheta, increment, servoDelay);
  moveServoPhi(raiseAnglePhi[count], homeAnglePhi, increment, servoDelay);
  delay(stepDelay);
  LCD.clear();

  //STEP 8: Rotate the animal back to the middle.
  RCServo2.write(90);
  LCD.print("cycle complete!");
  delay(stepDelay);
  LCD.clear();
}

//This function will pick up an animal in a certain direction, where dir = 0 is left and dir = 1 is right.
void dropoffAnimal(int clawQRDPin, bool dir) {

  //Initialize the angles of the theta and phi joints. These can be obtained from the MATLAB code.
  //Home Angles correspond to position of x = 2.5 in., y = 0.5 in.,
  // and joint angles of thetaJoint = 131, and phiJoint = 90
  //Home Angles correspond to position of x = 2.5 in., y = 0.5 in.,
  // and joint angles of thetaJoint = 131, and phiJoint = 90
  //Extended angles correspond to position of x = 6.5 in. to x = 9.5 in., and y = 0.5 in.
  //Raise angles correspond to position of x = 6.5 in. to x = 9.5 in., and y = 6 in.,
  const int homeAngleThetaJoint = 131;
  const int homeAnglePhiJoint = 90;
  const int extendAngleThetaJoint = 135;
  const int extendAnglePhiJoint = 50;
  const int raiseAngleThetaJoint = 180;
  const int raiseAnglePhiJoint = 73;
  const int homeAngleTheta = jointToServoTheta(homeAngleThetaJoint);
  const int homeAnglePhi = jointToServoPhi(homeAnglePhiJoint);
  int extendAngleTheta = jointToServoTheta(extendAngleThetaJoint);
  int extendAnglePhi = jointToServoPhi(extendAnglePhiJoint);
  int raiseAngleTheta = jointToServoTheta(raiseAngleThetaJoint);
  int raiseAnglePhi = jointToServoPhi(raiseAnglePhiJoint);

  //Increment in degrees for the servo motors to step by as they move over a larger arc.
  const int increment = 2;
  //Value in milliseconds between incrementing the servo so that it moves smoothly, and the value in milliseconds
  //between doing the different steps in the pickup of animal procedure.
  const int servoDelay = 25;
  const int stepDelay = 2000;
  const int arduinoOutputPin = 10;

  //STEP 1: Set the servo motors to the home position (claw near base of the arm)
  RCServo0.write(homeAngleTheta);
  RCServo1.write(homeAnglePhi);
  RCServo2.write(90);
  LCD.print("Moving to home");
  delay(stepDelay);

  //STEP 2: Rotate the arm until it faces in the direction of the drop off
  LCD.print("Rotating Arm");
  if (dir) {
    RCServo2.write(179);
  } else {
    RCServo2.write(0);
  }
  delay(stepDelay);

  //STEP 3: Ensure the claw is closed around the animal before the arm extends.
  //This sends a signal to the arduino uno, which will open the claw.
  //This is why there is a longer delay, to allow the arduino to open the claw.
  LCD.print("Closing");
  digitalWrite(arduinoOutputPin, LOW);
  delay(2 * stepDelay);

  //STEP 4: Extend the claw to release the animal
  LCD.print("moving to pickup position");
  moveServoTheta(homeAngleTheta, extendAngleTheta, increment, servoDelay);
  moveServoPhi(homeAnglePhi, extendAnglePhi, increment, servoDelay);
  delay(stepDelay);

  //STEP 5: Release the animal.
  //This sends a signal to the arduino uno, which will close the claw.
  //This is why there is a longer delay, to allow the arduino to close the claw.
  LCD.print("Opening");
  digitalWrite(arduinoOutputPin, HIGH);
  delay(2 * stepDelay);

  //STEP 6: Move the claw back to home.
  LCD.print("moving back to home");
  moveServoTheta(extendAngleTheta, homeAngleTheta, increment, servoDelay);
  moveServoPhi(extendAnglePhi, homeAnglePhi, increment, servoDelay);
  delay(stepDelay);

  //STEP 7: Rotate the animal back to the middle.
  RCServo2.write(90);
  LCD.print("cycle complete!");
  delay(stepDelay);
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

