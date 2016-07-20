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
  
void setup() {
#include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(10, OUTPUT);
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
  const int clawQRDPin = 5;
  const int clawQRDThreshold = 200;

  const int numStopsClaw = 7;
  const int homeAngleThetaJoint = 131;
  const int homeAnglePhiJoint = 90;
  const int retractAngleThetaJoint = 210;
  const int retractAnglePhiJoint = 121;
  const int extendAngleThetaJoint[numStopsClaw] = {132, 133, 135, 137, 139, 141, 144};
  const int extendAnglePhiJoint[numStopsClaw] = {60, 57, 53, 50, 46, 43, 39};
  const int raiseAngleThetaJoint[numStopsClaw] = {196, 195, 195, 195, 193, 193, 193};
  const int raiseAnglePhiJoint[numStopsClaw] = {79, 74, 68, 61, 51, 51, 51};

  const int homeAngleTheta = jointToServoTheta(homeAngleThetaJoint);
  const int homeAnglePhi = jointToServoPhi(homeAnglePhiJoint);
  const int retractAngleTheta = jointToServoTheta(retractAngleThetaJoint);
  const int retractAnglePhi = jointToServoPhi(retractAnglePhiJoint);
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
  LCD.print("Moving to home");
  RCServo0.write(homeAngleTheta);
  RCServo1.write(homeAnglePhi);
  RCServo2.write(90);
  delay(stepDelay);
  LCD.clear();

  //STEP 2: Rotate the arm until it faces in the direction of the passenger.
  LCD.print("Rotating Arm");
  if (dir) {
    //Turn arm to the right
    RCServo2.write(179);
  } else {
    //Turn arm to the left
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
  Serial.println("QRDVal");
  Serial.println(QRDVal);
  moveServoTheta(homeAngleTheta, extendAngleTheta[0], increment, servoDelay);
  moveServoPhi(homeAnglePhi, extendAnglePhi[0], increment, servoDelay);
  int count;
  for (count = 0; count < numStopsClaw - 1; count++) {
    moveServoTheta(extendAngleTheta[count], extendAngleTheta[count + 1], increment, servoDelay);
    moveServoPhi(extendAnglePhi[count], extendAnglePhi[count + 1], increment, servoDelay);
    LCD.print(count);
    Serial.println("QRDVal");
    Serial.println(QRDVal);
    delay(1000);
    if (QRDVal < clawQRDThreshold) {
      break;
    } else {
      QRDVal = analogRead(clawQRDPin);
    }
  }
  delay(stepDelay);
  LCD.clear();

  //STEP 5: Close the claw around the animal's torso.
  //This sends a signal to the arduino uno, which will close the claw.
  //This is why there is a longer delay, to allow the arduino to close the claw.
  LCD.print("Closing claw");
  digitalWrite(arduinoOutputPin, LOW);
  delay(2*stepDelay);
  LCD.clear();

  //STEP 6: Raise the claw, with animal in hand.
  LCD.print("Raising claw");
  moveServoTheta(extendAngleTheta[count], raiseAngleTheta[count], increment, servoDelay);
  moveServoPhi(extendAnglePhi[count], raiseAnglePhi[count], increment, servoDelay);
  delay(stepDelay);
  LCD.clear();

  //STEP 7: Retract claw at high height.
  LCD.print("Retracting claw");
  moveServoPhi(raiseAnglePhi[count],retractAnglePhi, increment, servoDelay);
  moveServoTheta(raiseAngleTheta[count], retractAngleTheta, increment, servoDelay);
  delay(stepDelay);
  LCD.clear(); 
  
  //STEP 7: Move the claw back to home.
  LCD.print("moving back to home");
  moveServoPhi(retractAnglePhi, homeAnglePhi, increment, servoDelay);
  moveServoTheta(retractAngleTheta, homeAngleTheta, increment, servoDelay);
  delay(stepDelay);
  LCD.clear();

  //STEP 8: Rotate the animal back to the middle.
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


