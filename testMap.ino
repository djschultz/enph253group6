#include <phys253.h>
#include <LiquidCrystal.h>
#include <StandardCplusplus.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <set>
#include <algorithm>

using namespace std;
ohserialstream cout(Serial);
//printing for debugging
template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}

//Matrix for storing directions
int directionMatrix[20][20] = {};
int availableNodes[20][4] = { 50 };

//list of deadend nodes (-1 being the one returned when not found)
int deadEnds[] = { 0, 3, 4, 9, 12, 17, -1 };

//Motor Inputs
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 0

int intersectionCount = 0;
int btwnIntersection = 0;

//Speeds
#define FORWARD_SPEED 75
#define DRIVING_TURN_SPEED 90
#define BACKING_TURN_SPEED -70
#define ROTATE_TURN_SPEED 130

//Directions
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

int currentDirection = SOUTH; //initial direction is always south
int prevNode = 0;
int currentNode = 1; //starting node, set 1 if starting at 0 and 18 if starting at 17
bool havePassenger = false;

int endingNode = 15; //for testing the map

           // TINAH INPUTS
int motorLeft = 3;
int motorRight = 0;

const int QRDIntersectionPinLeft = 1;
const int QRDIntersectionPinRight = 5;
const int QRDIntersectionPinMiddle = 3;

const int QRDTapePinLeft = 4;
const int QRDTapePinRight = 2;

const int QSDPinLeft = 0;
const int QSDPinRight = 1;
const int QSDPinFront = 2;
const int QSDPinBeacon = 3;

const int multiplexSwitchPin = 8;

// CLARIFYING CONSTANTS
int defaultSpeed = 75;
int count = 0;
int numLoops = 100;
bool LEFT = 1;
bool RIGHT = 0;

// TAPE FOLLOWING INITIALIZATION
bool QRDTapeRight;
bool QRDTapeLeft;
bool QRDIntersectionRight;
bool QRDIntersectionLeft;
bool QRDIntersectionMiddle;
int QRDError = 0;
int QRDErrorPrev = 0;
int QRDErrorPrevDiff = 0;
int QRDErrorPrevDiffTime = 0;
unsigned long timePrev = 0;
unsigned long timeCurr = 0;
double deltaTime;

//IR Initialiation
int QSDLeft;
int QSDRight;
int QSDFront;
int QSDBeacon;
int directionOfPickup;
int directionOfDropoff;

const int passengerIRThreshold = 300;
const int beaconIRThreshold = 500;

bool detectedIntersection = false;

// PID INITIALIZATION
int derivTerm;
int derivGain;
int propTerm;
int propGain;
int newSpeedLeft;
int newSpeedRight;
bool lastDir;

// MULTIPLEXING INITIALIZATION
bool multiplexDir;

//Claw setup
bool ResetClaw = false;
int dropPickupIndex = 0;
const int clawQRDPin = 5;
const int clawQRDThreshold = 100;
const int arduinoOutputPin = 10;
void setup(void)
{
  // put your setup code here, to run once:
#include <phys253setup.txt>
  Serial.begin(9600);

  // default direction forwards
  pinMode(multiplexSwitchPin, OUTPUT);
  pinMode(10, OUTPUT);

  multiplexDir = HIGH;
  digitalWrite(multiplexSwitchPin, multiplexDir);
  digitalWrite(10, LOW);
  lastDir = LEFT;

  directionMatrix[0][1] = SOUTH;
  directionMatrix[1][0] = NORTH;
  directionMatrix[1][3] = SOUTH;
  directionMatrix[1][6] = EAST;
  directionMatrix[2][1] = NORTH;
  directionMatrix[2][7] = EAST;
  directionMatrix[3][4] = EAST;
  directionMatrix[4][5] = SOUTH;
  directionMatrix[5][3] = WEST;
  directionMatrix[5][4] = NORTH;
  directionMatrix[5][6] = SOUTH;
  directionMatrix[6][1] = WEST;
  directionMatrix[6][5] = NORTH;
  directionMatrix[6][7] = SOUTH;
  directionMatrix[6][8] = EAST;
  directionMatrix[7][2] = WEST;
  directionMatrix[7][6] = NORTH;
  directionMatrix[7][15] = EAST;
  directionMatrix[8][6] = WEST;
  directionMatrix[8][10] = NORTH;
  directionMatrix[8][11] = SOUTH;
  directionMatrix[9][10] = SOUTH;
  directionMatrix[10][8] = WEST;
  directionMatrix[10][9] = NORTH;
  directionMatrix[10][11] = EAST;
  directionMatrix[11][8] = SOUTH;
  directionMatrix[11][10] = NORTH;
  directionMatrix[11][14] = EAST;
  directionMatrix[12][13] = SOUTH;
  directionMatrix[13][12] = NORTH;
  directionMatrix[13][14] = SOUTH;
  directionMatrix[13][16] = EAST;
  directionMatrix[14][11] = WEST;
  directionMatrix[14][13] = NORTH;
  directionMatrix[14][15] = SOUTH;
  directionMatrix[14][18] = EAST;
  directionMatrix[15][7] = WEST;
  directionMatrix[15][14] = NORTH;
  directionMatrix[15][19] = EAST;
  directionMatrix[16][13] = WEST;
  directionMatrix[17][18] = SOUTH;
  directionMatrix[18][14] = WEST;
  directionMatrix[18][16] = NORTH;
  directionMatrix[18][19] = SOUTH;
  directionMatrix[19][14] = WEST;
  directionMatrix[19][18] = NORTH;

  //available nodes arrays
  availableNodes[0][1] = 1;
  //int node1[3] = { 0, 2, 6 };
  availableNodes[1][0] = 0;
  availableNodes[1][1] = 2;
  availableNodes[1][2] = 6;
  //  int node2[2] = { 1, 7 };
  availableNodes[2][0] = 1;
  availableNodes[2][1] = 7;

  //  int node3[1] = { 5 };
  availableNodes[3][0] = 5;
  //  int node4[1] = { 5 };
  availableNodes[4][0] = 5;
  //  int node5[3] = { 3, 4, 6 };
  availableNodes[5][0] = 3;
  availableNodes[5][1] = 4;
  availableNodes[5][2] = 6;
  //int node6[4] = { 1, 5, 7, 8 };
  availableNodes[6][0] = 1;
  availableNodes[6][1] = 5;
  availableNodes[6][2] = 7;
  availableNodes[6][3] = 8;

  //  int node7[3] = { 2, 6, 15 };
  availableNodes[7][0] = 2;
  availableNodes[7][1] = 6;
  availableNodes[7][2] = 15;

  //  int node8[3] = { 6, 10, 11 };
  availableNodes[8][0] = 6;
  availableNodes[8][1] = 10;
  availableNodes[8][2] = 11;

  //  int node9[1] = { 10 };
  availableNodes[9][0] = 10;
  //  int node10[3] = { 8, 9, 11 };
  availableNodes[10][0] = 8;
  availableNodes[10][1] = 9;
  availableNodes[10][2] = 11;

  //  int node11[3] = { 8, 10, 14 };
  availableNodes[11][0] = 8;
  availableNodes[11][1] = 10;
  availableNodes[11][2] = 14;

  //  int node12[1] = { 13 };
  availableNodes[12][0] = 13;
  //  int node13[3] = { 12, 14, 16 };
  availableNodes[13][0] = 12;
  availableNodes[13][1] = 14;
  availableNodes[13][2] = 16;

  //int node14[4] = { 11, 13, 15, 18 };
  availableNodes[14][0] = 11;
  availableNodes[14][1] = 13;
  availableNodes[14][2] = 15;
  availableNodes[14][3] = 18;

  //int node15[3] = { 7, 14, 19 };
  availableNodes[15][0] = 7;
  availableNodes[15][1] = 14;
  availableNodes[15][2] = 19;

  //int node16[1] = { 13 };
  availableNodes[16][0] = 13;
  //int node17[1] = { 18 };
  availableNodes[17][0] = 18;
  //int node18[3] = { 14, 17, 19 };
  availableNodes[18][0] = 14;
  availableNodes[18][1] = 17;
  availableNodes[18][2] = 19;

  //int node19[2] = { 15, 18 };
  availableNodes[19][0] = 15;
  availableNodes[19][1] = 18;

}
void loop() {
  if (!ResetClaw) {
    RCServo0.write(22);
    RCServo1.write(0);
    RCServo2.write(90);
    ResetClaw = true;
  }

  LCD.setCursor(5, 1);

  //tape follow normally
  //START PID
  timeCurr = millis();
  propGain = knob(6);
  derivGain = knob(7);

  // QRDs are true when there is tape below them
  QRDTapeRight = digitalRead(QRDTapePinRight);
  QRDTapeLeft = digitalRead(QRDTapePinLeft);
  QRDIntersectionLeft = digitalRead(QRDIntersectionPinLeft);
  QRDIntersectionRight = digitalRead(QRDIntersectionPinRight);

  QSDLeft = analogRead(QSDPinLeft);
  QSDRight = analogRead(QSDPinRight);
  QSDFront = analogRead(QSDPinFront);
  QSDBeacon = analogRead(QSDPinBeacon);

  //for debugging purposes
  if (QRDIntersectionLeft) {
    LCD.setCursor(0, 1);
    LCD.print("L");
  }
  if (QRDTapeLeft) {
    LCD.setCursor(1, 1);
    LCD.print("Y");
  }
  if (QRDTapeRight) {
    LCD.setCursor(2, 1);
    LCD.print("Y");
  }
  if (QRDIntersectionRight) {
    LCD.setCursor(3, 1);
    LCD.print("R");
  }

  if (QRDError != QRDErrorPrevDiff) {
    QRDErrorPrevDiff = QRDError;
    QRDErrorPrevDiffTime = timeCurr;
  }

  //The QRD Errors correspond to whether or not the QRD is
  //on the tape, and which side of the tape it is on.
  //If the QRDRight and QRDLeft are both true, then there is
  //no error in the position of the QRD reading apparatus.
  //
  //If the QRDRight is true (right side is on tape) and
  //the QRDLeft is false (left side is off tape) then the
  //QRD error is -1, which means the tape following apparatus
  //is slightly off to the left. Vice versa for the QRD error
  //being +1;
  //
  //If both QRD sensors are false (both off of the tape)
  //Then, depending on what the previous reading was
  //the QRD error will be completely off to the right
  //or completely off to the left.
  //QRDError = 0: Robot is on tape
  //QRDError = -1: Robot is slightly off to the left
  //QRDError = +1: Robot is slightly off to the right
  //QRDError = -2: Robot is far off to the left
  //QRDError = +2: Robot is far off to the right
  if (QRDTapeRight && QRDTapeLeft) {
    QRDError = 0;
  }
  else if (QRDTapeRight && !QRDTapeLeft) {
    QRDError = -1;
  }
  else if (!QRDTapeRight && QRDTapeLeft) {
    QRDError = 1;
  }
  else {
    if (QRDErrorPrev > 0) {
      QRDError = 2;
    }
    else {
      QRDError = -2;
    }
  }

  //check if we are at a dead end, and all QRDS are off the tape - then turn around
   bool exists = find(deadEnds, deadEnds + 7, currentNode) != deadEnds + 7;
   if (exists && abs(QRDError) > 1) {
    turnAround();
   }
  // if the robot is off the tape, tape follow
  // calculate the derivation & propotional gains
  derivTerm = derivGain * (QRDError - QRDErrorPrevDiff) / (timeCurr - QRDErrorPrevDiffTime);
  propTerm = propGain * QRDError;
  deltaTime = (timeCurr - timePrev) / 1000.0;

  newSpeedLeft = defaultSpeed - propTerm - derivTerm;
  newSpeedRight = defaultSpeed + propTerm + derivTerm;

  QRDErrorPrev = QRDError;
  timePrev = timeCurr;

  // do not exceed the maximum speeds for either motor
  if (newSpeedLeft > 255) newSpeedLeft = 255;
  else if (newSpeedLeft < -255) newSpeedLeft = -255;
  if (newSpeedRight > 255) newSpeedRight = 255;
  else if (newSpeedRight < -255) newSpeedRight = -255;
  btwnIntersection++;

  //if intersection QRDs pick up signal, we are at a node and calculate direction we want
  if ((QRDIntersectionRight || QRDIntersectionLeft) && btwnIntersection > 150) {
    btwnIntersection = 0;
    intersectionCount = 0;
    while (QRDIntersectionRight || QRDIntersectionLeft) {
      intersectionCount++;
      if (intersectionCount > 20) { break; } //detect the intersection at least 10 times, to make sure it wasn't a QRD mistake
    }
    if (intersectionCount > 20) {
      if (!havePassenger) {
        prevNode = currentNode;
        delay(120);
        currentNode = turnTowardQSDSignal(QSDLeft, QSDRight, QSDFront, currentDirection, currentNode);
      }
      else {
        //available directions at current node:
        //int[] availableDirections = availableNodes[currentNode];
        int idealNode = availableNodes[currentNode][0];
        int idealDiff = endingNode - idealNode;

        //loop through all available paths to find the ideal path to get to endingNode
        for (int index = 0; index < 4; index++) {
          int currAvailNode = availableNodes[currentNode][index];
          if (currAvailNode == 50)
            continue;

          int currDiff = endingNode - currAvailNode;
          if (abs(currDiff) < abs(idealDiff)) {
            idealNode = currAvailNode;
            idealDiff = currDiff;
          }
        }
        LCD.print("iN:");
        LCD.print(idealNode);

        int directionToGo = directionMatrix[currentNode][idealNode];
        LCD.clear();
        LCD.print("CD:");
        LCD.print(currentDirection);
        LCD.print("DTG:");
        LCD.print(directionToGo);
        delay(120);

        moveInDirection(currentDirection, directionToGo);
        //account for direction changes in circle
        if (currentNode == 8 && idealNode == 10) {
          currentDirection = EAST;
        }
        else if (currentNode == 8 && idealNode == 11) {
          currentDirection = NORTH;
        }
        else if (currentNode == 10) {
          currentDirection = SOUTH;
        }
        else if (currentNode == 11 && idealNode == 10) {
          currentDirection = WEST;
        }
        else if (currentNode == 11 && idealNode == 8) {
          currentDirection = NORTH;
        }
        else {
          currentDirection = directionToGo;
        }
        prevNode = currentNode;
        currentNode = idealNode;
      }
    }
  }



  //check IR if we dont have passenger
  if (!havePassenger) {
    motor.speed(motorLeft, 0);
    motor.speed(motorRight, 0);
    Serial.println("LEFT QSD");
    Serial.println(QSDLeft);
    Serial.println("RIGHT QSD");
    Serial.println(QSDRight);
    Serial.println("FRONT QSD");
    Serial.println(QSDFront);
    if (QSDLeft > passengerIRThreshold) {
      directionOfPickup = LEFT;
     // pickupAnimal(directionOfPickup);
    }

    else if (QSDRight > passengerIRThreshold) {
      directionOfPickup = RIGHT;
      //pickupAnimal(directionOfPickup);
    }
    if (abs(currentNode - 7) < abs(currentNode - 15))
      endingNode = 7;
    else {
      endingNode = 15;
    }
    havePassenger = true;
  }

  //Dropoff if we do have passenger
  else {
    motor.speed(motorLeft, 0);
    motor.speed(motorRight, 0);
    if (QSDBeacon > beaconIRThreshold) {
      if (currentNode == 15 && prevNode == 7) dropoffAnimal(RIGHT);
      else if (currentNode == 7 && prevNode == 15) dropoffAnimal(LEFT);
    }

   // havePassenger = false;
  }

  
  //move in the appropriate direction
  motor.speed(motorLeft, newSpeedLeft);
  motor.speed(motorRight, newSpeedRight);
  LCD.print(newSpeedLeft);
  LCD.print(newSpeedRight);
  // this determines when the print the output data to the serial port.
  LCD.clear();
  LCD.print("DG: ");
  LCD.print(derivGain);
  LCD.print(" ");
  LCD.print("PG: ");
  LCD.print(propGain);
  LCD.print(currentNode);
}

void turnLeft() {
  //move forward a little first
  motor.speed(motorLeft, FORWARD_SPEED);
  motor.speed(motorRight, FORWARD_SPEED);
  LCD.print("Lturn");
  //delay(500);
  //turn 90 degrees to the left
  motor.speed(RIGHT_MOTOR, DRIVING_TURN_SPEED);
  motor.speed(LEFT_MOTOR, BACKING_TURN_SPEED);
  delay(100);
  while (!(digitalRead(QRDTapePinRight) && digitalRead(QRDTapePinLeft))) {
    delay(1);

  }
}

void turnRight() {
  //move forward a little first
  motor.speed(motorRight, FORWARD_SPEED);
  motor.speed(motorLeft, FORWARD_SPEED);
  //delay(500);
  //turn 90 degrees to the right
  motor.speed(LEFT_MOTOR, DRIVING_TURN_SPEED);
  motor.speed(RIGHT_MOTOR, BACKING_TURN_SPEED);
  delay(100);

  while (!(digitalRead(QRDTapePinRight) && digitalRead(QRDTapePinLeft))) {
    delay(1);

  }
}

void turnForward() {
  motor.speed(RIGHT_MOTOR, FORWARD_SPEED);
  motor.speed(LEFT_MOTOR, FORWARD_SPEED);
  delay(10);
}

void turnAround() {
  motor.speed(RIGHT_MOTOR, ROTATE_TURN_SPEED);
  motor.speed(LEFT_MOTOR, (-1)*ROTATE_TURN_SPEED);
  delay(200);
  while (!(digitalRead(QRDTapePinRight) && digitalRead(QRDTapePinLeft))) {
  }
}

void moveInDirection(int currentDirection, int directionOfNode) {
  int difference = currentDirection - directionOfNode;
  if (difference == 0)
  {
    turnForward();
  }
  else if (abs(difference) == 2) {
    turnAround();
  }
  else if (difference == -1 || difference == 3) {
    turnRight();
    QRDError = -2;

  }
  else if (difference == 1 || difference == -3) {
    turnLeft();
    QRDError = 2;
  }
}

int turnTowardQSDSignal(int QSDLeft, int QSDRight, int QSDForward, int currentDirection, int currentNode) {
  int ableToTurn[3] = {0}; //Left, Right, Forward - nodes from current node
  int IRSignals[3] = {0};

  for(int i = 0; i < 4; i++){
    int availNode = availableNodes[currentNode][i];
    if(availNode != 50){
        int availDir = currentDirection - directionMatrix[currentNode][availNode];
        if(availDir == 0){
          ableToTurn[2] = availNode;
          IRSignals[2] = QSDForward;
        }
        else if (availDir == -1 || availDir == 3) {
          ableToTurn[1] = availNode;
          IRSignals[1] = QSDRight;
        }
        else if (availDir == 1 || availDir == -3){
          ableToTurn[0] = availNode;  
          IRSignals[0] = QSDLeft;
        }
     }
  }
  int maxSignal = 0;
  int indexOfMax = 0;
  for (int i = 0; i < 3; i++) {
    if (IRSignals[i] > maxSignal) {
      maxSignal = IRSignals[i];
      indexOfMax = i;
    }
  }

  if(indexOfMax == 0){ turnLeft(); }
  else if(indexOfMax == 1){ turnRight(); }
  else if(indexOfMax == 2){ turnForward(); }

  return ableToTurn[indexOfMax];
}

//-------ARM CODE-------
//right = 180 degrees
//left = 0 degrees
//where Theta is Servo0 
//and Phi is Servo1
void pickupAnimal(bool dir){
  int stepDelay = 500;
  
  int extendAngleTheta[] = { 31.29, 0.35, 15.29, 27.95, 30.06, 32.17, 52.38, 75.06, 98.09, 110.45}; //every half inch from 5" to 11"
  int extendAnglePhi[] = { 13.18, 69.79, 79.63, 82.97, 100.55, 115.31, 123.22, 139.57, 150.29, 172.89 }; //every half inch from 5" to 11"
  int raiseAngleTheta[] = {150.51, 113.38, 87.3, 82.44, 66.3, 69.26, 69.25, 62.4, 50}; //every half inch from 8" to 5"
  int raiseAnglePhi[] = {106.15, 80.2, 82.27, 71.02, 58.54, 62.75, 49.22, 33.75, 13.18};
  
  int numExtendSteps = 10;
  int numRaiseSteps = 9;
  //Rotate arm to correct side
  LCD.clear();
  LCD.print("Rotating Arm");
  if(!dir) {turnAround(); reverseDirection();}
  delay(stepDelay);

  //Open claw
  LCD.clear();
  LCD.print("Opening claw");
  RCServo2.write(80);
  delay(stepDelay);
  
  //Reach out to animal in increments
  LCD.clear();
  LCD.print("Moving to animal");

  for(int i = 0; i < numExtendSteps; i++){
    RCServo0.write(extendAngleTheta[i]);
    RCServo1.write(extendAnglePhi[i]);
    double QRDVal = analogRead(clawQRDPin);
    Serial.println("QRDVal");
    Serial.println(QRDVal);
    delay(1.5*stepDelay);
    Serial.println(i);
    if(QRDVal < clawQRDThreshold){
      dropPickupIndex = i;
      break;
    }    
  }

  //Close claw
  LCD.clear();
  LCD.print("Closing claw");
  RCServo2.write(110);
  delay(stepDelay);

  //Lift animal
  LCD.clear();
  LCD.print("Lifting animal");
  if (dropPickupIndex >= (numRaiseSteps - 1)) {  //if we picked up the animal at 8"
    dropPickupIndex = 0;
    RCServo0.write(raiseAngleTheta[dropPickupIndex]);
    RCServo1.write(raiseAnglePhi[dropPickupIndex]);
    RCServo0.write(raiseAngleTheta[dropPickupIndex + 1]);
    RCServo1.write(raiseAnglePhi[dropPickupIndex + 1]);
  }
  else { 
    dropPickupIndex = abs(7 - dropPickupIndex);
    RCServo0.write(raiseAngleTheta[dropPickupIndex]);
    RCServo1.write(raiseAnglePhi[dropPickupIndex]);
  }

  RCServo0.write(raiseAngleTheta[numRaiseSteps - 1 ]);
  RCServo1.write(raiseAnglePhi[numRaiseSteps - 1]);
  
  delay(stepDelay);

   //Rotate back to new home (right side of robot)
  LCD.clear();
  LCD.print("Going home");
  delay(5000);  

}

void dropoffAnimal(bool dir) {
  int stepDelay = 500;

  //Rotate arm to dropoff side
  if(!dir) {turnAround();
  reverseDirection();}

  delay(stepDelay);
  RCServo0.write(66.3);
  RCServo1.write(68.54);
    delay(stepDelay);

  //Open claw
  LCD.clear();
  LCD.print("Opening claw");
  RCServo2.write(80);
  delay(stepDelay);

  //Go home
    RCServo0.write(22);
    RCServo1.write(0);
}

void reverseDirection(){
  if(currentDirection == NORTH) {currentDirection = SOUTH;}  
  else  if(currentDirection == SOUTH) {currentDirection = NORTH;}  
  else if(currentDirection == EAST) {currentDirection = WEST;}  
  else  if(currentDirection == WEST) {currentDirection = EAST;}  
}
