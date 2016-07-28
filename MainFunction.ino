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

// ------ MAP ------ // 
//Matrix for storing possible directions from each node
int directionMatrix[20][20] = {};
//Matrix for storing the accessible nodes from each node 
int availableNodes[20][4] = { { 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 },{ 50, 50, 50, 50 } };
//Default path when we start at 0th node
int zeroPath[34] = { 0, 1, 6,  5,  3,  5,  4,  5,  6,   8, 10, 9, 10, 11, 14, 13, 12, 13, 16, 13, 14, 18, 17, 18, 19, 15, 14, 11,  8,  6, 7, 2,  1, 0 };
//Default path when we start at 17th node
int seventeenPath[34] = { 17, 18, 14, 13, 16, 13, 12, 13, 14, 11, 10, 9, 10, 8,  6,   5,  4,  5,  3,  5,  6,  1,  0,  1,  2,  7,  6,  8, 11, 14, 15, 19, 18, 17 };

//Directions
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

int currentDirection = SOUTH; //initial direction is always south
int prevNode = 0;
int currentNode = 1; //starting node, set 1 if starting at 0 and 18 if starting at 17
bool havePassenger = false;
bool startAtZero = true; //set which node we're starting at
int endingNode = 15; //for testing the map


// ------- MOTOR INPUTS ------- //
#define motorLeft 3
#define motorRight 0

//Speeds
#define FORWARD_SPEED 75
#define DRIVING_TURN_SPEED 90
#define BACKING_TURN_SPEED -70
#define ROTATE_TURN_SPEED 50


// -------- TINAH INPUTS ---------- //
//QRD SENSORS
const int QRDTapePinLeft = 0;
const int QRDTapePinRight = 2;

const int QRDIntersectionPinLeft = 1;
const int QRDIntersectionPinRight = 3;

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

int intersectionCount = 0; //for checking that we are at an intersection (read QRD signal multiple times)
int btwnIntersection = 0; //for delaying intersection detection between intersections

//IR SENSORS
const int QSDPinLeft = 0;
const int QSDPinRight = 1;
const int QSDPinFront = 2;
const int QSDPinBeacon = 3;

//IR Initialiation
int QSDLeft;
int QSDRight;
int QSDFront;
int QSDBeacon;
int directionOfPickup;
int directionOfDropoff;

const int passengerIRThreshold = 300;
const int frontPassengerIRThreshold = 150;
const int beaconIRThreshold = 100;

// CLARIFYING CONSTANTS
int defaultSpeed = 100;
int count = 0;
int numLoops = 100;
bool LEFT = 1;
bool RIGHT = 0;

// ------- PID -------- // 
// PID INITIALIZATION
int prevNumLoops = 0;
int currNumLoops = 1;
int derivTerm;
int derivGain;
int propTerm;
int propGain;
int newSpeedLeft;
int newSpeedRight;

// ------- CLAW & ARM ------- //
bool ResetClaw = false;
int dropPickupIndex = 0;
const int clawQRDPin = 5;
const int clawQRDThreshold = 150;
const int arduinoOutputPin = 10;

double thetaHome = 0.0; //angle
double phiHome = 0.0; //angle
double clawClose = 103; //angle
double clawOpen = 75; //angle
bool start = false;

// ------- COLLISION SENSORS ------ //
int pin0 = 8;
int pin1 = 9;
int pin2 = 10;
int pin3 = 11;
int pin4 = 12;
int pin5 = 13;
int pin6 = 14;
int pin7 = 15;



void setup(void)
{
  // put your setup code here, to run once:
#include <phys253setup.txt>
  Serial.begin(9600);

  directionMatrix[0][1] = SOUTH;
  directionMatrix[1][0] = NORTH;
  directionMatrix[1][2] = SOUTH;
  directionMatrix[1][6] = EAST;
  directionMatrix[2][1] = NORTH;
  directionMatrix[2][7] = EAST;
  directionMatrix[3][5] = EAST;
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
  directionMatrix[18][17] = NORTH;
  directionMatrix[18][19] = SOUTH;
  directionMatrix[19][15] = WEST;
  directionMatrix[19][18] = NORTH;

  //available nodes arrays
  availableNodes[0][0] = 1;
  availableNodes[1][0] = 0;
  availableNodes[1][1] = 2;
  availableNodes[1][2] = 6;
  availableNodes[2][0] = 1;
  availableNodes[2][1] = 7;
  availableNodes[3][0] = 5;
  availableNodes[4][0] = 5;
  availableNodes[5][0] = 3;
  availableNodes[5][1] = 4;
  availableNodes[5][2] = 6;
  availableNodes[6][0] = 1;
  availableNodes[6][1] = 5;
  availableNodes[6][2] = 7;
  availableNodes[6][3] = 8;
  availableNodes[7][0] = 2;
  availableNodes[7][1] = 6;
  availableNodes[7][2] = 15;
  availableNodes[8][0] = 6;
  availableNodes[8][1] = 10;
  availableNodes[8][2] = 11;
  availableNodes[9][0] = 10;
  availableNodes[10][0] = 8;
  availableNodes[10][1] = 9;
  availableNodes[10][2] = 11;
  availableNodes[11][0] = 8;
  availableNodes[11][1] = 10;
  availableNodes[11][2] = 14;
  availableNodes[12][0] = 13;
  availableNodes[13][0] = 12;
  availableNodes[13][1] = 14;
  availableNodes[13][2] = 16;
  availableNodes[14][0] = 11;
  availableNodes[14][1] = 13;
  availableNodes[14][2] = 15;
  availableNodes[14][3] = 18;
  availableNodes[15][0] = 7;
  availableNodes[15][1] = 14;
  availableNodes[15][2] = 19;
  availableNodes[16][0] = 13;
  availableNodes[17][0] = 18;
  availableNodes[18][0] = 14;
  availableNodes[18][1] = 17;
  availableNodes[18][2] = 19;
  availableNodes[19][0] = 15;
  availableNodes[19][1] = 18;

}


void loop() {
  //set starting at 0th node
  if (startbutton() && !start) {
    currentNode = zeroPath[1];
    prevNode = zeroPath[0];
    startAtZero = true;
    start = true;
  }
  //set starting at 17th node
  else if (stopbutton() && !start) {
    currentNode = seventeenPath[1];
    prevNode = seventeenPath[0];
    startAtZero = false;
    start = true;
  }

  //reset claw & arm position
  if (!ResetClaw) {
    RCServo0.write(thetaHome);
    RCServo1.write(phiHome);
    RCServo2.write(clawOpen);
    ResetClaw = true;
  }
/*
  while (digitalRead(pin0) || digitalRead(pin1) || digitalRead(pin2) || digitalRead(pin3)) {
    turnForward();
  }
  while (digitalRead(pin4) || digitalRead(pin5) || digitalRead(pin6) || digitalRead(pin7)) {
    reverseDirection();
    turnAround();
  }*/

  LCD.setCursor(5, 1);

  //tape follow normally
  //START PID
  propGain = knob(6);
  derivGain = knob(7);

  // QRDs are true when there is tape below them
  QRDTapeRight = digitalRead(QRDTapePinRight);
  QRDTapeLeft = digitalRead(QRDTapePinLeft);
  QRDIntersectionLeft = digitalRead(QRDIntersectionPinLeft);
  QRDIntersectionRight = digitalRead(QRDIntersectionPinRight);

  QSDLeft = 0;//analogRead(QSDPinLeft);
  QSDRight = 0;//analogRead(QSDPinRight);
  QSDFront = 0;//analogRead(QSDPinFront);
  QSDBeacon = 0;//analogRead(QSDPinBeacon);

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
    prevNumLoops = currNumLoops;
    currNumLoops = 1;
   // QRDErrorPrevDiffTime = timeCurr;
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

  // if the robot is off the tape, tape follow
  // calculate the derivation & propotional gains and the appropriate tape following speeds
  
  derivTerm = (int)((float)derivGain * (float)(QRDError - QRDErrorPrevDiff) / (float)(prevNumLoops + currNumLoops));
  propTerm = propGain * QRDError;
  //deltaTime = (timeCurr - timePrev) / 1000.0;

  newSpeedLeft = defaultSpeed - propTerm - derivTerm;
  newSpeedRight = defaultSpeed + propTerm + derivTerm;

  QRDErrorPrev = QRDError;
  timePrev = timeCurr;

  // do not exceed the maximum speeds for either motor
  if (newSpeedLeft > 255) newSpeedLeft = 255;
  else if (newSpeedLeft < -255) newSpeedLeft = -255;
  if (newSpeedRight > 255) newSpeedRight = 255;
  else if (newSpeedRight < -255) newSpeedRight = -255;
  
  if(newSpeedLeft < 10 && newSpeedLeft > 0) {
    newSpeedLeft = 10;
  }
  else if(newSpeedLeft > -10 && newSpeedLeft < 0){
    newSpeedLeft = -10;
  }
 if(newSpeedRight < 10 && newSpeedRight > 0) {
    newSpeedRight = 10;
  }
  else if(newSpeedRight > -10 && newSpeedRight < 0){
    newSpeedRight = -10;
  }

  btwnIntersection++;

  //DEAD END DETECTION
  if ((currentNode == 0 || currentNode == 3 || currentNode == 4 || currentNode == 9 || currentNode == 12 || currentNode == 17) && (digitalRead(pin5) || digitalRead(pin6)) ) {
   // Serial.println("At dead end, turn around");

    reverseDirection();
    turnAround();
    prevNode = currentNode;
    if (currentNode == 0) {
      currentNode = 1;
    }
    else if (currentNode == 3 || currentNode == 4) {
      currentNode = 5;
    }
    else if (currentNode == 9) {
      currentNode = 10;
    }
    else if (currentNode == 12 || currentNode == 16) {
      currentNode = 13;
    }
    else if (currentNode == 17) {
      currentNode = 18;
    }
  }

  //if intersection QRDs pick up signal, we are at a node and calculate direction we want
  else if ((QRDIntersectionRight || QRDIntersectionLeft) && btwnIntersection > 100) {
    btwnIntersection = 0;
    intersectionCount = 0;
    while (QRDIntersectionRight || QRDIntersectionLeft) {
      intersectionCount++;
      if (intersectionCount > 50) {
        break;  //detect the intersection at least 10 times, to make sure it wasn't a QRD mistake
      }
    }
    if (intersectionCount > 50) {
    //  stopMotors();
    //  delay(1000);
      if (!havePassenger) { 
        //if we don't have a passenger, head in direction of greatest QSD signal

        int tempNode = prevNode;
        prevNode = currentNode;
        int currDir = currentDirection;
        currentNode = turnTowardQSDSignal(QSDLeft, QSDRight, QSDFront, currentDirection, prevNode, tempNode);
        //debugging
        LCD.clear();
        LCD.print("CN ");
        LCD.print(prevNode);
        LCD.print(" NN ");
        LCD.print(currentNode);
        LCD.setCursor(3, 1);
        LCD.print(" CD ");
        LCD.print(currDir);
        LCD.print(" ND ");
        LCD.print(directionMatrix[prevNode][currentNode]);
        //  delay(5000);

        moveInDirection(currDir, directionMatrix[prevNode][currentNode]);
        currentDirection = directionMatrix[prevNode][currentNode];
        
        if (prevNode == 8 && currentNode == 10) {
          currentDirection = EAST;
        }
        else if (prevNode == 8 && currentNode == 11) {
          currentDirection = NORTH;
        }
        else if (prevNode == 10) {
          currentDirection = SOUTH;
        }
        else if (prevNode == 11 && currentNode == 10) {
          currentDirection = WEST;
        }
        else if (prevNode == 11 && currentNode == 8) {
          currentDirection = NORTH;
        }
      }
      else {
        //we are trying to head to an "ending node" (either 7 or 15) for dropoff since we have a passenger
        //calculate ideal node to turn to 

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
  if (!havePassenger && (QSDLeft > 0 || QSDRight > 0)) {
    LCD.print("QSD");
    LCD.print(QSDLeft);
    LCD.print(" ");
    LCD.print(QSDRight);
    LCD.print(" ");
    LCD.print(QSDFront);
    newSpeedLeft = newSpeedLeft / 2.5;
    newSpeedRight = newSpeedRight / 2.5;
    //Serial.println("LEFT QSD");
    //Serial.println(QSDLeft);
    //Serial.println("RIGHT QSD");
    //Serial.println(QSDRight);
    //Serial.println("FRONT QSD");
    //Serial.println(QSDFront);
    if (QSDLeft > passengerIRThreshold) {
      stopMotors();
      pickupAnimal(LEFT);
      while (analogRead(QSDPinLeft) > passengerIRThreshold) {
        pickupAnimal(LEFT);
      }
      havePassenger = true;
      if (abs(currentNode - 7) < abs(currentNode - 15))
        endingNode = 7;
      else {
        endingNode = 15;
      }
    }
    else if (QSDRight > passengerIRThreshold) {
      stopMotors();
      pickupAnimal(RIGHT);
      while (analogRead(QSDPinRight) > passengerIRThreshold) {
        pickupAnimal(RIGHT); //try again
      }
      havePassenger = true;
      if (abs(currentNode - 7) < abs(currentNode - 15))
        endingNode = 7;
      else {
        endingNode = 15;
      }
    }
  }
  else if (havePassenger) {
    //move across edge between 7th and 15th node
    if (currentNode == 7)
      endingNode = 15;
    else if (currentNode == 15)
      endingNode = 7;
  }
  //Dropoff if we do have passenger
  //Serial.print("QSD Beacon value: ");
  //Serial.print(QSDBeacon);
  if (havePassenger && QSDBeacon > beaconIRThreshold) {
    delay(1000);
    stopMotors();
    dropoffAnimal(LEFT);
    havePassenger = false;
    if (currentNode == 15 && prevNode == 7) {
      dropoffAnimal(RIGHT);
      havePassenger = false;
    }
    else if (currentNode == 7 && prevNode == 15) {
      dropoffAnimal(LEFT);
      havePassenger = false;
    }
  }

  Serial.println("LEFT ");
  Serial.println(newSpeedLeft);
  Serial.println("RIGHT ");
  Serial.println(newSpeedRight);
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
  motor.speed(motorRight, DRIVING_TURN_SPEED);
  motor.speed(motorLeft, BACKING_TURN_SPEED);
  delay(50);
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
  motor.speed(motorLeft, DRIVING_TURN_SPEED);
  motor.speed(motorRight, BACKING_TURN_SPEED);
  delay(50);

  while (!(digitalRead(QRDTapePinRight) && digitalRead(QRDTapePinLeft))) {
    delay(1);

  }
}

void turnForward() {
  motor.speed(motorRight, FORWARD_SPEED);
  motor.speed(motorLeft, FORWARD_SPEED);
  delay(10);
}

void turnAround() {
  motor.speed(motorRight, ROTATE_TURN_SPEED);
  motor.speed(motorLeft, (-1)*ROTATE_TURN_SPEED);
  delay(5);
  while (!(digitalRead(QRDTapePinRight) && digitalRead(QRDTapePinLeft))) {
  }
}

void turn90(bool dir) {
  if (dir) { //turn left 
    motor.speed(motorRight, ROTATE_TURN_SPEED);
    motor.speed(motorLeft, (-1)*ROTATE_TURN_SPEED);
  }
  else {
    motor.speed(motorRight, (-1)*ROTATE_TURN_SPEED);
    motor.speed(motorLeft, ROTATE_TURN_SPEED);
  }

  delay(5);
  while (analogRead(QSDPinFront) < frontPassengerIRThreshold) {
  }
  stopMotors();
}

void turnBack90(bool dir) {
  if(dir) {//turn right now
    motor.speed(motorRight, (-1)*ROTATE_TURN_SPEED);
    motor.speed(motorLeft, ROTATE_TURN_SPEED);
  }
  else {
    motor.speed(motorRight, ROTATE_TURN_SPEED);
    motor.speed(motorLeft, (-1)*ROTATE_TURN_SPEED);
  }

  delay(5);
  while (!digitalRead(QRDTapePinLeft) && !digitalRead(QRDTapePinRight)) {
  }
  stopMotors();
}

void stopMotors() {
  motor.speed(motorRight, 0);
  motor.speed(motorLeft, 0);
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

int turnTowardQSDSignal(int QSDLeft, int QSDRight, int QSDForward, int currentDirection, int currentNode, int prevNode) {
  int ableToTurn[3] = { -1, -1, -1 }; //Left, Forward, Right - nodes from current node
  int IRSignals[3] = { 0 };
 // Serial.println("Current Direction");
 // Serial.println(currentDirection);
  for (int i = 0; i < 4; i++) {
    int availNode = availableNodes[currentNode][i];
    if (availNode != 50) {
      int availDir = currentDirection - directionMatrix[currentNode][availNode];
      if (availDir == -1 || availDir == 3) {
    //    Serial.println("Right Node");
    //    Serial.println(availNode);

        ableToTurn[2] = availNode;
        IRSignals[2] = QSDRight;
      }
      else if (availDir == 0) {
     //   Serial.println("Forward Node");
     //   Serial.println(availNode);
        ableToTurn[1] = availNode;
        IRSignals[1] = QSDForward;
      }
      else if (availDir == 1 || availDir == -3) {
      //  Serial.println("Left Node");
      //  Serial.println(availNode);
        ableToTurn[0] = availNode;
        IRSignals[0] = QSDLeft;
      }
    }
  }
  int maxSignal = 0;
  int minSignal = 200;
  int indexOfMax = -1;
  int indexOfMin = -1;
  for (int i = 0; i < 3; i++) {
    //  Serial.println("IR Signal ith");
    //  Serial.println( IRSignals[i]);
    if (IRSignals[i] > maxSignal) {
      maxSignal = IRSignals[i];
      indexOfMax = i;
    }
    if (IRSignals[i] < minSignal) {
      minSignal = IRSignals[i];
      indexOfMin = i;
    }
  }

  maxSignal = maxSignal - minSignal; //account for noise offset, if any

  //No signals found, go on predetermined path
  //  delay(10000);
  int idealNode = -1;
  if (maxSignal == 0) {
    if (startAtZero) {
      for (int i = 0; i < 32; i++) {
        if (zeroPath[i] == prevNode && zeroPath[i + 1] == currentNode) {
          idealNode = zeroPath[i + 2];
        }
        else if (currentNode == 0 && prevNode == 1) {
          idealNode = 1;
        }
      }
    }
    else {
      for (int i = 0; i < 32; i++) {
        if (seventeenPath[i] == prevNode && seventeenPath[i + 1] == currentNode) {
          idealNode = seventeenPath[i + 2];
        }
        else if (currentNode == 17 && prevNode == 18) {
          idealNode = 18;
        }
      }
    }
    if (idealNode == -1) { //not on mapped path
                 // Serial.println("Trying to go to leftmost Node");
      if (currentNode == 15 && prevNode == 7) { //just finished a dropoff, exit the dropoff zone and go back to mapped route11
        idealNode = 14;
      }
      else if (currentNode == 7 && prevNode == 15) {
        idealNode = 6;
      }
      else if (ableToTurn[0] != -1) {
        idealNode = ableToTurn[0];
      }
      else if (ableToTurn[1] != -1) {
        idealNode = ableToTurn[1];
      }
      else if (ableToTurn[2] != -1) {
        idealNode = ableToTurn[2];
      }
    }
  }
  //if there is a max IR signal, go towards it
  else if (indexOfMax == 0 && ableToTurn[0] != -1) {
    idealNode = ableToTurn[indexOfMax];
  }
  else if (indexOfMax == 1 && ableToTurn[1] != -1) {
    idealNode = ableToTurn[indexOfMax];
  }
  else if (indexOfMax == 2 && ableToTurn[2] != -1) {
    idealNode = ableToTurn[indexOfMax];
  }

  return idealNode;
}

void reverseDirection() {
  if (currentDirection == NORTH) {
    currentDirection = SOUTH;
  }
  else  if (currentDirection == SOUTH) {
    currentDirection = NORTH;
  }
  else if (currentDirection == EAST) {
    currentDirection = WEST;
  }
  else  if (currentDirection == WEST) {
    currentDirection = EAST;
  }
}


//-------ARM CODE-------
//right = 180 degrees (false)
//left = 0 degrees (true)
//where Theta is Servo0
//and Phi is Servo1
void pickupAnimal(bool dir) {
  int stepDelay = 250;

  int extendAngleTheta[] = { 0, 36.74, 47.46, 54.84, 63.63, 67.5, 74.06, 84.55, 84.55, 99.49, 104.05, 110.86, 120.49, 132.89}; //every quarter inch from 6" to 10"
  int extendAnglePhi[] = { 0, 80.51, 83.14, 87.36, 87.36, 98.44, 97.73, 97.15, 104.59, 107.4, 112.85, 115.84, 120.59, 129.2}; //every quarter inch from 6" to 10"
  int raiseAngleTheta[] = { 100.01, 0.18}; //every half inch from 8" to 5"
  int raiseAnglePhi[] = { 0, 0};

  int numExtendSteps = 10;
  int numRaiseSteps = 9;
  //Rotate arm to correct side
  LCD.clear();
  LCD.print("Turning ");
  if(dir){
    LCD.print("LEFT");
  }
  else{
    LCD.print("RIGHT");
  }
  turn90(dir);
  delay(stepDelay);

  //Open claw
  LCD.clear();
  LCD.print("Opening claw");
  RCServo2.write(clawOpen);
  delay(stepDelay);

  //Reach out to animal in increments
  LCD.clear();
  LCD.print("Moving to animal");

  for (int i = 0; i < numExtendSteps; i++) {
    RCServo0.write(extendAngleTheta[i]);
    RCServo1.write(extendAnglePhi[i]);
    double QRDVal = analogRead(clawQRDPin);
  //  Serial.println("QRDVal");
  //  Serial.println(QRDVal);
    delay(1.5 * stepDelay);
    // Serial.println(i);
    if (QRDVal < clawQRDThreshold || i > 6) {
      dropPickupIndex = i;
      break;
    }
  }

  //Close claw
  LCD.clear();
  LCD.print("Closing claw");
  RCServo2.write(clawClose);
  delay(stepDelay);

  //Lift animal
  LCD.clear();
  LCD.print("Lifting animal");
  for (int i = 0; i < 2; i++) {
    RCServo0.write(raiseAngleTheta[i]);
    RCServo1.write(raiseAnglePhi[i]);
  }

  delay(stepDelay);

  //Rotate back to new home (right side of robot)
  LCD.clear();
  LCD.print("Going home");
  turnBack90(dir);
}

void dropoffAnimal(bool dir) {
  int stepDelay = 250;

  //Rotate arm to dropoff side
  LCD.clear();
  LCD.print("Turning to dropoff");
  if(dir){
    motor.speed(motorLeft, -50);
    motor.speed(motorRight, 50);
  }
  else{
    motor.speed(motorLeft, 50);
    motor.speed(motorRight, -50);
  }
   delay(200);  
   stopMotors();

  delay(stepDelay);
  RCServo0.write(8.61);
  RCServo1.write(149.41);
  delay(stepDelay);

  //Open claw
  LCD.clear();
  LCD.print("Opening claw");
  RCServo2.write(clawOpen);
  delay(stepDelay);

  //Go home
  RCServo0.write(thetaHome);
  RCServo1.write(phiHome);

  turnBack90(dir);
}
