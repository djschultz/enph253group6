#include <phys253.h>
#include <LiquidCrystal.h>

// TINAH INPUTS
int motorLeft = 3;
int motorRight = 0;

const int QRDIntersectionPinLeft = 5;
const int QRDIntersectionPinRight = 1;
const int QRDIntersectionPinMiddle = 3;

const int QRDTapePinLeft = 4;
const int QRDTapePinRight = 2;

const int multiplexSwitchPin = 8;

// CLARIFYING CONSTANTS
int defaultSpeed = 75;
int count = 0;
int numLoops = 500;
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

void setup() {
#include <phys253setup.txt>

  // initialize serial monitor
  Serial.begin(9600);

  // default direction forwards
  pinMode(multiplexSwitchPin, OUTPUT);
  multiplexDir = HIGH;
  digitalWrite(multiplexSwitchPin, multiplexDir);
  lastDir = LEFT;
}

void loop() {
  //change direction if startbutton is pressed
  if (startbutton()) {
    multiplexDir = !multiplexDir;
    digitalWrite(multiplexSwitchPin, multiplexDir);
  }

  //START PID
  timeCurr = millis();
  propGain = knob(6);
  derivGain = knob(7);

  // QRDs are true when there is tape below them
  QRDTapeRight = digitalRead(QRDTapePinRight);
  QRDTapeLeft = digitalRead(QRDTapePinLeft);
  QRDIntersectionLeft = digitalRead(QRDIntersectionPinLeft);
  QRDIntersectionRight = digitalRead(QRDIntersectionPinRight);

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
  } else if (QRDTapeRight && !QRDTapeLeft) {
    QRDError = -1;
  } else if (!QRDTapeRight && QRDTapeLeft) {
    QRDError = 1;
  } else {
    if (QRDErrorPrev > 0) {
      QRDError = 2;
    } else {
      QRDError = -2;
    }
  }

  // if the robot is off the tape, tape follow
  // calculate the derivation & propotional gains
  derivTerm = (QRDError - QRDErrorPrevDiff) / (timeCurr - QRDErrorPrevDiffTime);
  propTerm = propGain * QRDError;
  deltaTime = (timeCurr - timePrev) / 1000.0;

  newSpeedLeft = defaultSpeed - propTerm - derivTerm;
  newSpeedRight = defaultSpeed + propTerm + derivTerm;

  QRDErrorPrev = QRDError;
  timePrev = timeCurr;

  // do not exceed the maximum speeds for the motor
  if (newSpeedLeft > 255) {
    newSpeedLeft = 255;
  }
  else if (newSpeedLeft < -255) {
    newSpeedLeft = -255;
  }

  if (newSpeedRight > 255) {
    newSpeedRight = 255;
  }
  else if (newSpeedRight < -255) {
    newSpeedRight = -255;
  }

  motor.speed(motorLeft, newSpeedLeft);
  motor.speed(motorRight, newSpeedRight);


  // if the robot is on the tape, check for intersections
  if (QRDError == 0) {
    if (lastDir == LEFT && QRDIntersectionRight) {
      LCD.setCursor(3, 1);
      LCD.print("Turn Right ");
      motor.speed(motorLeft, 150);
      motor.speed(motorRight, 0);
      lastDir = RIGHT;
      delay(500);
      QRDError = 5;
    }

    else if (lastDir == RIGHT && QRDIntersectionLeft) {
      LCD.setCursor(3, 1);
      LCD.print("Turn Left ");
      motor.speed(motorRight, 150);
      motor.speed(motorLeft, 0);
      lastDir = LEFT;
      delay(500);
      QRDError = 5;
    }
  }

  // this determines when the print the output data to the serial port.
  LCD.clear();
  LCD.print("DG: ");
  LCD.print(derivGain);
  LCD.print(" ");
  LCD.print("PG: ");
  LCD.print(propGain);

  if (QRDIntersectionLeft) {
    LCD.setCursor(0, 1);
    LCD.print("L");
  }

  if (QRDIntersectionRight) {
    LCD.setCursor(1, 1);
    LCD.print("R");
  }

  if (count % numLoops == 0) {
    Serial.println("Multiplexing Direction: ");
    Serial.println(multiplexDir);
    Serial.println("QRD Tape Right: ");
    Serial.println(QRDTapeRight);
    Serial.println("QRD Tape Left: ");
    Serial.println(QRDTapeLeft);
    Serial.println("QRD Error: ");
    Serial.println(QRDError);
    Serial.println("Speed Right Motor: ");
    Serial.println(newSpeedRight);
    Serial.println("Speed Left Motor: ");
    Serial.println(newSpeedLeft);
    Serial.println("Prop Term");
    Serial.println(propTerm);
    Serial.println("Deriv Term");
    Serial.println(derivTerm);

    count = 0;
  }

  count++;
}
