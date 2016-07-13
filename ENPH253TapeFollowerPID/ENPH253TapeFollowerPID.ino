#include <phys253.h>
#include <LiquidCrystal.h>

double threshold = 23.0;
int QRDError = 0;
int QRDErrorPrev = 0;
int QRDErrorPrevDiff = 0;
int QRDErrorPrevDiffTime = 0;
unsigned long timePrev = 0;
unsigned long timeCurr = 0;
int intTerm;
int derivTerm;
int intTermPrev;
int motorLeft = 2;
int motorRight = 3;
int defaultSpeed = 200;
int propGain;
int newSpeedLeft;
int newSpeedRight;
int count = 0;
int intGain;
int derivGain;
int highLimitInt;
int lowLimitInt;
int propTerm;
double deltaTime;
int slopeCurr;
int slopePrev;

void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

double QRDRight = analogRead(A0);
double QRDLeft = analogRead(A2);

if(count%50 == 0){
timeCurr = millis();
}

if(QRDError != QRDErrorPrevDiff){
  QRDErrorPrevDiff = QRDError;
  QRDErrorPrevDiffTime = timeCurr;
}

if(QRDRight > threshold && QRDLeft > threshold){
  QRDError = 0;
} else if(QRDRight > threshold && QRDLeft < threshold){
  QRDError = -1;
} else if(QRDRight < threshold && QRDLeft > threshold){
  QRDError = 1;
} else {
  if(QRDErrorPrev > 0){
  QRDError = 5;
  } else{
   QRDError = -5;
   }
}

propGain = knob(6);
derivGain = knob(7);

/*
if(startbutton()){
  threshold = knob(6);
  intGain = knob(7);
} else {
  propGain = knob(6);
  derivGain = knob(7);
}*/

slopeCurr = (QRDError - QRDErrorPrevDiff)/(timeCurr - QRDErrorPrevDiffTime);
propTerm = propGain*QRDError;
deltaTime = (timeCurr - timePrev)/1000.0;
intTerm = intTermPrev + intGain*deltaTime*QRDError;
derivTerm = slopeCurr;

if(intTerm > highLimitInt){
  intTerm = 100;
} 

if(intTerm < lowLimitInt){
  intTerm = -100;
}

newSpeedLeft = defaultSpeed - propTerm - intTerm - derivTerm;
newSpeedRight = defaultSpeed + propTerm + intTerm + derivTerm;

if(newSpeedLeft > 700){
  newSpeedLeft = 600;
}

if(newSpeedRight > 700){
  newSpeedRight = 600;
}

motor.speed(motorLeft, newSpeedLeft);
motor.speed(motorRight, newSpeedRight);

QRDErrorPrev = QRDError;
timePrev = timeCurr;
slopePrev = slopeCurr;

if(count%5000 == 0){
LCD.clear();
LCD.print("DG: ");
LCD.print(derivGain);
LCD.print("PG: ");
LCD.print(propGain);

Serial.println("QRD Right: ");
Serial.println(QRDRight);
Serial.println("QRD Left: ");
Serial.println(QRDLeft);
Serial.println("QRD Error: ");
Serial.println(QRDError);
Serial.println("Speed Right Motor: ");
Serial.println(newSpeedRight);
Serial.println("Speed Left Motor: ");
Serial.println(newSpeedLeft);
Serial.println("Threshold: ");
Serial.println(threshold);
Serial.println("Proportional Gain: ");
Serial.println(propGain);
Serial.println("Integral Gain: ");
Serial.println(intGain);
Serial.println("Derivative Gain: ");
Serial.println(derivGain);
Serial.println("Current Time: ");
Serial.println(timeCurr);
Serial.println("Previous Time: ");
Serial.println(timePrev);
Serial.println("Delta Time: ");
Serial.println(deltaTime);

} 

count++;

}
