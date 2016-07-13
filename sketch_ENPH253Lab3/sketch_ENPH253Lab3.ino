#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
int  myKnob = knob(7);
float val1 = 510.0*myKnob/1023.0 - 255.0;
motor.speed(3,val1);

int yuKnob = knob(6);
float val2 = 180.0*yuKnob/1023.0;
RCServo0.write(val2);
}
