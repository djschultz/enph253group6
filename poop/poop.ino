#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.speed(0,50);
  motor.speed(3,50);
}
