#include <phys253.h>
#include <LiquidCrystal.h>
int angle = 0;
int count = 1;

void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
int knob6Val = knob(6);
int knob7Val = knob(7);
if(count%5000 == 0){
Serial.println("Knob 6");
Serial.println(knob6Val);
Serial.println("Knob 7");
Serial.println(knob7Val);
}
RCServo0.write(180.0*knob6Val/1024.0);
RCServo1.write(180.0*knob7Val/1024.0);
count++;
}
