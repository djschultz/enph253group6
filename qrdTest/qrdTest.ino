#include <phys253.h>
#include <LiquidCrystal.h>

int QRDVal;

void setup() {
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  QRDVal = analogRead(5);
  Serial.println(QRDVal);
  delay(500);
}
