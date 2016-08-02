#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
  Serial.begin(9600);
}

void loop() {
  //arbitrary speeds to test motors
  motor.speed(0, 50);
  motor.speed(3, 50);

  //serial print sensor values
  int irLeft = analogRead(2);
  int irRight = analogRead(0);
  int irFront = analogRead(1);
  int irBeacon = analogRead(3);

  int qrdIntersectionLeft = digitalRead(1);
  int qrdLeft = digitalRead(0);
  int qrdRight = digitalRead(2);
  int qrdIntersectionRight = digitalRead(3);

  int qrdProximity = analogRead(5);

  LCD.clear();

  Serial.print("qsdLeft: ");
  Serial.print(irLeft);
  Serial.print(" qsdRight: " );
  Serial.print(irRight);
  Serial.print(" qsdFront: ");
  Serial.print(irFront);
  Serial.print(" qsdBeacon: ");
  Serial.print(irBeacon);

  Serial.print(" qrdIntLeft: ");
  Serial.print(qrdIntersectionLeft);
  Serial.print(" qrdLeft: ");
  Serial.print(qrdLeft);
  Serial.print(" qrdRight: ");
  Serial.print(qrdRight);
  Serial.print(" qrdIntRight: ");
  Serial.print(qrdIntersectionRight);

  Serial.print(" qrdProx: ");
  Serial.print(qrdProximity);

  Serial.print(" Collisions: ");
  Serial.print(digitalRead(8));
  Serial.print(digitalRead(9));
  Serial.print(digitalRead(10));
  Serial.print(digitalRead(11));
  Serial.print(digitalRead(12));
  Serial.print(digitalRead(13));
  Serial.print(digitalRead(14));
  Serial.print(digitalRead(15));

  //knob controlled servos to test servos
  double servoTheta = 180.0 * knob(6) / 1024.0;
  double servoPhi = 180.0 * knob(7) / 1024.0;

  RCServo0.write(servoTheta);
  RCServo1.write(servoPhi);

  Serial.print(" Theta: ");
  Serial.print(servoTheta);
  
  Serial.print(" Phi: ");
  Serial.println(servoPhi);

  delay(250);
}
