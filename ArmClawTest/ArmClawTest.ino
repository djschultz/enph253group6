#include <phys253.h>
#include <LiquidCrystal.h>
int count;
const double openClaw = 155;
const double closedClaw = 184;

void setup() {
#include <phys253setup.txt>

  // initialize serial monitor
  Serial.begin(9600);
  count = 0;;
}

void loop() {
  double pVolt = analogRead(0);
  LCD.clear();
  LCD.print(pVolt);


  double servoTheta = 180.0 * knob(6) / 1024.0;
  double servoPhi = 180.0 * knob(7) / 1024.0;

  if (count % 1000 == 0) {
    Serial.println("Servo Theta");
    Serial.println(servoTheta);
    Serial.println("Servo Phi");
    Serial.println(servoPhi);
    double jointTheta = servoTheta / 2.0 + 120.0;
    double jointPhi = -(servoPhi) / 2.0 + 90;
    Serial.println("Joint Theta");
    Serial.println(jointTheta);
    Serial.println("Joint Phi");
    Serial.println(jointPhi);
  }


  RCServo0.write(servoTheta);
  RCServo1.write(servoPhi);


  count++;


  if (startbutton()) {
    motor.speed(1, 200);
  }


  else if (stopbutton()) {
    motor.speed(1, -200);
  }

  else {
    motor.speed(1, 0);
  }
}
