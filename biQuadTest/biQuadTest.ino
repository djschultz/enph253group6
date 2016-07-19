#include <phys253.h>
#include <LiquidCrystal.h>

const int biquadPin = 0;
int count = 0;

void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

double biquadVal = analogRead(biquadPin);
if(count%2000 == 0){
  LCD.clear();
  LCD.print(5.0*biquadVal/1024.0);
  Serial.println(5.0*biquadVal/1024.0);
}

count++;

}
