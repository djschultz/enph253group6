#include <phys253.h>
#include <LiquidCrystal.h>
int pinNum;
bool startButton;
void setup() {
  // put your setup code here, to run once:
#include <phys253setup.txt>
Serial.begin(9600);
pinNum = 8;
pinMode(pinNum, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//startButton = startbutton();
//Serial.print(startButton);
Serial.print("HI CALVIN");

//if(startButton){
//  digitalWrite(pinNum, startButton);
//}

}
