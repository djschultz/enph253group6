#include <phys253.h>          
#include <LiquidCrystal.h>    
 
void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
}

void loop() {
  LCD.clear();
  int angle = knob(6) - 90;
  if(angle > 90)
  angle = 90;
  // put your main code here, to run repeatedly:
analogWrite(36,map(angle,-90,90,0,255));
LCD.print(angle);

}
