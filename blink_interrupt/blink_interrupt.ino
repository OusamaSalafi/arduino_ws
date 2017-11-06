#include <PinChangeInt.h>

const byte pinILS1 = A1;
const byte pinILS2 = A2;
const byte pinILS3 = A3;

int state1 = 1;
int state3 = 1;
int state2 = 1;

void interruptILS1()
{
  state1=!state1;
  digitalWrite(13,state1);
}
 
void interruptILS2()
{
  state2=!state2;
  digitalWrite(12,state2);
}
 
void interruptILS3()
{
  state3=!state3;
  digitalWrite(11,state3);
}

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  
  digitalWrite(13,state1);
  digitalWrite(12,state2);
  digitalWrite(11,state3);
  
  //pinMode(2, INPUT_PULLUP);
  //pinMode(pinILS2, INPUT_PULLUP);
  //pinMode(pinILS3, INPUT_PULLUP);
   
  PCintPort::attachInterrupt(digitalPinToInterrupt(pinILS1), interruptILS1, RISING);
  PCintPort::attachInterrupt(pinILS2, interruptILS2, CHANGE);
  PCintPort::attachInterrupt(pinILS3, interruptILS3, CHANGE);
}
 
void loop()
{

}
