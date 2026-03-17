// letsarduino.com
// [Project 11] - 2 Servos Using a Joystick 
//  (thumbstick) + Arduino

#include <Servo.h>  

int pointPin = 9;   
int pointPotPin = A2;  
int tiltPin =  9;   
int tiltPotPin = A3;  

int servo1Degree=90;
int servo2Degree=90;

Servo pointServo;  
Servo tiltServo;    

int pointPotValue;         
int pointServoPosition;    
int tiltPotValue;         
int tiltServoPosition;    

void setup()   
{
  Serial.begin(9600);
  //pointServo.attach(pointPin);   
  tiltServo.attach(tiltPin);         
}

void loop()  
{
  pointPotValue  = analogRead(pointPotPin); 
  tiltPotValue  = analogRead(tiltPotPin);  
  Serial.print(pointPotValue); Serial.print(",");
  Serial.print(tiltPotValue); Serial.print(",");

  if (pointPotValue>1000) {
    servo1Degree+=1;
  } else if (pointPotValue<50) {
    servo1Degree-=1;
  } else {
    servo1Degree=servo1Degree;
  }
  if (servo1Degree<0) {
    servo1Degree=0;
  } else if (servo1Degree>180) {
    servo1Degree=180;
  }

  if (tiltPotValue>1000) {
    servo2Degree+=1;
  } else if (tiltPotValue<50) {
    servo2Degree-=1;
  } else {
    servo2Degree=servo2Degree;
  }
  if (servo2Degree<0) {
    servo2Degree=0;
  } else if (servo2Degree>180) {
    servo2Degree=180;
  }
  //pointServo.write(servoDegree);      
  Serial.print(servo1Degree); Serial.print(",");
  Serial.println(servo2Degree);
  tiltServo.write(servo2Degree);       
  delay(20);    
}
