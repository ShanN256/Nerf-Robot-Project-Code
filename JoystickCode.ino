#include <Servo.h>  

int pointPin = 10;   
int pointPotPin = A2;  
int tiltPin =  9;   
int tiltPotPin = A3;  

int pointServoDegree=90;
int tiltServoDegree=90;

Servo pointServo;  
Servo tiltServo;    

int pointPotValue;         
int pointServoPosition;    
int tiltPotValue;         
int tiltServoPosition;    

int pointIncrementSize=3;
int tiltIncrementSize=1;

int pointMaxAngle=180;
int pointMinAngle=0;
int tiltMaxAngle=180;
int tiltMinAngle=0;

int buttonPin=12;
int buttonState = 0;

void setup()   
{
  Serial.begin(9600);
  pointServo.attach(pointPin);   
  tiltServo.attach(tiltPin);   

  pinMode(buttonPin, INPUT_PULLUP);
}

void loop()  
{ 
  pointPotValue  = analogRead(pointPotPin); 
  tiltPotValue  = analogRead(tiltPotPin); 
  buttonState = digitalRead(buttonPin); 
  Serial.print(pointPotValue); Serial.print(",");
  Serial.print(tiltPotValue); Serial.print(",");
  Serial.print(buttonState); Serial.print(",");

  if (pointPotValue>1000) {
    pointServoDegree+=pointIncrementSize;
  } else if (pointPotValue<50) {
    pointServoDegree-=pointIncrementSize;
  } else {
    pointServoDegree=pointServoDegree;
  }
  if (pointServoDegree<pointMinAngle) {
    pointServoDegree=pointMinAngle;
  } else if (pointServoDegree>pointMaxAngle) {
    pointServoDegree=pointMaxAngle;
  }

  if (tiltPotValue>1000) {
    tiltServoDegree+=tiltIncrementSize;
  } else if (tiltPotValue<50) {
    tiltServoDegree-=tiltIncrementSize;
  } else {
    tiltServoDegree=tiltServoDegree;
  }
  if (tiltServoDegree<tiltMinAngle) {
    tiltServoDegree=tiltMinAngle;
  } else if (tiltServoDegree>tiltMaxAngle) {
    tiltServoDegree=tiltMaxAngle;
  }     
  Serial.print(pointServoDegree); Serial.print(",");
  Serial.println(tiltServoDegree);

  pointServo.write(pointServoDegree); 
  tiltServo.write(tiltServoDegree);         
  delay(20);    
}
