#include <Servo.h>  

//Pusher Variables
Servo pusherServo;

//const int zeroButtonPin = 5;
const int shootButtonPin = 8;
const int pusherPin = 11;

//int zeroButtonState = 0;
int shootButtonState = 0;

//Boundary angle positions
int zeroPosition = 100;
int boundary1 = 30;
int boundary2 = 115;

boolean shootState=true;

unsigned long startTime;

//Aiming Variables
int pointPin = 10;   
int pointPotPin = A2;  
int tiltPin =  12;   
int tiltPotPin = A3;  

int pointServoDegree=90;
int tiltServoDegree=90;

Servo pointServo;  
Servo tiltServo;    

int pointPotValue;         
int pointServoPosition;    
int tiltPotValue;         
int tiltServoPosition;    

int pointIncrementSize=1;
int tiltIncrementSize=1;

int pointMaxAngle=180;
int pointMinAngle=0;
int tiltMaxAngle=130;
int tiltMinAngle=116;

void setup()   
{
  Serial.begin(9600);
  pointServo.attach(pointPin);   
  tiltServo.attach(tiltPin);   
  pusherServo.attach(pusherPin);
  pinMode(shootButtonPin, INPUT_PULLUP);
}

void loop()  
{ 
  pointPotValue  = analogRead(pointPotPin); 
  tiltPotValue  = analogRead(tiltPotPin); 
  shootButtonState = !digitalRead(shootButtonPin);

  Serial.print(pointPotValue); Serial.print(",");
  Serial.print(tiltPotValue); Serial.print(",");
  Serial.print(shootButtonState); Serial.print(",");

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

  if (shootButtonState == HIGH && (millis()-startTime>700)) {
    startTime=millis();
    pusherServo.write(boundary2);
    delay(400);
    pusherServo.write(boundary1);
    delay(300);
  }

  pointServo.write(pointServoDegree); 
  tiltServo.write(tiltServoDegree);         
  delay(20);    
}
