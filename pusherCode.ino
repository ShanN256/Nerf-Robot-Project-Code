#include <Servo.h>

Servo myServo;

const int zeroButtonPin = 5;
const int shootButtonPin = 12;
const int servoPin = 2;

int zeroButtonState = 0;
int shootButtonState = 0;

//Boundary angle positions
int zeroPosition = 100;
int boundary1 = 60;
int boundary2 = 140;

boolean shootState=true;

unsigned long startTime;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  pinMode(shootButtonPin, INPUT_PULLUP);
}

void loop() {
  //zeroButtonState = !digitalRead(zeroButtonPin);
  shootButtonState = !digitalRead(shootButtonPin);
  Serial.println(shootButtonState);
  if (zeroButtonState == HIGH) {
    myServo.write(zeroPosition);
  } else if (shootButtonState == HIGH && (millis()-startTime>400)) {
    startTime=millis();
    if (shootState==true) {
      myServo.write(boundary1);
      shootState=!shootState;
    } else {
      myServo.write(boundary2);
      shootState=!shootState;
    }
  }
}
