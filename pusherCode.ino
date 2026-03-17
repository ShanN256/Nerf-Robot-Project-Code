#include <Servo.h>

Servo myServo;

const int zeroButtonPin = 5;
const int shootButtonPin = 6;
const int servoPin = 2;

int zeroButtonState = 0;
int shootButtonState = 0;

//Boundary angle positions
int zeroPosition = 100;
int boundary1 = 60;
int boundary2 = 140;

boolean shootState=true;

void setup() {
  myServo.attach(servoPin);
  pinMode(zeroButtonPin, INPUT_PULLUP);
}

void loop() {
  zeroButtonState = !digitalRead(zeroButtonPin);
  shootButtonState = !digitalRead(shootButtonPin);
  if (zeroButtonState == HIGH) {
    myServo.write(zeroPosition);
  } else if (shootButtonPin == HIGH) {
    if (shootState==true) {
      myServo.write(boundary1);
      shootState=false;
    } else {
      myServo.write(boundary2);
      shootState=true;
    }
  }
}
