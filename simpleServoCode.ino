#include <Servo.h>

Servo myServo1;  // create servo object to control a servo
Servo myServo2;
Servo myServo3;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myServo1.attach(10);  // attaches the servo on pin 9 to the servo object
  myServo2.attach(6);
  myServo3.attach(7);
}

void loop() {
  myServo1.write(180);
  myServo2.write(180);
  myServo3.write(180);
  delay(1000);
  myServo1.write(0);
  myServo2.write(0);
  myServo3.write(0);
  delay(1000);


  /*
  for (int i=0; i<=180; i++) {
    myservo.write(i);
    delay(3);
  }
  for (int i=180; i>=0; i--) {
    myservo.write(i);
    delay(3);
  }
  */
}
