#include <Servo.h>

int servoPin = 9; //not connected yet   
int buttonPin = 2;

Servo pusherServo; 

int zeroPosition = 0;
int loadPosition = 90; // limit can be applied 

bool lastButtonState = HIGH; 

void setup()   
{
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pusherServo.attach(servoPin);    
  //pusher starts fully extended
  pusherServo.write(zeroPosition); 
}

void loop()  
{
  bool buttonState = digitalRead(buttonPin);
  // high - low means button has been pressed
  if(lastButtonState == HIGH && buttonState == LOW){
    loadAndFire(); // function to fire bullet 
  }
  // assigns new last button state 
  lastButtonState = buttonState;
  delay(20); // debounce
}

  void loadAndFire(){
    pusherServo.write(loadPosition); // moves servo to loading position when button is pressed
    delay(1000); // time for bullet to drop 
    pusherServo.write(zeroPosition); // moves back to fully extended 
    delay(300); 
  }  


