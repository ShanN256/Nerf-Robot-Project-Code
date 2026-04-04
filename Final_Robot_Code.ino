#include <Servo.h>  //Simplifies PWM control of Servo motors
#include <Wire.h>  //Enables I2C communication between peripherals with read and write functions
#include <math.h>  //Additional Mathematical functions including trigonometric functions

/*
Pin Layout:
Joystick shoot button  pin 8
Joystick Analog pins A2, A3

point servo pin 10
tilt servo pin 12
pusherPin 11
*/

const float SMOOTHING_FACTOR = 0.3; // Weighting for first order low pass filter. 
//Takes weighted average between new incomming value and previous smoothed value

float smoothed_roll = 0.0; //Holds the smoothed value

//IMU Register addresses
const uint8_t FXOS_ADDR = 0x1E;
#define WHO_AM_I      0x0D
#define CTRL_REG1     0x2A
#define OUT_X_MSB     0x01

#define ACC_SCALE 4096.0  // 2g mode, 14-bit

//Enable and Direction Pins for Motordriver (moving axis)
#define motor_pin1 2 //E1
#define motor_pin2 3 //D1
#define motor_pin3 4 //E2
#define motor_pin4 5 //D2

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(FXOS_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(FXOS_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(FXOS_ADDR, (uint8_t)1);
  return Wire.read();
}

//Read values
void readAccel(float* ax, float* ay, float* az) {
  Wire.beginTransmission(FXOS_ADDR);
  Wire.write(OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(FXOS_ADDR, (uint8_t)6);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();

  *ax = raw_ax / ACC_SCALE;
  *ay = raw_ay / ACC_SCALE;
  *az = raw_az / ACC_SCALE;
}

//Pusher Variables
Servo pusherServo;

const int shootButtonPin = 8; //Input from Joystick
const int pusherPin = 11; //Servo Output

int shootButtonState = 0; //Stores boolean value of Joystick button as an integer

//Pusher boundary angle positions
const int boundary1 = 30;
const int boundary2 = 115;

unsigned long startTime; //holds timestamp of when a pusher servo action has started
//Used to determine if pusher action has been completed before being able to push again

//Aiming Variables
Servo pointServo;  
Servo tiltServo;    

const int pointPin = 10;  //Servo Output
const int pointPotPin = A2;  //Input from Joystick
const int tiltPin =  12;  //Servo Output
const int tiltPotPin = A3;  //Input from Joystick

//Variable that store the servo angle. 
int pointServoDegree=90;
int tiltServoDegree=90;

//Variable that stores Joystick Analog values
int pointPotValue;         
int tiltPotValue;         

//Determines servo sensitivity to Joystick control
const int pointIncrementSize=1;
const int tiltIncrementSize=1;

const int pointMaxAngle=180;
const int pointMinAngle=0;
const int tiltMaxAngle=130;
const int tiltMinAngle=116;

void setup()   
{
  Serial.begin(9600); //All values used in the program are transmitted to the Serial Monitor for easy debugging

  pointServo.attach(pointPin);   
  tiltServo.attach(tiltPin);   
  pusherServo.attach(pusherPin);

  pinMode(shootButtonPin, INPUT_PULLUP); //Using the Arduinos internal resistors to ensure stable logic levels

  Wire.begin();
  delay(100);

  uint8_t id = readRegister(WHO_AM_I);
  if(id != 0xC7) {
    Serial.println("FXOS8700 not detected!");
    while(1);
  }

  // Standby
  writeRegister(CTRL_REG1, 0x00);

  // Active mode
  writeRegister(CTRL_REG1, 0x0D);

  //Driving Motors
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(motor_pin3, OUTPUT);
  pinMode(motor_pin4, OUTPUT);

  Serial.println("Setup");
}

void loop()  
{ 
  //Reading data from Joystick
  pointPotValue  = analogRead(pointPotPin); 
  tiltPotValue  = analogRead(tiltPotPin); 
  shootButtonState = !digitalRead(shootButtonPin); //shoot button pin reads 0 (LOW) when button is pressed
  //This is inverted to make logic more intuitive later

  Serial.print(pointPotValue); Serial.print(",");
  Serial.print(tiltPotValue); Serial.print(",");
  Serial.print(shootButtonState); Serial.print(",");

  /*
  Joystick value ranges from 0 - 1023
  When fully turned, the Joystick outputs values very close to the extremeties, 0 and 1023
  The Joystick values rest at around 500
  We implement a deadzone that accept values greater than 1000 and less than 50
  This reduces the chance of accidental turning
  */

  //Increment point servo degree according to Joystick value
  if (pointPotValue>1000) {
    pointServoDegree+=pointIncrementSize;
  } else if (pointPotValue<50) {
    pointServoDegree-=pointIncrementSize;
  }
  //Ensure point servo degree remains within boundaries
  pointServoDegree = constrain(pointServoDegree, pointMinAngle, pointMaxAngle);

  //Increment tilt servo degree according to Joystick value
  if (tiltPotValue>1000) {
    tiltServoDegree+=tiltIncrementSize;
  } else if (tiltPotValue<50) {
    tiltServoDegree-=tiltIncrementSize;
  }
  //Ensure tilt servo degree remains within boundaries
  tiltServoDegree = constrain(tiltServoDegree, tiltMinAngle, tiltMaxAngle);    

  Serial.print(pointServoDegree); Serial.print(",");
  Serial.print(tiltServoDegree); Serial.print(",");

  //If the shoot button pressed and previous pusher action is complete
  if (shootButtonState == HIGH && (millis()-startTime>700)) {
    startTime=millis();
    pusherServo.write(boundary2);
    delay(400);
    pusherServo.write(boundary1);
    delay(300);
  }

  //Update Aiming servo angles
  pointServo.write(pointServoDegree); 
  tiltServo.write(tiltServoDegree); 

  //IMU Section
  float ax, ay, az;
  readAccel(&ax, &ay, &az);

  //Compute Roll
  float roll = atan(-ax / sqrt(ay*ay + az*az));

  // Convert radians to degrees
  roll *= 180.0 / M_PI;

  roll = constrain(roll, -40, 40); //Constrain the roll value between -40 and 40

  //Takes weighted average between new incomming value and previous smoothed value
  smoothed_roll = SMOOTHING_FACTOR * roll + (1.0 - SMOOTHING_FACTOR) * smoothed_roll;
  
  //Turn Moving motors according to IMU Deadzone using motordriver
  if (smoothed_roll>39) { //If rolled Right
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, HIGH);
    digitalWrite(motor_pin3, HIGH);
    digitalWrite(motor_pin4, LOW);
    Serial.print(smoothed_roll);
    Serial.println("Right");
  } else if (smoothed_roll<-39) { //If rolled Left
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);
    digitalWrite(motor_pin3, HIGH);
    digitalWrite(motor_pin4, HIGH);
    Serial.print(smoothed_roll);
    Serial.println("Left");
  } else { //Disable Motors
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin3, LOW);
    Serial.println(smoothed_roll);
  }            
}
