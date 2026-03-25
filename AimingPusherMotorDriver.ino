#include <Servo.h>  
#include <Wire.h>
#include <math.h>

/*
shoot button (joystick) pin 8

Joystick Analog pins
point servo pin 12
tilt servo pin 10
pusherPin 11
*/

//IMU Code
const float SMOOTHING_FACTOR = 0.3;
float smoothed_roll = 0.0;  // Holds the smoothed value
float smoothed_pitch = 0.0;

const uint8_t FXOS_ADDR = 0x1E;
#define WHO_AM_I      0x0D
#define CTRL_REG1     0x2A
#define M_CTRL_REG1   0x5B
#define M_CTRL_REG2   0x5C
#define OUT_X_MSB     0x01

#define ACC_SCALE 4096.0  // 2g mode, 14-bit
#define MAG_SCALE 0.1     // uT per LSB

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
void readAccelMag(float* ax, float* ay, float* az, float* mx, float* my, float* mz) {
  Wire.beginTransmission(FXOS_ADDR);
  Wire.write(OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(FXOS_ADDR, (uint8_t)12);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  int16_t raw_mx = (Wire.read() << 8) | Wire.read();
  int16_t raw_my = (Wire.read() << 8) | Wire.read();
  int16_t raw_mz = (Wire.read() << 8) | Wire.read();

  *ax = raw_ax / ACC_SCALE;
  *ay = raw_ay / ACC_SCALE;
  *az = raw_az / ACC_SCALE;
  *mx = raw_mx * MAG_SCALE;
  *my = raw_my * MAG_SCALE;
  *mz = raw_mz * MAG_SCALE;
}

//Pusher Variables
Servo pusherServo;

const int shootButtonPin = 8;
const int pusherPin = 11;

int shootButtonState = 0;

//Pusher boundary angle positions
int boundary1 = 30;
int boundary2 = 115;

unsigned long startTime;

//Aiming Variables
Servo pointServo;  
Servo tiltServo;    

int pointPin = 10;   
int pointPotPin = A2;  
int tiltPin =  12;   
int tiltPotPin = A3;  

int pointServoDegree=90;
int tiltServoDegree=90;

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

  Wire.begin();
  delay(100);

  uint8_t id = readRegister(WHO_AM_I);
  if(id != 0xC7) {
    Serial.println("FXOS8700 not detected!");
    while(1);
  }

  // Standby
  writeRegister(CTRL_REG1, 0x00);

  // Enable magnetometer, hybrid mode
  writeRegister(M_CTRL_REG1, 0x1F);
  writeRegister(M_CTRL_REG2, 0x20);

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
  shootButtonState = !digitalRead(shootButtonPin);

  Serial.print(pointPotValue); Serial.print(",");
  Serial.print(tiltPotValue); Serial.print(",");
  Serial.print(shootButtonState); Serial.print(",");
  //Increment servo degree according to Joystick value
  if (pointPotValue>1000) {
    pointServoDegree+=pointIncrementSize;
  } else if (pointPotValue<50) {
    pointServoDegree-=pointIncrementSize;
  } else {
    pointServoDegree=pointServoDegree;
  }
  //Ensure degree remains within boundaries
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

  //If the shoot button pressed and pusher action is complete
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
  float ax, ay, az, mx, my, mz;
  readAccelMag(&ax, &ay, &az, &mx, &my, &mz);

  // ---- Compute Roll & Pitch ----
  float roll  = atan2(ay, az);
  float pitch = atan(-ax / sqrt(ay*ay + az*az));

  // ---- Tilt-compensated Yaw ----
  float Xh = mx * cos(pitch) + mz * sin(pitch);
  float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
  float yaw = atan2(-Yh, Xh);

  // Convert radians to degrees
  roll  *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  yaw   *= 180.0 / M_PI;
  if(yaw < 0) yaw += 360.0;

  roll = constrain(roll, -40, 30);
  pitch = constrain(pitch, -40, 40);

  //smoothed_roll = SMOOTHING_FACTOR * roll + (1.0 - SMOOTHING_FACTOR) * smoothed_roll;
  smoothed_pitch = SMOOTHING_FACTOR * pitch + (1.0 - SMOOTHING_FACTOR) * smoothed_pitch;
  
  //Turn Moving motors according to IMU Deadzone using motordriver
  if (smoothed_pitch>39) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, HIGH);
    digitalWrite(motor_pin3, HIGH);
    digitalWrite(motor_pin4, LOW);
    Serial.print(smoothed_pitch);
    Serial.println("Right");
  } else if (smoothed_pitch<-39) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);
    digitalWrite(motor_pin3, HIGH);
    digitalWrite(motor_pin4, HIGH);
    Serial.print(smoothed_pitch);
    Serial.println("Left");
  } else {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin3, LOW);
    Serial.println(smoothed_pitch);
  }            
}
