#include <Wire.h>
#include <math.h>

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

void setup() {
  Serial.begin(9600);
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

void loop() {
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
  
  if (smoothed_pitch>39) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);
    digitalWrite(motor_pin3, HIGH);
    digitalWrite(motor_pin4, HIGH);
    Serial.print(smoothed_pitch);
    Serial.println("Right");
  } else if (smoothed_pitch<-39) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, HIGH);
    digitalWrite(motor_pin3, HIGH);
    digitalWrite(motor_pin4, LOW);
    Serial.print(smoothed_pitch);
    Serial.println("Left");
  } else {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin3, LOW);
    Serial.println(smoothed_pitch);
  }
}
