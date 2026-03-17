#include <Wire.h>
#include <math.h>

// ------------------- I2C Address & Registers -------------------
const uint8_t FXOS_ADDR = 0x1E;

#define WHO_AM_I      0x0D
#define CTRL_REG1     0x2A
#define M_CTRL_REG1   0x5B
#define M_CTRL_REG2   0x5C
#define OUT_X_MSB     0x01

#define ACC_SCALE 4096.0  // 2g mode, 14-bit
#define MAG_SCALE 0.1     // uT per LSB

// ------------------- Wire Helper Functions -------------------
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

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  delay(100);

  uint8_t id = readRegister(WHO_AM_I);
  if(id != 0xC7) {
    Serial.println("FXOS8700 not detected!");
    while(1);
  }
  Serial.println("FXOS8700 detected!");

  // Standby
  writeRegister(CTRL_REG1, 0x00);

  // Enable magnetometer, hybrid mode
  writeRegister(M_CTRL_REG1, 0x1F);
  writeRegister(M_CTRL_REG2, 0x20);

  // Active mode
  writeRegister(CTRL_REG1, 0x0D);
  Serial.println("Finished Setting UP");
}

// ------------------- Loop -------------------
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

  // ---- Print CSV: roll,pitch,yaw,ax,ay,az,mx,my,mz ----
  Serial.print(roll, 2);  Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.print(yaw, 2); Serial.println(",");
  /*Serial.print(ax, 3);     Serial.print(",");
  Serial.print(ay, 3);     Serial.print(",");
  Serial.print(az, 3);     Serial.println(",");*/
  /*Serial.print(mx, 3);     Serial.print(",");
  Serial.print(my, 3);     Serial.print(",");
  Serial.println(mz, 3);*/

  delay(50); // 20 Hz
}
