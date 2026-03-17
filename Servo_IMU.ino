#include <Wire.h>
#include <math.h>
#include <Servo.h>

Servo myservo;

// Smoothing factor: 0.0 (very smooth) to 1.0 (no smoothing)
// Start with 0.2–0.4 for good balance
const float SMOOTHING_FACTOR = 0.3;
float smoothed_ax = 0.0;  // Holds the smoothed value

// ------------------- I2C Address & Registers -------------------
const uint8_t FXOS_ADDR = 0x1E;

#define WHO_AM_I      0x0D
#define CTRL_REG1     0x2A //Accelerometer control
#define M_CTRL_REG1   0x5B //Control for magnetic sensors functions
#define M_CTRL_REG2   0x5C //Control for magnetic sensors functions
#define OUT_X_MSB     0x01

#define ACC_SCALE 4096.0  // 2g mode, 14-bit
#define MAG_SCALE 0.1     // uT per LSB

// ------------------- Wire Helper Functions -------------------
void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(FXOS_ADDR); //Begins transmission
  Wire.write(reg); //Address for transmisions
  Wire.write(val); //Uses the pointer then adds a hex value, or command to the register
  Wire.endTransmission(); //Finalises the sequence and releases the bus for data/commands to go through
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(FXOS_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(FXOS_ADDR, (uint8_t)1);
  return Wire.read();
}

void readAccelMag(float* ax, float* ay, float* az, float* mx, float* my, float* mz) {
  //Setting register pointer
  Wire.beginTransmission(FXOS_ADDR);
  Wire.write(OUT_X_MSB); //start register 0x01
  Wire.endTransmission(false); //Holds the I2C bus
  Wire.requestFrom(FXOS_ADDR, (uint8_t)12); 
  //Requests 12 bytes since we have 6 axis 
  //Each axis has 2 bytes?
  //2 byte==16bit
  //Not sure why this is, maybe the sensor has 8 bit registers
  //Why do registers operate in this way,
  //How do we get the different address then? 

  //Combining Most and Least significant byte packages to store as one 16 bit int
  //Automatically increments registers
  //FIFO ORDER
  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  int16_t raw_mx = (Wire.read() << 8) | Wire.read();
  int16_t raw_my = (Wire.read() << 8) | Wire.read();
  int16_t raw_mz = (Wire.read() << 8) | Wire.read();

  //Converts values to realistic results
  *ax = raw_ax / ACC_SCALE;
  *ay = raw_ay / ACC_SCALE;
  *az = raw_az / ACC_SCALE;
  *mx = raw_mx * MAG_SCALE;
  *my = raw_my * MAG_SCALE;
  *mz = raw_mz * MAG_SCALE;
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600); //For printing
  Wire.begin(); //Start communication
  delay(100);

  myservo.attach(9);

  uint8_t id = readRegister(WHO_AM_I); 
  //Reads a value from a register @ WHO_AM_I
  //Returns a hex value that can be used to determine if the correct device is connected
  if(id != 0xC7) {
    Serial.println("FXOS8700 not detected!");
    while(1);
  }

  // Standby
  writeRegister(CTRL_REG1, 0x00); //Data Ready Status

  // Enable magnetometer, hybrid mode
  //What does this mean?
  //A magnetometer is a device that measures magnetic dipole moment, focus on moment
  //In electrical engineering a transient response is a change from an equilibrium state
  writeRegister(M_CTRL_REG1, 0x1F); //Transient event threshold
  writeRegister(M_CTRL_REG2, 0x20); //Transient debounce counter

  // Active mode
  // Not sure how to read the commands, and understand what is happening here
  writeRegister(CTRL_REG1, 0x0D);
}

// ------------------- Loop -------------------
void loop() {
  float ax, ay, az, mx, my, mz;
  readAccelMag(&ax, &ay, &az, &mx, &my, &mz);

  // Clamp raw value to ±4 range
  ax = constrain(ax, -4.0, 4.0);

  // Apply exponential smoothing: smoothed = α·current + (1-α)·previous
  smoothed_ax = SMOOTHING_FACTOR * ax + (1.0 - SMOOTHING_FACTOR) * smoothed_ax;

  // Map smoothed value to servo angle
  int angle = (int)mapf(smoothed_ax, -4.0, 4.0, 0, 180);
  myservo.write(angle);

  // Optional: debug output
  Serial.print(ax, 3); Serial.print(",");
  Serial.println(smoothed_ax, 3);

  delay(50); // ~20 Hz update rate
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
