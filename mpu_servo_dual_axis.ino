/*
  Step 7: MPU-6050 tilt -> dual-servo (pan/tilt) tracking
  -----------------------------------------------------------
  Builds on the single-axis version. Now BOTH servos track the MPU-6050
  in real time: Servo 1 (D9) tracks pitch (forward/back tilt), Servo 2
  (D10) tracks roll (side-to-side tilt).

  Only uses the Wire and Servo libraries (no MPU6050 helper library,
  per project rules).

  Wiring (GY-521 -> Nano):
    VCC -> 5V
    GND -> GND
    SCL -> A5
    SDA -> A4
    AD0 -> leave disconnected (I2C address defaults to 0x68)

  Wiring (Servo 1, pitch axis -> Nano):
    Signal (orange/yellow) -> D9
    Power  (red)           -> 5V
    Ground (brown/black)   -> GND

  Wiring (Servo 2, roll axis -> Nano):
    Signal (orange/yellow) -> D10
    Power  (red)           -> 5V
    Ground (brown/black)   -> GND

  Angle math (accelerometer-only, valid when the board is roughly static --
  i.e. gravity is the dominant force acting on it):
    roll  = atan2(accelY, accelZ)
    pitch = atan2(-accelX, sqrt(accelY^2 + accelZ^2))
  Raw accel counts are first divided by 16384.0 to convert to g's, since
  the MPU-6050 defaults to a +/-2g range (16384 LSB = 1g per the datasheet).

  Note: the accelerometer can only sense tilt relative to gravity
  (pitch/roll). It cannot sense rotation around the vertical axis (yaw),
  since gravity looks the same regardless of which way the board is
  facing while upright. Roll is used to drive the second (pan-style)
  servo as the practical stand-in for yaw with this sensor.
*/

#include <Wire.h>
#include <math.h>
#include <Servo.h>

// MPU-6050 register addresses (from the register map datasheet)
const uint8_t MPU_ADDR      = 0x68; // default I2C address when AD0 is low/unconnected
const uint8_t REG_WHO_AM_I  = 0x75;
const uint8_t REG_PWR_MGMT1 = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // first of 14 contiguous data registers

// Default full-scale sensitivity for the accelerometer (+/-2g range)
const float ACCEL_SENSITIVITY = 16384.0; // LSB per g

// Servo setup
Servo pitchServo; // Servo 1 -- tracks pitch
Servo rollServo;  // Servo 2 -- tracks roll
const uint8_t PITCH_SERVO_PIN = 9;
const uint8_t ROLL_SERVO_PIN  = 10;

// Raw sensor values, filled in by readRawData()
int16_t accelX, accelY, accelZ;
int16_t temperatureRaw;
int16_t gyroX, gyroY, gyroZ;

// Computed tilt angles, filled in by computeAngles(), in degrees
float rollDeg, pitchDeg;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to be ready (needed on some boards; harmless on Nano)
  }

  Wire.begin(); // join the I2C bus as controller

  // 1. Confirm we can talk to the chip at all
  uint8_t whoAmI = readRegister(REG_WHO_AM_I);
  Serial.print(F("WHO_AM_I register reads: 0x"));
  Serial.println(whoAmI, HEX);
  if (whoAmI != 0x68) {
    Serial.println(F("Unexpected value -- check wiring (SDA/SCL swapped? AD0 pulled high?)"));
  } else {
    Serial.println(F("MPU-6050 detected successfully."));
  }

  // 2. Wake the sensor up. On power-on it boots into sleep mode (bit 6 of
  //    PWR_MGMT_1 is set), so we must write 0 there to start sampling.
  writeRegister(REG_PWR_MGMT1, 0x00);
  delay(100); // give it a moment to stabilize after waking

  // 3. Attach both servos
  pitchServo.attach(PITCH_SERVO_PIN);
  rollServo.attach(ROLL_SERVO_PIN);
}

void loop() {
  readRawData();
  computeAngles();
  driveServos();

  Serial.print(F("Accel  X: ")); Serial.print(accelX);
  Serial.print(F("  Y: "));      Serial.print(accelY);
  Serial.print(F("  Z: "));      Serial.print(accelZ);

  Serial.print(F("   |   Gyro  X: ")); Serial.print(gyroX);
  Serial.print(F("  Y: "));            Serial.print(gyroY);
  Serial.print(F("  Z: "));            Serial.print(gyroZ);

  Serial.print(F("   |   Temp(raw): ")); Serial.print(temperatureRaw);

  Serial.print(F("   |   Roll: "));  Serial.print(rollDeg, 2);
  Serial.print(F(" deg   Pitch: "));  Serial.print(pitchDeg, 2);
  Serial.println(F(" deg"));

  delay(100); // ~10 Hz update rate, smooth enough for both servos to follow visibly
}

// Convert the current raw accelerometer readings into roll/pitch angles
// (in degrees) and store them in the rollDeg/pitchDeg globals.
void computeAngles() {
  float ax = accelX / ACCEL_SENSITIVITY; // convert raw counts -> g's
  float ay = accelY / ACCEL_SENSITIVITY;
  float az = accelZ / ACCEL_SENSITIVITY;

  rollDeg  = atan2(ay, az) * 180.0 / PI;
  pitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

// Map the computed pitch/roll angles (-90..+90 degrees each) onto each
// servo's 0..180 degree range and command both servos to move there.
void driveServos() {
  float pitchServoAngle = pitchDeg + 90.0; // shift -90..+90 into 0..180
  pitchServoAngle = constrain(pitchServoAngle, 0.0, 180.0); // safety clamp
  pitchServo.write((int)pitchServoAngle);

  float rollServoAngle = rollDeg + 90.0; // shift -90..+90 into 0..180
  rollServoAngle = constrain(rollServoAngle, 0.0, 180.0); // safety clamp
  rollServo.write((int)rollServoAngle);
}

// Read a single byte from the given register address.
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start, keep the bus held
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Write a single byte to the given register address.
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Read all 14 raw data bytes starting at ACCEL_XOUT_H in one burst and
// pack them into the global int16_t variables above. The sensor stores
// each value as two bytes, high byte first (big-endian).
void readRawData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  accelX         = (Wire.read() << 8) | Wire.read();
  accelY         = (Wire.read() << 8) | Wire.read();
  accelZ         = (Wire.read() << 8) | Wire.read();
  temperatureRaw = (Wire.read() << 8) | Wire.read();
  gyroX          = (Wire.read() << 8) | Wire.read();
  gyroY          = (Wire.read() << 8) | Wire.read();
  gyroZ          = (Wire.read() << 8) | Wire.read();
}
