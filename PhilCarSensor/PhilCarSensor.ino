/* Testing framework for all components in HAB telemetry package */
// CFC sensor --> A11
// Methane sensor --> A12
// PIR sensor --> 6
// Smoke signal relay --> 8
// Siren relay --> 7
// Altimeter --> I2C
// Sensiron (Parallax) temp/humidity sensor --> I2C
// Accelerometer --> A13/A14/A15
// Compass --> I2C
// Gyroscope --> I2C
// GPS --> Serial1
// SD shield --> "SdInfo" sketch in SdFat examples
// GPRS shield --> Call of "AT+CCLK?" AT command to check time

/*
Changes:
tripLED --> tripLED
readyLED --> powerLED

Removed all digital and analog pins not relevant to this sketch


*/

const boolean debugMode = true;

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>

#define compAddr 0x1E  // Compass
#define gyroAddr 0x69  // Gyroscope

// Digital pins
const int powerLED = 4;  // Pin for LED to indicate entry of main loop (program start)
const int tripLED = 5;  // Pin for LED to indicate detection of landing
const int gprsPowPin = 9;  // Pin for software power-up of GPRS shield
// Analog pins
const int accelXPin = A15;  // Accelerometer X axis
const int accelYPin = A14;  // Accelerometer Y axis
const int accelZPin = A13;  // Accelerometer Z axis

// Sensor Global Variables
int accelX, accelY, accelZ;
int gyroX, gyroY, gyroZ;
int compX, compY, compZ;
// Sensor Offsets
int accelXOffset, accelYOffset, accelZOffset;
int gyroOffset, gyroXOffset, gyroYOffset, gyroZOffset;
// GPS Global Variables
unsigned long age;
float flat, flon, altitude;
int satellites, hdop;
// Strings
String accelerometerString, compassString, gyroscopeString, gpsString;

// Library Definitions
SoftwareSerial smsSerial(7, 8);
TinyGPS gps;  // GPS

void setup() {
  Serial.begin(9600);  // Debug Serial

  /*Serial1.begin(4800);  // GPS Serial
  if (!Serial1.available()) {
    while (!Serial1.available()) {
      delay(1);
    }
  }*/

  smsSerial.begin(19200);  // SMS Serial

  // Software power-up of GPRS shield
  digitalWrite(gprsPowPin, LOW);
  delay(100);
  digitalWrite(gprsPowPin, HIGH);
  delay(500);
  digitalWrite(gprsPowPin, LOW);
  delay(100);

  Wire.begin();

  Serial.println(F("Initiating component testing."));
  Serial.println();
}

void loop() {
    readAccelerometer();
    readCompass();
    readGyroscope();
    delay(1000);
}

// Pololu Accelerometer
void readAccelerometer() {
  int accelX = analogRead(accelXPin);
  int accelY = analogRead(accelYPin);
  int accelZ = analogRead(accelZPin);

  String accelerometerString = String(accelX) + "," + String(accelY) + "," + String(accelZ);

#ifdef debugMode
  Serial.print(F("Accel. values: "));
  Serial.println(accelerometerString);
#endif
}

// Compass
void readCompass() {
  uint8_t x_msb; // X-axis most significant byte
  uint8_t x_lsb; // X-axis least significant byte
  uint8_t y_msb; // Y-axis most significant byte
  uint8_t y_lsb; // Y-axis least significant byte
  uint8_t z_msb; // Z-axis most significant byte
  uint8_t z_lsb; // Z-axis least significant byte
  // Get the value from the sensor
  if ((i2cReadByte(compAddr, 0x3, &x_msb) == 0) &&
      (i2cReadByte(compAddr, 0x4, &x_lsb) == 0) &&
      (i2cReadByte(compAddr, 0x5, &y_msb) == 0) &&
      (i2cReadByte(compAddr, 0x6, &y_lsb) == 0) &&
      (i2cReadByte(compAddr, 0x7, &z_msb) == 0) &&
      (i2cReadByte(compAddr, 0x8, &z_lsb) == 0)) {
    uint8_t compX = x_msb << 8 | x_lsb;
    uint8_t compY = y_msb << 8 | y_lsb;
    uint8_t compZ = z_msb << 8 | z_lsb;

    String compassString = String(compX) + "," + String(compY) + "," + String(compZ);

#ifdef debugMode
    Serial.print(F("Comp. values: "));
    Serial.println(compassString);
#endif
  }
  else if(debugMode == true) Serial.println(F("Failed to read from sensor."));
}

void readGyroscope() {
  int gyroX, gyroY, gyroZ;
  uint8_t x_msb; // X-axis most significant byte
  uint8_t x_lsb; // X-axis least significant byte
  uint8_t y_msb; // Y-axis most significant byte
  uint8_t y_lsb; // Y-axis least significant byte
  uint8_t z_msb; // Z-axis most significant byte
  uint8_t z_lsb; // Z-axis least significant byte

  if ((i2cReadByte(gyroAddr, 0x29, &x_msb) == 0) &&
      (i2cReadByte(gyroAddr, 0x28, &x_lsb) == 0) &&
      (i2cReadByte(gyroAddr, 0x2B, &y_msb) == 0) &&
      (i2cReadByte(gyroAddr, 0x2A, &y_lsb) == 0) &&
      (i2cReadByte(gyroAddr, 0x2D, &z_msb) == 0) &&
      (i2cReadByte(gyroAddr, 0x2C, &z_lsb) == 0)) {
    uint16_t x = x_msb << 8 | x_lsb;
    uint16_t y = y_msb << 8 | y_lsb;
    uint16_t z = z_msb << 8 | z_lsb;

    gyroX = (int)x;
    gyroY = (int)y;
    gyroZ = (int)z;

    String gyroString = String(gyroX) + "," + String(gyroY) + "," + String(gyroZ);

#ifdef debugMode
    Serial.print(F("Gyro. values: "));
    Serial.println(gyroString);
#endif
  }
  else Serial.println(F("Failed to read from sensor."));
}

// Read a byte on the i2c interface
int i2cReadByte(uint8_t addr, uint8_t reg, uint8_t *data) {
  // Do an i2c write to set the register that we want to read from
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  // Read a byte from the device
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) *data = Wire.read();
  else return -1;  // Read nothing back
  return 0;
}

// Write a byte on the i2c interface
void i2cWriteByte(uint8_t addr, uint8_t reg, byte data) {
  Wire.beginTransmission(addr);  // Begin the write sequence
  Wire.write(reg);  // First byte is to set the register pointer
  Wire.write(data);  // Write the data byte
  Wire.endTransmission();  // End the write sequence; bytes are actually transmitted now
}

void gprsDateTime() {
  Serial2.println("AT+CCLK?");
  if (!Serial2.available()) {
    while (!Serial2.available()) {
      delay(1);
    }
  }
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);
    delay(10);
  }
}

void gpsGetData() {
  boolean newdata = false;
  unsigned long start = millis();
  while (millis() - start < 1000) { // Update every 5 seconds
    if (feedgps()) newdata = true;
    if (newdata) gpsdump(gps);
  }
}

void gpsdump(TinyGPS &gps) {
  satellites = gps.satellites();
  hdop = gps.hdop();
  gps.f_get_position(&flat, &flon, &age);
  altitude = gps.f_altitude();
}

// Feed
boolean feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read())) return true;
  }
  return false;
}
