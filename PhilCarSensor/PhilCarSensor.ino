/*
  Sensor Unit (Phil's Car - 1973 Datsun 240Z)

  Components:
  1) Shields
    - Custom sensor protoshield
      - Accelerometer
      - Compass
      - Gyroscope
  2) Components
    - 12V Siren
*/

//#define debugMode

#include <Time.h>
#include <TinyGPS.h>
#include <Wire.h>

#define compAddr 0x1E  // Compass
#define gyroAddr 0x69  // Gyroscope

// Digital pins
const int readyOut = 2;  // Ready signal for ArduBerry functions
const int calibButton = 3;  // Relay gating power to intruder siren
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
const int gpsSatMinimum = 4;  // May want to change to 6 for real launch
const int gpsHDOPMinimum = 250;  // May want to change to 200 for real launch
int satellites, hdop;
float gpsLat, gpsLon, gpsAltitudeFt, gpsSpeedMPH, gpsCourse;

// Strings
String accelerometerString, compassString, gyroscopeString, gpsString;

// Library Definitions
TinyGPS gps;  // GPS

// Booleans
boolean gpsLock = false;
boolean readGPSDateTime = false;
boolean sensorZeroMode = false;
boolean dataUpdated = false;

void setup() {
  pinMode(readyOut, OUTPUT);
  digitalWrite(readyOut, LOW);
  pinMode(calibButton, INPUT);

  Serial.begin(38400);  // Debug Serial

  Serial1.begin(4800);  // GPS Serial
  if (!Serial1.available()) {  // Wait for GPS data stream to begin before proceeding
    while (!Serial1.available()) {
      delay(10);
    }
  }

  Serial2.begin(38400);  // Data connection to ArduBerry

  Wire.begin();  // Initiate I2C connection

  // Setup I2C sensors
  i2cWriteByte(compAddr, 0x0, 0x10);  // Compass - Default value
  i2cWriteByte(compAddr, 0x1, 0x20);  // Compass - Default value
  i2cWriteByte(compAddr, 0x2, 0x0);  // Compass - \Continuous measurement mode
  delay(100);  // Wait to synchronize
  i2cWriteByte(gyroAddr, 0x20, 0x1F);  // Gyroscope - Turn on all axes and disable power down
  i2cWriteByte(gyroAddr, 0x22, 0x08);  // Gyroscope - Enable control ready signal
  i2cWriteByte(gyroAddr, 0x23, 0x80);  // Gyroscope - Set scale (0x80 = 500deg/sec)
  delay(100);  // Wait to synchronize

  // Wait for GPS to acquire enough satellites and have sufficient HDOP (precision)
  readGPS();
  if (satellites < gpsSatMinimum || hdop > gpsHDOPMinimum) {
    while (satellites < gpsSatMinimum || hdop > gpsHDOPMinimum) {
      readGPS();
#ifdef debugMode
      Serial.print(F("Sats: "));
      Serial.println(satellites);
      Serial.print(F("HDOP: "));
      Serial.println(hdop);
#endif
      delay(5000);
    }
  }
  //#ifdef debugMode
  Serial.println(F("GPS satellites acquired with sufficient precision."));
  Serial.println();
  delay(1000);
  //#endif
  gpsLock = true;

  // Set internal date and time from GPS
  readGPSDateTime = true;
  readGPS();
  //#ifdef debugMode
  Serial.println(F("Date & Time set from GPS data."));
  Serial.println();
  delay(100);
  //#endif
  readGPSDateTime = false;

  digitalWrite(readyOut, HIGH);
  //#ifdef debugMode
  Serial.println(F("Setup complete."));
  //#endif
}

void loop() {
#ifndef debugMode
  if (!Serial2.available()) {
    while (!Serial2.available()) {
      delay(100);
    }
  }
  String commandString = "";
  if (Serial2.available()) {
    while (Serial2.available()) {
      char c = Serial2.read();
      commandString += c;
      delay(100);
    }
    Serial.print(F("commandString: "));
    Serial.println(commandString);
  }
#else
  if (!Serial.available()) {
    while (!Serial.available()) {
      delay(100);
    }
  }
  String commandString = "";
  if (Serial.available()) {
    while (Serial.available()) {
      char c = Serial.read();
      commandString += c;
      delay(100);
    }
  }
#endif
  if (commandString.length() == 1) {
    int menuCommand = commandString.toInt();
    modeMenu(menuCommand);
  }
  else {
#ifdef debugMode
    Serial.println(F("Invalid command."));
#endif
  }
  dataUpdated = false;
  delay(1000);
}

// Mode menu controlled by ArduBerry via serial connection (Serial2 tx)
void modeMenu(int menuCommand) {
  switch (menuCommand) {
    // Sensor calibration (zeroing)
    case 0:
#ifdef debugMode
      Serial.println(F("Waiting for calibration command..."));
#endif
      digitalWrite(readyOut, LOW);
#ifndef debugMode
      if (digitalRead(calibButton) == LOW) {
        while (digitalRead(calibButton) == LOW) {
          delay(10);
        }
      }
      if (digitalRead(calibButton) == HIGH) {
        while (digitalRead(calibButton) == HIGH) {
          delay(100);
        }
      }
#else
      if (!Serial.available()) {
        while (!Serial.available()) {
          delay(10);
        }
      }
      if (Serial.available()) {
        while (Serial.available()) {
          char c = Serial.read();
          if (c == 'B') break;
          delay(10);
        }
      }
#endif
      zeroSensors();
      digitalWrite(readyOut, HIGH);
      Serial.println(F("Calibration complete."));
      break;

    // Read GPS
    case 1:
      readGPS();
      if (gpsLat != 0 && gpsLon != 0) dataUpdated = true;
      sendData(1);
      break;

    // Read sensors
    case 2:
      readAccelerometer();
      readCompass();
      readGyroscope();
      if (gpsLat != 0 && gpsLon != 0) dataUpdated = true;
      sendData(2);
      break;

    // Read GPS for date & time only
    case 3:
      readGPS();
      sendData(3);
      break;

    default:
      break;
  }
}

void sendData(int menuCommand) {
  switch (menuCommand) {
    case 0:
      break;

    // GPS
    case 1:
#ifdef debugMode
      Serial.print("S");
      Serial.print("C");
      Serial.print(gpsLat, 6);
      Serial.print(",");
      Serial.print(gpsLon, 6);
      Serial.print("D");
      Serial.print(dataUpdated);
      Serial.print(",");
      Serial.print(satellites);
      Serial.print(",");
      Serial.print(hdop);
      Serial.print(",");
      Serial.print(gpsAltitudeFt);
      Serial.print(",");
      Serial.print(gpsSpeedMPH);
      Serial.print(",");
      Serial.print(gpsCourse);
      Serial.print("E");
      Serial.println();
      Serial.flush();
#else
      Serial2.print("S");
      Serial2.print("C");
      Serial2.print(gpsLat, 6);
      Serial2.print(",");
      Serial2.print(gpsLon, 6);
      Serial2.print("D");
      Serial2.print(dataUpdated);
      Serial2.print(",");
      Serial2.print(satellites);
      Serial2.print(",");
      Serial2.print(hdop);
      Serial2.print(",");
      Serial2.print(gpsAltitudeFt);
      Serial2.print(",");
      Serial2.print(gpsSpeedMPH);
      Serial2.print(",");
      Serial2.print(gpsCourse);
      Serial2.print("E");
      Serial2.flush();
#endif
      break;

    // Sensors
    case 2:
#ifdef debugMode
      Serial.print("S");
      Serial.print("D");
      Serial.print(dataUpdated);
      Serial.print("A");
      Serial.print(accelerometerString);
      Serial.print("C");
      Serial.print(compassString);
      Serial.print("G");
      Serial.print(gyroscopeString);
      Serial.print("E");
      Serial.println();
      Serial.flush();
#else
      Serial2.print("S");
      Serial2.print("D");
      Serial2.print(dataUpdated);
      Serial2.print("A");
      Serial2.print(accelerometerString);
      Serial2.print("C");
      Serial2.print(compassString);
      Serial2.print("G");
      Serial2.print(gyroscopeString);
      Serial2.print("E");
      Serial2.flush();
#endif
      break;

    // Date & Time from GPS
    case 3:
#ifdef debugMode
      Serial.print("S");
      Serial.print("M");
      Serial.print(month());
      Serial.print("D");
      Serial.print(day());
      Serial.print("Y");
      Serial.print(year());
      Serial.print("h");
      Serial.print(hour());
      Serial.print("m");
      Serial.print(minute());
      Serial.print("s");
      Serial.print(second());
      Serial.print("E");
      Serial.println();
      Serial.flush();
#else
      Serial2.print("S");
      Serial2.print("M");
      Serial2.print(month());
      Serial2.print("D");
      Serial2.print(day());
      Serial2.print("Y");
      Serial2.print(year());
      Serial2.print("h");
      Serial2.print(hour());
      Serial2.print("m");
      Serial2.print(minute());
      Serial2.print("s");
      Serial2.print(second());
      Serial2.print("E");
      Serial2.flush();
#endif
      break;

    default:
      break;
  }
}

// Pololu Accelerometer
void readAccelerometer() {
  if (sensorZeroMode == false) {
    int accelX = analogRead(accelXPin) - accelXOffset;
    int accelY = analogRead(accelYPin) - accelYOffset;
    int accelZ = analogRead(accelZPin) - accelZOffset;

    accelerometerString = String(accelX) + "," + String(accelY) + "," + String(accelZ);
  }
  else {
    accelX = analogRead(accelXPin);
    accelY = analogRead(accelYPin);
    accelZ = analogRead(accelZPin);
  }
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

    compassString = String(compX) + "," + String(compY) + "," + String(compZ);
  }
  else Serial.println(F("Failed to read from sensor."));
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

    if (sensorZeroMode == false) {
      gyroX = (int)x - gyroXOffset;
      gyroY = (int)y - gyroYOffset;
      gyroZ = (int)z - gyroZOffset;

      gyroscopeString = String(gyroX) + "," + String(gyroY) + "," + String(gyroZ);
    }
    else {
      gyroX = (int)x;
      gyroY = (int)y;
      gyroZ = (int)z;
    }
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

void readGPS() {
  boolean newdata = false;
  unsigned long start = millis();
  while (millis() - start < 1000) { // Update every 5 seconds
    if (feedgps()) newdata = true;
    if (newdata) gpsdump(gps);
  }
}

void gpsdump(TinyGPS & gps) {
  float flat, flon, altitude;
  unsigned long age;
  byte month, day, hour, minute, second, hundredths;
  int year;

  if (gpsLock == false) {
    satellites = gps.satellites();
    hdop = gps.hdop();
  }
  else {
    if (readGPSDateTime == true) {
      const int gpsGmtOffsetHours = -5;  // Offset in hours of current timezone from GMT (I think it only accepts + vals b/c it gets converted to a 'byte')
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
      setTime(hour, minute, second, day, month, year);
      adjustTime(gpsGmtOffsetHours * SECS_PER_HOUR);
    }
    else {
      gps.f_get_position(&flat, &flon, &age);
      gpsLat = flat;
      gpsLon = flon;
      altitude = gps.f_altitude();
      gpsAltitudeFt = altitude / 3.048;
      gpsCourse = gps.f_course();
      gpsSpeedMPH = gps.f_speed_mph();

      gpsString = String(satellites) + "," + String(hdop) + "," + String(gpsAltitudeFt) + "," + String(gpsCourse) + "," + String(gpsSpeedMPH);
    }
  }
}

boolean feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read())) return true;
  }
  return false;
}

// Calibration & Configuration
void zeroSensors() {
  sensorZeroMode = true;
  const int sampleNumber = 10;

  int accelXTot = 0;
  int accelYTot = 0;
  int accelZTot = 0;
  int gyroXTot = 0;
  int gyroYTot = 0;
  int gyroZTot = 0;

  // Accelerometer
  for (int x = 0; x < sampleNumber; x++) {
    readAccelerometer();
    accelXTot += accelX;
    accelYTot += accelY;
    accelZTot += accelZ;
    delay(100);
  }
  accelXOffset = accelXTot / sampleNumber;
  accelYOffset = accelYTot / sampleNumber;
  accelZOffset = accelZTot / sampleNumber;

  // Gyroscope
  for (int y = 0; y < sampleNumber; y++) {
    readGyroscope();
    if (abs(gyroX) > gyroOffset) gyroOffset = gyroX;
    if (abs(gyroY) > gyroOffset) gyroOffset = gyroY;
    if (abs(gyroZ) > gyroOffset) gyroOffset = gyroZ;
    gyroXTot += gyroX;
    gyroYTot += gyroY;
    gyroZTot += gyroZ;
    delay(100);
  }
  gyroOffset = sqrt(gyroOffset);
  gyroXOffset = (gyroXTot / sampleNumber) / 10;
  gyroYOffset = (gyroYTot / sampleNumber) / 10;
  gyroZOffset = (gyroZTot / sampleNumber) / 10;

  sensorZeroMode = false;
}
