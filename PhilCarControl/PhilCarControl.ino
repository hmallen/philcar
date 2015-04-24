/*
  Control Unit (Phil's Car - 1973 Datsun 240Z)

  ArduBerry Connected to Raspberry Pi 2 Model B

  Functions:
  1) Normal
  2) Trip

  Components:
  1) Shields
    - GPRS
    - SD
  2) Components
    - Photoresistor
    - Relay
    - Power jack (unregulated)
    - Voltage regulator
    - IR

  Commands:

  Data types:
    0 = Date & Time only
    1 = GPS
    2 = GPS/Sensors (Fast acquisition)
*/

#define debugMode

#include <SdFat.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"

const int gpsGmtOffsetHours = -5;  // Offset in hours of current timezone from GMT (I think it only accepts + vals b/c it gets converted to a 'byte')

// Digital outputs
const int ledRelay = 2;  // Relay to power IR LED's for night-vision imaging
// Digital inputs
const int tripSwitch = 3;  // Switch to trigger all "trip mode" functions and ready the system for sensor calibration
const int calibButton = 4;  // Momentary push-button to trigger sensor zeroing
const int readyIn = 5;  // Input from Arduino Mega signaling ready status

const int chipSelect = 10; // SPI chip select pin (Uno: 10/Mega: 53)

const unsigned long normalUpdateInterval = 600000;  // Delay between normal data updates without command input (ms)

// Libraries & Assignments
SoftwareSerial dataSerial(A2, A3);
SoftwareSerial smsSerial(7, 8);
SdFat SD;
File logFile;
RTC_DS1307 rtc;
DateTime now;

// Strings
String dataString;
String dataUpdated;
String dateString, timeString;
String monthString, dayString, yearString, hourString, minuteString, secondString;
String satellites, hdop;
String gpsLat, gpsLon;
String gpsAltitudeFt, gpsSpeedMPH, gpsCourse;
String accelerometerString, compassString, gyroscopeString;
//String accelX, accelY, accelZ;
//String compX, compY, compZ;
//String gyroX, gyroY, gyroZ;

// Booleans
boolean syncRequired = true;

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(ledRelay, OUTPUT);
  digitalWrite(ledRelay, LOW);
  pinMode(tripSwitch, INPUT);
  pinMode(calibButton, INPUT);
  pinMode(readyIn, INPUT);

  Serial.begin(9600);  // Serial connection for Raspberry Pi OR debugging on Arduino Uno
  dataSerial.begin(19200);  // Software serial connection to Arduino Mega

  Wire.begin();

  rtc.begin();
  if (!rtc.isrunning()) {
    Serial.println(F("RTC failed to initialize."));
    Serial.println(F("Arbitrary date set until sync completes."));
    rtc.adjust(DateTime(2015, 7, 30, 13, 0, 0));
  }
  else syncRequired = false;

  if (!SD.begin(chipSelect, SPI_FULL_SPEED)) SD.initErrorHalt();
  else Serial.println(F("SD card initialized."));

  if (digitalRead(readyIn) == LOW) {
    while (digitalRead(readyIn) == LOW) {
      delay(10);
    }
  }
  if (digitalRead(readyIn) == HIGH) {
    Serial.println(F("Sensor unit ready. Retrieving date & time."));
    sendRequest(4);  // Retrieve date & time from GPS on Arduino Mega
    Serial.println(F("Setting real-time clock."));
    setDateTime();  // Set RTC according to values retrieved from GPS on Arduino Mega
#ifdef debugMode
    Serial.print(F("Date/Time: "));
    Serial.print(dateString);
    Serial.print(", ");
    Serial.print(timeString);
    Serial.println();
#endif
  }
  if (!logFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    SD.errorHalt("Failed to open log file for data write.");
  }
  else {
    logFile.println("SYSTEM BOOT & SUCCESSFULL SETUP");
    logFile.print(dateString);
    logFile.print(",");
    logFile.println(timeString);
    logFile.close();
  }
}

void loop() {
  unsigned long startTime = millis();
  Serial.println(F("Ready for command..."));
  while ((millis() - startTime) < normalUpdateInterval) {
    if (Serial.available()) {
      String rpiString = "";
      while (Serial.available()) {
        char c = Serial.read();
        rpiString += c;
        delay(10);
      }
      dataSerial.print(rpiString);
      if (!dataSerial.available()) {
        while (!dataSerial.available()) {
          delay(10);
        }
      }
      if (dataSerial.available()) {
        while (dataSerial.available()) {
          char c = dataSerial.read();

          delay(10);
        }
      }
      Serial.println();
    }
    if (dataSerial.available()) {
      String dataCommandString = "";
      while (dataSerial.available()) {
        char c = Serial.read();
        dataCommandString += c;
        // INSERT RESPONSE TO ARDUINO MEGA SERIAL COMMAND
        delay(10);
      }
    }
    delay(10);
  }
}

void sendRequest(int menuCommand) {
  dataString = "";
  switch (menuCommand) {
    // Calibration commands
    case 0:
      dataSerial.write("0");  // Receiving-end stores as String
      if (digitalRead(calibButton) == LOW) {
        while (digitalRead(calibButton) == LOW) {
          delay(10);
        }
      }
      if (digitalRead(calibButton) == HIGH) {
        dataSerial.print('0');  // Receiving-end waiting for character
        while (digitalRead(calibButton) == HIGH || digitalRead(readyIn) == LOW) {
          delay(100);
        }
      }
      break;

    // GPS
    case 1:
      dataSerial.print("1");  // GPS
      if (!dataSerial.available()) {
        while (!dataSerial.available()) {
          delay(10);
        }
      }
      if (dataSerial.available()) {
        while (dataSerial.available()) {
          char c = dataSerial.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 1);  // GPS
        writeSDLog(1);  // GPS
      }
      break;

    // Sensors
    case 2:
      dataSerial.print("2");  // Sensors
      if (!dataSerial.available()) {
        while (!dataSerial.available()) {
          delay(10);
        }
      }
      if (dataSerial.available()) {
        while (dataSerial.available()) {
          char c = dataSerial.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 2);  // Sensors
        writeSDLog(2);  // Sensors
      }
      break;

    // GPS & Sensors
    case 3:
      dataSerial.print("3");  // GPS & Sensors
      if (!dataSerial.available()) {
        while (!dataSerial.available()) {
          delay(10);
        }
      }
      if (dataSerial.available()) {
        while (dataSerial.available()) {
          char c = dataSerial.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 3);  // GPS & Sensors
        writeSDLog(3);  // GPS & Sensors
      }
      break;

    // Date & Time
    case 4:  // Date & Time
      dataSerial.print("4");
      if (!dataSerial.available()) {
        while (!dataSerial.available()) {
          delay(10);
        }
      }
      dataString = "";
      if (dataSerial.available()) {
        while (dataSerial.available()) {
          char c = dataSerial.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 4);  // Date & Time
        assembleDateTime();
        setDateTime();  // Set RTC to date & time acquired from GPS data
        writeSDLog(4);  // Date & Time0
      }
      break;

    default:
      break;
  }
}

// Parse data received from sensor unit
void parseData(String rawData, int dataType) {
  int charMarker;
  switch (dataType) {
    case 0:
      // Data update status
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'E') break;
        if (c != 'D') dataUpdated += c;
      }
      break;

    // GPS
    case 1:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'S') break;
      }
      // Data update status
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'L') break;
        if (c != 'D') dataUpdated += c;
      }
      // Satellites
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'H') break;
        satellites += c;
      }
      // HDOP
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'T') break;
        hdop += c;
      }
      // Latitude
      gpsLat = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'N') break;
        gpsLat += c;
      }
      // Longitude
      gpsLon = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'a') break;
        gpsLon += c;
      }
      // Altitude (ft)
      gpsAltitudeFt = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 's') break;
        gpsAltitudeFt += c;
      }
      // Speed (MPH)
      gpsSpeedMPH = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'c') break;
        gpsSpeedMPH += c;
      }
      // Course
      gpsCourse = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'E') break;
        gpsCourse += c;
      }
      break;

    // Sensors
    case 2:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'S') break;
      }
      // Data update status
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'A') break;
        if (c != 'D') dataUpdated += c;
      }
      // Accelerometer string
      accelerometerString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'C') break;
        accelerometerString += c;
      }
      // Compass string
      compassString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'G') break;
        compassString += c;
      }
      // Gyroscope string
      gyroscopeString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'E') break;
        gyroscopeString += c;
      }
      break;

    // GPS & Sensors
    case 3:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'S') break;
      }
      // Data update status
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'L') break;
        if (c != 'D') dataUpdated += c;
      }
      // Satellites
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'H') break;
        satellites += c;
      }
      // HDOP
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'T') break;
        hdop += c;
      }
      // Latitude
      gpsLat = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'N') break;
        gpsLat += c;
      }
      // Longitude
      gpsLon = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'a') break;
        gpsLon += c;
      }
      // Altitude (ft)
      gpsAltitudeFt = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 's') break;
        gpsAltitudeFt += c;
      }
      // Speed (MPH)
      gpsSpeedMPH = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'c') break;
        gpsSpeedMPH += c;
      }
      // Course
      gpsCourse = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'A') break;
        gpsCourse += c;
      }
      // Accelerometer string
      accelerometerString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'C') break;
        accelerometerString += c;
      }
      // Compass string
      compassString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'G') break;
        compassString += c;
      }
      // Gyroscope string
      gyroscopeString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'E') break;
        gyroscopeString += c;
      }
      break;

    // Date & Time
    case 4:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'S') break;
      }
      // Month
      monthString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'D') break;
        if (c != 'M') monthString += c;
      }
      // Day
      dayString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'Y') break;
        dayString += c;
      }
      // Year
      yearString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'h') break;
        yearString += c;
      }
      // Hour
      hourString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 'm') break;
        hourString += c;
      }
      // Minute
      minuteString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        charMarker = i + 1;
        if (c == 's') break;
        minuteString += c;
      }
      // Second
      secondString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'E') break;
        secondString += c;
      }
      assembleDateTime();
      break;

    default:
      break;
  }
}

void assembleDateTime() {
  String monthTemp, dayTemp, yearTemp, hourTemp, minuteTemp, secondTemp;

  // Date formatting
  if (now.month() < 10) monthTemp = "0" + String(now.month());
  else monthTemp = String(now.month());
  if (now.day() < 10) dayTemp = "0" + String(now.day());
  else dayTemp = String(now.day());
  yearTemp = String(now.year());
  dateString = monthTemp + "-" + dayTemp + "-" + yearTemp;

  // Time formatting
  if (now.hour() < 10) hourTemp = "0" + String(now.hour());
  else hourTemp = String(now.hour());
  if (now.minute() < 10) minuteTemp = "0" + String(now.minute());
  else minuteTemp = String(now.minute());
  if (now.second() < 10) secondTemp = "0" + String(now.second());
  else secondTemp = String(now.second());
  timeString = hourTemp + ":" + minuteTemp + ":" + secondTemp;
}

// Configuration & Calibration
void setDateTime() {
  int month, day, year, hour, minute, second;
  month = monthString.toInt();
  day = dayString.toInt();
  year = yearString.toInt();
  hour = hourString.toInt();
  minute = minuteString.toInt();
  second = secondString.toInt();
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  now = rtc.now();
}

void writeSDLog(int writeMode) {
  if (!logFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    SD.errorHalt("Failed to open log file for data write.");
  }
  else {
    switch (writeMode) {
      // Calibration
      case 0:
        break;

      // GPS data read
      case 1:
        assembleDateTime();
        logFile.println(F("GPS DATA READ"));
        logFile.print(dataUpdated);
        logFile.print(",");
        logFile.print(dateString);
        logFile.print(",");
        logFile.print(timeString);
        logFile.print(",");
        logFile.print(satellites);
        logFile.print(",");
        logFile.print(hdop);
        logFile.print(",");
        logFile.print(gpsLat);
        logFile.print(",");
        logFile.print(gpsLon);
        logFile.print(",");
        logFile.print(gpsAltitudeFt);
        logFile.print(",");
        logFile.print(gpsSpeedMPH);
        logFile.print(",");
        logFile.println(gpsCourse);
        logFile.close();
        break;

      // Sensor data read
      case 2:
        assembleDateTime();
        logFile.println(F("SENSOR DATA READ"));
        logFile.print(dataUpdated);
        logFile.print(",");
        logFile.print(dateString);
        logFile.print(",");
        logFile.print(timeString);
        logFile.print(",");
        logFile.print(accelerometerString);
        logFile.print(",");
        logFile.print(compassString);
        logFile.print(",");
        logFile.println(gyroscopeString);
        logFile.close();
        break;

      // GPS & Sensor data read ("TRIP MODE")
      case 3:
        assembleDateTime();
        logFile.println(F("GPS & SENSOR DATA READ"));
        logFile.print(dataUpdated);
        logFile.print(",");
        logFile.print(dateString);
        logFile.print(",");
        logFile.print(timeString);
        logFile.print(",");
        logFile.print(satellites);
        logFile.print(",");
        logFile.print(hdop);
        logFile.print(",");
        logFile.print(gpsLat);
        logFile.print(",");
        logFile.print(gpsLon);
        logFile.print(",");
        logFile.print(gpsAltitudeFt);
        logFile.print(",");
        logFile.print(gpsSpeedMPH);
        logFile.print(",");
        logFile.print(gpsCourse);
        logFile.print(",");
        logFile.print(accelerometerString);
        logFile.print(",");
        logFile.print(compassString);
        logFile.print(",");
        logFile.println(gyroscopeString);
        logFile.close();
        break;

      // Date & Time data read
      case 4:
        assembleDateTime();
        logFile.println("DATE & TIME READ");
        logFile.print(dateString);
        logFile.print(",");
        logFile.println(timeString);
        logFile.close();
        break;

      default:
        break;
    }
  }
}

void checkDateTimeSync() {

}
