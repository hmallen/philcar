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
#define gpsUpdateFrequency 30000
#define sensorUpdateFrequency 1000

#include <SdFat.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"

const int gpsGmtOffsetHours = -5;  // Offset in hours of current timezone from GMT (I think it only accepts + vals b/c it gets converted to a 'byte')

// Digital outputs
const int ledRelay = 2;  // Relay to power IR LED's for night-vision imaging
const int smsPowerPin = 9;  // Pin to trigger software power-on of gprs shield

// Digital inputs
const int tripSwitch = 3;  // Switch to trigger all "trip mode" functions and ready the system for sensor calibration
const int calibButton = 4;  // Momentary push-button to trigger sensor zeroing
const int readyIn = 5;  // Input from Arduino Mega signaling ready status
const int rpiReadyIn = 6;  // Output to trigger sensor unit reset (CURRENTLY UNUSED)

const int chipSelect = 10; // SPI chip select pin (Uno: 10/Mega: 53)

const unsigned long normalUpdateInterval = 600000;  // Delay between normal data updates without command input (ms)

// Libraries & Assignments
SoftwareSerial dataSerial(A3, A2);
SoftwareSerial smsSerial(8, 7);
SdFat SD;
File logFile;
RTC_DS1307 rtc;
DateTime now;

// Strings
String dataString;
String dataUpdated;
String dateString, timeString;
String gpsCoordString, gpsDataString;
String accelerometerString, compassString, gyroscopeString;

// Booleans
boolean firstLoop = true;
boolean rtcSyncRequired = true;

// Timers
unsigned long gpsLastUpdate = 0;
unsigned long sensorLastUpdate = 0;

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(ledRelay, OUTPUT);
  digitalWrite(ledRelay, LOW);
  pinMode(tripSwitch, INPUT);
  pinMode(calibButton, INPUT);
  pinMode(readyIn, INPUT);
  pinMode(rpiReadyIn, INPUT);

  // Software power-on of gprs shield
  /*digitalWrite(smsPowerPin, LOW);
  delay(100);
  digitalWrite(smsPowerPin, HIGH);
  delay(500);
  digitalWrite(smsPowerPin, LOW);
  delay(100);*/

  Serial.begin(38400);  // Serial connection for Raspberry Pi OR debugging on Arduino Uno

  // Configure GPRS output for proper parsing
  /*smsSerial.begin(19200);
  delay(100);
  smsSerial.println(F("ATE0"));
  delay(100);
  smsSerial.println(F("ATQ1"));
  delay(100);
  smsSerial.println(F("ATV0"));
  delay(100);
  smsSerial.println(F("AT+CMGF=1"));
  delay(100);
  #ifdef debugMode
   Serial.print(F("Deleting all unnecessary SMS messages..."));
   #endif
   for (int x = 1; x < 31; x++) {
   smsSerial.print("AT+CMGD=");
   delay(100);
   smsSerial.print(x);
   delay(100);
   smsSerial.println(",3");
   delay(100);
   //smsSerialFlush();
   //delay(100);
   }
   #ifdef debugMode
   Serial.println(F("complete."));
   #endif*/
  //smsSerial.end();
  //delay(100);

  dataSerial.begin(38400);

  rpiWaitReady();  // Wait for ready signal from Raspberry Pi before proceeding

  Wire.begin();

  rtc.begin();
  if (!rtc.isrunning()) {
    Serial.println(F("RTC failed to initialize."));
    Serial.println(F("Arbitrary date set until sync completes."));
    rtc.adjust(DateTime(2015, 7, 30, 13, 0, 0));
  }

  if (!SD.begin(chipSelect, SPI_FULL_SPEED)) SD.initErrorHalt();
#ifdef debugMode
  else Serial.println(F("SD card initialized."));

  Serial.print(F("readyIn: "));
  Serial.println(digitalRead(readyIn));
#endif
  if (digitalRead(readyIn) == LOW) {
    while (digitalRead(readyIn) == LOW) {
      delay(10);
    }
  }
  if (digitalRead(readyIn) == HIGH) {
#ifdef debugMode
    Serial.println(F("Sensor unit ready. Retrieving date & time."));
#endif
    sendRequest(3);  // Retrieve date & time from GPS on Arduino Mega
    Serial.println(F("1"));
    assembleDateTime();
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
    logFile.println("SYSTEM BOOT & SETUP SUCCESSFUL");
    logFile.print(dateString);
    logFile.print(",");
    logFile.println(timeString);
    logFile.close();
  }
  Serial.println(F("R"));
}

void loop() {
  unsigned long smsStartTime = millis();
  //Serial.println(F("Ready for command..."));
  while ((millis() - gpsLastUpdate) < normalUpdateInterval) {
    // If command is received from RaspberryPi
    if (Serial.available()) {
      String rpiCommandString = "";
      while (Serial.available()) {
        char c = Serial.read();
        rpiCommandString += c;
        delay(10);
      }
      if (rpiCommandString.length() == 1) {
        int menuCommand = rpiCommandString.toInt();
        rpiMenu(menuCommand, false);
        if (firstLoop == true) firstLoop = false;
      }
      else {
#ifdef debugMode
        //Serial.println(F("Invalid command from RaspberryPi."));
#endif
      }

    }

    // If command is received from sensor unit
    if (dataSerial.available()) {
      String sensorCommandString = "";
      while (dataSerial.available()) {
        char c = Serial.read();
        sensorCommandString += c;
        delay(10);
      }
      if (sensorCommandString.length() == 1) {
        int menuCommand = sensorCommandString.toInt();
        sensorMenu(menuCommand);
      }
      else {
#ifdef debugMode
        //Serial.println(F("Invalid command from sensor unit."));
#endif
      }
    }

    // If command is received via SMS (Check at less frequent interval)
    /*if ((millis() - smsStartTime) > 10000) {
     dataSerial.end();
     delay(100);
     smsSerial.begin(19200);
     delay(100);
     smsSerial.println("AT+CMGL=\"REC UNREAD\"");
     for (int i = 0; i < 3; i++) {
     if (smsSerial.available()) {
     // SMS Reading & Parsing function
     }
     }
     delay(500);
     if (smsSerial.available()) {
     String sensorCommandString = "";
     while (smsSerial.available()) {
     char c = Serial.read();
     sensorCommandString += c;
     delay(10);
     }
     if (sensorCommandString.length() == 1) {
     int menuCommand = sensorCommandString.toInt();
     sensorMenu(menuCommand);
     }
     else {
     #ifdef debugMode
     Serial.println(F("Invalid command from sensor unit."));
     #endif
     }
     smsSerial.end();
     delay(100);  // Not sure if this is necessary
     }
     }*/

    if (digitalRead(rpiReadyIn) == 0) rpiWaitReady();

    delay(10);  // Delay between checks for incoming serial data
  }
}

void sendRequest(int menuCommand) {
  switch (menuCommand) {
    // Calibration commands
    case 0:
      dataSerial.print("0");  // Receiving-end stores as String
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
      }
      if (dataString == "0") {
        if (digitalRead(calibButton) == LOW) {
          while (digitalRead(calibButton) == LOW) {
            delay(10);
          }
        }
        if (digitalRead(calibButton) == HIGH) {
          while (digitalRead(calibButton) == HIGH) {
            delay(10);
          }
        }
        dataSerial.print("0");  // Receiving-end waiting for character
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
        }
#ifdef debugMode
        if (dataString == "0") Serial.println(F("Calibration succesful."));
        else Serial.println(F("Calibration failed."));
#endif
      }
#ifdef debugMode
      else Serial.println(F("Improper calibration ready response received."));
#endif
      break;

    // GPS
    case 1:
      dataSerial.print("1");  // GPS
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
        parseData(dataString, 1);  // GPS
        gpsLastUpdate = millis();
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
      dataString = "";
      if (dataSerial.available()) {
        while (dataSerial.available()) {
          char c = dataSerial.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 2);  // Sensors
        sensorLastUpdate = millis();
      }
      break;

    // Date & Time
    case 3:
      dataSerial.print("3");
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
        parseData(dataString, 3);  // Date & Time
      }
      break;

    default:
      break;
  }
}

// Menu of responses to RaspberryPi requests
void rpiMenu(int menuCommand, boolean tripMode) {
  switch (menuCommand) {
    case 0:
      break;

    // GPS data sent to RPi
    case 1:
      if ((millis() - gpsLastUpdate) > gpsUpdateFrequency || firstLoop == true || tripMode == true) sendRequest(1);
      Serial.println(gpsCoordString);
      Serial.println(gpsDataString);
      Serial.flush();
      break;

    // Sensor data sent to RPi
    case 2:
      if ((millis() - sensorLastUpdate) > sensorUpdateFrequency || tripMode == true) sendRequest(2);
      Serial.println(accelerometerString);
      Serial.println(compassString);
      Serial.println(gyroscopeString);
      Serial.flush();
      break;

    // Date & time sent to RPi
    case 3:
      assembleDateTime();
      Serial.println(dateString);
      Serial.println(timeString);
      Serial.flush();
      break;

    default:
      break;
  }
}

// Menu of responses to sensor unit requests
void sensorMenu(int menuCommand) {
  switch (menuCommand) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    default:
      break;
  }
}

// Parse data received from sensor unit
void parseData(String rawData, int dataType) {
  int charMarker;
  String monthString, dayString, yearString, hourString, minuteString, secondString;
  switch (dataType) {
    case 0:
      break;

    // GPS
    case 1:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'S') {
          charMarker = i + 1;
          break;
        }
      }
      // GPS Coordinates
      gpsCoordString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'D') {
          charMarker = i + 1;
          break;
        }
        if (c != 'C') gpsCoordString += c;
      }
      gpsDataString = "";
      // All other GPS data
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'E') {
          charMarker = i + 1;
          break;
        }
        gpsDataString += c;
      }
      break;

    // Sensors
    case 2:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'S') {
          charMarker = i + 1;
          break;
        }
      }
      // Data update status
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'A') {
          charMarker = i + 1;
          break;
        }
        if (c != 'D') dataUpdated += c;
      }
      // Accelerometer string
      accelerometerString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'C') {
          charMarker = i + 1;
          break;
        }
        accelerometerString += c;
      }
      // Compass string
      compassString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'G') {
          charMarker = i + 1;
          break;
        }
        compassString += c;
      }
      // Gyroscope string
      gyroscopeString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'E') break;
        gyroscopeString += c;
      }
      break;

    // Date & Time
    case 3:
      for (int i = 0; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'S') {
          charMarker = i + 1;
          break;
        }
      }
      // Month
      monthString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'D') {
          charMarker = i + 1;
          break;
        }
        if (c != 'M') monthString += c;
      }
      // Day
      dayString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'Y') {
          charMarker = i + 1;
          break;
        }
        dayString += c;
      }
      // Year
      yearString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'h') {
          charMarker = i + 1;
          break;
        }
        yearString += c;
      }
      // Hour
      hourString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'm') {
          charMarker = i + 1;
          break;
        }
        hourString += c;
      }
      // Minute
      minuteString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 's') {
          charMarker = i + 1;
          break;
        }
        minuteString += c;
      }
      // Second
      secondString = "";
      for (int i = charMarker; ; i++) {
        char c = rawData.charAt(i);
        if (c == 'E') break;
        secondString += c;
      }
      if (rtcSyncRequired == true) {
        rtc.adjust(DateTime(yearString.toInt(), monthString.toInt(), dayString.toInt(), hourString.toInt(), minuteString.toInt(), secondString.toInt()));
        now = rtc.now();
        rtcSyncRequired = false;
#ifdef debugMode
        Serial.println(F("RTC set from GPS date & time information."));
#endif
      }
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

void writeSDLog(int writeMode) {
  Serial.print(F("SD write..."));
  if (!logFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    SD.errorHalt("Failed to open log file for data write.");
  }
  else {
    switch (writeMode) {
      // Calibration
      case 0:
        logFile.println("DATE & TIME READ");
        logFile.print(dateString);
        logFile.print(",");
        logFile.println(timeString);
        logFile.flush();
        logFile.close();
        break;

      // GPS data read
      case 1:
        logFile.println(F("GPS DATA READ"));
        logFile.print(dataUpdated);
        logFile.print(",");
        logFile.print(dateString);
        logFile.print(",");
        logFile.print(timeString);
        logFile.print(",");
        logFile.print(gpsCoordString);
        logFile.print(",");
        logFile.println(gpsDataString);
        logFile.flush();
        logFile.close();
        break;

      // Sensor data read
      case 2:
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
        logFile.flush();
        logFile.close();
        break;

      default:
        break;
    }
  }
  Serial.println(F("done."));
}

void rpiWaitReady() {
  if (digitalRead(rpiReadyIn) == 0) {
    unsigned long waitStartTime = millis();
    while (digitalRead(rpiReadyIn) == 0) {
      if ((millis() - waitStartTime) > 10000) {
        Serial.println(F("Waiting for RPi..."));
        waitStartTime = millis();
      }
#ifdef debugMode
      String debugBreakString = "";
      if (Serial.available()) {
        while (Serial.available()) {
          char c = Serial.read();
          debugBreakString += c;
          delay(10);
        }
      }
      if (debugBreakString == "B") break;
#endif
      delay(100);
    }
  }
  Serial.println("R");
}
