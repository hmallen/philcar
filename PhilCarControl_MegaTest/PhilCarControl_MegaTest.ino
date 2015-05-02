/*
  Control Unit (Phil's Car - 1973 Datsun 240Z)

  Rerouted pins for Mega usage:
  SD Card (SPI):
  10 --> 53 (SS)
  11 --> 51 (MOSI)
  12 --> 50 (MISO)
  13 --> 52 (SCK)
  RTC (I2C):
  A4 --> 20 (SDA)
  A5 --> 21 (SCL)

  TO DO:
   - CHECK FOR FULL SMS STORAGE AND CLEAR IF NECESSARY
 */

#define debugMode
//#define smsRX 7
//#define smsTX 8
#define smsPowerPin 9
#define chipSelect 53  // SPI chip select pin (Uno: 10/Mega: 53)
#define gpsUpdateFrequency 30000
#define sensorUpdateFrequency 1000
#define irLEDDelay 8000

//#include <SoftwareSerial.h>
#include <SdFat.h>
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"

const int gpsGmtOffsetHours = -5;  // Offset in hours of current timezone from GMT (I think it only accepts + vals b/c it gets converted to a 'byte')

// Analog inputs
const int lightPin = A0;  // Photoresistor input to measure light levels for IR LED activation

// Digital inputs
const int tripSwitch = 4;  // Switch to trigger all "trip mode" functions and ready the system for sensor calibration
const int sensorReadyIn = 5;  // Input from Arduino Mega signaling ready status
const int rpiReadyIn = 6;  // Input to read ready state of RPi

// Digital outputs
const int controlReadyOut = A1;  // Output to signal control system ready for RPi
const int ledRelay = 2;  // Relay to power IR LED's for night-vision imaging
const int sirenRelay = 3;  // Relay to trigger siren mounted on sensor unit enclosure

const unsigned long normalUpdateInterval = 600000;  // Delay between normal data updates without command input (ms)

// Libraries & Assignments
//SoftwareSerial Serial1(smsRX, smsTX);
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
String smsNumber, smsMessage;

// Booleans
boolean firstLoop = true;
boolean rtcSyncRequired = true;
boolean newData = false;

// Timers
unsigned long gpsLastUpdate = 0;
unsigned long sensorLastUpdate = 0;

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(sirenRelay, OUTPUT);
  digitalWrite(sirenRelay, LOW);
  pinMode(smsPowerPin, OUTPUT);
  digitalWrite(smsPowerPin, HIGH);
  pinMode(ledRelay, OUTPUT);
  digitalWrite(ledRelay, LOW);
  pinMode(controlReadyOut, OUTPUT);
  digitalWrite(controlReadyOut, LOW);
  pinMode(tripSwitch, INPUT);
  pinMode(sensorReadyIn, INPUT);
  pinMode(rpiReadyIn, INPUT);

  Serial.begin(38400);  // Serial connection for Raspberry Pi OR debugging on Arduino Uno
  Serial1.begin(38400);  // Serial connection for sensor unit communication

#ifdef debugMode
  Serial.println(F("Beginning setup."));
#endif

  Wire.begin();
  rtc.begin();

  if (!rtc.isrunning()) {
    Serial.println(F("RTC failed to initialize."));
    Serial.println(F("Arbitrary date set until sync completes."));
    rtc.adjust(DateTime(2015, 7, 30, 13, 0, 0));
  }
  else {
#ifdef debugMode
    Serial.println(F("RTC initialized."));
#endif
  }

  Serial2.begin(19200);  // Serial connection for gprs shield communication
  smsStartup();  // Clear GPRS buffer while waiting for network connection

  rpiWaitReady();  // Wait for ready signal from Raspberry Pi before proceeding

  if (!SD.begin(chipSelect, SPI_FULL_SPEED)) SD.initErrorHalt();
#ifdef debugMode
  else Serial.println(F("SD card initialized."));
#endif
  if (digitalRead(sensorReadyIn) == LOW) {
#ifdef debugMode
    Serial.println(F("Waiting for sensor unit..."));
#endif
    while (digitalRead(sensorReadyIn) == LOW) {
#ifdef debugMode
      Serial.write(Serial1.read());
#endif
      delay(10);
    }
  }
  if (digitalRead(sensorReadyIn) == HIGH) {
#ifdef debugMode
    Serial.println(F("Sensor unit ready. Retrieving date & time."));
#endif
    sendRequest(3);  // Retrieve date & time from GPS on Arduino Mega
    assembleDateTime();
#ifdef debugMode
    Serial.print(F("Date / Time : "));
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
    logFile.print(", ");
    logFile.println(timeString);
    logFile.flush();
    logFile.close();
  }
  digitalWrite(controlReadyOut, HIGH);
  Serial.println(F("Setup complete."));
}

void loop() {
  const int smsCheckDelay = 10000;
  unsigned long smsDelayStart = millis();
  //Serial.println(F("Ready for command..."));
  while ((millis() - gpsLastUpdate) < normalUpdateInterval) {
    if ((millis() - smsDelayStart) > smsCheckDelay) {
      checkSMS();
      smsDelayStart = millis();
    }
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
        //Serial.println(F("Invalid command from RaspberryPi."));
      }
    }
    if (digitalRead(rpiReadyIn) == 0) rpiWaitReady();

    delay(10);  // Delay between checks for incoming serial data
  }
  sendRequest(1);
}

void sendRequest(int menuCommand) {
  switch (menuCommand) {
    // Calibration commands
    case 0:
      Serial1.print("0");  // Receiving-end stores as String
      if (!Serial1.available()) {
        while (!Serial1.available()) {
          delay(10);
        }
      }
      dataString = "";
      if (Serial1.available()) {
        while (Serial1.available()) {
          char c = Serial1.read();
          dataString += c;
          delay(10);
        }
      }
      if (dataString == "0") {
        if (digitalRead(sensorReadyIn) == HIGH) {
          while (digitalRead(sensorReadyIn) == HIGH) {
            delay(10);
          }
        }
        Serial1.print("0");  // Receiving-end waiting for character
        if (!Serial1.available()) {
          while (!Serial1.available()) {
            delay(10);
          }
        }
        dataString = "";
        if (Serial1.available()) {
          while (Serial1.available()) {
            char c = Serial1.read();
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
      Serial1.print("1");  // GPS
      if (!Serial1.available()) {
        while (!Serial1.available()) {
          delay(10);
        }
      }
      dataString = "";
      if (Serial1.available()) {
        while (Serial1.available()) {
          char c = Serial1.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 1);  // GPS
        gpsLastUpdate = millis();
      }
      break;

    // Sensors
    case 2:
      Serial1.print("2");  // Sensors
      if (!Serial1.available()) {
        while (!Serial1.available()) {
          delay(10);
        }
      }
      dataString = "";
      if (Serial1.available()) {
        while (Serial1.available()) {
          char c = Serial1.read();
          dataString += c;
          delay(10);
        }
        parseData(dataString, 2);  // Sensors
        sensorLastUpdate = millis();
      }
      break;

    // Date & Time
    case 3:
      Serial1.print("3");
      if (!Serial1.available()) {
        while (!Serial1.available()) {
          delay(10);
        }
      }
      dataString = "";
      if (Serial1.available()) {
        while (Serial1.available()) {
          char c = Serial1.read();
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
  int lightLevel;
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

    // Check light level and turn on IR LEDs if necessary
    case 4:
      lightLevel = map(analogRead(lightPin), 0, 1023, 0, 100);
      if (lightLevel <= 40) {
        digitalWrite(ledRelay, HIGH);
        delay(irLEDDelay);
        digitalWrite(ledRelay, LOW);
      }
      break;

    default:
      break;
  }
}

// Menu of responses to sensor unit requests
/*void sensorMenu(int menuCommand) {
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
}*/

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
  String sdZeroString, sdOneString, sdTwoString;
  Serial.print(F("SD write..."));
  if (!logFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    SD.errorHalt("Failed to open log file for data write.");
  }
  else {
    switch (writeMode) {
      // Calibration
      case 0:
        sdZeroString = dateString + ", " + timeString;
        logFile.println(F("DATE & TIME READ"));
        logFile.println(sdZeroString);
        logFile.flush();
        logFile.close();
        break;

      // GPS data read
      case 1:
        sdOneString = dataUpdated + ", " + dateString + ", " + timeString + ", " + gpsCoordString + ", " + gpsDataString;
        logFile.println(F("GPS DATA READ"));
        logFile.println(sdOneString);
        logFile.flush();
        logFile.close();
        break;

      // Sensor data read
      case 2:
        sdTwoString = dataUpdated + ", " + dateString + ", " + timeString + ", " + accelerometerString + ", " + compassString + ", " + gyroscopeString;
        logFile.println(F("SENSOR DATA READ"));
        logFile.println(sdTwoString);
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
  if (digitalRead(rpiReadyIn) == LOW) {
    unsigned long waitStartTime = millis();
    while (digitalRead(rpiReadyIn) == LOW) {
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
}

void checkSMS() {
  boolean newData = false;
#ifdef debugMode
  Serial.print(F("Checking SMS inbox..."));
#endif
  Serial2.println(F("AT+CMGL=\"REC UNREAD\""));
  delay(100);

  String smsRaw = "";
  if (Serial2.available()) {
    newData == true;
#ifdef debugMode
    Serial.println(F("SMS received."));
#endif
    while (Serial2.available()) {
      char c = Serial2.read();
      smsRaw += c;
      if (c == '\n' || c == '\r') continue;
      delay(100);
    }
#ifdef debugMode
    Serial.print(F("smsRaw: "));
    Serial.println(smsRaw);
    Serial.flush();
#endif
    //int smsLength = smsRaw.length();
    Serial.println(F("1"));
    parseSMS(smsRaw);
    Serial.println(F("2"));
    //Serial2.println("AT+CMGD=0,2");  // Delete all read messages
    smsFlush();
  }
  else {
#ifdef debugMode
    Serial.println(F("empty."));
#endif
  }

  if (newData == true) {
#ifdef debugMode
    Serial.print(F("smsNumber:  "));
    Serial.println(smsNumber);
    Serial.print(F("smsMessage: "));
    Serial.println(smsMessage);
#endif
    newData = false;
  }
}

void parseSMS(String smsRaw) {
  // Numbers permitted for SMS menu use
  String smsTargetNum0 = "+12145635266";  // HA
  String smsTargetNum1 = "+12146738002";  // PK
  String smsTargetNum2 = "+12143155885";  // GK
  String smsTargetNum3 = "+12147070200";  // TK
  String smsTargetNum4 = "+12146738003";  // MK
  String smsTargetNum5 = "+18327790024";  // Google Voice (HA)

  smsNumber = "";
  smsMessage = "";
  for (int x = (smsRaw.indexOf('+', 2)); ; x++) {
    char c = smsRaw.charAt(x);
    if (c == '\"') break;
    smsNumber += c;
  }
  for (int x = (smsRaw.indexOf('\n') + 1); ; x++) {
    char c = smsRaw.charAt(x);
    smsMessage += c;
  }
  if (smsNumber == smsTargetNum0 ||
      smsNumber == smsTargetNum1 ||
      smsNumber == smsTargetNum2 ||
      smsNumber == smsTargetNum3 ||
      smsNumber == smsTargetNum4 ||
      smsNumber == smsTargetNum5) smsMenu();
  else {
#ifdef debugMode
    Serial.println(F("SMS number not recognized."));
#endif
  }
}

void smsMenu() {
  int smsCommandPos = smsMessage.indexOf('@') + 1;
  if (smsCommandPos == -1) {
#ifdef debugMode
    Serial.println(F("Invalid command."));
#endif
  }
  else {
    int smsCommand = int(smsMessage.charAt(smsCommandPos)) - 48;
    switch (smsCommand) {
      case 0:
        Serial.println(F("0"));
        Serial.print(F("smsNumber:  "));
        Serial.println(smsNumber);
        Serial.print(F("smsMessage: "));
        Serial.println(smsMessage);
        /*
          Serial2.println(F("AT+CMGF=1"));
        delay(100);
        Serial2.print(F("\""));
        delay(100);
        Serial2.print(smsNumber);
        delay(100);
        Serial2.println(F("\""));
        delay(100);
        Serial2.print(F("http://maps.google.com/maps?q="));
        delay(100);
        Serial2.print(F("Datsun@"));
        delay(100);
        Serial2.print(gpsCoordString);
        delay(100);
        Serial2.print(F("&t=h&z=19&output=html /"));
        delay(100);
        Serial2.println((char)26);
        delay(100);
        Serial2.flush();

        if (!Serial2.available()) {
        while (!Serial2.available()) {
        delay(10);
        }
        }
        smsFlush();
        if (!Serial2.available()) {
        while (!Serial2.available()) {
        delay(10);
        }
        }
        smsFlush();*/
        break;
      case 1:
        Serial.println(F("1"));
        // LIVE CODE
        break;
      case 2:
        Serial.println(F("2"));
        // LIVE CODE
        break;
      default:
        break;
    }
  }
}

void smsStartup() {
  // Software power-on of gprs shield
  digitalWrite(smsPowerPin, LOW);
  delay(100);
  digitalWrite(smsPowerPin, HIGH);
  delay(500);
  digitalWrite(smsPowerPin, LOW);
  delay(100);

  delay(5000);

  const int smsStartupDelay = 8000;
  int smsDelayStart = millis();
  boolean firstLoop = true;
  for ( ; (millis() - smsDelayStart) < smsStartupDelay; ) {
    String startupBuffer = "";
    if (Serial2.available()) {
      while (Serial2.available()) {
        char c = Serial2.read();
#ifdef debugMode
        if (firstLoop == true) {
          Serial.println(F("SMS Buffer Flush:"));
          firstLoop = false;
        }
        Serial.println(startupBuffer);
#endif
        delay(100);
      }
      smsDelayStart = millis();
    }
  }
  // Configure GPRS output for proper parsing
  Serial2.println(F("ATE0"));
  delay(1000);
  Serial2.println(F("ATQ1"));
  delay(1000);
  Serial2.println(F("ATV0"));
  delay(1000);
  Serial2.println(F("AT+CMGF=1"));
  delay(1000);
  //Serial2.println(F("AT+CMGD=0,2"));  // Delete all saved texts
  //delay(1000);
  Serial2.flush();
  smsFlush();
}

void smsFlush() {
  if (Serial2.available()) {
    while (Serial2.available()) {
      //char c = Serial2.read();
      Serial2.write(Serial2.read());
      delay(100);
    }
  }
}
