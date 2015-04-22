/*
  Control Unit (Phil's Car - 1973 Datsun 240Z)
  
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
  
*/

#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <TinyGPS.h>

const int gpsGmtOffsetHours = -5;  // Offset in hours of current timezone from GMT (I think it only accepts + vals b/c it gets converted to a 'byte')

int A = 0;
int B = 1;
int C = 2;
int D = 3;
int E = 4;

// Digital outputs
const int powerLED = A;  // FIX!!!!
const int gpsLED = B;  // FIX!!!!
const int smsPower = C;  // FIX!!!!
// Digital inputs
const int tripSwitch = D;  // FIX!!!!
const int calibSwitch = E;  // FIX!!!!

void setup() {
  Serial.begin(9600);
  if (!SD.begin(10, 11, 12, 13)) {
    boolean sdErrorBool = true;
    Serial.println(F("SD card failed to initialize."));
    while (sdErrorBool == true) {
      delay(5000);
      if (SD.begin(10, 11, 12, 13)) {
        Serial.println(F("SD card initialized."));
        sdErrorBool = false;
      }
    }
  }
}

void loop() {
  Serial.println(F("MAIN LOOP"));
  delay(1000);
}
