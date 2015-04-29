"""
Phil's Car - 1973 Datsun 240Z
    - Request and receive data from ArduBerry

To Do:
    - Check function for sensor calib and call to calib if not
    - Same for date time (reg checks)
"""

import os
import sys
import time
from time import strftime, localtime
import datetime
import serial
"""
import xively

XIVELY_API_KEY = 'GR5ACBtf00P6nTxy0svr5Eqb3eWQgXOP7Pv7FtOQPWR7ZPG1'
XIVELY_FEED_ID = '69373229'
api = xively.XivelyAPIClient(XIVELY_API_KEY)
feed = api.feeds.get(XIVELY_FEED_ID)
"""

# Debug mode toggling
global debugMode
debugMode = True

# Serial communication channel to/from ArduBerry
global ser
ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=5)

# Local date and time
global month, day, year
global hour, minute, second
timeNow = strftime('%c', localtime())
hourNow = int(strftime('%H', localtime()))
minuteNow = int(strftime('%M', localtime()))

# Unparsed string data received from ArduBerry
global dateString, timeString
global gpsCoordString, gpsDataString
global accelerometerString, compassString, gyroscopeString
# Parsed data storage variables
global ardMonth, ardDay, ardYear
global ardHour, ardMinute, ardSecond
global dataUpdated
global gpsLat, gpsLon
global satellites, hdop
global gpsAltitudeFt, gpsSpeedMPH, gpsCourse
global accelX, accelY, accelZ
global compX, compY, compZ
global gyroX, gyroY, gyroZ

os.system("gpio mode 0 out")
os.system("gpio write 0 0")
time.sleep(1)

def st():
    time.sleep(.5)

def mainLoop():
    global dateString, timeString
    global gpsCoordString, gpsDataString
    global accelerometerString, compassString, gyroscopeString 
    while True:
        if debugMode == True:
            getSensorData(3)
            print dateString + ", " + timeString
            getSensorData(1)
            print gpsCoordString
            print gpsDataString
            getSensorData(2)
            print accelerometerString
            print compassString
            print gyroscopeString
            time.sleep(10)
    
def getSensorData(cmd):
    global ser
    global dateString, timeString
    global gpsCoordString, gpsDataString
    global accelerometerString, compassString, gyroscopeString

    if cmd == 1:
        ser.write('1')
        st()
        gpsCoordString = ser.readline().strip('\r\n')
        gpsDataString = ser.readline().strip('\r\n')
        
    elif cmd == 2:
        ser.write('2')
        st()
        accelerometerString = ser.readline().strip('\r\n')
        compassString = ser.readline().strip('\r\n')
        gyroscopeString = ser.readline().strip('\r\n')
        
    elif cmd == 3:
        ser.write('3')
        st()
        dateString = ser.readline().strip('\r\n')
        timeString = ser.readline().strip('\r\n')

def startupSerial():
    global ser
    setupComplete = False
    os.system("gpio write 0 1")
    time.sleep(1)
    while setupComplete == False:
        setupString = ser.readline().strip('\r\n')
        if setupString == "R":
            setupComplete = True
            print "Setup complete. Proceeding with program..."
        else:
            print setupString

startupSerial()
mainLoop()
