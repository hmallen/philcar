import os
import sys
import time
from time import strftime, localtime
import datetime
import serial
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print "Error importing RPi.GPIO! This is probably because you need superuser privileges. You can achieve this by using 'sudo' to run your script."
#import xively

"""
XIVELY_API_KEY = 'GR5ACBtf00P6nTxy0svr5Eqb3eWQgXOP7Pv7FtOQPWR7ZPG1'
XIVELY_FEED_ID = '69373229'
api = xively.XivelyAPIClient(XIVELY_API_KEY)
feed = api.feeds.get(XIVELY_FEED_ID)
"""

debugMode = True

dateTimeNow = strftime('%c', localtime())

if debugMode == True:
    print dateTimeNow

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, 1)
GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)


def mainLoop():
    """
    try:
        # Main functions
    except KeyboardInterrupt:
        print "Keyboard interrupt detected. Resetting RPi GPIO."
        GPIO.cleanup()
    """
        
    while True:
        if GPIO.input(12) == 0:
            syncDelay()
        for x in range(1, 4):
            sensorData = getSensorData(x)
            print "Sensor data: " + str(sensorData) + " (" + str(x) + ")"
            time.sleep(3)
        print "Data acquisition complete. Sleeping for 5 seconds."
        time.sleep(5)
    
def getSensorData(cmd):
    data = []

    ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=5)

    if cmd == 1:
        ser.write('1')
        st()
        gpsCoordString = ser.readline().strip('\r\n')
        data.append(gpsCoordString)
        gpsDataString = ser.readline().strip('\r\n')
        data.append(gpsDataString)
        
    elif cmd == 2:
        ser.write('2')
        st()
        accelerometerString = ser.readline().strip('\r\n')
        data.append(accelerometerString)
        compassString = ser.readline().strip('\r\n')
        data.append(compassString)
        gyroscopeString = ser.readline().strip('\r\n')
        data.append(gyroscopeString)
        
    elif cmd == 3:
        ser.write('3')
        st()
        dateString = ser.readline().strip('\r\n')
        data.append(dateString)
        timeString = ser.readline().strip('\r\n')
        data.append(timeString)

    return data

def syncDelay():
    ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=5)
    while GPIO.input(12) == 0:
        setupString = ser.readline().strip('\r\n')
        if GPIO.input(12) == 0:
            print str(setupString)

def st():
    time.sleep(.5)

syncDelay()
mainLoop()
