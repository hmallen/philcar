import os
import sys
import time
from time import strftime, localtime
import datetime
import serial
import picamera

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print "Error importing RPi.GPIO! This is probably because you need superuser privileges. You can achieve this by using 'sudo' to run your script."
import xively

debugMode = True

dateTimeLong = strftime('%c', localtime())

if debugMode == True:
    print dateTimeLong
    print

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, 1)
GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

global dataUpdated
global satellites, gpsLat, gpsLon, gpsAltitudeFt, gpsSpeedMPH, gpsCourse
global accelX, accelY, accelZ
global compX, compY, compZ
global gyroX, gyroY, gyroZ
global contMonth, contDay, contYear, contHour, contMinute, contSecond

def mainLoop():
    try:
        while True:
            if GPIO.input(12) == 0:
                syncDelay()
            for x in range(1, 4):
                sensorData = getSensorData(x)
                
                time.sleep(3)
            print "Data acquisition complete. Sleeping for 5 seconds."
            print
            time.sleep(5)
    except KeyboardInterrupt or RuntimeError:
        print "Keyboard interrupt or runtime error detected. Resetting RPi GPIO."
        print
        GPIO.cleanup()
    
def getSensorData(cmd):
    data = []
    
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5)

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

    elif cmd == 4:
        ser.write('4')
        st()
        captureImage()
        if debugMode == True:
            print "IR LED check performed. Capturing image."
        
    if cmd != 4:
        parseString(data, cmd)

def parseString(data, cmd):
    global dataUpdated
    global gpsLat, gpsLon
    global satellites, hdop, gpsAltitudeFt, gpsSpeedMPH, gpsCourse
    global accelX, accelY, accelZ
    global compX, compY, compZ
    global gyroX, gyroY, gyroZ
    global contMonth, contDay, contYear, contHour, contMinute, contSecond

    if cmd == 1:
        gpsLat, gpsLon = str(data[0]).split(",")
        dataUpdated, hdop, satellites, gpsAltitudeFt, gpsSpeedMPH, gpsCourse = str(data[1]).split(",")
        dataUpdated = dataUpdated.strip("'")
        gpsLat = gpsLat.strip("['")
        gpsLon = gpsLon.strip("'")
        gpsCourse = gpsCourse.strip("']")
        if debugMode == True:
            print "GPS"
            print "Data Updated:  " + str(dataUpdated)
            print "GPS Latitude:  " + str(gpsLat)
            print "GPS Longitude: " + str(gpsLon)
            print "Satellites:    " + str(satellites)
            print "HDOP:          " + str(hdop)
            print "Altitude (ft): " + str(gpsAltitudeFt)
            print "Speed (MPH):   " + str(gpsSpeedMPH)
            print "Course:        " + str(gpsCourse)
            print
        xivelyData = [int(dataUpdated), float(gpsLat), float(gpsLon), int(satellites), int(hdop),
                      float(gpsAltitudeFt), float(gpsSpeedMPH), float(gpsCourse)]
        try:
            xivelyUpdate(xivelyData)
            if debugMode == True:
                print "Data successfully uploaded to Xively."
                print
        except:
            if debugMode == True:
                print "Data failed to upload to Xively."
                print
        
    elif cmd == 2:
        accelX, accelY, accelZ = str(data[0]).split(",")
        compX, compY, compZ = str(data[1]).split(",")
        gyroX, gyroY, gyroZ = str(data[2]).split(",")
        accelX = accelX.strip("['")
        accelZ = accelZ.strip("'")
        if debugMode == True:
            print "ACCELEROMETER"
            print "X: " + str(accelX)
            print "Y: " + str(accelY)
            print "Z: " + str(accelZ)
            print "COMPASS"
            compX = compX.strip("'")
            compZ = compZ.strip("'")
            print "X: " + str(compX)
            print "Y: " + str(compY)
            print "Z: " + str(compZ)
            print "GYROSCOPE"
            gyroX = gyroX.strip("'")
            gyroZ = gyroZ.strip("']")
            print "X: " + str(gyroX)
            print "Y: " + str(gyroY)
            print "Z: " + str(gyroZ)
            print
        
    elif cmd == 3:
        contDate, contTime = str(data).split(",")
        contMonth, contDay, contYear = str(contDate).split("-")
        contHour, contMinute, contSecond = str(contTime).split(":")
        contMonth = contMonth.strip("['")
        contYear = contYear.strip("'")
        contHour = contHour.strip(" '")
        contSecond = contSecond.strip("']")
        if debugMode == True:
            print "CONTROL UNIT DATE/TIME"
            print "Month:  " + str(contMonth)
            print "Day:    " + str(contDay)
            print "Year:   " + str(contYear)
            print "Hour:   " + str(contHour)
            print "Minute: " + str(contMinute)
            print "Second: " + str(contSecond)
            print
        
    else:
        if debugMode == True:
            print "Invalid command."
            print

def xivelyUpdate(xivelyData):
    api = xively.XivelyAPIClient("MKPFAnS47P9FJAV2D7vw5M9MmHWdsEnj7zuCuJiaoyvua8jO")
    feed = api.feeds.get(1352564954)
    streamList = [dataUpdated, gpsLat, gpsLon, satellites, hdop,
                  gpsAltitudeFt, gpsSpeedMPH, gpsCourse]
    feed.datastreams = [
        xively.Datastreams(id=streamList[0], current_value=, at=now),
        xively.Datastreams(id=streamList[1], current_value=, at=now),
        xively.Datastreams(id=streamList[2], current_value=, at=now),
        xively.Datastreams(id=streamList[3], current_value=, at=now),
        xively.Datastreams(id=streamList[4], current_value=, at=now),
        xively.Datastreams(id=streamList[5], current_value=, at=now),
        xively.Datastreams(id=streamList[6], current_value=, at=now),
        xively.Datastreams(id=streamList[7], current_value=, at=now)
    ]

def captureImage():
    localroot = '/home/phil/datsun/'
    filename = localroot + timeStamp() + '.jpg'
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        time.sleep(4)
        camera.capture(filename)
        camera.stop_preview()

def timeStamp():
    month = strftime('%m', localtime())
    day = strftime('%d', localtime())
    year = strftime('%y', localtime())
    hour = strftime('%H', localtime())
    minute = strftime('%M', localtime())
    second = strftime('%S', localtime())
    currentDateTime = month + day + year + "-" + hour + minute + second
    return currentDateTime

def syncDelay():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    while GPIO.input(12) == 0:
        setupString = ser.readline().strip('\r\n')
        if GPIO.input(12) == 0:
            print str(setupString)
    time.sleep(3)
    ser.flushInput()

def st():
    time.sleep(.5)

syncDelay()
mainLoop()
