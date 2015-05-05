import os
import sys
import time
from time import strftime, localtime
import datetime
import serial
import picamera
import paramiko
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
    if cmd == 1:
        try:
            gpsLat, gpsLon = str(data[0]).split(",")
            dataUpdated, hdop, satellites, gpsAltitudeFt, gpsSpeedMPH, gpsCourse = str(data[1]).split(",")
            dataUpdated = dataUpdated.strip("'")
            gpsLat = gpsLat.strip("['")
            gpsLon = gpsLon.strip("'")
            gpsCourse = gpsCourse.strip("']")
        except:
            print "Serial data unavailable or unparsable."
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
        try:
            accelX, accelY, accelZ = str(data[0]).split(",")
            compX, compY, compZ = str(data[1]).split(",")
            gyroX, gyroY, gyroZ = str(data[2]).split(",")
            accelX = accelX.strip("['")
            accelZ = accelZ.strip("'")
        except:
            print "Serial data unavailable or unparsable."
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
        try:
            contDate, contTime = str(data).split(",")
            contMonth, contDay, contYear = str(contDate).split("-")
            contHour, contMinute, contSecond = str(contTime).split(":")
            contMonth = contMonth.strip("['")
            contYear = contYear.strip("'")
            contHour = contHour.strip(" '")
            contSecond = contSecond.strip("']")
        except:
            print "Serial data unavailable or unparsable."
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
    XIVELY_API_KEY = 'MKPFAnS47P9FJAV2D7vw5M9MmHWdsEnj7zuCuJiaoyvua8jO'
    XIVELY_FEED_ID = '1352564954'
    api = xively.XivelyAPIClient(XIVELY_API_KEY)
    feed = api.feeds.get(XIVELY_FEED_ID)
    
    streamList = ['dataUpdated', 'gpsLat', 'gpsLon', 'satellites', 'hdop',
                  'gpsAltitudeFt', 'gpsSpeedMPH', 'gpsCourse']

    xivelyAccessFeed(feed, False)
    
    feed.datastreams = [
        xively.Datastreams(id=streamList[0], current_value=xivelyData[0], at=now),
        xively.Datastreams(id=streamList[1], current_value=xivelyData[1], at=now),
        xively.Datastreams(id=streamList[2], current_value=xivelyData[2], at=now),
        xively.Datastreams(id=streamList[3], current_value=xivelyData[3], at=now),
        xively.Datastreams(id=streamList[4], current_value=xivelyData[4], at=now),
        xively.Datastreams(id=streamList[5], current_value=xivelyData[5], at=now),
        xively.Datastreams(id=streamList[6], current_value=xivelyData[6], at=now),
        xively.Datastreams(id=streamList[7], current_value=xivelyData[7], at=now)
    ]
    feed.update()

def xivelyAccessFeed(feed, returnRequest):
    try:
        dataUpdated = feed.datastreams.get('dataUpdated')
    except:
        dataUpdated = feed.datastreams.create('dataUpdated', tags='dataUpdated')
        
    try:
        gpsLat = feed.datastreams.get('gpsLat')
    except:
        gpsLat = feed.datastreams.create('gpsLat', tags='gpsLat')
        
    try:
        gpsLon = feed.datastreams.get('gpsLon')
    except:
        gpsLon = feed.datastreams.create('gpsLon', tags='gpsLon')
        
    try:
        satellites = feed.datastreams.get('satellites')
    except:
        satellites = feed.datastreams.create('satellites', tags='satellites')
        
    try:
        hdop = feed.datastreams.get('hdop')
    except:
        hdop = feed.datastreams.create('hdop', tags='hdop')
        
    try:
        gpsAltitudeFt = feed.datastreams.get('gpsAltitudeFt')
    except:
        gpsAltitudeFt = feed.datastreams.create('gpsAltitudeFt', tags='gpsAltitudeFt')
        
    try:
        gpsSpeedMPH = feed.datastreams.get('gpsSpeedMPH')
    except:
        gpsSpeedMPH = feed.datastreams.create('gpsSpeedMPH', tags='gpsSpeedMPH')
        
    try:
        gpsCourse = feed.datastreams.get('gpsCourse')
    except:
        gpsCourse = feed.datastreams.create('gpsCourse', tags='gpsCourse')

    if returnRequest == True:
        return {'dataUpdatedXively':dataUpdated, 'gpsLatXively':gpsLat, 'gpsLonXively':gpsLon,
                'satellitesXively':satellites, 'hdopXively':hdop, 'gpsAltitudeFtXively':gpsAltitudeFt,
                'gpsSpeedMPHXively':gpsSpeedMPH, 'gpsCourseXively':gpsCourse}
    

def captureImage():
    localroot = '/home/phil/datsun/images/'
    remoteroot = '/home/datsun/images/'
    filename = timeStamp() + '.jpg'
    localpath = localroot + filename
    remotepath = remoteroot + filename
    
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        time.sleep(4)
        camera.capture(localpath)
        camera.stop_preview()

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect('162.218.209.90', username="datsun", password="amanojaku")
        sftp = ssh.open_sftp()
        sftp.put(localpath, remotepath)
        sftp.close()
        ssh.close()
        if debugMode == True:
            print "Image capture uploaded to server."
    except:
        if debugMode == True:
            print "Image capture upload failed."

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
        st()
        if GPIO.input(12) == 0:
            print str(setupString)
    time.sleep(3)
    ser.flushInput()

def st():
    time.sleep(.5)

syncDelay()
mainLoop()
