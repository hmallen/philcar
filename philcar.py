import os
import sys
import time
from time import strftime, localtime
import datetime
import serial
import picamera
import paramiko
import xively
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print "Error importing RPi.GPIO! This is probably because you need superuser privileges. You can achieve this by using 'sudo' to run your script."

#global debugMode
#global sensorDebug
global firstLoop
global tripMode
debugMode = True
sensorDebug = False
firstLoop = True
tripMode = False

dateTimeLong = strftime('%c', localtime())

if debugMode == True:
    print dateTimeLong
    print

global feed
XIVELY_API_KEY = 'MKPFAnS47P9FJAV2D7vw5M9MmHWdsEnj7zuCuJiaoyvua8jO'
XIVELY_FEED_ID = '1352564954'
api = xively.XivelyAPIClient(XIVELY_API_KEY)
feed = api.feeds.get(XIVELY_FEED_ID)
global xivelyHeader
xivelyHeader = ['dataUpdated', 'gpsLat', 'gpsLon', 'satellites', 'hdop', 'gpsAltitudeFt', 'gpsSpeedMPH', 'gpsCourse']

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, 1)
GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

def mainLoop():
    global firstLoop
    global feed
    if firstLoop == True:
        datastreams = xivelyGetDatastreams(feed)
        datastreams['dataUpdated'].max_value = None
        datastreams['dataUpdated'].min_value = None
        datastreams['gpsLat'].max_value = None
        datastreams['gpsLat'].min_value = None
        datastreams['gpsLon'].max_value = None
        datastreams['gpsLon'].min_value = None
        datastreams['satellites'].max_value = None
        datastreams['satellites'].min_value = None
        datastreams['hdop'].max_value = None
        datastreams['hdop'].min_value = None
        datastreams['gpsAltitudeFt'].max_value = None
        datastreams['gpsAltitudeFt'].min_value = None
        datastreams['gpsSpeedMPH'].max_value = None
        datastreams['gpsSpeedMPH'].min_value = None
        datastreams['gpsCourse'].max_value = None
        datastreams['gpsCourse'].min_value = None
        firstLoop = False

    getSensorData(1)
    xivelyUpdate(sensorData)
    csvWriteData(sensorData)

def getSensorData(cmd):
    global debugMode
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
    global debugMode
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
            print "Data Updated:  " + dataUpdated
            print "GPS Latitude:  " + gpsLat
            print "GPS Longitude: " + gpsLon
            print "Satellites:    " + satellites
            print "HDOP:          " + hdop
            print "Altitude (ft): " + gpsAltitudeFt
            print "Speed (MPH):   " + gpsSpeedMPH
            print "Course:        " + gpsCourse
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
    global debugMode
    global feed
    datastreams = xivelyGetDatastreams(feed)
    try:
        datastreams['dataUpdatedXively'].current_value = sensorData[0]
        datastreams['dataUpdatedXively'].at = datetime.datetime.utcnow()
        datastreams['dataUpdatedXively'].update()

        datastreams['gpsLatXively'].current_value = sensorData[1]
        datastreams['gpsLatXively'].at = datetime.datetime.utcnow()
        datastreams['gpsLatXively'].update()

        datastreams['gpsLonXively'].current_value = sensorData[2]
        datastreams['gpsLonXively'].at = datetime.datetime.utcnow()
        datastreams['gpsLonXively'].update()

        datastreams['satellitesXively'].current_value = sensorData[3]
        datastreams['satellitesXively'].at = datetime.datetime.utcnow()
        datastreams['satellitesXively'].update()

        datastreams['hdopXively'].current_value = sensorData[4]
        datastreams['hdopXively'].at = datetime.datetime.utcnow()
        datastreams['hdopXively'].update()

        datastreams['gpsAltitudeFtXively'].current_value = sensorData[5]
        datastreams['gpsAltitudeFtXively'].at = datetime.datetime.utcnow()
        datastreams['gpsAltitudeFtXively'].update()

        datastreams['gpsSpeedMPHXively'].current_value = sensorData[6]
        datastreams['gpsSpeedMPHXively'].at = datetime.datetime.utcnow()
        datastreams['gpsSpeedMPHXively'].update()

        datastreams['gpsCourseXively'].current_value = sensorData[7]
        datastreams['gpsCourseXively'].at = datetime.datetime.utcnow()
        datastreams['gpsCourseXively'].update()
    except:
        if debugMode == True:
            print "Upload of data to Xively failed."
            print

def xivelyGetDatastreams(feed):
    global debugMode
    try:
        dataUpdated = feed.datastreams.get('dataUpdated')
    except:
        dataUpdated = feed.datastreams.create('dataUpdated', tags='dataUpdated')
        if debugMode == True:
            print "dataUpdated datastream created."
    try:
        gpsLat = feed.datastreams.get('gpsLat')
    except:
        gpsLat = feed.datastreams.create('gpsLat', tags='gpsLat')
        if debugMode == True:
            print "gpsLat datastream created."
    try:
        gpsLon = feed.datastreams.get('gpsLon')
    except:
        gpsLon = feed.datastreams.create('gpsLon', tags='gpsLon')
        if debugMode == True:
            print "gpsLon datastream created."
    try:
        satellites = feed.datastreams.get('satellites')
    except:
        satellites = feed.datastreams.create('satellites', tags='satellites')
        if debugMode == True:
            print "satellites datastream created."
    try:
        hdop = feed.datastreams.get('hdop')
    except:
        hdop = feed.datastreams.create('hdop', tags='hdop')
        if debugMode == True:
            print "hdop datastream created."
    try:
        gpsAltitudeFt = feed.datastreams.get('gpsAltitudeFt')
    except:
        gpsAltitudeFt = feed.datastreams.create('gpsAltitudeFt', tags='gpsAltitudeFt')
        if debugMode == True:
            print "gpsAltitudeFt datastream created."
    try:
        gpsSpeedMPH = feed.datastreams.get('gpsSpeedMPH')
    except:
        gpsSpeedMPH = feed.datastreams.create('gpsSpeedMPH', tags='gpsSpeedMPH')
        if debugMode == True:
            print "gpsSpeedMPH datastream created."
    try:
        gpsCourse = feed.datastreams.get('gpsCourse')
    except:
        gpsCourse = feed.datastreams.create('gpsCourse', tags='gpsCourse')
        if debugMode == True:
            print "gpsCourse datastream created."

    return {'dataUpdatedXively':dataUpdated, 'gpsLatXively':gpsLat, 'gpsLonXively':gpsLon,
            'satellitesXively':satellites, 'hdopXively':hdop, 'gpsAltitudeFtXively':gpsAltitudeFt,
            'gpsSpeedMPHXively':gpsSpeedMPH, 'gpsCourseXively':gpsCourse}

def csvWriteData(sensorData):
    global xivelyHeader
    localroot = '/home/pi/mystall-client/logs/'
    filename = timeStamp(2) + '.csv'
    filepath = localroot + filename
    
    if os.path.exists(filepath):
        f = open(filepath, 'a')
    else:
        f = open(filepath, 'a+')
        
    for element in xivelyHeader:
        f.write(element + ',')
        f.write('\n')

    for element in sensorData:
        if type(element) == str:
            f.write(element + ',')
        if type(element) == list:
            for i in element:
                f.write(i + ',')
    
    f.write('\n')
    f.close()    


def captureImage():
    global debugMode
    localroot = '/home/phil/datsun/images/'
    remoteroot = '/home/datsun/images/'
    filename = timeStamp(1) + '.jpg'
    localpath = localroot + filename
    remotepath = remoteroot + filename

    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        time.sleep(4)
        camera.capture(localpath)
        camera.stop_preview()
        if debugMode == True:
            print "Image captured."

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect('162.218.209.90', username="datsun", password="amanojaku")
        sftp = ssh.open_sftp()
        sftp.put(localpath, remotepath)
        sftp.close()
        ssh.close()
        if debugMode == True:
            print "Uploaded to server."
            print
    except:
        if debugMode == True:
            print "FTP upload failed."
            print

def timeStamp(cmd):
    month = strftime('%m', localtime())
    day = strftime('%d', localtime())
    year = strftime('%y', localtime())
    hour = strftime('%H', localtime())
    minute = strftime('%M', localtime())
    second = strftime('%S', localtime())
    if cmd == 1:
        currentDateTime = month + day + year + "-" + hour + minute + second
    elif cmd == 2:
        currentDateTime = month + day + year
    else:
        if debugMode == True:
            print "Invalid timeStamp() request command."
            print
    return currentDateTime

def syncDelay():
    global debugMode
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    while GPIO.input(12) == 0:
        setupString = ser.readline().strip('\r\n')
        st()
        if GPIO.input(12) == 0:
            if debugMode == True:
                print str(setupString)
    time.sleep(3)
    ser.flushInput()

def st():
    time.sleep(.5)

syncDelay()

while True:
    global debugMode
    global sensorDebug
    try:
        while GPIO.input(12) == 1:
            if sensorDebug == True:
                for x in range(1, 4):
                    sensorData = getSensorData(x)
                    time.sleep(3)
                debugDataPrint()
            else:
                while tripMode == False:
                    sleepTime = 60
                    mainLoop()
                    if debugMode == True:    
                        print "Data acquisition complete. Sleeping for " + str(sleepTime) + " seconds."
                        print
                    time.sleep(sleepTime)
        syncDelay()
    except KeyboardInterrupt or RuntimeError:
        print "Keyboard interrupt or runtime error detected. Resetting RPi GPIO."
        print
        GPIO.cleanup()
    
