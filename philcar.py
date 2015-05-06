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

global xivelyHeader

debugMode = True
sensorDebug = False
global firstLoop
firstLoop = True
tripMode = False
global loopCount
loopCount = 0

dateTimeLong = strftime('%c', localtime())

if debugMode == True:
    print dateTimeLong
    print

XIVELY_API_KEY = 'MKPFAnS47P9FJAV2D7vw5M9MmHWdsEnj7zuCuJiaoyvua8jO'
XIVELY_FEED_ID = '1352564954'
api = xively.XivelyAPIClient(XIVELY_API_KEY)
feed = api.feeds.get(XIVELY_FEED_ID)

xivelyHeader = ['dataUpdated', 'gpsLat', 'gpsLon', 'satellites', 'hdop', 'gpsAltitudeFt', 'gpsSpeedMPH', 'gpsCourse']

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, 1)
GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

def mainLoop():
    global firstLoop
    global loopCount
    loopCount = loopCount + 1
    if firstLoop == True:
        captureImage()
        datastreams = xivelyGetDatastreams(feed)
        datastreams['dataUpdatedXively'].max_value = None
        datastreams['dataUpdatedXively'].min_value = None
        datastreams['gpsLatXively'].max_value = None
        datastreams['gpsLatXively'].min_value = None
        datastreams['gpsLonXively'].max_value = None
        datastreams['gpsLonXively'].min_value = None
        datastreams['satellitesXively'].max_value = None
        datastreams['satellitesXively'].min_value = None
        datastreams['hdopXively'].max_value = None
        datastreams['hdopXively'].min_value = None
        datastreams['gpsAltitudeFtXively'].max_value = None
        datastreams['gpsAltitudeFtXively'].min_value = None
        datastreams['gpsSpeedMPHXively'].max_value = None
        datastreams['gpsSpeedMPHXively'].min_value = None
        datastreams['gpsCourseXively'].max_value = None
        datastreams['gpsCourseXively'].min_value = None
        firstLoop = False

    sensorData = getSensorData(1)
    if debugMode == True:
        print xivelyHeader
        print sensorData
        print
    xivelyUpdate(sensorData)
    csvWriteData(sensorData)
    if loopCount == 5:
        captureImage()
        loopCount = 0

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

    if cmd is not 4:
        dataString = parseString(data, cmd)
        return dataString

def parseString(data, cmd):
    if cmd == 1:
        try:
            gpsLat, gpsLon = str(data[0]).split(",")
            dataUpdated, satellites, hdop, gpsAltitudeFt, gpsSpeedMPH, gpsCourse = str(data[1]).split(",")
            dataUpdated = dataUpdated.strip("'")
            gpsLat = gpsLat.strip("['")
            gpsLon = gpsLon.strip("'")
            gpsCourse = gpsCourse.strip("']")
            data = [dataUpdated, gpsLat, gpsLon, satellites, hdop, gpsAltitudeFt, gpsSpeedMPH, gpsCourse]
            if sensorDebug == True:
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
        except:
            print "Serial data unavailable or unparsable."
        return data
            

    elif cmd == 2:
        try:
            accelX, accelY, accelZ = str(data[0]).split(",")
            compX, compY, compZ = str(data[1]).split(",")
            gyroX, gyroY, gyroZ = str(data[2]).split(",")
            accelX = accelX.strip("['")
            accelZ = accelZ.strip("'")
            if sensorDebug == True:
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
        except:
            print "Serial data unavailable or unparsable."

    elif cmd == 3:
        try:
            contDate, contTime = str(data).split(",")
            contMonth, contDay, contYear = str(contDate).split("-")
            contHour, contMinute, contSecond = str(contTime).split(":")
            contMonth = contMonth.strip("['")
            contYear = contYear.strip("'")
            contHour = contHour.strip(" '")
            contSecond = contSecond.strip("']")
            if sensorDebug == True:
                print "CONTROL UNIT DATE/TIME"
                print "Month:  " + str(contMonth)
                print "Day:    " + str(contDay)
                print "Year:   " + str(contYear)
                print "Hour:   " + str(contHour)
                print "Minute: " + str(contMinute)
                print "Second: " + str(contSecond)
                print
        except:
            print "Serial data unavailable or unparsable."

    else:
        if debugMode == True:
            print "Invalid command."
            print

def xivelyUpdate(sensorData):
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
    localroot = '/home/phil/datsun/logs/'
    filename = timeStamp(2) + '.csv'
    filepath = localroot + filename
    try:
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
    except:
        if debugMode == True:
            print "Write to CSV log failed."


def captureImage():
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
        sshImageUpdate = 'cp -u ' + remotepath + ' ~/live.jpg'
        ssh.exec_command(sshImageUpdate)
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
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    while GPIO.input(12) == GPIO.LOW:
        setupString = ser.readline().strip('\r\n')
        st()
        if GPIO.input(12) == 0:
            if debugMode == True:
                print str(setupString)
    time.sleep(3)
    ser.flushInput()
    return True

def st():
    time.sleep(.5)

syncDelay()

while True:
    try:
        while GPIO.input(12) == GPIO.HIGH:
            if sensorDebug == True:
                for x in range(1, 4):
                    sensorData = getSensorData(x)
                    time.sleep(3)
                debugDataPrint()
            else:
                while tripMode == False and GPIO.input(12) == 1:
                    sleepTime = 60
                    mainLoop()
                    if debugMode == True:    
                        print "Data acquisition complete. Sleeping for " + str(sleepTime) + " seconds."
                        print
                    time.sleep(sleepTime)
    except KeyboardInterrupt or RuntimeError:
        print "Keyboard interrupt or runtime error detected. Resetting RPi GPIO."
        print
        GPIO.cleanup()
        syncDelay()
