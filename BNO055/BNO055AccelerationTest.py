#IMPORT STATEMENTS
import logging
import sys
import time
import math
import datetime
import sys
from os import system, remove
import os
import Adafruit_BMP.BMP085 as BMP085
#import Gnuplot #, Gnuplot.funcutils

from Adafruit_BNO055 import BNO055

#Base point for altitude
pad_alt=0

#=============== FUNCTIONS ======================
def zero_bmp180():
    pad = bmp180.read_altitude()
    pad += bmp180.read_altitude()
    pad += bmp180.read_altitude()
    pad += bmp180.read_altitude()
    pad = pad/4
    print("bmp180 zero at {} m".format(pad_alt))
    return pad

def check_bmp180():
    print("\nSensitivity mode = {}".format(bmp180._mode))
    print("Temp = {0:0.2F} *C".format(bmp180.read_temperature()))
    print("Pressure = {0:0.2F} Pa".format(bmp180.read_pressure()))
    print("Altitude (ASL) = {0:0.2F} m".format(bmp180.read_altitude()))
    print("Altitude (AGL) = {0:0.2F} m".format(bmp180.read_altitude()-pad_alt))
    print("Sealevel Pressure = {0:0.2F} Pa \n".format(bmp180.read_sealevel_pressure()))

def storeData(whatFile, source) : #FUNCTION USED TO STORE DATA TO A FILE
    if (source == "BNO055"):
        whatFile.write('{0},{1},{2:02F}\n'.format(source,times[0],accelerationFromBNO[0]))
    else:
        whatFile.write('{0},{1},{1:02F}\n'.format(source,times[0],altitudes[0]))
        
def collectData(): #FUNCTION USED TO COLLECT DATA
    dataFileString = 'sensorData{}.dat'.format(datetime.datetime.now().strftime("%y_%m_%d_%H_%M"))
    dataFile = open(dataFileString, "w")

    tmpFile = open('tmpPlotCmd.gp', "w")
    print os.getcwd()
    tmpFile.write('plot %s/%s"'%(os.getcwd(), dataFileString))
    tmpFile.close()

    #Variable for counting the number of readings taken
    loopCount = 0

    try:    
        while True:
            # Read the Euler angles for heading, roll, pitch(all in degrees).
            heading, roll, pitch = bno.read_euler()
            # Read the calibration status, 0 = uncalibrated and 3=fully calibrated.
            sys, gyro, accel, mag = bno.get_calibration_status()
            # Print everything out
            print('Heading={0:0.2F} Roll={1:0.2F} Pitch{2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
                  heading, roll, pitch, sys, gyro, accel, mag))
            xa, ya, za = bno.read_linear_acceleration()
            xg, yg, zg = bno.read_gravity()

            print('Current Linear Accelaration: x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xa,ya,za))
            print('Current Accelaration due to gravity: x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xg,yg,zg))

            adotg = ((xa*xg)+(ya*yg)+(za*zg))
            
            #calulation to find the gradient of linear acceleration onto the acceleration of gravity
            gradient = adotg/(((xg**2)+(yg**2)+(zg**2)))

            #Calculations to find each component of acceleration using the gradient of linear accerlation onto the accerlation of gravity
            xComponentOfAcceleration = gradient*xg
            yComponentOfAcceleration = gradient*yg
            zComponentOfAcceleration = gradient*zg

            #Final calculation finding the magnitude of accerlation in the direction of gravity (e.g. the upwards
            #linear accerlation regardless of orientation of the sensor.)
            magnitudeOfAccelerationInZDirection = (((xComponentOfAcceleration**2)+(yComponentOfAcceleration**2)+(zComponentOfAcceleration**2))**0.5)
            print('Current Acceleration in the Z direction (vector): x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xComponentOfAcceleration,yComponentOfAcceleration,zComponentOfAcceleration))
            print('Magnitude of acceleration in the Z Direction = {0:0.2F}'.format(magnitudeOfAccelerationInZDirection))

            #Calculate theta using gradient -- cos(theta) = gradient
            print("The gradient equals {0}".format(gradient))
            theta = math.acos(gradient)
            #Convert theta into degrees
            theta = (theta*180)/ math.pi

            #test if theta is greater than 90 or and acceleration is negative, if it is assume magnitude of acceleration
            #is negative
            if theta > 90 and theta < 270:
                currentAccerlationInZDirection = -1*magnitudeOfAccelerationInZDirection
            else :
                currentAccerlationInZDirection = magnitudeOfAccelerationInZDirection

            print('Current accerlation in the Z Direction = {0:0.2F} \n\n'.format(currentAccerlationInZDirection))

            #UPDATE TIME AND ACCELERATION LISTS
            times.insert(0,int(round(time.time() * 1000)) - programStartTime)
            accelerationFromBNO.insert(0, currentAccerlationInZDirection)
            altitudes.insert(0, bmp180.read_altitude() - pad_alt)


            if loopCount < (NUM_OF_READINGS+1): #If not enough readings, do nothing
                loopCount += 1
            else: #If NUM_OF_READINGS has been met, kick oldest reading out
                times.pop()
                accelerationFromBNO.pop()
                altitudes.pop()
                storeData(dataFile,"BNO055")
                storeData(dataFile,"BMP100")
            
                        
            time.sleep(.1)

    except KeyboardInterrupt: #Hit ctrl + c to end loop
        dataFile.close()
        
        system('gnuplot - persist tmpPlotCmd.gp')
        remove('tmpPlotCmd.gp')
        pass
#=============== END FUNCTIONS ==================




#=============== SETUP ===========================
#=============== VARIABLE SETUP =================
#Number of readings stored in each vector
NUM_OF_READINGS = 9

#Zero the time
programStartTime = int(round(time.time()*1000))

#Create list for acceleration readings
accelerationFromBNO = [None] * NUM_OF_READINGS

#Create list for altitude readings
altitudes = [None] * NUM_OF_READINGS

#Create list for times
times = [None] * NUM_OF_READINGS

#=============== END VARIABLE SETUP =============

#=============== PARAMETER SETUP =================
BMP180_SENS = 3 #0,1,2,3
#============ END PARAMETER SETUP ================

#=============== BNO055 SETUP ===================
#Create and configure the BNO sensor connection.
#Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO05! Is the sensor connected?')

#Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

#print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
#=============== END BNO055 SETUP ===============

#=============== BMP180 SETUP ===================
bmp180 = BMP085.BMP085()
bmp180.__init__(mode = BMP180_SENS)  # set sensitivity

pad_alt = zero_bmp180()

check_bmp180

#============= END BMP180 SETUP =================

#=============== FILE SETUP =====================

#=============== END FILE SETUP =================
#=============== END SETUP =======================


#===============MAIN PROGRAM=====================
while True:
    collectData()
    sys.exit()
    #TODO!!!!! YOU NEED TO WRITE TO TWO FILES.  ONE FOR BMP100 READINGS AND ONE FOR CURRENT ACCELERATION !!!!
    #SEE VDS_Tronics_1_0.py AS A REFERENCE!!!

    # Other values you can optionally read:
    # Orientation as a quarterion:
    #x,y,z,w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer
    # Gyroscope data (in degrees per second)
    #x,y,z = bno.read_gyroscope
    # Accelerometer data (in meters per second squared):
    #x,y,z = bno.read_accelerometer()
    # Linear accerlation data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_accerlation()
    # Gravity accerlation data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.time.sleep(1)
