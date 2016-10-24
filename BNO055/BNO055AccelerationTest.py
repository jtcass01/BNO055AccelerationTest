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
from Adafruit_BNO055 import BNO055

#Base point for altitude
pad_alt=0

#=============== FUNCTIONS ======================
    #=========== BEGIN BMP SPECIFIC FUNCTIONS====
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
    #============ END BMP SPECIFIC FUNCTIONS=====
    
    #=========== BEGIN BNO SPECIFIC FUNCTIONS====
def set2GRange(self):  #BNO055 Function used to set precision of Accelerometer
    BNO055_ACC_CONFIG_ADDR = 0X08
        
    #switch to config mode
    self._config_mode()

    #Set acceleration sensitivity to G2
    self._write_byte(BNO055_ACC_CONFIG_ADDR,0X0C)

    #switch back to operation mode
    self._operation_mode()
    print("BNO acceleration configured to 2G")
    
def calibrateBNO():
    sys, gyro, accel, mag = bno.get_calibration_status()

    print("Calibrating BNO055...")

    calibrationCount = 0

    while(calibrationCount < 5):
        sys, gyro, accel, mag = bno.get_calibration_status()

        # Read the calibration status, 0 = uncalibrated and 3=fully calibrated.
        print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sys, gyro, accel, mag))

        if accel > 2:
            calibrationCount += 1
        else:
            calibrationCount = 0
            
        time.sleep(1)

    print("BNO055 fully calibrated, calibration file has been updated.")
    calibrationFile = open("CalibrationFile.dat","w")
    for item in bno.get_calibration():
        calibrationFile.write("{},".format(str(item)))

def getInts(text):
    integer = 0
    count = 0
    output = list()
    for i in range(0,len(text),1):
        if(text[i] == ','):
            count = 0
            output.insert(0,integer)
        else:
            if(count == 0):
                integer = int(text[i])
                count += 1
            else:
                integer *= 10
                integer += int(text[i])
    return output

def setCalibration():  ##TO DO!! FIX THIS FUNCTION
    print("Calibrating BNO055 using CalibrationFile.dat")
    fileData = open("CalibrationFile.dat")
    inputLine = fileData.readline()
    calibrationData = list()

    calibrationData = getInts(inputLine)

    bno.set_calibration(calibrationData)        

    sys, gyro, accel, mag = bno.get_calibration_status()

    # Read the calibration status, 0 = uncalibrated and 3=fully calibrated.
    print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sys, gyro, accel, mag))
    
    #============ END BNO SPECIFIC FUNCTIONS=====

    #========= BEGIN GENERAL FUNCTIONS ==========
def storeData(whatFile, source) : #FUNCTION USED TO STORE DATA TO A FILE
    if (source == "BNO055"):
        whatFile.write('{0},{1},{2:0.3F}\n'.format(source,BNOTimes[0],accelerationFromBNO[0]))
    else:
        whatFile.write('{0},{1},{2:0.3F}\n'.format(source,BNOTimes[0],altitudes[0]))

def calulateVelocity():
    #Variables needed for caluation
    sumBMPTimes = 0
    sumBMPTimes2 = 0
    sumAlt = 0
    sumAltTimes = 0
    leftSide = 0
    rightSide = 0
    
    #Find sums for BMP
    for i in range(0, NUM_ALT_READINGS):
        sumBMPTimes += BMPTimes[i]
        sumBMPTimes2 += (BMPTimes[i]**2)
        sumAlt += altitudes[i]
        sumAltTimes += (altitudes[i] * BMPTimes[i])        

    #Calulate left side of equation
    leftSide =  ((sumBMPTimes * sumAlt) - (NUM_ALT_READINGS * sumAltTimes)) / (((sumBMPTimes)**2) - (NUM_ALT_READINGS* sumBMPTimes2))

    #Calculate rightSide of equation
    for i in range((NUM_ACCEL_READINGS/2),(NUM_ACCEL_READINGS-1),1):
        rightSide += (.5*(accelerationFromBNO[i] + accelerationFromBNO[i+1])*(BNOTimes[i+1]-BNOTimes[i]))

    return (leftSide + rightSide)

def collectData(): #FUNCTION USED TO COLLECT DATA

    #===============Collect Data Setup===============    
    #Zero the time
    programStartTime = int(round(time.time()*1000))

    JakeDataFileString = 'JakeSensorData{}.dat'.format(datetime.datetime.now().strftime("%y_%m_%d_%H_%M"))
    JakeDataFile = open(JakeDataFileString, "w")
    JakeDataFile.write("source,time,reading\n")

    errorFileString = 'errorFile{}.dat'.format(datetime.datetime.now().strftime("%y_%m_%d_%H_%M"))
    errorFile = open(errorFileString, "w")
    errorFile.write("time, error\n")

    BenDataFileString = 'BenSensorData{}.dat'.format(datetime.datetime.now().strftime("%y_%m_%d_%H_%M"))
    BenDataFile = open(BenDataFileString, "w")
    BenDataFile.write("time,altitude,acceleration\n")

    #Variable for counting the number of readings taken
    loopCount = 0
    #=============End Collect Data Setup=============

    try:    
        while True:
            sys, gyro, accel, mag = bno.get_calibration_status()

            if accel > 2:
                # Read the Euler angles for heading, roll, pitch(all in degrees).
                heading, roll, pitch = bno.read_euler()

                # Read the linear acceleration and acceleration of gravity vector components in m/(s^2)
                xa, ya, za = bno.read_linear_acceleration()
                xg, yg, zg = bno.read_gravity()

                #Find magnitudes of linear acceleration and gravity
                magnitudeOfLAccel = ((xa**2)+(ya**2)+(za**2))**.5
                magnitudeOfGrav = ((xg**2)+(yg**2)+(zg**2))**.5

                print('Current Linear Accelaration: x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xa,ya,za))
                print('Current Accelaration due to gravity: x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xg,yg,zg))

                #Calculate the dot product of linear acceleration and gravity
                adotg = ((xa*xg)+(ya*yg)+(za*zg))
                
                #calulation to find the component of linear acceleration in the direction of gravity
                componentOfAinG = adotg/(((xg**2)+(yg**2)+(zg**2)))

                #Calculations to find each component of acceleration using the gradient of linear accerlation onto the accerlation of gravity
                xComponentOfAcceleration = componentOfAinG*xg
                yComponentOfAcceleration = componentOfAinG*yg
                zComponentOfAcceleration = componentOfAinG*zg

                #Final calculation finding the magnitude of accerlation in the direction of gravity (e.g. the upwards
                #linear accerlation regardless of orientation of the sensor.)
                magnitudeOfAccelerationInZDirection = (((xComponentOfAcceleration**2)+(yComponentOfAcceleration**2)+(zComponentOfAcceleration**2)))
                print('Current Acceleration in the Z direction (vector): x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xComponentOfAcceleration,yComponentOfAcceleration,zComponentOfAcceleration))
                print('Magnitude of acceleration in the Z Direction = {0:0.2F}'.format(magnitudeOfAccelerationInZDirection))

                #Calculate theta using definition of dot product -- cos(theta) = the dot product of LA and G over their magnitudes
                defOfDotProduct = adotg / (magnitudeOfLAccel*magnitudeOfGrav)
                print("The definition of dot product give us cos(theta) = {0}".format(defOfDotProduct))
                theta = math.acos(defOfDotProduct)
                #Convert theta into degrees from radians
                theta = (theta*180)/ math.pi

                #test if theta is greater than 90 or and acceleration is negative, if it is assume magnitude of acceleration
                #is negative
                if theta > 90 and theta < 270:
                    currentAccerlationInZDirection = magnitudeOfAccelerationInZDirection
                else :
                    currentAccerlationInZDirection = -1*magnitudeOfAccelerationInZDirection

                print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
                print("Theta: {0:0.2F}".format(theta))
                print('Current accerlation in the Z Direction = {0:0.3F} \n\n'.format(currentAccerlationInZDirection))

                #UPDATE TIME AND ACCELERATION LISTS
                BNOTimes.insert(0,int(round(time.time() * 1000)) - programStartTime)
                accelerationFromBNO.insert(0, currentAccerlationInZDirection)
                altitudes.insert(0, bmp180.read_altitude() - pad_alt)


                if (loopCount < (NUM_ACCEL_READINGS+1)) or (loopCount < (NUM_ALT_READINGS+1)): #If not enough readings, do nothing
                    loopCount += 1
                else: #If NUM_OF_READINGS has been met, kick oldest reading out
                    BNOTimes.pop()
                    accelerationFromBNO.pop()
                    altitudes.pop()
                    storeData(JakeDataFile,"BNO055")
                    storeData(JakeDataFile,"BMP100")
                    BenDataFile.write('{0},{1},{2:0.3F}\n'.format(BNOTimes[0],altitudes[0],accelerationFromBNO[0]))
            else:
                BNOTimes.insert(0,int(round(time.time() * 1000)) - programStartTime)
                print("{0},BNO055's Accelerometer is not calibrated.".format(BNOTimes[0]))
                
                if loopCount < (NUM_OF_READINGS+1): #If not enough readings, do nothing
                    loopCount += 1
                else: #If NUM_OF_READINGS has been met, kick oldest reading out
                    BNOTimes.pop()
                    errorFile.write("{0},BNO055's Accelerometer is not calibrated.".format(BNOTimes[0]))

    except KeyboardInterrupt: #Hit ctrl + c to end loop
        JakeDataFile.close()
        BenDataFile.close()
        errorFile.close()
        pass
    #========== END GENERAL FUNCTIONS ===========


#=============== SETUP ===========================
#=============== VARIABLE SETUP =================
#Number of readings stored in each vector
NUM_ACCEL_READINGS = 14
NUM_ALT_READINGS = 14


#Create list for acceleration readings
accelerationFromBNO = range(0,NUM_ACCEL_READINGS)

#Create list for BNO055 times
BNOTimes = range(0,NUM_ACCEL_READINGS)

#Create list for altitude readings
altitudes = range(0,NUM_ALT_READINGS)

#Create list for BMP180 times
BMPTimes = range(0,NUM_ALT_READINGS)

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

#Set BNO055's Acceleration range to +/2G
set2GRange(bno)

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
#=============== END BNO055 SETUP ===============

#=============== BMP180 SETUP ===================
bmp180 = BMP085.BMP085()
bmp180.__init__(mode = BMP180_SENS)  # set sensitivity

pad_alt = zero_bmp180()

check_bmp180

#============= END BMP180 SETUP =================
#=============== END SETUP =======================


#===============MAIN PROGRAM=====================
while True:
    print("\n===============MAIN MENU===============")
    user_ans = input("Select a number: \n 1) Calibrate BNO055 manually \n 2) Calibrate BNO055 from file \n 3) Collect Data \n 4) Exit \n")
    if user_ans == 1:
        calibrateBNO()
    elif user_ans == 2:
        setCalibration()
    elif user_ans == 3:
        print("\nCollecting data...")
        collectData()
    elif user_ans == 4:
        print("goodbye")
        sys.exit()
    else:
        print("invalid input")
        
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
