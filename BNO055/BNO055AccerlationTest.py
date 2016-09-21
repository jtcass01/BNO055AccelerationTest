#IMPORT STATEMENTS
import logging
import sys
import time

from Adafruit_BNO055 import BNO055

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

print('Reading BNO055 data, press Ctrl-C to quit...')
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

    #calulation to find the gradient of linear acceleration onto the acceleration of gravity
    gradient = (((xa*xg)+(ya*yg)+(za*zg))/(((xg**2)+(yg**2)+(zg**2))**0.5))

    #Calculations to find each component of acceleration using the gradient of linear accerlation onto the accerlation of gravity
    xComponentOfAcceleration = gradient*xg
    yComponentOfAcceleration = gradient*yg
    zComponentOfAcceleration = gradient*zg

    #Final calculation finding the magnitude of accerlation in the direction of gravity (e.g. the upwards
    #linear accerlation regardless of orientation of the sensor.)
    magnitudeOfAccelerationInZDirection = (((xComponentOfAcceleration**2)+(yComponentOfAcceleration**2)+(zComponentOfAcceleration**2))**0.5)
    print('Current Acceleration in the Z direction: x={0:0.2F} y={1:0.2F} z={2:0.2F}'.format(xComponentOfAcceleration,yComponentOfAcceleration,zComponentOfAcceleration))
    print('Magnitude of acceleration in the Z Direction = {0:0.2F}'.format(magnitudeOfAccelerationInZDirection))
    
    time.sleep(.5)

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
