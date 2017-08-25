import serial
import pynmea2

def parseGPS(str):
    if str.find('GGA') > 0:
        msg = pynmea2.parse(str)
        print "Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s" % (msg.timestamp,msg.lat,msg.lat_dir,msg.lon,msg.lon_dir,msg.altitude,msg.altitude_units)


#replace the serial port and the baud rate with the desired values"
#serialPort = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.5)
serialPort = serial.Serial("/dev/ttyS2", 4800, timeout=0.5)


while True:
		stri = serialPort.readline()
		parseGPS(stri)
