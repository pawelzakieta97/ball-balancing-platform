import serial
ser = serial.Serial('/dev/ttyACM1',115200)
ser.write({0})
ser.write({0})
