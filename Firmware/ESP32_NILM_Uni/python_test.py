import serial
ser = serial.Serial("/dev/ttyUSB0", 12000000) #open first serial port
ser.write("Hallo Welt\n") #write a string
ser.close() #close port
