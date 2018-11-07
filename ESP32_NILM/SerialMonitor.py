
import sys
from serial import Serial

ser = Serial(port=sys.argv[1], baudrate=sys.argv[2])

while True:
  data = ser.readline()
  if data:
    sys.stdout.write(data)

