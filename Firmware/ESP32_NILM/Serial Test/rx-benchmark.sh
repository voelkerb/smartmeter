#!/bin/bash
sleep 0.5

# --dtr 0 = ESP32 nicht reseten
#python -m serial.tools.miniterm /dev/ttyUSB0 115200 --dtr 1 --ask

if [ $# -lt 2 ]    # lt = lower then
   then
   echo "Error: At least two arguments required!"
   echo "serial.sh /dev/ttyUSB0 2000000"
   exit 1
fi



stty -F "$1" 1:0:18bb:0:3:1c:7f:8:4:1:3c:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0
stty -F "$1" "$2"

echo "Open Device $1, Baud rate $2 "
echo

pv "$1" | dd of=/dev/null
