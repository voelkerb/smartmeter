#!/bin/bash

echo "Press [CTRL+C] to stop.."

while true
do
  read data
  echo "$data" > /dev/ttyUSB0
done


