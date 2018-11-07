
ARDUINO_VARIANT = node32s
FLASH_FREQ = 80m
UPLOAD_SPEED = 921600
SERIAL_PORT = /dev/ttyUSB0
# uncomment and set the right serail baud according to your sketch (default to 115200)
SERIAL_BAUD = 2000000

CFLAGS=-D UART0_BAUD 

# uncomment this to use the 1M SPIFFS mapping
#SPIFFS_SIZE = 1
ARDUINO_ARCH=esp32
#USER_DEFINE = -D_SSID_=\"YourSSID\" -D_WIFI_PASSWORD_=\"YourPassword\"
#OTA_IP = 192.168.1.184
#OTA_PORT = 8266 
#OTA_AUTH = password

#

include ~/arduino-makefile/Esp8266-Arduino-Makefile/espXArduino.mk

program: upload serial

serial:
	gnome-terminal -e "sh $(ROOT_DIR)/serial.sh $(SERIAL_PORT) $(SERIAL_BAUD)" --hide-menubar
	#gnome-terminal -e "bash $(ROOT_DIR)/serial-tx.sh $(SERIAL_PORT) $(SERIAL_BAUD)" --hide-menubar
  # gnome-terminal -e "python SerialMonitor $(SERIAL_PORT) $(SERIAL_BAUD)" --hide-menubar


