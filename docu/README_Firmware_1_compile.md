[smartmeter]: (https://github.com/voelkerb/smartmeter)

# How to compile Firmware Version 1

## Compile from source using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP32 environment. See [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md):
  * Enter _https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp32 platform_
* Select _ESP32 Dev kit_ with _240MHz_ from _Tools > Board_ 

    <img src="/docu/figures/FirmwareSelectPort.png" width="500">
    
* Open the latest firmware "Version_X_X".
* Press _Compile_

## Upload using USB interface

* Connect the [smartmeter] to the USB interface
* Select no more than 2Mbauds
* Press _Compile and Upload_
