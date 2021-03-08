[smartmeter]: (https://github.com/voelkerb/smartmeter)

# How to compile Firmware Version 2

## Compile from source using Arduino or PlatformIO

* In order to upload the firmware, you must install the latest version of the Arduino ESP32 environment. See [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md):
  * Enter _https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json_ into _Arduino > Settings > Additional Board Manager URLs_ field.
  * Open Boards Manager from _Tools > Board_ menu and install _esp32 platform_
* Select _ESP32 Dev kit_ with _240MHz_ and _PSRAM enabled_ from _Tools > Board_ 

    <img src="/docu/figures/FirmwareSelectPort.png" width="500">
    
* Open the latest firmware "Version_X_X".
* Press _Compile_

## Upload using USB interface

* Connect the [smartmeter] to USB
* Select no more than 2Mbauds
* Press _Compile and Upload_

## Upload using Arduino-OTA

* Select the [smartmeter] you want to upload to from the available network ports
* If the corresponding [smartmeter] is not shown, make sure you are in the same network or try to reset the [smartmeter].

    <img src="/docu/figures/NetworkPort.png" width="400">

* Upload using password "energy"

    <img src="/docu/figures/Password.png" width="300">

## Upload using custom Uploader

  <img src="/docu/figures/upload.gif">

* Compile the firmware
* Copy the path of the compiled binary _elf_ or _bin_ (the _bin_ will be used either way)

    <img src="/docu/figures/CopyBin.png">

* Use the upload script
  ```bash
  python3 upload.py smartmeter <pathToElfOrBin> 
  ```
  Select one or mulitple [smartmeter] from the provided list
  ```
  Available Devices:
  #  smartmeter:              Device:                  IP:                      
  0  smartmeter001            smartmeter001            192.168.0.145      
  Press ENTER to continue with all devices.
  Deselect specific devices e.g.: -2,-4,-7
  Select specific devices e.g.: 1,3,5,6
  Search again a/A
  Press r/R to reset
  e/E to cancel Or press CTR-C to exit program
  ```
  Press enter and watch the magic
