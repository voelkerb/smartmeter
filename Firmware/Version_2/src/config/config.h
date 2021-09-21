/***************************************************
 Library for configuration.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef CONFIG_h
#define CONFIG_h

#include <EEPROM.h>

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Include for network config
#include "../network/src/network.h"
#include "../time/src/timeHandling.h"


// If you cahange any of these values, the config on all devices will be bricked
#define MAX_STRING_LEN 25
#define MAX_IP_LEN 17
#define MAX_DNS_LEN 25
#define MAX_NAME_LEN MAX_STRING_LEN


#define NAME_START_ADDRESS 0
#define EEPROM_SIZE sizeof(NetworkConf)+sizeof(MeterConfiguration)+2

#define NO_SERVER "-"


// packed required to store in EEEPROM efficiently
struct __attribute__((__packed__)) MeterConfiguration {
  //NOTE: Relay state must be first in configuration
  double energy[3] = {0.0};                 // amount of energy consumed per Grid Leg (Wh)
  float calibration[6] = {1.0f};            // Calibration parameter for current and voltage per leg
  char mqttServer[MAX_DNS_LEN] = {'\0'};    // MQTT Server DNS name
  char streamServer[MAX_DNS_LEN] = {'\0'};  // Stream Server DNS name
  char timeServer[MAX_DNS_LEN] = {'\0'};    // Time Server DNS name
  int8_t resetHour = -1;                    // Perform reliability reset (hour)
  int8_t resetMinute = -1;                  // Perform reliability reset (minute) -1 to disable
  Timestamp energyReset{0,0};               // Time when energy was reset last time
}; 


class Configuration {
  public:
    Configuration();

    void init();  
    void load();
    void store();
    void makeDefault(bool resetName=true);

    void setName(char * newName);


    int addWiFi(char * ssid, char * pwd);
    bool removeWiFi(char * ssid);
    void loadWiFi();
    void storeWiFi();
    void setMQTTServerAddress(char * serverAddress);
    void setStreamServerAddress(char * serverAddress);
    void setTimeServerAddress(char * serverAddress);
    void setEnergy(double energyL1, double energyL2, double energyL3);
    void setCalibration(float * values);

    NetworkConf netConf;
    MeterConfiguration myConf;

  private:
    void storeMyConf();
    // bool storeString(unsigned int address, char * str);
    // bool loadString(unsigned int address, char * str, unsigned int max_len);
};

#endif
