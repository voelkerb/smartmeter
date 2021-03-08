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

// If you cahange any of these values, the config on all devices will be bricked
#define MAX_WIFI_APS 4
#define MAX_STRING_LEN 25
#define MAX_IP_LEN 17
#define MAX_DNS_LEN 25
#define MAX_NAME_LEN MAX_STRING_LEN
#define MAX_SSID_LEN MAX_STRING_LEN
#define MAX_PWD_LEN MAX_STRING_LEN
#define CALIBRATION_LEN 6
#define NAME_START_ADDRESS 0
#define WIFI_START_ADDRESS NAME_START_ADDRESS+MAX_NAME_LEN+1
#define MQTT_START_ADDRESS WIFI_START_ADDRESS+MAX_WIFI_APS*(MAX_SSID_LEN+MAX_PWD_LEN+2)
#define STREAM_SERVER_ADDRESS MQTT_START_ADDRESS+MAX_IP_LEN+1
#define TIME_SERVER_ADDRESS STREAM_SERVER_ADDRESS+MAX_IP_LEN+1

#define CALIBRATION_START_ADDRESS TIME_SERVER_ADDRESS+MAX_DNS_LEN+1
// Add configs here
#define EEPROM_SIZE CALIBRATION_START_ADDRESS+CALIBRATION_LEN*sizeof(float)+2

#define NO_SERVER "-"

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
    void setCalibration(float * values);

    char name[MAX_NAME_LEN];
    char mqttServer[MAX_IP_LEN];
    char streamServer[MAX_IP_LEN];
    char timeServer[MAX_DNS_LEN];
    unsigned int numAPs;
    char wifiSSIDs[MAX_WIFI_APS][MAX_STRING_LEN];
    char wifiPWDs[MAX_WIFI_APS][MAX_STRING_LEN];

    float cal[CALIBRATION_LEN];
  private:

    bool storeString(unsigned int address, char * str);
    bool loadString(unsigned int address, char * str, unsigned int max_len);
};

#endif
