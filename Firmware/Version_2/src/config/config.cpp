/***************************************************
 Library for Configuration handling.

 Feel free to use the code as it is.

 Benjamin Voelker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Computer Science
 ****************************************************/

#include "config.h"
#if __has_include("privateConfig.h")
#include "privateConfig.h"
#endif

Configuration::Configuration() {
  snprintf(myConf.mqttServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.streamServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.timeServer, MAX_DNS_LEN, NO_SERVER);
  myConf.resetHour = myConf.resetMinute = -1;

  for (int i = 0; i < 6; i++) myConf.calibration[i] = 1.0;
  for (int i = 0; i < 3; i++) myConf.energy[i] = 0.0;
}

void Configuration::init() {
  // Load the MDNS name from eeprom
  EEPROM.begin(EEPROM_SIZE);
  static_assert(EEPROM_SIZE <= 512, "Max eeprom size is 512");
}

void Configuration::makeDefault(bool resetName) {
  if (resetName) {
    snprintf(netConf.name, MAX_NAME_LEN, "smartmeterX");
  }
  
  snprintf(myConf.mqttServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.streamServer, MAX_DNS_LEN, NO_SERVER);
  snprintf(myConf.timeServer, MAX_DNS_LEN, "time.google.com");

  for (size_t i = 0; i < MAX_WIFI_APS; i++) {
    sprintf(&netConf.SSIDs[i][0], NO_SERVER);
    sprintf(&netConf.PWDs[i][0], NO_SERVER);
  }
  netConf.numAPs = 0;
  // #define in privateConfig
  #ifdef NUM_STANDARD_WIFIS
  for (int i = 0; i < NUM_STANDARD_WIFIS; i++) {
    strcpy(netConf.SSIDs[netConf.numAPs], MY_STANDARD_WIFIS[i*2]);
    strcpy(netConf.PWDs[netConf.numAPs], MY_STANDARD_WIFIS[i*2+1]);
    netConf.numAPs++;
  }
  #endif

  for (int i = 0; i < 6; i++) myConf.calibration[i] = 1.0f;
  for (int i = 0; i < 3; i++) myConf.energy[i] = 0.0;
  myConf.resetHour = myConf.resetMinute = -1;
}

void Configuration::load() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.get(address, netConf);
  // Spot invalid config
  if (netConf.numAPs > MAX_WIFI_APS) {
    makeDefault(false);
    store();
    return;
  }
  address += sizeof(netConf);

  EEPROM.get(address, myConf);
  address += sizeof(myConf);

  // sanity check


  for (int i = 0; i < 6; i++) {
    // Sanity check
    if (isnan(myConf.calibration[i]) or
        myConf.calibration[i] < 0.1 or 
        myConf.calibration[i] > 10.0) {
      myConf.calibration[i] = 1.0f;
    }
  }
}

void Configuration::store() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.put(address, netConf);
  address += sizeof(netConf);
  EEPROM.put(address, myConf);
  address += sizeof(myConf);
  EEPROM.commit();
}

void Configuration::storeMyConf() {
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.put(address, myConf);
  EEPROM.commit();
}

void Configuration::setCalibration(float * values) {
  for (int i = 0; i < 6; i++) myConf.calibration[i] = values[i];
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.put(address, myConf);
  EEPROM.commit();
}

void Configuration::setEnergy(double energyL1, double energyL2, double energyL3) {
  myConf.energy[0] = energyL1;
  myConf.energy[1] = energyL2;
  myConf.energy[2] = energyL3;
  uint32_t address = NAME_START_ADDRESS + sizeof(netConf);
  EEPROM.put(address, myConf.energy); // TODO: Does this work?
  EEPROM.commit();
}

bool Configuration::removeWiFi(char * ssid) {
  int idx = -1;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    if (strcmp(ssid, netConf.SSIDs[ap]) == 0) {
      idx = ap;
      break;
    }
  }
  if (idx == -1) return false;
  for (size_t i = idx; i < netConf.numAPs-1; i++) {
    strcpy(netConf.SSIDs[i], netConf.SSIDs[i+1]);
    strcpy(netConf.PWDs[i], netConf.PWDs[i+1]);
  }
  // Indicate end of array here
  strcpy(netConf.SSIDs[netConf.numAPs-1], "");
  strcpy(netConf.PWDs[netConf.numAPs-1], "");

  netConf.numAPs--;
  storeWiFi(); // Note a little bit overhead but who cares
  return true;
}

int Configuration::addWiFi(char * ssid, char * pwd) {
  int idx = -1;
  for (int ap = 0; ap < MAX_WIFI_APS; ap++) {
    if (strcmp(ssid, netConf.SSIDs[ap]) == 0) {
      idx = ap;
      break;
    }
  }
  if (idx != -1) return -1;
  if (netConf.numAPs >= MAX_WIFI_APS) return 0;
  strcpy(netConf.SSIDs[netConf.numAPs], ssid);
  strcpy(netConf.PWDs[netConf.numAPs], pwd);
  netConf.numAPs++;
  storeWiFi(); // NOTE: a little bit overhead but who cares
  return 1;
}

void Configuration::loadWiFi() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.get(address, netConf);

}

void Configuration::storeWiFi() {
  uint32_t address = NAME_START_ADDRESS;
  EEPROM.put(address, netConf);
  EEPROM.commit();
}


void Configuration::setMQTTServerAddress(char * serverAddress) {
  snprintf(myConf.mqttServer, MAX_DNS_LEN, serverAddress);
  storeMyConf();
}

void Configuration::setTimeServerAddress(char * serverAddress) {
  snprintf(myConf.timeServer, MAX_DNS_LEN, serverAddress);
  storeMyConf();
}

void Configuration::setStreamServerAddress(char * serverAddress) {
  snprintf(myConf.streamServer, MAX_DNS_LEN, serverAddress);
  storeMyConf();
}

void Configuration::setName(char * newName) {
  snprintf(netConf.name, MAX_DNS_LEN, newName);
  EEPROM.put(NAME_START_ADDRESS, netConf);
  EEPROM.commit();
}