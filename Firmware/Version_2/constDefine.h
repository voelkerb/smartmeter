// Private keys can be defined in this file
// Create it and insert e.g. LORA or WIFI keys
#if __has_include("privateConfig.h")
#include "privateConfig.h"
#endif


// Serial Speed and DEBUG option
// #define SERIAL_SPEED 115200
#define SERIAL_SPEED 2000000
// #define SERIAL_SPEED 3000000
// #define DEBUG_DEEP


#define VERSION "2.2"

#define SENT_LIFENESS_TO_CLIENTS
#define SEND_INFO_ON_CLIENT_CONNECT
// #define REPORT_ENERGY_ON_LIFENESS
// #define REPORT_INDIVIDUAL_GRID_LINE_VALUES

#define SEND_INFO_ON_CLIENT_CONNECT
// Only allow cmds over serial if we are not sampling over tcp
// #define CMD_OVER_SERIAL_WHILE_TCP_SAMPLING

// Correction threshold in s
#define NTP_CORRECT_SAMPLINGRATE
#define CORRECT_SAMPLING_THRESHOLD 0.009 // This is more or less half a cycle
#define MAX_CORRECT_SAMPLES 10

// Default values
#define STANDARD_UDP_PORT 54323
#define STANDARD_TCP_SAMPLE_PORT 54321
#define STANDARD_TCP_STREAM_PORT 54322
#define DEFAULT_SR 8000


// Location time difference between us (Freiburg, Germany) and NTP time
#define LOCATION_TIME_OFFSET 3600//7200 // 2 hours or (2*60*60)

#define MDNS_UPDATE_INTERVAL 30000
#define TCP_UPDATE_INTERVAL 100
#define LIFENESS_UPDATE_INTERVAL 1000
#define RTC_UPDATE_INTERVAL 30000
#define STREAM_SERVER_UPDATE_INTERVAL 30000
#define MQTT_UPDATE_INTERVAL 5000
#define ENERGY_UPDATE_INTERVAL 19000

// We allow a max of 3 tcp clients for performance reasons
#define MAX_CLIENTS 3

#define DEF_LOG_FILE "/log.txt"

#define DEF_LOG_PREFIX_SERIAL "Info:"
#define DEF_LOG_PREFIX "Info:"

#define DEF_DATA_PREFIX "Data:"
#define PREFIX_SIZE strlen(DATA_PREFIX)


// All available pins of ESP32
#define IO0 0
#define IO1 1 // UART 0 TX
#define IO2 2
#define IO3 3 // UART 0 RX
#define IO4 4 // HSPI H0
#define IO5 5 // Strapping (must be pulled low at boot)
#define IO6 6 // SPI CLK      // Blocked for SPI Flash
#define IO7 7 // SPI Q        // Blocked for SPI Flash
#define IO8 8 // SPI          // Blocked for SPI Flash
#define IO9 9 // UART 1 RX    // Blocked for SPI Flash
#define IO10 10 // UART 1 TX  // Blocked for SPI Flash
#define IO11 11 // SPI CS     // Blocked for SPI Flash
#define IO12 12 // HSPI Q     // Strapping (must be pulled low at boot)
#define IO13 13 // HSPI ID
#define IO14 14 // HSPI CLK   // Strapping
#define IO15 15 // HSPI CS    // Strapping
#define IO16 16 // Not Available (PSRAM)
#define IO17 17 // Not Available (PSRAM)
#define IO18 18 // VSPI CLK
#define IO19 19 // VSPI MOSI
// #define IO20 20 // non existent
#define IO21 21 // VSPI H2
#define IO22 22 // VSPI WP
#define IO23 23 // VSPI MISO
// #define IO24 24 // non existent
#define IO25 25 // DAC_1
#define IO26 26 // DAC_2
#define IO27 27
// #define IO28 28 // non existent
// #define IO29 29 // non existent
// #define IO30 30 // non existent
// #define IO31 31 // non existent
#define IO32 32
#define IO33 33
#define IO34 34  // Input only
#define IO35 35  // Input only
#define IO36 36  // Input only
// #define IO37 37 // non existent
// #define IO38 38 // non existent
#define IO39 39  // Input only



// ADE Things
#define ADE_SPI_BUS VSPI
// // TODO: Change
#define ADE_RESET_PIN   IO2
// NOTE: Sharing the same pin as ERROR LED; Obacht  
#define ADE_PM1_PIN   IO4
// // Event Pin/Data Ready
#define ADE_DREADY_PIN    IO34


// Used SPI pins
// TODO: Does this work?
#define ADE_SCK IO12
#define ADE_MISO IO13
#define ADE_CS IO14
#define ADE_MOSI IO15


// Ethernet RMII
#define PHY_POWER   IO5
#define EMAC_TX_EN  IO21
#define EMAC_TXD0   IO19
#define EMAC_TXD1   IO22
#define EMAC_RXD0   IO25
#define EMAC_RXD1   IO26
#define EMAC_RX_DV  IO27
#define EMAC_TX_CLK IO0
#define SMI_MDC     IO23
#define SMI_MDIO    IO18

// RTC
#define SDA     IO32
#define SCL     IO33
#define RTC_INT IO36



#define ERROR_LED IO4

// AVAILABLE Pins
// IO1  // UART TX
// IO3  // UART RX
// IO4

// IDEA:
// ADE:
// N_IRQ0_PIN  IO36 // input
// N_IRQ1_PIN  IO39 // input
// Über iO expander:
// PM0_PIN      // output
// PM1_PIN      // output
// CF1          // output
// CF2          // output
// CF3          // output
// CF4          // output


// Buffering stuff
#define MAX_SEND_SIZE 1024 // 512
// PSRAM Buffer
#define PS_BUF_SIZE 3*1024*1024 + 512*1024
#define RAM_BUF_SIZE 40*1024
#define COMMAND_MAX_SIZE 600

// Communication commands
#define CMD_SAMPLE "sample"
#define CMD_REQ_SAMPLES "reqSamples"
#define CMD_FLOW "cts"
#define CMD_STOP "stop"
#define CMD_RESTART "restart"
#define CMD_DAILY_RESTART "dailyRestart"
#define CMD_FACTORY_RESET "factoryReset"
#define CMD_BASIC_RESET "basicReset"
#define CMD_INFO "info"
#define CMD_MDNS "mdns"
#define CMD_NTP "ntp"
#define CMD_ADD_WIFI "addWifi"
#define CMD_REMOVE_WIFI "delWifi"
#define CMD_RESET_ENERGY "resetEnergy"
#define CMD_CLEAR_LOG "clearLog"
#define CMD_GET_LOG "getLog"
#define CMD_MQTT_SERVER "mqttServer"
#define CMD_STREAM_SERVER "streamServer"
#define CMD_TIME_SERVER "timeServer"
#define CMD_LOG_LEVEL "log"
#define CMD_CALIBRATION "calibration"
#define CMD_ADE "ade9k"
#define CMD_GET_POWER "getPower"
#define CMD_GET_VOLTAGE "getVoltage"
#define CMD_GET_CURRENT "getCurrent"
#define CMD_GET_ENERGY "getEnergy"
#define CMD_GET_PERIOD "getPeriod"
#define CMD_GET_PHASE "getPhaseAngle"

#define LOG_LEVEL_ALL "all"
#define LOG_LEVEL_DEBUG "debug"
#define LOG_LEVEL_INFO "info"
#define LOG_LEVEL_WARNING "warning"
#define LOG_LEVEL_ERROR "error"

#define MAX_UNIT_STR_LENGTH 20
#define MAX_MEASURE_STR_LENGTH 20

#define MEASURE_VI "v,i,v,i,v,i"
#define MEASURE_VI_COMPATIBILITY "v,i"
#define MEASURE_VI_L1 "v,i_L1"
#define MEASURE_VI_L2 "v,i_L2"
#define MEASURE_VI_L3 "v,i_L3"
#define MEASURE_PQ "p,q"
#define MEASURE_PQ_LONG "p,p,p,q,q,q"
#define MEASURE_VI_RMS "v,i_RMS"
#define MEASURE_VI_RMS_LONG "v,v,v,i,i,i_RMS"


#define UNIT_VI "V,mA,V,mA,V,mA"
#define UNIT_VI_LX "V,mA"
#define UNIT_PQ "W,W,W,VAR,VAR,VAR"
#define UNIT_VI_RMS "V,V,V,mA,mA,mA"

#include <ETH.h>
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN//ETH_CLOCK_GPIO17_OUT//ETH_CLOCK_GPIO0_IN//ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN   PHY_POWER
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        1
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     SMI_MDC
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    SMI_MDIO



#define MAX_STRING_LENGTH_STACK 40


#define MQTT_TOPIC_BASE "powermeter"
#define MQTT_TOPIC_SEPARATOR '/'
#define MQTT_TOPIC_SWITCH "switch"
#define MQTT_TOPIC_SWITCH_ON "on"
#define MQTT_TOPIC_SWITCH_OFF "off"
#define MQTT_TOPIC_SAMPLE "sample"
#define MQTT_TOPIC_SAMPLING "sampling"
#define MQTT_TOPIC_STATE "state"
#define MQTT_TOPIC_CMD "cmd"
#define MQTT_TOPIC_INFO "info"

#define MAX_MQTT_TOPIC_LEN sizeof(MQTT_TOPIC_BASE) + sizeof(MQTT_TOPIC_STATE) + sizeof(MQTT_TOPIC_SWITCH) + 4*sizeof(MQTT_TOPIC_SEPARATOR) + 2+MAX_NAME_LEN

