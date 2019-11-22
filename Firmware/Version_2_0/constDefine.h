

// Serial Speed and DEBUG option
#define SERIAL_SPEED 115200
// #define SERIAL_SPEED 3000000
// #define DEBUG_DEEP
#define SENT_LIFENESS_TO_CLIENTS


#define VERSION "2.0"

// Default values
#define STANDARD_UDP_PORT 54323
#define STANDARD_TCP_SAMPLE_PORT 54321
#define STANDARD_TCP_STREAM_PORT 54322
#define DEFAULT_SR 8000


// Location time difference between us (Freiburg, Germany) and NTP time 
#define LOCATION_TIME_OFFSET 3600//7200 // 2 hours or (2*60*60)


const char * LOG_FILE = "/log.txt";

const char* ntpServerName = "time.uni-freiburg.de";//"0.de.pool.ntp.org";

const char LOG_PREFIX_SERIAL[] = "Info:";
const char LOG_PREFIX[] = "Info:";




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



// AVAILABLE Pins
// IO1  // UART TX
// IO3  // UART RX
// IO4

// IDEA:
// ERROR LED IO4 ^^
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
const int PS_BUF_SIZE = 3*1024*1024 + 512*1024;
const int RAM_BUF_SIZE = 40*1024;
#define COMMAND_MAX_SIZE 400

// Communication commands
#define CMD_SAMPLE "sample"
#define CMD_STOP "stop"
#define CMD_RESTART "restart"
#define CMD_RESET "factoryReset"
#define CMD_INFO "info"
#define CMD_MDNS "mdns"
#define CMD_NTP "ntp"
#define CMD_ADD_WIFI "addWifi"
#define CMD_REMOVE_WIFI "delWifi"
#define CMD_CLEAR_LOG "clearLog"
#define CMD_GET_LOG "getLog"


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