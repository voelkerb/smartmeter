/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <Preferences.h>  // NVS

#include "uart-parser.h"
#include "ADE9000.h"
#include "HardwareSerial.h"


char uart_rx_buffer [50] = { 0 };
uint8_t uart_rx_count = 0;

ADE9000 *ADE9k2;
HardwareSerial *uart2 = NULL;
Preferences preferences;  // NVS
uint8_t _channels = 6;

bool continuous_mode = false;

volatile bool time_flag = false;
volatile bool sample_rate_4k = false;
volatile bool sample_rate_32k = false;

// WiFi flags
volatile bool wifi_connected = false;
volatile bool wifi_enable = false;

// WiFi network name and password:
const char * defaultSSID = "nilm_nomap";
const char * defaultPW = "silkykayak943";
//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * defaultUdpAddress = "192.168.178.19";
const int defaultUdpPort = 3333;


char currentSSID[30];
char currentPW[30];
char currentRemoteAddr[30];
int currentUdpPort;

//The udp library class
WiFiUDP udp;

typedef void (*voidFuncPtr)(void);

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t xHandle = NULL;
TaskHandle_t xHandleProducer = NULL;


//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set
          Serial.print("Info: WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(), currentUdpPort);
          udp.beginPacket(currentRemoteAddr, currentUdpPort);
          wifi_connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("Info: WiFi lost connection");
          wifi_connected = false;
          break;
    }
}


// _____________________________________________________________________________
void uart_parser_setup(ADE9000 *ade, HardwareSerial *uart, uint8_t channels) {
  ADE9k2 = ade;
  uart2 = uart;
  _channels = channels;
  WiFi.onEvent(WiFiEvent);
  delay (1000);
  if (preferences.begin("wifi", true)) {
    if(!preferences.getString("ssid", currentSSID, 30)) {
      strncpy(currentSSID, defaultSSID, strlen(defaultSSID));
      currentSSID[ strlen(defaultSSID) ] = 0;
    }

    if(!preferences.getString("pw", currentPW, 30)) {
      strncpy(currentPW, defaultPW, strlen(defaultPW));
      currentPW[ strlen(defaultSSID) ] = 0;
    }

    if(!preferences.getString("addr", currentRemoteAddr, 30)) {
      strncpy(currentRemoteAddr, defaultUdpAddress, strlen(defaultUdpAddress));
      currentRemoteAddr[ strlen(defaultUdpAddress) ] = 0;
    }
    currentUdpPort = preferences.getUInt("udp", defaultUdpPort);
    preferences.end();

//    Serial.print("SSID: ");
//    Serial.print(currentSSID);
//    Serial.print(" PW: ");
//    Serial.print(currentPW);
//    Serial.print(" UDP: ");
//    Serial.print(currentRemoteAddr);
//    Serial.print(" : ");
//    Serial.println(currentUdpPort);

  };
}


uint32_t wifi_timer_ms = 0;
uint8_t wifi_curren_sample_mode = 0;

// _____________________________________________________________________________
void uart_parser(void) {
  if (uart2->available()) {
    char c = uart2->read();

    if (c == '\n' || uart_rx_count >= 48) {
      parseCommand(uart_rx_buffer);
      uart_rx_buffer[0] = 0;
      uart_rx_count = 0;
    } else {
      uart_rx_buffer[uart_rx_count++] = c;
      uart_rx_buffer[uart_rx_count] = 0;
    }
  }

  // Der WiFi Interrupt Haendler kann nicht die WiFi Verbindung zurücksetzen,
  // dies führt zu cache exceptions
  // Der Consumer-Task muss beendet werden, damit es nach der erneuten Verbindung
  // nicht zu einem Fehler im RTOS kommt.
  if (wifi_enable && !wifi_connected && wifi_timer_ms + 20000 < millis()) {
    if (continuous_mode) {
      if (sample_rate_32k) {
        wifi_curren_sample_mode = 32;
      } else if (sample_rate_4k) {
        wifi_curren_sample_mode = 4;
      } else {
        wifi_curren_sample_mode = 8;
      }
      parseCommand("stop");
    }
    uart2->println("Info: wifi reconnect...");
    //WiFi.disconnect();
    //WiFi.begin(networkName, networkPswd);
    WiFi.reconnect();
    wifi_timer_ms = millis();
  }

  // wiederherstellung des smaplingmodus nach einem reconnect
  if (wifi_connected && wifi_curren_sample_mode) {
    if (wifi_curren_sample_mode == 32) {
        parseCommand("start32k");
      } else if (wifi_curren_sample_mode == 4) {
        parseCommand("start4k");
      } else {
        parseCommand("start8k");
      }
    wifi_curren_sample_mode = 0;
  }
}


// _____________________________________________________________________________
volatile uint32_t sampling_rate_counter = 0;


const uint16_t addr32k = 0x5000;  // 32k Register @ addr 0x500(0)
const uint16_t addr8k = 0x5100;  // 8k Register @ addr 0x510(0)


uint8_t burst_read_tx_array [31] = { 0, };
uint8_t burst_read_rx_array [31] = { 0, };
const uint8_t bytes = 31; // max 64 Byte!  2 (addr) + 7*4 (7ADC@4Byte) + 1 (bitshift)


// _____________________________________________________________________________
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
    time_flag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}



// _____________________________________________________________________________
typedef struct{
  uint32_t adc[7] = { 0 };
}CurrentADC;

xQueueHandle xQueue;


bool volatile IRAM_timeout = false;

// _____________________________________________________________________________
void IRAM_ATTR ISR_ADC_READY(void) { // DREADY ISR from ADE9000
  if(sample_rate_32k) {
    burst_read_tx_array[0] = addr32k >> 8;    // addr [15 - 8]
    burst_read_tx_array[1] = addr32k | 0x8;   // addr [7 - 4], read-bit[3]
  } else {
    burst_read_tx_array[0] = addr8k >> 8;    // addr [15 - 8]
    burst_read_tx_array[1] = addr8k | 0x8;   // addr [7 - 4], read-bit[3]
  }


  CurrentADC data2;

  ADE9k2->burst_read_SPI_Buffer(burst_read_rx_array, bytes);
  ADE9k2->dataBitShift(burst_read_rx_array, bytes);
  ADE9k2->burst_write_SPI(burst_read_tx_array, bytes);

  BaseType_t xHigherPriorityTaskWoken;

  for (uint8_t i = 0; i < 7; i++){
    data2.adc[i] = burst_read_rx_array[ 2+i*4 ] << 24;
    data2.adc[i] |= burst_read_rx_array[ 3+i*4 ] << 16;
    data2.adc[i] |= burst_read_rx_array[ 4+i*4 ] << 8;
    data2.adc[i] |= burst_read_rx_array[ 5+i*4 ];
  }

  BaseType_t xStatus = xQueueSendToBackFromISR( xQueue, &data2,
      &xHigherPriorityTaskWoken );

  // BaseType_t xStatus = xQueueSendToBack( xQueue, &data, 10 );
  // check whether sending is ok or not
  if( xStatus == pdPASS ) {
    // uart2->println("Info: ISR send Data ");
  } else {
    //timeout = true;
    //uart2->println("Info: ISR error");
    IRAM_timeout = true;
  }

  xTaskResumeFromISR( xHandle );
}


bool QueuTimeout = false;
bool volatile toggle = false;
uint8_t wifi_timer = 0;

uint32_t debug_counter = 0;

// _____________________________________________________________________________
void consumerTask( void * parameter) {
  vTaskSuspend( NULL );  // ISR wakes up the task

  while(1){

    CurrentADC data;
    BaseType_t xStatus = xQueueReceive( xQueue, &data, 0);

    if(xStatus == pdPASS) {
      toggle = !toggle;
      if((sample_rate_4k && toggle)  || !sample_rate_4k) {
        sampling_rate_counter++;


        if (wifi_enable) {
          if (wifi_connected) {


            udp.write((uint8_t*) &debug_counter, 4);
            debug_counter++;
            udp.write((uint8_t *) &data.adc[0], 4*_channels);
            udp.write('\r');
            udp.write('\n');

        } else {


        }
      } else {


      // FIXME Daten conversion
          uint8_t *buffer = (uint8_t *) &data.adc[0];
          uart2->write((uint8_t *) (++buffer), 3);
          buffer = (uint8_t *) &data.adc[1];
          uart2->write((uint8_t *) (++buffer), 3);
          buffer = (uint8_t *) &data.adc[2];
          uart2->write((uint8_t *) (++buffer), 3);
          buffer = (uint8_t *) &data.adc[3];
          uart2->write((uint8_t *) (++buffer), 3);
          buffer = (uint8_t *) &data.adc[4];
          uart2->write((uint8_t *) (++buffer), 3);
          buffer = (uint8_t *) &data.adc[5];
          uart2->write((uint8_t *) (++buffer), 3);
          uart2->write("\r\n");

          // uart2->write((uint8_t *) data.adc, 4*_channels);  // 6 ADCs
          //uart2->write((uint8_t *) (&data.adc[0]), 4*1);  // 2 ADCs
          //uart2->write((uint8_t *) (&data.adc[1]), 4*1);  // 2 ADCs
          // uart2->write("\r\n");
        }
    }
    } else {
      QueuTimeout = true;
    }



    if (time_flag == true) {
      portENTER_CRITICAL_ISR(&timerMux);
      time_flag = false;
      portEXIT_CRITICAL_ISR(&timerMux);



      uart2->print("Info: ");
      uart2->print(sampling_rate_counter);
      uart2->println(" Sps");


      if(IRAM_timeout) {
        uart2->println("Info:Lost interrupt!");
        IRAM_timeout = false;
      }

      if(QueuTimeout) {
        uart2->println("Info:Queue timeout!");
        QueuTimeout = false;
      }
      sampling_rate_counter = 0;
    }

    if(!uxQueueMessagesWaiting(xQueue)) {
      vTaskSuspend( NULL );  // release computing time, ISR wakes up the task.
    }
  }
}


// r 16 0xABC
// r 32 0xABC
// w 16 0xABC 0xABDB       16 Bit
// w 32 0xABC 0xABDBABAB   32 Bit
// start8k
// start32k
// stop
// wifi ...



// _____________________________________________________________________________
void parseCommand(char *str) {

  uart2->print("Info:PARSING: ");
  uart2->println(str);

  if (strncmp(str, "start", 5) == 0 && !continuous_mode) {

    if(strcmp(&str[5], "4k") == 0) {
      continuous_mode = true;
      uart2->println("Info:start the 4kSps continuous RAW mode");
    } else if(strcmp(&str[5], "8k") == 0) {
      continuous_mode = true;
      uart2->println("Info:start the 8kSps continuous RAW mode");
    } else if(strcmp(&str[5], "32k") == 0) {
      continuous_mode = true;
      sample_rate_32k = true;
      uart2->println("Info:start the 32kSps continuous RAW mode");
    }
    else {
      uart2->print("Info:unknown sample rate: ");
      uart2->println(&str[5]);
      uart2->println("Info:Usage: \"start8k\" or \"start32k\"");
      return;
    }

    uart2->println("Info:Send \"stop\" to exit RAW mode.");
    delay(500);

    if (xHandle == NULL) { //
      timer = timerBegin(0, 80, true);
      timerAttachInterrupt(timer, &onTimer, true);
      timerAlarmWrite(timer, 1000000, true);
      timerAlarmEnable(timer);

      xQueue = xQueueCreate(200, sizeof(CurrentADC));
      xTaskCreatePinnedToCore(  consumerTask,     /* Task function. */
        "Consumer",       /* String with name of task. */
        4096,            /* Stack size in words. */
        NULL,             /* Parameter passed as input of the task */
        1,                /* Priority of the task. */
        &xHandle,            /* Task handle. */
        0);
    }

    if(strcmp(&str[5], "4k") == 0) {
      // DREADY = Sinc4 + IIR LPF output at 8 kSPS.
      ADE9k2->write_16(0x4A0,0x0200);
      sample_rate_4k = true;
    }

    if(strcmp(&str[5], "8k") == 0) {
      // DREADY = Sinc4 + IIR LPF output at 8 kSPS.
      ADE9k2->write_16(0x4A0,0x0200);
    }

    if(strcmp(&str[5], "32k") == 0) {
      // DREADY = Sinc4 + IIR LPF output at 32 kSPS.
      ADE9k2->write_16(0x4A0,0x0000);
    }

    //delay(500);
    // select which function to output on:Here DREADY
    ADE9k2->write_16(0x481,0x000C);

    ADE9k2->burst_read_en(true);

    delay(500);

    ADE9k2->setupInterrupt(ISR_ADC_READY);
    return;
  }

  if ( strcmp(str, "?") == 0) {
    uart2->println("Info:Setup done");
  }

  if ( strcmp(str, "stop") == 0 || strcmp(str, "exit") == 0) {  // stoping continuous mode
    if(!continuous_mode)
      return;
    ADE9k2->clearInterrupt();
    continuous_mode = false;
    sample_rate_4k = false;
    sample_rate_32k = false;
    delay(500);
    sampling_rate_counter = 0;
    ADE9k2->burst_read_en(false);
    uart2->println("Info:stop the continuous mode");

    if(wifi_enable) {
      udp.endPacket();
    }
    return;
  }

  if (continuous_mode) {  // ignore Data in continuous_mode
    return;
  }

  if (strcmp(str, "?") == 0) {
    uart2->println("Info:Setup done");
    return;
  }


  // wifi setup ################################################################
  if (strncmp(str, "wifi ", 4) == 0) {
    if (strncmp(&str[5], "enable", 6) == 0) {

      wifi_enable = true;
      WiFi.disconnect(true);
      WiFi.begin(currentSSID, currentPW);
      WiFi.setHostname("NILM-ESP32");
      Serial.print("Info:waiting for connection; SSID: ");
      Serial.println(currentSSID);

    } else if (strncmp(&str[5], "disable", 7) == 0) {

      wifi_enable = false;
      Serial.println("Info:disable WiFi");
      WiFi.disconnect(true);

    } else if (strncmp(&str[5], "setup", 5) == 0) {

      if(wifi_enable) {
        Serial.println("Info:Error: WiFi is enable");
        return;
      }

      if (strncmp(&str[11], "ssid", 4) == 0) {

        if (str[15] != 0 && str[16] != 0) {
          Serial.print("Info:new SSID: ");
          strncpy(currentSSID, &str[16], strlen(&str[16]));
          currentSSID[strlen(&str[16])] = 0;
          Serial.println(currentSSID);

          if (preferences.begin("wifi", false)) {
            preferences.putString("ssid", currentSSID);
            preferences.end();
          }
        }

      } else if (strncmp(&str[11], "pw", 2) == 0) {

        if (str[13] != 0 && str[14] != 0) {
          Serial.print("Info:new password: ");
          strncpy(currentPW, &str[14], strlen(&str[14]));
          currentPW[strlen(&str[14])] = 0;
          Serial.println(currentPW);

          if (preferences.begin("wifi", false)) {
            preferences.putString("pw", currentPW);
            preferences.end();
          }
        }

      } else if (strncmp(&str[11], "udp", 3) == 0) {

        if (str[14] != 0 && str[15] != 0) {
          Serial.print("Info:new udpPort: ");
          char *pRemaining;  // for ASCII to Int conversion
          currentUdpPort = strtoul(&str[15], &pRemaining, 0);
          Serial.println(currentUdpPort);
          if (preferences.begin("wifi", false)) {
            preferences.putUInt("udp", currentUdpPort);
            preferences.end();
          }
        }

      } else if (strncmp(&str[11], "remote", 6) == 0) {

        if (str[17] != 0 && str[18] != 0) {
          Serial.print("Info:new remote server: ");

          strncpy(currentRemoteAddr, &str[18], strlen(&str[18]));
          currentRemoteAddr[ strlen(&str[18]) ] = 0;

          Serial.println(currentRemoteAddr);

          if (preferences.begin("wifi", false)) {
            preferences.putString("addr", currentRemoteAddr);
            preferences.end();
          }

        }
      }

    }

    return;
  }





  bool write_modus = false;
  bool bit16_modus = false;

  if(str[0] == 'w') {
    write_modus = true;
  }
  else if(str[0] == 'r') {

  }
  else {
    uart2->println("Info: Error: Invalid Data!");
    return;
  }

  char *pRemaining;  // for ASCII to Int conversion
  uint8_t modus = strtoul(&str[1], &pRemaining, 0);

  if(modus == 16) {
    bit16_modus = true;
  }else if(modus =! 32){
    uart2->println("Info: Error: Invalid Bit Modus!");
    return;
  }

  uint16_t addr = strtoul(&pRemaining[1], &pRemaining, 16); // HEX Data in
  if(addr > 0x6BC) {
    uart2->println("Info: Error: Invalid addr!");
    return;
  }

  uint32_t data_for_write = strtoul(&pRemaining[1], &pRemaining, 16);

  if(bit16_modus && data_for_write > 0xffff) {
    uart2->println("Info: Error: Deta is larger than 16Bit!");
    return;
  }

  if (write_modus) {
    uart2->print("Info: w ");
  } else {
    uart2->print("Info: r ");
  }

  if (bit16_modus) {
    uart2->print("16 ");
  } else {
    uart2->print("32 ");
  }

  uart2->print("0x");
  uart2->print(addr, HEX);

  if (write_modus) {
    uart2->print(" 0x");
    uart2->print(data_for_write, HEX);
  }

  uart2->println();

  if (write_modus) {
    if(bit16_modus) {
      ADE9k2->write_16(addr, data_for_write);
    } else {
      ADE9k2->write_32(addr, data_for_write);
    }
  } else {
    uint32_t rx_data = 0;
    if (bit16_modus) {
      rx_data = ADE9k2->read_16(addr);
    } else {
      rx_data = ADE9k2->read_32(addr);
    }
    uart2->print("Info: rx 0x");
    uart2->println(rx_data, HEX);
  }

  return;
}
