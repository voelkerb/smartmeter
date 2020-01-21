#include <Arduino.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <time.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <ESPmDNS.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <esp_bt.h>
#include <esp32-hal-cpu.h>
#include <rom/rtc.h>

#include "constDefine.h"
#include "src/ADE9000/ADE9000.h"
#include "src/config/config.h"
#include "src/ringbuffer/ringbuffer.h"
#include "src/network/network.h"
#include "src/rtc/rtc.h"
#include "src/time/timeHandling.h"
#include "src/logger/logger.h"

void isr_adc_ready();
void IRAM_ATTR sqwvTriggered();

// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);
// SPIFFS logger
SPIFFSLogger spiffsLog(false, &LOG_FILE[0], &timeStr, &LOG_PREFIX_SERIAL[0], WARNING);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();

Configuration config;

Rtc rtc(RTC_INT, SDA, SCL);
TimeHandler myTime(ntpServerName, LOCATION_TIME_OFFSET, &rtc);

// ADE900 Object
ADE9000 ade9k(ADE_RESET_PIN, ADE_DREADY_PIN, ADE_SPI_BUS);

RingBuffer ringBuffer(PS_BUF_SIZE, true);

// counter holds # isr calls
volatile uint32_t counter = 0;
// Last micros() count of isr call
volatile long nowTs = 0;
volatile long lastTs = 0;

static uint8_t sendbuffer[MAX_SEND_SIZE+16] = {0};
// Buffer read/write position
uint32_t psdReadPtr = 0;
uint32_t psdWritePtr = 0;
volatile uint32_t writePtr = 0;

// Open two TCP ports, one for commands and sampling and one for a raw data stream
WiFiServer server(STANDARD_TCP_SAMPLE_PORT);
WiFiServer streamServer(STANDARD_TCP_STREAM_PORT);

// TIMER stuff
// Calculate the number of cycles we have to wait
volatile uint32_t TIMER_CYCLES_FAST = (1000000) / DEFAULT_SR; // Cycles between HW timer inerrupts
volatile uint32_t timer_next;
volatile uint32_t timer_now;


// Internal state machine for sampling
enum SampleState{STATE_IDLE, STATE_SAMPLE};

SampleState state = STATE_IDLE;
SampleState next_state = STATE_IDLE;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum Measures{STATE_3xVI, STATE_VI};
enum StreamType{USB, TCP, UDP, TCP_RAW};

struct StreamConfig {
  bool prefix = false;                      // Send data with "data:" prefix
  Measures measures = STATE_3xVI;           // The measures to send
  uint8_t measurementBytes = 24;             // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  StreamType stream = USB;                  // Channel over which to send
  unsigned int countdown = 0;               // Start at specific time or immidiately
  uint32_t chunkSize = 0;                   // Chunksize of one packet sent
  IPAddress ip;                             // Ip address of data sink
  uint16_t port = STANDARD_TCP_SAMPLE_PORT; // Port of data sink
};

StreamConfig streamConfig;

// Internal samplingrate always either 8k or 32k other sr are derived by leavout samples...
int intSamplingRate = 8000;
uint8_t leavoutSamples = 0;

// Stuff for frequency calculation
volatile long freqCalcStart;
volatile long freqCalcNow;
volatile long freq = 0;

// TCP clients and current connected state, each client is assigned a logger
WiFiClient client[MAX_CLIENTS];
bool clientConnected[MAX_CLIENTS] = {false};
StreamLogger * streamLog[MAX_CLIENTS];

// Strean client is for ffmpeg direct streaming
WiFiClient streamClient;
// tcp stuff is send over this client 
WiFiClient * sendClient;
// UDP used for streaming
WiFiUDP udpClient;

// Command stuff send over what ever
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
String response = "";

// Information about the current sampling period
unsigned long samplingCountdown = 0;
unsigned long startSamplingMillis = 0;
unsigned long samplingDuration = 0;
unsigned long sentSamples = 0;
volatile unsigned long packetNumber = 0;
volatile unsigned long totalSamples = 0;

// Some timer stuff s.t. things are updated regularly and not at full speed
long lifenessUpdate = millis();
long mdnsUpdate = millis();
long tcpUpdate = millis();
long rtcUpdate = millis();

// Current CPU speed
unsigned int coreFreq = 0;

// test stuff
long testMillis = 0;
long testMillis2 = 0;
uint16_t testSamples = 0;

// Mutex for 1s RTC Interrupt
portMUX_TYPE sqwMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE intterruptSamplesMux = portMUX_INITIALIZER_UNLOCKED;
bool firstSqwv = true;
volatile int sqwCounter = 0;

volatile uint32_t interruptSamples = 0;

float values[7] = {0};

TaskHandle_t xHandle = NULL;

/************************ SETUP *************************/
void setup() {
  // Setup serial communication
  Serial.begin(SERIAL_SPEED);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, HIGH);

  // At first init the rtc module to get 
  bool successAll = true;

  // Init the logging module
  logger.setTimeGetter(&timeStr);
  // Add Serial logger
  logger.addLogger(&serialLog);
  // Add spiffs logger
  logger.addLogger(&spiffsLog);
  // Init all loggers
  logger.init();

  // init the stream logger array
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    StreamLogger * theStreamLog = new StreamLogger(NULL, &timeStr, &LOG_PREFIX[0], INFO);
    streamLog[i] = theStreamLog;
  }

  bool success = rtc.init();
  
  config.load();

  coreFreq = getCpuFrequencyMhz();
  logger.log(DEBUG, "%s @ firmware %s/%s", config.name, __DATE__, __TIME__);
  logger.log(DEBUG, "Core @ %u MHz", coreFreq);

  success = ringBuffer.init();
  if (!success) logger.log(ERROR, "PSRAM init failed");
  successAll &= success;
  // Try to use internal ram for buffering, but still indicate the missing ram
  if (!success) {
    success = ringBuffer.setSize(RAM_BUF_SIZE, false);
    if (!success) logger.log(ERROR, "RAM init failed: %i bytes", RAM_BUF_SIZE);
  }

  // Setup ADE9000
  ade9k.initSPI(ADE_SCK, ADE_MISO, ADE_MOSI, ADE_CS);
  success = ade9k.init(&isr_adc_ready);
  if (!success) logger.log(ERROR, "ADE Init Failed");
  successAll &= success;
  
  if (successAll) digitalWrite(ERROR_LED, LOW);
  logger.log(ALL, "Connecting Network");

  Network::init(&config, onNetworkConnect, onNetworkDisconnect, true);
  Network::initPHY(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  setupOTA();

  response.reserve(2*COMMAND_MAX_SIZE);


  setInfoString(&command[0]);
  logger.log(&command[0]);

  logger.log(ALL, "Setup done");

  lifenessUpdate = millis();
  mdnsUpdate = millis();
  tcpUpdate = millis();
  rtcUpdate = millis();
}

// the loop routine runs over and over again forever:
void loop() {

  // For error check during sampling
  if (sqwCounter) {
    // If it is still > 0, our loop is too slow, 
    // reasons may be e.g. slow tcp write performance
    if (sqwCounter > 1) {
      logger.log(WARNING, "Loop %us behind", sqwCounter);
    }
    // Reset to 0
    portENTER_CRITICAL(&sqwMux);
    sqwCounter = 0;
    portEXIT_CRITICAL(&sqwMux);
    // Ignore first sqwv showing missing samples since
    // sqwv did not start with sampling
    // NOTE: is this valid?
    // TODO: can we reset the sqwv somehow?
    
    // logger.log(INFO, "RTC: %s", rtc.timeStr());
  }

  // Stuff done on idle
  if (state == STATE_IDLE) {
    onIdle();
  // Stuff on sampling
  } else if (state == STATE_SAMPLE) {
    onSampling();
  }

  onIdleOrSampling();
  
  // If we only have 200 ms before sampling should start, wait actively
  if (next_state != STATE_IDLE) {
    if (streamConfig.countdown != 0 and (streamConfig.countdown - millis()) < 200) {
      // Disable any wifi sleep mode
      esp_wifi_set_ps(WIFI_PS_NONE);
      state = next_state;
      logger.log(DEBUG, "StartSampling @ %s", myTime.timeStr());
      // Calculate time to wait
      int32_t mydelta = streamConfig.countdown - millis();
      // Waiting here is not nice, but we wait for zero crossing
      if (mydelta > 0) delay(mydelta-1);
      startSampling();
      // Reset sampling countdown
      streamConfig.countdown = 0;
    }
  }

  // Watchdog
  yield();
}


char * timeStr() {
  char * ttime = myTime.timeStr(true);
  return ttime;
}

void onNetworkConnect() {
  logger.log(ALL, "Network Connected");
  logger.log(ALL, "IP: %s", Network::localIP().toString().c_str());
  // The stuff todo if we have a network connection (and hopefully internet as well)
  if (Network::connected and not Network::apMode) {
    myTime.updateNTPTime(true);
  }

  // Reinit mdns
  initMDNS();
  // Start the TCP server
  server.begin();
  streamServer.begin();

  // Reset lifeness and MDNS update
  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

void onNetworkDisconnect() {
  logger.log(ERROR, "Network Disconnected");

  if (state != STATE_IDLE) {
    logger.log(ERROR, "Stop sampling (Network disconnect)");
    stopSampling();
  }
}

void onIdleOrSampling() {

  // Handle serial requests
  if (Serial.available()) {
    handleEvent(Serial);
  }

  // Handle tcp requests
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (client[i].available() > 0) {
      handleEvent(client[i]);
    }
  }

  // Handle tcp clients connections

  if ((long)(millis() - tcpUpdate) >= 0) {
    tcpUpdate += TCP_UPDATE_INTERVAL;
    // Handle disconnect
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i] and !client[i].connected()) {
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }

    // Handle connect
    WiFiClient newClient = server.available();
    if (newClient) {
      onClientConnect(newClient);
    }
  }
}


void onIdle() {
  spiffsLog.flush();
  // Arduino OTA
  ArduinoOTA.handle();

  // Re-advertise MDNS service service every 30s 
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  if ((long)(millis() - mdnsUpdate) >= 0) {
    mdnsUpdate += MDNS_UPDATE_INTERVAL;
    // initMDNS();
    //MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
  }

  if ((long)(millis() - rtcUpdate) >= 0) {
    rtcUpdate += RTC_UPDATE_INTERVAL;
    if (rtc.connected) rtc.update();
  }
    
  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    lifenessUpdate += LIFENESS_UPDATE_INTERVAL;
    logger.log("");
  }

  // Look for people connecting over the stream server
  // If one connected there, immidiately start streaming data
  if (!streamClient.connected()) {
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      // Set everything to default settings
      streamConfig.stream = TCP_RAW;
      streamConfig.prefix = false;
      streamConfig.samplingRate = DEFAULT_SR;
      streamConfig.measures = STATE_VI;
      streamConfig.ip = streamClient.remoteIP();
      streamConfig.port = streamClient.remoteIP();
      sendClient = &streamClient;
      startSampling();
    }
  }
}

void onClientConnect(WiFiClient &newClient) {
  logger.log("Client with IP %s connected on port %u", newClient.remoteIP().toString().c_str(), newClient.remotePort());
  
  // Loop over all clients and look where we can store the pointer... 
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (!clientConnected[i]) {
      client[i] = newClient;
      // Set connected flag
      clientConnected[i] = true;
      streamLog[i]->_type = INFO; // This might be later reset
      streamLog[i]->_stream = (Stream*)&client[i];
      logger.addLogger(streamLog[i]);
      return;
    }
  }
  logger.log("To much clients, could not add client");
  newClient.stop();
}

void onClientDisconnect(WiFiClient &oldClient, int i) {
  logger.log("Client discconnected %s port %u", oldClient.remoteIP().toString().c_str(), oldClient.remotePort());
  logger.removeLogger(streamLog[i]);
  streamLog[i]->_stream = NULL;
}

void onSampling() {
  // ______________ Send data to sink ________________
  writeChunks(false);

  // Output sampling frequency regularly
  if (freq != 0) {
    long fr = freq;
    freq = 0;
    float frequency = (float)streamConfig.samplingRate/(float)((fr)/1000000.0);//  Logging only works for USB rates smaller 8000Hz
    if (!(streamConfig.stream == USB and streamConfig.samplingRate > 4000)) {
      logger.log("%.2fHz, %us", frequency, totalSamples);
    }
  }

  // ______________ Handle Disconnect ________________

  // NOTE: Cannot detect disconnect for USB
  if (streamConfig.stream == USB) {
  } else if (streamConfig.stream == TCP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of UDP means disconnecting from tcp port
  } else if (streamConfig.stream == UDP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP/UDP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of raw stream means stop
  } else if (streamConfig.stream == TCP_RAW) {
    // Check for intended connection loss
    if (!sendClient->connected()) {
      logger.log(INFO, "TCP Stream disconnected");
      stopSampling();
    }
  }
}

// Depending on data sink, send data
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    if (streamConfig.stream == UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, streamConfig.chunkSize);
      udpClient.endPacket();
    } else if (streamConfig.stream == TCP) {
      writeData(*sendClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == TCP_RAW) {
      writeData(*sendClient, streamConfig.chunkSize);
    } else if (streamConfig.stream == USB) {
      writeData(Serial, streamConfig.chunkSize);
    }
  }
  if (tail) {
    if (streamConfig.stream == UDP) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, ringBuffer.available());
      udpClient.endPacket();
    } else if (streamConfig.stream == TCP) {
      writeData(*sendClient, ringBuffer.available());
    } else if (streamConfig.stream == TCP_RAW) {
      writeData(*sendClient, ringBuffer.available());
    } else if (streamConfig.stream == USB) {
      writeData(Serial, ringBuffer.available());
    }
  }
}

// Data prefix 
const char data_id[5] = {'D','a','t','a',':'};
void writeData(Stream &getter, uint16_t size) {
  if (size <= 0) return;
  uint32_t start = 0;
  if (streamConfig.prefix) {
    memcpy(&sendbuffer[start], (void*)&data_id[0], sizeof(data_id));
    start += sizeof(data_id);
    memcpy(&sendbuffer[start], (void*)&size, sizeof(uint16_t));
    start += sizeof(uint16_t);
    memcpy(&sendbuffer[start], (void*)&packetNumber, sizeof(uint32_t));
    start += sizeof(uint32_t);
    packetNumber += 1;
  }

  ringBuffer.read(&sendbuffer[start], size);
  // Everything is sent at once (hopefully)
  uint32_t sent = getter.write((uint8_t*)&sendbuffer[0], size+start);
  // Terminate with \r\n for usb // TODO: Can we remove this
  if (streamConfig.stream == USB) getter.write("\r\n"); 
  if (sent > start) sentSamples += (sent-start)/streamConfig.measurementBytes;
}



// // triggered each second to active a new generation of <samplingrate> samples 
void IRAM_ATTR sqwvTriggered() {
  // // Disable timer if not already done in ISR
  // timerAlarmDisable(timer);
  // Reset counter if not already done in ISR
  // testSamples = counter;
  
  // timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
  // We take one sample and enable the timing interrupt afterwards
  // this should allow the ISR e.g. at 4kHz to take 3999 samples during the next second
  // timerWrite(timer, 0);
  // sample_ISR();
  // Enable timer
  // timerAlarmEnable(timer);
  // indicate to main that a second has passed and store time 
  portENTER_CRITICAL_ISR(&sqwMux);
  sqwCounter++;
  portEXIT_CRITICAL(&sqwMux);
  portENTER_CRITICAL_ISR(&intterruptSamplesMux);
  interruptSamples = 0;
  portEXIT_CRITICAL(&intterruptSamplesMux);
  
  testMillis2 = millis();
}


xQueueHandle xQueue = xQueueCreate(2000, sizeof(CurrentADC)); ;

bool volatile IRAM_timeout = false;


CurrentADC isrData;
float data[7] = {0.0};
volatile int skipper = 0;

volatile uint8_t cntLeavoutSamples = 0;


// _____________________________________________________________________________
void IRAM_ATTR isr_adc_ready() {
  // Skip this interrup if we want less samples
  cntLeavoutSamples++;
  if (cntLeavoutSamples < leavoutSamples) return;
  cntLeavoutSamples = 0;
  if (interruptSamples >= streamConfig.samplingRate) return;
  portENTER_CRITICAL_ISR(&intterruptSamplesMux);
  interruptSamples++;
  portEXIT_CRITICAL(&intterruptSamplesMux);
  CurrentADC adc;
  ade9k.readRawMeasurement(&adc);
  BaseType_t xHigherPriorityTaskWoken;
  BaseType_t xStatus = xQueueSendToBackFromISR( xQueue, &adc, &xHigherPriorityTaskWoken );

  // BaseType_t xStatus = xQueueSendToBack( xQueue, &data, 10 );
  // check whether sending is ok or not
  if( xStatus == pdPASS ) {
  } else {
    IRAM_timeout = true;
  }

  xTaskResumeFromISR( xHandle );
}

bool QueueTimeout = false;
float values2[7] = {0.0};

size_t sendstart = 0;
uint16_t myChunkSize = (512/24)*24;
long bufftimer = millis();
// _____________________________________________________________________________
void sample_timer_task( void * parameter) {
  vTaskSuspend( NULL );  // ISR wakes up the task

  CurrentADC adc;
  while(state == STATE_SAMPLE){
    bool dum;

    BaseType_t xStatus = xQueueReceive( xQueue, &adc, 0);

    if(xStatus == pdPASS) {

      ade9k.convertRawMeasurement(&adc, &values2[0]);

      counter++;
      totalSamples++;
      if (counter >= streamConfig.samplingRate) {
        freqCalcNow = micros();
        freq = freqCalcNow-freqCalcStart;
        freqCalcStart = freqCalcNow;
        counter = 0;

        if(IRAM_timeout) {
          logger.log(ERROR, "Lost interrupt!");
          IRAM_timeout = false;
        }

        if(QueueTimeout) {
          logger.log(ERROR, "Queue timeout!");
          QueueTimeout = false;
        }
      }

      // For serial there is no need to buffer it anyway
      if (streamConfig.stream == USB) {
        Serial.write((uint8_t*)&values2[0], streamConfig.measurementBytes);
        // Serial.write((uint8_t*)&values2[0], 16);
        Serial.println("");
      } else {
        // if (sendstart == 0) {
        //   memcpy(&sendbuffer[sendstart], (void*)&data_id[0], sizeof(data_id));
        //   sendstart += sizeof(data_id);
        //   memcpy(&sendbuffer[sendstart], (void*)&myChunkSize, sizeof(uint16_t));
        //   sendstart += sizeof(uint16_t);
        //   memcpy(&sendbuffer[sendstart], (void*)&packetNumber, sizeof(uint32_t));
        //   sendstart += sizeof(uint32_t);
        //   packetNumber += 1;
        // }
        // memcpy(&sendbuffer[sendstart], (uint8_t*)&values2[0], streamConfig.measurementBytes);
        // sendstart += streamConfig.measurementBytes;
        // if (sendstart>=myChunkSize) {
        //   tcpClient.write(sendbuffer, sendstart);
        //   sendstart = 0;
        // }

        bool success = ringBuffer.write((uint8_t*)&values2[0], streamConfig.measurementBytes);
        if (!success and millis() - bufftimer > 1000) {
          bufftimer = millis();
          logger.log(ERROR, "Overflow during ringBuffer write");
        }
      }

    } else {
      QueueTimeout = true;
    }

    if(!uxQueueMessagesWaiting(xQueue)) {
      vTaskSuspend( NULL );  // release computing time, ISR wakes up the task.
    }
  }
  vTaskDelete( NULL );
}

// To display only full percent updates
unsigned int oldPercent = 0;
void setupOTA() {
  // Same name as mdns
  ArduinoOTA.setHostname(config.name);
  ArduinoOTA.setPassword("energy"); 
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    logger.log("Start updating");
    // free(buffer);
  });
  ArduinoOTA.onEnd([]() {
    logger.log("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress / (total / 100));
    if (percent != oldPercent) {
      logger.log("Progress: %u%%\r", (progress / (total / 100)));
      oldPercent = percent;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    logger.append("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) logger.append("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) logger.append("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) logger.append("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) logger.append("Receive Failed");
    else if (error == OTA_END_ERROR) logger.append("End Failed");
    logger.flush(ERROR);
    // No matter what happended, simply restart
    ESP.restart();
  });
  // Enable OTA
  ArduinoOTA.begin();
}

// _____________________________________________________________________________

/****************************************************
 * A request happended, handle it
 ****************************************************/
void handleEvent(Stream &getter) {
  if (!getter.available()) return;
  getter.readStringUntil('\n').toCharArray(command,COMMAND_MAX_SIZE);
  // Be backwards compatible to "?" command all the time
  if (command[0] == '?') {
    getter.println(F("Info:Setup done"));
    return;
  }
  #ifdef DEBUG_DEEP
  logger.log(INFO, command);
  #endif

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docRcv, command);
  // Test if parsing succeeds.
  if (error) {
    // Remove all unallowed json characters to prevent error 
    uint32_t len = strlen(command);
    if (len > 10) len = 10;
    for (int i = 0; i < len; i++) {
      if (command[i] == '\r' || command[i] == '\n' || command[i] == '"' || command[i] == '}' || command[i] == '{') command[i] = '_';
    }
    logger.log(ERROR, "deserializeJson() failed: %.10s", &command[0]);
    return;
  }
  //docSend.clear();
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  response = "";
  handleJSON(getter);
  if (docSend.isNull() == false) {
    if (docSend["error"].as<bool>()) {
      logger.log(ERROR, (const char*)docSend["msg"]);
    }
    getter.flush();
    response = "";
    serializeJson(docSend, response);
    response = "Info:" + response;
    getter.println(response);
    // This will be too long for the logger
    // logger.log(response.c_str());
  }
  response = "";
  command[0] = '\0';
}

void setBusyResponse() {
  if (sendClient != NULL) {
    response = "Device with IP: ";
    response += sendClient->localIP().toString();
    response += " currently sampling"; 
  } else {
    response = "Currently sampling";
  }
}

void handleJSON(Stream &getter) {
  // All commands look like the following:
  // {"cmd":{"name":"commandName", "payload":{<possible data>}}}
  // e.g. mdns

  const char* cmd = docRcv[F("cmd")]["name"];
  JsonObject root = docRcv.as<JsonObject>();
  if (cmd == nullptr) {
    docSend["msg"] = F("JSON format error, syntax: {\"cmd\":{\"name\":<cmdName>, \"[payload\":{<data>}]}}");
    return;
  }

  /*********************** SAMPLING COMMAND ****************************/
  // e.g. {"cmd":{"name":"sample", "payload":{"type":"Serial", "rate":4000}}}
  if(strcmp(cmd, CMD_SAMPLE) == 0) {
    if (state == STATE_IDLE) {
      // For sampling we need type payload and rate payload
      const char* typeC = root["cmd"]["payload"]["type"];
      const char* measuresC = root["cmd"]["payload"]["measures"];
      int rate = docRcv["cmd"]["payload"]["rate"].as<int>();
      unsigned long ts = docRcv["cmd"]["payload"]["time"].as<unsigned long>();
      bool prefix = docRcv["cmd"]["payload"]["prefix"].as<bool>();
      JsonVariant prefixVariant = root["cmd"]["payload"]["prefix"];

      docSend["error"] = true;
      if (typeC == nullptr or rate == 0) {
        response = "Not a valid \"sample\" command";
        if (typeC == nullptr) response += ", \"type\" missing";
        if (rate == 0) response += ", \"rate\" missing";
        docSend["msg"] = response;
        return;
      }
      // We can only do these rates
      if (32000%rate == 0) {
        if (rate <= 8000) intSamplingRate = 8000;
        else intSamplingRate = 32000;
        int leavout = intSamplingRate/rate;
        leavoutSamples = leavout;
      } else {
        response = "SamplingRate could not be set to ";
        response += rate;
        docSend["msg"] = response;
        return;
      }
      if (measuresC == nullptr) {
        streamConfig.measures = STATE_3xVI;
        streamConfig.measurementBytes = 24;
      } else if (strcmp(measuresC, "v,i") == 0) {
        streamConfig.measures = STATE_VI;
        streamConfig.measurementBytes = 8;
      } else {
        response = "Unsupported measures";
        response += measuresC;
        docSend["msg"] = response;
        return;
      }
      streamConfig.prefix = true;
      // If we do not want a prefix, we have to disable this if not at extra port
      if (!prefixVariant.isNull()) {
        streamConfig.prefix = prefix;
      }
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"Serial", "rate":4000}}}
      if (strcmp(typeC, "Serial") == 0) {
        streamConfig.stream = USB;
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"TCP", "rate":4000}}}
      } else if (strcmp(typeC, "TCP") == 0) {
        sendClient = (WiFiClient*)&getter; 
        streamConfig.stream = TCP;
        streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
        streamConfig.ip = sendClient->remoteIP();
      // e.g. {"cmd":{"name":"sample", "payload":{"type":"UDP", "rate":4000}}}
      } else if (strcmp(typeC, "UDP") == 0) {
        streamConfig.stream = UDP;
        int port = docRcv["cmd"]["payload"]["port"].as<int>();
        if (port > 80000 || port <= 0) {
          streamConfig.port = STANDARD_UDP_PORT;
          response = "Unsupported UDP port";
          response += port;
          docSend["msg"] = response;
          return;
        } else {
          streamConfig.port = port;
        }
        sendClient = (WiFiClient*)&getter;
        docSend["port"] = streamConfig.port;
        streamConfig.ip = sendClient->remoteIP();
      } else if (strcmp(typeC, "FFMPEG") == 0) {

        bool success = streamClient.connected();
        if (!success) {
          // Look for people connecting over the streaming server and connect them
          streamClient = streamServer.available();
          if (streamClient && streamClient.connected()) success = true;
        }
        if (success) {
          response = F("Connected to TCP stream");
        } else {
          docSend["msg"] = F("Could not connect to TCP stream");
          return;
        }
      } else {
        response = F("Unsupported sampling type: ");
        response += typeC;
        docSend["msg"] = response;
        return;
      }
      // Set global sampling variable
      streamConfig.samplingRate = rate;
      calcChunkSize();
      
      docSend["sampling_rate"] = streamConfig.samplingRate;
      docSend["chunk_size"] = streamConfig.chunkSize;
      docSend["conn_type"] = typeC;
      docSend["measurement_bytes"] = streamConfig.measurementBytes;
      docSend["prefix"] = streamConfig.prefix;
      docSend["cmd"] = CMD_SAMPLE;

      next_state = STATE_SAMPLE;

      if (ts != 0) {
        response += F("Should sample at: ");
        response += myTime.timeStr(ts, 0);
        // Update ntp time actively wait for finish
        myTime.updateNTPTime(true);
        uint32_t delta = ts - myTime.utc_seconds();
        uint32_t nowMs = millis();
        delta *= 1000;
        delta -= myTime.milliseconds();
        if (delta > 20000 or delta < 500) {
          response += F("//nCannot start sampling in: "); response += delta; response += F("ms");
          streamConfig.countdown = 0;
        } else {
          response += F("//nStart sampling in: "); response += delta; response += F("ms");
          streamConfig.countdown = nowMs + delta;
          docSend["error"] = false;
        }
        docSend["msg"] = String(response);
        return;
      }
      docSend["error"] = false;
      state = next_state;
      // UDP packets are not allowed to exceed 1500 bytes, so keep size reasonable
      startSampling();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** STOP COMMAND ****************************/
  // e.g. {"cmd":{"name":"stop"}}
  else if (strcmp(cmd, CMD_STOP) == 0) {
    // State is reset in stopSampling
    stopSampling();
    // Write remaining chunks with tail
    writeChunks(true);
    docSend["msg"] = F("Received stop command");
    docSend["sample_duration"] = samplingDuration;
    docSend["samples"] = totalSamples;
    docSend["sent_samples"] = sentSamples;
    docSend["ip"] = Network::localIP().toString();
    docSend["avg_rate"] = totalSamples/(samplingDuration/1000.0);
    docSend["cmd"] = CMD_STOP;
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":{"name":"restart"}}
  else if (strcmp(cmd, CMD_RESTART) == 0) {
    ESP.restart();
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":{"name":"factoryReset"}}
  else if (strcmp(cmd, CMD_RESET) == 0) {
    config.makeDefault();
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  // e.g. {"cmd":{"name":"info"}}
  else if (strcmp(cmd, CMD_INFO) == 0) {
    docSend["cmd"] = "info";
    docSend["type"] = F("smartmeter");
    docSend["version"] = VERSION;
    String compiled = __DATE__;
    compiled += " ";
    compiled += __TIME__;
    docSend["compiled"] = compiled;
    docSend["sys_time"] = myTime.timeStr();
    docSend["name"] = config.name;
    docSend["ip"] = Network::localIP().toString();
    docSend["sampling_rate"] = streamConfig.samplingRate;
    docSend["buffer_size"] = ringBuffer.getSize();
    docSend["psram"] = ringBuffer.inPSRAM();
    docSend["rtc"] = rtc.connected;
    docSend["state"] = state != STATE_IDLE ? "busy" : "idle";
    String ssids = "[";
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      if (i < config.numAPs-1) ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":{"name":"mdns", "payload":{"name":"newName"}}}
  else if (strcmp(cmd, CMD_MDNS) == 0) {
    if (state == STATE_IDLE) {
      docSend["error"] = true;
      const char* newName = docRcv["cmd"]["payload"]["name"];
      if (newName == nullptr) {
        docSend["msg"] = F("MDNS name required in payload with key name");
        return;
      }
      if (strlen(newName) < MAX_NAME_LEN) {
        config.setName((char * )newName);
      } else {
        response = F("MDNS name too long, only string of size ");
        response += MAX_NAME_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * name = config.name;
      response = F("Set MDNS name to: ");
      response += name;
      //docSend["msg"] = sprintf( %s", name);
      docSend["msg"] = response;
      docSend["mdns_name"] = name;
      docSend["error"] = false;
      initMDNS();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** ADD WIFI COMMAND ****************************/
  // e.g. {"cmd":{"name":"addWifi", "payload":{"ssid":"ssidName","pwd":"pwdName"}}}
  else if (strcmp(cmd, CMD_ADD_WIFI) == 0) {
    if (state == STATE_IDLE) {
      docSend["error"] = true;
      const char* newSSID = docRcv["cmd"]["payload"]["ssid"];
      const char* newPWD = docRcv["cmd"]["payload"]["pwd"];
      if (newSSID == nullptr or newPWD == nullptr) {
        docSend["msg"] = F("WiFi SSID and PWD required, for open networks, fill empty pwd");
        return;
      }
      bool success = false;
      if (strlen(newSSID) < MAX_SSID_LEN and strlen(newPWD) < MAX_PWD_LEN) {
        success = config.addWiFi((char * )newSSID, (char * )newPWD);
      } else {
        response = F("SSID or PWD too long, max: ");
        response += MAX_SSID_LEN;
        response += F(", ");
        response += MAX_PWD_LEN;
        docSend["msg"] = response;
        return;
      }
      if (success)  {
        char * name = config.wifiSSIDs[config.numAPs-1];
        char * pwd = config.wifiPWDs[config.numAPs-1];
        response = F("New Ap, SSID: ");
        response += name;
        response += F(", PW: ");
        response += pwd;
        //docSend["msg"] = sprintf( %s", name);
        docSend["ssid"] = name;
        docSend["pwd"] = pwd;
        docSend["error"] = false;
      } else {
        response = F("MAX # APs reached, need to delete first");
      }

      docSend["msg"] = response;
      String ssids = "[";
      for (int i = 0; i < config.numAPs; i++) {
        ssids += config.wifiSSIDs[i];
        ssids += ", ";
      }
      ssids += "]";
      docSend["ssids"] = ssids;
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** DEl WIFI COMMAND ****************************/
  // e.g. {"cmd":{"name":"delWifi", "payload":{"ssid":"ssidName"}}}
  else if (strcmp(cmd, CMD_REMOVE_WIFI) == 0) {
    if (state == STATE_IDLE) {
      docSend["error"] = true;
      const char* newSSID = docRcv["cmd"]["payload"]["ssid"];
      if (newSSID == nullptr) {
        docSend["msg"] = F("Required SSID to remove");
        return;
      }
      bool success = false;
      if (strlen(newSSID) < MAX_SSID_LEN) {
        success = config.removeWiFi((char * )newSSID);
      } else {
        response = F("SSID too long, max: ");
        response += MAX_SSID_LEN;
        docSend["msg"] = response;
        return;
      }
      if (success)  {
        response = F("Removed SSID: ");
        response += newSSID;
        docSend["error"] = false;
      } else {
        response = F("SSID ");
        response += newSSID;
        response += F(" not found");
      }
      docSend["msg"] = response;
      String ssids = "[";
      for (int i = 0; i < config.numAPs; i++) {
        ssids += config.wifiSSIDs[i];
        ssids += ", ";
      }
      ssids += "]";
      docSend["ssids"] = ssids;
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** NTP COMMAND ****************************/
  // e.g. {"cmd":{"name":"ntp"}}
  else if (strcmp(cmd, CMD_NTP) == 0) {
    if (myTime.updateNTPTime(true)) {
      docSend["msg"] = "Time synced";
      docSend["error"] = false;
    } else {
      docSend["msg"] = "Error syncing time";
      docSend["error"] = true;
    }
    char * timeStr = myTime.timeStr();
    docSend["current_time"] = timeStr;
  }

  /*********************** Clear Log COMMAND ****************************/
  // e.g. {"cmd":{"name":"clearLog"}}
  else if (strcmp(cmd, CMD_CLEAR_LOG) == 0) {
    if (state == STATE_IDLE) {
      docSend["error"] = false;
      spiffsLog.clear();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** Get Log COMMAND ****************************/
  // e.g. {"cmd":{"name":"getLog"}}
  else if (strcmp(cmd, CMD_GET_LOG) == 0) {
    if (state == STATE_IDLE) {
      spiffsLog.flush();
      docSend["error"] = false;
      bool hasRow = spiffsLog.nextRow(&command[0]);
      getter.printf("%s{\"cmd\":\"log\",\"msg\":\"", &LOG_PREFIX[0]);
      getter.printf("*** LOGFile *** //n");
      while(hasRow) {
        getter.printf("%s//n", &command[0]);
        hasRow = spiffsLog.nextRow(&command[0]);
      }
      getter.println("*** LOGFile *** \"}");
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
}


/****************************************************
 * Stop Sampling will go into idle state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  if (rtc.connected) rtc.disableInterrupt(); // TODO

  state = STATE_IDLE;
  next_state = STATE_IDLE;
  
  ade9k.stopSampling();

  samplingDuration = millis() - startSamplingMillis;
  if (streamClient && streamClient.connected()) streamClient.stop();
  // Reset all variables
  streamConfig.countdown = 0;
  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

/****************************************************
 * Calculate the chunk size depending on the
 * samplingrate and sample method
 ****************************************************/
void calcChunkSize() {
  uint32_t chunkSize = int(float(0.1*(float)(streamConfig.samplingRate*streamConfig.measurementBytes)));
  // Keep within bounds
  if (chunkSize > MAX_SEND_SIZE) chunkSize = MAX_SEND_SIZE;
  else if (chunkSize < streamConfig.measurementBytes) chunkSize = streamConfig.measurementBytes;
  // Calc next base two
  chunkSize--;
  chunkSize |= chunkSize >> 1;
  chunkSize |=chunkSize >> 2;
  chunkSize |= chunkSize >> 4;
  chunkSize |= chunkSize >> 8;
  chunkSize |= chunkSize >> 16;
  chunkSize++;
  // Upper bound is MAX_SEND_SIZE (see buffersize)
  chunkSize = min((int)chunkSize, MAX_SEND_SIZE);
  // For UDP we only allow 512, because no nagle algorithm will split
  // the data into subframes like in the tcp case
  if (streamConfig.stream == UDP) {
    chunkSize = min((int)chunkSize, 512);
  } else if (streamConfig.stream == USB) {
    // For mac we only must read after max 1020 bytes so chunk size must be smaller 
    chunkSize = 128;
    // chunkSize = min((int)chunkSize, 16);
  }
  // Look that we always have an integer multiple of a complete measruement inside a chunk
  chunkSize = chunkSize/streamConfig.measurementBytes*streamConfig.measurementBytes;
  chunkSize = max((int)streamConfig.measurementBytes, (int)chunkSize);
  streamConfig.chunkSize = chunkSize;
}

/****************************************************
 * Start Sampling requires to start the interrupt and
 * to calculate the chunkSize size depending on
 * the samplingrate currently set. Furthermore,
 * all buffer indices are reset to the default values.
 ****************************************************/
inline void startSampling() {
  // Reset all variables
  state = STATE_SAMPLE;

  xTaskCreatePinnedToCore(sample_timer_task,     /* Task function. */
        "sampleTask",       /* String with name of task. */
        4096*2,            /* Stack size in words. */
        NULL,             /* Parameter passed as input of the task */
        20,                /* Priority of the task. */
        &xHandle,            /* Task handle. */
        1); // Same task as the loop
        //  Network::ethernet? 1 : 0); // On wifi use core 0 on ethernet core 1 since wifi requires core 0 to be mostly idle

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  interruptSamples = 0;
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
  
  if (rtc.connected) rtc.enableInterrupt(1, sqwvTriggered); // TODO
  
  freqCalcNow = millis();
  freqCalcStart = millis();
  startSamplingMillis = millis();
  
  ade9k.startSampling(intSamplingRate);
}



/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  char * name = config.name;
  if (strlen(name) == 0) {
    logger.log(ERROR, "Sth wrong with mdns");
    strcpy(name,"smartMeterX");
  }
  // Setting up MDNs with the given Name
  logger.log("MDNS Name: %s", name);
  if (!MDNS.begin(String(name).c_str())) {             // Start the mDNS responder for esp8266.local
    logger.log(ERROR, "Setting up MDNS responder!");
  }
  MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
}

// Make sure enough memory is allocated for str
void setInfoString(char * str) {
  int idx = 0;
  idx += sprintf(&str[idx], "\n");
  // Name and firmware
  idx += sprintf(&str[idx], "%s @ firmware: %s/%s", config.name, __DATE__, __TIME__);
  idx += sprintf(&str[idx], "\nUsing %d bytes ", ringBuffer.getSize());
  // PSRAM or not
  if (ringBuffer.inPSRAM()) {
    idx += sprintf(&str[idx], "PSRAM");
  } else {
    idx += sprintf(&str[idx], "RAM");
  }

  if (rtc.connected) {
    idx += sprintf(&str[idx], "\nRTC is connected ");
    if (rtc.lost) {
      idx += sprintf(&str[idx], "but has lost time");
    } else {
      idx += sprintf(&str[idx], "and has valid time");
    }
    idx += sprintf(&str[idx], "\nRTCTime: ");
    idx += sprintf(&str[idx], rtc.timeStr());
  } else {
    idx += sprintf(&str[idx], "\nNo RTC");
  }

  idx += sprintf(&str[idx], "\nCurrent system time: ");
  idx += sprintf(&str[idx], myTime.timeStr());
  // All SSIDs
  String ssids = "";
  for (int i = 0; i < config.numAPs; i++) {
    ssids += config.wifiSSIDs[i];
    if (i < config.numAPs-1) ssids += ", ";
  }
  idx += sprintf(&str[idx], "\nKnown Networks: [%s]", ssids.c_str());
  idx += sprintf(&str[idx], "\n");
}
