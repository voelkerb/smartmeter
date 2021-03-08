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
#include "enums.h"
#include "src/multiLogger/src/multiLogger.h"
#include "src/time/src/timeHandling.h"
#include "src/DS3231_RTC/src/DS3231_RTC.h"
#include "src/ADE9000/src/ADE9000.h"
#include "src/config/config.h"
#include "src/network/src/network.h"
#include "src/ringbuffer/src/ringbuffer.h"
#include "src/mqtt/src/mqtt.h"

void IRAM_ATTR isr_adc_ready();
void IRAM_ATTR sqwvTriggered(void* instance);
void ntpSynced();

// Serial logger
StreamLogger serialLog((Stream*)&Serial, &timeStr, &LOG_PREFIX_SERIAL[0], ALL);
// SPIFFS logger
SPIFFSLogger spiffsLog(false, &LOG_FILE[0], &timeStr, &LOG_PREFIX_SERIAL[0], WARNING);

// MultiLogger logger(&streamLog, &timeStr);
// Create singleton here
MultiLogger& logger = MultiLogger::getInstance();

Configuration config;

Rtc rtc(RTC_INT, SDA, SCL);
TimeHandler myTime(config.timeServer, LOCATION_TIME_OFFSET, &rtc, &ntpSynced);

// ADE900 Object
ADE9000 ade9k(ADE_RESET_PIN, ADE_DREADY_PIN, ADE_PM1_PIN, ADE_SPI_BUS);

RingBuffer ringBuffer(PS_BUF_SIZE, true);

// counter holds # isr calls
volatile uint32_t counter = 0;
// Last micros() count of isr call
volatile long nowTs = 0;
volatile long lastTs = 0;

#define SEND_BUF_SIZE MAX_SEND_SIZE+16
static uint8_t sendbuffer[SEND_BUF_SIZE] = {0};
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

SampleState state = SampleState::IDLE;
SampleState next_state = SampleState::IDLE;

// Define it before StreamType enum redefines it
MQTT mqtt;

struct StreamConfig {
  bool prefix = false;                      // Send data with "data:" prefix
  bool flowControl = false;                 // ifg flow ctr is used
  Measures measures = Measures::VI;         // The measures to send
  uint8_t measurementBytes = 24;            // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  StreamType stream = StreamType::USB;      // Channel over which to send
  unsigned int countdown = 0;               // Start at specific time or immidiately
  uint32_t chunkSize = 0;                   // Chunksize of one packet sent
  IPAddress ip;                             // Ip address of data sink
  uint16_t port = STANDARD_TCP_SAMPLE_PORT; // Port of data sink
  size_t numValues = 24/sizeof(float);
  Timestamp startTs;
  Timestamp stopTs;
  int slots = 0;
  int slot = 0;
};
bool rts; // ready to send

int correctSamples = 0;

char measureStr[MAX_MEASURE_STR_LENGTH] = {'\0'};
char unitStr[MAX_UNIT_STR_LENGTH] = {'\0'};

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
// Init getter to point to sth, might not work otherwise
Stream * newGetter = (Stream*)&Serial;
// tcp stuff is send over this client 
Stream * sendStream = (Stream*)&Serial;
WiFiClient * sendClient;
// UDP used for streaming
WiFiUDP udpClient;

// Stream client for external stream server
WiFiClient exStreamServer;
bool exStreamServerNewConnection = true;

// Command stuff send over what ever
char command[COMMAND_MAX_SIZE] = {'\0'};
StaticJsonDocument<2*COMMAND_MAX_SIZE> docRcv;
StaticJsonDocument<2*COMMAND_MAX_SIZE> docSend;
StaticJsonDocument<COMMAND_MAX_SIZE> docSample;

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
long mqttUpdate = millis();

// Current CPU speed
unsigned int coreFreq = 0;


// test stuff
bool criticalMsgAvailable = false;
portMUX_TYPE criticalMsgMux = portMUX_INITIALIZER_UNLOCKED;
char criticalMsg[MAX_MQTT_PUB_TOPIC_INFO] = {'\0'};
long testMillis = 0;
long testMillis2 = 0;
uint16_t testSamples = 0;

portMUX_TYPE correctionMux = portMUX_INITIALIZER_UNLOCKED;

// Mutex for 1s RTC Interrupt
portMUX_TYPE sqwMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE intterruptSamplesMux = portMUX_INITIALIZER_UNLOCKED;
bool firstSqwv = true;
volatile int sqwCounter = 0;
volatile int sqwCounter2 = 0;

volatile uint32_t interruptSamples = 0;

// Must be large enough to handle maximum mnumber of floats for one measurement
// e.g. V_L1, V_L2, V_L3, I_L1, I_L2, I_L3
float values[7] = {0};

TaskHandle_t xHandle = NULL;

bool updating = false;

char mqttTopicPubSwitch[MAX_MQTT_PUB_TOPIC_SWITCH+MAX_NAME_LEN] = {'\0'};
char mqttTopicPubSample[MAX_MQTT_PUB_TOPIC_SAMPLE+MAX_NAME_LEN] = {'\0'};
char mqttTopicPubInfo[MAX_MQTT_PUB_TOPIC_INFO+MAX_NAME_LEN] = {'\0'};


// HW Timer and mutex for sapmpling ISR
hw_timer_t * timer = NULL;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

void (*outputWriter)(bool) = &writeChunks;
/************************ SETUP *************************/
void setup() {
  // Setup serial communication
  Serial.begin(SERIAL_SPEED);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, HIGH);

  // At first init the rtc module to get 
  bool successAll = true;

  bool success = rtc.init();
  
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

  
  config.init();
  config.load();

  coreFreq = getCpuFrequencyMhz();
  logger.log(DEBUG, "%s @ firmware %s/%s", config.netConf.name, __DATE__, __TIME__);
  logger.log(DEBUG, "Core @ %u MHz", coreFreq);


  success = ringBuffer.init();
  if (!success) logger.log(ERROR, "PSRAM init failed");
  successAll &= success;
  // Try to use internal ram for buffering, but still indicate the missing ram
  if (!success) {
    success = ringBuffer.init();
    if (!success) logger.log(ERROR, "RAM init failed: %i bytes", RAM_BUF_SIZE);
  }
  // NOTE: ERROR LED and ADE PM1 share the same pin.
  // For normal working condition, PM1 need to be low
  digitalWrite(ERROR_LED, LOW);

  // Setup ADE9000
  ade9k.initSPI(ADE_SCK, ADE_MISO, ADE_MOSI, ADE_CS);
  success = ade9k.init(&isr_adc_ready);
  if (!success) logger.log(ERROR, "ADE Init Failed");
  successAll &= success;
  
  if (successAll) digitalWrite(ERROR_LED, LOW);
  else digitalWrite(ERROR_LED, HIGH);
  logger.log(ALL, "Connecting Network");

  // Init network connection
  Network::init(&config.netConf, onNetworkConnect, onNetworkDisconnect, false, &logger);
  Network::initPHY(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  setupOTA();

  response.reserve(2*COMMAND_MAX_SIZE);

  // Set mqtt and callbacks
  mqtt.init(config.mqttServer, config.netConf.name);
  mqtt.onConnect = &onMQTTConnect;
  mqtt.onDisconnect = &onMQTTDisconnect;
  mqtt.onMessage = &mqttCallback;

  timer_init();

  // Init send buffer
  snprintf((char *)&sendbuffer[0], SEND_BUF_SIZE, "%s", DATA_PREFIX);

  // print some info about this powermeter
  printInfoString();

  // TODO: This is only for testing purpose, if sqwv interrupt gets lost without sampling
  // if (rtc.connected) {
  //   rtc.enableInterrupt(1, sqwvTriggered2); // TODO
  //   setupSqwvInterrupt();
  // }
  logger.log(ALL, "Setup done");
}

/************************ Loop *************************/
void loop() {
  // We don't do anything else while updating
  if (updating) return;

  // Stuff done on idle
  if (state == SampleState::IDLE) {
    onIdle();
  // Stuff on sampling
  } else {
    onSampling();
  }

  onIdleOrSampling();
  
  // If we only have 200 ms before sampling should start, wait actively
  if (next_state != SampleState::IDLE) {
    if (streamConfig.countdown != 0 and (streamConfig.countdown - millis()) < 200) {
      // Disable any wifi sleep mode
      if (not Network::ethernet) esp_wifi_set_ps(WIFI_PS_NONE);
      state = next_state;
      logger.log(DEBUG, "StartSampling @ %s", myTime.timeStr());
      // Calculate time to wait
      int32_t mydelta = streamConfig.countdown - millis();
      // Waiting here is not nice, but we wait for zero crossing
      if (mydelta > 0) delay(mydelta-1);
      if (state == SampleState::SAMPLE) {
        startSampling();
      } else {
        startLatchedSampling(true);
      }
      // Reset sampling countdown
      streamConfig.countdown = 0;
    }
  }

  // if (criticalMsgAvailable) {
  //   portENTER_CRITICAL(&criticalMsgMux);
  //   criticalMsgAvailable = false;
  //   portEXIT_CRITICAL(&criticalMsgMux);
  //   if (not firstSqwv and testMillis2 > 1050) {
  //     logger.log(ERROR, "Sqwv took: %li ms", testMillis2);
  //     if (firstSqwv) firstSqwv = false;
  //   } else {
  //     logger.log(INFO, "Sqwv took: %li ms", testMillis2);
  //   }
  // }

  // Watchdog
  yield();
}

/****************************************************
 * What todo independent of sampling state
 ****************************************************/
void onIdleOrSampling() {
  // MQTT loop
  mqtt.update();


  #ifndef CMD_OVER_SERIAL_WHILE_TCP_SAMPLING
  if (streamConfig.stream == StreamType::USB or state == SampleState::IDLE) {
  #endif  
    // Handle serial requests
    if (Serial.available()) {
      logger.log(ERROR, "MESSAGE Over Serial");
      handleEvent(&Serial);
    }
  #ifndef CMD_OVER_SERIAL_WHILE_TCP_SAMPLING
  }
  #endif  

  // Handle tcp requests
  for (size_t i = 0; i < MAX_CLIENTS; i++) {
    if (client[i].available() > 0) {
      handleEvent(&client[i]);
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

/****************************************************
 * Things todo regularly if we are not sampling
 ****************************************************/
void onIdle() {

  // Arduino OTA
  ArduinoOTA.handle();
  
  spiffsLog.flush();

  // Re-advertise MDNS service service every 30s 
  // TODO: no clue why, but does not work properly for esp32 (maybe it is the mac side)
  if ((long)(millis() - mdnsUpdate) >= 0) {
    mdnsUpdate += MDNS_UPDATE_INTERVAL;
    if ((long)(millis() - mdnsUpdate) >= 0) mdnsUpdate = millis() + MDNS_UPDATE_INTERVAL; 
    // initMDNS();
    //MDNS.addService("_elec", "_tcp", STANDARD_TCP_STREAM_PORT);
  }

  if ((long)(millis() - rtcUpdate) >= 0) {
    rtcUpdate += RTC_UPDATE_INTERVAL;
    if ((long)(millis() - rtcUpdate) >= 0) rtcUpdate = millis() + RTC_UPDATE_INTERVAL; 
    if (rtc.connected) rtc.update();
  }
    
  // Update lifeness only on idle every second
  if ((long)(millis() - lifenessUpdate) >= 0) {
    lifenessUpdate += LIFENESS_UPDATE_INTERVAL;
    if ((long)(millis() - lifenessUpdate) >= 0) lifenessUpdate = millis() + LIFENESS_UPDATE_INTERVAL; 
    logger.log("");
  }

  // Update stuff over mqtt
  if (mqtt.connected()) {
    if ((long)(millis() - mqttUpdate) >= 0) {
      mqttUpdate += MQTT_UPDATE_INTERVAL;
      // On long time no update, avoid multiupdate
      if ((long)(millis() - mqttUpdate) >= 0) mqttUpdate = millis() + MQTT_UPDATE_INTERVAL; 
      sendStatusMQTT();
    }
  }
  
  /*
   * NOTE: The streamserver has to send start signal by itself now,
   * Lets see if this is a better idea
   * If you want to reenable automatic sending, uncomment it
   * exStreamServer is now more an external server where we announce that we are now ready
   * to send data
   */
  // Check external stream server connection
  // if (exStreamServer.connected() and exStreamServerNewConnection) {
  //   logger.log("Stream Server connected start sampling");
  //   // If streamserver sends stop, we need a way to prevent a new start of sampling
  //   // This is how we achieve it.
  //   exStreamServerNewConnection = false;
  //   // Set everything to default settings
  //   standardConfig();
  //   streamConfig.port = STANDARD_TCP_STREAM_PORT;
  //   sendClient = (WiFiClient*)&exStreamServer;
  //   sendStream = sendClient;
  //   startSampling();
  //   // Must be done after start st startTS is in data
  //   sendStreamInfo(sendClient);
  // }

  // Look for people connecting over the stream server
  // If one connected there, immidiately start streaming data
  if (!streamClient.connected()) {
    streamClient = streamServer.available();
    if (streamClient.connected()) {
      standardConfig();
      streamConfig.prefix = false;
      streamConfig.port = STANDARD_TCP_STREAM_PORT;
      sendClient = (WiFiClient*)&streamClient;
      sendStream = sendClient;
      startSampling();
    }
  }
}

/****************************************************
 * Send Status info over mqtt 
 ****************************************************/
void sendStatusMQTT() {
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();

  float value = 0.0;


  
  ade9k.readActivePower(&values[0]);
  value = values[0]+values[1]+values[2];
  docSend["power"] = round2<float>(value);

  // We want current in A
  ade9k.readCurrentRMS(&values[0]);
  value = values[0]+values[1]+values[2]+values[3];
  docSend["current"] = round2<float>(value/1000.0);

  // unit is watt hours and we want kwj
  // TODO: This needs some major reworking
  // ade9k.readActiveEnergy(&data[0], &logger);
  // docSend["energy"] = value;

  // use avg voltage here
  ade9k.readVoltageRMS(&values[0]);
  value = (values[0]+values[1]+values[2])/3.0;
  docSend["volt"] = round2<float>(value);
  
  docSend["ts"] = myTime.timestampStr(true);
  response = "";
  serializeJson(docSend, response);
  logger.log("MQTT msg: %s", response.c_str());
  mqtt.publish(mqttTopicPubSample, response.c_str());
}

/****************************************************
 * Send Stream info to 
 ****************************************************/
void sendDeviceInfo(Stream * sender) {
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  docSend["cmd"] = "info";
  docSend["type"] = F("smartmeter");
  docSend["version"] = VERSION;
  String compiled = __DATE__;
  compiled += " ";
  compiled += __TIME__;
  docSend["compiled"] = compiled;
  docSend["sys_time"] = myTime.timeStr();
  docSend["name"] = config.netConf.name;
  docSend["ip"] = Network::localIP().toString();
  docSend["mqtt_server"] = config.mqttServer;
  docSend["stream_server"] = config.streamServer;
  docSend["time_server"] = config.timeServer;
  docSend["sampling_rate"] = streamConfig.samplingRate;
  docSend["buffer_size"] = ringBuffer.getSize();
  docSend["psram"] = ringBuffer.inPSRAM();
  docSend["rtc"] = rtc.connected;
  docSend["state"] = state != SampleState::IDLE ? "busy" : "idle";
  String ssids = "[";
  for (size_t i = 0; i < config.netConf.numAPs; i++) {
    ssids += config.netConf.SSIDs[i];
    if (i < config.netConf.numAPs-1) ssids += ", ";
  }
  ssids += "]";
  docSend["ssids"] = ssids;
  if (not Network::ethernet) {
    docSend["ssid"] = WiFi.SSID();
    docSend["rssi"] = WiFi.RSSI();
    docSend["bssid"] = Network::getBSSID();
  }
  response = "";
  serializeJson(docSend, response);
  response = LOG_PREFIX + response;
  sender->println(response);
}

/****************************************************
 * Send Stream info to 
 ****************************************************/
void sendStreamInfo(Stream * sender) {
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  docSend["name"] = config.netConf.name;
  docSend["measures"] = measuresToStr(streamConfig.measures);
  docSend["chunksize"] = streamConfig.chunkSize;
  docSend["samplingrate"] = streamConfig.samplingRate;
  docSend["measurements"] = streamConfig.numValues;
  docSend["prefix"] = streamConfig.prefix;
  docSend["cmd"] = CMD_SAMPLE;
  docSend["unit"] = unitToStr(streamConfig.measures);
  response = "";
  serializeJson(docSend, response);
  response = LOG_PREFIX + response;
  sender->println(response);
}

/****************************************************
 * return unit as str
 ****************************************************/
char * unitToStr(Measures measures) {
  switch (measures) {
    case Measures::VI:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_VI);
      break;
    case Measures::VI_L1:
    case Measures::VI_L2:
    case Measures::VI_L3:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_VI_LX);
      break;
    case Measures::PQ:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_PQ);
      break;
    case Measures::VI_RMS:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, UNIT_VI_RMS);
      break;
    default:
      snprintf(unitStr, MAX_UNIT_STR_LENGTH, "unknown");
      break;
  }
  return &unitStr[0];
}

/****************************************************
 * Convert measure to str
 ****************************************************/
char * measuresToStr(Measures measures) {
  switch (measures) {
    case Measures::VI:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI);
      break;
    case Measures::VI_L1:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI_L1);
      break;
    case Measures::VI_L2:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI_L2);
      break;
    case Measures::VI_L3:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI_L3);
      break;
    case Measures::PQ:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_PQ_LONG);
      break;
    case Measures::VI_RMS:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, MEASURE_VI_RMS_LONG);
      break;
    default:
      snprintf(measureStr, MAX_MEASURE_STR_LENGTH, "unknown");
      break;
  }
  return &measureStr[0];
}

/****************************************************
 * Set standard configuration
 ****************************************************/
void standardConfig() {
  streamConfig.slot = 0;
  streamConfig.slots = 0;
  streamConfig.stream = StreamType::TCP;
  streamConfig.prefix = true;
  streamConfig.flowControl = false;
  streamConfig.samplingRate = DEFAULT_SR;
  streamConfig.measures = Measures::VI;
  streamConfig.ip = streamClient.remoteIP();
  streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
  outputWriter = &writeChunks;
  streamConfig.measurementBytes = 8;
  streamConfig.measurementBytes = 24;
  streamConfig.numValues = streamConfig.measurementBytes/sizeof(float);
  calcChunkSize();
  rts = true;
}

uint32_t sent = 0;
/****************************************************
 * What todo only on sampling (send data, etc)
 ****************************************************/
void onSampling() {
  // TODO: Chaos aufräumen
  // Simple seconds send slot calculation
  if (streamConfig.slots != 0) {
    Timestamp now = myTime.timestamp();
    while (now.seconds%streamConfig.slots == streamConfig.slot) {
      if (ringBuffer.available() > streamConfig.chunkSize) {
        writeData(*sendStream, streamConfig.chunkSize);
        sent += streamConfig.chunkSize;
      } else {
        break;
      }
      now = myTime.timestamp();
    }
    if (now.seconds%streamConfig.slots != streamConfig.slot and sent != 0) {
      if (ringBuffer.available() > streamConfig.chunkSize) {
        sent = ringBuffer.available()/streamConfig.measurementBytes;
        logger.log(WARNING, "not finisched %u missing", sent);
      } else {
        sent = sent/streamConfig.measurementBytes;
        logger.log("sent %u samples", sent);
      }
      sent = 0;
    }
  }
  //  if (streamConfig.slots != 0) {
  //   Timestamp now = myTime.timestamp();
  //   if (now.seconds%streamConfig.slots == streamConfig.slot) {
  //     rts = true;
  //   } else {
  //     rts = false;
  //   }
  // }

  // Send data to sink
  if (outputWriter and rts) outputWriter(false);

  // For error check during sampling
  if (sqwCounter) {
    portENTER_CRITICAL(&sqwMux);
    sqwCounter--;
    portEXIT_CRITICAL(&sqwMux);
    // if (!firstSqwv and testSamples != streamConfig.samplingRate) {
    if (!firstSqwv and testSamples != streamConfig.samplingRate) {
      response = "MISSED ";
      response += streamConfig.samplingRate - testSamples;
      response += " SAMPLES";
      logger.log(ERROR, response.c_str());
    }
    // If it is still > 0, our loop is too slow, 
    // reasons may be e.g. slow tcp write performance
    if (sqwCounter) {
      logger.log(WARNING, "Loop %us behind", sqwCounter);
      // Reset to 0
      portENTER_CRITICAL(&sqwMux);
      sqwCounter = 0;
      portEXIT_CRITICAL(&sqwMux);
    }
    // Ignore first sqwv showing missing samples since
    // sqwv did not start with sampling
    // NOTE: is this valid?
    // TODO: can we reset the sqwv somehow?
    if (firstSqwv) firstSqwv = false;
    // testMillis = testMillis2;
  }

  // Output sampling frequency regularly
  if (freq != 0) {
    long fr = freq;
    freq = 0;
    float frequency = (float)streamConfig.samplingRate/(float)((fr)/1000000.0);//  Logging only works for USB rates smaller 8000Hz
    if (!(streamConfig.stream == StreamType::USB and streamConfig.samplingRate > 4000)) {
      logger.log("%.2fHz, %us", frequency, totalSamples);
    }
  }

  // ______________ Handle Disconnect ________________

  // NOTE: Cannot detect disconnect for USB
  if (streamConfig.stream == StreamType::USB) {
  } else if (streamConfig.stream == StreamType::TCP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP disconnected while streaming");
      stopSampling();
    }
  // Disconnect of UDP means disconnecting from tcp port
  } else if (streamConfig.stream == StreamType::UDP) {
    if (!sendClient->connected()) {
      logger.log(ERROR, "TCP/UDP disconnected while streaming");
      stopSampling();
    }
  } else if (streamConfig.stream == StreamType::MQTT) {
    if (!mqtt.connected()) {
      logger.log(ERROR, "MQTT Server disconnected while streaming");
      stopSampling();
    }
  }
}

/****************************************************
 * Time getter function can be called at any time
 * Note: static function required for time getter 
 * function. e.g. for logger class
 ****************************************************/
char * timeStr() {
  return myTime.timeStr(true);
}

/****************************************************
 * If MQTT Server connection was successfull
 ****************************************************/
void onMQTTConnect() {
  logger.log("MQTT connected to %s", mqtt.ip);
  mqttSubscribe();
}

/****************************************************
 * If MQTT Server disconnected
 ****************************************************/
void onMQTTDisconnect() {
  logger.log("MQTT disconnected from %s", mqtt.ip);
}

/****************************************************
 * If ESP is connected to Network successfully
 ****************************************************/
void onNetworkConnect() {
  if (not Network::apMode) {
    logger.log(ALL, "Network Connected");
    logger.log(ALL, "IP: %s", Network::localIP().toString().c_str());

    // Reinit mdns
    initMDNS();
    
    // The stuff todo if we have a network connection (and hopefully internet as well)
    myTime.updateNTPTime();
    mqttUpdate = millis() + MQTT_UPDATE_INTERVAL;
    if (!mqtt.connect()) logger.log(ERROR, "Cannot connect to MQTT Server %s", mqtt.ip);
  } else {
    logger.log(ALL, "Network AP Opened");
  }

  // Start the TCP server
  server.begin();
  streamServer.begin();

  initStreamServer();
}

/****************************************************
 * If ESP disconnected from network
 ****************************************************/
void onNetworkDisconnect() {
  logger.log(ERROR, "Network Disconnected");

  if (state != SampleState::IDLE) {
    logger.log(ERROR, "Stop sampling (Network disconnect)");
    stopSampling();
  }
  if (mqtt.connected()) mqtt.disconnect();
}

/****************************************************
 * If a tcp client connects.
 * We store them in list and add logger
 ****************************************************/
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
      #ifdef SEND_INFO_ON_CLIENT_CONNECT
      sendDeviceInfo((Stream*)&client[i]);
      #endif
      client[i].setTimeout(10);
      return;
    }
  }
  logger.log("To much clients, could not add client");
  newClient.stop();
}

/****************************************************
 * If a tcp client disconnects.
 * We must remove the logger
 ****************************************************/
void onClientDisconnect(WiFiClient &oldClient, int i) {
  logger.log("Client discconnected %s port %u", oldClient.remoteIP().toString().c_str(), oldClient.remotePort());
  logger.removeLogger(streamLog[i]);
  streamLog[i]->_stream = NULL;
}

/****************************************************
 * Write chunks of data for TCP and Serial
 * depending on data sink
 ****************************************************/
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    writeData(*sendStream, streamConfig.chunkSize);
  }
  if (tail) {
    while(ringBuffer.available() > streamConfig.chunkSize) writeData(*sendStream, streamConfig.chunkSize);
    writeData(*sendStream, ringBuffer.available());
  }
}

/****************************************************
 * Write data via UDP, this requires special packet
 * handling
 ****************************************************/
void writeChunksUDP(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    udpClient.beginPacket(streamConfig.ip, streamConfig.port);
    writeData(udpClient, streamConfig.chunkSize);
    udpClient.endPacket();
  }
  if (tail) {
    while(ringBuffer.available() > streamConfig.chunkSize) {
      udpClient.beginPacket(streamConfig.ip, streamConfig.port);
      writeData(udpClient, streamConfig.chunkSize);
      udpClient.endPacket();
    }
    udpClient.beginPacket(streamConfig.ip, streamConfig.port);
    writeData(udpClient, ringBuffer.available());
    udpClient.endPacket();
  }
}

/****************************************************
 * Write data to mqtt sink, this requires special
 * formatiing of the data
 ****************************************************/
void writeDataMQTT(bool tail) {
  if(ringBuffer.available() < streamConfig.measurementBytes) return;

  // May be the most inefficient way to do it 
  // TODO: Improve this


  while(ringBuffer.available() >= streamConfig.measurementBytes) {

    JsonObject objSample = docSample.to<JsonObject>();
    objSample.clear();
    objSample["unit"] = unitStr;
    objSample["ts"] = myTime.timestampStr();
    ringBuffer.read((uint8_t*)&values[0], streamConfig.measurementBytes);
    // JsonArray array = docSend["values"].to<JsonArray>();
    // for (int i = 0; i < streamConfig.numValues; i++) objSample["values"][i] = values[i];
    JsonArray array = objSample["values"].to<JsonArray>();
    for (int i = 0; i < streamConfig.numValues; i++) array.add(values[i]);
    response = "";
    serializeJson(docSample, response);
    logger.log(response.c_str());
    mqtt.publish(mqttTopicPubInfo, response.c_str());
    sentSamples++;
  }
}

/****************************************************
 * Write one chunk of data to sink.
 ****************************************************/
void writeData(Stream &getter, uint16_t size) {
  if (size <= 0) return;
  uint32_t start = 0;
  if (streamConfig.prefix) {
    memcpy(&sendbuffer[start], (void*)&DATA_PREFIX[0], PREFIX_SIZE);
    start += PREFIX_SIZE;
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
  if (streamConfig.stream == StreamType::USB) getter.write("\r\n"); 
  if (sent > start) sentSamples += (sent-start)/streamConfig.measurementBytes;
  // sentSamples += size/streamConfig.measurementBytes;
}



/****************************************************
 * Sampling interrupt, A new ADC reading is available
 * Sample is read and put into queue
 * NOTE: Function must be small and quick
 ****************************************************/
// xQueueHandle xQueue = xQueueCreate(2000, sizeof(CurrentADC));
xQueueHandle xQueue = xQueueCreate(1000, sizeof(bool));
bool volatile IRAM_timeout = false;
float data[7] = {0.0};
volatile uint8_t cntLeavoutSamples = 0;
void IRAM_ATTR isr_adc_ready() {
  // Skip this interrup if we want less samples
  cntLeavoutSamples++;
  if (cntLeavoutSamples < leavoutSamples) return;
  cntLeavoutSamples = 0;
  if (interruptSamples >= streamConfig.samplingRate) return;
  portENTER_CRITICAL_ISR(&intterruptSamplesMux);
  interruptSamples++;
  portEXIT_CRITICAL(&intterruptSamplesMux);

  BaseType_t xHigherPriorityTaskWoken;
  // CurrentADC adc;
  // ade9k.readRawMeasurement(&adc);
  // BaseType_t xStatus = xQueueSendToBackFromISR( xQueue, &adc, &xHigherPriorityTaskWoken );
  bool success = false;
  BaseType_t xStatus = xQueueSendToBackFromISR( xQueue, &success, &xHigherPriorityTaskWoken );

  // BaseType_t xStatus = xQueueSendToBack( xQueue, &data, 10 );
  // check whether sending is ok or not
  if( xStatus == pdPASS ) {
  } else {
    IRAM_timeout = true;
  }

  xTaskResumeFromISR( xHandle );
}

/****************************************************
 * RTOS task for handling a new sample
 ****************************************************/
bool QueueTimeout = false;
void sample_timer_task( void * parameter) {
  vTaskSuspend( NULL );  // ISR wakes up the task

  // We can only get all data from adc, if we want a single phase, skip the rest
  uint8_t offset = 0;
  if (streamConfig.measures == Measures::VI) offset = 0;
  else if (streamConfig.measures == Measures::VI_L1) offset = 0;
  else if (streamConfig.measures == Measures::VI_L2) offset = 2;
  else if (streamConfig.measures == Measures::VI_L3) offset = 4;

  CurrentADC adc;
  bool success = false;
  while(state != SampleState::IDLE){

    BaseType_t xStatus = xQueueReceive( xQueue, &success, 0);

    if(xStatus == pdPASS) {


      counter++;

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

      int samplesToTake = 1;
      // If we are in the middle of a second sqwv
      if (totalSamples % streamConfig.samplingRate == 1) {
        // samplesCorrect > 0 means we have too less samples
        // samplesCorrect < 0 means we have too much samples
        // If we have to coorect samples upwards
        if (correctSamples > 0) {
          samplesToTake++;
          portENTER_CRITICAL_ISR(&correctionMux);
          correctSamples--;
          portEXIT_CRITICAL_ISR(&correctionMux);
        } else if (correctSamples < 0) {
          samplesToTake--;
          portENTER_CRITICAL_ISR(&correctionMux);
          correctSamples++;
          portEXIT_CRITICAL_ISR(&correctionMux);
        }
      }    
      // No matter what, we need to remove the sample from the fifo     
      ade9k.readRawMeasurement(&adc);
      ade9k.convertRawMeasurement(&adc, &data[0]);

      // With correction, this should normally be 1
      for (int i = 0; i < samplesToTake; i++) {
        totalSamples++;

        bool success = ringBuffer.write((uint8_t*)&(data[offset]), streamConfig.measurementBytes);
        if (!success) {
          uint32_t lostSamples = ringBuffer.getSize()/streamConfig.measurementBytes;
          logger.log(ERROR, "Overflow during ringBuffer write, lost %lu samples", lostSamples);
          ESP.restart();
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



/****************************************************
 * Sampling interrupt, must be small and quick
 ****************************************************/
volatile uint32_t mytime = micros();
static SemaphoreHandle_t timer_sem;

void IRAM_ATTR latched_sample_ISR() {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Vaiables for frequency count
  portENTER_CRITICAL_ISR(&counterMux);
  counter++;
  portEXIT_CRITICAL(&counterMux);

  xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
  
  if ( xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
  }
}


/****************************************************
 * RTOS task for sampling
 ****************************************************/
void IRAM_ATTR latched_sample_timer_task(void *param) {
  timer_sem = xSemaphoreCreateBinary();

  int test = 0;

  while (state != SampleState::IDLE) {
    xSemaphoreTake(timer_sem, portMAX_DELAY);
    
    // Vaiables for frequency count
    if (counter >= streamConfig.samplingRate) {
      freqCalcNow = micros();
      freq = freqCalcNow-freqCalcStart;
      freqCalcStart = freqCalcNow;
      if (rtc.connected) timerAlarmDisable(timer);
    }

    int samplesToTake = 1;
    // If we are in the middle of a second sqwv
    if (totalSamples % streamConfig.samplingRate == 1) {
      // samplesCorrect > 0 means we have too less samples
      // samplesCorrect < 0 means we have too much samples
      // If we have to coorect samples upwards
      if (correctSamples > 0) {
        samplesToTake++;
        portENTER_CRITICAL_ISR(&correctionMux);
        correctSamples--;
        portEXIT_CRITICAL_ISR(&correctionMux);
      } else if (correctSamples < 0) {
        samplesToTake--;
        portENTER_CRITICAL_ISR(&correctionMux);
        correctSamples++;
        portEXIT_CRITICAL_ISR(&correctionMux);
      }
    } 
    // With correction, this should normally be 1
    for (int i = 0; i < samplesToTake; i++) {
      totalSamples++;

      if (streamConfig.measures == Measures::VI_RMS) {
        ade9k.readVoltageRMS(&data[0]);
        ade9k.readCurrentRMS(&data[3]);
      } else if (streamConfig.measures == Measures::PQ) {
        ade9k.readActivePower(&data[0]);
        ade9k.readReactivePower(&data[3]);
      }
      if (!ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes)) {
        uint32_t lostSamples = ringBuffer.getSize()/streamConfig.measurementBytes;
        logger.log(ERROR, "Overflow during ringBuffer write, lost %lu samples", lostSamples);
        ESP.restart();
      }
    }

  }
  vTaskDelete( NULL );
}


/****************************************************
 * Setting up the SQWV Interrupt must be done on 
 * core 0 because internally same isr is used for 
 * all pin interrupts on a core
 * and this somehow conflicts with the ade interrupt
 ****************************************************/
TaskHandle_t Task1;
void setupSqwvInterrupt() {
  // Enable 1Hz output of RTC
  rtc.enableRTCSqwv(1);
  
  // NOTE: Seems to work

  // if (state == SampleState::SAMPLE_LATCHED) {
  //   // Setup sqwv interrupt on core 0
  //   xTaskCreatePinnedToCore(
  //         setupSqwvInterruptC0,        
  //         "setupSqwvInterruptC0",    
  //         4000,      
  //         NULL,
  //         10, 
  //         &Task1,
  //         1); 
  // } else {
  //   // Setup sqwv interrupt on core 0
  //   xTaskCreatePinnedToCore(
  //         setupSqwvInterruptC0,        
  //         "setupSqwvInterruptC0",    
  //         4000,      
  //         NULL,
  //         10, 
  //         &Task1,
  //         0); 
  // }
  // Setup sqwv interrupt on core 0
  xTaskCreatePinnedToCore(
        setupSqwvInterruptC0,        
        "setupSqwvInterruptC0",    
        4000,      
        NULL,
        10, 
        &Task1,
        0); 
}
const gpio_num_t rtc_int_pin = (gpio_num_t)RTC_INT;

void setupSqwvInterruptC0( void * parameter ) {
  gpio_install_isr_service(3);

  // Configure interrupt for low level
  gpio_config_t config = {
		.pin_bit_mask = 1ull << rtc_int_pin,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&config));
  esp_err_t err;
  
  err = gpio_set_intr_type(rtc_int_pin, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) Serial.printf("set intr type sqwv failed with error 0x%x \r\n", err);

  err = gpio_isr_handler_add(rtc_int_pin, &sqwvTriggered, (void *) nullptr);
  if (err != ESP_OK) Serial.printf("handler add sqwv failed with error 0x%x \r\n", err);
  
  vTaskDelete(NULL); // destroy this task 
}


/****************************************************
 * Disable the SQWV Interrupt, which must be done on 
 * the core it is enabled
 ****************************************************/
TaskHandle_t Task2;
void disableSqwvInterrupt() {
  // Interrupt must be deactivated from core it is activated

  // NOTE: Seems to work (unneccessary?)

  // if (state == SampleState::SAMPLE_LATCHED) {
  //   xTaskCreatePinnedToCore(
  //       disableSqwvInterruptC0,
  //       "disableSqwvInterruptC0", 
  //       4000,
  //       NULL,
  //       10,
  //       &Task2,
  //       1); 
  // } else {
  //   xTaskCreatePinnedToCore(
  //       disableSqwvInterruptC0,
  //       "disableSqwvInterruptC0", 
  //       4000,
  //       NULL,
  //       10,
  //       &Task2,
  //       0); 
  // }

  xTaskCreatePinnedToCore(
      disableSqwvInterruptC0,
      "disableSqwvInterruptC0", 
      4000,
      NULL,
      10,
      &Task2,
      0); 
}
void disableSqwvInterruptC0( void * parameter ) {
  gpio_isr_handler_remove(rtc_int_pin);
  gpio_uninstall_isr_service();
  vTaskDelete(NULL); // destroy this task 
}


/****************************************************
 * A SQWV signal from the RTC is generated, we
 * use this signal to handle second tasks and
 * on sampling make sure that we achieve
 * the appropriate <samplingrate> samples each second
 ****************************************************/
volatile bool sqwvIsHigh = true;
long lastSqwvTime = millis();
void IRAM_ATTR sqwvTriggered(void * instance) {
  // Interrupt is for LOW and HIGH, we keep changing this during interrupt
  if (sqwvIsHigh) {
		gpio_set_intr_type(rtc_int_pin, GPIO_INTR_LOW_LEVEL);
		sqwvIsHigh = false;

    // This is the low triggered interrupt we are talking about
    long now = millis();
    portENTER_CRITICAL_ISR(&criticalMsgMux);
    criticalMsgAvailable = true;
    testMillis2 = now-lastSqwvTime;
    portEXIT_CRITICAL_ISR(&criticalMsgMux);
    lastSqwvTime = now;
    if (state == SampleState::SAMPLE_LATCHED) {
      // We take one sample and enable the timing interrupt afterwards
      // this should allow the ISR e.g. at 4kHz to take 3999 samples during the next second
      timerAlarmDisable(timer);
      timerWrite(timer, 0);
      // Reset counter if not already done in ISR
      portENTER_CRITICAL_ISR(&counterMux);
      testSamples = counter;
      counter = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      latched_sample_ISR();
      // Enable timer
      timerAlarmEnable(timer);
    } else {
      portENTER_CRITICAL_ISR(&intterruptSamplesMux);
      testSamples = interruptSamples;
      interruptSamples = 0;
      portEXIT_CRITICAL_ISR(&intterruptSamplesMux);
    }
    
    // WE NEED to add artificial data here
    if (testSamples < streamConfig.samplingRate) {
      // TODO
      uint16_t missingSamples = streamConfig.samplingRate - testSamples;
      if (missingSamples < 5) {
        for (int i = 0; i < missingSamples; i++) {
          if (!ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes)) {
            // TODO: Make this out of interrupt
            portENTER_CRITICAL_ISR(&criticalMsgMux);
            criticalMsgAvailable = true;
            snprintf(&criticalMsg[0], MAX_MQTT_PUB_TOPIC_INFO, "Cannot fill ringbuffer");
            portEXIT_CRITICAL_ISR(&criticalMsgMux);
            break;
          }
        }
      } else {
        // This is a serious one
      } 
    // Somehow expected but treated in ISR itself
    } else if (testSamples > streamConfig.samplingRate) {
      testSamples = streamConfig.samplingRate;
    }
    // indicate to main that a second has passed and store time 
    portENTER_CRITICAL_ISR(&sqwMux);
    sqwCounter++;
    sqwCounter2++;
    portEXIT_CRITICAL_ISR(&sqwMux);
    
	} else {
		gpio_set_intr_type(rtc_int_pin, GPIO_INTR_HIGH_LEVEL);
		sqwvIsHigh = true;
	}
}


// void IRAM_ATTR sqwvTriggered2() {
//   long now = millis();
//   portENTER_CRITICAL_ISR(&criticalMsgMux);
//   criticalMsgAvailable = true;
//   testMillis2 = now-lastSqwvTime;
//   portEXIT_CRITICAL_ISR(&criticalMsgMux);
//   lastSqwvTime = now;
// }




/****************************************************
 * Stop Sampling will go into idle state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  if (rtc.connected) {
    disableSqwvInterrupt();
  }

  if (state == SampleState::SAMPLE) {
    ade9k.stopSampling();
  } else {
    turnInterrupt(false);
  }

  state = SampleState::IDLE;
  next_state = SampleState::IDLE;

  samplingDuration = millis() - startSamplingMillis;
  streamConfig.stopTs = myTime.timestamp();
  if (streamClient && streamClient.connected()) streamClient.stop();
  // Reset all variables
  streamConfig.countdown = 0;
  sampleCB();
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
  if (streamConfig.stream == StreamType::UDP) {
    chunkSize = min((int)chunkSize, 512);
  } else if (streamConfig.stream == StreamType::USB) {
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
void startSampling() {
  // Reset all variables
  state = SampleState::SAMPLE;
  sampleCB();
  xTaskCreatePinnedToCore(sample_timer_task,     /* Task function. */
        "sampleTask",       /* String with name of task. */
        4096*2,            /* Stack size in words. */
        NULL,             /* Parameter passed as input of the task */
        32,                /* Priority of the task. */
        &xHandle,            /* Task handle. */
        1); // Same task as the loop
        //  Network::ethernet? 1 : 0); // On wifi use core 0 on ethernet core 1 since wifi requires core 0 to be mostly idle

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  interruptSamples = 0;
  correctSamples = 0;
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
  // This will reset the sqwv pin
  firstSqwv = true;
  sqwCounter = 0;
  sqwCounter2 = 0;
  
  if (rtc.connected) {
    // rtc.enableInterrupt(1, sqwvTriggered); // TODO
    setupSqwvInterrupt();
  }
  
  freqCalcNow = micros();
  freqCalcStart = micros();
  startSamplingMillis = millis();
  streamConfig.startTs = myTime.timestamp();
  ade9k.startSampling(intSamplingRate);
}


/****************************************************
 * Start latched Sampling requires to start the interrupt
 * to calculate the chunkSize size depending on
 * the samplingrate currently set. Furthermore,
 * all buffer indices are reset to the default values.
 ****************************************************/
void startLatchedSampling(bool waitVoltage) {
  // Reset all variables
  state = SampleState::SAMPLE_LATCHED;
  sampleCB();
  xTaskCreatePinnedToCore(  latched_sample_timer_task,     /* Task function. */
    "Consumer",       /* String with name of task. */
    4096,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    32,                /* Priority of the task. */
    &xHandle,            /* Task handle. */
    1);

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  sqwCounter = 0;
  sqwCounter2 = 0;
  correctSamples = 0;
  TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
  // This will reset the sqwv pin
  firstSqwv = true;
  sqwCounter = 0;
  sqwCounter2 = 0;

  if (rtc.connected) {
    // rtc.enableInterrupt(1, sqwvTriggered); // TODO
    setupSqwvInterrupt();
  }
  // If we should wait for voltage to make positive zerocrossing
  if (waitVoltage) {
    float values[3] = {1.0};
    while(values[0] > 0) {
      ade9k.readVoltageRMS(&values[0]);
      delay(2);
      yield();
    }
    while(values[0] < 0) {
      ade9k.readVoltageRMS(&values[0]); 
      delay(2);
      yield();
    }
  }
  freqCalcNow = micros();
  freqCalcStart = micros();
  startSamplingMillis = millis();
  streamConfig.startTs = myTime.timestamp();
  turnInterrupt(true);
}

/****************************************************
 * Detach or attach interupt for latched sampling
 ****************************************************/
void turnInterrupt(bool on) {
  cli();//stop interrupts
  if (on) {
    // timer_init();
    // The timer runs at 80 MHZ, independent of cpu clk
    timerAlarmWrite(timer, TIMER_CYCLES_FAST, true);
    timerWrite(timer, 0);
    timerAlarmEnable(timer);
  } else {
    if (timer != NULL) timerAlarmDisable(timer);
    // timer = NULL;
  }
  sei();
}

/****************************************************
 * Init the timer of the esp32
 ****************************************************/
void timer_init() {
  // Timer base freq is 80Mhz
  timer = NULL;
  // Make a 1 Mhz clk here
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &latched_sample_ISR, true);
}


/****************************************************
 * Check streamserver connection, sicne connect is 
 * a blocking task, we make it nonblocking using 
 * a separate freerots task
 ****************************************************/
TaskHandle_t streamServerTaskHandle = NULL;
void checkStreamServer(void * pvParameters) {
  bool first = true;
  while (not updating) {
    if (!exStreamServer.connected()                         // If not already connected
        and state == SampleState::IDLE                      // and in idle mode
        and Network::connected and not Network::apMode      // Network is STA mode
        and strcmp(config.streamServer, NO_SERVER) != 0) {  // and server is set
      if (exStreamServer.connect(config.streamServer, STANDARD_TCP_STREAM_PORT, 5)) { // Connect with 2 seconds timeout
        // Timeout must be large enough for handshare, otherwise server says connected but esp not
        logger.log("Connected to StreamServer: %s", config.streamServer);
        onClientConnect(exStreamServer);
      } else {
        if (first) {
          logger.log(WARNING, "Cannot connect to StreamServer: %s", config.streamServer);
          first = false;
        }
      }
    } 
    vTaskDelay(STREAM_SERVER_UPDATE_INTERVAL);
  }
  vTaskDelete(streamServerTaskHandle); // destroy this task 
}

/****************************************************
 * Init an external server to which data is streamed 
 * automatically if it is there. Enable checker 
 ****************************************************/
void initStreamServer() {
  if (strcmp(config.streamServer, NO_SERVER) != 0 and !exStreamServer.connected()) {
    logger.log("Try to connect Stream Server: %s", config.streamServer);
    // exStreamServer.connect(config.streamServer, STANDARD_TCP_STREAM_PORT);
    // Handle reconnects
    if (streamServerTaskHandle == NULL) {
      xTaskCreate(
            checkStreamServer,   /* Function to implement the task */
            "streamServerTask", /* Name of the task */
            10000,      /* Stack size in words */
            NULL,       /* Task input parameter */
            1,          /* Priority of the task */
            &streamServerTaskHandle);  /* Task handle */
    }
  }
}

/****************************************************
 * Init the MDNs name from eeprom, only the number ist
 * stored in the eeprom, construct using prefix.
 ****************************************************/
void initMDNS() {
  char * name = config.netConf.name;
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

/****************************************************
 * Subscribe to the mqtt topics we want to listen
 * and build the publish topics
 ****************************************************/
void mqttSubscribe() {
  if (!mqtt.connected()) {
    logger.log(ERROR, "Sth wrong with mqtt Server");
    return;
  }

  // Build publish topics
  snprintf(&mqttTopicPubSwitch[0], MAX_MQTT_TOPIC_LEN, "%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  snprintf(&mqttTopicPubSwitch[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  snprintf(&mqttTopicPubSwitch[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SWITCH);
  snprintf(&mqttTopicPubSample[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SAMPLE);
  snprintf(&mqttTopicPubInfo[0], MAX_MQTT_TOPIC_LEN, "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.netConf.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_INFO);
}

/****************************************************
 * Nice formatted info str with all available infos 
 * of this device
 * NOTE: Make sure enough memory is allocated for str
 ****************************************************/
void printInfoString() {
  // Name and firmware
  logger.log("%s @ firmware: %s/%s", config.netConf.name, __DATE__, __TIME__);

  logger.log("Using %d bytes %s", ringBuffer.getSize(), ringBuffer.inPSRAM()?"PSRAM":"RAM");
 
  if (rtc.connected) {
    logger.log("RTC is connected %s", rtc.lost?"but has lost time":"and has valid time");
  } else {
    logger.log("No RTC");
  }
  logger.append("Known Networks: [");
  for (size_t i = 0; i < config.netConf.numAPs; i++) {
    logger.append("%s%s", config.netConf.SSIDs[i], i < config.netConf.numAPs-1?", ":"");
  }
  logger.flushAppended();
}

/****************************************************
 * Callback if sampling will be perfomed
 ****************************************************/
void sampleCB() {
  if (mqtt.connected()) mqtt.publish(mqttTopicPubSample, state==SampleState::IDLE ? MQTT_TOPIC_SWITCH_OFF: MQTT_TOPIC_SWITCH_ON);
  // Do not allow network changes on sampling
  Network::allowNetworkChange = state==SampleState::IDLE ? false : true;
}

/****************************************************
 * Display sampling information to logger
 ****************************************************/
void onSamplingInfo() {
  if (state != SampleState::IDLE) {
    unsigned long _totalSamples = totalSamples;
    Timestamp now = myTime.timestamp();
    uint32_t durationMs = (now.seconds - streamConfig.startTs.seconds)*1000;
    durationMs += (int(now.milliSeconds) - int(streamConfig.startTs.milliSeconds));
    float avgRate = _totalSamples/(durationMs/1000.0);
    uint32_t _goalSamples = float(durationMs/1000.0)*streamConfig.samplingRate;
    int samplesOff = int(_goalSamples - _totalSamples);
    float secondsOff = float(samplesOff)/float(streamConfig.samplingRate);
    logger.log(INFO, "After NTP Sync: avg rate %.3f Hz", avgRate);
    LogType type = LogType::INFO;
    if (abs(secondsOff) > 0.02) type = LogType::WARNING;
    logger.log(type, "Should be: %lu, is %lu samples", _goalSamples, _totalSamples);
    logger.log(type, "Offset of %.3f s, SecondInterrupts %i, SamplingDur %.3f", secondsOff, sqwCounter2, float(durationMs/1000.0));
    logger.log(INFO, "From %s", myTime.timestampStr(streamConfig.startTs));
    logger.log(INFO, "To %s",  myTime.timestampStr(now));

    #ifdef NTP_CORRECT_SAMPLINGRATE
    if (abs(secondsOff) >= CORRECT_SAMPLING_THRESHOLD) correctSampling(samplesOff);
    #endif
  }
}

void correctSampling(int samplesToCorrect) {
  int samplesCorrect = samplesToCorrect;
  // Cap it so that signal will not be distored too much
  if (samplesCorrect > MAX_CORRECT_SAMPLES) samplesCorrect = MAX_CORRECT_SAMPLES;
  else if (samplesCorrect < -MAX_CORRECT_SAMPLES) samplesCorrect = -MAX_CORRECT_SAMPLES;
  // samplesCorrect > 0 means we have too less samples
  // samplesCorrect < 0 means we have too much samples
  portENTER_CRITICAL(&correctionMux);
  correctSamples += samplesCorrect;
  portEXIT_CRITICAL(&correctionMux);
  logger.log(WARNING, "Correcting: %i samples", samplesCorrect);
}


/****************************************************
 * Callback when NTP syncs happened
 ****************************************************/
void ntpSynced(unsigned int confidence) {
  logger.log(INFO, "NTP synced with conf: %u", confidence);
  if (state != SampleState::IDLE) onSamplingInfo();
}


/****************************************************
 * Setup the OTA updates progress
 ****************************************************/
// To display only full percent updates
unsigned int oldPercent = 0;
void setupOTA() {
  // Same name as mdns
  ArduinoOTA.setHostname(config.netConf.name);
  ArduinoOTA.setPassword("energy"); 
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    updating = true;
    Network::allowNetworkChange = false;
    logger.log("Start updating");

    if (state != SampleState::IDLE) {
      logger.log(WARNING, "Stop Sampling, Update Requested");
      stopSampling();

      // // Write remaining chunks with tailr
      // if (outputWriter != NULL) outputWriter(true);
      // docSend["msg"] = F("Received stop command");
      // docSend["sample_duration"] = samplingDuration;
      // docSend["samples"] = totalSamples;
      // docSend["sent_samples"] = sentSamples;
      // docSend["start_ts"] = myTime.timestampStr(streamConfig.startTs);
      // docSend["stop_ts"] = myTime.timestampStr(streamConfig.stopTs);
      // docSend["ip"] = WiFi.localIP().toString();
      // docSend["avg_rate"] = totalSamples/(samplingDuration/1000.0);
      // docSend["cmd"] = CMD_STOP;

      // serializeJson(docSend, response);
      // response = LOG_PREFIX + response;
      // if (sendStream) sendStream->println(response);
    }

    // Disconnecting all connected clients
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i]) {
        // client[i].stop(); 
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }

    if (exStreamServer.connected()) exStreamServer.stop();
    if (streamClient.connected()) streamClient.stop();
    
    // Stopping all other tcp stuff
    streamServer.stop();
    streamServer.close();
    server.stop();
    server.close();
    mqtt.disconnect();
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress / (total / 100));
    if (percent != oldPercent) {
      Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
      oldPercent = percent;
    }
  });  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    // No matter what happended, simply restart
    ESP.restart();
  });
  // Enable OTA
  ArduinoOTA.begin();
}