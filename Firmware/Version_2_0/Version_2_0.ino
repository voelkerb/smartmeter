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
#include "src/mqtt/mqtt.h"

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
ADE9000 ade9k(ADE_RESET_PIN, ADE_DREADY_PIN, ADE_PM1_PIN, ADE_SPI_BUS);

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
enum class SampleState{IDLE, SAMPLE, SAMPLE_LATCHED};

SampleState state = SampleState::IDLE;
SampleState next_state = SampleState::IDLE;

// Define it before StreamType enum redefines it
MQTT mqtt;

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or RMS
enum class Measures{VI, VI_L1, VI_L2, VI_L3, VI_RMS, PQ};
enum class StreamType{USB, TCP, UDP, TCP_RAW, MQTT};

struct StreamConfig {
  bool prefix = false;                      // Send data with "data:" prefix
  Measures measures = Measures::VI;           // The measures to send
  uint8_t measurementBytes = 24;             // Number of bytes for each measurement entry
  unsigned int samplingRate = DEFAULT_SR;   // The samplingrate
  StreamType stream = StreamType::USB;                  // Channel over which to send
  unsigned int countdown = 0;               // Start at specific time or immidiately
  uint32_t chunkSize = 0;                   // Chunksize of one packet sent
  IPAddress ip;                             // Ip address of data sink
  uint16_t port = STANDARD_TCP_SAMPLE_PORT; // Port of data sink
  char unit[20] = {'\0'};
  size_t numValues = 24/sizeof(float);
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
// Init getter to point to sth, might not work otherwise
Stream * newGetter = (Stream*)&Serial;
// tcp stuff is send over this client 
Stream * sendStream = (Stream*)&Serial;
WiFiClient * sendClient;
// UDP used for streaming
WiFiUDP udpClient;

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

// Current CPU speed
unsigned int coreFreq = 0;

void (*outputWriter)(bool) = &writeChunks;

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
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

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
  
  config.init();
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

  Network::init(&config, onNetworkConnect, onNetworkDisconnect, true);
  Network::initPHY(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  setupOTA();

  response.reserve(2*COMMAND_MAX_SIZE);

  // Set mqtt and callbacks
  mqtt.init(config.mqttServer, config.name);
  mqtt.onConnect = &onMQTTConnect;
  mqtt.onDisconnect = &onMQTTDisconnect;
  mqtt.onMessage = &mqttCallback;

  timer_init();

  setInfoString(&command[0]);
  logger.log(&command[0]);

  logger.log(ALL, "Setup done");

  lifenessUpdate = millis();
  mdnsUpdate = millis();
  tcpUpdate = millis();
  rtcUpdate = millis();
}

/************************ Loop *************************/
void loop() {
  if (updating) return;

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
      esp_wifi_set_ps(WIFI_PS_NONE);
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

  // Watchdog
  yield();
}

/****************************************************
 * What todo independent of sampling state
 ****************************************************/
void onIdleOrSampling() {
  // MQTT loop
  mqtt.update();

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

/****************************************************
 * Things todo regularly if we are not sampling
 ****************************************************/
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
      streamConfig.stream = StreamType::TCP_RAW;
      streamConfig.prefix = false;
      streamConfig.samplingRate = DEFAULT_SR;
      streamConfig.measures = Measures::VI;
      streamConfig.ip = streamClient.remoteIP();
      streamConfig.port = streamClient.remoteIP();
      sendClient = (WiFiClient*)&streamClient;
      sendStream = sendClient;
      startSampling();
    }
  }
}

/****************************************************
 * What todo only on sampling (send data, etc)
 ****************************************************/
void onSampling() {
  // ______________ Send data to sink ________________
  outputWriter(false);

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
  // Disconnect of raw stream means stop
  } else if (streamConfig.stream == StreamType::TCP_RAW) {
    // Check for intended connection loss
    if (!sendClient->connected()) {
      logger.log(INFO, "TCP Stream disconnected while streaming");
      stopSampling();
    }
  } else if (streamConfig.stream == StreamType::MQTT) {
    if (!mqtt.connected) {
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
  char * ttime = myTime.timeStr(true);
  return ttime;
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
  logger.log(ALL, "Network Connected");
  logger.log(ALL, "IP: %s", Network::localIP().toString().c_str());
  // The stuff todo if we have a network connection (and hopefully internet as well)
  if (Network::connected and not Network::apMode) {
    myTime.updateNTPTime(true);
  }

  // Reinit mdns
  initMDNS();

  // Connect mqtt
  if (!mqtt.connect()) logger.log(ERROR, "Cannot connect to MQTT Server %s", mqtt.ip);

  // Start the TCP server
  server.begin();
  streamServer.begin();

  // Reset lifeness and MDNS update
  lifenessUpdate = millis();
  mdnsUpdate = millis();
}

/****************************************************
 * If ESP disconnected from network
 ****************************************************/
void onNetworkDisconnect() {
  logger.log(ERROR, "Network Disconnected");

  if (mqtt.connected) mqtt.disconnect();
  if (state != SampleState::IDLE) {
    logger.log(ERROR, "Stop sampling (Network disconnect)");
    stopSampling();
  }
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
 * Write all remaining chunks of data over channel
 * depending on data sink
 ****************************************************/
void writeChunks(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    writeData(*sendStream, streamConfig.chunkSize);
  }
  if (tail) {
    writeData(*sendStream, ringBuffer.available());
  }
}

void writeChunksUDP(bool tail) {
  while(ringBuffer.available() > streamConfig.chunkSize) {
    udpClient.beginPacket(streamConfig.ip, streamConfig.port);
    writeData(udpClient, streamConfig.chunkSize);
    udpClient.endPacket();
  }
  if (tail) {
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


  while(ringBuffer.available() >= streamConfig.measurementBytes) {
    ringBuffer.read((uint8_t*)&values[0], streamConfig.measurementBytes);
    for (int i = 0; i < streamConfig.numValues; i++) docSample["values"][i] = values[i];
    // JsonArray array = docSend["values"].to<JsonArray>();
    // for (int i = 0; i < streamConfig.numValues; i++) array.add(value[i]);
    // docSend["unit"] = streamConfig.unit;
    docSample["ts"] = myTime.timeStr();
    response = "";
    serializeJson(docSample, response);
    // logger.log(response.c_str());
    mqtt.publish(mqttTopicPubInfo, response.c_str());
    sentSamples++;
  }
}

/****************************************************
 * Write one chunk of data to sink.
 ****************************************************/
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
  if (streamConfig.stream == StreamType::USB) getter.write("\r\n"); 
  if (sent > start) sentSamples += (sent-start)/streamConfig.measurementBytes;
  // sentSamples += size/streamConfig.measurementBytes;
}



/****************************************************
 * Sampling interrupt, A new ADC reading is available
 * Sample is read and put into queue
 * NOTE: Function must be small and quick
 ****************************************************/
xQueueHandle xQueue = xQueueCreate(2000, sizeof(CurrentADC)); ;
bool volatile IRAM_timeout = false;
CurrentADC isrData;
float data[7] = {0.0};
volatile int skipper = 0;
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

/****************************************************
 * RTOS task for handling a new sample
 ****************************************************/
bool QueueTimeout = false;
float values2[7] = {0.0};
size_t sendstart = 0;
uint16_t myChunkSize = (512/24)*24;
long bufftimer = millis();
void sample_timer_task( void * parameter) {
  vTaskSuspend( NULL );  // ISR wakes up the task

  // We can only get all data from adc, if we want a single phase, skip the rest
  uint8_t offset = 0;
  if (streamConfig.measures == Measures::VI) offset = 0;
  else if (streamConfig.measures == Measures::VI_L1) offset = 0;
  else if (streamConfig.measures == Measures::VI_L2) offset = 2;
  else if (streamConfig.measures == Measures::VI_L3) offset = 4;

  CurrentADC adc;
  while(state != SampleState::IDLE){
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
      if (streamConfig.stream == StreamType::USB) {
        Serial.write((uint8_t*)&values2[offset], streamConfig.measurementBytes);
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

        bool success = ringBuffer.write((uint8_t*)&(values2[offset]), streamConfig.measurementBytes);
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


/****************************************************
 * Sampling interrupt, must be small and quick
 ****************************************************/
static SemaphoreHandle_t timer_sem;
volatile uint32_t mytime = micros();
void IRAM_ATTR latched_sample_ISR() {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
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

  float data[7] = { 0.0 };
  int test = 0;

  while (state != SampleState::IDLE) {
    xSemaphoreTake(timer_sem, portMAX_DELAY);
    
    // Vaiables for frequency count
    counter++;
    totalSamples++;
    if (counter >= streamConfig.samplingRate) {
      freqCalcNow = micros();
      freq = freqCalcNow-freqCalcStart;
      freqCalcStart = freqCalcNow;
      counter = 0;
      if (rtc.connected) timerAlarmDisable(timer);
    }

    if (streamConfig.measures == Measures::VI_RMS) {
      ade9k.readVoltageRMS(&data[0]);
      ade9k.readCurrentRMS(&data[3]);
    } else if (streamConfig.measures == Measures::PQ) {
      ade9k.readActivePower(&data[0]);
      ade9k.readReactivePower(&data[3]);
    }
    if (!ringBuffer.write((uint8_t*)&data[0], streamConfig.measurementBytes)) {
      logger.log(ERROR, "Cannot fill ringbuffer");
    }

  }
  vTaskDelete( NULL );
}

/****************************************************
 * A SQWV signal from the RTC is generated, we
 * use this signal to handle second tasks and
 * on sampling make sure that we achieve
 * the appropriate <samplingrate> samples each second
 ****************************************************/
void IRAM_ATTR sqwvTriggered() {
  // // Disable timer if not already done in ISR
  // timerAlarmDisable(timer);
  // Reset counter if not already done in ISR
  // testSamples = counter;
  
  if (state == SampleState::SAMPLE_LATCHED) {
    // We take one sample and enable the timing interrupt afterwards
    // this should allow the ISR e.g. at 4kHz to take 3999 samples during the next second
    timerWrite(timer, 0);
    latched_sample_ISR();
    // Enable timer
    timerAlarmEnable(timer);
  } else {
    portENTER_CRITICAL_ISR(&intterruptSamplesMux);
    interruptSamples = 0;
    portEXIT_CRITICAL(&intterruptSamplesMux);
  }
  // indicate to main that a second has passed and store time 
  portENTER_CRITICAL_ISR(&sqwMux);
  sqwCounter++;
  portEXIT_CRITICAL(&sqwMux);
  
  testMillis2 = millis();
}

/****************************************************
 * Setup the OTA updates progress
 ****************************************************/
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
    Network::allowNetworkChange = false;
    updating = true;
    logger.log("Start updating");


    for (size_t i = 0; i < MAX_CLIENTS; i++) {
      if (clientConnected[i]) {
        // client[i].stop(); 
        onClientDisconnect(client[i], i);
        clientConnected[i] = false;
      }
    }

    if (streamClient.connected()) streamClient.stop();
    
    // Disconnect all connected clients
    streamServer.stop();
    streamServer.close();
    server.stop();
    server.close();
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


/****************************************************
 * Stop Sampling will go into idle state, stop the
 * interrupt and if any stream client is connected,
 * will send EOF to the client
 ****************************************************/
void stopSampling() {
  if (rtc.connected) rtc.disableInterrupt(); // TODO

  if (state == SampleState::SAMPLE) {
    ade9k.stopSampling();
  } else {
    turnInterrupt(false);
  }

  state = SampleState::IDLE;
  next_state = SampleState::IDLE;

  samplingDuration = millis() - startSamplingMillis;
  if (streamClient && streamClient.connected()) streamClient.stop();
  // Reset all variables
  streamConfig.countdown = 0;
  lifenessUpdate = millis();
  mdnsUpdate = millis();
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
inline void startSampling() {
  // Reset all variables
  state = SampleState::SAMPLE;
  sampleCB();
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
    10,                /* Priority of the task. */
    &xHandle,            /* Task handle. */
    1);

  counter = 0;
  sentSamples = 0;
  totalSamples = 0;
  TIMER_CYCLES_FAST = (1000000) / streamConfig.samplingRate; // Cycles between HW timer inerrupts
  calcChunkSize();
  ringBuffer.reset();
  samplingDuration = 0;
  packetNumber = 0;
  // This will reset the sqwv pin
  firstSqwv = true;
  sqwCounter = 0;
  if (rtc.connected) rtc.enableInterrupt(1, sqwvTriggered);
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
  freqCalcNow = millis();
  freqCalcStart = millis();
  startSamplingMillis = millis();
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

/****************************************************
 * Subscribe to the mqtt topics we want to listen
 * and build the publish topics
 ****************************************************/
void mqttSubscribe() {
  if (!mqtt.connected) {
    logger.log(ERROR, "Sth wrong with mqtt Server");
    return;
  }

  // Build publish topics
  sprintf(&mqttTopicPubSwitch[0], "%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR);

  response = mqttTopicPubSwitch;
  response += "+";
  logger.log("Subscribing to: %s", response.c_str());
  mqtt.subscribe(response.c_str());
  
  sprintf(&mqttTopicPubSwitch[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SWITCH);
  sprintf(&mqttTopicPubSample[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_SAMPLE);
  sprintf(&mqttTopicPubInfo[0], "%s%c%s%c%s%c%s", MQTT_TOPIC_BASE, MQTT_TOPIC_SEPARATOR, config.name, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_STATE, MQTT_TOPIC_SEPARATOR, MQTT_TOPIC_INFO);
}

/****************************************************
 * Nice formatted info str with all available infos 
 * of this device
 * NOTE: Make sure enough memory is allocated for str
 ****************************************************/
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
  idx += sprintf(&str[idx], "\nMQTT Server: %s", config.mqttServer);

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

/****************************************************
 * Callback if sampling will be perfomed
 ****************************************************/
void sampleCB() {
  if (mqtt.connected) mqtt.publish(mqttTopicPubSample, state==SampleState::IDLE ? MQTT_TOPIC_SWITCH_OFF: MQTT_TOPIC_SWITCH_ON);
  // Do not allow network changes on sampling
  Network::allowNetworkChange = state==SampleState::IDLE ? false : true;
}