/****************************************************
 * A request happended, handle it
 ****************************************************/
void handleEvent(Stream * getter) {
  if (!getter->available()) return;
  getter->readStringUntil('\n').toCharArray(command,COMMAND_MAX_SIZE);
  // Be backwards compatible to "?" command all the time
  if (command[0] == '?') {
    getter->println(F("Info:Setup done"));
    return;
  }
  #ifdef DEBUG_DEEP
  logger.log(INFO, command);
  #endif

  newGetter = getter;

  response = "";
  if (!parseCommand()) return;
  handleJSON();

  JsonObject object = docSend.as<JsonObject>();
  if (object.size()) {
    getter->flush();
    response = "";
    serializeJson(docSend, response);
    response = LOG_PREFIX + response;
    getter->println(response);
    // This will be too long for the logger
    logger.log(response.c_str());
  }
  response = "";
  command[0] = '\0';
}

/****************************************************
 * Decode JSON command from string to json dictionary
 ****************************************************/
bool parseCommand() {
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(docRcv, command);
  
  // Test if parsing succeeds.
  if (error) {
    // Remove all unallowed json characters to prevent error 
    uint32_t len = strlen(command);
    if (len > 10) len = 10;
    for (size_t i = 0; i < len; i++) {
      if (command[i] == '\r' || command[i] == '\n' || command[i] == '"' || command[i] == '}' || command[i] == '{') command[i] = '_';
    }
    logger.log(ERROR, "deserializeJson() failed: %.10s", &command[0]);
    return false;
  }
  //docSend.clear();
  JsonObject obj = docSend.to<JsonObject>();
  obj.clear();
  return true;
}

/****************************************************
 * Handle a command in form of a JSON dict
 ****************************************************/
void setBusyResponse() {
  if (sendClient != NULL) {
    response = "Device with IP: ";
    response += sendClient->localIP().toString();
    response += " currently sampling"; 
  } else {
    response = "Currently sampling";
  }
}

/****************************************************
 * Indicate that someone else is currently sampling 
 * and we cannot perform the requested action
 * This function just build the string msg for it
 ****************************************************/
void handleJSON() {
  // All commands look like the following:
  // {"cmd":"commandName", "payload":{<possible data>}}
  // e.g. mdns

  const char* cmd = docRcv["cmd"];
  JsonObject root = docRcv.as<JsonObject>();
  if (cmd == nullptr) {
    docSend["msg"] = F("JSON format error, syntax: {\"cmd\":<cmdName>, \"[payload\":{<data>}]}}");
    return;
  }

  /*********************** SAMPLING COMMAND ****************************/
  // e.g. {"cmd":"sample", "payload":{"type":"Serial", "rate":4000}}
  if(strcmp(cmd, CMD_SAMPLE) == 0) {
    if (state == SampleState::IDLE) {
      // For sampling we need type payload and rate payload
      const char* typeC = root["payload"]["type"];
      const char* measuresC = root["payload"]["measures"];
      int rate = docRcv["payload"]["rate"].as<int>();
      unsigned long ts = docRcv["payload"]["time"].as<unsigned long>();
      bool prefix = docRcv["payload"]["prefix"].as<bool>();
      JsonVariant prefixVariant = root["payload"]["prefix"];

      docSend["error"] = true;
      if (typeC == nullptr or rate == 0) {
        response = "Not a valid \"sample\" command";
        if (typeC == nullptr) response += ", \"type\" missing";
        if (rate == 0) response += ", \"rate\" missing";
        docSend["msg"] = response;
        return;
      }
 
      streamConfig.measurementBytes = 8;
      bool latchMode = false;

      if (measuresC == nullptr or strcmp(measuresC, MEASURE_VI) == 0) {
        streamConfig.measures = Measures::VI;
        streamConfig.measurementBytes = 24;
      } else if (strcmp(measuresC, MEASURE_VI_L1) == 0 
                 or strcmp(measuresC, MEASURE_VI_COMPATIBILITY) == 0) {
        streamConfig.measures = Measures::VI_L1;
      } else if (strcmp(measuresC, MEASURE_VI_L2) == 0) {
        streamConfig.measures = Measures::VI_L2;
      } else if (strcmp(measuresC, MEASURE_VI_L3) == 0) {
        streamConfig.measures = Measures::VI_L3;
      } else if (strcmp(measuresC, MEASURE_PQ) == 0) {
        streamConfig.measures = Measures::PQ;
        streamConfig.measurementBytes = 24;
        // PQ only possible in latchMode
        latchMode = true;
      } else if (strcmp(measuresC, MEASURE_VI_RMS) == 0) {
        streamConfig.measures = Measures::VI_RMS;
        streamConfig.measurementBytes = 24;
        // RMS only possible in latchMode
        latchMode = true;
      } else {
        response = "Unsupported measures";
        response += measuresC;
        docSend["msg"] = response;
        return;
      }
      streamConfig.numValues = streamConfig.measurementBytes/sizeof(float);
      
      // Only important for MQTT sampling_rate
      // Moved away due to som earduinojson bug
      // objSample.clear();
      // objSample["unit"] = unitToStr(streamConfig.measures);
      // JsonArray array = objSample["values"].to<JsonArray>();
      // for (int i = 0; i < streamConfig.numValues; i++) array.add(0.0f);

      if (not latchMode) {
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
      // latchmode: Max samplingrate is 50Hz
      } else {
        if (rate > 50) {
          response = "SamplingRate cannot be higher than 50Hz in latch mode: ";
          response += measuresC;
          response += " sampling";
          docSend["msg"] = response;
          return;
        }
      }

      streamConfig.prefix = true;
      // If we do not want a prefix, we have to disable this if not at extra port
      if (!prefixVariant.isNull()) {
        streamConfig.prefix = prefix;
      }
      outputWriter = &writeChunks;
      // e.g. {"cmd":"sample", "payload":{"type":"Serial", "rate":4000}}
      if (strcmp(typeC, "Serial") == 0) {
        streamConfig.stream = StreamType::USB;// e.g. {"cmd":"sample", "payload":{"type":"MQTT", "rate":4000}}
        sendStream = (Stream*)&Serial; 
      } else if (strcmp(typeC, "MQTT") == 0) {
        streamConfig.stream = StreamType::MQTT;
        outputWriter = &writeDataMQTT;
      // e.g. {"cmd":"sample", "payload":{"type":"TCP", "rate":4000}}
      } else if (strcmp(typeC, "TCP") == 0) {
        sendClient = (WiFiClient*)newGetter;
        sendStream = sendClient; 
        // sendClient = &client[0]; 
        streamConfig.stream = StreamType::TCP;
        streamConfig.port = STANDARD_TCP_SAMPLE_PORT;
        streamConfig.ip = sendClient->remoteIP();
      // e.g. {"cmd":"sample", "payload":{"type":"UDP", "rate":4000}}
      } else if (strcmp(typeC, "UDP") == 0) {
        streamConfig.stream = StreamType::UDP;
        int port = docRcv["payload"]["port"].as<int>();
        if (port > 80000 || port <= 0) {
          streamConfig.port = STANDARD_UDP_PORT;
          response = "Unsupported UDP port";
          response += port;
          docSend["msg"] = response;
          return;
        } else {
          streamConfig.port = port;
        }
        outputWriter = &writeChunksUDP;
        sendClient = (WiFiClient*)newGetter;
        sendStream = sendClient; 
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

      docSend["measures"] = measuresToStr(streamConfig.measures);
      docSend["chunksize"] = streamConfig.chunkSize;
      docSend["samplingrate"] = streamConfig.samplingRate;
      docSend["conn_type"] = typeC;
      docSend["measurements"] = streamConfig.numValues;
      docSend["prefix"] = streamConfig.prefix;
      docSend["cmd"] = CMD_SAMPLE;
      docSend["unit"] = unitToStr(streamConfig.measures);

      next_state = SampleState::SAMPLE;

      if (ts != 0) {
        response += F("Should sample at: ");
        response += myTime.timeStr(ts, 0);
        // Wait random time to start ntp update s.t. not all devices start ntp request at the same time
        int waitMillis = random(200);
        delay(waitMillis);
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

      if (latchMode) {
        startLatchedSampling(false);
      } else {
        startSampling();
      }
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** STOP COMMAND ****************************/
  // e.g. {"cmd":"stop"}
  else if (strcmp(cmd, CMD_STOP) == 0) {
    if (state == SampleState::IDLE) return;
    // State is reset in stopSampling
    stopSampling();
    // Write remaining chunks with tail
    outputWriter(true);
    JsonObject obj = docSend.to<JsonObject>();
    obj.clear();
    docSend["msg"] = F("Received stop command");
    docSend["sample_duration"] = samplingDuration;
    docSend["samples"] = totalSamples;
    docSend["sent_samples"] = sentSamples;
    docSend["ip"] = Network::localIP().toString();
    docSend["avg_rate"] = totalSamples/(samplingDuration/1000.0);
    docSend["cmd"] = CMD_STOP;
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":"restart"}
  else if (strcmp(cmd, CMD_RESTART) == 0) {
    ESP.restart();
  }

  /*********************** RESTART COMMAND ****************************/
  // e.g. {"cmd":"factoryReset"}
  else if (strcmp(cmd, CMD_RESET) == 0) {
    config.makeDefault();
    config.store();
    ESP.restart();
  }

  /*********************** INFO COMMAND ****************************/
  // e.g. {"cmd":"info"}
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
    docSend["mqtt_server"] = config.mqttServer;
    docSend["time_server"] = config.timeServer;
    docSend["sampling_rate"] = streamConfig.samplingRate;
    docSend["buffer_size"] = ringBuffer.getSize();
    docSend["psram"] = ringBuffer.inPSRAM();
    docSend["rtc"] = rtc.connected;
    docSend["state"] = state != SampleState::IDLE ? "busy" : "idle";
    String ssids = "[";
    for (int i = 0; i < config.numAPs; i++) {
      ssids += config.wifiSSIDs[i];
      if (i < config.numAPs-1) ssids += ", ";
    }
    ssids += "]";
    docSend["ssids"] = ssids;
  }

  /*********************** MDNS COMMAND ****************************/
  // e.g. {"cmd":"mdns", "payload":{"name":"newName"}}
  else if (strcmp(cmd, CMD_MDNS) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newName = docRcv["payload"]["name"];
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
  /*********************** MQTT Server COMMAND ****************************/
  // e.g. {"cmd":"mqttServer", "payload":{"server":"<ServerAddress>"}}
  else if (strcmp(cmd, CMD_MQTT_SERVER) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newServer = docRcv["payload"]["server"];
      if (newServer == nullptr) {
        docSend["msg"] = F("MQTTServer address required in payload with key server");
        return;
      }
      if (strlen(newServer) < MAX_IP_LEN) {
        config.setMQTTServerAddress((char * )newServer);
      } else {
        response = F("MQTTServer address too long, only string of size ");
        response += MAX_IP_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * address = config.mqttServer;
      response = F("Set MQTTServer address to: ");
      response += address;
      //docSend["msg"] = sprintf( %s", name);
      docSend["msg"] = response;
      docSend["mqtt_server"] = address;
      docSend["error"] = false;
      mqtt.init(config.mqttServer, config.name);
      mqtt.connect();

    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** Stream Server COMMAND ****************************/
  // e.g. {"cmd":"streamServer", "payload":{"server":"<ServerAddress>"}}
  else if (strcmp(cmd, CMD_STREAM_SERVER) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newServer = docRcv["payload"]["server"];
      if (newServer == nullptr) {
        docSend["msg"] = F("StreamServer address required in payload with key server");
        return;
      }
      if (strlen(newServer) < MAX_IP_LEN) {
        config.setStreamServerAddress((char * )newServer);
      } else {
        response = F("StreamServer address too long, only string of size ");
        response += MAX_IP_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * address = config.streamServer;
      response = F("Set StreamServer address to: ");
      response += address;
      //docSend["msg"] = sprintf( %s", name);
      docSend["msg"] = response;
      docSend["stream_server"] = address;
      docSend["error"] = false;
      initStreamServer();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** Time Server COMMAND ****************************/
  // e.g. {"cmd":"streamServer", "payload":{"server":"<ServerAddress>"}}
  else if (strcmp(cmd, CMD_TIME_SERVER) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newServer = docRcv["payload"]["server"];
      if (newServer == nullptr) {
        docSend["msg"] = F("StreamServer address required in payload with key server");
        return;
      }
      if (strlen(newServer) < MAX_DNS_LEN) {
        config.setTimeServerAddress((char * )newServer);
      } else {
        response = F("TimeServer address too long, only string of size ");
        response += MAX_DNS_LEN;
        response += F(" allowed");
        docSend["msg"] = response;
        return;
      }
      char * address = config.timeServer;
      response = F("Set TimeServer address to: ");
      response += address;
      //docSend["msg"] = sprintf( %s", name);
      docSend["msg"] = response;
      docSend["time_server"] = address;
      docSend["error"] = false;
      myTime.updateNTPTime(true);
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
  /*********************** ADD WIFI COMMAND ****************************/
  // e.g. {"cmd":"addWifi", "payload":{"ssid":"ssidName","pwd":"pwdName"}}
  else if (strcmp(cmd, CMD_ADD_WIFI) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newSSID = docRcv["payload"]["ssid"];
      const char* newPWD = docRcv["payload"]["pwd"];
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
  // e.g. {"cmd":"delWifi", "payload":{"ssid":"ssidName"}}
  else if (strcmp(cmd, CMD_REMOVE_WIFI) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = true;
      const char* newSSID = docRcv["payload"]["ssid"];
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
  // e.g. {"cmd":"ntp"}
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
  // e.g. {"cmd":"clearLog"}
  else if (strcmp(cmd, CMD_CLEAR_LOG) == 0) {
    if (state == SampleState::IDLE) {
      docSend["error"] = false;
      spiffsLog.clear();
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }

  /*********************** Get Log COMMAND ****************************/
  // e.g. {"cmd":"getLog"}
  else if (strcmp(cmd, CMD_GET_LOG) == 0) {
    if (state == SampleState::IDLE) {
      spiffsLog.flush();
      docSend["error"] = false;
      bool hasRow = spiffsLog.nextRow(&command[0]);
      newGetter->printf("%s{\"cmd\":\"log\",\"msg\":\"", &LOG_PREFIX[0]);
      newGetter->printf("*** LOGFile *** //n");
      while(hasRow) {
        newGetter->printf("%s//n", &command[0]);
        hasRow = spiffsLog.nextRow(&command[0]);
      }
      newGetter->println("*** LOGFile *** \"}");
    } else {
      setBusyResponse();
      docSend["msg"] = response;
      docSend["state"] = "busy";
    }
  }
}

/****************************************************
 * A mqtt msg was received for a specific 
 * topic we subscribed to. See mqttSubscribe function. 
 ****************************************************/
void mqttCallback(char* topic, byte* message, unsigned int length) {
  memcpy(&command[0], message, length);
  command[length] = '\0';
  logger.log("MQTT msg on topic: %s: %s", topic, command);

  // Search for last topic separator
  size_t topicLen = strlen(topic);
  // On not found, this will start from the beginning of the topic string
  int lastSep = -1;
  for (size_t i = 0; i < topicLen; i++) {
    if (topic[i] == '\0') break;
    if (topic[i] == MQTT_TOPIC_SEPARATOR) lastSep = i;
  }
  char * topicEnd = &topic[lastSep+1];


  if(strcmp(topicEnd, MQTT_TOPIC_CMD) == 0) {
    // message was already copied to command array
    parseCommand();
    handleJSON();
    JsonObject object = docSend.as<JsonObject>();
    if (object.size()) {
      response = "";
      serializeJson(docSend, response);
      // This might be too long for the logger
      logger.log(response.c_str());
      mqtt.publish(mqttTopicPubInfo, response.c_str());
    }
  }  
  // state request (like info)
  else if(strcmp(topicEnd, MQTT_TOPIC_STATE) == 0) {
    JsonObject obj = docSend.to<JsonObject>();
    obj.clear();
    docSend["sampleState"] = state == SampleState::IDLE ? "idle" : "sampling";
    docSend["ts"] = myTime.timeStr();
    response = "";
    serializeJson(docSend, response);
    logger.log(response.c_str());
    mqtt.publish(mqttTopicPubInfo, response.c_str());
  } 
  // Request a sample
  // TODO: Implement
  else if(strcmp(topicEnd, MQTT_TOPIC_SAMPLE) == 0) {
    logger.log("MQTT wants sample");
    float value[4] = {-1.0};
    char unit[4] = {'\0'};
    if(strcmp(command, "v") == 0) {
      ade9k.readVoltageRMS(&value[0]);
      sprintf(unit, "V");
    }
    else if(strcmp(command, "i") == 0) {
      ade9k.readCurrentRMS(&value[0]);
      sprintf(unit, "mA");
    }
    else if(strcmp(command, "q") == 0) {
      ade9k.readReactivePower(&value[0]);
      sprintf(unit, "var");
    }
    else if(strcmp(command, "s") == 0) {
      ade9k.readApparentPower(&value[0]);
      sprintf(unit, "VA");
    // default is active power
    } else {
      ade9k.readActivePower(&value[0]);
      sprintf(unit, "W");
    }
    JsonObject obj = docSend.to<JsonObject>();
    obj.clear();

    char valuestr[40];
    snprintf(valuestr, 40, "%.2f,%.2f,%.2f", value[0], value[1], value[2]);
    docSend["value"] = valuestr;
    docSend["unit"] = unit;
    docSend["ts"] = myTime.timeStr();
    response = "";
    serializeJson(docSend, response);
    logger.log(response.c_str());
    mqtt.publish(mqttTopicPubInfo, response.c_str());
  }
  // response = "";
  // command[0] = '\0';
}
