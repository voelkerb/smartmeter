[SmartMeter]: (https://github.com/voelkerb/SmartMeter)

# How to interface with a SmartMeter

## The Basics
The [SmartMeter] has to be connected to power. \
At first boot, the [SmartMeter] will open a WiFi-Network called "smartmeterX".\
It is not password protected, so you can simply connect to it. 
Open a TCP connection to IP ```smartmeterX.local``` (or ```192.168.4.1```) at port ```54321```.
This will show you basic information of the [SmartMeter] as a JSON encoded messages.
```bash
Info:{"cmd":"info","type":"SmartMeter","version":"2.2","compiled":"Mar_2_2021_09:57:58","sys_time":"03/02/2021 10:37:07.662","name":"smartmeterX","ip":"192.168.4.1","mqtt_server":"-","stream_server":"-","time_server":"time.google.com","sampling_rate":4000,"buffer_size":3670016,"psram":true,"rtc":false,"calV":1,"calI":1,"state":"idle","relay":1,"calibration":[1,1],"ssids":"[energywifi]","ssid":"-","rssi":-71,"bssid":"-"}
```
You will further notice that a lifeness log messages is sent each second.
We will further see how to use commands to interface with a SmartMeter.

**Each command sent to the [SmartMeter] over USB or a TCP channel requires JSON encoding and is answered by a JSON encoded answer message preceded by a the text "Info:". If an error occurred while processing the commmand, the error key is set true in the returned JSON.**

**Each command sent to the [PowerMeter] over USB or a TCP channel requires JSON encoding as ```{"cmd":<cmdName>,[optional]}``` and is answered by a JSON encoded answer message preceded by a the text "Info:". If an error occurred while processing the commmand, the error key is set true in the returned JSON.**

## Overview
| CMD        | Description         
| ------------- |-------------|
| ["info"](#info)                                | Get basic information                        |
| ["mdns"](#name)                                | Set new mDNS and general [PowerMeter] name   |
| ["addWifi"](#wifi)                             | Add WiFi AP                                  |
| ["delWifi"](#wifi)                             | Remove WiFi AP                               |
| ["calibration"](#calibration)                  | Set calibration parameter                    |
| ["restart"](#restart)                          | Restart [PowerMeter]                         |
| ["dailyRestart"](#restart)                     | Schedule reset every day                     |
| ["factoryReset"](#reset)                       | Reset to standard values                     |
| ["basicReset"](#reset)                         | Reset everything but the name                |
| ["resetEnergy"](#reset-energy)                 | Reset accumulated energy                     |
| ["sample"](#using-a-sampling-command)          | Start receiving high frequency data          |
| ["reqSamples"](#using-a-sampling-command)      | Get certain amount of high frequency samples |
| ["cts"](#pause-streaming)                      | Allow/forbid to stream samples               |
| ["stop"](#stop-streaming)                      | Stop receiving data                          |
| ["log"](#log-level)                            | Change log level                             |
| ["clearLog"](#log)                             | Clear logs                                   |
| ["getLog"](#log)                               | Returns all warning and error log messages   |
| ["mqttServer"](#mqtt)                          | Set MQTT server                              |
| ["streamServer"](#using-a-stream-server)       | Set stream server                            |
| ["ntp"](#time-synchronization)                 | Trigger NTP sync                             |
| ["timeServer"](#time-synchronization)          | Set time server for NTP                      |

## Info
```{"cmd":"info"}```\
The info command will print information about the [SmartMeter]. As this is the most often required command, it is automatically sent to each new connected TCP client. 

## Log Level
```{"cmd":"log","level":<logLevel>}```\
Choose between the following log levels: ```all, debug, info, warning, error```\
Each new connection is initialized with log level ```info```.
```bash
Info:{"error":false,"msg":"Log Level set to: warning"}
```

## Name
```{"cmd":"mdns","payload":{"name":"<newName>"}}```\
This command should be used to give the [SmartMeter] a new unique name.
It will be used to announce presence over MDNS and if no known network is found, an AP is opened using this name.

```bash
Info:[I]03/02 10:42:24: MDNS Name: smartmeterX
Info:{"error":false,"msg":"Set MDNS name to: smartmeterX","mdns_name":"smartmeterX"}
```

## Restart
```{"cmd":"restart"}```\
Restarts the [SmartMeter]. The TCP connection will be lost.

## Reset
```{"cmd":"factoryReset"}```\
Resets everything to defaults. The default values are:
- name: _smartmeterX_
- wifi aps: _energywifi_ -> (pwd: _silkykayak943_)
- timeserver: _time.google.com_
- streamserver: -
- mqtt broker: -
- _energy<LX><U/I>_ = 1.0
- _energy<LX>_ = 0
- _resetHour_ = _resetMinute_ = -1

If you want to reset everything except the name, use: ```{"cmd":"basicReset"}```

## Log
```{"cmd":"getLog"}```\
The [SmartMeter] has some decent logging capabilities. Despite logging over a connected TCP connection and Serial. _Warning_ and _Error_ messages are logged to internal flash memory.
The _getLog_ command will dump the all logs in the flash and contains two JSON response messages.
```bash
Info:{"cmd":"log","msg":"*** LOGFile *** [E]03/02 11:26:20: Cannot connect to MQTT Server -//n*** LOGFile *** "}
Info:{"error":false}
```
All newlines are replaces by "_//n_".\
To clear all log messages, use ```{"cmd":"clearLog"}```.

## Calibration
```{"cmd":"calibration","cal":[<calV_l1>,<calI_l1>,<calV_l2>,<calI_l2>,<calV_l3>,<calI_l3>]```\
Calibrate the [SmartMeter].
```<calV_LX>``` and ```<calI_LX>``` are floating point values with the default value 1.0.
Every measured voltage and current value is multiplied by the calibration factor. Power and Energy are multiplied by ```<calV_LX>```*```<calI_LX>```.

## Reset Energy
```{"cmd":"resetEnergy"}```\
Simply resets the accumulated energy stored in flash. 

## Time synchronization
```{"cmd":"ntp"}```\
Will perform an NTP time synchronization.
```
Info:[I]03/02 11:28:14: NTP synced with conf: 11
Info:{"msg":"Time synced","error":false,"current_time":"03/02/2021 11:27:40.182"}
```
To set a new NTP timeserver, use the command: ```{"cmd":"timeServer", "payload":{"server":"<serverAddress>"}}```. The _serverAddress_ can be also be its DNS name e.g. time.google.com
```bash
Info:[I]03/02 11:28:14: NTP synced with conf: 11
Info:{"error":false,"msg":"Set TimeServer address to: time.google.com","time_server":"time.google.com"}
```

## WiFi
Add wifi AP: ```{"cmd":"addWifi", "payload":{"ssid":"<ssidName>","pwd":"<pwdName>"}}```
```bash
Info:{"error":false,"ssid":"Test","pwd":"TestPwd","msg":"New Ap, SSID: Test, PW: TestPwd","ssids":"[energywifi, Test]"}
```
Remove a wifi AP: ```{"cmd":"delWifi", "payload":{"ssid":"<ssidName>"}}```
```bash
Info:{"error":false,"msg":"Removed SSID: Test","ssids":"[energywifi]"}
```

## MQTT
Once connected to an MQTT server, the SmartMeter will publish the current power consumption each 5 seconds and each state change of the relay. 
To set the MQTT server, use the command: ```{"cmd":"mqttServer", "payload":{"server":"<ServerAddress>"}}```
Currently, only the MQTT standard server port ```1883``` is supported. 
```bash
Info:[I]03/02 11:38:42: MQTT connected to 192.168.0.13
Info:[I]03/02 11:38:42: Subscribing to: SmartMeter/+
Info:[I]03/02 11:38:42: Subscribing to: SmartMeter/smartmeterX/+
Info:{"error":false,"msg":"Set MQTTServer address to: 192.168.0.13","mqtt_server":"192.168.0.13"}
```

NOTE: In sampling mode, MQTT is disabled for performance reason.


### Receiving MQTT messages
Each 5 seconds, information about the power consumption is sent.
It contains the sum of the _active power_ in _Watt_ and _active energy_ in _kWh_ of all grid lines, the mean of the _RMS voltage_ in _V_ of all grid lines, the sum of the _RMS current in _A_ of all grid lines and the current timestamp.
You can also enable 
```bash
powermeter/smartmeter001/state/sample {"power":1116.89,"current":4.82,"volt":236.56,"ts":"1614681575"}
```

### Sending MQTT messages
Mqtt can also be used to send any command. Special topics are used to switch the relay or to get basic electricity related measurements, but any of the available commands can be sent.
* Sample a single value using topic ```powermeter/<name>/sample``` and message one of ```v,i,p,q,s``` (_RMS voltage_,_RMS Current_, _active_, _reactive_, or _apparent power_ ). On no, or any other message, the _active power_ is sent.
  ```bash
  mosquitto_pub -h 192.168.0.13 -t 'powermeter/smartmeter001/sample' -m 'v'
  ````
  The response contains a JSON dictionary with the key _value_, _unit_ and _ts_, example:
  ```bash
  powermeter/smartmeter001/state/info {"value":"228.43,228.48,228.33","unit":"V","ts":"03/08/2021 11:42:34.848"}
  ```
  The individual values represent the values for the indivudal grid lines (L1, L2, L3)
* You can also send any command, which can be sent over a bare TCP connection using topic ```powermeter/<name>/cmd```
  ```bash
  mosquitto_pub -h 192.168.0.13 -t 'powermeter/smartmeter001/cmd' -m '{"cmd":"info"}'
  ````
* If you have multiple SmartMeters and want to send the message to all at the same time (e.g. to change some global settings such as the MQTT broker), you can simple ditch the specific SmartMeter name and all will answer. 
  ```bash
  powermeter/sample p
  powermeter/smartmeter001/state/sample {"value":"228.43,228.48,228.33","unit":"V","ts":"03/08/2021 11:42:34.848"}
  powermeter/SmartMeter002/state/sample {"value":"228.44,228.45,229.10","unit":"V","ts":"03/08/2021 11:42:34.947"}
  ...
  ```

## Getting High Frequency Data
Finally, to get some high frequency data out of the SmartMeters beyond whats possible using [mqtt](#mqtt), you have multiple possibilities. 

### Using FFmpeg
Using ffmpeg is by far the most simple way to store high frequency data. Thus, the sampling rate remains at 4kHz and only current and voltage measurements are streamed.
On your host computer, simply install [ffmpeg](https://ffmpeg.org) and run the following:
```bash
ffmpeg -nostdin -hide_banner -fflags +nobuffer+flush_packets -f f32le -ar 9000.0 -guess_layout_max 0 -ac 2.0 -flush_packets 1 -thread_queue_size 512 -analyzeduration 0 -i tcp://<SmartMeterIP>:54322 -c:a wavpack -frame_size 8000 -metadata:s:a:0 CHANNELS=2 -metadata:s:a:0 CHANNEL_TAGS="v_l1,i_l1,v_l2,i_l2,v_l3,i_l3" -metadata:s:a:0 title="<smartmeterName>" -map 0 -y output.mkv 
```
You can directly use the MDNS name for ```<smartmeterIP>```. Some of the settings in call such as the metadata are of course optional. 

### Using a stream server
At your future data sink (which simply might be your computer), host a TCP server at port ```54322```.
Find you IP address and set it as the target stream server for each SmartMeter using the command ```{"cmd":"streamServer", "payload":{"server":"<YourIp>"}}```.
The [SmartMeter] will check if the stream server is available each 30s and automatically connects to it.
For the data format, see [Data Format](#data-format).

### Using a sampling command
This is the most complicated command of the SmartMeter. It has multiple and potentially optional parameter which will be explained in the following. 
```bash
{"cmd":"sample", "payload":{"type":"<type>", "measures":"<measures>", "rate":<rate>, "prefix":<prefix>, "port":<port>,"time":<time>,"flowCtr":<flowCtr>,"slot":[<slot>,<slots>],"ntpConf":<ntpConf>}}
```
Example:
The response for the command ```{"cmd":"stop","payload":{"type":"TCP","rate":1}}``` is:
```bash
Info:{"error":false,"measures":"v,i","chunksize":8,"samplingrate":1,"conn_type":"TCP","measurements":2,"prefix":true,"flowCtr":false,"cmd":"sample","unit":"V,mA","startTs":"1614697441.119"}
```

Parameters:
* ```<type>``` + optional: ```<port>```: The Type of the data stream.
  * "Serial": sampled data is sent over USB Serial connection
  * "TCP": sampled data is sent over TCP connection (the one you opened to it). 
  * "UDP": sampled data is sent over a UDP connection. You need to specify the UDP Port with ```<port>```.
  * "MQTT": sampled data is sent over MQTT (broker must be set, see [MQTT](#mqtt)). 
  * "FFMPEG": connect with an ffmpeg call to the stream server on port ```54322```. No prefix is supported and no line ending, just the raw data. Can be used directly with ffmpeg streaming. Allows to change settings while using [ffmpeg streaming](#using-ffmpeg).

* ```<rate>```: The goal sampling rate of the data.
  * Integer value between 1 and 32000 (default: 8000)

* ```<measures>``` (optional):
  * "v,i" or "v,i,v,i,v,i": will return 3x _voltage_ and _current_ as "v_l1,i_l1,v_l2,i_l2,v_l3,i_l3"
  * "v,i_L1": will return _voltage_ and _current_ of L1
  * "v,i_L2": will return _voltage_ and _current_ of L2
  * "v,i_L3": will return _voltage_ and _current_ of L3
  * "p,q" or "p,p,p,q,q,q": will return 3x _active_ and _reactive power_ as "p_l1,p_l2,p_l3,q_l1,q_l2,q_l3"
  * "v,i_RMS" or "v,v,v,i,i,i_RMS": will return 3x _RMS voltage_ and _RMS current_
  * Default: "v,i"

* ```<prefix>``` (optional):
  * "true" or "false": if _true_, each chunk of measurement will be sent with the prefix ```"Data:"<chunk_length><packet_num><data>```
  * ```<chunk_length>```: 2 bytes stating the length the amount of bytes in ```<data>```, format: _```<H```_
  * ```<packet_num>```: running packet number as 4 bytes integer, format: _```<I```_
  * Default: _true_

* ```<ntpConf>``` (optional):
  * Integer, interpreted as milliseconds. The NTP request before starting the sampling needs to be confident within this threshold value. 
  * NOTE: NTP requests are send over UDP to the specified NTP server. The time it takes to get the answer needs to be considered as well for millisecond resolution. As the request has to be sent to the server and from the server back to the [SmartMeter], half of the time the request took is added to the received NTP time. The request is thus only confident up to the time it took to receive the response, as in the worst case - if a response took 10ms - it could be 0ms for sending to the server and 10ms for getting the response. This would mean, the time is off the actual time about 5ms - which is our confidence level. As most sampling is stored relative (start time + sampling rate), getting the start time as exact as possible is crucial. 
  * Default: no NTP request is sent

* ```<flowCtr>``` (optional):
  * "true" or "false": if _true_, the sink has to request _x_ amount of samples with the ```reqSamples``` command or actively enable sending with a [cts](#pause-streaming) command.
  * Default: _false_
  * Request ```<numSamples>``` using the command: ```{"cmd":"reqSamples","samples":<numSamples>}```
    * ```<numSamples>```: long, must be between 10 and 2000
    * NOTE: In order for the command to work, the [SmartMeter] must be sampling and during the sampling command ```flowCtr``` must have been set to _true_!

* ```<time>``` (optional):
  * Unix timestamp at which sampling should be started. The timestamp must be in the future more than 500ms but is not allowed to be further in time then 20s. 
  * NOTE: This can be used to start sampling with multiple devices at an exact point in time. Sampling further starts at a positive voltage zero crossing. Therewith, SmartMeters at the same phase are synchronized within 1/f<sub>L</sub> with f<sub>L</sub> being the grid line frequency.  
  * Default: Sampling is started immediately 

* ```[<slot>,<slots>]``` (optional):
  * ```<slot>``` integer, the slot number in which data is sent
  * ```<slots>``` integer, the total number of slots
  * The idea is that only one device sends data at the same time in a network with multiple device ```d_i```. Each device will only send data if the following condition is true: ```now.seconds%slots == slot```
  * Example: 3 devices _d<sub>i</sub>_ with configs: _d<sub>0</sub> = [0,3]_, _d<sub>1</sub> = [1,3]_, _d<sub>2</sub> = [2,3]_. All devices send data each 3 seconds. e.g. _d<sub>0</sub>_ at second _0,3,6,..._ 
  * NOTE: This only works, if all data sampled can be send out in this second. If you e.g. have 10 devices, one device has to send 10s of data every 10s within just 1s. If the SmartMeter is not able to sent all data within this second, buffer overflows will occur. However, it avoids wifi/tcp collisions caused by multiple SmartMeters.
  * Default: _false_

### Pause streaming
You can use flow control during sampling with the command: ```{"cmd":"cts","value":<value>}```
* ```<value>``` can be _true_ or _false_. _True_ to let device stream data, _false_ to pause.
* NOTE: does not necessarily have to be paired with sampling in which ```flowCtr``` was set to _true_.

### Stop streaming 
```{"cmd":"stop"}```\
Send this command to stop sampling or streaming. As a response, you get information about the sampling process.
```bash
Info:{"msg":"Received stop command","sample_duration":16157,"samples":16,"sent_samples":16,"start_ts":"1614697441.119","stop_ts":"1614697457.276","ip":"192.168.0.138","avg_rate":0.955618,"cmd":"stop"}
```

## Misc
### Availability
You can send ```?``` without a JSON encoding to test if the device is still reachable. It will return ```Info: Setup done``` and will stop sampling! (Just for backward compatibility)

### Keepalive
As a keepalive command, you can send ```!```. This simply gets ignored but might prevent the TCP connection from being closed automatically if nothing is sent.

## Data Format
The data is returned encoded as 32 Bit MSB first float values.
If not explicitly disabled using ```"prefix":"false"```, data is preceded by a prefix.\
```"Data:"<chunk_length><packet_num><data>```
  * ```<chunk_length>```: 2 bytes describing the length of this data chunk, format: _```<H```_
  * ```<packet_num>```: running packet number as 4 bytes integer, format: _```<I```_
If disable or the stream type is set to FFMPEG, just the ```<data>``` is returned.
