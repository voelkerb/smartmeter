# SmartMeter

The SmartMeter is a WiFi and Ethernet equipped electricity meter that to be installed in a home's fuse box. 
The development started with hardware [Version 1.0](/Schematic/Version_1). It consists of an ESP32 microcontroller and a dedicated electricity monitoring chip. As the main idea behind this project was to record voltage and current waveforms at higher frequencies, several drawbacks for this use case were fixed with Hardware [Version 2.0](/Schematic/Version_2). It features the same Microcontroller but with 8MB internal storage for data buffering, an RTC for precise time keeping while sampling, a faster USB serial connection with up to 12Mbaud and a 100Mbit ethernet interface.

If you are just interested in the consumption of a single appliance, see the [PowerMeter](https://github.com/voelkerb/powermeter) instead.


## Why to use?
It's open source, easy to use, easy to build and offers lots of flexibility. 

The hardware is...
* tested to work reliable with 230V.
* rather cheap (all components sum up to around 70€).

The firmware supports...
* existing 2.4GHz WPA2 networks or can create its own network.
* to receive commands via USB, TCP or MQTT.
* to store the configuration in the internal non volatile memory.
* to provide data at sampling rates from 1/5Hz all the way up to 32kHz.
* to calibrate the measurements.
* integration into most existing smart home systems e.g. HomeKit (via homebridge and MQTT).
* FFMpeg streaming. This allows to store high frequency data directly into files (e.g. MKV).


## How to build one?
You want to build your own SmartMeter? As the PCBs are provided in this repo, you can simply make your own.

Depending on your requirements, you might either build [Version 1.0](/Schematic/Version_1) or [Version 2.0](/Schematic/Version_2). 
* [Version 1.0](/Schematic/Version_1) features an ESP32 microcontroller.
    * Benefits: 
        * The parts are cheaper (around 50€).
    * Drawbacks:
        * Cannot stream data of more than 8kHz.
        * No ethernet interface.
* [Version 2.0](/Schematic/Version_2) features an ESP32 microcontroller and a 16A bistable relay.
    * Benefits: 
        * 8MB of internal storage allows to buffer data. If you stream high frequency data at e.g. 2kHz, this can hold up to 200s of data on network dropouts.
        * An RTC keeps track of time allowing you to synchronize sampling rates during high frequency sampling.
    * Drawbacks:
        * A little bit more expensive (parts around 70€).

Steps to build your own SmartMeter:
1. Make the PCB ([Version 1.0](/Schematic/Version_1) or [Version 2.0](/Schematic/Version_2)) with [SampleBoard](/Schematic/SampleBoard)). I recommend to use [JLC](https://jlcpcb.com) as it is super cheap and the quality is still decent.
2. Buy the parts listed under [BOM](/BOM).
3. Solder everything together. We used a small reflow oven, but it could also be done using a fine soldering iron. 
4. Buy a matching housing. We highly recommend to use [this one](https://www.voelkner.de/products/694730/Weltron-MR9-C-FA-RAL7035-ABS-Hutschienen-Gehaeuse-157.5-x-90-x-68-ABS-Lichtgrau-RAL-7035-1St..html) as the PCB was specially designed for it, the mounting holes match, and it is safe to use with 230V. It further can be installed inside fuse boxes at regular DIN rails. Also see the CAM milling files to generate cutouts for USB and ethernet and mounting plates for the sampler board (see [Housing](/Hosuing)). 
5. Install the SmartMeter at the DIN rail inside fuse box (that should be done by an electrician).

<img src="/docu/figures/installation2.jpeg" width="450px">

6. Wire the SmartMeter using the following diagram. 

<img src="/docu/figures/installation.png">

7. Upload the [firmware](/Firmware) according to the instructions for your version. 
8. Interface with the SmartMeter accordingly as stated [here](/docu/README_Firmware_Cmds.md) 

# Use cases
## Smart homes
There are plenty of use cases for a smart plug inside a smart home.
* Get the power consumption of your home using a voice assistant or your smartphone
* Include it in home automation software to switch appliances based on an automation. There are lots of possibilities here, e.g.: 
    * Trigger an alert if the consumption exceeds a limit.
<img src="/docu/figures/eveApp.jpeg" width="400px">
* Implement your own Non-Intrusive Load Monitoring System to get the individual appliance power consumption without adding smart plugs to each and every appliance.
* Monitor the power consumption of your home over day.
<img src="/docu/figures/home.jpg">

## For research purpose
* Analyze high frequency voltage and current waveforms.
* Record high frequency electricity datasets (see e.g. the [FIRED](https://github.com/voelkerb/FIRED_dataset_helper) dataset which can be used for Non-Intrusive Load Monitoring).


# License:
## Firmware:
Copyright (c) 2019 Benjamin Völker. All rights reserved.
This work is licensed under the terms of the [CC 4.0 licence](https://creativecommons.org/licenses/by/4.0/).

## Hardware:
Copyright (c) 2019 Benjamin Völker. All rights reserved.
This work is licensed under the terms of the [TAPR Open Hardware License](https://web.tapr.org/TAPR_Open_Hardware_License_v1.0.txt).

# Reference

Please cite our publications if you compare to or use this system:
* Benjamin Völker, Philipp M. Scholl, and Bernd Becker. 2019. Semi-Automatic Generation and Labeling of Training Data for Non-intrusive Load Monitoring. In Proceedings of the Tenth ACM International Conference on Future Energy Systems (e-Energy '19). Association for Computing Machinery, New York, NY, USA, 17–23. DOI:https://doi.org/10.1145/3307772.3328295
 
* Benjamin Völker, Marc Pfeifer, Philipp M. Scholl, and Bernd Becker. 2020. FIRED: A Fully-labeled hIgh-fRequency Electricity Disaggregation Dataset. In Proceedings of the 7th ACM International Conference on Systems for Energy-Efficient Buildings, Cities, and Transportation (BuildSys '20). Association for Computing Machinery, New York, NY, USA, 294–297. DOI:https://doi.org/10.1145/3408308.3427623

* Völker, B.; Pfeifer, M.; Scholl, P.M.; Becker, B. A Framework to Generate and Label Datasets for Non-Intrusive Load Monitoring. Energies 2021, 14, 75. https://doi.org/10.3390/en14010075


Kudos also go to Pascal Verboket & Valentin Czisch for developing the first draft schematics prior to [Version 1.0](/Schematic/Version_1).
