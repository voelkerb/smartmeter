/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#include <SPI.h>

#include "ADE9000.h"
#include "uart-parser.h"

ADE9000 *ADE9k;

//HardwareSerial Serial2(1);

void setup() {
  Serial.begin(2000000); // strt 20 MHz SPI-Mode
  //Serial2.begin(12000000, SERIAL_8N1, -1, 2, false); // FTDI 12Mbaud
  ADE9k = new ADE9000(VSPI);
  ADE9k->setup();
  uart_parser_setup(ADE9k, &Serial);
  //Serial.setDebugOutput(NULL);
  delay(500);
}

void loop() {
  uart_parser();
}
