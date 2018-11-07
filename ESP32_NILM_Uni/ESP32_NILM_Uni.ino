/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#include <SPI.h>

#include "ADE9000.h"
#include "uart-parser.h"

ADE9000 *ADE9k;

HardwareSerial Serial2(1);

void setup() {
  Serial.begin(2000000);
  Serial.println("\nInfo:Setup start");
  //Serial2.begin(2000000, SERIAL_8N1, 2, 0, false); // FTDI 12Mbaud
  ADE9k = new ADE9000(VSPI);
  ADE9k->setup();
  // pass serial to uart handler use 6 channels
  uart_parser_setup(ADE9k, &Serial, 6);
  delay(500);
  Serial.println("Info:Setup done");
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000);
  uart_parser();
}
