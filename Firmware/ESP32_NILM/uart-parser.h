/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */


#ifndef UART_PARSER_H_
#define UART_PARSER_H_

#include "ADE9000.h"

void uart_parser_setup(ADE9000 *ade, HardwareSerial *uart);

void uart_parser(void);

void parseCommand(char *str);

#endif  // UART_PARSER_H_
