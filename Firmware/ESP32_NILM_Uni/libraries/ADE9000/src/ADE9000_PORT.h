/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#ifndef ADE9000_PORT_H_
#define ADE9000_PORT_H_


#define PM0_PIN       22
#define PM1_PIN       21

#define N_RESET_PIN   16

// Interrupt Request Output. This pins are active low logic output.
#define N_IRQ0_PIN    32
#define N_IRQ1_PIN    34

// Calibration Frequency (CF) Logic Outputs
#define CF1_PIN       33
#define CF2_PIN       25
#define CF3_PIN       26

// Event Pin/Data Ready
#define CF4_PIN       27
#define N_EBENT_PIN   CF4_PIN
#define DREADY_PIN    CF4_PIN

#endif  // ADE9000_PORT_H_
