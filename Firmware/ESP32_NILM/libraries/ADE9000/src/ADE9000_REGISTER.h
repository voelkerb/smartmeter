/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#ifndef ADE9000_REGISTER_H_
#define ADE9000_REGISTER_H_

#define AI_LPF_DAT  0x510  // channel A ADC waveforms at 8 kSPS.
#define AV_LPF_DAT  0x511  // Voltage channel A ADC waveforms at 8 kSPS.
#define BI_LPF_DAT  0x512
#define BV_LPF_DAT  0x513
#define CI_LPF_DAT  0x514
#define CV_LPF_DAT  0x515
#define NI_LPF_DAT  0x516

// CONFIG1
#define CONFIG1                     0x481
#define CONFIG1_EXT_REF             15
#define CONFIG1_BURST_EN            11
#define CONFIG1_DIP_SWELL_IRQ_MODE  10
#define CONFIG1_PWR_SETTLE          8
#define CONFIG1_CF_ACC_CLR          5
#define CONFIG1_CF4_CFG             2
#define CONFIG1_CF3_CFG             1
#define CONFIG1_SWRST               0

#endif  // ADE9000_REGISTER_H_
