/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#ifndef ADE9000_H_
#define ADE9000_H_

#define ADE9000_DEFAULT_SPI   VSPI

#include <SPI.h>

class ADE9000 {
 private:
  SPIClass *_spi;

  int8_t _spi_num;
  int8_t _ss;

  bool _burst_en;



 public:
  // Constructor build a new SPI object
   ADE9000(uint8_t spi_bus = VSPI);  // FIXME
  ~ADE9000(void);



  // config Interrupt, reset ADE9000 and conig it with defaults
  void setup(void);

  // reset the ADE9000
  void reset(void);

  // open SPI (you can reconfig the SPI signals)
  void begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t ss = -1);

  // setup ISR, used the IRAM_ATTR Macro for initialization!!!
  // Exampe: void IRAM_ATTR ISR_FUNC_NAME(void) { ... }
  // [ ... ]
  // call: setupInterrupt( ISR_FUNC_NAME );
  void setupInterrupt(void (*)(void));

  void clearInterrupt(void);

  // close SPI
  void end(void);

  void dataBitShift(uint8_t *data, uint16_t size);

  uint16_t read_16(uint16_t addr);

  uint32_t read_32(uint16_t addr);

  void write_16(uint16_t addr, uint16_t data);

  void write_32(uint16_t addr, uint32_t data);

  void burst_read_en(bool enable);

  void burst_read(uint16_t addr, uint32_t *data, uint16_t size);

  // Unfortunately the "RX-Buffer-Full-interrupt" of the ESP-SPI does not work
  // as expected, therefore the SPI bus must be accessed close to the hardware:
  // write Data on SPI out
  void burst_write_SPI(uint8_t *tx_data, uint8_t bytes);

  // read RX Buffer. Returns "False" if the SPI is still busy.
  bool burst_read_SPI_Buffer(uint8_t *rx_data, uint8_t bytes);
};

#endif  // ADE9000_H_
