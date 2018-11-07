/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#include <SPI.h>
#include <driver/spi_common.h>
#include <soc/spi_struct.h>

// #include <esp32/rom/crc.h> // crc.h des ESP32 liefter falsche Ergebnisse
#include "CRC-16-CCITT.h"  

#include "esp32-hal-spi.h"

#include "ADE9000.h"
#include "ADE9000_REGISTER.h"
#include "ADE9000_PORT.h"

#include "esp32-hal-spi.h"

#include "RTOS-ESP32-helper.h"


struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};


typedef struct{
  uint32_t adc[7] = { 0 };
}CurrentADC;

static ADE9000 *_ADE9k;  // pointer for ISR

volatile uint16_t counter_loop = 0;
volatile bool timeout = false;


char hex(uint8_t i) {
  i = i & 0xf;
  if(i <= 9) {
    return i + '0';
  } else if (i <= 0xf) {
    return i - 10 + 'A';
  }
  return '!';
}

void my_hex(uint32_t data) {
  Serial.print(" ");
  char str[9];
  str[8] = 0;

  str[7] = hex(data);
  data = data >> 4;
  str[6] = hex(data);
  data = data >> 4;
  str[5] = hex(data);
  data = data >> 4;
  str[4] = hex(data);
  data = data >> 4;
  str[3] = hex(data);
  data = data >> 4;
  str[2] = hex(data);
  data = data >> 4;
  str[1] = hex(data);
  data = data >> 4;
  str[0] = hex(data);
  Serial.print(str);
}


// _____________________________________________________________________________
ADE9000::ADE9000(uint8_t spi_bus) :_spi_num(spi_bus), _ss(-1) {
  _spi = new SPIClass(spi_bus);
  _ADE9k = this;
  _burst_en = false;
}


// _____________________________________________________________________________
ADE9000::~ADE9000() {
  delete _spi;
  _ADE9k = NULL;
}


// _____________________________________________________________________________
void ADE9000::setup(void){

  // Power Mode
  pinMode(PM0_PIN, OUTPUT);
  pinMode(PM1_PIN, OUTPUT);
  pinMode(N_RESET_PIN, OUTPUT);

  this->reset();

  this->begin();
  // DREADY = Sinc4 + IIR LPF output at 8 kSPS.
  // this->write_16(0x4A0,0x0200);
  // select which function to output on:Here DREADY
  // this->write_16(0x481,0x000C);

  pinMode(DREADY_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(DREADY_PIN), ISR_dready, RISING);
}


// _____________________________________________________________________________
void ADE9000::reset(void){
  detachInterrupt(digitalPinToInterrupt(DREADY_PIN));
  digitalWrite(N_RESET_PIN, LOW);
  delay(100);
  digitalWrite(N_RESET_PIN, HIGH);
  delay(500);
}


// _____________________________________________________________________________
void ADE9000::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) {
  _spi->setDataMode(SPI_MODE0);
  _spi->setFrequency(20000000);

  _spi->begin(sck, miso, mosi, ss);
  _spi->setHwCs(true);

  // Wäre schön, wenn die delay auch in die richtige Richtung gehen würden
  // MISO wird nicht später, sondern früher ausgewertet :-(
  //spi->dev->pin.ck_idle_edge = 0;
  //_spi->bus()->ctrl2.miso_delay_num = 0;
  //_spi->bus()->dev->ctrl2.miso_delay_num  = 0b1000;
  //_spi->bus()->dev->ctrl2.miso_delay_mode = 3;
 
  // FIXME leider ist die Variablen _ss usw von der SPIClass private...
  if (sck == -1 && miso == -1 && mosi == -1 && ss == -1) {
    _ss = (_spi_num == VSPI) ? SS : 15;
  } else {
    _ss = ss;
  }

}


// _____________________________________________________________________________
typedef void (*voidFuncPtr)(void);

void ADE9000::setupInterrupt(voidFuncPtr handleInterrupt) {
  attachInterruptPinnedToCore(digitalPinToInterrupt(DREADY_PIN),
      handleInterrupt, RISING, 1);
}

// _____________________________________________________________________________
void ADE9000::clearInterrupt(void) {
  detachInterrupt(DREADY_PIN);
}


// _____________________________________________________________________________
void ADE9000::end(void) {
  // digitalWrite(_ss, HIGH);
  // pinMode(_ss, INPUT);
}


// _____________________________________________________________________________
void ADE9000::dataBitShift(uint8_t *data, uint16_t size) {
  for (uint16_t i = 0; i < size - 1; i++){
    data[i] = data[i] << 1;
    data[i] |= data[i+1] >> 7;
  }
  data[size-1] = data[size-1] << 1;
}


// _____________________________________________________________________________
uint16_t ADE9000::read_16(uint16_t addr) {
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[7];
  uint8_t tx_array[7] = { 0 };

  addr = addr << 4;

  tx_array[0] = addr >> 8;    // addr [15 - 8]
  tx_array[1] = addr | 0x8;   // addr [7 - 4], read-bit[3]

  _spi->transferBytes( tx_array, rx_array, 7);

  dataBitShift(rx_array, 7);

  uint8_t rx_data[2];
  rx_data[0] = rx_array[2];
  rx_data[1] = rx_array[3];


  uint16_t rx_crc = (rx_array[4] << 8) | rx_array[5];

  // CRC check
  if (!_burst_en) {
    if ( rx_crc != CRCCCITT(rx_data, 2, 0xffff, 0)) {
      Serial.println("\nCRC ERROR!");
      Serial.print("RX  : 0x");
      Serial.println(rx_crc , HEX);
      Serial.print("calc: 0x");
      Serial.println(CRCCCITT(rx_data, 2, 0xffff, 0), HEX);
    }
  }
  return (rx_array[2] << 8) | rx_array[3];
}


// _____________________________________________________________________________
uint32_t ADE9000::read_32(uint16_t addr) {
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[9];
  uint8_t tx_array[9] = { 0 };

  addr = addr << 4;

  tx_array[0] = addr >> 8;    // addr [15 - 8]
  tx_array[1] = addr | 0x8;   // addr [7 - 4], read-bit[3]

  _spi->transferBytes( tx_array, rx_array, 9);

  dataBitShift(rx_array, 9);

  uint8_t rx_data[4];
  rx_data[0] = rx_array[2];
  rx_data[1] = rx_array[3];
  rx_data[2] = rx_array[4];
  rx_data[3] = rx_array[5];

  uint16_t rx_crc = (rx_array[6] << 8) | rx_array[7];

  // CRC check
  if (!_burst_en) {
    if ( rx_crc != CRCCCITT(rx_data, 4, 0xffff, 0)) {
      Serial.println("\nCRC ERROR!");
      // Serial.print("RX  : 0x");
      // Serial.println(rx_crc , HEX);
      // Serial.print("calc: 0x");
      // Serial.println(CRCCCITT(rx_data, 4, 0xffff, 0), HEX);
    }
  }
  uint32_t data = rx_array[2] << 24;
  data |= rx_array[3] << 16;
  data |= rx_array[4] << 8;
  data |= rx_array[5];

  return data;
}


// _____________________________________________________________________________
void ADE9000::write_16(uint16_t addr, uint16_t data){
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[4];
  uint8_t tx_array[4] = { 0 };

  uint16_t caddr = addr << 4;

  tx_array[0] = caddr >> 8;    // addr [15 - 8]
  tx_array[1] = caddr & ~(0x8);   // addr [7 - 4], read-bit[3]

  tx_array[2] = data >> 8;
  tx_array[3] = data;

  _spi->transferBytes( tx_array, rx_array, 4);

  // verify write operation
  uint16_t read_data = this->read_16(addr);
  if(read_data != data) {
    Serial.println("Error");
    Serial.print("write: 0x");
    Serial.println(data , HEX);
    Serial.print("read:  0x");
    Serial.println(read_data, HEX);
  }
}


// _____________________________________________________________________________
void ADE9000::write_32(uint16_t addr, uint32_t data){
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[6];
  uint8_t tx_array[6] = { 0 };

  uint16_t caddr = addr << 4;

  tx_array[0] = caddr >> 8;    // addr [15 - 8]
  tx_array[1] = caddr | 0x8;   // addr [7 - 4], read-bit[3]

  tx_array[2] = data >> 24;
  tx_array[3] = data >> 16;
  tx_array[4] = data >> 8;
  tx_array[5] = data;

  _spi->transferBytes( tx_array, rx_array, 6);

  // verify write operation
  uint32_t read_data = this->read_32(addr);
  if(read_data != data) {
    Serial.println("Error");
    Serial.print("write: 0x");
    Serial.println(data , HEX);
    Serial.print("read:  0x");
    Serial.println(read_data, HEX);
  }
}


// _____________________________________________________________________________
void ADE9000::burst_read_en(bool enable){
  uint16_t buffer = this->read_16(CONFIG1);
  if(enable){
    _burst_en = true;
    write_16(CONFIG1, buffer | (1 << CONFIG1_BURST_EN) );
  } else {
    write_16(CONFIG1, buffer & ~(1 << CONFIG1_BURST_EN) );
    _burst_en = false;
  }
}


// _____________________________________________________________________________
void ADE9000::burst_read(uint16_t addr, uint32_t *data, uint16_t size){
    // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[ 2 + (size*4) ];
  uint8_t tx_array[ 2 + (size*4) ] = { 0 };

  if (!_burst_en) {
    this->burst_read_en(true);
  }

  tx_array[0] = addr >> 8;    // addr [15 - 8]
  tx_array[1] = addr | 0x8;   // addr [7 - 4], read-bit[3]

  _spi->transferBytes( tx_array, rx_array, 2 + size*4);

  for (uint8_t i = 0; i < size; i++){
    data[i] = rx_array[ 2+i*4 ] << 24;
    data[i] |= rx_array[ 3+i*4 ] << 16;
    data[i] |= rx_array[ 4+i*4 ] << 8;
    data[i] |= rx_array[ 5+i*4 ];
  }
}


// _____________________________________________________________________________
void ADE9000::burst_write_SPI(uint8_t *tx_data, uint8_t bytes) {
  if(!_spi->bus()) {
    return;
  }
  int i;

  if(bytes > 64) {
    bytes = 64;
  }

  uint32_t words = (bytes + 3) / 4;//16 max

  uint32_t wordsBuf[16] = {0,};
  uint8_t * bytesBuf = (uint8_t *) wordsBuf;

  memcpy(bytesBuf, tx_data, bytes);//copy data to buffer

  _spi->bus()->dev->mosi_dlen.usr_mosi_dbitlen = ((bytes * 8) - 1);
  _spi->bus()->dev->miso_dlen.usr_miso_dbitlen = ((bytes * 8) - 1);

  for(i=0; i<words; i++) {
    _spi->bus()->dev->data_buf[i] = wordsBuf[i];    //copy buffer to spi fifo
  }

  _spi->bus()->dev->cmd.usr = 1;
}

// _____________________________________________________________________________
bool ADE9000::burst_read_SPI_Buffer(uint8_t *rx_data, uint8_t bytes) {
  if(!_spi->bus()) {
    return false;
  }
  int i;

  if(bytes > 64) {
    bytes = 64;
  }

  uint32_t words = (bytes + 3) / 4;//16 max

  uint32_t wordsBuf[16] = {0,};
  uint8_t * bytesBuf = (uint8_t *) wordsBuf;

  // spi->dev->mosi_dlen.usr_mosi_dbitlen = ((bytes * 8) - 1);
  // spi->dev->miso_dlen.usr_miso_dbitlen = ((bytes * 8) - 1);


  // FIXME dont use "bytes", use (spi->dev->miso_dlen.usr_miso_dbitlen+1) / 8

  while(_spi->bus()->dev->cmd.usr);

  for(i=0; i<words; i++) {
      wordsBuf[i] = _spi->bus()->dev->data_buf[i];//copy spi fifo to buffer
  }
  memcpy(rx_data, bytesBuf, bytes);//copy buffer to output
  return true;
}
