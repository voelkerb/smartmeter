// Soruce: http://automationwiki.com/index.php/CRC-16-CCITT


#ifndef CRC_16_CCITT_H_
#define CRC_16_CCITT_H_



// leider funktioniert die CRC Berechnung des ESP32 (esp32/rom/crc.h) nicht wie
// erwartet, deswegen die Software implementation.

uint16_t CRCCCITT(uint8_t *data, uint16_t length, uint16_t seed, uint16_t final);

#endif  // CRC_16_CCITT_H_
