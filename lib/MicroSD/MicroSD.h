// MicroSD.h
#pragma once

#ifndef MICROSD_H
#define MICROSD_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <stdarg.h>

class MicroSD {
public:
  enum WriteStatus : uint16_t {
    WRITE_OK              = 0x0000,
    WRITE_CARD_MISSING    = 0x0001,
    WRITE_HEADER_OPEN_ERR = 0x0002,
    WRITE_APPEND_OPEN_ERR = 0x0004,
    WRITE_PRINT_ERR       = 0x0008,
    WRITE_FLUSH_ERR       = 0x0010,
    WRITE_CLOSE_ERR       = 0x0020
  };

  MicroSD();
  /**
   * Initialize the SD card. Returns true on success.
   * chipSelect: CS pin (default D8 on Wemos D1 Mini).
   * spiSpeed: SPI clock speed in Hz (use SD_SCK_MHZ macro).
   */
  bool init(uint8_t chipSelect = 15, uint32_t spiSpeed = SD_SCK_MHZ(50));

  /**
   * Get approximate SD card capacity in MB.
   * Note: free space calculation via cluster scanning is too slow on ESP8266.
   */
  float getCapacityMB();
  
  /**
   * Get free SD card space in MB by summing file sizes in root directory.
   */
  float getFreeMB();

  /**
   * Build a comma‐separated string of `count` float values, each printed to two decimals.
   * Usage: make_str(3, 1.23, 4.56, 7.89) --> "1.23,4.56,7.89"
   */
  String make_str(uint8_t count, float first, ...);

  /**
   * Append a line to file. If file did not exist, create it & write header_str first.
   */
  uint16_t write_data(const char* filename,
                      const char* header_str,
                      const char* data_str,
                      uint32_t logFreq_s);

  uint32_t cardMissingCount() const;
  uint32_t headerOpenFailCount() const;
  uint32_t appendOpenFailCount() const;
  uint32_t printFailCount() const;
  uint32_t flushFailCount() const;
  uint32_t closeFailCount() const;

  /**
   * Delete a file from the card. Returns true on success.
   */
  bool delete_file(const char* filename);

private:
  bool     cardPresent = false;  // true if init() succeeded
  uint8_t  _chipSelect = 15;
  uint32_t _spiSpeed = SD_SCK_MHZ(50);
  int      error_status = 0;
  uint32_t _cardMissingCount = 0;
  uint32_t _headerOpenFailCount = 0;
  uint32_t _appendOpenFailCount = 0;
  uint32_t _printFailCount = 0;
  uint32_t _flushFailCount = 0;
  uint32_t _closeFailCount = 0;
  SdFat    SD;                   // SdFat card object
  SdFile   dataFile;             // file handle
  // CRC functions that allow to ensure data integrity
  uint16_t crc16_ccitt_compute(const char* data, uint16_t init = 0xFFFF, uint16_t poly = 0x1021) ;
  String   hex4(uint16_t v);

};

#endif  // MICROSD_H
