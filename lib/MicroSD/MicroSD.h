// MicroSD.h

#ifndef MICROSD_H
#define MICROSD_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <stdarg.h>

class MicroSD {
public:
  MicroSD();
  /**
   * Initialize the SD card. Returns true on success.
   * chipSelect: CS pin (default D8 on Wemos D1 Mini).
   * spiSpeed: SPI clock speed in Hz (use SD_SCK_MHZ macro).
   */
  bool init(uint8_t chipSelect = 15, uint32_t spiSpeed = SD_SCK_MHZ(50));

  /**
   * Build a comma‐separated string of `count` float values, each printed to two decimals.
   * Usage: make_str(3, 1.23, 4.56, 7.89) --> "1.23,4.56,7.89"
   */
  String make_str(uint8_t count, float first, ...);

  /**
   * Append a line to file. If file did not exist, create it & write header_str first.
   */
  void write_data(const String& filename,
                  const String& header_str,
                  const String& data_str);

  /**
   * Delete a file from the card. Returns true on success.
   */
  bool delete_file(const String& filename);

private:
  bool     cardPresent = false;  // true if init() succeeded
  SdFat    SD;                   // SdFat card object
  SdFile   dataFile;             // file handle
};

#endif  // MICROSD_H