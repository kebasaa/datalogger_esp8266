// MicroSD.cpp

#include "MicroSD.h"

MicroSD::MicroSD() {}

bool MicroSD::init(uint8_t chipSelect, uint32_t spiSpeed) {
  _chipSelect = chipSelect;
  _spiSpeed = spiSpeed;
  // Initialize SPI and mount SD card
  if (!SD.begin(chipSelect, spiSpeed)) {
    cardPresent = false;
    return false;
  }
  cardPresent = true;
  return true;
}

/**
 * Get SD card free space in MB
 */
float MicroSD::getCapacityMB() {
  if (!cardPresent) return 0.0f;
  // Total sectors on card
  uint64_t sectors = SD.card()->sectorCount();
  uint64_t bytes = sectors * 512ULL;
  return bytes / (1024.0f * 1024.0f);
}

float MicroSD::getFreeMB() {
  if (!cardPresent) return 0.0f;

  // Total capacity in bytes
  uint64_t totalBytes = SD.card()->sectorCount() * 512ULL;

  // Sum sizes of all files in root directory
  uint64_t usedBytes = 0;
  SdFile root;
  if (root.open("/", O_READ)) {
    do {
      if (root.isFile()) {
        usedBytes += root.fileSize();
      }
    } while (root.openNext(&root, O_READ));
    root.close();
  }

  // Compute free bytes
  uint64_t freeBytes = (usedBytes < totalBytes) ? (totalBytes - usedBytes) : 0;
  return freeBytes / (1024.0f * 1024.0f);
}

String MicroSD::make_str(uint8_t count, float first, ...) {
  if (count == 0) return String();

  String result = String(first, 2);
  va_list args;
  va_start(args, first);
  for (uint8_t i = 1; i < count; ++i) {
    float val = va_arg(args, double); // floats promoted to double
    result += ",";
    result += String(val, 2);
  }
  va_end(args);
  return result;
}

uint16_t MicroSD::write_data(const char* filename,
                             const char* header_str,
                             const char* data_str,
                             uint32_t logFreq_s) {
  uint16_t status = WRITE_OK;
  if (!cardPresent && !init(_chipSelect, _spiSpeed)) {
    _cardMissingCount++;
    return WRITE_CARD_MISSING;
  }

  // Estimate required space for one full day of logs
  size_t entrySize = strlen(data_str) + 2; // data + newline
  uint32_t entriesPerDay = 86400UL / logFreq_s;
  uint64_t requiredBytes = uint64_t(entrySize) * entriesPerDay;

  // Ensure enough free space by deleting oldest files
  uint64_t freeBytes = uint64_t((getFreeMB()-10) * 1024.0f * 1024.0f);
  while (freeBytes < requiredBytes) {
    SdFile root;
    String oldestName;
    uint16_t oldestDate = 0xFFFF;
    uint16_t oldestTime = 0xFFFF;
    if (!root.open("/", O_READ)) break;
    char nameBuf[64];
    do {
      if (root.isFile()) {
        uint16_t fdate, ftime;
        if (root.getModifyDateTime(&fdate, &ftime)) {
          if (fdate < oldestDate || (fdate == oldestDate && ftime < oldestTime)) {
            oldestDate = fdate;
            oldestTime = ftime;
            // get filename into buffer
            root.getName(nameBuf, sizeof(nameBuf));
            oldestName = String(nameBuf);
          }
        }
      }
    } while (root.openNext(&root, O_READ));
    root.close();
    if (oldestName.length() == 0) break;
    Serial.print("Oldest file:"); Serial.println(oldestName);
    SD.remove(oldestName.c_str());
    freeBytes = uint64_t(getFreeMB() * 1024.0f * 1024.0f);
  }

  // If file does not exist, create and write header
  String header_out_str = String(header_str) + "CRC16";
  if (!SD.exists(filename)) {
    dataFile.open(filename, O_CREAT | O_WRITE);
    if (dataFile) {
      if(!dataFile.println(header_out_str)) {
        status |= WRITE_PRINT_ERR;
        _printFailCount++;
      }
      if(!dataFile.sync()) {
        status |= WRITE_FLUSH_ERR;
        _flushFailCount++;
      }
      if(!dataFile.close()) {
        status |= WRITE_CLOSE_ERR;
        _closeFailCount++;
      }
    } else {
      status |= WRITE_HEADER_OPEN_ERR;
      _headerOpenFailCount++;
    }
  }

  // Append data line
  // First compute CRC16 and add it
  uint16_t crc = crc16_ccitt_compute(data_str); // Append CRC as last CSV field, e.g. "23.5,47.1,1023,AB3F"
  String out_str = String(data_str) + hex4(crc);
  // Now write data
  dataFile.open(filename, O_WRITE | O_APPEND);
  if (dataFile) {
    //dataFile.println(data_str);
    if(!dataFile.println(out_str)) {
      status |= WRITE_PRINT_ERR;
      _printFailCount++;
    }
    if(!dataFile.sync()) {
      status |= WRITE_FLUSH_ERR;
      _flushFailCount++;
    }
    if(!dataFile.close()) {
      status |= WRITE_CLOSE_ERR;
      _closeFailCount++;
    }
  } else {
    status |= WRITE_APPEND_OPEN_ERR;
    _appendOpenFailCount++;
  }
  return status;
}

uint32_t MicroSD::cardMissingCount() const { return _cardMissingCount; }
uint32_t MicroSD::headerOpenFailCount() const { return _headerOpenFailCount; }
uint32_t MicroSD::appendOpenFailCount() const { return _appendOpenFailCount; }
uint32_t MicroSD::printFailCount() const { return _printFailCount; }
uint32_t MicroSD::flushFailCount() const { return _flushFailCount; }
uint32_t MicroSD::closeFailCount() const { return _closeFailCount; }

bool MicroSD::delete_file(const char* filename) {
  if (!cardPresent) return false;
  return SD.remove(filename);
}

uint16_t MicroSD::crc16_ccitt_compute(const char* data, uint16_t init, uint16_t poly) {
  const uint8_t *ptr = (const uint8_t*)data;
  size_t len = strlen(data);
  uint16_t crc = init;
  while (len--) {
    crc ^= ((uint16_t)(*ptr++) << 8);
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x8000) crc = (crc << 1) ^ poly;
      else crc <<= 1;
    }
  }
  return crc;
}

// Helper: convert uint16_t to 4-digit hex string (uppercase)
String MicroSD::hex4(uint16_t v) {
  char buf[5];
  sprintf(buf, "%04X", v & 0xFFFF);
  return String(buf);
}
