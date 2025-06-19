// MicroSD.cpp

#include "MicroSD.h"

MicroSD::MicroSD() {}

bool MicroSD::init(uint8_t chipSelect, uint32_t spiSpeed) {
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

void MicroSD::write_data(const char* filename,
                         const char* header_str,
                         const char* data_str,
                         uint32_t logFreq_s) {
  if (!cardPresent) return;

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
  if (!SD.exists(filename)) {
    dataFile.open(filename, O_CREAT | O_WRITE);
    if (dataFile) {
      dataFile.println(header_str);
      dataFile.close();
    }
  }

  // Append data line
  dataFile.open(filename, O_WRITE | O_APPEND);
  if (dataFile) {
    dataFile.println(data_str);
    dataFile.close();
  }
}

bool MicroSD::delete_file(const char* filename) {
  if (!cardPresent) return false;
  return SD.remove(filename);
}
