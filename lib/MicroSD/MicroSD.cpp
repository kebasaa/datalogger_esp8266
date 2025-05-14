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

void MicroSD::write_data(const String& filename,
                         const String& header_str,
                         const String& data_str) {
  if (!cardPresent) return;

  // If file does not exist, create and write header
  if (!SD.exists(filename.c_str())) {
    dataFile.open(filename.c_str(), O_CREAT | O_WRITE);
    if (dataFile) {
      dataFile.println(header_str);
      dataFile.close();
    }
  }

  // Append data line
  dataFile.open(filename.c_str(), O_WRITE | O_APPEND);
  if (dataFile) {
    dataFile.println(data_str);
    dataFile.close();
  }
}

bool MicroSD::delete_file(const String& filename) {
  if (!cardPresent) return false;
  return SD.remove(filename.c_str());
}
