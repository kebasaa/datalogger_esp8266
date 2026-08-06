/*
  Read data from the BME280 sensor
*/
#pragma once

#ifndef TCA9548_multiplexerh_h
#define TCA9548_multiplexerh_h

#include "Arduino.h"

#include <Wire.h>
#include <TCA9548.h>

// "no channel currently selected". Must fit in uint8_t: the TCA9548A has 8
// channels, so any value above 7 works and 0xFF is the conventional one.
#define MULTI_NO_BUS 0xFF

class MULTI {
  public:
    // main Class
    MULTI(uint8_t addr = 0x70);

    // Functions
    bool init(byte addr = 0x70);
    bool enableBus(uint8_t bus);
    bool disableBus(uint8_t bus);
    bool disableAllBuses();
    bool disableCurrentBus();

  private:
    TCA9548 mp;
    bool    hardware_present = false;
    int     error_status = 0;
    //uint8_t _channels = 0;
    uint8_t _currently_active_bus = MULTI_NO_BUS;
};

#endif
