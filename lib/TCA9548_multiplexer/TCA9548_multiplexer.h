/*
  Read data from the BME280 sensor
*/
#pragma once

#ifndef TCA9548_multiplexerh_h
#define TCA9548_multiplexerh_h

#include "Arduino.h"

#include <Wire.h>
#include <TCA9548.h>

class MULTI {
  public:
    // main Class
    MULTI(uint8_t addr = 0x70);
  
    // Functions
    bool init(byte addr = 0x70);
    bool enableBus(uint8_t bus);
    bool disableBus(uint8_t bus);
    bool disableCurrentBus();

  private:
    TCA9548 mp;
    bool multiplexerPresent = false;
    //uint8_t _channels = 0;
    uint8_t _currently_active_bus = 0;
};

#endif
