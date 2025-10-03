/*
 * ADS1115 Analog-to-digital converter
 *
 */
#pragma once

#ifndef ADS1115h_h
#define ADS1115h_h

#include "Arduino.h"

#include "config.h"
#if I2C_MULTI
#include "TCA9548_multiplexer.h"
#endif

class ADS {
  public:
    // main Class
  #if I2C_MULTI
    ADS(MULTI* mux = nullptr, uint8_t bus = 0);
    void setMultiplexer(MULTI* mux, uint8_t bus);
  #else
    ADS(void);
  #endif
  
    // Functions
    //----------
    bool   init(void);
	  double read_mV(int port, int gain);
    double read_val(int port, int gain, int conversion);
    void   nothing(void);

  private:
    bool sensorPresent = false;
    int  error_status = 0;
    int  i2c_bus_id = 1;

  
  #if I2C_MULTI
    MULTI* _mux = nullptr;
    uint8_t _mux_bus = 0;
  #endif
};

#endif
