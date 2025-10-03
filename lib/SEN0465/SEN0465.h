/*
 * SEN0465 O2 sensor (https://github.com/DFRobot/DFRobot_MultiGasSensor)
 */
#pragma once

#ifndef SEN0465h_h
#define SEN0465h_h

#include "Arduino.h"

// H4plugins has ON and OFF defined. These need to be de-activated for the code to work
#ifdef ON
  #undef ON
#endif
#ifdef OFF
  #undef OFF
#endif
#include <DFRobot_MultiGasSensor.h>

#include "config.h"
#if I2C_MULTI
#include "TCA9548_multiplexer.h"
#endif

class SEN0465 {
  public:
    // main Class
    //SEN0465(TwoWire &wire = Wire);
  #if I2C_MULTI
    SEN0465(MULTI* mux, uint8_t bus, TwoWire &wire = Wire);
    void setMultiplexer(MULTI* mux, uint8_t bus);
  #else
    SEN0465(TwoWire &wire = Wire);
  #endif
  
    // Functions
    //----------
    bool   init(uint8_t addr = 0x77);
    String gasType(void);
	  float  airO2(void);
    float  airT(void);
    float  rawV(void);
	
  private:
    DFRobot_GAS_I2C sen;
    bool sensorPresent = false;
    int  error_status = 0;
    int  i2c_bus_id = 1;

  #if I2C_MULTI
    MULTI* _mux = nullptr;
    uint8_t _mux_bus = 0;
  #endif
};

#endif
