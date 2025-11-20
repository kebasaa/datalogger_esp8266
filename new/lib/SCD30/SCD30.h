/*
 * SCD CO2 sensor
 */
#pragma once

#ifndef SCD30h_h
#define SCD30h_h

#include "Arduino.h"
#include <SparkFun_SCD30_Arduino_Library.h>

#include "config.h"
#if I2C_MULTI
#include "TCA9548_multiplexer.h"
#endif

class SCD {
  public:
    public:
    // main Class
  #if I2C_MULTI
    SCD(MULTI* mux = nullptr, uint8_t bus = 0);
    void setMultiplexer(MULTI* mux, uint8_t bus);
  #else
    SCD(void);
  #endif
  
    // Functions
    //----------
    bool  init(void);
    float airT(void);
    float airRH(void);
    float airCO2(void);
	
	  void set_interval(int interval);
	  void set_air_pressure(float pressure_Pa);
    bool calibrate_with_reference(uint16_t reference_gas);
	  void read_calibration_value(void);
    void enable_self_calibration(bool enable=false);
	
  private:
    SCD30 co2sensor;
    bool  sensorPresent = false;
    int   error_status = 0;
    int   i2c_bus_id = 1;
    float co2_value = 0;
    float rh_value = 0;
    float t_value = 0;

  #if I2C_MULTI
    MULTI* _mux = nullptr;
    uint8_t _mux_bus = 0;
  #endif
};

#endif
