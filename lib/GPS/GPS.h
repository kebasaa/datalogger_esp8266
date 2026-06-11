/*
 * GPS sensor
 */
#pragma once

#ifndef GPSh_h
#define GPSh_h

#include "Arduino.h"
#include <Wire.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPS++.h>

#include "config.h"
#if I2C_MULTI
#include "TCA9548_multiplexer.h"
#endif

class GPS {
  public:
    // main Class
  #if I2C_MULTI
    GPS(MULTI* mux = nullptr, uint8_t bus = 0);
    void setMultiplexer(MULTI* mux, uint8_t bus);
  #else
    GPS(void);
  #endif
  
    // Functions
    //----------
    bool   init();
    uint32_t update_values(void);
    bool   dateValid(void);
    bool   timeValid(void);
    bool   locationValid(void);
    bool   altitudeValid(void);
    uint32_t charsProcessed(void);
    uint32_t gpsAgeMs(void);
    uint32_t locationAgeMs(void);
    uint32_t satellites(void);
    float  hdop(void);
    uint16_t year(void);
    uint8_t month(void);
    uint8_t day(void);
    uint8_t hour(void);
    uint8_t minute(void);
    uint8_t second(void);
    String get_timestamp(void);
    String get_date(void);
    String get_location(void);
    String get_short_location(void);
    float  get_lat(void);
    float  get_lon(void);
    float  get_alt(void);
    unsigned long seconds_since_midnight(void);

  private:
    int i2c_bus_id = 1;
    int error_status = 0;

  #if I2C_MULTI
    MULTI* _mux = nullptr;
    uint8_t _mux_bus = 0;
  #endif
};

#endif
