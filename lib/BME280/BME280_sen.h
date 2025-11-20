/*
  Read data from the BME280 sensor
*/
#pragma once

#ifndef BME280senh_h
#define BME280senh_h

#include "Arduino.h"
#include <Wire.h>
#include <BME280I2C.h>
//#include <Adafruit_BME280.h>
#include <math.h>

#include "config.h"
#if I2C_MULTI
#include "TCA9548_multiplexer.h"
#endif

class BME {
  public:
    // main Class
  #if I2C_MULTI
    BME(MULTI* mux = nullptr, uint8_t bus = 0);
    void setMultiplexer(MULTI* mux, uint8_t bus);
  #else
    BME(void);
  #endif
  
    // Functions
    bool  init(void);
    void  update_pressure_cal(float cal = 584.0);
    void  update_temperature_cal(float cal1 = -0.0075, float cal2 = 1.1291);
    void  update_measurements(int time_delay, int freq);
    float airT(void);
    float airRH(void);
    float airP(void);
    float current_temperature = 0;
    float current_humidity = 0;
    float current_pressure = 0;

  private:
    //Adafruit_BME280 _bme;
    BME280I2C _bme;
    int   i2c_bus_id = 1;
    bool  sensorPresent = false;
    int   error_status = 0;
    // correction factor due to sensor inaccuracy, sea level calibration 584 Pa
    float _pressure_cal = 584.0;
    float _temperature_cal1 = -0.0075;
    float _temperature_cal2 = 1.1291;

  #if I2C_MULTI
    MULTI* _mux = nullptr;
    uint8_t _mux_bus = 0;
  #endif
};

#endif
