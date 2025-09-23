/*
  Read data from the BME280 sensor
*/
#pragma once

#ifndef BME280h_h
#define BME280h_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <math.h>

class BME {
  public:
    // main Class
    BME(void);
  
    // Functions
    bool  init(byte addr = 0x76);
    void  update_pressure_cal(float cal = 584.0);
    void  update_temperature_cal(float cal1 = -0.0075, float cal2 = 1.1291);
    void  update_measurements(int time_delay, int freq);
    float airT(void);
    float airRH(void);
    float airP(void);
    float altitude_asl(void);
    float altitude_agl(void);

    //float air_heat_capacity(float x);
    //float air_density(float x);
    //float air_water_mole_frac(float x);

  private:
    Adafruit_BME280 bme_sensor;
    int   i2c_bus_id = 1;
    bool  sensorPresent = false;
    int   error_status = 0;
    float starting_altitude = 0;
    // correction factor due to sensor inaccuracy, sea level calibration 584 Pa
    float _pressure_cal = 584.0;
    float _temperature_cal1 = -0.0075;
    float _temperature_cal2 = 1.1291;
};

#endif
