/*
  Read data from the BME280 sensor
*/

#ifndef BME280h_h
#define BME280h_h

#include "Arduino.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

class BME {
  public:
    // main Class
    BME(void);
  
    // Functions
    bool  init(byte addr = 0x76);
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
    bool sensorPresent = false;
    float starting_altitude = 0;
};

#endif
