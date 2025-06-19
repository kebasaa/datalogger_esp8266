/*
 * GPS sensor
 */

#ifndef GPSh_h
#define GPSh_h

#include "Arduino.h"
#include <Wire.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPS++.h>

class GPS {
  public:
    // main Class
    GPS(void);
  
    // Functions
    //----------
    bool init(TwoWire *i2cBus);
    void update_values(void);
    String get_timestamp(void);
    String get_date(void);
    String get_location(void);
    float get_lat(void);
    float get_lon(void);
    float get_alt(void);
};

#endif
