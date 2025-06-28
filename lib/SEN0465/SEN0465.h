/*
 * SCD CO2 sensor
 */

#ifndef SEN0465h_h
#define SEN0465h_h

#include "Arduino.h"
//#include <SparkFun_SEN0465_Arduino_Library.h>

class SCD {
  public:
    // main Class
    SCD(void);
  
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
    float o2sensor;
    bool sensorPresent = false;
};

#endif
