/*
 * SEN0465 O2 sensor (https://github.com/DFRobot/DFRobot_MultiGasSensor)
 */

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

class SEN0465 {
  public:
    // main Class
    //SEN0465(void);
    SEN0465(TwoWire &wire = Wire);
  
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
};

#endif
