/*
 * ADS1115 Analog-to-digital converter
 *
 */

#ifndef ADS1115h_h
#define ADS1115h_h

#include "Arduino.h"

class ADS {
  public:
    // main Class
    ADS(void);
  
    // Functions
    //----------
    bool   init(void);
	  double read_mV(int port, int gain);
    double read_val(int port, int gain, int conversion);
    void   nothing(void);

  private:
    int error_status = 0;
    int i2c_bus_id = 1;
};

#endif
