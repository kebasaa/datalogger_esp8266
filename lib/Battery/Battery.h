/*
 * Battery
 */

#ifndef Batteryh_h
#define Batteryh_h

#include <ESP8266WiFi.h>
#include <math.h>

class Battery {
  public:
    // main Class
    Battery(void);
  
    // Functions
    //----------
    bool init(void);
	  float battery_mV(void);
	  float battery_pc(void);
    bool  is_battery_present(void);
 
  private:
    int  analogPin = A0;
    bool batteryPresent = false;

};

#endif
