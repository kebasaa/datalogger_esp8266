/*
  Read data from the TCA9548 multiplexer
*/

#include "TCA9548_multiplexer.h"

MULTI::MULTI(uint8_t addr) : mp(addr, &Wire){
  // Nothing to be done here
}

bool MULTI::init(byte addr){
  // Start the multiplexer
  if (! mp.begin(addr)) {
    hardware_present = false;
    error_status = 1; // Error 1: Initialisation failed
  } else {
    hardware_present = true;
  }
  // Additionally check if the multiplexer was detected
  if (! mp.isConnected()) {
    hardware_present = false;
    error_status = 2; // Error 2: Detection failed
  } else {
    hardware_present = true;
  }
  
  // Now tell the calling function if the initialisation was successful
  if(!hardware_present){
    return(false);
  } else {
    return(true);
  }
}

bool MULTI::enableBus(uint8_t bus){
  // Check if all buses are inactive, otherwise de-activate them
  if(_currently_active_bus != 0){
    mp.disableChannel(_currently_active_bus);
  }
  mp.enableChannel(bus);
  _currently_active_bus = bus;
  return(mp.isEnabled(bus));
}

bool MULTI::disableBus(uint8_t bus){
  // Disable the given bus
  if(_currently_active_bus == bus){
    mp.disableChannel(bus);
    _currently_active_bus = 0;
  } else {
    Serial.print("Bus "); Serial.print(bus); Serial.println(" not currently active");
  }
  return(mp.isEnabled(bus));
}

bool MULTI::disableCurrentBus(){
  uint8_t bus = _currently_active_bus;
  mp.disableChannel(bus);
  _currently_active_bus = 0;
  return(mp.isEnabled(bus));
}
