/*
  Read data from the TCA9548 multiplexer
*/

#include "TCA9548_multiplexer.h"

MULTI::MULTI(uint8_t addr) : mp(addr, &Wire){
  // Nothing to be done here
}

bool MULTI::init(byte addr){
  hardware_present = false;
  error_status = 0;

  // Start the multiplexer
  if (! mp.begin(addr)) {
    error_status = 1; // Error 1: Initialisation failed
    return(false);
  }

  // Additionally check if the multiplexer was detected
  if (! mp.isConnected()) {
    error_status = 2; // Error 2: Detection failed
    return(false);
  }

  for(int i=0; i<8; i++){
    mp.disableChannel(i);
    _currently_active_bus = 9999;
  }

  hardware_present = true;
  error_status = 0;
  return(true);
}

bool MULTI::enableBus(uint8_t bus){
  // Check if all buses are inactive, otherwise de-activate them
  if(_currently_active_bus != 9999){
    mp.disableChannel(_currently_active_bus);
  }
  mp.enableChannel(bus);
  _currently_active_bus = bus;
  return(mp.isEnabled(bus));
}

bool MULTI::disableBus(uint8_t bus){
  // Disable the given
  if(_currently_active_bus == bus){
    mp.disableChannel(bus);
    _currently_active_bus = 9999;
  } else {
    Serial.print("Bus "); Serial.print(bus); Serial.println(" not currently active");
  }
  return(mp.isEnabled(bus));
}

bool MULTI::disableAllBuses(){
  bool anyEnabled = false;
  for(int i=0; i<8; i++){
    mp.disableChannel(i);
    anyEnabled = anyEnabled || mp.isEnabled(i);
  }
  _currently_active_bus = 9999;
  return(anyEnabled);
}

bool MULTI::disableCurrentBus(){
  if(_currently_active_bus == 9999){
    return(false);
  }
  uint8_t bus = _currently_active_bus;
  mp.disableChannel(bus);
  _currently_active_bus = 9999;
  return(mp.isEnabled(bus));
}
