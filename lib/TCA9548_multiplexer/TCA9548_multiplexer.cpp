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

  // Start the multiplexer. NOTE: TCA9548::begin() takes a CHANNEL MASK, not an
  // address - the address is fixed by the constructor. Passing addr here (0x70)
  // used to enable channels 4, 5 and 6 at once, bridging three mux segments onto
  // the bus until the disable loop below ran. Start with everything off.
  (void)addr;
  if (! mp.begin(0x00)) {
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
    _currently_active_bus = MULTI_NO_BUS;
  }

  hardware_present = true;
  error_status = 0;
  return(true);
}

bool MULTI::enableBus(uint8_t bus){
  // Check if all buses are inactive, otherwise de-activate them
  if(_currently_active_bus != MULTI_NO_BUS){
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
    _currently_active_bus = MULTI_NO_BUS;
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
  _currently_active_bus = MULTI_NO_BUS;
  return(anyEnabled);
}

bool MULTI::disableCurrentBus(){
  if(_currently_active_bus == MULTI_NO_BUS){
    return(false);
  }
  uint8_t bus = _currently_active_bus;
  mp.disableChannel(bus);
  _currently_active_bus = MULTI_NO_BUS;
  return(mp.isEnabled(bus));
}
