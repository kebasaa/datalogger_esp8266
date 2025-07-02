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
    multiplexerPresent = false;
  } else {
    multiplexerPresent = true;
  }
  // Additionally check if the multiplexer was detected
  if (! mp.isConnected()) {
    multiplexerPresent = false;
  } else {
    multiplexerPresent = true;
  }
  
  // Now tell the calling function if the initialisation was successful
  if(!multiplexerPresent){
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

/*
  _channels = mp.channelCount();
  Serial.print("CHAN:\t");
  Serial.println(mp.channelCount());

  Serial.print("MASK:\t");
  Serial.println(mp.getChannelMask(), HEX);
  for (int chan = 0; chan < _channels; chan++)
  {
    Serial.print("PRE:\t");
    Serial.print(mp.isEnabled(chan));
    mp.enableChannel(chan);
    Serial.print("\t");
    Serial.println(mp.isEnabled(chan));
    delay(100);
  }
  Serial.println();
  mp.setChannelMask(0x00);

  Serial.print("MASK:\t");
  Serial.println(mp.getChannelMask(), HEX);
  for (int chan = 0; chan < _channels; chan++)
  {
    mp.enableChannel(chan);

    Serial.print("MASK:\t");
    Serial.println(mp.getChannelMask(), HEX);
    delay(100);
  }
  for (int chan = 0; chan < _channels; chan++)
  {
    mp.disableChannel(chan);
    Serial.print("MASK:\t");
    Serial.println(mp.getChannelMask(), HEX);
    delay(100);
  }
  Serial.println();
*/
