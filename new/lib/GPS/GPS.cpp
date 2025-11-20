/*
 * GPS sensor
 */
 
//#include "Arduino.h"
#include "GPS.h"

I2CGPS myI2CGPS;

TinyGPSPlus gps_interpreter;

GPS::GPS(
#if I2C_MULTI
  MULTI* mux, uint8_t bus
#endif
) {
#if I2C_MULTI
  _mux = mux;
  _mux_bus = bus;
#endif
}

#if I2C_MULTI
void GPS::setMultiplexer(MULTI* mux, uint8_t bus) {
  _mux = mux;
  _mux_bus = bus;
}
#endif

// Simple RAII guard to enable/disable a mux channel while in scope
#if I2C_MULTI
struct BusGuard {
  MULTI* mux;
  uint8_t bus;
  BusGuard(MULTI* m, uint8_t b) : mux(m), bus(b) {
    //if(mux) Serial.print("Enablebus! ");
    if(mux) mux->enableBus(bus);
  }
  ~BusGuard() {
    //if(mux) Serial.print("Disablebus! ");
    if(mux) mux->disableBus(bus);
  }
};
#endif

bool GPS::init(){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(!myI2CGPS.begin()){
    error_status = 1;
    return(false);
  } else {
    error_status = 0;
    return(true);
  }
}

void GPS::update_values(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  while(myI2CGPS.available()){
    gps_interpreter.encode(myI2CGPS.read());
  }
}

String GPS::get_timestamp(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  String timestamp = "";

  timestamp += String(gps_interpreter.date.year()) + "-";
  if (gps_interpreter.date.month() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.date.month()) + "-";
  if (gps_interpreter.date.day() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.date.day()) + " ";
  if (gps_interpreter.time.hour() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.time.hour()) + ":";
  if (gps_interpreter.time.minute() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.time.minute()) + ":";
  if (gps_interpreter.time.second() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.time.second()); //+ ".";
  //if (gps_interpreter.time.centisecond() < 10) timestamp += "0";
  //timestamp += String(gps_interpreter.time.centisecond());
  
  return(timestamp);
}

String GPS::get_date(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  String timestamp = "";

  timestamp += String(gps_interpreter.date.year());
  if (gps_interpreter.date.month() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.date.month());
  if (gps_interpreter.date.day() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.date.day());
  
  return(timestamp);
}


String GPS::get_location(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  String loc = "";

  if(gps_interpreter.location.isValid()){
    error_status = 0;
    loc += String(gps_interpreter.location.lat(),8) + ",";
    loc += String(gps_interpreter.location.lng(),8) + ",";
  }else{
    error_status = 2;
    loc += ",,";
  }

  if(gps_interpreter.altitude.isValid()){
    error_status = 0;
    loc += String(gps_interpreter.altitude.meters(),2) + ",";
  }else{
    error_status = 2;
    loc += ",";
  }

  loc += String(gps_interpreter.satellites.value()) + ","; // Number of satellites in view
  loc += String(gps_interpreter.hdop.value()/100.0, 2);    // HDOP
  
  return(loc);
}

String GPS::get_short_location(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  String loc = "";

  if(gps_interpreter.location.isValid()){
    error_status = 0;
    loc += String(gps_interpreter.location.lat(),4) + ",";
    loc += String(gps_interpreter.location.lng(),4) + ",";
    loc += String(gps_interpreter.altitude.meters(),0);
  }else{
    loc += "NO GPS SIGNAL";
  }
  
  return(loc);
}

float GPS::get_lat(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  float latitude = 0;
  if (gps_interpreter.location.isValid()){
    error_status = 0;
    latitude = gps_interpreter.location.lat();
  }else{
    error_status = 2;
    float latitude = float(NAN);
  }
  return(latitude);
}

float GPS::get_lon(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  float longitude = 0;
  if (gps_interpreter.location.isValid()){
    error_status = 0;
    longitude = gps_interpreter.location.lng();
  }else{
    error_status = 2;
    float longitude = float(NAN);
  }
  return(longitude);
}

float GPS::get_alt(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  float alt = 0;
  if (gps_interpreter.altitude.isValid()){
    error_status = 0;
    alt = gps_interpreter.altitude.meters();
  }else{
    error_status = 2;
    float alt = float(NAN);
  }
  return(alt);
}

unsigned long GPS::seconds_since_midnight(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if (gps_interpreter.time.isValid()){
    return gps_interpreter.time.hour() * 3600UL +
           gps_interpreter.time.minute() * 60UL +
           gps_interpreter.time.second();
  } else {
    return 0; // or some error flag
  }
}
