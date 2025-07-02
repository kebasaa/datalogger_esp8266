/*
 * GPS sensor
 */
 
#include "Arduino.h"
#include "GPS.h"

I2CGPS myI2CGPS;

TinyGPSPlus gps_interpreter;

GPS::GPS(void){
  // GPS
}

bool GPS::init(){
  if(!myI2CGPS.begin()){
    return(false);
  } else {
    return(true);
  }
}

void GPS::update_values(void){
  while(myI2CGPS.available()){
    gps_interpreter.encode(myI2CGPS.read());
  }
}

String GPS::get_timestamp(void){
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
  String timestamp = "";

  timestamp += String(gps_interpreter.date.year());
  if (gps_interpreter.date.month() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.date.month());
  if (gps_interpreter.date.day() < 10) timestamp += "0";
  timestamp += String(gps_interpreter.date.day());
  
  return(timestamp);
}


String GPS::get_location(void){
  String loc = "";

  if(gps_interpreter.location.isValid()){
    loc += String(gps_interpreter.location.lat(),8) + ",";
    loc += String(gps_interpreter.location.lng(),8) + ",";
  }else{
    loc += ",,";
  }

  if(gps_interpreter.altitude.isValid()){
    loc += String(gps_interpreter.altitude.meters(),2) + ",";
  }else{
    loc += ",";
  }

  loc += String(gps_interpreter.satellites.value()) + ","; // Number of satellites in view
  loc += String(gps_interpreter.hdop.value()/100.0, 2);    // HDOP
  
  return(loc);
}

float GPS::get_lat(void){
  float latitude = 0;
  if (gps_interpreter.location.isValid()){
    latitude = gps_interpreter.location.lat();
  }else{
    float latitude = float(NAN);
  }
  return(latitude);
}

float GPS::get_lon(void){
  float longitude = 0;
  if (gps_interpreter.location.isValid()){
    longitude = gps_interpreter.location.lng();
  }else{
    float longitude = float(NAN);
  }
  return(longitude);
}

float GPS::get_alt(void){
  float alt = 0;
  if (gps_interpreter.altitude.isValid()){
    alt = gps_interpreter.altitude.meters();
  }else{
    float alt = float(NAN);
  }
  return(alt);
}
