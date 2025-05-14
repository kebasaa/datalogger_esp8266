/*
 * Battery reading
 */

#include "bat.h"

Battery::Battery(void){
  // Battery
}

bool Battery::init(void){
  pinMode(analogPin, INPUT);

  return(is_battery_present());
}

bool Battery::is_battery_present(void){
  unsigned int raw = 0;
  // Test presence of a battery
  raw = analogRead(analogPin);
  if(raw < 250){
    batteryPresent = false;
    return(false);
  } else {
    batteryPresent = true;
    return(true);
  }
}
    
float Battery::battery_mV(void){
  unsigned int raw = 0;
  float bat_mV = 0.0;
	
	// Read raw data
	raw = analogRead(analogPin);
	// Convert to mV
  bat_mV = raw / 1024.0 * 4200;

  is_battery_present();
	if(! batteryPresent){
	  bat_mV = NAN;
  }
	
	return(bat_mV);
}

float Battery::battery_pc(void){
	float bat_mV = 0.0, bat_perc = 0.0;
	bat_mV = battery_mV();
	
	// Convert mV to %
	bat_perc = (bat_mV-3000)*100/1200; // 4200 mV is 100%, 3000 mV is 0%
  
	return(bat_perc);
}
