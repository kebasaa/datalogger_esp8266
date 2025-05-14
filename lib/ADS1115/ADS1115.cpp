/*
 * ADS1115 Analog-to-digital converter
 * https://github.com/RobTillaart/ADS1X15
 */

 
#include "Arduino.h"
#include "ADS1115.h"

#include "ADS1X15.h"
ADS1115 ads_board(0x48);

ADS::ADS(void){
  // ADS
}

bool ADS::init(void){
  if(!ads_board.begin()){
    return(false);
  } else {
    ads_board.setGain(16); // Gain 16: ±0.256V
    ads_board.readADC(1);
    return(true);
  }
}

double ADS::read_mV(int port, int gain){
  int16_t val = 0;
  // Gain 16: ±0.256V
  ads_board.setGain(gain);

	// Read data
	val = ads_board.readADC(port);
	double f = ads_board.toVoltage();  // voltage factor
  //Serial.print(val); Serial.print("\t");
  //Serial.print(f,6); Serial.print("\t");
  //Serial.println(mV,6);

  // Convert to mV
  double mV = val * f * 1000;
  // Negative values should be made 0
  mV = (mV <= 0) ? 0.0 : mV; // If smaller than 0, make the value 0
	
	return(mV);
}

double ADS::read_val(int port, int gain, int conversion){
  double mV = read_mV(port, gain);
  // convert to actual value and return
  return(mV * conversion);
}

void  ADS::nothing(void){
	// Nothing
  Serial.print("Mode (0=cont): "); Serial.println(ads_board.getMode());
}
