/*
 * ADS1115 Analog-to-digital converter
 * https://github.com/RobTillaart/ADS1X15
 */

 
#include "Arduino.h"
#include "ADS1115.h"

#include "ADS1X15.h"
ADS1115 ads_board(0x48);

ADS::ADS(
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
void ADS::setMultiplexer(MULTI* mux, uint8_t bus) {
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
    if(mux) mux->enableBus(bus);
  }
  ~BusGuard() {
    if(mux) mux->disableBus(bus);
  }
};
#endif

bool ADS::init(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  if(!ads_board.begin()){
    error_status = 1; // Initialisation failed
    sensorPresent = false;
    return(false);
  } else {
    error_status = 0;
    sensorPresent = true;
    ads_board.setGain(16); // Gain 16: ±0.256V
    ads_board.readADC(1);
    return(true);
  }
}

double ADS::read_mV(int port, int gain){
  if(sensorPresent == false){
    return(float(NAN));
  }
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  int16_t val = 0;
  // Gain 16: ±0.256V
  ads_board.setGain(gain);

	// Read data
	val = ads_board.readADC(port);
	double f = ads_board.toVoltage();  // voltage factor

  // Convert to mV
  double mV = val * f * 1000;
  error_status = (mV < 0) ? 2 : 0; // Make error_status 2 when values were <0, otherwise 0
  // Force negative values to be 0
  mV = (mV <= 0) ? 0.0 : mV; // If smaller than 0, make the value 0
	
	return(mV);
}

double ADS::read_val(int port, int gain, int conversion){
  double mV = read_mV(port, gain);
  // convert to actual value and return
  return(mV * conversion);
}

void  ADS::nothing(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

	// Nothing
  Serial.print("Mode (0=cont): "); Serial.println(ads_board.getMode());
}
