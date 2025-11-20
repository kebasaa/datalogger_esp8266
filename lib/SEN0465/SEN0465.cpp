/*
 * SEN0465 O2 sensor
 */
 
#include "Arduino.h"
#include "SEN0465.h"

//SEN0465::SEN0465(TwoWire &wire) : sen(&wire){}

SEN0465::SEN0465(
#if I2C_MULTI
  MULTI* mux, uint8_t bus,
#endif
  TwoWire &wire) : sen(&wire){
#if I2C_MULTI
  _mux = mux;
  _mux_bus = bus;
#endif
}

#if I2C_MULTI
void SEN0465::setMultiplexer(MULTI* mux, uint8_t bus) {
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

bool SEN0465::init(uint8_t addr){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  // Initialise, but disable autocalibration
  if (! sen.begin()) {
    // Valid SEN0465 sensor NOT found, check wiring!
    sensorPresent = false;
    error_status = 1; // Initialisation failed
    return(false);
  } else {
    // Mode of obtaining data: the main controller needs to request the sensor for data
    sen.changeAcquireMode(sen.PASSIVITY);
    // Turn on temperature compensation (sen.ON vs. sen.OFF)
    sen.setTempCompensation(sen.ON);
    // Specify that the sensor initialisation worked
    sensorPresent = true;
    return(true);
  }
}

String SEN0465::gasType(void){
  if(!sensorPresent){
    return("nan");
  }
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  return(sen.queryGasType());
}

// % vol (or ppm)
float SEN0465::airO2(void){
  if(!sensorPresent){
    return(float(NAN));
  }

#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  // The O2 sensor reports in %vol
  // To convert to mmol/mol, multiply by 10. To convert to ppm, multiply by 10000
  float O2 = 0.0;
  O2 = sen.readGasConcentrationPPM() * 10;
  return(O2);
}

// Temperature [°C]
float SEN0465::airT(void){
  if(!sensorPresent){
    return(float(NAN));
  }

#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  float T = 0.0;
  T = sen.readTempC();
  return(T);
}

// Raw sensor voltage
float SEN0465::rawV(void){
  if(!sensorPresent){
    return(float(NAN));
  }

#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  float V = 0.0;
  V = sen.getSensorVoltage();
  return(V);
}
