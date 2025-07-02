/*
 * SEN0465 O2 sensor
 */
 
#include "Arduino.h"
#include "SEN0465.h"

//SEN0465::SEN0465(void){}
SEN0465::SEN0465(TwoWire &wire) : sen(&wire){}

bool SEN0465::init(uint8_t addr){
  // Initialise, but disable autocalibration
  if (! sen.begin()) {
    // Valid SEN0465 sensor NOT found, check wiring!
    sensorPresent = false;
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
    return("NA");
  }
  return(sen.queryGasType());
}

// % vol (or ppm)
float SEN0465::airO2(void){
  float O2 = 0.0;
  O2 = sen.readGasConcentrationPPM();
  if(!sensorPresent){
    return(float(NAN));
  }
  return(O2);
}

// Temperature [°C]
float SEN0465::airT(void){
  float T = 0.0;
  if(!sensorPresent){
    return(float(NAN));
  } else {
    T = sen.readTempC();
    return(T);
  }
}

// Raw sensor voltage
float SEN0465::rawV(void){
  float V = 0.0;
  if(!sensorPresent){
    return(float(NAN));
  } else {
    V = sen.getSensorVoltage();
    return(V);
  }
}
