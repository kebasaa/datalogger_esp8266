/*
 * SCD CO2 sensor
 */
 
#include "Arduino.h"
#include "SCD30.h"

#include <Wire.h>
#include <SparkFun_SCD30_Arduino_Library.h>

SCD::SCD(
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
void SCD::setMultiplexer(MULTI* mux, uint8_t bus) {
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

bool SCD::init(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  // Initialise, but disable autocalibration
  if (! co2sensor.begin(Wire, false)) {
    // Valid SCD-30 sensor NOT found, check wiring!
    sensorPresent = false;
    error_status = 1; // Initialisation failed
    return(false);
  } else {
    sensorPresent = true;
    return(true);
  }
}

float SCD::airT(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(!sensorPresent){
    return(float(NAN));
  }
  
  if(co2sensor.dataAvailable()){
    t_value = co2sensor.getTemperature();
  }
  return(t_value);
}

float SCD::airRH(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(!sensorPresent){
    return(float(NAN));
  }
  
  if(co2sensor.dataAvailable()){
    rh_value = co2sensor.getHumidity();
  }
  return(rh_value);
}

float SCD::airCO2(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(!sensorPresent){
    return(float(NAN));
  }
  
  if(co2sensor.dataAvailable()){
    co2_value = co2sensor.getCO2();
  }
  return(co2_value);
}

void SCD::set_interval(int interval){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
	// Set measurement interval in s
  if(sensorPresent){
    co2sensor.setMeasurementInterval(interval);
    //delay(200); // When changing settings, the sensor isn't ready right away
    // Note: delay commands are bad in async programming, and measurements don't start right away, i.e. this is not strictly needed
  }
}

void SCD::set_air_pressure(float pressure_Pa){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
	// Set air pressure in mBar or hPa
  if(sensorPresent){
    co2sensor.setAmbientPressure(pressure_Pa/100);
  }
}

void SCD::read_calibration_value(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
	  uint16_t settingVal;

    if(sensorPresent){
      if (co2sensor.getForcedRecalibration(&settingVal) == true){
        Serial.print(F("Forced recalibration factor (ppm) is "));
        Serial.println(settingVal);
      } else {
        Serial.print(F("getForcedRecalibration failed!"));
      }
    }
}

bool SCD::calibrate_with_reference(uint16_t reference_gas){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(sensorPresent){
    if (!co2sensor.setForcedRecalibrationFactor(reference_gas)){
      Serial.println("Failed to force recalibration with reference");
      return(false);
    }
  }
  return(true);
}

void SCD::enable_self_calibration(bool enable){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(sensorPresent){
    co2sensor.setAutoSelfCalibration(enable);
  }
}
