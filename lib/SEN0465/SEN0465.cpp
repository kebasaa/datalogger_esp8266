/*
 * SEN0465 O2 sensor
 */
 
#include "Arduino.h"
#include "SEN0465.h"

SCD::SCD(void){
  // SCD
}

bool SCD::init(void){
  // Initialise, but disable autocalibration
  if (! o2sensor.begin(Wire, false)) {
    // Valid SCD-30 sensor NOT found, check wiring!
    sensorPresent = false;
    return(false);
  } else {
    sensorPresent = true;
    return(true);
  }
}

float SCD::airT(void){
  float T = 0;
  T = o2sensor.getTemperature();
  if(!sensorPresent){
    return(float(NAN));
  }
  return(T);
}

float SCD::airRH(void){
  float RH = 0;
  RH = o2sensor.getHumidity();
  if(!sensorPresent){
    return(float(NAN));
  }
  return(RH);
}

float SCD::airCO2(void){
  float CO2 = 0.0;
  CO2 = o2sensor.getCO2();
  if(!sensorPresent){
    return(float(NAN));
  }
  return(CO2);
}

void SCD::set_interval(int interval){
	// Set measurement interval in s
  if(!sensorPresent){
    o2sensor.setMeasurementInterval(interval);
    delay(200); // When changing settings, the sensor isn't ready right away
    // delay commands are bad in async programming
  }
}

void SCD::set_air_pressure(float pressure_Pa){
	  // Set air pressure in mBar or hPa
    if(!sensorPresent){
      o2sensor.setAmbientPressure(pressure_Pa/100);
    }
}

void SCD::read_calibration_value(void){
	  uint16_t settingVal;

    if(!sensorPresent){
      if (o2sensor.getForcedRecalibration(&settingVal) == true){
        Serial.print(F("Forced recalibration factor (ppm) is "));
        Serial.println(settingVal);
      } else {
        Serial.print(F("getForcedRecalibration failed!"));
      }
    }
}

bool SCD::calibrate_with_reference(uint16_t reference_gas){
  if(!sensorPresent){
    if (!o2sensor.setForcedRecalibrationFactor(reference_gas)){
      Serial.println("Failed to force recalibration with reference");
      return(false);
    }
  }
  return(true);
}

void SCD::enable_self_calibration(bool enable){
  if(!sensorPresent){
    o2sensor.setAutoSelfCalibration(enable);
  }
}
