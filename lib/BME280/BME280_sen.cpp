/*
  Read data from the BME280 sensor
*/

#include "BME280_sen.h"

#define SEALEVELPRESSURE_HPA (1013.15)

BME::BME(
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
void BME::setMultiplexer(MULTI* mux, uint8_t bus) {
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
    //if(mux) Serial.print("Enablebus (BME ");Serial.print(bus);Serial.print(")! ");
    if(mux) mux->enableBus(bus);
  }
  ~BusGuard() {
    //if(mux) Serial.print("Disablebus (BME ");Serial.print(bus);Serial.print(")! ");
    delay(5);
    if(mux) mux->disableBus(bus);
  }
};
#endif

bool BME::init(){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif

  if (! _bme.begin()) {
    sensorPresent = false;
    error_status = 1; // Initialisation failed
  } else {
    sensorPresent = true;
  }
  
  /*
  _bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X2, // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);
  */
  // Actually recommended for weather:
  /*_bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF); // NOTE: suggested sampling rate: 1min
  */
  // Recommended for high pressure accuracy:
  // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  /*_bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X2, // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X16, 
                  Adafruit_BME280::STANDBY_MS_0_5 ); // NOTE: suggested sampling rate: a lot more*/
  
  // Now tell the calling function if the initialisation was successful
  if(!sensorPresent){
    return(false);
  } else {
    return(true);
  }
}

void BME::update_pressure_cal(float cal){
  // Store calibration values
  _pressure_cal = cal; // Offset value in [Pa]
}

void BME::update_temperature_cal(float cal1, float cal2){
  // Based on T = -0.0075 * pow(T,2) + 1.1291 * T;
  // No correction would have cal1 = 0 and cal2 = 1
  _temperature_cal1 = cal1; // -0.0075
  _temperature_cal2 = cal2; // 1.1291
}


void  BME::update_measurements(int time_delay, int freq){
  //
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  // Number of data-points
  int max_measurements = time_delay / freq;
  for(int i = 1; i <= max_measurements; i++){
    _bme.read(current_pressure, current_temperature, current_humidity, tempUnit, presUnit);
    delay(freq);
  }
}

float BME::airT(void){
  if(! sensorPresent) return float(NAN);
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  float T = 0.0;
  // Temperature [°C]
  T = _bme.temp();
  // Temperature correction: apply calibration curve
  //T = -0.0075 * pow(T,2) + 1.1291 * T;
  T = _temperature_cal1 * pow(T,2) + _temperature_cal2 * T;
  return(T);
}

float BME::airRH(void){
  // relative humidity [%]
  if(! sensorPresent) return float(NAN);
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  return(_bme.hum());
}

float BME::airP(void){
  // Pressure [Pa] (This makes sure that the unit is Pa)
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  if(! sensorPresent) return float(NAN);
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  return(_bme.pres(presUnit) + _pressure_cal); // correction factor due to sensor inaccuracy, sea level calibration 584 Pa
}
