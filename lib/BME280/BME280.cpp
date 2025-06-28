/*
  Read data from the BME280 sensor
*/

#include "BME280.h"

#define SEALEVELPRESSURE_HPA (1013.15)

//Adafruit_BME280 bme_sensor;

BME::BME(){
  // Nothing to be done here
}

bool BME::init(byte addr){
  if (! bme_sensor.begin(addr)) {
    Serial.printf("BME280 0x%02X NOT found", addr);
    sensorPresent = false;
  } else {
    sensorPresent = true;
  }
  
  /*
  bme_sensor.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X2, // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);
  */
  // Actually recommended for weather:
  /*bme_sensor.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF); // NOTE: suggested sampling rate: 1min
  */
  // Recommended for high pressure accuracy:
  bme_sensor.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X2, // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X16, 
                  Adafruit_BME280::STANDBY_MS_0_5 ); // NOTE: suggested sampling rate: a lot more*/
  
  // Take some initial measurements
  for(int i = 1; i < 100; i++){
    bme_sensor.takeForcedMeasurement();
    delay(50);
  }
  
  starting_altitude = bme_sensor.readAltitude(SEALEVELPRESSURE_HPA);
  
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
  // Number of data-points
  int max_measurements = time_delay / freq;
  for(int i = 1; i <= max_measurements; i++){
    bme_sensor.takeForcedMeasurement();
    delay(freq);
  }
}

float BME::airT(void){
  
  float T = 0.0;
  // Temperature [°C]
  if(! sensorPresent){
    return(float(NAN));
  } else {
    bme_sensor.takeForcedMeasurement(); // has no effect in normal mode
    // temperature correction
    T = bme_sensor.readTemperature();
    // Apply calibration curve
    //T = -0.0075 * pow(T,2) + 1.1291 * T;
    T = _temperature_cal1 * pow(T,2) + _temperature_cal2 * T;
    return(T);
  }
}

float BME::airRH(void){
  // relative humidity [%]
  if(! sensorPresent){
    return(float(NAN));
  } else {
    return(bme_sensor.readHumidity());
  }
}

float BME::airP(void){
  // Pressure [Pa]
  if(! sensorPresent){
    return(float(NAN));
  } else {
    return(bme_sensor.readPressure() + _pressure_cal); // correction factor due to sensor inaccuracy, sea level calibration 584 Pa
  }
}

float BME::altitude_asl(void){
  // Altitude [m]
  if(! sensorPresent){
    return(float(NAN));
  } else {
    return(bme_sensor.readAltitude(SEALEVELPRESSURE_HPA + _pressure_cal/100)); // correction factor in hPa
  }
}

float BME::altitude_agl(void){
  // Altitude [m]
  if(! sensorPresent){
    return(float(NAN));
  } else {
    return(bme_sensor.readAltitude(SEALEVELPRESSURE_HPA) - starting_altitude);
  }
}
