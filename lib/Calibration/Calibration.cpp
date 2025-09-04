/*
  Calibration routines for sensors. 
*/

#include "Calibration.h"
#include "h4_wrapper.h"
#include <cmath>

Cal::Cal(){}

// Create all calibration variables in persistent storage
void Cal::init_all_calibrations(std::vector<String> gases, int numSensors) {
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 1; sensor <= numSensors; sensor++) {
        for (auto& calType : _calTypes) {
          String var_name = "";
          if(calType == "diff"){
            var_name += dataType + "_" + gas + "_" + calType;
          } else {
            var_name += dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          }
          create_var(var_name, 0.0);
          Serial.print("Creating variable "); Serial.println(var_name); //DEBUG
        }
      }
    }
  }
}

String Cal::get_cal_header(std::vector<String> gases, int numSensors){
  String cal_header = "";
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 1; sensor <= numSensors; sensor++) {
        for (auto& calType : _calTypes) {
          if(calType == "diff"){
            cal_header = dataType + "_" + gas + "_" + calType;
          } else {
            cal_header = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;
          }
          Serial.print("Calibration file header: "); Serial.println(cal_header); //DEBUG
        }
      }
    }
  }
  return(cal_header);
}

int Cal::set_zero(String currentGas, int currentSensor, float zero_ref, float zero_measured){
  // DO SOMETHING
  return(0);
}
int Cal::set_span(String currentGas, int currentSensor, float zero_ref, float zero_measured){
  // DO SOMETHING
  return(0);
}
int Cal::set_diff(String currentGas, float sen1_measured, float sen2_measured){
  // DO SOMETHING
  return(0);
}

// Persistent storage of variable, creates it if it doesn't exist or updates the value
// - Can be something like "h2o_zero", or "o2_span" or similar
// - Should get initialised with 0.0
// - E.g., save_var("h2o_zero");
void Cal::create_var(String var_type, float val){
  // Convert string type
  std::string var_type_str = std::string(var_type.c_str());
  // Convert float to int
  int32_t val_int = static_cast<int32_t>(lroundf(val * 100000.0f));
  // Check if value exists, if not create it. This check is important to avoid overwriting existing data
  if(!h4_gvExists(var_type_str.c_str())){
    h4_gvSetInt(var_type_str.c_str(), val_int, true);
  }
}


// Note: Calibration values need to be saved at every calibration event
// Calibration involves measuring before calibration, then changing values, then measuring again

// 4 calibrations are necessary:
// - Zero (known value and measured). The known zero is in case we don't have absolute zero but just use a low value
// - Span (known value and measured)
// - Equal values of same sensors on each bus? This is not necessary if the above calibration was done in the same reference gas,
//   but it is if no absolute calibration is possible...

// A digital button on wifi is necessary for each calibration method and each sensor separately
// E.g., BME280 has one calibration, SCD-30 has one too
// Sometimes, cross-calibration of values is possible, e.g., air temperature (most sensors) or RH (BME280, SCD-30)

// Calibration values have to be stored both on the SD card, and in permanent memory
// There should be a "factory reset" button



// Persistent storage of variable, creates it if it doesn't exist or updates the value
// - Can be something like "h2o_zero", or "o2_span" or similar
// - E.g., save_var("h2o_zero", 0.0032);
// For each gas/parameter, there are 4 values to store, e.g.:
// - co2_zero_mes
// - co2_zero_ref
// - co2_span_mes
// - co2_span_ref
/*void Cal::update_var(String var_type, float val){
  std::string var_type_str = std::string(var_type.c_str());
  int32_t val_int = static_cast<int32_t>(lroundf(val * 100000.0f));
  h4_gvSetInt(var_type_str.c_str(), val_int, true);
}*/

// Read persistently stored variable
float Cal::read_var(String var_type){
  std::string var_type_std = std::string(var_type.c_str());
  if(h4_gvExists(var_type_std.c_str())){
    int val_int = h4_gvGetInt(var_type_std.c_str());
    return(val_int / 100000.0); // In order to preserve decimals, restore from larger value stored as int)
  } else {
    return(NAN);
  }
}

float Cal::calibrate_measurement(String var_type, float measurement_raw){
  float zero_mes = read_var(var_type + "zero_mes"); // E.g., read "h2o_zero_mes"
  float zero_ref = read_var(var_type + "zero_ref");
  float span_mes = read_var(var_type + "span_mes");
  float span_ref = read_var(var_type + "span_ref");

  float gain = (span_ref - zero_ref) / (span_mes - zero_mes);
  float measurement_corrected = (measurement_raw - zero_ref) * gain + zero_mes;

  return(measurement_corrected);
}

/*
void something(void){
  // Add calibration routine for Serial port commands
  h4p.addCmd(
  "cal",
  0,                    
  [&](const std::string& args){
    // Parse: <type> <index> <mode> <ref>
    std::istringstream iss(args);
    std::string type, mode;
    int idx;
    double ref;
    if(!(iss >> type >> idx >> mode >> ref)) {
      Serial.println(F("Usage: cal <co2|o2|h2o> <1|2> <zero|span> <value>"));
      return;
    }
    // Lowercase normalize
    for(auto &c:type)  c = tolower(c);
    for(auto &c:mode)  c = tolower(c);

    // Get sensor reading
    double meas = 0;
    if(type=="co2"){
      meas = scd.airCO2();     // ppm
    }
    else if(type=="o2"){
      meas = sen.airO2();      // ppm
    }
    else if(type=="h2o"){
      meas = bme.airRH();      // %RH
    }
    else {
      Serial.println(F("Unknown sensor; must be co2, o2, or h2o"));
      return;
    }

    // Echo to user
    Serial.printf(
      "CAL %s %d %s: ref=%.2f, meas=%.2f\n",
      type.c_str(), idx, mode.c_str(), ref, meas
    );

    // Build tags and store both reference and measurement
    char tagRef[32], tagMeas[32];
    snprintf(tagRef,  sizeof(tagRef),  "cal_%s_%d_%s_ref",  type.c_str(), idx, mode.c_str());
    snprintf(tagMeas, sizeof(tagMeas), "cal_%s_%d_%s_meas", type.c_str(), idx, mode.c_str());

    h4p.gvSet(tagRef,  ref);
    h4p.gvSet(tagMeas, meas);

    Serial.printf("Stored [%s]=%.2f and [%s]=%.2f\n",
                  tagRef,  h4p.gvGetDouble(tagRef),
                  tagMeas, h4p.gvGetDouble(tagMeas));
  }
);



}
*/