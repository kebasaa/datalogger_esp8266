/*
  Calibration routines for sensors. 
*/

#include "Calibration.h"
#include "h4_wrapper.h"
#include <cmath>

Cal::Cal(){
  // Nothing to be done here
}

bool Cal::is_near_zero(float x, float eps){
    return std::fabs(x) <= eps;
}

// Create all calibration variables in persistent storage
void Cal::init_all_calibrations(std::vector<String> gases, int numSensors) {
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 1; sensor <= numSensors; sensor++) {
        for (auto& calType : _calTypes) {
          String var_name = "";
          if(calType == "diff"){
            var_name = "sen_" + gas + "_" + String(sensor) + "_" + calType;
          } else {
            var_name = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          }
          create_var(var_name, 0.0);
          Serial.print("Creating variable "); Serial.println(var_name); //DEBUG
        }
      }
    }
  }
}

int Cal::set_calibration_coeff(String calType, String currentGas, int currentSensor, float ref, float measured){
  // Save the variable to persistent storage
  String var_reference = "ref_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Actual real zero value
  String var_measured =  "sen_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Sensor measurement for the zero
  // Update the values
  Serial.print("var: "); Serial.print(var_reference); Serial.print(" - "); Serial.println(ref);
  Serial.print("mes: "); Serial.print(var_reference); Serial.print(" - "); Serial.println(measured);
  update_var(var_reference, ref);
  update_var(var_measured, measured);
  return(0);
}

float Cal::read_calibration_var(String dataType, String calType, String currentGas, int currentSensor){
  // Read only sensor measurement values from persistent storage. We assume that reference values are known
  // dataType is either "sen" or "ref"
  if((dataType != "sen") & (dataType != "ref")){
    return(float(NAN));
  }
  // calType is either "zero" or "span"
  if((calType != "zero") & (calType != "span")){
    return(float(NAN));
  }
  if((currentGas == "All") | (currentSensor == 0)){
    return(float(NAN));
  }
  //Serial.print("Var name read cal: "); Serial.println(dataType + "_" + currentGas + "_" + String(currentSensor) + "_" + calType);
  // E.g., sen_h2o_1_zero
  float var = read_var(dataType + "_" + currentGas + "_" + String(currentSensor) + "_" + calType);
  return(var);
}

float Cal::calibrate_linear(String currentGas, int currentSensor, float currentMeasurement){
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span");
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span");

  // Catch potential errors:
  // Check if there is a slope at all
  if(is_near_zero(var_sen_span - var_sen_zero)){
    return(float(NAN));
  }
  // Check if the span variables are still 0 (typically after factory reset)
  if(is_near_zero(var_sen_span) | is_near_zero(var_ref_span)){
    return(float(NAN));
  }
  // Check if the zero sensor value was measured
  if(var_sen_zero == 0){
    return(float(NAN));
  }

  float gain = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  float offset = var_ref_zero - gain * var_sen_zero;
  // apply calibration
  float currentCalibrated = gain * currentMeasurement + offset;

  //float currentCalibrated = var_ref_zero + (currentMeasurement - var_sen_zero) * (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  return(currentCalibrated);
}

float Cal::calibrate_differential(String currentGas, float currentMeasurement){
  // Get calibration variables
  float var_diff_sen1 = read_var("sen_" + currentGas + "_1_diff");
  float var_diff_sen2 = read_var("sen_" + currentGas + "_2_diff");

  // Catch potential errors:
  if((var_diff_sen1 == 0) | (var_diff_sen2 == 0)){
    return(float(NAN));
  }

  float offset = var_diff_sen1 - var_diff_sen2;

  // apply calibration
  float currentCalibrated = currentMeasurement + offset;

  return(currentCalibrated);
}

// Read persistently stored variable
float Cal::read_var(String var_type){
  std::string var_type_std = std::string(var_type.c_str());
  if(h4_gvExists(var_type_std.c_str())){
    int val_int = h4_gvGetInt(var_type_std.c_str());
    if(val_int == -9999){
      return(float(NAN));
    } else {
      return(val_int / 100000.0); // In order to preserve decimals, restore from larger value stored as int)
    }
  } else {
    return(float(NAN));
  }
}

float Cal::read_differential_var(String currentGas, int currentSensor){
  // Read only sensor measurement values from persistent storage. We assume that reference values are known
  float var = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff");
  return(var);
}

int Cal::set_diff(String currentGas, float sen1_measurement, float sen2_measurement){
  // Save the variable to persistent storage
  String var_sen1 = "sen_" + currentGas + "_1_diff";
  String var_sen2 = "sen_" + currentGas + "_2_diff";
  // Update the values
  create_var(var_sen1, sen1_measurement);
  create_var(var_sen2, sen2_measurement);
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
    Serial.print(var_type); Serial.println(" doesn't exist");
    if(isnan(val)){
      h4_gvSetInt(var_type_str.c_str(), -9999, true);
    } else {
      h4_gvSetInt(var_type_str.c_str(), val_int, true);
    }
  } else {
    float var = read_var(var_type);
    Serial.print(var_type); Serial.print(" exists: "); Serial.println(var);
  }
}

void Cal::update_var(String var_type, float val){
  // Convert string type
  std::string var_type_str = std::string(var_type.c_str());
  // Convert float to int
  int32_t val_int = static_cast<int32_t>(lroundf(val * 100000.0f));
  // Check if value exists, if not create it. This check is important to avoid overwriting existing data
  if(isnan(val)){
    h4_gvSetInt(var_type_str.c_str(), -9999, true);
  } else {
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


float Cal::read_linear_slope(String currentGas, int currentSensor, float currentMeasurement){
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span");
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span");

  // Catch potential errors:
  // Check if there is a slope at all
  if(is_near_zero(var_sen_span - var_sen_zero)){
    return(float(NAN));
  }
  // Check if the span variables are still 0 (typically after factory reset)
  if(is_near_zero(var_sen_span) | is_near_zero(var_ref_span)){
    return(float(NAN));
  }
  // Check if the zero sensor value was measured
  if(var_sen_zero == 0){
    return(float(NAN));
  }

  // The gain is the slope
  float slope = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);

  //float currentCalibrated = var_ref_zero + (currentMeasurement - var_sen_zero) * (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  return(slope);
}

float Cal::read_linear_intercept(String currentGas, int currentSensor, float currentMeasurement){
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span");
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span");

  // Catch potential errors:
  // Check if there is a slope at all
  if(is_near_zero(var_sen_span - var_sen_zero)){
    return(float(NAN));
  }
  // Check if the span variables are still 0 (typically after factory reset)
  if(is_near_zero(var_sen_span) | is_near_zero(var_ref_span)){
    return(float(NAN));
  }
  // Check if the zero sensor value was measured TODO CHECK
  if(var_sen_zero == 0){
    return(float(NAN));
  }

  // The offset is the intercept
  float gain = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  float intercept = var_ref_zero - gain * var_sen_zero;
  
  return(intercept);
}

// Cycles through all calibration variables and adds their names into a single string
String Cal::get_all_cal_header(std::vector<String> gases, int numSensors){
  String cal_header = "";
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 1; sensor <= numSensors; sensor++) {
        for (auto& calType : _calTypes) {
          if((dataType == "sen") & (calType == "diff")){
            cal_header += "sen_" + gas + "_" + String(sensor) + "_" + calType + ",";
          } else if((dataType == "ref") & (calType == "diff")){
            // There is no reference for differential calibrations
          } else {
            cal_header += dataType + "_" + gas + "_" + String(sensor) + "_" + calType + ",";
          }
        }
      }
    }
  }
  return(cal_header);
}

// Cycles through all calibration variables and adds their values into a single string
String Cal::get_all_cal_data(std::vector<String> gases, int numSensors){
  String cal_data = "";
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 1; sensor <= numSensors; sensor++) {
        for (auto& calType : _calTypes) {
          if((dataType == "sen") & (calType == "diff")){
            cal_data += String(read_var("sen_" + gas + "_" + String(sensor) + "_" + calType)) + ",";
          } else if((dataType == "ref") & (calType == "diff")){
            // There is no reference for differential calibrations
          } else {
            cal_data += String(read_var(dataType + "_" + gas + "_" + String(sensor) + "_" + calType)) + ",";
          }
        }
      }
    }
  }
  return(cal_data);
}
