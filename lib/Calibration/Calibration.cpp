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
            cal_header += dataType + "_" + gas + "_" + calType;
          } else {
            cal_header += dataType + "_" + gas + "_" + String(sensor) + "_" + calType;
          }
        }
      }
    }
  }
  Serial.print("Calibration file header: "); Serial.println(cal_header); //DEBUG
  return(cal_header);
}


int Cal::set_calibration_coeff(String calType, String currentGas, int currentSensor, float zero_ref, float zero_measured){
  // Save the variable to persistent storage
  String var_reference = "ref_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Actual real zero value
  String var_measured =  "sen_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Sensor measurement for the zero
  // Update the values
  create_var(var_reference, zero_ref);
  create_var(var_measured, zero_measured);
  return(0);
}

float Cal::calibrate_zero_span(String currentGas, int currentSensor, float currentMeasurement){
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span");
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero");
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span");

  // Catch potential errors:
  // Check if there is a slope at all
  if(is_near_zero(var_sen_span - var_sen_zero)){
    return(NAN);
  }
  // Check if the span variables are still 0 (typically after factory reset)
  if(is_near_zero(var_sen_span) | is_near_zero(var_ref_span)){
    return(NAN);
  }
  // Check if the zero sensor value was measured
  if(var_sen_zero == 0){
    return(NAN);
  }

  float gain = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  float offset = var_ref_zero - gain * var_sen_zero;
  // apply calibration
  float currentCalibrated = gain * currentMeasurement + offset;

  //float currentCalibrated = var_ref_zero + (currentMeasurement - var_sen_zero) * (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  return currentCalibrated;
}

/*
int Cal::set_zero(String currentGas, int currentSensor, float zero_ref, float zero_measured){
  // Save the variable to persistent storage
  String var_zero_ref =      "ref_" + currentGas + "_" + String(currentSensor) + "_zero"; // Actual real zero value
  String var_zero_measured = "sen_" + currentGas + "_" + String(currentSensor) + "_zero"; // Sensor measurement for the zero
  // Update the values
  create_var(var_zero_ref, zero_ref);
  create_var(var_zero_measured, zero_measured);
  return(0);
}
int Cal::set_span(String currentGas, int currentSensor, float span_ref, float span_measured){
  // Save the variable to persistent storage
  String var_span_ref =      "ref_" + currentGas + "_" + String(currentSensor) + "_span"; // Actual real zero value
  String var_span_measured = "sen_" + currentGas + "_" + String(currentSensor) + "_span"; // Sensor measurement for the zero
  // Update the values
  create_var(var_span_ref, span_ref);
  create_var(var_span_measured, span_measured);
  return(0);
}
  */
int Cal::set_diff(String currentGas, float sen1_measured, float sen2_measured){
  // TODO: FUTURE WORK
  // Just store the measurements of sensor 1 & 2 separately. The difference is the measurement. Here we assume no linear correlation difference, though this could be added

  return(0);
}

// TODO
float Cal::diff_calibration(float a1, float b1,
                            float a2, float b2,
                            float sensor2_reading){

    // NOTE: This could be changed to storing slope & intercept
    float slope = 1.0f;     // default slope
    float intercept = 0.0f; // default intercept

    if (is_near_zero(a2) && is_near_zero(b2)) {
        // Offset calibration
        intercept = a1 - b1;
    } else {
        // Linear calibration when possible
        if (!is_near_zero(b2 - b1)) {
            slope = (a2 - a1) / (b2 - b1);
            intercept = a1 - slope * b1;
        } else if (is_near_zero(a2 - a1)) {
            intercept = a1 - b1; // If the slope is too small recur to simple offset
        } else {
            return NAN; // Other cases: invalid calibration
        }
    }

    return slope * sensor2_reading + intercept; // Calibrated output
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
