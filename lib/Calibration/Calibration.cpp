/*
  Calibration routines for sensors. 
*/

#include "Calibration.h"
#include "h4_wrapper.h"
#include <cmath>
#include "Utils.h"

Cal::Cal(){
  // Nothing to be done here
}

/*
bool Cal::is_near_zero(float x, float eps){
    return std::fabs(x) <= eps;
}*/

// Create all calibration variables in persistent storage
void Cal::init_all_calibrations(std::vector<String> gases, int numSensors) {
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 0; sensor < numSensors; sensor++) {
        for (auto& calType : calTypes) {
          // In the case of differential calibration, dataType == "ref" is low values and "sen" is high values
          String var_name = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          Serial.print("Creating variable "); Serial.print(var_name); Serial.print(": "); //DEBUG
          create_var(var_name, float(NAN));
        }
      }
    }
  }
}

void Cal::show_all_calibrations(std::vector<String> gases, int numSensors) {
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 0; sensor < numSensors; sensor++) {
        for (auto& calType : calTypes) {
          // In the case of differential calibration, dataType == "ref" is low values and "sen" is high values
          String var_name = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          Serial.print("  - "); Serial.print(var_name); Serial.print(": "); //DEBUG
          Serial.println(read_var(var_name));
        }
      }
    }
  }
}

// Reset all calibration variables in persistent storage
void Cal::reset_all_calibrations(std::vector<String> gases, int numSensors){
  for (auto& dataType : _dataTypes) {
    for (auto& gas : gases) {
      for (int sensor = 0; sensor < numSensors; sensor++) {
        for (auto& calType : calTypes) {
          // In the case of differential calibration, dataType == "ref" is low values and "sen" is high values
          String var_name = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          update_var(var_name, float(NAN));
          Serial.print("Resetting variable "); Serial.println(var_name); //DEBUG
        }
      }
    }
  }
}

int Cal::set_calibration_coeff(String calType, String currentGas, int currentSensor, float ref, float measured, unsigned long secs_since_midnight){
  // Save the variable to persistent storage
  String var_reference = "ref_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Actual real zero value
  String var_measured =  "sen_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Sensor measurement for the zero
  
  if(calType == "zero"){
    zero_cal_time_s = secs_since_midnight;
  }
  // Wrap around when time counting went across midnight (<0), wrap around
  if((secs_since_midnight - zero_cal_time_s) < 0){
    secs_since_midnight += 86400;
  }
  Serial.print("Secs since zero cal: "); Serial.println(secs_since_midnight - zero_cal_time_s);

  // Tests if too little or too much time has passed since zero calibration. For temperature we allow long time intervals
  if((calType == "span") && ((secs_since_midnight - zero_cal_time_s) > 7200) && (currentGas != "temperature")){
    Serial.println(">2h since zero calibration. Redo 0 calibration");
    return(1); // Error 1: Too much time since zero calibration
  } else if((calType == "span") && ((secs_since_midnight - zero_cal_time_s) < 10)){
    Serial.println("<10s since zero calibration. Are you sure the span gas is stable?");
    return(2); // Error 2: Too little time since zero calibration
  } else if (calType != "span"){
    Serial.println("Zero calibration:");
  } else {
    Serial.println("Span calibration:");
  }
  // Update the values
  Serial.print("    var: "); Serial.print(var_reference); Serial.print(" - "); Serial.println(ref);
  Serial.print("    mes: "); Serial.print(var_measured); Serial.print(" - "); Serial.println(measured);
  update_var(var_reference, ref);
  update_var(var_measured, measured);

  Serial.print("Setting: "); Serial.print(var_measured); Serial.print(" - "); Serial.println(var_reference);//DEBUG

  return(0);
}

float Cal::read_calibration_var(String dataType, String calType, String currentGas, int currentSensor){
  // Read only sensor measurement values from persistent storage. We assume that reference values are known
  // dataType is either "sen" or "ref"
  if((dataType != "sen") && (dataType != "ref")){
    return(float(NAN));
  }
  // calType is either "zero" or "span"
  if((calType != "zero") && (calType != "span")){
    return(float(NAN));
  }
  if((currentGas == "All") || (currentSensor == -9999)){
    return(float(NAN));
  }
  // E.g., sen_h2o_1_zero
  Serial.print("Reading: "); Serial.println(dataType + "_" + currentGas + "_" + String(currentSensor) + "_" + calType); //DEBUG
  float var = read_var(dataType + "_" + currentGas + "_" + String(currentSensor) + "_" + calType);
  return(var);
}

Cal::CalibrationCoeffs Cal::get_calibration_coefficients(String currentGas, int currentSensor){
  CalibrationCoeffs coeffs;
  
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero"); // Reference gas, zero
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span"); // Reference gas, span
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero"); // Sensor reading at zero
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span"); // Sensor reading at span

  if (std::isnan(var_sen_zero) || std::isnan(var_sen_span)){
    // Zero/Span not calibrated, differential only
    var_ref_zero = read_var("ref_" + currentGas + "_0_diff"); // Sensor 0 (the reference), low value
    var_ref_span = read_var("sen_" + currentGas + "_0_diff"); // Sensor 0 (the reference), high value
    var_sen_zero = read_var("ref_" + currentGas + "_1_diff"); // Sensor 1 (to be adjusted), low value
    var_sen_span = read_var("sen_" + currentGas + "_1_diff"); // Sensor 1 (to be adjusted), high value
    coeffs.flag = 1; // Differential only
  }

  // In a differential calibration, sensor 1 is not adjusted as it is the reference
  if((currentSensor == 0) && (std::isnan(var_sen_zero) || std::isnan(var_sen_span))){
    coeffs.flag   = 4; // Raw value returned
    coeffs.offset = 0;
    coeffs.gain   = 1;
    return(coeffs);
  }

  // Catch potential errors:
  // Check if there is a slope at all
  if(!std::isnan(var_sen_span) && Utils::is_near_zero(var_sen_span - var_sen_zero)){
    coeffs.flag   = -1; // Indicates an error, invalid data
    coeffs.offset = float(NAN);
    coeffs.gain   = float(NAN);
    return(coeffs);
  }

  coeffs.gain = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
  coeffs.offset = var_ref_zero - coeffs.gain * var_sen_zero;
  
  // Check if the zero variables are still NAN (typically after factory reset)
  if(std::isnan(var_ref_zero) || std::isnan(var_sen_zero)){
    coeffs.flag   = 4; // Indicates an error, bad data, return raw data
    coeffs.offset = 0;
    coeffs.gain   = 1;
    //return(coeffs);
  }
  
  // Simple offset calculation when the high differential isn't available
  if(!std::isnan(var_ref_zero) && std::isnan(var_ref_span)){
    coeffs.flag   = 2; // Indicates a simple offset
    coeffs.gain   = 1;
    coeffs.offset = var_ref_zero - var_sen_zero;
  }

  return(coeffs);
}

Cal::CalibrationResult Cal::calibrate_linear(String currentGas, int currentSensor, float currentMeasurement){
  CalibrationCoeffs coeffs = get_calibration_coefficients(currentGas, currentSensor);
  CalibrationResult res; // To store the results

  // Apply calibration
  res.flag = coeffs.flag; // Indicate the type of calibration done
  res.calibratedValue = coeffs.gain * currentMeasurement + coeffs.offset;

  return(res);
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

// Cycles through all calibration variables and adds their names into a single string
String Cal::get_all_cal_header(std::vector<String> gases, int numSensors){
  String cal_header = "";
  for (auto& gas : gases) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      for (auto& dataType : _dataTypes) {
        for (auto& calType : calTypes) {
          cal_header += dataType + "_" + gas + "_" + String(sensor) + "_" + calType + ",";
        }
      }
      // Adds the coefficients (slope/gain & intercept/offset)
      cal_header += gas + "_" + String(sensor) + "_gain,";
      cal_header += gas + "_" + String(sensor) + "_offset,";
      cal_header += gas + "_" + String(sensor) + "_flag,";
    }
  }
  return(cal_header);
}

// Cycles through all calibration variables and adds their values into a single string
String Cal::get_all_cal_data(std::vector<String> gases, int numSensors){
  String cal_data = "";
  CalibrationCoeffs coeffs;
  for (auto& gas : gases) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      for (auto& dataType : _dataTypes) {
        for (auto& calType : calTypes) {
          cal_data += String(read_var(dataType + "_" + gas + "_" + String(sensor) + "_" + calType)) + ",";
        }
      }
      // Adds the coefficients (slope/gain & intercept/offset)
      coeffs = get_calibration_coefficients(gas, sensor);
      cal_data += String(coeffs.gain) + ",";
      cal_data += String(coeffs.offset) + ",";
      cal_data += String(coeffs.flag) + ",";
    }
  }
  return(cal_data);
}

// dataType is "ref" or "sen", where "ref" is the low value and "sen" is high value
int Cal::set_differential_coeff(String dataType, String currentGas,
                                float sen0_measurement, float sen1_measurement,
                                unsigned long secs_since_midnight){
  // Save the variable to persistent storage
  String var_sen0 = dataType + "_" + currentGas + "_0_diff"; // Sensor 1 value, e.g. ref_co2_1_diff
  String var_sen1 = dataType + "_" + currentGas + "_1_diff"; // Sensor 2 value

  // First check if the value of "ref" (low) is too similar to that of "sen" (high)
  float var_sen0_low = read_var("ref_" + currentGas + "_0_diff");
  float var_sen1_low = read_var("ref_" + currentGas + "_1_diff");
  if((dataType == "sen") &
     ((std::fabs(sen0_measurement - var_sen0_low) < 100) |
      (std::fabs(sen1_measurement - var_sen1_low) < 100))){
    Serial.println("High calibration value too similar or below that of low calibration. Please repeat calibration");
    // Therefore, set to 0 and to the difference so that the "high" calibration value gets ignored
    // This results in a slope of 1 and an intercept that always yields just the plain difference
    update_var(var_sen0, 0);
    update_var(var_sen1, read_var("ref_" + currentGas + "_1_diff")); // Theoretically it should be ref_2 - ref_1, but ref_1=0
    return(1); // Error 1 = Not enough difference between high and low values
  }

  // Calculate time between calibrations
  if(dataType == "ref"){
    diff_low_cal_time_s = secs_since_midnight;
  }
  // Calculated time is 0 when "ref" (i.e., the low diff calibration)
  // Otherwise it's a known number of seconds
  int time_since_last_cal = (secs_since_midnight - diff_low_cal_time_s);
  Serial.print("Secs since diff low cal: "); Serial.println(time_since_last_cal);
  
  // Wrap around when time <0
  if(time_since_last_cal < 0){
    // Time counting went across midnight, wrap around
    diff_low_cal_time_s = 86400 + diff_low_cal_time_s;
  }

  // Check how much time has passed
  if((time_since_last_cal > 0) && (time_since_last_cal < 10)){
    Serial.println("<10s since low calibration. Are you sure the high concentration is stable?");
    return(3); // Error 3 = Too little time since "low" calibration
  } else if((time_since_last_cal > 600)  && (currentGas != "temperature")){ // >10min. To do a simple offset calibration, wait 10min then click "diff span"
    // A simple offset calibration, i.e. there won't be a linear equation
    Serial.println(">10min since low calibration. Apply simple offset, not linear equation");
    diff_low_cal_time_s = 0;
    if(dataType == "sen"){
      // Set to 0 and to the difference so that the "high" calibration value gets ignored
      // This results in a slope of 1 and an intercept that always yields just the plain difference
      update_var(var_sen0, float(NAN));
      update_var(var_sen1, float(NAN));
    }
    return(4); // Error 4 = Too much time since "low" calibration, only offset applied
  } else {
    // Either time_since_last_cal is 0 (i.e., the "ref" or low calibration),
    // Or it is between 10-600s (i.e., the "sen" or high calibration)
    Serial.println("All good, doing calibration");
    update_var(var_sen0, sen0_measurement);
    update_var(var_sen1, sen1_measurement);
    // When doing the "high" calibration, update the zero & span values ONLY if there are already values there
    if((dataType == "sen") && (!std::isnan(read_var("sen_" + currentGas + "_1_span")))){
      update_cal_from_diff(currentGas);
      Serial.println("    UPDATE cal from differential");
    }
  }

  return(0); // No error
}

// Update calibration coefficients of sensor 2 based on diff calibration
int Cal::update_cal_from_diff(String currentGas){
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_0_zero");
  float var_ref_span = read_var("ref_" + currentGas + "_0_span");
  
  // Get differential readings
  String var_sen0_low  = "ref_" + currentGas + "_0_diff";
  String var_sen0_high = "sen_" + currentGas + "_0_diff";
  String var_sen1_low  = "ref_" + currentGas + "_1_diff";
  String var_sen1_high = "sen_" + currentGas + "_1_diff";
  float var_diff_sen0_low  = read_var(var_sen0_low);
  float var_diff_sen0_high = read_var(var_sen0_high);
  float var_diff_sen1_low  = read_var(var_sen1_low);
  float var_diff_sen1_high = read_var(var_sen1_high);

  // Slope & intercept of diff calibration
  float slope_diff     = (var_diff_sen0_high - var_diff_sen0_low) / (var_diff_sen1_high - var_diff_sen1_low);
  float intercept_diff = var_diff_sen0_low - slope_diff * var_diff_sen1_low;

  // Update values
  float var_sen_1_zero =  -intercept_diff/slope_diff;
  float var_sen_1_span = (var_ref_span - intercept_diff)/slope_diff;

  // Store again
  update_var("sen_" + currentGas + "_1_zero", var_sen_1_zero);
  update_var("sen_" + currentGas + "_1_span", var_sen_1_span);

  return(0);
}
