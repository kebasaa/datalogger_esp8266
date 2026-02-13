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
  // Default is raw data
  coeffs.gain = 1.0f;
  coeffs.offset = 0.0f;
  coeffs.flag = -1;

  int otherSensor = (currentSensor == 0) ? 1 : 0;

  // ------------------ READ NEEDED VALUES ONCE ------------------
  // Get absolute calibration variables for current sensor
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero"); // Reference gas, zero
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span"); // Reference gas, span
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero"); // Sensor reading at zero
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span"); // Sensor reading at span

  // Absolute values for otherSensor (needed in many cases)
  float other_ref_zero = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_ref_span = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_span");
  float other_sen_zero = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_sen_span = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_span");

  // Differential values (read once)
  // Mapping used here:
  //   cur_low  = reference low (cur_low)
  //   cur_high  = reference high (cur_high)
  //   other_low  = sensor1 low  (other_low)
  //   S_high  = sensor1 high (other_high)
  float R_low    = read_var("ref_" + currentGas + "_0_diff"); // Sensor 0 (the reference), low value
  float R_high   = read_var("sen_" + currentGas + "_0_diff"); // Sensor 0 (the reference), high value
  float S_low  = read_var("ref_" + currentGas + "_1_diff"); // Sensor 1 (to be adjusted), low value
  float S_high = read_var("sen_" + currentGas + "_1_diff"); // Sensor 1 (to be adjusted), high value

  auto has = [&](float v){ return !std::isnan(v); };
  bool abs_all_present = has(var_ref_zero) && has(var_ref_span) && has(var_sen_zero) && has(var_sen_span);
  bool diff_all_present = has(R_low) && has(R_high) && has(S_low) && has(S_high);

  // ------------------ 1) Full absolute calibration for currentSensor ------------------
  if (abs_all_present) {
    // compute gain/offset normally
    if (Utils::is_near_zero(var_sen_span - var_sen_zero)) {
      coeffs.flag = -1;
      coeffs.gain = 1.0f; coeffs.offset = 0.0f;
      Serial.println("Absolute present but sensor span==zero -> returning raw (-1).");
      return coeffs;
    }
    coeffs.gain = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
    coeffs.offset = var_ref_zero - coeffs.gain * var_sen_zero;
    coeffs.flag = 0;
    Serial.println("Return full absolute calibration (flag 0).");
    return coeffs;
  }

  // ------------------ 2) Full differential available -> multiple sub-cases ------------------
  if (diff_all_present) {

    // If no slope in S, differential invalid
    if (Utils::is_near_zero(S_high - S_low)) {
      // cannot compute differential slope meaningfully
      coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
      Serial.println("Differential present but sensor slope zero -> returning raw (-1).");
      return coeffs;
    }

    // differential slope S->R for sensor1
    float gain_diff = (R_high - R_low) / (S_high - S_low);
    float offset_diff = R_low - gain_diff * S_low; // this maps a sensor1 raw -> reference units

    // 2a) If currentSensor has either absolute zero pair OR absolute span pair,
    //     then:
    //       - if currentSensor == 0 -> it's the reference: do not adjust it (raw)
    //       - if currentSensor == 1 -> we can do differential + absolute offset (flag = 1)
    if ((has(var_ref_zero) && has(var_sen_zero)) || (has(var_ref_span) && has(var_sen_span))) {
      if (currentSensor == 0) {
        coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
        Serial.println("currentSensor==0 and has absolute partial: do not apply differential to reference -> raw (-1).");
        return coeffs;
      } else {
        // sensor 1: differential gain & offset from diffs, plus adjust offset to align with absolute point(s)
        // If we have zero pair for currentSensor use that to compute absolute adjustment:
        float adj_offset = offset_diff; // start with diff offset
        if (has(var_ref_zero) && has(var_sen_zero)) {
          // measured at zero (sen_zero) should map to ref_zero. Compute correction = ref_zero - (gain_diff * sen_zero + offset_diff)
          float predicted_ref_from_diff = gain_diff * var_sen_zero + offset_diff;
          adj_offset = offset_diff + (var_ref_zero - predicted_ref_from_diff);
        } else if (has(var_ref_span) && has(var_sen_span)) {
          float predicted_ref_from_diff = gain_diff * var_sen_span + offset_diff;
          adj_offset = offset_diff + (var_ref_span - predicted_ref_from_diff);
        }
        coeffs.flag = 1;
        coeffs.gain = gain_diff;
        coeffs.offset = adj_offset;
        Serial.println("Differential + absolute offset computed for currentSensor (flag 1).");
        return coeffs;
      }
    }

    // 2b) If otherSensor has either absolute zero pair OR absolute span pair
    if ((has(other_ref_zero) && has(other_sen_zero)) || (has(other_ref_span) && has(other_sen_span))) {
      if (currentSensor == 0) {
        // Use differential mapping and otherSensor absolute offset to compute equivalent offset for currentSensor.
        // Since sensor0 is reference, leave it raw.
        coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
        Serial.println("currentSensor==0 and other sensor has absolute -> reference kept raw (flag -1).");
        return coeffs;
      } else {
        // currentSensor == 1, compute differential then adjust offset to match otherSensor's absolute pair
        float adj_offset = offset_diff;
        if (has(other_ref_zero) && has(other_sen_zero)) {
          float predicted_ref_from_diff = gain_diff * other_sen_zero + offset_diff;
          adj_offset = offset_diff + (other_ref_zero - predicted_ref_from_diff);
        } else if (has(other_ref_span) && has(other_sen_span)) {
          float predicted_ref_from_diff = gain_diff * other_sen_span + offset_diff;
          adj_offset = offset_diff + (other_ref_span - predicted_ref_from_diff);
        }
        coeffs.flag = 1;
        coeffs.gain = gain_diff;
        coeffs.offset = adj_offset;
        Serial.println("Differential + offset computed for currentSensor using otherSensor absolute (flag 1).");
        return coeffs;
      }
    }

    // 2c) Neither sensor has absolute pairs -> differential-only
    if (!has(other_ref_zero) && !has(other_ref_span) && !has(var_ref_zero) && !has(var_ref_span)) {
      if (currentSensor == 0) {
        coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
        Serial.println("No absolute pairs anywhere: currentSensor==0 -> raw (-1).");
        return coeffs;
      } else {
        coeffs.flag = 2; // differential only
        coeffs.gain = gain_diff;
        coeffs.offset = offset_diff;
        Serial.println("Differential only for sensor1 (flag 2).");
        return coeffs;
      }
    }
  } // end diff_all_present

  // ------------------ 3) No differential present -> try absolute partial offset for currentSensor ------------------
  // If currentSensor has either zero pair (ref+meas) or span pair, return simple absolute offset
  if ( (has(var_ref_zero) && has(var_sen_zero)) || (has(var_ref_span) && has(var_sen_span)) ) {
    coeffs.flag = 4;
    coeffs.gain = 1.0f;
    if (has(var_ref_zero) && has(var_sen_zero)) {
      coeffs.offset = var_ref_zero - var_sen_zero;
      Serial.println("Absolute zero pair present -> simple absolute offset (flag 4).");
    } else {
      coeffs.offset = var_ref_span - var_sen_span;
      Serial.println("Absolute span pair present -> simple absolute offset (flag 4).");
    }
    return coeffs;
  }

  // ------------------ 4) Partial diffs exist (only lows or only highs) and no absolute for currentSensor ------------------
  // If currentSensor==0 -> raw
  if ( (has(R_low) && has(S_low)) || (has(R_high) && has(S_high)) ) {
    if (currentSensor == 0) {
      coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
      Serial.println("Only partial diffs and currentSensor==0 -> raw (-1).");
      return coeffs;
    } else {
      // currentSensor == 1 -> differential offset only
      coeffs.flag = 3;
      coeffs.gain = 1.0f;
      if (has(R_low) && has(S_low)) coeffs.offset = R_low - S_low;
      else coeffs.offset = R_high - S_high;
      Serial.println("Partial diffs -> differential offset for sensor1 (flag 3).");
      return coeffs;
    }
  }

  // ------------------ 5) Nothing usable -> raw fallback ------------------
  coeffs.flag = -1;
  coeffs.gain = 1.0f;
  coeffs.offset = 0.0f;
  Serial.println("No calibration data available -> returning raw (-1).");
  return coeffs;
}

int Cal::fix_calibration_coefficients(String currentGas, int currentSensor){
  // returns 0 if nothing needed or fixes applied successfully, >0 to indicate non-fatal conditions
  int otherSensor = (currentSensor == 0) ? 1 : 0;

  // Read all absolute & differential values for both sensors once
  float cur_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero"); // Reference gas, zero
  float cur_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span"); // Reference gas, span
  float cur_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero"); // Sensor reading at zero
  float cur_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span"); // Sensor reading at span

  // Absolute values for otherSensor (needed in many cases)
  float other_ref_zero = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_ref_span = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_span");
  float other_sen_zero = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_sen_span = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_span");

  float cur_low    = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_diff"); // current sensor, low value
  float cur_high   = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff"); // current sensor, high value
  float other_low  = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_diff");   // other sensor, low value
  float other_high = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_diff");   // other sensor, high value

  auto has = [&](float v){ return !std::isnan(v); };

  // ---------- 1) If both sensors have full absolute pairs, but diffs missing -> create diffs ----------
  //-----------------------------------------------------------------------------------------------------
  bool currentSensor_abs_full = has(cur_ref_zero) &&   has(cur_ref_span) &&   has(cur_sen_zero) &&   has(cur_sen_span);
  bool otherSensor_abs_full =   has(other_ref_zero) && has(other_ref_span) && has(other_sen_zero) && has(other_sen_span);

  if (currentSensor_abs_full && otherSensor_abs_full) {
    // Create diffs from absolute pairs if missing
    if (std::isnan(cur_low) || std::isnan(other_low)){
	  update_var("ref_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_zero); // current sensor, low value
	  update_var("ref_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_zero); // other sensor, low value
	}
    if (std::isnan(cur_high) || std::isnan(other_high)){
	  update_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_span); // current sensor, high value
	  update_var("sen_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_span); // other sensor, high value
	}
    Serial.println("Created missing differential entries from full absolute pairs.");
    return 0;
  }

  // ---------- 2) If some absolute pair and full diffs present, we can compute abs for missing values ----------
  //-------------------------------------------------------------------------------------------------------------
  bool diffs_present = has(cur_low) && has(cur_high) && has(other_low) && has(other_high);

  if (diffs_present) {
	if (!Utils::is_near_zero(cur_high - cur_low) && !Utils::is_near_zero(other_high - other_low)) {
	  // Gain & offset from diffs (maps current to other sensor)
	  float d_gain_CO   = (cur_high - cur_low) / (other_high - other_low);
	  float d_offset_CO = cur_low - d_gain_CO * other_low;
	  // Gain & offset from diffs (maps other to current sensor)
	  float d_gain_OC   = (other_high - other_low) / (cur_high - cur_low);
	  float d_offset_OC = other_low - d_gain_OC * cur_low;
	  
	  // Flags
	  bool cur_zero_present   = !std::isnan(cur_sen_zero)   && !std::isnan(cur_ref_zero);
	  bool cur_span_present   = !std::isnan(cur_sen_span)   && !std::isnan(cur_ref_span);
	  bool other_zero_present = !std::isnan(other_sen_zero) && !std::isnan(other_ref_zero);
	  bool other_span_present = !std::isnan(other_sen_span) && !std::isnan(other_ref_span);
	  
      // a) Determine current sensor's zero & span from other sensor
      //============================================================
	  if(otherSensor_abs_full && (std::isnan(cur_sen_zero) || std::isnan(cur_sen_span))) {
		// Calculate the theoretical measured calibration values
		float cur_sen_zero_calc = d_gain_CO * other_sen_zero + d_offset_CO;
		float cur_sen_span_calc = d_gain_CO * other_sen_span + d_offset_CO;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero", other_ref_zero);
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_span", other_ref_span);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero", cur_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_span", cur_sen_span_calc);
		Serial.print("ref_" + currentGas + "_" + String(currentSensor) + "_zero: "); Serial.println(other_ref_zero);
		Serial.print("ref_" + currentGas + "_" + String(currentSensor) + "_span: "); Serial.println(other_ref_span);
		Serial.print("sen_" + currentGas + "_" + String(currentSensor) + "_zero: "); Serial.println(cur_sen_zero_calc);
		Serial.print("sen_" + currentGas + "_" + String(currentSensor) + "_span: "); Serial.println(cur_sen_span_calc);
		return 0;
	  }
	  // b) Determine other sensor's zero & span from current sensor
      //=============================================================
	  if(currentSensor_abs_full && (std::isnan(other_sen_zero) || std::isnan(other_sen_span))) {
		// Calculate the theoretical measured calibration values
		float other_sen_zero_calc = d_gain_OC * cur_sen_zero + d_offset_OC;
		float other_sen_span_calc = d_gain_OC * cur_sen_span + d_offset_OC;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero", cur_ref_zero);
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_span", cur_ref_span);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero", other_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_span", other_sen_span_calc);
		Serial.print("ref_" + currentGas + "_" + String(otherSensor) + "_zero: "); Serial.println(cur_ref_zero);
		Serial.print("ref_" + currentGas + "_" + String(otherSensor) + "_span: "); Serial.println(cur_ref_span);
		Serial.print("sen_" + currentGas + "_" + String(otherSensor) + "_zero: "); Serial.println(other_sen_zero_calc);
		Serial.print("sen_" + currentGas + "_" + String(otherSensor) + "_span: "); Serial.println(other_sen_span_calc);
		return 0;
	  }
	  // c) Determine from current sensor's zero & other sensor's span
      //==============================================================
	  if (cur_zero_present && !other_zero_present && !cur_span_present && other_span_present) {
		// Calculate the theoretical measured calibration values
		float other_sen_zero_calc = d_gain_OC * cur_sen_zero + d_offset_OC;
		float cur_sen_span_calc = d_gain_CO * other_sen_span + d_offset_CO;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero", cur_ref_zero);
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_span", other_ref_span);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero", other_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_span", cur_sen_span_calc);
		return 0;
	  }
	  // d) Determine from other sensor's zero & current sensor's span
      //==============================================================
	  if (!cur_zero_present && other_zero_present && cur_span_present && !other_span_present) {
		// Calculate the theoretical measured calibration values
		float cur_sen_zero_calc = d_gain_CO * other_sen_zero + d_offset_CO;
		float other_sen_span_calc = d_gain_OC * cur_sen_span + d_offset_OC;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero", other_ref_zero);
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_span", cur_ref_span);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero", cur_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_span", other_sen_span_calc);
		return 0;
	  }
    }
  }
  
  // ---------- 3) If only one differential (low or high) exists but there is a corresponding absolute pair for the other for both,
  //               we can create a full set of diffs by combining the absolute values
  //-------------------------------------------------------------------------------------------------------------

  bool diff_low_pair  = has(cur_low) && has(other_low);
  bool diff_high_pair = has(cur_high) && has(other_high);
  bool abs_zero_pair  = has(cur_sen_zero) && has(other_sen_zero);
  bool abs_span_pair  = has(cur_sen_span) && has(other_sen_span);
  
  if(diff_low_pair && abs_span_pair && !diff_high_pair){
    // Create diff_high_pair using absolute span pair
	update_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_span); // current sensor, high value
	update_var("sen_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_span); // other sensor, high value
	  
  }
  if(abs_zero_pair && diff_high_pair && !diff_low_pair){
    // Create diff_low_pair using absolute zero pair
	update_var("ref_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_zero); // current sensor, low value
	update_var("ref_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_zero); // other sensor, low value
  }

  // ---------- 4) If we can't reconstruct meaningful missing values, do nothing ----------
  Serial.println("No further reconstruction possible (either only single isolated points available or nothing).");
  return 1;
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
