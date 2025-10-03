/*
  Calculate parameters from sensor readings
*/
#pragma once

//#ifndef Calibrationh_h
//#define Calibrationh_h

#include "Arduino.h"
#include <vector>
#include <string>

#include "config.h"

class Cal {
  public:
    // main Class
    Cal(void);
  
    // Functions
    void  init_all_calibrations(std::vector<String> gases, int numSensors);
    void  reset_all_calibrations(std::vector<String> gases, int numSensors);
    void  create_var(String var_type, float val);
    void  update_var(String var_type, float val);
    float read_var(String var_type);
    int   set_calibration_coeff(String calType, String currentGas, int currentSensor, float zero_ref, float zero_measured, unsigned long secs_since_midnight);
    float calibrate_linear(String currentGas, int currentSensor, float currentMeasurement);
    float read_calibration_var(String calType, String sensorType, String currentGas, int currentSensor);

    int   set_differential_coeff(String dataType, String currentGas, float sen1, float sen2, unsigned long secs_since_midnight);
    int   update_cal_from_diff(String currentGas);

    float read_linear_slope(String currentGas, int currentSensor, float currentMeasurement);
    float read_linear_intercept(String currentGas, int currentSensor, float currentMeasurement);

    String get_all_cal_header(std::vector<String> gases, int numSensors);
    String get_all_cal_data(std::vector<String> gases, int numSensors);
    
  private:
    // Functions
    bool is_near_zero(float x, float eps = 1e-6f);
    // Measured sensor value ("sen") or reference ("ref"), as the reference value also needs to be stored
    std::vector<String> _dataTypes = {"sen", "ref"};
    std::vector<String> _calTypes = {"zero", "span", "diff"};

  #if I2C_MULTI
    unsigned long zero_cal_time_s = 0;
    unsigned long diff_low_cal_time_s = 0;
  #endif
};

//#endif
