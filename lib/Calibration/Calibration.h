/*
  Calculate parameters from sensor readings
*/
#pragma once

//#ifndef Calibrationh_h
//#define Calibrationh_h

#include "Arduino.h"
#include <vector>
#include <string>

class Cal {
  public:
    // main Class
    Cal(void);
  
    // Functions
    void  create_var(String var_type, float val);
    //void  update_var(String var_type, float val); // Currently not used
    float read_var(String var_type);
    void init_all_calibrations(std::vector<String> gases, int numSensors);
    String get_cal_header(std::vector<String> gases, int numSensors);
    int set_calibration_coeff(String calType, String currentGas, int currentSensor, float zero_ref, float zero_measured);
    int set_diff(String currentGas, float sen1_measurement, float sen2_measurement);
    float calibrate_zero_span(String currentGas, int currentSensor, float currentMeasurement);
    float calibrate_differential(String currentGas, float currentMeasurement);
    
  private:
    // Functions
    bool is_near_zero(float x, float eps = 1e-6f);
    // Measured sensor value ("sen") or reference ("ref"), as the reference value also needs to be stored
    std::vector<String> _dataTypes = {"sen", "ref"};
    std::vector<String> _calTypes = {"zero", "span", "diff"};
};

//#endif
