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
    void  update_var(String var_type, float val);
    float read_var(String var_type);
    float calibrate_measurement(String var_type, float measurement_raw);
    void init_all_calibrations(std::vector<String> gases, int numSensors);
    String get_cal_header(std::vector<String> gases, int numSensors);
    int set_zero(String currentGas, int currentSensor, float zero_ref, float zero_measured);
    int set_span(String currentGas, int currentSensor, float zero_ref, float zero_measured);
    int set_diff(String currentGas, float sen1_measured, float sen2_measured);

  private:
    // Measured sensor value ("sen") or reference ("ref"), as the reference value also needs to be stored
    std::vector<String> _dataTypes = {"sen", "ref"};
    std::vector<String> _calTypes = {"zero", "span", "diff"};
    // Nothing
};

//#endif
