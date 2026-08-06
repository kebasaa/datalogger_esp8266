/*
  Calculate parameters from sensor readings
*/
#pragma once

//#ifndef Calibrationh_h
//#define Calibrationh_h

#include "Arduino.h"
#include <vector>
#include <string>
#include <map>

#include "config.h"

class Cal {
  public:
    // main Class
    Cal(void);

    // Structures
    struct CalibrationResult {
      float calibratedValue;
      int   flag;
    };

    struct CalibrationCoeffs {
      float offset;
      float gain;
      int   flag;
    };
  
    // ---- Calibration sessions -------------------------------------------
    // While a session is open, calibration values are kept in RAM only and
    // nothing is written to flash. commit_session() writes them all in one go;
    // discard_session() throws them away without writing anything. Everything
    // that reads a calibration value goes through read_var(), so the web UI and
    // the live measurements see the pending values straight away.
    bool   session_open(void) const;
    void   open_session(void);
    void   commit_session(void);
    void   discard_session(void);
    size_t pending_count(void) const;

    // ---- Value storage ----------------------------------------------------
    // Bind the store to this logger's quantity list and sensor count, load what
    // was saved, and migrate anything an older firmware left in H4's persistent
    // globals. Call once, after the quantities vector has been filled and before
    // anything reads a calibration.
    //
    // Values live in the fixed array below rather than in H4's globals map. That
    // map cost ~120 bytes per entry and a fully calibrated two-sensor logger has
    // 60 of them - around 7 KB, on a board that has about 10 KB of heap free with
    // the WiFi access point running. The array is 240 bytes and does not grow
    // with how much has been calibrated. See the .cpp for the file format.
    void  storage_begin(const std::vector<String>& quantities, int numSensors);

    // Functions
    void  show_all_calibrations(std::vector<String> quantities, int numSensors);
    void  reset_all_calibrations(std::vector<String> quantities, int numSensors);
    void  create_var(String var_type, float val);
    void  update_var(String var_type, float val);
    void  erase_var(const std::string& name);
    float read_var(String var_type);
    int   set_calibration_coeff(String calType, String currentGas, int currentSensor, float zero_ref, float zero_measured, unsigned long secs_since_midnight);
    CalibrationCoeffs get_calibration_coefficients(String currentGas, int currentSensor);
    int   fix_calibration_coefficients(String currentGas, int currentSensor);
    void  fix_all_calibrations(std::vector<String> quantities, int numSensors);
    CalibrationResult calibrate_linear(String currentGas, int currentSensor, float currentMeasurement);
    float read_calibration_var(String dataType, String calType, String currentGas, int currentSensor);

    int   set_differential_coeff(String dataType, String currentGas, float sen1, float sen2, unsigned long secs_since_midnight);
    int   update_cal_from_diff(String currentGas);

    String get_all_cal_header(std::vector<String> quantities, int numSensors);
    String get_all_cal_data(std::vector<String> quantities, int numSensors);

    std::vector<String> calTypes = {"zero", "span", "diff"};

    // "No calibration recorded" marker for the timestamps below. Distinct from
    // 0, which is a valid time (midnight).
    static const unsigned long NO_CAL_TIME = (unsigned long)-1;

    // Give the "how long since the last calibration" checks a starting point.
    // Used by the serial "set" command, which enters values by hand and is meant
    // to be accepted whatever the timing.
    void force_cal_time_reference(unsigned long secs_since_midnight);

  private:
    // ---- Value storage ----------------------------------------------------
    // Sized for the compile-time maximum, not the configured one, so the array
    // is a fixed 2 * 8 * 2 * 3 floats regardless of how many quantities this
    // build ends up using. 240 bytes of that is live for the usual five.
    static const size_t MAX_QUANTITIES = 8;
    static const size_t MAX_SENSORS    = 2;
    static const size_t CAL_SLOTS      = 2 * MAX_QUANTITIES * MAX_SENSORS * 3;

    float               _values[CAL_SLOTS];
    std::vector<String> _quantities;      // copied once in storage_begin()
    int                 _numSensors = 0;
    bool                _storage_ready = false;

    // Name -> array index. Returns -1 for a name this build has no slot for,
    // which is how an unknown line in the saved file is ignored rather than
    // landing on the wrong value.
    int   _slot(const std::string& name) const;
    // Reverse: rebuild the stored name for a slot, for writing the file out.
    bool  _slot_name(size_t idx, std::string& out) const;

    bool  _load_file(const char* path);
    bool  _write_file(const char* path);
    void  _persist_values();
    size_t _migrate_from_globals();

    // Functions
    //bool is_near_zero(float x, float eps = 1e-6f);
    // Encode and hand a single value to persistent storage. "save" controls
    // whether the file is rewritten straight away.
    void write_var(const std::string& name, float val, bool save);
    // Measured sensor value ("sen") or reference ("ref"), as the reference value also needs to be stored
    std::vector<String> _dataTypes = {"sen", "ref"};

    // Calibration values changed since the session was opened, not yet written
    // to flash. Empty whenever no session is open.
    std::map<std::string, float> _pending;
    bool _session = false;

    unsigned long zero_cal_time_s = NO_CAL_TIME;
    unsigned long diff_low_cal_time_s = NO_CAL_TIME;
};

//#endif
