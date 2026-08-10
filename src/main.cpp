// ============================================================================
//                        DATALOGGER  —  CONFIGURATION
//
//  Edit only the values in this block, then upload. Everything else lives in
//  the framework (datalogger.h) and does not need to be touched.
// ============================================================================

// --- How often to record a measurement (in seconds) ---
#define SAMPLE_INTERVAL_SECONDS   20

// --- WiFi -------------------------------------------------------------------
//  1 = the logger makes its own WiFi hotspot; open the dashboard by joining it
//      (works in the field, no router needed). This is the default.
//  0 = the logger joins your existing WiFi. If you choose 0, un-comment and
//      fill in WIFI_SSID / WIFI_PASS below.
#define WIFI_ACCESS_POINT_MODE    1
// #define WIFI_SSID   "MyNetwork"
// #define WIFI_PASS   "MyPassword"

// --- Passwords --------------------------------------------------------------
#define HOTSPOT_PASSWORD      "datalogger"   // WiFi password in access-point mode
#define DASHBOARD_USER        "admin"        // web dashboard login name
#define DASHBOARD_PASSWORD    "datalogger"   // web dashboard login password

// --- Single-instance parts (1 = yes, 0 = no) --------------------------------
#define ENABLE_MICROSD     1   // MicroSD card logging
#define ENABLE_BATTERY     1   // battery voltage / charge reading
#define ENABLE_GPS         1   // GPS module (date, time, location)
#define ENABLE_CALIBRATION 1   // sensor calibration + web calibration interface

// --- Calibration ------------------------------------------------------------
// While a calibration session is open, calibrating changes values in memory
// only - they are all saved together the moment you close it (this spares the
// memory chip, which wears out if written to too often). If a session is left
// open and untouched for this many minutes, it saves itself so the work is not
// lost. Set to 0 to never save automatically.
#define CALIBRATION_TIMEOUT_MINUTES  30

// --- Sensors on the I2C multiplexer -----------------------------------------
// For each sensor type, list the multiplexer channel(s) it is wired to. You may
// run two of the same type on two different channels (e.g. a reference and a
// field sensor, which the differential calibration compares). Comment a line
// out to disable that sensor type entirely.
//   * Max 2 channels per type (differential calibration compares 2 sensors).
//   * Channels may be any TCA9548A channel number (0-7), e.g. "0, 2".
#define BME280_BUSES     0, 1   // air temperature / humidity / pressure
#define SCD30_BUSES      0, 1   // CO2
#define SEN0465_BUSES    0, 1   // O2
// #define MLX90614_BUSES   0   // infrared / thermal (off)
// #define ADS1115_BUSES    0   // analog-to-digital converter (single channel only)

// Note: the multiplexer itself (I2C_MULTI) is configured in src/config.h,
// because the sensor libraries must be compiled with the same setting.
// ============================================================================
//                          END OF CONFIGURATION
// ============================================================================

#include <datalogger.h>
