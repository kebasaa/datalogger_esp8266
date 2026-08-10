#pragma once
// ============================================================================
//  Config translation layer.
//
//  Turns the friendly settings a beginner writes at the top of main.cpp into
//  the internal macro names the framework uses, and supplies sensible defaults
//  for anything left unset (so an empty config still builds and runs).
//
//  You should not need to edit this file. To change what the logger does, edit
//  the configuration block in main.cpp. Advanced hardware options (I2C bus
//  topology, radios, TLS) live in config.h.
// ============================================================================

// ---------------------------------------------------------------- Sampling --
#ifdef SAMPLE_INTERVAL_SECONDS
  #define MEASUREMENT_INTERVAL SAMPLE_INTERVAL_SECONDS
#endif
#ifndef MEASUREMENT_INTERVAL
  #define MEASUREMENT_INTERVAL 20
#endif

// -------------------------------------------------------------------- WiFi --
// 1 = the logger runs its own access point (offline dashboard, no router).
// 0 = the logger joins an existing network (define WIFI_SSID / WIFI_PASS).
//
// These two are the exception to the pattern used everywhere else in this file.
// H4Plugins compiles its own .cpp files as separate translation units that
// never include a project header, and H4P_USE_WIFI_AP guards a data member of
// class H4P_WiFi as well as whole functions. Defining it only here would give
// the project and the library two different layouts for the same class, and the
// access point would never start because _startAP() was never compiled at all.
//
// scripts/project_defines.py therefore reads WIFI_ACCESS_POINT_MODE and
// HOTSPOT_PASSWORD out of main.cpp and passes them as global -D flags, so both
// halves of the build agree. The guards below defer to that, and only supply a
// value when this header is used without the pre-build script.
#ifndef H4P_USE_WIFI_AP
  #ifdef WIFI_ACCESS_POINT_MODE
    #define H4P_USE_WIFI_AP WIFI_ACCESS_POINT_MODE
  #else
    #define H4P_USE_WIFI_AP 1
  #endif
#endif

// Access-point WiFi password
#ifndef DATALOGGER_AP_PASSWORD
  #ifdef HOTSPOT_PASSWORD
    #define DATALOGGER_AP_PASSWORD HOTSPOT_PASSWORD
  #else
    #define DATALOGGER_AP_PASSWORD "datalogger"
  #endif
#endif

// Web dashboard login
#ifdef DASHBOARD_USER
  #define DATALOGGER_UI_USER DASHBOARD_USER
#endif
#ifndef DATALOGGER_UI_USER
  #define DATALOGGER_UI_USER "admin"
#endif
#ifdef DASHBOARD_PASSWORD
  #define DATALOGGER_UI_PASSWORD DASHBOARD_PASSWORD
#endif
#ifndef DATALOGGER_UI_PASSWORD
  #define DATALOGGER_UI_PASSWORD "datalogger"
#endif

// ------------------------------------------------------ Sensors / hardware --
#ifdef ENABLE_MICROSD
  #define USE_MICROSD ENABLE_MICROSD
#endif
#ifndef USE_MICROSD
  #define USE_MICROSD 1
#endif

#ifdef ENABLE_BATTERY
  #define USE_BATTERY ENABLE_BATTERY
#endif
#ifndef USE_BATTERY
  #define USE_BATTERY 1
#endif

#ifdef ENABLE_GPS
  #define USE_GPS ENABLE_GPS
#endif
#ifndef USE_GPS
  #define USE_GPS 1
#endif

// Multiplexer sensor types are enabled by listing their bus(es) in main.cpp
// (e.g. #define BME280_BUSES 0, 1). Presence of the *_BUSES macro turns the
// type on; the bus list itself is consumed later in dl_objects.h.
#ifdef BME280_BUSES
  #define USE_BME280 1
#else
  #define USE_BME280 0
#endif

#ifdef SCD30_BUSES
  #define USE_SCD30 1
#else
  #define USE_SCD30 0
#endif

#ifdef SEN0465_BUSES
  #define USE_SEN0465 1
#else
  #define USE_SEN0465 0
#endif

#ifdef MLX90614_BUSES
  #define USE_MLX90614 1
#else
  #define USE_MLX90614 0
#endif

#ifdef ADS1115_BUSES
  #define USE_ADS1115 1
#else
  #define USE_ADS1115 0
#endif

// --------------------------------------------------------------- Calibration --
#ifdef ENABLE_CALIBRATION
  #define USE_CAL ENABLE_CALIBRATION
#endif
#ifndef USE_CAL
  #define USE_CAL 1
#endif

// How long a calibration may sit switched on and untouched before it saves
// itself, in minutes. 0 = never save automatically.
#ifdef CALIBRATION_TIMEOUT_MINUTES
  #define CAL_SESSION_TIMEOUT_MIN CALIBRATION_TIMEOUT_MINUTES
#endif
#ifndef CAL_SESSION_TIMEOUT_MIN
  #define CAL_SESSION_TIMEOUT_MIN 30
#endif

// Environmental physics helpers are on by default and are REQUIRED whenever
// calibration is enabled (calibration converts humidity <-> mole fraction).
#ifndef USE_ENV
  #define USE_ENV 1
#endif
#if USE_CAL && !USE_ENV
  #undef USE_ENV
  #define USE_ENV 1
#endif

// -------------------------------------------------------------- Diagnostics --
// Built-in self-test routine (developers only); off unless explicitly enabled.
#ifndef RUN_TEST
  #define RUN_TEST 0
#endif
