#pragma once
// ============================================================================
//  Datalogger framework — internal wiring.
//
//  DO NOT include this file from anywhere except main.cpp. It defines the H4
//  framework's global objects, which may exist in exactly ONE compiled unit;
//  including it a second time from another .cpp would break the build.
//
//  Beginners: you never need to open or edit this file. Everything you can
//  configure lives at the top of main.cpp.
// ============================================================================

#include <Arduino.h>

// Translate the friendly settings from main.cpp (SAMPLE_INTERVAL_SECONDS,
// ENABLE_*, WIFI_ACCESS_POINT_MODE, ...) into the internal macro names the
// framework uses, and fill in defaults for anything left unset. Must come
// BEFORE config.h and the H4 headers so those see the resolved values.
#include "dl_config_map.h"

// Advanced / hardware settings shared with the sensor libraries and config.cpp
// (I2C bus topology, radios, TLS). Edited only for non-standard hardware.
#include "config.h"

#define H4P_VERBOSE 1 // To see what's going on
#include <H4Plugins.h>
H4_USE_PLUGINS(PROJ_BAUD_RATE, H4_Q_CAPACITY, false) // Serial baud rate, Q size, SerialCmd autostop

// Optional guard-rails: catch common mis-configurations early.
#if defined(WIFI_SSID) && H4P_USE_WIFI_AP
#warning "WIFI_SSID is set but access-point mode is on; SSID/PASS are ignored. Set WIFI_ACCESS_POINT_MODE 0 to join a network."
#endif

// C-style H4 accessors (declarations). Also included by lib/Calibration; the
// matching definitions live in dl_glue.h (this single translation unit).
#include "h4_wrapper.h"

// ---- Framework implementation, relocated verbatim from the old main.cpp ----
// Order is significant: each fragment depends on symbols defined above it.
#include "dl_glue.h"      // h4_gv* definitions, core includes, BIG_SIZE
#include "dl_objects.h"   // plugin + sensor objects, global state, buffers/structs
#include "dl_core.h"      // CSV/time/RTC/heap helpers, I2C+GPS recovery, WiFi/MQTT callbacks
#include "dl_webui.h"     // calibration command handlers
#include "dl_api.h"       // HTTP/JSON API + SPA file serving (defines dl_toast)
#include "dl_sampling.h"  // processData / processDataBuffered sampling routine
#include "dl_setup.h"     // h4setup() — MUST be last
