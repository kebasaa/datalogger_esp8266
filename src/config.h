#pragma once
#include <string>

// ============================================================================
//  ADVANCED settings — shared with the sensor libraries (lib/) and config.cpp,
//  so they must live here rather than in main.cpp. Most users never edit this
//  file. The everyday configuration (sampling rate, WiFi, which sensors are
//  connected) is at the top of main.cpp.
// ============================================================================

#ifndef H4P_SECURE
#define H4P_SECURE 1
#endif

// Optional radios / network clients (off by default). These are read here and
// in config.cpp, so they stay in this shared header.
#define USE_MQTT 0
#define USE_HTTPREQ 0
#define USE_BLESERVER 0
#define USE_BLECLIENT 0

#define SECURE_MQTT 1
#define SECURE_WEBSERVER 0
#define SECURE_HTTPREQ 1

// Firmware version string (shown on the serial console at boot).
#define WS_VERSION "0.5"

// ---------------------------------------------------------------------------
//  I2C bus topology. I2C_MULTI selects single-bus vs dual-bus (TCA9548A
//  multiplexer) wiring. This MUST be set here: the sensor libraries in lib/
//  are compiled separately and change their layout based on it, so it cannot
//  be moved into main.cpp.
// ---------------------------------------------------------------------------
#define I2C_MULTI    1
#if I2C_MULTI
#define I2C_BUS0     0
#define I2C_BUS1     0
#define I2C_BUS2     1
#define I2C_BUS3     1
#endif

// ESP8266/RP2040 don't support TLS
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_RP2040)
#undef H4P_SECURE
#undef SECURE_WEBSERVER
#undef SECURE_HTTPREQ
#define H4P_SECURE 		0
#define SECURE_WEBSERVER 0
#define SECURE_HTTPREQ  0
#endif

#if !(ARDUINO_ARCH_ESP32 && CONFIG_BT_ENABLED && CONFIG_BT_BLUEDROID_ENABLED) // !H4P_BLE_AVAILABLE
#undef USE_BLECLIENT
#undef USE_BLESERVER
#define USE_BLECLIENT 	0
#define USE_BLESERVER 	0
#endif

#if USE_HTTPREQ
#if SECURE_HTTPREQ && !H4P_SECURE
#warning "Activate H4P_SECURE if attempting to secure the HTTP requests"
#undef SECURE_HTTPREQ
#define SECURE_HTTPREQ 0
#endif
#endif

#if SECURE_WEBSERVER && !H4P_SECURE
#warning "Activate H4P_SECURE if attempting to secure the webserver"
#undef SECURE_WEBSERVER
#define SECURE_WEBSERVER 0
#endif

#if SECURE_MQTT && !H4P_SECURE
#warning "Activate H4P_SECURE if attempting to secure the MQTT Client"
#undef SECURE_MQTT
#define SECURE_MQTT 0
#endif

#if USE_MQTT
extern const char *MQTT_SERVER;
#if SECURE_MQTT
extern std::string MQTT_CERT;
#endif // H4P_SECURE
#endif // USE_MQTT

#if SECURE_WEBSERVER
extern std::string WEBSERVER_CERT;
extern std::string WEBSERVER_PRIV_KEY;
#endif // SECURE_WEBSERVER

#if USE_HTTPREQ && SECURE_HTTPREQ
// ISRG Root X1 certificate, the CA of Let's Encrypt
extern std::string test_root_ca;
#endif // SECURE_HTTPREQ
