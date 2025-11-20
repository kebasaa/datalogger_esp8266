#pragma once
#include <string>

#ifndef H4P_SECURE
#define H4P_SECURE 1
#endif

#define USE_MQTT 0
#define USE_HTTPREQ 0
#define USE_BLESERVER 0
#define USE_BLECLIENT 0

#define SECURE_MQTT 1
#define SECURE_WEBSERVER 0
#define SECURE_HTTPREQ 1

// Activate/Deactivate sensors

// Measurement settings
#define MEASUREMENT_INTERVAL 20
#define WS_VERSION "0.4"

// Multiple i2c buses
#define I2C_MULTI    1
#if I2C_MULTI
#define I2C_BUS0     0
#define I2C_BUS1     0
#define I2C_BUS2     1
#define I2C_BUS3     1
#endif

// Components
#define RUN_TEST     0
#define USE_MICROSD  1
#define USE_BATTERY  1
#define USE_GPS      1
#define USE_BME280   1
#define USE_SCD30    1
#define USE_SEN0465  1
#define USE_MLX90614 0
#define USE_ADS1115  0
#define USE_ENV      1
#define USE_CAL      1
#if USE_CAL
#define USE_ENV      1 // Environmental calculations are required for calibration
#endif

// Activate/Deactivate sensors (end)

struct SensorConfig {
    bool useBME280 = USE_BME280;
    bool useSCD30 = USE_SCD30;
    bool useSEN0465 = USE_SEN0465;
    bool useMLX90614 = USE_MLX90614;
    bool useADS1115 = USE_ADS1115;
    bool useGPS = USE_GPS;
    bool useBattery = USE_BATTERY;
    bool useMicroSD = USE_MICROSD;
    bool useEnv = USE_ENV;
    bool useCal = USE_CAL;
    bool i2cMulti = I2C_MULTI;
    int measurementInterval = MEASUREMENT_INTERVAL;
};

extern SensorConfig sensorConfig;

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

// See config.cpp
extern const char WIFI_SSID[];
extern const char WIFI_PASS[];

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
