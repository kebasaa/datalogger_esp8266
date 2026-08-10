#pragma once
// ============================================================================
//  Datalogger framework internals: Plugin + sensor objects, global state, buffers/structs
//  Auto-relocated verbatim from the old monolithic main.cpp. Included ONLY by
//  datalogger.h (one translation unit). Beginners never need to open this.
// ============================================================================

char* dev_name = "Datalogger";
#if H4P_USE_WIFI_AP
String wifitype = "WIFI: AP mode";
H4P_WiFi h4wifi(dev_name);
#else
String wifitype = "WIFI: Client mode";
H4P_WiFi h4wifi(WIFI_SSID, WIFI_PASS, dev_name);
#endif


#if USE_MQTT
#if H4P_USE_WIFI_AP
H4P_AsyncMQTT h4mqtt;
#else
H4P_AsyncMQTT h4mqtt(MQTT_SERVER);
#endif // H4P_USE_WIFI_AP
uint8_t big[BIG_SIZE];
H4_TIMER mqttSender;
H4_TIMER bigSender;
#endif // USE_MQTT

//H4P_Heartbeat h4hb; // Show uptime

#if USE_HTTPREQ
H4P_AsyncHTTP h4ah;
H4_TIMER httpReqTimer;
//void HTTPClient();
#endif
//void publishDevice(const std::string &topic, const std::string &payload);
//void publishDevice(const std::string &topic, long long payload);

int n_measurements = 0;

// Run a test
#if RUN_TEST
# include <test.h>
Test test;
#endif

// I2C multiplexer (TCA9548A). I2C_MULTI is always 1 (see config.h); it is the
// ABI-shared switch the sensor libraries are compiled against.
#include <TCA9548_multiplexer.h>
MULTI mp;

// --- Distinct set of multiplexer channels actually in use -------------------
// Filled by bind_sensors() as the union of every per-type bus list. All I2C
// health/recovery bookkeeping is indexed by POSITION into this array.
constexpr size_t MAX_I2C_BUSES = 8;               // the TCA9548A has 8 channels
uint8_t i2c_buses[MAX_I2C_BUSES];
size_t  num_i2c_buses = 0;
void i2c_bus_union_add(uint8_t ch) {
  for (size_t i = 0; i < num_i2c_buses; ++i) if (i2c_buses[i] == ch) return;
  if (num_i2c_buses < MAX_I2C_BUSES) i2c_buses[num_i2c_buses++] = ch;
}
int i2c_bus_pos(uint8_t ch) {                     // channel -> position, -1 if unused
  for (size_t i = 0; i < num_i2c_buses; ++i) if (i2c_buses[i] == ch) return (int)i;
  return -1;
}

// MicroSD card reader
#if USE_MICROSD
# include <MicroSD.h>
MicroSD sd;
#endif

// Battery
#if USE_BATTERY
# include <Battery.h>
Battery bat;
#endif

// GPS device
#if USE_GPS
# include <GPS.h>
// The XA1110 is wired to the main I2C bus, not through the TCA9548A (null mux).
GPS gps(nullptr, 0);
#endif

// --- Multiplexer sensors ----------------------------------------------------
// One storage/pointer/count trio per type, sized from the *_BUSES list the user
// wrote in main.cpp. Objects are default-constructed here and bound to their
// channel in bind_sensors(). See main.cpp for how to add/remove sensors.

// T, RH, P sensor
#if USE_BME280
#include <BME280_sen.h>
static const uint8_t bme_buses[] = { BME280_BUSES };
constexpr size_t BME_COUNT = sizeof(bme_buses) / sizeof(bme_buses[0]);
static_assert(BME_COUNT <= 2, "Max 2 BME280 sensors (differential calibration compares 2 sensors)");
BME  bme_storage[BME_COUNT];
BME* bme_sensors[BME_COUNT];
#endif

// CO2 sensor
#if USE_SCD30
#include <SCD30.h>
static const uint8_t scd_buses[] = { SCD30_BUSES };
constexpr size_t SCD_COUNT = sizeof(scd_buses) / sizeof(scd_buses[0]);
static_assert(SCD_COUNT <= 2, "Max 2 SCD30 sensors (differential calibration compares 2 sensors)");
SCD  scd_storage[SCD_COUNT];
SCD* scd_sensors[SCD_COUNT];
#endif

// LWR sensor (thermal / IR)
#if USE_MLX90614
#include <MLX90614.h>
static const uint8_t mlx_buses[] = { MLX90614_BUSES };
constexpr size_t MLX_COUNT = sizeof(mlx_buses) / sizeof(mlx_buses[0]);
static_assert(MLX_COUNT <= 2, "Max 2 MLX90614 sensors");
MLX  mlx_storage[MLX_COUNT];
MLX* mlx_sensors[MLX_COUNT];
#endif

// ADS1115 analog-to-digital converter
#if USE_ADS1115
# include <ADS1115.h>
static const uint8_t ads_buses[] = { ADS1115_BUSES };
constexpr size_t ADS_COUNT = sizeof(ads_buses) / sizeof(ads_buses[0]);
static_assert(ADS_COUNT <= 1, "ADS1115 multi-bus is untested (shared driver object); use a single channel");
ADS  ads_storage[ADS_COUNT];
ADS* ads_sensors[ADS_COUNT];
#endif

// SEN0465 O2 sensor
#if USE_SEN0465
#include <SEN0465.h>
static const uint8_t sen_buses[] = { SEN0465_BUSES };
constexpr size_t SEN_COUNT = sizeof(sen_buses) / sizeof(sen_buses[0]);
static_assert(SEN_COUNT <= 2, "Max 2 SEN0465 sensors (differential calibration compares 2 sensors)");
SEN0465  sen_storage[SEN_COUNT];
SEN0465* sen_sensors[SEN_COUNT];
#endif

// Calculate environmental parameters
#if USE_ENV
#include <Environmental.h>
Env env;
#endif

#include <Utils.h>
Utils utils;

#if USE_CAL
#include <Calibration.h>
Cal cal;
// Everything that can be calibrated: gases plus temperature and air pressure.
std::vector<String> quantities;// = {"co2", "o2", "h2o"};
// Calibration stores coefficients per (quantity, sensor 0..numSensors-1). Sized
// to the largest per-type sensor count (<=2) by bind_sensors().
int numSensors = 1;
int currentSensor = -9999;
String currentGas = "all";
String currentDiffGas = "all";
bool calibration_running = false;
// Repeating task that reconstructs missing calibration values, and the timer
// that saves a calibration the user left switched on. Both are cancellable.
H4_TIMER cal_fix_timer = nullptr;
H4_TIMER cal_session_timeout_timer = nullptr;
#endif

// Bind every multiplexer sensor object to its channel, build the channel union,
// and size calibration storage. Must run once, first thing in h4setup(), before
// any sensor init / header build / sampling.
void bind_sensors() {
  num_i2c_buses = 0;
#if USE_BME280
  for (size_t i = 0; i < BME_COUNT; ++i) { bme_storage[i].setMultiplexer(&mp, bme_buses[i]); bme_sensors[i] = &bme_storage[i]; i2c_bus_union_add(bme_buses[i]); }
#endif
#if USE_SCD30
  for (size_t i = 0; i < SCD_COUNT; ++i) { scd_storage[i].setMultiplexer(&mp, scd_buses[i]); scd_sensors[i] = &scd_storage[i]; i2c_bus_union_add(scd_buses[i]); }
#endif
#if USE_SEN0465
  for (size_t i = 0; i < SEN_COUNT; ++i) { sen_storage[i].setMultiplexer(&mp, sen_buses[i]); sen_sensors[i] = &sen_storage[i]; i2c_bus_union_add(sen_buses[i]); }
#endif
#if USE_MLX90614
  for (size_t i = 0; i < MLX_COUNT; ++i) { mlx_storage[i].setMultiplexer(&mp, mlx_buses[i]); mlx_sensors[i] = &mlx_storage[i]; i2c_bus_union_add(mlx_buses[i]); }
#endif
#if USE_ADS1115
  for (size_t i = 0; i < ADS_COUNT; ++i) { ads_storage[i].setMultiplexer(&mp, ads_buses[i]); ads_sensors[i] = &ads_storage[i]; i2c_bus_union_add(ads_buses[i]); }
#endif
  // Sort the channel union ascending so status columns (i2c_busN_*) are stable.
  for (size_t i = 1; i < num_i2c_buses; ++i) {
    uint8_t key = i2c_buses[i];
    size_t j = i;
    while (j > 0 && i2c_buses[j - 1] > key) { i2c_buses[j] = i2c_buses[j - 1]; --j; }
    i2c_buses[j] = key;
  }
#if USE_CAL
  // Size calibration storage to the largest count among the gas-producing types.
  numSensors = 1;
  #if USE_BME280
  if ((int)BME_COUNT > numSensors) numSensors = (int)BME_COUNT;
  #endif
  #if USE_SCD30
  if ((int)SCD_COUNT > numSensors) numSensors = (int)SCD_COUNT;
  #endif
  #if USE_SEN0465
  if ((int)SEN_COUNT > numSensors) numSensors = (int)SEN_COUNT;
  #endif
#endif
}

// Timezone setup
//H4P_Timekeeper h4tk(NTP1, NTP2, LocalTZO);  // Time support
//boolean ClockValid = false;  // Flag indicating whether to display the clock or not.

// Wifi
boolean WiFiValid = false;  // Flag indicating a valid WiFi Connection is active.

const size_t DATA_ROW_BUFFER_SIZE = 3072;
const uint16_t ROW_STATUS_TRUNCATED = 0x8000;
const uint32_t I2C_SLOW_THRESHOLD_MS = 1000;
const uint32_t I2C_RECOVERY_CONSECUTIVE_ERRORS = 3;
const uint32_t I2C_RECOVERY_COOLDOWN_MS = 300000UL;
// Microseconds any one bit-wait may spend waiting for a slave to release SCL.
// This CANNOT be tightened to bound a stuck bus: the SCD30 legitimately stretches
// SCL for tens of milliseconds while it prepares a measurement (its datasheet
// allows up to 150 ms), which is why SparkFun's driver sets 200 ms of its own
// accord. Undercutting that breaks real reads. We match the SCD30's figure so no
// driver silently fights another, and deal with a genuinely stuck bus by
// detecting it up front instead - see dl_i2c_bus_recover() and i2c_bus_ok.
const uint32_t I2C_CLOCK_STRETCH_US = 200000;

// Set once at boot. False means SCL was still held low after a recovery attempt,
// i.e. no I2C device can be reached and probing them would only burn the full
// stretch limit on every bit of every transaction (~50 s of boot time observed).
bool i2c_bus_ok = false;
const uint32_t GPS_FRESH_MAX_AGE_MS = 120000UL;
const uint32_t GPS_STALE_CONSECUTIVE_SAMPLES = 3;
const uint32_t GPS_RECOVERY_COOLDOWN_MS = 300000UL;
const uint16_t GPS_SLICE_MAX_MS = 20;
const uint16_t GPS_SLICE_MAX_CHARS = 64;
const uint32_t GPS_POLL_INTERVAL_MS = 250;
const uint32_t GPS_QUARANTINE_AFTER_RECOVERIES = 1;
const uint32_t GPS_QUARANTINE_MS = 1200000UL;
const uint32_t CRITICAL_SENSOR_LOSS_SAMPLES = 45;
const uint32_t GPS_I2C_HARD_FAULT_SAMPLES = 6;
const uint32_t SUPERVISOR_MIN_UPTIME_MS = 600000UL;
const uint32_t SUPERVISOR_RESTART_COOLDOWN_MS = 1800000UL;
const uint32_t BOOT_GPS_GRACE_MS = 120000UL;
const uint16_t GPS_YEAR_MIN = 2026;
const uint16_t GPS_YEAR_MAX = 2030;
const uint32_t GPS_MAX_TIME_JUMP_SECONDS = 86400UL;

char data_header[DATA_ROW_BUFFER_SIZE] = "";
char data_row_buf[DATA_ROW_BUFFER_SIZE] = "";
char status_row_buf[1024] = "";
char reset_reason_buf[48] = "unknown";
char supervisor_restart_reason_buf[32] = "";
char boot_time_source_buf[12] = "none";
char timestamp_calc_buf[20] = "";
char timestamp_file_date_buf[9] = "";
uint32_t boot_ms = 0;
uint32_t boot_id = 0;
uint32_t sample_counter = 0;
uint16_t last_data_write_status = 0;
bool data_header_ready = false;
bool gps_boot_locked = false;
bool boot_status_written = false;
bool sd_status_pending = false;
char last_valid_gps_date[9] = "";
uint32_t gps_stale_count = 0;
uint32_t gps_recovery_count = 0;
uint32_t gps_poll_timeout_count = 0;
uint32_t gps_i2c_recovery_count = 0;
uint32_t gps_stale_recovery_count = 0;
uint32_t gps_quarantine_count = 0;
uint32_t gps_quarantine_until_ms = 0;
uint32_t last_gps_recovery_ms = 0;
bool gps_stale_active = false;
bool gps_quarantine_active = false;
bool gps_time_sane = false;
bool gps_stale_status_pending = false;
bool gps_recovered_status_pending = false;
bool gps_i2c_stale_status_pending = false;
bool gps_i2c_recovery_status_pending = false;
bool gps_i2c_quarantine_status_pending = false;
bool gps_i2c_recovered_status_pending = false;
bool gps_time_rejected_status_pending = false;
bool boot_time_from_rtc_status_pending = false;
uint32_t last_fresh_gps_epoch = 0;
uint32_t last_fresh_gps_millis = 0;
bool last_fresh_gps_epoch_valid = false;
// The clock currently in use came from the web UI, not from GPS or the RTC.
// Load-bearing for two things: is_gps_epoch_sane() must not measure a real GPS
// fix against a hand-entered clock (a phone more than a day out would otherwise
// reject the fix forever), and the size of the correction is reported once when
// GPS does take over.
bool manual_time_active = false;
bool manual_time_jump_pending = false;
long manual_time_jump_s = 0;
bool rtc_time_valid = false;
bool boot_time_from_rtc = false;
uint32_t rtc_boot_epoch = 0;
uint32_t gps_rejected_time_count = 0;
uint32_t valid_sensor_value_count = 0;
uint32_t missing_sensor_value_count = 0;
uint32_t critical_sensor_loss_count = 0;
uint32_t gps_i2c_hard_fault_count = 0;
uint32_t supervisor_reset_count = 0;
uint32_t last_supervisor_reset_ms = 0;
bool sensor_data_lost_status_pending = false;
bool supervisor_restart_pending = false;
bool supervisor_restart_status_pending = false;
H4_TIMER gps_poll_timer;
uint32_t gps_chars_since_sample = 0;

// I2C health/recovery counters, indexed by POSITION in i2c_buses[] (0..num_i2c_buses-1).
uint32_t i2c_slow_count[MAX_I2C_BUSES] = {0};
uint32_t i2c_error_count[MAX_I2C_BUSES] = {0};
uint32_t i2c_consecutive_error_count[MAX_I2C_BUSES] = {0};
uint32_t i2c_recovery_count[MAX_I2C_BUSES] = {0};
uint32_t i2c_last_recovery_ms[MAX_I2C_BUSES] = {0};
bool i2c_recovery_status_pending = false;

struct RtcRestartRecord {
  uint32_t magic;
  uint32_t version;
  uint32_t epoch;
  uint32_t sample_counter;
  uint32_t reason;
  uint32_t crc;
};

const uint32_t RTC_RESTART_MAGIC = 0xDACE2026UL;
const uint32_t RTC_RESTART_VERSION = 1;
const uint32_t RTC_RESTART_OFFSET = 64;
const uint32_t RTC_REASON_GPS_I2C_HARD_FAULT = 1;
// A clock set by hand from the web UI. Distinct from the fault code above so the
// stored record still says why it was written; the reason is carried through to
// the CSV, and reusing 1 would report a hardware fault that never happened.
const uint32_t RTC_REASON_MANUAL_TIME = 2;

struct CsvBuffer {
  char* buf;
  size_t size;
  size_t len;
  bool truncated;
};
