#include <Arduino.h>
#include "config.h"

#define H4P_VERBOSE 1 // To see what's going on
#include <H4Plugins.h>
H4_USE_PLUGINS(PROJ_BAUD_RATE, H4_Q_CAPACITY, false) // Serial baud rate, Q size, SerialCmd autostop

// This creates wrapper functions that can't be declared anywhere else than in main.cpp
#include "h4_wrapper.h"
// Permanent storage of data/variables
bool h4_gvExists(std::string name){
  return h4p.gvExists(name);
}
void h4_gvSetInt(std::string name, int value, bool save){
  h4p.gvSetInt(name, value, save);
}
void h4_gvSetString(std::string name, std::string value, bool save){
  h4p.gvSetstring(name, value, save);
}
int h4_gvGetInt(std::string name){
  return h4p.gvGetInt(name);
}

// Store whether any viewers are connected
bool viewersConnected = false;

#include <math.h>
#include <stdarg.h>
#include <string.h>

// Default I2C bus on D2=SDA, D1=SCL
#include <Wire.h>

//H4P_SerialLogger h4sl;
//H4P_PinMachine h4gm; // For buttons

#ifdef ARDUINO_ARCH_ESP8266
#define BIG_SIZE 500
#else
#define BIG_SIZE 13000
#endif

char* dev_name = "Datalogger";
#if H4P_USE_WIFI_AP
String wifitype = "WIFI: AP mode";
H4P_WiFi h4wifi; //h4wifi(dev_name);
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

#if I2C_MULTI
#include <TCA9548_multiplexer.h>
MULTI mp;
int i2c_buses[] = {0,1};
#endif

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
#if I2C_MULTI
GPS gps(&mp, 2); // Not connected to the multiplexer, this will not affect anything
#else
GPS gps();
#endif
#endif

// T, RH, P sensor
#if USE_BME280
#include <BME280_sen.h>
#if I2C_MULTI
BME bme1(&mp, 0); // Initialise buses 0 & 1
BME bme2(&mp, 1);
BME* bme_sensors[] = { &bme1, &bme2 };
#else
BME bme(&mp, 1); // Initialise bus 1 only
BME* bme_sensors[] = { &bme };
#endif
#endif

// CO2 sensor
#if USE_SCD30
#include <SCD30.h>
#if I2C_MULTI
SCD scd1(&mp, 0); // Initialise buses 0 & 1
SCD scd2(&mp, 1);
SCD* scd_sensors[] = { &scd1, &scd2 };
#else
SCD scd(&mp, 1); // Initialise bus 2 only
SCD* scd_sensors[] = { &scd };
#endif
#endif

// LWR sensor
#if USE_MLX90614
#include <MLX90614.h>
#if I2C_MULTI
MLX mlx1(&mp, 0); // Initialise buses 0 & 1
MLX mlx2(&mp, 1);
MLX* mlx_sensors[] = { &mlx1, &mlx2 };
#else
MLX mlx(&mp, 1); // Initialise bus 1 only
MLX* mlx_sensors[] = { &mlx };
#endif
#endif

// ADS1115 analog-to-digital converter, not used in this project
#if USE_ADS1115
# include <ADS1115.h>
#if I2C_MULTI
ADS ads1(&mp, 0); // Initialise buses 0 & 1
ADS ads2(&mp, 1);
ADS* ads_sensors[] = { &ads1, &ads2 };
#else
ADS ads(&mp, 1); // Initialise bus 1 only
ADS* ads_sensors[] = { &ads };
#endif
#endif

// SEN0465 O2 sensor
#if USE_SEN0465
#include <SEN0465.h>
#if I2C_MULTI
SEN0465 sen1(&mp, 0); // Initialise buses 0 & 1
SEN0465 sen2(&mp, 1);
SEN0465* sen_sensors[] = { &sen1, &sen2 };
#else
SEN0465 sen(&mp, 1); // Initialise bus 1 only
SEN0465* sen_sensors[] = { &sen };
#endif
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
std::vector<String> gases;// = {"co2", "o2", "h2o"};
// numSensors depends on whether "Multi" is active
#if I2C_MULTI
  int numSensors = 2;
  #else
  int numSensors = 1;
#endif
int currentSensor = -9999;
String currentGas = "all";
String currentDiffGas = "all";
bool calibration_running = false;
#endif

H4_TIMER  TIM0;

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
const uint32_t GPS_FRESH_MAX_AGE_MS = 120000UL;
const uint32_t GPS_STALE_CONSECUTIVE_SAMPLES = 3;
const uint32_t GPS_RECOVERY_COOLDOWN_MS = 300000UL;

char data_header[DATA_ROW_BUFFER_SIZE] = "";
char data_row_buf[DATA_ROW_BUFFER_SIZE] = "";
char status_row_buf[512] = "";
char reset_reason_buf[48] = "unknown";
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
uint32_t last_gps_recovery_ms = 0;
bool gps_stale_active = false;
bool gps_stale_status_pending = false;
bool gps_recovered_status_pending = false;
uint32_t last_fresh_gps_epoch = 0;
uint32_t last_fresh_gps_millis = 0;
bool last_fresh_gps_epoch_valid = false;

#if I2C_MULTI
uint32_t i2c_slow_count[2] = {0, 0};
uint32_t i2c_error_count[2] = {0, 0};
uint32_t i2c_consecutive_error_count[2] = {0, 0};
uint32_t i2c_recovery_count[2] = {0, 0};
uint32_t i2c_last_recovery_ms[2] = {0, 0};
#else
uint32_t i2c_slow_count[1] = {0};
uint32_t i2c_error_count[1] = {0};
uint32_t i2c_consecutive_error_count[1] = {0};
uint32_t i2c_recovery_count[1] = {0};
uint32_t i2c_last_recovery_ms[1] = {0};
#endif
bool i2c_recovery_status_pending = false;

struct CsvBuffer {
  char* buf;
  size_t size;
  size_t len;
  bool truncated;
};

void csv_init(CsvBuffer& csv, char* buf, size_t size) {
  csv.buf = buf;
  csv.size = size;
  csv.len = 0;
  csv.truncated = false;
  if (size > 0) csv.buf[0] = '\0';
}

void csv_append_raw(CsvBuffer& csv, const char* text) {
  if (csv.len >= csv.size) {
    csv.truncated = true;
    return;
  }
  int written = snprintf(csv.buf + csv.len, csv.size - csv.len, "%s", text ? text : "");
  if (written < 0 || (size_t)written >= csv.size - csv.len) {
    csv.len = csv.size - 1;
    csv.buf[csv.len] = '\0';
    csv.truncated = true;
  } else {
    csv.len += (size_t)written;
  }
}

void csv_appendf(CsvBuffer& csv, const char* fmt, ...) {
  if (csv.len >= csv.size) {
    csv.truncated = true;
    return;
  }
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(csv.buf + csv.len, csv.size - csv.len, fmt, args);
  va_end(args);
  if (written < 0 || (size_t)written >= csv.size - csv.len) {
    csv.len = csv.size - 1;
    csv.buf[csv.len] = '\0';
    csv.truncated = true;
  } else {
    csv.len += (size_t)written;
  }
}

void csv_field(CsvBuffer& csv, const char* text) {
  csv_append_raw(csv, text);
  csv_append_raw(csv, ",");
}

void csv_uint(CsvBuffer& csv, uint32_t value) {
  csv_appendf(csv, "%lu,", (unsigned long)value);
}

void csv_int(CsvBuffer& csv, int value) {
  csv_appendf(csv, "%d,", value);
}

void csv_float(CsvBuffer& csv, float value, uint8_t decimals = 2) {
  if (isnan(value) || isinf(value)) {
    csv_append_raw(csv, ",");
  } else {
    char tmp[24];
    dtostrf(value, 0, decimals, tmp);
    csv_append_raw(csv, tmp);
    csv_append_raw(csv, ",");
  }
}

void sanitize_csv_text(const String& src, char* dst, size_t dst_size) {
  if (dst_size == 0) return;
  size_t j = 0;
  for (size_t i = 0; i < src.length() && j < dst_size - 1; i++) {
    char c = src.charAt(i);
    dst[j++] = (c == ',' || c == '\r' || c == '\n') ? ' ' : c;
  }
  dst[j] = '\0';
}

bool is_leap_year(uint16_t year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

uint8_t days_in_month(uint16_t year, uint8_t month) {
  static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month == 2 && is_leap_year(year)) return 29;
  if (month < 1 || month > 12) return 31;
  return days[month - 1];
}

uint32_t utc_to_epoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  uint32_t days = 0;
  for (uint16_t y = 1970; y < year; y++) days += is_leap_year(y) ? 366UL : 365UL;
  for (uint8_t m = 1; m < month; m++) days += days_in_month(year, m);
  days += (uint32_t)(day - 1);
  return days * 86400UL + (uint32_t)hour * 3600UL + (uint32_t)minute * 60UL + second;
}

void epoch_to_utc(uint32_t epoch, uint16_t& year, uint8_t& month, uint8_t& day, uint8_t& hour, uint8_t& minute, uint8_t& second) {
  uint32_t days = epoch / 86400UL;
  uint32_t rem = epoch % 86400UL;
  hour = rem / 3600UL;
  rem %= 3600UL;
  minute = rem / 60UL;
  second = rem % 60UL;
  year = 1970;
  while (true) {
    uint16_t year_days = is_leap_year(year) ? 366 : 365;
    if (days < year_days) break;
    days -= year_days;
    year++;
  }
  month = 1;
  while (true) {
    uint8_t month_days = days_in_month(year, month);
    if (days < month_days) break;
    days -= month_days;
    month++;
  }
  day = days + 1;
}

void format_utc_timestamp(uint32_t epoch, char* dst, size_t dst_size) {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  epoch_to_utc(epoch, year, month, day, hour, minute, second);
  snprintf(dst, dst_size, "%04u-%02u-%02u %02u:%02u:%02u",
           (unsigned)year, (unsigned)month, (unsigned)day,
           (unsigned)hour, (unsigned)minute, (unsigned)second);
}

void format_utc_date(uint32_t epoch, char* dst, size_t dst_size) {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  epoch_to_utc(epoch, year, month, day, hour, minute, second);
  snprintf(dst, dst_size, "%04u%02u%02u", (unsigned)year, (unsigned)month, (unsigned)day);
}

uint32_t calculated_timestamp_epoch(bool gps_time_fresh, uint32_t now_ms, const char** source) {
#if USE_GPS
  if (gps_time_fresh) {
    last_fresh_gps_epoch = utc_to_epoch(gps.year(), gps.month(), gps.day(), gps.hour(), gps.minute(), gps.second());
    last_fresh_gps_millis = now_ms;
    last_fresh_gps_epoch_valid = true;
    if (source) *source = "gps";
    return last_fresh_gps_epoch;
  }
#endif
  if (last_fresh_gps_epoch_valid) {
    if (source) *source = "uptime";
    return last_fresh_gps_epoch + ((now_ms - last_fresh_gps_millis) / 1000UL);
  }
  if (source) *source = "none";
  return 0;
}

uint32_t heap_free_now() {
#if defined(ARDUINO_ARCH_ESP32)
  return _HAL_freeHeap(MALLOC_CAP_INTERNAL);
#else
  return _HAL_freeHeap();
#endif
}

uint32_t heap_max_block_now() {
#if defined(ARDUINO_ARCH_ESP32)
  return _HAL_maxHeapBlock(MALLOC_CAP_INTERNAL);
#else
  return _HAL_maxHeapBlock();
#endif
}

uint32_t heap_min_block_now() {
#if defined(ARDUINO_ARCH_ESP32)
  return _HAL_minHeapBlock(MALLOC_CAP_INTERNAL);
#else
  return _HAL_minHeapBlock();
#endif
}

uint32_t i2c_slow_total() {
  uint32_t total = 0;
  for (size_t i = 0; i < sizeof(i2c_slow_count) / sizeof(i2c_slow_count[0]); i++) total += i2c_slow_count[i];
  return total;
}

uint32_t i2c_error_total() {
  uint32_t total = 0;
  for (size_t i = 0; i < sizeof(i2c_error_count) / sizeof(i2c_error_count[0]); i++) total += i2c_error_count[i];
  return total;
}

uint32_t i2c_recovery_total() {
  uint32_t total = 0;
  for (size_t i = 0; i < sizeof(i2c_recovery_count) / sizeof(i2c_recovery_count[0]); i++) total += i2c_recovery_count[i];
  return total;
}

void build_data_header() {
  CsvBuffer header;
  csv_init(header, data_header, sizeof(data_header));
  csv_append_raw(header, "boot_id,sample_counter,uptime_ms,timestamp_boot_ms,reset_reason,");
#if USE_GPS
  csv_append_raw(header, "gps_date_valid,gps_time_fresh,gps_location_valid,gps_location_fresh,gps_chars_processed,gps_age_ms,gps_location_age_ms,gps_stale_count,gps_recovery_count,timestamp_utc,timestamp_calc_utc,timestamp_calc_source,lat,lon,alt,nb_sat,HDOP,");
#endif
#if USE_BATTERY
  csv_append_raw(header, "bat.mV,bat.perc,");
#endif

#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
  for (size_t i = 0; i < n; ++i) {
#endif
#if USE_ADS1115
    csv_append_raw(header, "Sin.W_m2,Sout.W_m2,");
#endif
#if USE_BME280
    csv_append_raw(header, "bme_T.C,bme_RH.perc,bme_P.Pa,bme_H2O.mmol_mol,");
  #if USE_CAL
    csv_append_raw(header, "bme_T.C.cal,bme_T.flag,bme_P.Pa.cal,bme_P.flag,bme_H2O.mmol_mol.cal,bme_H2O.mmol_mol.flag,bme_RH.perc.cal,bme_RH.flag,");
  #else
    csv_append_raw(header, "bme_T.C,bme_RH.perc,bme_P.Pa,bme_H2O.mmol_mol,");
  #endif
#endif
#if USE_SCD30
    csv_append_raw(header, "scd_T.C,scd_RH.perc,scd_CO2.ppm,");
  #if USE_CAL
    csv_append_raw(header, "scd_CO2.ppm.cal,scd_CO2.flag,");
  #else
    csv_append_raw(header, "scd_CO2.ppm,");
  #endif
#endif
#if USE_SEN0465
    csv_append_raw(header, "sen_T.C,sen_O2.mmol_mol,sen_raw_V,");
  #if USE_CAL
    csv_append_raw(header, "sen_O2.mmol_mol.cal,sen_O2.flag,");
  #else
    csv_append_raw(header, "sen_O2.mmol_mol,");
  #endif
#endif
#if USE_MLX90614
    csv_append_raw(header, "mlx_T.C,mlx_obj_T.C,");
#endif
#if I2C_MULTI
  }
#endif
  csv_append_raw(header, "free_heap,max_heap_block,min_heap_block,i2c_slow_count,i2c_error_count,i2c_recovery_count,");
  for (size_t i = 0; i < sizeof(i2c_error_count) / sizeof(i2c_error_count[0]); i++) {
    csv_appendf(header, "i2c_bus%u_error_count,i2c_bus%u_consecutive_error_count,i2c_bus%u_recovery_count,", (unsigned)i, (unsigned)i, (unsigned)i);
  }
  csv_append_raw(header, "write_status,");
  data_header_ready = !header.truncated;
}

void recoverI2C(size_t bus_index) {
#if I2C_MULTI
  mp.disableAllBuses();
#endif
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
  Wire.end();
#endif
  Wire.begin();
#if USE_BME280
  bme_sensors[bus_index]->init();
#endif
#if USE_SCD30
  if (scd_sensors[bus_index]->init()) {
    scd_sensors[bus_index]->enable_self_calibration(false);
    scd_sensors[bus_index]->set_interval(2);
  }
#endif
#if USE_SEN0465
  sen_sensors[bus_index]->init();
#endif
  i2c_recovery_count[bus_index]++;
  i2c_last_recovery_ms[bus_index] = millis();
  i2c_recovery_status_pending = true;
}

void record_i2c_health(size_t bus_index, uint32_t elapsed_ms, bool error_seen) {
  if (elapsed_ms > I2C_SLOW_THRESHOLD_MS) i2c_slow_count[bus_index]++;
  if (error_seen || elapsed_ms > I2C_SLOW_THRESHOLD_MS) {
    if (i2c_consecutive_error_count[bus_index] == 0) i2c_error_count[bus_index]++;
    i2c_consecutive_error_count[bus_index]++;
  } else {
    i2c_consecutive_error_count[bus_index] = 0;
    return;
  }
  uint32_t now_ms = millis();
  if (i2c_consecutive_error_count[bus_index] >= I2C_RECOVERY_CONSECUTIVE_ERRORS &&
      (i2c_last_recovery_ms[bus_index] == 0 || now_ms - i2c_last_recovery_ms[bus_index] >= I2C_RECOVERY_COOLDOWN_MS)) {
    recoverI2C(bus_index);
  }
}

bool isGPSDateValid();
bool isGPSTimeFresh();
bool isGPSLocationFresh();
void recoverGPS();

uint16_t write_status_row(const char* event, uint16_t related_write_status) {
#if USE_MICROSD
  if (!gps_boot_locked || last_valid_gps_date[0] == '\0') return MicroSD::WRITE_CARD_MISSING;
  char status_fn[32];
  const char* status_file_date = timestamp_file_date_buf[0] ? timestamp_file_date_buf : last_valid_gps_date;
  snprintf(status_fn, sizeof(status_fn), "status_%s.csv", status_file_date);
  const char* status_header =
    "event,boot_id,sample_counter,uptime_ms,reset_reason,free_heap,max_heap_block,min_heap_block,"
    "gps_date_valid,gps_time_fresh,gps_location_valid,gps_location_fresh,gps_stale_count,gps_recovery_count,"
    "i2c_error_count,i2c_recovery_count,related_write_status,card_missing_count,header_open_fail_count,"
    "append_open_fail_count,print_fail_count,flush_fail_count,close_fail_count,";
  CsvBuffer row;
  csv_init(row, status_row_buf, sizeof(status_row_buf));
  csv_field(row, event);
  csv_uint(row, boot_id);
  csv_uint(row, sample_counter);
  csv_uint(row, millis() - boot_ms);
  csv_field(row, reset_reason_buf);
  csv_uint(row, heap_free_now());
  csv_uint(row, heap_max_block_now());
  csv_uint(row, heap_min_block_now());
#if USE_GPS
  csv_int(row, isGPSDateValid() ? 1 : 0);
  csv_int(row, isGPSTimeFresh() ? 1 : 0);
  csv_int(row, gps.locationValid() ? 1 : 0);
  csv_int(row, isGPSLocationFresh() ? 1 : 0);
  csv_uint(row, gps_stale_count);
  csv_uint(row, gps_recovery_count);
#else
  csv_int(row, 0);
  csv_int(row, 0);
  csv_int(row, 0);
  csv_int(row, 0);
  csv_uint(row, 0);
  csv_uint(row, 0);
#endif
  csv_uint(row, i2c_error_total());
  csv_uint(row, i2c_recovery_total());
  csv_uint(row, related_write_status);
  csv_uint(row, sd.cardMissingCount());
  csv_uint(row, sd.headerOpenFailCount());
  csv_uint(row, sd.appendOpenFailCount());
  csv_uint(row, sd.printFailCount());
  csv_uint(row, sd.flushFailCount());
  csv_uint(row, sd.closeFailCount());
  return sd.write_data(status_fn, status_header, status_row_buf, 86400);
#else
  return 0;
#endif
}

bool isGPSDateValid() {
    #if USE_GPS
        if (!gps.dateValid()) return false;
        int year = gps.year();
        return (year >= 2025) && (year <= 2050);
    #else
        return false; // Or true, depending on if you want to proceed without GPS
    #endif
}

bool isGPSTimeFresh() {
    #if USE_GPS
        return isGPSDateValid() && gps.timeValid() && gps.gpsAgeMs() <= GPS_FRESH_MAX_AGE_MS;
    #else
        return false;
    #endif
}

bool isGPSLocationFresh() {
    #if USE_GPS
        return gps.locationValid() && gps.locationAgeMs() <= GPS_FRESH_MAX_AGE_MS;
    #else
        return false;
    #endif
}

void recoverGPS() {
    #if USE_GPS
    #if I2C_MULTI
        mp.disableAllBuses();
    #endif
    #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
        Wire.end();
    #endif
        Wire.begin();
        gps.init();
        gps_recovery_count++;
        last_gps_recovery_ms = millis();
    #endif
}

void onWiFiConnect() {
	Serial.printf("Wifi connected");
	Serial.printf("IP4 Address: %s\n",WiFi.localIP().toString().c_str());
	WiFiValid = true;
#if USE_HTTPREQ
	h4.queueFunction(HTTPClient);
	httpReqTimer = h4.every(60000, HTTPClient);
#endif
}
void onWiFiDisconnect() {
	Serial.printf("WiFi Disconnected\n");
#if USE_HTTPREQ
	h4.cancel(httpReqTimer);
#endif
}
#if USE_MQTT
void onMQTTConnect() {
	mqttSender = h4.every(2000, []()
					  {
						  publishDevice("heap", _HAL_freeHeap());
						  publishDevice("uptime",h4p.gvGetstring(upTimeTag()));
						  publishDevice("maxbloc",_HAL_maxHeapBlock());
					  });
	
	bigSender = h4.every(3000,[]{
		Serial.printf("SENDING BIG\n");
		h4mqtt.publish("big", &big[0], BIG_SIZE, 1);
	});
}
void onMQTTDisconnect() {
	Serial.printf("onMQTTDisconnect()\n");
	h4.cancel(mqttSender);
	h4.cancel(bigSender);
}
#endif

void absolute_calibration(const std::vector<String> gas_list, String abs_cal_type, int sensor, float ref_value);
void differential_calibration(const std::vector<String> gas_list, String dif_cal_type);

// Show calibration coefficients through the Serial Port. The syntax is:
// show_cal
uint32_t cal_show_cmd(std::vector<std::string> vs) {
  Serial.println("Calibration coefficients:");
  cal.show_all_calibrations(gases, numSensors);
  return H4_CMD_OK;
}

// Manually set calibration coefficients through the Serial Port. The syntax is:
// set/zero/co2/1/12.07/13.08
// set/span/co2/2/376.87/402.87
// Where the first number is the reference, and the second the measured value by the sensor for that reference
uint32_t cal_set_cmd(std::vector<std::string> vs) {
  Serial.println("Manually entering sensor calibration coefficients");
  Serial.printf("Params received: %d\n", vs.size());
  if (vs.size() < 5) {
    Serial.println("Min. 5 parameters: type (zero/span), gas, sensor number, ref. value and measured value");
    return H4_CMD_TOO_FEW_PARAMS;
  } else if(vs.size() > 5){
    Serial.println("Max. 5 parameters: type (zero/span), gas, sensor number, ref. value and measured value");
    return H4_CMD_TOO_MANY_PARAMS;
  }
  // Collect the values
  String abs_cal_type = vs[0].c_str();
  String cal_gas = vs[1].c_str();
  int sensor_nb = std::stoi(vs[2].c_str());
  float ref_value = std::stof(vs[3]);
  float sen_value = std::stof(vs[4]);
  // Forceds acceptance of serial port values no matter the timing
  int ts = 0;
  if(abs_cal_type == "span"){ ts = 20; }
  // Set calibration values
  cal.set_calibration_coeff(abs_cal_type, cal_gas, sensor_nb, ref_value, sen_value, ts);
  return H4_CMD_OK;
}

// Calibration through the Serial Port. The syntax is:
// cal/span/h2o/0/1.31
// cal/zero/co2/1
// cal/zero/co2/1/0.54
// cal/span/h2o/2/409.87
// cal/diff/co2/low
// cal/diff/co2/high
uint32_t cal_cmd(std::vector<std::string> vs) {
    Serial.println("Sensor calibration initiated");
    Serial.printf("Params received: %d\n", vs.size());
    if (vs.size() < 2) {
        Serial.println("Calibration command requires at least 2 parameters: type and gas");
        return H4_CMD_TOO_FEW_PARAMS;
    }
    std::string cal_type = vs[0];
    std::string cal_gas = vs[1];
    // Validate calibration type
    if (!utils.in_list(cal.calTypes, String(cal_type.c_str()))) {
        Serial.printf("Invalid calibration type: %s (must be %s)\n", cal_type.c_str(), utils.make_text_list(cal.calTypes));
        return H4_CMD_PAYLOAD_FORMAT;
    }
    // Validate gas
    if (!utils.in_list(gases, String(cal_gas.c_str()))) {
        Serial.printf("Invalid calibration gas: %s (must be %s)\n", cal_gas.c_str(), utils.make_text_list(gases));
        return H4_CMD_PAYLOAD_FORMAT;
    }
    //Serial.printf("Type: %s, Gas: %s\n", cal_type.c_str(), cal_gas.c_str());
    // After validation, update the global variables
    int cal_sensor = 0;
    float cal_ref_value = 0.0;
    if (cal_type == "zero") {
        // Requires sensor (3 params), cal_fef_value optional (default 0.0)
        if (vs.size() == 2) {
            Serial.println("Zero calibration requires a sensor number");
            return H4_CMD_TOO_FEW_PARAMS;
        } else if (vs.size() == 3) {
            cal_sensor = std::stoi(vs[2].c_str());
            cal_ref_value = 0.0; // Default
            Serial.printf("Zero cal: Gas: %s, Sensor=%d, Reference=%.2f\n", cal_gas.c_str(), cal_sensor, cal_ref_value);
            // Perform calibration
            absolute_calibration({cal_gas.c_str()}, "zero", cal_sensor, cal_ref_value);
        } else if (vs.size() == 4) {
            cal_sensor = std::stoi(vs[2].c_str());
            cal_ref_value = std::stoi(vs[3]);
            Serial.printf("Zero cal: Gas: %s, Sensor=%d, Reference=%.2f\n", cal_gas.c_str(), cal_sensor, cal_ref_value);
            // Perform calibration
            absolute_calibration({cal_gas.c_str()}, "zero", cal_sensor, cal_ref_value);
        } else {
            Serial.println("Too many params for zero calibration");
            return H4_CMD_TOO_MANY_PARAMS;
        }
    } else if (cal_type == "span") {
        // Requires sensor and cal_ref_value (4 params)
        if (vs.size() < 4) {
            Serial.println("Span calibration requires sensor and reference value");
            return H4_CMD_TOO_FEW_PARAMS;
        } else if (vs.size() == 4) {
            cal_sensor = std::stoi(vs[2].c_str());
            cal_ref_value = std::stof(vs[3]);
            Serial.printf("Span cal: Gas: %s, Sensor=%d, Reference=%.2f\n", cal_gas.c_str(), cal_sensor, cal_ref_value);
            // Perform calibration
            currentGas = cal_gas.c_str(); // There is a clause that checks if currentGas is all in absolute_calibration(). This avoids that
            absolute_calibration({cal_gas.c_str()}, "span", cal_sensor, cal_ref_value);
        } else {
            Serial.println("Too many params for span calibration");
            return H4_CMD_TOO_MANY_PARAMS;
        }
    } else if (cal_type == "diff") {
        // Requires type, gas and low/high (3 params), no sensor/cal_ref_value
        if(vs.size() > 3) {
            Serial.println("Diff calibration requires at least 2 parameters: gas and type (low/high)");
            return H4_CMD_TOO_FEW_PARAMS;
        } else if (vs.size() == 3) {
            // "sen" is for the high calibration, "ref" for low
            String low_high = "";
            if(vs[2] == "low"){
              low_high = "ref";
            } else if (vs[2] == "high"){
              low_high = "sen";
            } else {
              // Bad command, don't calibrate
              return H4_CMD_PAYLOAD_FORMAT;
            }
            Serial.printf("Diff cal: Gas: %s, Type=%s\n", cal_gas.c_str(), low_high);
            differential_calibration({cal_gas.c_str()}, low_high);
        } else if(vs.size() > 3) {
            Serial.println("Diff calibration does not accept sensor/reference value");
            return H4_CMD_TOO_MANY_PARAMS;
        }
    }
    return H4_CMD_OK;
}

// Measure for 5s and average, when button is pressed
float measure_gas(String gas, int sensor){
  float gas_measured = float(NAN);
  if((gas == "all") || (gas == "") || (sensor == -9999)){
    return(float(NAN));
  }
  // Note that the sensor list is 0-indexed, i.e. the number needs to be -1
  //sensor -= 1;

  if(gas == "co2"){
  #if USE_SCD30
    gas_measured = scd_sensors[sensor]->airCO2();
  #endif
  }
  if (gas == "o2"){
  #if USE_SEN0465
    gas_measured = sen_sensors[sensor]->airO2();
  #endif
  }
  if (gas == "h2o"){
  #if USE_BME280
    // This sensor measures RH, but that's not useful for calibration, so the mole fraction has to be calculated
    gas_measured = env.air_water_mole_frac(bme_sensors[sensor]->airT(),
                                           bme_sensors[sensor]->airRH(),
                                           bme_sensors[sensor]->airP());
  #endif
  }
  if (gas == "temperature"){
  #if USE_BME280
    gas_measured = bme_sensors[sensor]->airT();
  #endif
  }
  if (gas == "pressure"){
  #if USE_BME280
    gas_measured = bme_sensors[sensor]->airP();
  #endif
  }
  
  return(gas_measured);
}

void onViewersConnect() {
  viewersConnected = true;
	Serial.printf("onViewersConnect\n");
  h4wifi.uiAddText("Timestamp (UTC)", "");
  h4wifi.uiAddText("Location", "");
  h4wifi.uiAddText("Battery", "");
  h4wifi.uiAddText("---","");

  h4wifi.uiAddText("Zero & Span","Calibration");
  h4wifi.uiAddDropdown("Gas Sensor",{
    {"All","all"},
    {"H₂O","h2o"},
    {"CO₂","co2"},
    {"O₂", "o2"},
    {"Temperature", "temperature"},
  });
  h4wifi.uiAddDropdown("Sensor Number",{
    {"-9999","-9999"},
    {"0","0"},
    {"1","1"}
  });
  h4wifi.uiAddInput("Zero gas");
  h4wifi.uiAddImgButton("setzero");
  h4wifi.uiAddInput("Span gas");
  h4wifi.uiAddImgButton("setspan");
  Serial.print("Current Gas: "); Serial.println(currentGas);
  h4wifi.uiAddText("Gas",currentGas.c_str());
  h4wifi.uiAddText("Sensor",currentSensor);
  h4wifi.uiAddText("Zero value",CSTR(String(cal.read_calibration_var("sen", "zero", currentGas, currentSensor))));
  h4wifi.uiAddText("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
  h4wifi.uiAddText("Span value",CSTR(String(cal.read_calibration_var("sen", "span", currentGas, currentSensor))));
  h4wifi.uiAddText("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
  h4wifi.uiAddText("Raw value", "");
  h4wifi.uiAddText("Calibrated value", "");

  h4wifi.uiAddText("----","");
  h4wifi.uiAddText("Alternative:","Δ Calibration");
  h4wifi.uiAddDropdown("Δ Type",{
    {"All","all"},
    {"H₂O","h2o"},
    {"CO₂","co2"},
    {"O₂", "o2"},
    {"Temperature", "temperature"},
    {"Pressure", "pressure"},
  });
  h4wifi.uiAddImgButton("setdiffLow");
  h4wifi.uiAddImgButton("setdiffHigh");
  h4wifi.uiAddText("Δ Gas",currentDiffGas.c_str());
  h4wifi.uiAddText("Sen. 0 value","");
  h4wifi.uiAddText("Sen. 1 value","");
  h4wifi.uiAddText("Sen. 1 (Cal.)","");
  
  h4wifi.uiAddText("---","");
  h4wifi.uiAddImgButton("reset");
  
  // Every second, update the UI (only while someone is connected to the webUI)
  TIM0=h4.every(1000,[](){
    #if USE_GPS
    if (!isGPSDateValid()) {
      h4wifi.uiSetValue("Timestamp (UTC)","NO GPS SIGNAL");
      h4wifi.uiSetValue("Location","NO GPS SIGNAL");
    } else {
      h4wifi.uiSetValue("Timestamp (UTC)",CSTR(gps.get_timestamp()));
      h4wifi.uiSetValue("Location",CSTR(gps.get_short_location()));
    }
    #else
    h4wifi.uiSetValue("Timestamp (UTC)","No GPS module");
    #endif
    float measured_value = measure_gas(currentGas, currentSensor);
    Cal::CalibrationResult cal_result = cal.calibrate_linear(currentGas, currentSensor, measured_value);
    //float calibrated_value = cal.calibrate_linear(currentGas, currentSensor, measured_value);
    h4wifi.uiSetValue("Battery",CSTR(String(bat.battery_pc())));
    if(!calibration_running){
      h4wifi.uiSetValue("Raw value",CSTR(String(measured_value)));
      h4wifi.uiSetValue("Calibrated value",CSTR(String(cal_result.calibratedValue)));
      h4wifi.uiSetValue("Sen. 0 value",CSTR(String(measure_gas(currentDiffGas, 0))));
      h4wifi.uiSetValue("Sen. 1 value",CSTR(String(measure_gas(currentDiffGas, 1))));
    }
    measured_value = measure_gas(currentDiffGas, 1);
    //calibrated_value = cal.calibrate_linear(currentDiffGas, 1, measured_value);
    cal_result = cal.calibrate_linear(currentDiffGas, 1, measured_value);
    h4wifi.uiSetValue("Sen. 1 (Cal.)",CSTR(String(cal_result.calibratedValue)));
  });       
}

void onViewersDisconnect() {
	Serial.printf("onViewersDisconnect\n");
  h4.cancel({TIM0}); // Stop updating the UI
  viewersConnected = false;
}

// TODO: Can I use this?
void h4_set_ui_text(std::string  field, std::string  value){
  h4wifi.uiSetValue(field, value);
}

void save_calibration_coefficients(void){
  // Save calibration data to file
  String cal_data_str = "";
  String cal_header = "";
  #if USE_GPS
    if (!isGPSDateValid()) {
      h4wifi.uiMessage("Error: No GPS fix");
      Serial.print(F("Invalid GPS date"));
      return;
    }
    String calibration_fn = "calibration_" + gps.get_date() + ".csv";
    cal_header   = "timestamp_utc,";
    cal_data_str = gps.get_timestamp() + ",";
  #else
    String data_fn = "calibration.csv";
  #endif
  cal_header   += cal.get_all_cal_header(gases, numSensors);
  cal_data_str += cal.get_all_cal_data(gases, numSensors);
  #if USE_MICROSD
    // Write data to disk
    sd.write_data(calibration_fn.c_str(),
                  cal_header.c_str(),
                  cal_data_str.c_str(),
                  86400); // logging max 1x/day, used to calculate space on the SD
                          // (should always be enough as there is only little data from calibrations)
  #endif
  return;
}

void absolute_calibration(const std::vector<String> gas_list, String abs_cal_type, int sensor, float ref_value = 0.0){
  // Don't do a span calibration for all gases at once
  if((abs_cal_type == "span") && (currentGas == "all")){ return; }
  // Don't run another calibration if it's already running
  if (calibration_running) {
    Serial.println("Calibration already running - ignoring request");
    return;
  }
  calibration_running = true;

  // Debug, show reference value
  Serial.print("Ref.: "); Serial.println(ref_value);

  // Decide how many sensors we are reading:
  // - When multiple gases are passed, 'sensor' parameter holds # sensors to probe
  // - otherwise treat sensor as a single sensor id (so num_sensors = 1)
  const size_t num_gases = gas_list.size();
  int num_sensors = (num_gases > 1) ? sensor+1 : 1;
  if(num_sensors <= 0) num_sensors = 1; // Safeguard

  // shared accumulators (captured into lambdas by value), multiplying gases by number of sensors
  auto cumulative_data = std::make_shared<std::vector<float>>(num_gases * num_sensors, 0.0f);
  const int n_measurements = 10;
  
  h4.nTimesRandom(
        // Measure "n_measurements" times (every ~1.0-1.1s) & average
        n_measurements, 1000, 1100,
        // Run at every iteration: measure every gas and accumulate
        [gas_list, cumulative_data, num_sensors, sensor,abs_cal_type]() {
            for (size_t gas = 0; gas < gas_list.size(); gas++) { // Cycle through all gases
              if (num_sensors > 1) { // sensor is the number of sensors to be measured
                for (int s = 0; s < num_sensors; s++) { // measure all sensors for this gas
                  size_t idx = gas * num_sensors + s; // index into flattened vector
                  (*cumulative_data)[idx] += measure_gas(gas_list[gas], s);
                }
              } else { // If only 1 gas was given, num_sensors is 1, the for loop is executed once
                // "sensor" is the sensor ID and is the one measured here
                size_t idx = gas * num_sensors + 0; // This always creates index 0
                (*cumulative_data)[idx] += measure_gas(gas_list[gas], sensor);
              }
            }
            if((abs_cal_type == "zero") && (viewersConnected == true)){
              h4wifi.uiSetValue("Zero value","In progress");
              h4wifi.uiSetValue("Zero reference","In progress");
            } else if (viewersConnected == true) {
              h4wifi.uiSetValue("Span value","In progress");
              h4wifi.uiSetValue("Span reference","In progress");
            }
        },
        // Run when timer finishes: compute averages & store/set differential for each gas
        [gas_list, cumulative_data, sensor, num_sensors, n_measurements, abs_cal_type, ref_value]() {
            const unsigned long ts = gps.seconds_since_midnight(); // single timestamp
            for (size_t gas = 0; gas < gas_list.size(); gas++) {
              for (int s = 0; s < num_sensors; s++) {
                float ref_val = 0;
                if(gas_list[gas] == "h2o"){
                  ref_val = env.dewpoint_to_mole_frac(ref_value, // Dewpoint in °C
                                           bme_sensors[s]->airP()); // Convert dewpoint to H2O mole fraction [mmol/mol]
                } else {
                  ref_val = ref_value;
                }
                size_t idx = gas * num_sensors + s; // index into flattened vector
                float sensor_measured = (*cumulative_data)[idx] / static_cast<float>(n_measurements);
                // set differential for each gas
                cal.set_calibration_coeff(abs_cal_type, gas_list[gas], s, ref_val, sensor_measured, ts);
                  
                Serial.print("Gas: "); Serial.println(gas_list[gas]);
                Serial.print("  Sensor: "); Serial.println(s);
                Serial.print("    Value: "); Serial.println(sensor_measured);
                Serial.print("    Reference: "); Serial.print(ref_val);
                if(gas_list[gas] == "h2o"){Serial.println(" [mmol/mol]");}else{Serial.println("");}
                if((abs_cal_type == "zero") && (viewersConnected == true)){
                  h4wifi.uiSetValue("Zero value", CSTR(String(sensor_measured, 2)));
                  h4wifi.uiSetValue("Zero reference", CSTR(String(ref_val, 2)));
                } else if (viewersConnected == true) {
                  h4wifi.uiSetValue("Span value", CSTR(String(sensor_measured, 2)));
                  h4wifi.uiSetValue("Span reference", CSTR(String(ref_val, 2)));
                }
            }
            // reset accumulators (may be optional since shared_ptr is freed after lambda ends)
            std::fill(cumulative_data->begin(), cumulative_data->end(), 0.0f);

            if(abs_cal_type == "span"){
              save_calibration_coefficients();
            }
            calibration_running = false;
          }
        },
        0, false
    );
}

void onsetzeroButton(){
  h4wifi.uiMessage("Zero Calibration");
  Serial.println("Zero Calibration");

  if(currentSensor == -9999){ return; }
  
  if(currentGas == "all"){
    // If all gases are zero'd, then set to them to exactly 0.0
    absolute_calibration(gases, "zero", numSensors, 0.0);
  } else {
    String input_field = "Zero gas"; // Creates "Zero gas" or "Span gas" (note the upper-case)
    float ref_value = std::stof(std::string(h4p.gvGetstring(input_field.c_str()).c_str()));
    absolute_calibration({currentGas}, "zero", currentSensor, ref_value);
  }
}

void onsetspanButton(){
  h4wifi.uiMessage("Span Calibration");
  Serial.println("Span Calibration");

  if(currentSensor == -9999){ return; }

  if(currentGas == "all"){
    h4wifi.uiMessage("ERROR: Please calibrate span individually");
    Serial.println("ERROR: Please calibrate span individually");
    // Note: This will return, doing nothing (because span can't be set for different gases at once)
    absolute_calibration(gases, "span", numSensors, float(NAN));
  } else {
    String input_field = "Zero gas"; // Creates "Zero gas" or "Span gas" (note the upper-case)
    float ref_value = std::stof(std::string(h4p.gvGetstring(input_field.c_str()).c_str()));
    absolute_calibration({currentGas}, "span", currentSensor, ref_value);
  }

}

// Differential calibration
void differential_calibration(const std::vector<String> gas_list, String dif_cal_type){
  //h4wifi.uiMessage("Differential calibration running...");
  //Serial.println("Differential calibration running...");
  if (calibration_running) {
    Serial.println("Calibration already running - ignoring request");
    return;
  }
  calibration_running = true;

  // shared accumulators (captured into lambdas by value)
  auto cumulative_s1 = std::make_shared<std::vector<float>>(gas_list.size(), 0.0f);
  auto cumulative_s2 = std::make_shared<std::vector<float>>(gas_list.size(), 0.0f);
  const int n_measurements = 10;

    h4.nTimesRandom(
        // Measure "n_measurements" times (every ~1.0-1.1s) & average
        n_measurements, 1000, 1100,
        // Run at every iteration: measure every gas and accumulate
        [gas_list, cumulative_s1, cumulative_s2]() {
            for (size_t i = 0; i < gas_list.size(); ++i) {
                (*cumulative_s1)[i] += measure_gas(gas_list[i], 0);
                (*cumulative_s2)[i] += measure_gas(gas_list[i], 1);
            }
            h4wifi.uiSetValue("Sen. 0 value", "In progress");
            h4wifi.uiSetValue("Sen. 1 value", "In progress");
        },
        // Run when timer finishes: compute averages & store/set differential for each gas
        [gas_list, cumulative_s1, cumulative_s2, n_measurements, dif_cal_type]() {
            const unsigned long ts = gps.seconds_since_midnight(); // single timestamp
            for (size_t i = 0; i < gas_list.size(); ++i) {
                float avg0 = (*cumulative_s1)[i] / n_measurements;
                float avg1 = (*cumulative_s2)[i] / n_measurements;
                // set differential for each gas
                cal.set_differential_coeff(dif_cal_type, gas_list[i], avg0, avg1, ts);

                Serial.print("Gas: "); Serial.println(gas_list[i]);
                Serial.print("Sen. 0 avg: "); Serial.println(avg0);
                Serial.print("Sen. 1 avg: "); Serial.println(avg1);
            }

            // reset accumulators (optional since shared_ptr is freed after lambda ends)
            for (size_t i = 0; i < gas_list.size(); ++i) { (*cumulative_s1)[i] = 0.0f; (*cumulative_s2)[i] = 0.0f; }

            if(viewersConnected == true){
              h4wifi.uiSetValue("Sen. 0 value", "");
              h4wifi.uiSetValue("Sen. 1 value", "");
            }

            if(dif_cal_type == "sen"){
              save_calibration_coefficients();
            }
            calibration_running = false;
        },
        0, false
    );
}

void onsetdiffLowButton(){
  h4wifi.uiMessage("Differential calibration (low value) running...");
  Serial.println("Differential calibration (low)");

  if(currentDiffGas == "all"){
    differential_calibration(gases, "ref"); // "sen" is for the high calibration, "ref" for low
  } else {
    differential_calibration({currentDiffGas}, "ref");  // "sen" is for the high calibration, "ref" for low
  }
}

void onsetdiffHighButton(){
  h4wifi.uiMessage("Differential calibration (high value) running...");
  Serial.println("Differential calibration (high)");

  if(currentDiffGas == "all"){
    differential_calibration(gases, "sen"); // "sen" is for the high calibration, "ref" for low
  } else {
    differential_calibration({currentDiffGas}, "sen");  // "sen" is for the high calibration, "ref" for low
  }
}

void onresetButton(){
  h4wifi.uiMessage("RESET");
  Serial.println("RESET");
  cal.reset_all_calibrations(gases, numSensors);
}

H4P_EventListener allexceptmsg(H4PE_VIEWERS | H4PE_GVCHANGE,[](const std::string& svc,H4PE_TYPE t,const std::string& msg){
  switch(t){
    case H4PE_VIEWERS:
      H4P_SERVICE_ADAPTER(Viewers);
      break;
    case H4PE_GVCHANGE:
      // Connect buttons to relevant functions
      H4P_TACT_BUTTON_CONNECTOR(setspan);
      H4P_TACT_BUTTON_CONNECTOR(setzero);
      H4P_TACT_BUTTON_CONNECTOR(setdiffLow);
      H4P_TACT_BUTTON_CONNECTOR(setdiffHigh);
      H4P_TACT_BUTTON_CONNECTOR(reset);
      // Process button presses
      if(svc=="Gas Sensor"){ // Test Gas Sensor selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Gas",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which gas was chosen
        currentGas = msg.c_str();
        Serial.print("currentGas, currentSensor (GAS): "); Serial.print(currentGas); Serial.print(" "); Serial.println(currentSensor);
        h4wifi.uiSetValue("Zero value",CSTR(String(cal.read_calibration_var("sen", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span value",CSTR(String(cal.read_calibration_var("sen", "span", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
        h4p.gvSetstring("Zero gas",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor)))); // Change input field value when dropdown selection changes
        h4p.gvSetstring("Span gas",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor)))); // Change input field value when dropdown selection changes
        break;
      }
      if(svc=="Sensor Number"){ // Test sensor number selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Sensor",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which sensor was chosen & convert to int
        String msg_str = msg.c_str();
        currentSensor = msg_str.toInt();
        Serial.print("currentGas, currentSensor (#): "); Serial.print(currentGas); Serial.print(" "); Serial.println(currentSensor);
        h4wifi.uiSetValue("Zero value",CSTR(String(cal.read_calibration_var("sen", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span value",CSTR(String(cal.read_calibration_var("sen", "span", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
        // Change input field default value when dropdown selection changes
        h4p.gvSetstring("Zero gas",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
        h4p.gvSetstring("Span gas",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
        break;
      }

      if(svc=="Δ Type"){ // Test differential gas selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Δ Gas",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which gas was chosen
        currentDiffGas = msg.c_str();
        break;
      }
      break;
  }
});

/*
void onRTC(){
  Serial.println("Clock valid!");
  ClockValid = true;
  Serial.printf("\nReceived NTP time: %s (UTC)\n\n", CSTR(h4tk.strfDateTime( "%a %Y-%m-%d %H:%M:%S", h4tk.clockEPOCHLocal()) ));
}
*/

/*
void h4pGlobalEventHandler(const std::string& svc,H4PE_TYPE t,const std::string& msg)
{
	switch (t)
	{
		H4P_DEFAULT_SYSTEM_HANDLER;
	case H4PE_SERVICE:
		H4P_SERVICE_ADAPTER(WiFi);
#if USE_MQTT
		H4P_SERVICE_ADAPTER(MQTT);
#endif
		break;
	case H4PE_VIEWERS:
		H4P_SERVICE_ADAPTER(Viewers);
		break;
	default:
		break;
	}
}
*/

void processDataBuffered(void){
  CsvBuffer row;
  csv_init(row, data_row_buf, sizeof(data_row_buf));
  Cal::CalibrationResult cal_result;
  uint32_t gps_chars_this_sample = 0;
  bool gps_date_valid = false;
  bool gps_time_fresh = false;
  bool gps_location_valid = false;
  bool gps_location_fresh = false;

#if USE_GPS
  gps_chars_this_sample = gps.update_values();
  gps_date_valid = isGPSDateValid();
  gps_time_fresh = isGPSTimeFresh();
  gps_location_valid = gps.locationValid();
  gps_location_fresh = isGPSLocationFresh();
  if (gps_time_fresh) {
    String date = gps.get_date();
    strncpy(last_valid_gps_date, date.c_str(), sizeof(last_valid_gps_date) - 1);
    last_valid_gps_date[sizeof(last_valid_gps_date) - 1] = '\0';
    gps_boot_locked = true;
  }
  if (!gps_boot_locked) {
    Serial.println(F("Waiting for first fresh GPS date/time before logging"));
    return;
  }
  if (!gps_time_fresh && gps_chars_this_sample == 0) {
    gps_stale_count++;
    if (!gps_stale_active && gps_stale_count >= GPS_STALE_CONSECUTIVE_SAMPLES) {
      gps_stale_active = true;
      gps_stale_status_pending = true;
    }
    uint32_t now_ms = millis();
    if (gps_stale_count >= GPS_STALE_CONSECUTIVE_SAMPLES &&
        (last_gps_recovery_ms == 0 || now_ms - last_gps_recovery_ms >= GPS_RECOVERY_COOLDOWN_MS)) {
      recoverGPS();
    }
  } else if (gps_time_fresh) {
    if (gps_stale_active) gps_recovered_status_pending = true;
    gps_stale_active = false;
    gps_stale_count = 0;
  }
#else
  gps_boot_locked = true;
#endif

  if (!data_header_ready) build_data_header();
  sample_counter++;

  uint16_t row_status = last_data_write_status;
  uint32_t now_ms = millis();
  uint32_t uptime_ms = now_ms - boot_ms;
  const char* timestamp_calc_source = "";
  uint32_t timestamp_calc_epoch = calculated_timestamp_epoch(gps_time_fresh, now_ms, &timestamp_calc_source);
  bool timestamp_calc_valid = timestamp_calc_epoch > 0;
  if (timestamp_calc_valid) {
    format_utc_timestamp(timestamp_calc_epoch, timestamp_calc_buf, sizeof(timestamp_calc_buf));
    format_utc_date(timestamp_calc_epoch, timestamp_file_date_buf, sizeof(timestamp_file_date_buf));
  } else {
    timestamp_calc_buf[0] = '\0';
    timestamp_file_date_buf[0] = '\0';
  }

  csv_uint(row, boot_id);
  csv_uint(row, sample_counter);
  csv_uint(row, uptime_ms);
  csv_uint(row, now_ms);
  csv_field(row, reset_reason_buf);

#if USE_GPS
  csv_int(row, gps_date_valid ? 1 : 0);
  csv_int(row, gps_time_fresh ? 1 : 0);
  csv_int(row, gps_location_valid ? 1 : 0);
  csv_int(row, gps_location_fresh ? 1 : 0);
  csv_uint(row, gps_chars_this_sample);
  csv_uint(row, gps.gpsAgeMs());
  csv_uint(row, gps.locationAgeMs());
  csv_uint(row, gps_stale_count);
  csv_uint(row, gps_recovery_count);
  if (gps_time_fresh) {
    csv_field(row, timestamp_calc_buf);
  } else {
    csv_field(row, "");
  }
  csv_field(row, timestamp_calc_buf);
  csv_field(row, timestamp_calc_source);
  if (gps_location_fresh) {
    csv_float(row, gps.get_lat(), 8);
    csv_float(row, gps.get_lon(), 8);
  } else {
    csv_field(row, "");
    csv_field(row, "");
  }
  if (gps_location_fresh && gps.altitudeValid()) csv_float(row, gps.get_alt(), 2);
  else csv_field(row, "");
  csv_uint(row, gps.satellites());
  csv_float(row, gps.hdop(), 2);
#endif

#if USE_BATTERY
  csv_int(row, bat.battery_mV());
  csv_int(row, bat.battery_pc());
#endif

#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
#else
  size_t n = 1;
#endif
  for (size_t i = 0; i < n; ++i) {
    uint32_t bus_start_ms = millis();
    bool bus_error = false;

#if USE_ADS1115
    float ads_in = ads_sensors[i]->read_val(1, 16, 25.83);
    float ads_out = ads_sensors[i]->read_val(2, 16, 30.98);
    bus_error |= isnan(ads_in) || isnan(ads_out);
    csv_float(row, ads_in, 2);
    csv_float(row, ads_out, 2);
#endif

#if USE_BME280
    float bme_temp = bme_sensors[i]->airT();
    float bme_rh   = bme_sensors[i]->airRH();
    float bme_p    = bme_sensors[i]->airP();
    float bme_h2o  = env.air_water_mole_frac(bme_temp, bme_rh, bme_p);
    bus_error |= isnan(bme_temp) || isnan(bme_rh) || isnan(bme_p) || isnan(bme_h2o);
    csv_float(row, bme_temp, 2);
    csv_float(row, bme_rh, 2);
    csv_float(row, bme_p, 2);
    csv_float(row, bme_h2o, 2);
  #if USE_CAL
    float rh_flag = 0;
    cal_result = cal.calibrate_linear("temperature", i, bme_temp);
    float cal_temp = cal_result.calibratedValue;
    csv_float(row, cal_temp, 2);
    csv_int(row, cal_result.flag);
    rh_flag += cal_result.flag;
    cal_result = cal.calibrate_linear("pressure", i, bme_p);
    float cal_p = cal_result.calibratedValue;
    csv_float(row, cal_p, 2);
    csv_int(row, cal_result.flag);
    rh_flag += cal_result.flag;
    cal_result = cal.calibrate_linear("h2o", i, bme_h2o);
    float cal_h2o = cal_result.calibratedValue;
    csv_float(row, cal_h2o, 2);
    csv_int(row, cal_result.flag);
    rh_flag += cal_result.flag;
    float cal_rh = env.air_relative_humidity(cal_temp, cal_p, cal_h2o);
    csv_float(row, cal_rh, 2);
    csv_float(row, rh_flag, 2);
  #else
    csv_float(row, bme_temp, 2);
    csv_float(row, bme_rh, 2);
    csv_float(row, bme_p, 2);
    csv_float(row, bme_h2o, 2);
  #endif
#endif

#if USE_SCD30
  #if USE_BME280
    if (sample_counter % 3 == 0) scd_sensors[i]->set_air_pressure(bme_p);
  #endif
    scd_sensors[i]->getData();
    float scd_T   = scd_sensors[i]->airT();
    float scd_RH  = scd_sensors[i]->airRH();
    float scd_CO2 = scd_sensors[i]->airCO2();
    bus_error |= isnan(scd_T) || isnan(scd_RH) || isnan(scd_CO2);
    csv_float(row, scd_T, 2);
    csv_float(row, scd_RH, 2);
    csv_float(row, scd_CO2, 2);
  #if USE_CAL
    cal_result = cal.calibrate_linear("co2", i, scd_CO2);
    csv_float(row, cal_result.calibratedValue, 2);
    csv_int(row, cal_result.flag);
  #else
    csv_float(row, scd_CO2, 2);
  #endif
#endif

#if USE_SEN0465
    float sen_T = sen_sensors[i]->airT();
    float sen_O2 = sen_sensors[i]->airO2();
    float sen_raw_v = sen_sensors[i]->rawV();
    bus_error |= isnan(sen_T) || isnan(sen_O2) || isnan(sen_raw_v);
    csv_float(row, sen_T, 2);
    csv_float(row, sen_O2, 4);
    csv_float(row, sen_raw_v, 6);
  #if USE_CAL
    cal_result = cal.calibrate_linear("o2", i, sen_O2);
    csv_float(row, cal_result.calibratedValue, 4);
    csv_int(row, cal_result.flag);
  #else
    csv_float(row, sen_O2, 4);
  #endif
#endif

#if USE_MLX90614
    float mlx_t = mlx_sensors[i]->airT();
    float mlx_obj_t = mlx_sensors[i]->objT();
    bus_error |= isnan(mlx_t) || isnan(mlx_obj_t);
    csv_float(row, mlx_t, 2);
    csv_float(row, mlx_obj_t, 2);
#endif

    record_i2c_health(i, millis() - bus_start_ms, bus_error);
  }

  csv_uint(row, heap_free_now());
  csv_uint(row, heap_max_block_now());
  csv_uint(row, heap_min_block_now());
  csv_uint(row, i2c_slow_total());
  csv_uint(row, i2c_error_total());
  csv_uint(row, i2c_recovery_total());
  for (size_t i = 0; i < sizeof(i2c_error_count) / sizeof(i2c_error_count[0]); i++) {
    csv_uint(row, i2c_error_count[i]);
    csv_uint(row, i2c_consecutive_error_count[i]);
    csv_uint(row, i2c_recovery_count[i]);
  }
  if (row.truncated) row_status |= ROW_STATUS_TRUNCATED;
  csv_uint(row, row_status);
  if (row.truncated) row_status |= ROW_STATUS_TRUNCATED;

#if USE_GPS
  char data_fn[32];
  const char* data_file_date = timestamp_file_date_buf[0] ? timestamp_file_date_buf : last_valid_gps_date;
  snprintf(data_fn, sizeof(data_fn), "data_%s.csv", data_file_date);
#else
  const char* data_fn = "data.csv";
#endif

#if USE_MICROSD
  uint16_t write_status = sd.write_data(data_fn,
                                        data_header,
                                        data_row_buf,
                                        MEASUREMENT_INTERVAL);
  last_data_write_status = write_status;
  if (write_status != MicroSD::WRITE_OK) sd_status_pending = true;
  if (write_status == MicroSD::WRITE_OK && (!boot_status_written || sd_status_pending)) {
    uint16_t status_write_status = write_status_row(boot_status_written ? "sd_recovered" : "boot", write_status);
    if (status_write_status == MicroSD::WRITE_OK) {
      boot_status_written = true;
      sd_status_pending = false;
    } else {
      sd_status_pending = true;
    }
  }
  if (write_status == MicroSD::WRITE_OK && gps_stale_status_pending) {
    if (write_status_row("gps_stale", write_status) == MicroSD::WRITE_OK) gps_stale_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && gps_recovered_status_pending) {
    if (write_status_row("gps_recovered", write_status) == MicroSD::WRITE_OK) gps_recovered_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && i2c_recovery_status_pending) {
    if (write_status_row("i2c_recovery", write_status) == MicroSD::WRITE_OK) i2c_recovery_status_pending = false;
  }
#endif

  Serial.println(data_row_buf);
}

// Collect measurements
void processData(void){
  processDataBuffered();
}

#if 0
void processDataLegacyDisabled(void){
  // Prepare output string
  String data_str = "";

  // Measurement count
  int n_measurements = 0;
  String header = "";
  String cal_header = "";

  Cal::CalibrationResult cal_result;

#if USE_GPS
  // Add timestamps and location
  gps.update_values();
  data_str = data_str + gps.get_timestamp() + ",";
  data_str = data_str + gps.get_location() + ",";
  header += "timestamp_utc,lat,lon,alt,nb_sat,HDOP,";
#endif

#if USE_BATTERY
  header += "bat.mV,bat.perc,";
  data_str += String(bat.battery_mV()) + ",";    // Battery charge     [mV]
  data_str += String(bat.battery_pc()) + ",";    // Battery charge     [%]
  n_measurements += 2;
#endif

  // Go through i2c buses
#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
  for (size_t i = 0; i < n; ++i) {
#endif
  
  #if USE_ADS1115
    header += "Sin.W_m2,Sout.W_m2,";
    data_str += String(ads_sensors[i]->read_val(1, 16, 25.83), 2) + ","; // ADC of Apogee SP-510, A1, Gain 16: ±0.256V, convert by 25.83 W m-2 / mV
    data_str += String(ads_sensors[i]->read_val(2, 16, 30.98), 2) + ","; // ADC of Apogee SP-610, A2, Gain 16: ±0.256V, convert by 30.98 W m-2 / mV
    n_measurements += 2;
  #endif
  #if USE_BME280
    // Read data
    float bme_temp = bme_sensors[i]->airT();
    float bme_rh   = bme_sensors[i]->airRH();
    float bme_p    = bme_sensors[i]->airP();
    float bme_h2o  = env.air_water_mole_frac(bme_temp, bme_rh, bme_p); // H2O mole fraction [mmol/mol]
    // Uncalibrated values
    header += "bme_T.C,bme_RH.perc,bme_P.Pa,bme_H2O.mmol_mol,";
    data_str += String(bme_temp, 2) + ",";    // T                  [°C]
    data_str += String(bme_rh, 2) + ",";      // RH                 [%]
    data_str += String(bme_p, 2) + ",";       // Pressure           [Pa]
    data_str += String(bme_h2o, 2) + ",";     // H2O mole fraction  [mmol/mol]
    n_measurements += 4;
    #if USE_CAL
      header += "bme_T.C.cal,bme_T.flag,bme_P.Pa.cal,bme_P.flag,bme_H2O.mmol_mol.cal,bme_H2O.mmol_mol.flag,bme_RH.perc.cal,bme_RH.flag,";
      float rh_flag = 0;
      // Calibrated temperature
      cal_result = cal.calibrate_linear("temperature", i, bme_temp);
      float cal_temp = cal_result.calibratedValue;
      data_str += String(cal_temp, 2) + ",";                        // Temperature        [C]
      data_str += String(cal_result.flag) + ",";                    // Data quality flag after calibration
      rh_flag += cal_result.flag;
      // Calibrated pressure
      cal_result = cal.calibrate_linear("pressure", i, bme_p);
      float cal_p = cal_result.calibratedValue;
      data_str += String(cal_p, 2) + ",";      // Pressure
      data_str += String(cal_result.flag) + ",";                    // Data quality flag after calibration
      rh_flag += cal_result.flag;
      // Calibrated H2O mole fraction
      cal_result = cal.calibrate_linear("h2o", i, bme_h2o);
      float cal_h2o = cal_result.calibratedValue;
      data_str += String(cal_h2o, 2) + ",";     // H2O mole fraction [mmol/mol]
      data_str += String(cal_result.flag) + ",";                   // Data quality flag after calibration
      rh_flag += cal_result.flag;
      // Calibrated RH, calculate from calibrated T, P and H2O
      float cal_rh = env.air_relative_humidity(cal_temp, cal_p, cal_h2o);
      data_str += String(cal_rh, 2) + ",";      // RH
      data_str += String(rh_flag) + ",";// Data quality flag after calibration, combining the 3 existing flags
      n_measurements += 4;
      // FLAGS
      // -1 Indicates an error, invalid data
      // 2  Indicates a simple offset
      // 4  Indicates an error, bad data, return raw data
    #else
      header += "bme_T.C,bme_RH.perc,bme_P.Pa,bme_H2O.mmol_mol,";
      data_str += String(bme_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
      data_str += String(bme_sensors[i]->airRH(), 2) + ",";         // RH                 [%]
      data_str += String(bme_sensors[i]->airP(), 2) + ",";          // Pressure           [Pa]
      float bme_h2o = env.air_water_mole_frac(bme_sensors[i]->airT(),
                                           bme_sensors[i]->airRH(),
                                           bme_sensors[i]->airP()); // H2O mole fraction [mmol/mol]
      data_str += String(bme_h2o, 2) + ",";     // H2O mole fraction [mmol/mol]

      n_measurements += 4;
    #endif
  #endif
  #if USE_SCD30
    #if USE_BME280
    // Always update air pressure in CO2 sensor before using it
    scd_sensors[i]->set_air_pressure(bme_sensors[i]->airP());
    #endif
    scd_sensors[i]->getData(); // Read the sensor and store the values
    // Read data
    float scd_T   = scd_sensors[i]->airT();
    float scd_RH  = scd_sensors[i]->airRH();
    float scd_CO2 = scd_sensors[i]->airCO2();
    header += "scd_T.C,scd_RH.perc,scd_CO2.ppm,";
    data_str += String(scd_T, 2) + ",";          // Temperature        [C]
    data_str += String(scd_RH, 2) + ",";         // RH                 [%]
    data_str += String(scd_CO2, 2) + ",";        // CO2                [ppm]
    n_measurements += 3;
    #if USE_CAL
      header += "scd_CO2.ppm.cal,scd_CO2.flag,";
      cal_result = cal.calibrate_linear("co2", i, scd_CO2);
      data_str += String(cal_result.calibratedValue, 2) + ",";      // CO2 concentration  [ppm]
      data_str += String(cal_result.flag) + ",";                    // Data quality flag after calibration
      n_measurements += 2;
    #else
      header += "scd_CO2.ppm,";
      data_str += String(scd_sensors[i]->airCO2(), 2) + ",";      // CO2 concentration  [ppm]
      n_measurements += 1;
    #endif
  #endif
  #if USE_SEN0465
    header += "sen_T.C,sen_O2.mmol_mol,";
    // Read data
    float sen_T = sen_sensors[i]->airT();        // Temperature        [C]
    float sen_O2 = sen_sensors[i]->airO2();      // Oxygen             [ppm]
    data_str += String(sen_T, 2) + ",";
    data_str += String(sen_O2, 2) + ",";
    n_measurements += 2;
    #if USE_CAL
      header += "sen_O2.mmol_mol.cal,sen_O2.flag,";
      cal_result = cal.calibrate_linear("o2", i, sen_O2);
      data_str += String(cal_result.calibratedValue, 2) + ",";     // Oxygen             [ppm]
      data_str += String(cal_result.flag) + ",";                   // Data quality flag after calibration
      n_measurements += 2;
    #else
      header += "sen_O2.mmol_mol,";
      data_str += String(sen_sensors[i]->airO2(), 2) + ",";          // Oxygen             [ppm]
      n_measurements += 1;
    #endif
  #endif
  #if USE_MLX90614
    header += "mlx_T.C,mlx_obj_T.C,";
    data_str += String(mlx_sensors[i]->airT(), 2) + ",";          // Air temperature    [C]
    data_str += String(mlx_sensors[i]->objT(), 2) + ",";          // Object temperature [C]
    n_measurements += 2;
  #endif

#if I2C_MULTI
  }
#endif

  // Remove "nan" strings to shorten CSV output
  data_str.replace("nan", "");

  // Save data
  //------------
  // Creates daily file name
  int year = 2020;
#if USE_GPS
  if (!isGPSDateValid()) {
    Serial.println(F("Invalid GPS date"));
    return;
  }
  String data_fn = "data_" + gps.get_date() + ".csv";
#else
  String data_fn = "data.csv";
#endif

#if USE_MICROSD
  // Write data to disk
  sd.write_data(data_fn.c_str(),
                header.c_str(),
                data_str.c_str(),
                MEASUREMENT_INTERVAL); // logging frequency last [s]
#endif
  
  // Debug: Show data on Serial output
  Serial.println(data_str);
}
#endif

void h4setup(){
  // Show info on WIFI
  Serial.print(wifitype);
#if !H4P_USE_WIFI_AP
  Serial.print(F(" connecting to ")); Serial.println(WIFI_SSID);
#else
  Serial.println(F(" connecting")); 
#endif

#if USE_MQTT
	h4p[brokerTag()]=MQTT_SERVER;
	for (int i=0;i<BIG_SIZE;i++) {
		big[i]=i;
	}
#endif

#if H4P_SECURE
#if USE_HTTPREQ && SECURE_HTTPREQ
	auto testRootCA = reinterpret_cast<const uint8_t*>(const_cast<char*>(test_root_ca.c_str()));
	h4ah.secureTLS(testRootCA, test_root_ca.length() + 1);
	Serial.printf("HTTP CERT Validation: %s\n", H4AsyncClient::isCertValid(testRootCA, test_root_ca.length() + 1) ? "SUCCEEDED" : "Failed");
#endif // SECURE_HTTPREQ

#if USE_MQTT && SECURE_MQTT
	auto mqCert = reinterpret_cast<const uint8_t*>(const_cast<char*>(MQTT_CERT.c_str()));
	Serial.printf("MQTT CERT Validation: %s\n", H4AsyncClient::isCertValid(mqCert,MQTT_CERT.length()+1) ? "SUCCEEDED" : "Failed");
	h4mqtt.secureTLS(mqCert, MQTT_CERT.length()+1);
#endif // USE_MQTT

#if SECURE_WEBSERVER
	Serial.printf("WEBSERVER CERT Validation: %s\n", H4AsyncClient::isCertValid((const uint8_t*)WEBSERVER_CERT.c_str(), WEBSERVER_CERT.length() + 1) ? "SUCCEEDED" : "Failed");
	Serial.printf("WEBSERVER KEY Validation: %s\n", H4AsyncClient::isPrivKeyValid((const uint8_t*)WEBSERVER_PRIV_KEY.c_str(), WEBSERVER_PRIV_KEY.length() + 1) ? "SUCCEEDED" : "Failed");
	h4wifi.hookWebserver([](){
		h4wifi.secureTLS((const uint8_t*)WEBSERVER_PRIV_KEY.c_str(), WEBSERVER_PRIV_KEY.length() + 1, 
							NULL, 0,
						(const uint8_t*)WEBSERVER_CERT.c_str(), WEBSERVER_CERT.length() + 1);
		h4wifi.useSecurePort();
	});
#endif // SECURE_WEBSERVER

	h4wifi.authenticate("admin","admin");

#endif // H4P_SECURE

#if USE_MQTT
	h4.every(300, []()
			 {
#if defined(ARDUINO_ARCH_ESP32)
				Serial.printf("H=%u M=%u m=%u S=%u\n", _HAL_freeHeap(MALLOC_CAP_INTERNAL), _HAL_maxHeapBlock(MALLOC_CAP_INTERNAL), _HAL_minHeapBlock(MALLOC_CAP_INTERNAL), uxTaskGetStackHighWaterMark(NULL));
#else
				Serial.printf("H=%u M=%u m=%u\n", _HAL_freeHeap(), _HAL_maxHeapBlock(), _HAL_minHeapBlock());
#endif
				h4p["heap"] = _HAL_freeHeap();
				h4p["pool"] = mbx::pool.size();
				});
#endif

  Serial.println("Starting v"WS_VERSION);
  boot_ms = millis();
#if defined(ARDUINO_ARCH_ESP8266)
  boot_id = ESP.getChipId() ^ micros();
  sanitize_csv_text(ESP.getResetReason(), reset_reason_buf, sizeof(reset_reason_buf));
#else
  boot_id = micros();
  sanitize_csv_text(String("unknown"), reset_reason_buf, sizeof(reset_reason_buf));
#endif

  Serial.println(F(""));
  Serial.println(F("Initialisation:"));

  // Add a Serial command for calibration when Wifi is not available/connected
  h4p.addCmd("cal", 0, 0, cal_cmd);
  h4p.addCmd("set", 0, 0, cal_set_cmd);
  h4p.addCmd("show_cal", 0, 0, cal_show_cmd);

  // Non-i2c devices
#if RUN_TEST
  Serial.print(F("- TEST:                     "));
  Serial.println(test.init() ? F("Success") : F("Failed"));
#endif

  // SPI devices
#if USE_MICROSD
Serial.print(F("- MicroSD:                  "));
  if(! sd.init()){
    Serial.println(F("Failed (SD card missing)"));
  } else {
    Serial.println(F("Success"));
    Serial.print("  Storage capacity:      "); Serial.print(sd.getCapacityMB(), 2); Serial.println(" MB");
    Serial.print("  Free storage capacity: "); Serial.print(sd.getFreeMB(), 2); Serial.println(" MB");
  }
#endif

  // Analogue devices
#if USE_BATTERY
  Serial.print(F("- Battery:                  "));
  Serial.println(bat.init() ? F("Success") : F("Failed (Battery not detected)"));
#endif

  // i2c multiplexer
#if I2C_MULTI
  Serial.print(F("- i2c multiplexer           "));
  if(! mp.init()){
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Success"));
  }
#endif
  Wire.begin();

#if USE_GPS
  Serial.print(F("  - XA1110 GPS:             "));
  Serial.println(gps.init() ? F("Success") : F("Failed"));
#endif

// Creates gas list
#if USE_BME280
  gases.push_back("temperature"); // Add to the gases list
  gases.push_back("h2o"); // Add to the gases list
#endif
#if USE_SCD30
  gases.push_back("co2"); // Add to the gases list
#endif
#if USE_SEN0465
  gases.push_back("o2"); // Add to the gases list
#endif
#if USE_CAL
  cal.init_all_calibrations(gases, numSensors);
#endif

  // Go through i2c buses
#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
  for (size_t i = 0; i < n; ++i) {
    // Enable the i2c bus
    Serial.print(F("- i2c bus "));Serial.print(i2c_buses[i]);Serial.println(F(":"));
#endif

#if USE_BME280
  Serial.print(F("  - BME280 sensor:            "));
  Serial.println(bme_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if USE_SCD30
  Serial.print(F("  - SCD-30 sensor:            "));
  Serial.println(scd_sensors[i]->init() ? F("Success") : F("Failed"));
  scd_sensors[i]->enable_self_calibration(false); // don't calibrate, we want to do that manually
  scd_sensors[i]->set_interval(2);                // minimum every 2s
#endif

#if USE_SEN0465
  Serial.print(F("  - SEN0465 sensor:           "));
  Serial.println(sen_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if USE_ADS1115
  Serial.print(F("  - ADS1115 (Analog-digital): "));
  Serial.println(ads_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if USE_MLX90614
  Serial.print(F("  - MLX90614 sensor:    "));
  Serial.println(mlx_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if I2C_MULTI
  }
#endif

  Serial.println(F("Initialisation completed"));
  Serial.println(F(""));
  
  // Print IP address to Serial
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

  build_data_header();

  // Set up regular measurements
  h4.every(MEASUREMENT_INTERVAL * 1000, processData);

  // Every minute, check all calibrations and fix if necessary
  #if USE_CAL
  h4.every(60 * 1000, [](){cal.fix_all_calibrations(gases, numSensors);});
  #endif
}

/*
void publishDevice(const std::string &topic, const std::string &payload){
	Serial.printf("Publishing %s to %s\n", CSTR(payload), CSTR(topic));
#if USE_MQTT
	h4mqtt.publishDevice(topic, payload, 1);
#endif
}

void publishDevice(const std::string &topic, long long payload){
	publishDevice(topic, stringFromInt(payload, "%lu"));
}
*/
#if USE_HTTPREQ
void HTTPClient() {
#if SECURE_HTTPREQ
	h4ah.GET("https://www.howsmyssl.com/a/check", [](ARMA_HTTP_REPLY reply){
#else
	h4ah.GET("http://jsonplaceholder.typicode.com/todos/1", [](ARMA_HTTP_REPLY reply){
#endif
		auto rCode = reply.httpResponseCode;
		auto response = reply.asJsonstring();
		auto headers = reply.responseHeaders;
		Serial.printf("code %d response %s\n", rCode, response.c_str());

		for (auto &h:headers) {
			Serial.printf("%s : %s\n", h.first.c_str(), h.second.c_str());
		}
		headers.clear();
		publishDevice("response", response);
	});
}
#endif

