#pragma once
// ============================================================================
//  Datalogger framework internals: CSV/time/RTC/heap helpers, I2C+GPS recovery, WiFi/MQTT callbacks
//  Auto-relocated verbatim from the old monolithic main.cpp. Included ONLY by
//  datalogger.h (one translation unit). Beginners never need to open this.
// ============================================================================

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

uint32_t rtc_record_crc(const RtcRestartRecord& record) {
  return record.magic ^ record.version ^ record.epoch ^ record.sample_counter ^ record.reason ^ 0xA5A55A5AUL;
}

bool read_rtc_restart_record(RtcRestartRecord& record) {
#if defined(ARDUINO_ARCH_ESP8266)
  if (!ESP.rtcUserMemoryRead(RTC_RESTART_OFFSET, (uint32_t*)&record, sizeof(record))) return false;
  if (record.magic != RTC_RESTART_MAGIC || record.version != RTC_RESTART_VERSION) return false;
  if (record.crc != rtc_record_crc(record)) return false;
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  epoch_to_utc(record.epoch, year, month, day, hour, minute, second);
  return year >= GPS_YEAR_MIN && year <= GPS_YEAR_MAX;
#else
  (void)record;
  return false;
#endif
}

void write_rtc_restart_record(uint32_t epoch, uint32_t reason) {
#if defined(ARDUINO_ARCH_ESP8266)
  if (epoch == 0) return;
  RtcRestartRecord record;
  record.magic = RTC_RESTART_MAGIC;
  record.version = RTC_RESTART_VERSION;
  record.epoch = epoch;
  record.sample_counter = sample_counter;
  record.reason = reason;
  record.crc = rtc_record_crc(record);
  ESP.rtcUserMemoryWrite(RTC_RESTART_OFFSET, (uint32_t*)&record, sizeof(record));
#else
  (void)epoch;
  (void)reason;
#endif
}

uint32_t gps_epoch_now() {
#if USE_GPS
  return utc_to_epoch(gps.year(), gps.month(), gps.day(), gps.hour(), gps.minute(), gps.second());
#else
  return 0;
#endif
}

bool is_gps_epoch_sane(uint32_t gps_epoch, uint32_t now_ms) {
  if (gps_epoch == 0) return false;
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  epoch_to_utc(gps_epoch, year, month, day, hour, minute, second);
  if (year < GPS_YEAR_MIN || year > GPS_YEAR_MAX) return false;
  // The jump check exists to reject a GPS reading that disagrees wildly with a
  // clock we already trust. A hand-entered clock is NOT one we trust more than
  // GPS: measuring the fix against it would mean a phone set a week wrong
  // rejects the real time forever, which is the precise opposite of what the
  // manual clock is for. So skip the comparison while the anchor is manual - the
  // year window above still applies.
  if (last_fresh_gps_epoch_valid && !manual_time_active) {
    uint32_t expected_epoch = last_fresh_gps_epoch + ((now_ms - last_fresh_gps_millis) / 1000UL);
    uint32_t delta = gps_epoch > expected_epoch ? gps_epoch - expected_epoch : expected_epoch - gps_epoch;
    if (delta > GPS_MAX_TIME_JUMP_SECONDS) return false;
  }
  return true;
}

uint32_t calculated_timestamp_epoch(bool gps_time_fresh, bool gps_time_sane_now, uint32_t now_ms, const char** source) {
#if USE_GPS
  if (gps_time_fresh && gps_time_sane_now) {
    uint32_t gps_epoch = gps_epoch_now();
    // GPS has just taken over from a clock that was set by hand. Work out how far
    // out that clock had drifted before overwriting it, so the correction can be
    // reported once - it is the only way to know after the fact whether the data
    // logged on the manual clock carries a meaningful timestamp error.
    if (manual_time_active) {
      uint32_t was = last_fresh_gps_epoch + ((now_ms - last_fresh_gps_millis) / 1000UL);
      manual_time_jump_s = (long)gps_epoch - (long)was;
      manual_time_jump_pending = true;
      manual_time_active = false;
    }
    last_fresh_gps_epoch = gps_epoch;
    last_fresh_gps_millis = now_ms;
    last_fresh_gps_epoch_valid = true;
    strncpy(boot_time_source_buf, "gps", sizeof(boot_time_source_buf) - 1);
    boot_time_source_buf[sizeof(boot_time_source_buf) - 1] = '\0';
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
  for (size_t i = 0; i < num_i2c_buses; i++) total += i2c_slow_count[i];
  return total;
}

uint32_t i2c_error_total() {
  uint32_t total = 0;
  for (size_t i = 0; i < num_i2c_buses; i++) total += i2c_error_count[i];
  return total;
}

uint32_t i2c_recovery_total() {
  uint32_t total = 0;
  for (size_t i = 0; i < num_i2c_buses; i++) total += i2c_recovery_count[i];
  return total;
}

void build_data_header() {
  CsvBuffer header;
  csv_init(header, data_header, sizeof(data_header));
  csv_append_raw(header, "boot_id,sample_counter,uptime_ms,timestamp_boot_ms,reset_reason,boot_time_source,rtc_time_valid,supervisor_restart_reason,");
#if USE_GPS
  csv_append_raw(header, "gps_date_valid,gps_time_fresh,gps_time_sane,gps_location_valid,gps_location_fresh,gps_chars_processed,gps_age_ms,gps_location_age_ms,gps_stale_count,gps_recovery_count,gps_poll_timeout_count,gps_i2c_recovery_count,gps_quarantine_active,gps_quarantine_count,gps_rejected_time_count,timestamp_utc,timestamp_calc_utc,timestamp_calc_source,lat,lon,alt,nb_sat,HDOP,");
#endif
#if USE_BATTERY
  csv_append_raw(header, "bat.mV,bat.perc,");
#endif

  // Type-major: for each sensor type, one column group per instance, suffixed
  // with the multiplexer channel (.b<N>). This order MUST match the data-row
  // order emitted in dl_sampling.h::processDataBuffered.
#if USE_ADS1115
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    unsigned b = ads_buses[i];
    csv_appendf(header, "Sin.W_m2.b%u,Sout.W_m2.b%u,", b, b);
  }
#endif
#if USE_BME280
  for (size_t i = 0; i < BME_COUNT; ++i) {
    unsigned b = bme_buses[i];
    csv_appendf(header, "bme_T.C.b%u,bme_RH.perc.b%u,bme_P.Pa.b%u,bme_H2O.mmol_mol.b%u,", b, b, b, b);
  #if USE_CAL
    csv_appendf(header, "bme_T.C.cal.b%u,bme_T.flag.b%u,bme_P.Pa.cal.b%u,bme_P.flag.b%u,bme_H2O.mmol_mol.cal.b%u,bme_H2O.mmol_mol.flag.b%u,bme_RH.perc.cal.b%u,bme_RH.flag.b%u,", b, b, b, b, b, b, b, b);
  #else
    csv_appendf(header, "bme_T.C.b%u,bme_RH.perc.b%u,bme_P.Pa.b%u,bme_H2O.mmol_mol.b%u,", b, b, b, b);
  #endif
  }
#endif
#if USE_SCD30
  for (size_t i = 0; i < SCD_COUNT; ++i) {
    unsigned b = scd_buses[i];
    csv_appendf(header, "scd_T.C.b%u,scd_RH.perc.b%u,scd_CO2.ppm.b%u,", b, b, b);
  #if USE_CAL
    csv_appendf(header, "scd_CO2.ppm.cal.b%u,scd_CO2.flag.b%u,", b, b);
  #else
    csv_appendf(header, "scd_CO2.ppm.b%u,", b);
  #endif
  }
#endif
#if USE_SEN0465
  for (size_t i = 0; i < SEN_COUNT; ++i) {
    unsigned b = sen_buses[i];
    csv_appendf(header, "sen_T.C.b%u,sen_O2.mmol_mol.b%u,sen_raw_V.b%u,", b, b, b);
  #if USE_CAL
    csv_appendf(header, "sen_O2.mmol_mol.cal.b%u,sen_O2.flag.b%u,", b, b);
  #else
    csv_appendf(header, "sen_O2.mmol_mol.b%u,", b);
  #endif
  }
#endif
#if USE_MLX90614
  for (size_t i = 0; i < MLX_COUNT; ++i) {
    unsigned b = mlx_buses[i];
    csv_appendf(header, "mlx_T.C.b%u,mlx_obj_T.C.b%u,", b, b);
  }
#endif
  csv_append_raw(header, "free_heap,max_heap_block,min_heap_block,i2c_slow_count,i2c_error_count,i2c_recovery_count,");
  for (size_t i = 0; i < num_i2c_buses; i++) {
    unsigned b = i2c_buses[i];
    csv_appendf(header, "i2c_bus%u_error_count,i2c_bus%u_consecutive_error_count,i2c_bus%u_recovery_count,", b, b, b);
  }
  csv_append_raw(header, "valid_sensor_value_count,missing_sensor_value_count,critical_sensor_loss_count,gps_i2c_hard_fault_count,supervisor_reset_count,");
  csv_append_raw(header, "write_status,");
  data_header_ready = !header.truncated;
}

// ---------------------------------------------------------------------------
//  I2C bus health
//
//  The ESP8266 bit-bangs I2C in software, so every bit-wait can burn up to the
//  clock-stretch limit while a slave holds SCL low. Three of the sensor drivers
//  (SparkFun_I2C_GPS, DFRobot_MultiGasSensor and Adafruit_I2CDevice) call
//  Wire.begin() from their own init(), and twi_init() resets that limit to the
//  150 ms core default every time - so setting it once next to the first
//  Wire.begin() achieves nothing by the time sampling starts. It has to be
//  re-applied after the LAST sensor has initialised, and after every recovery.
// ---------------------------------------------------------------------------
void dl_i2c_apply_limits() {
#if defined(ARDUINO_ARCH_ESP8266)
  Wire.setClockStretchLimit(I2C_CLOCK_STRETCH_US);
#endif
}

// Clock a slave off the bus if it is mid-transaction, then issue a STOP.
// Wire.begin() cannot do this on its own: it only re-runs pinMode(), so a slave
// left holding SDA by a reset stays stuck across every subsequent reboot - the
// ESP resets, its sensors do not - and only removing power clears it.
// Returns false if the bus is still unusable afterwards.
bool dl_i2c_bus_recover() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);

  // Nothing a master can do about this one: we drive SCL, so if it reads low
  // something else is holding it down and only a power cycle will help.
  if (digitalRead(SCL) == LOW) return false;
  if (digitalRead(SDA) == HIGH) return true;      // already idle

  // Nine clocks is enough for a slave to finish any byte it thinks it is sending.
  for (int i = 0; i < 9 && digitalRead(SDA) == LOW; ++i) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);                   // released, so stretching still works
    delayMicroseconds(5);
  }
  bool freed = digitalRead(SDA) == HIGH;

  // STOP condition: SDA rises while SCL is high.
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
  return freed;
}

// Full bus reset: unstick, re-init the peripheral, restore the stretch limit.
bool dl_i2c_bus_reset() {
  bool ok = dl_i2c_bus_recover();
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
  Wire.end();
#endif
  Wire.begin();
  dl_i2c_apply_limits();
  return ok;
}

void recoverI2C(size_t pos) {
  mp.disableAllBuses();
  dl_i2c_bus_reset();
  // A glitched multiplexer has to be brought back too, otherwise every channel
  // switch below silently addresses nothing.
  mp.init();
  uint8_t ch = i2c_buses[pos];
  // Re-init every sensor wired to this channel. MLX and ADS are intentionally
  // omitted, matching the original behaviour (MLX has no meaningful re-init;
  // ADS shares a single global driver object).
#if USE_BME280
  for (size_t i = 0; i < BME_COUNT; ++i) if (bme_buses[i] == ch) bme_sensors[i]->init();
#endif
#if USE_SCD30
  for (size_t i = 0; i < SCD_COUNT; ++i) if (scd_buses[i] == ch && scd_sensors[i]->init()) {
    scd_sensors[i]->enable_self_calibration(false);
    scd_sensors[i]->set_interval(2);
  }
#endif
#if USE_SEN0465
  for (size_t i = 0; i < SEN_COUNT; ++i) if (sen_buses[i] == ch) sen_sensors[i]->init();
#endif
  mp.disableAllBuses();
  // The driver init()s above call Wire.begin() internally, which puts the stretch
  // limit back to the 150 ms default. Restore it or the next stuck device stalls
  // the loop for seconds.
  dl_i2c_apply_limits();
  i2c_recovery_count[pos]++;
  i2c_last_recovery_ms[pos] = millis();
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
    "boot_time_source,rtc_time_valid,supervisor_restart_reason,"
    "gps_date_valid,gps_time_fresh,gps_time_sane,gps_location_valid,gps_location_fresh,gps_stale_count,gps_recovery_count,"
    "gps_poll_timeout_count,gps_i2c_recovery_count,gps_quarantine_active,gps_quarantine_count,gps_rejected_time_count,"
    "valid_sensor_value_count,missing_sensor_value_count,critical_sensor_loss_count,gps_i2c_hard_fault_count,supervisor_reset_count,"
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
  csv_field(row, boot_time_source_buf);
  csv_int(row, rtc_time_valid ? 1 : 0);
  csv_field(row, supervisor_restart_reason_buf);
#if USE_GPS
  csv_int(row, isGPSDateValid() ? 1 : 0);
  csv_int(row, isGPSTimeFresh() ? 1 : 0);
  csv_int(row, gps_time_sane ? 1 : 0);
  csv_int(row, gps.locationValid() ? 1 : 0);
  csv_int(row, isGPSLocationFresh() ? 1 : 0);
  csv_uint(row, gps_stale_count);
  csv_uint(row, gps_recovery_count);
  csv_uint(row, gps_poll_timeout_count);
  csv_uint(row, gps_i2c_recovery_count);
  csv_int(row, gps_quarantine_active ? 1 : 0);
  csv_uint(row, gps_quarantine_count);
  csv_uint(row, gps_rejected_time_count);
#else
  csv_int(row, 0);
  csv_int(row, 0);
  csv_int(row, 0);
  csv_int(row, 0);
  csv_int(row, 0);
  csv_uint(row, 0);
  csv_uint(row, 0);
  csv_uint(row, 0);
  csv_uint(row, 0);
  csv_int(row, 0);
  csv_uint(row, 0);
  csv_uint(row, 0);
#endif
  csv_uint(row, valid_sensor_value_count);
  csv_uint(row, missing_sensor_value_count);
  csv_uint(row, critical_sensor_loss_count);
  csv_uint(row, gps_i2c_hard_fault_count);
  csv_uint(row, supervisor_reset_count);
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
        return (year >= GPS_YEAR_MIN) && (year <= GPS_YEAR_MAX);
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

bool isGPSTimeSane(uint32_t now_ms) {
    #if USE_GPS
        if (!isGPSTimeFresh()) return false;
        return is_gps_epoch_sane(gps_epoch_now(), now_ms);
    #else
        (void)now_ms;
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
        // Do not reset Wire or the multiplexer here: GPS is on the direct bus
        // and a shared-bus reset can make otherwise healthy sensors disappear.
        gps.init();
        gps_recovery_count++;
        gps_i2c_recovery_count++;
        last_gps_recovery_ms = millis();
        gps_i2c_recovery_status_pending = true;
    #endif
}

void count_sensor_value(float value, uint32_t& valid_count, uint32_t& missing_count) {
  if (isnan(value)) missing_count++;
  else valid_count++;
}

bool all_i2c_buses_degraded() {
  if (num_i2c_buses == 0) return false;   // a sensorless build must not trip the supervisor
  for (size_t i = 0; i < num_i2c_buses; i++) {
    if (i2c_consecutive_error_count[i] == 0) return false;
  }
  return true;
}

void start_gps_quarantine(uint32_t now_ms) {
  gps_quarantine_active = true;
  gps_quarantine_until_ms = now_ms + GPS_QUARANTINE_MS;
  gps_quarantine_count++;
  gps_i2c_quarantine_status_pending = true;
#if I2C_MULTI
  mp.disableAllBuses();
#endif
}

void update_gps_quarantine(uint32_t now_ms) {
  if (gps_quarantine_active && (int32_t)(now_ms - gps_quarantine_until_ms) >= 0) {
    gps_quarantine_active = false;
  }
}

bool activate_rtc_boot_time(uint32_t now_ms) {
  if (!rtc_time_valid || gps_boot_locked) return false;
  last_fresh_gps_epoch = rtc_boot_epoch;
  last_fresh_gps_millis = boot_ms;
  last_fresh_gps_epoch_valid = true;
  gps_boot_locked = true;
  boot_time_from_rtc = true;
  strncpy(boot_time_source_buf, "rtc", sizeof(boot_time_source_buf) - 1);
  boot_time_source_buf[sizeof(boot_time_source_buf) - 1] = '\0';
  format_utc_date(rtc_boot_epoch + ((now_ms - boot_ms) / 1000UL), last_valid_gps_date, sizeof(last_valid_gps_date));
  boot_time_from_rtc_status_pending = true;
  return true;
}

void update_sensor_supervisor(uint32_t now_ms) {
  bool all_sensor_loss = all_i2c_buses_degraded() && valid_sensor_value_count == 0;
  bool gps_i2c_hard_fault = gps_quarantine_active && all_sensor_loss;
  if (gps_i2c_hard_fault) {
    if (gps_i2c_hard_fault_count == 0) sensor_data_lost_status_pending = true;
    gps_i2c_hard_fault_count++;
  } else {
    gps_i2c_hard_fault_count = 0;
  }

  if (gps_i2c_hard_fault_count >= GPS_I2C_HARD_FAULT_SAMPLES && gps_boot_locked) {
    supervisor_reset_count++;
    last_supervisor_reset_ms = now_ms;
    strncpy(supervisor_restart_reason_buf, "gps_i2c_hard_fault", sizeof(supervisor_restart_reason_buf) - 1);
    supervisor_restart_reason_buf[sizeof(supervisor_restart_reason_buf) - 1] = '\0';
    supervisor_restart_pending = true;
    supervisor_restart_status_pending = true;
    return;
  }

  bool critical_loss = all_sensor_loss;
  if (critical_loss) {
    if (critical_sensor_loss_count == 0) sensor_data_lost_status_pending = true;
    critical_sensor_loss_count++;
  } else {
    critical_sensor_loss_count = 0;
    return;
  }

  if (critical_sensor_loss_count < CRITICAL_SENSOR_LOSS_SAMPLES) return;
  if (!gps_boot_locked) return;
  if ((now_ms - boot_ms) < SUPERVISOR_MIN_UPTIME_MS) return;
  if (last_supervisor_reset_ms != 0 && (now_ms - last_supervisor_reset_ms) < SUPERVISOR_RESTART_COOLDOWN_MS) return;

  supervisor_reset_count++;
  last_supervisor_reset_ms = now_ms;
  strncpy(supervisor_restart_reason_buf, "sensor_loss", sizeof(supervisor_restart_reason_buf) - 1);
  supervisor_restart_reason_buf[sizeof(supervisor_restart_reason_buf) - 1] = '\0';
  supervisor_restart_pending = true;
  supervisor_restart_status_pending = true;
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
