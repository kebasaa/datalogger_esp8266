#pragma once
// ============================================================================
//  Datalogger framework internals: processData / processDataBuffered sampling routine
//  Auto-relocated verbatim from the old monolithic main.cpp. Included ONLY by
//  datalogger.h (one translation unit). Beginners never need to open this.
// ============================================================================

// Temporary: find where a sample cycle's heap goes.
//
// Fixing the Set clock button made the logger actually log, and that exposed a
// cost no earlier capture could show: with gps_boot_locked holding logging off,
// a sample cycle had never once run while a web client was attached. Measured
// the first time it did - free heap 4840 before the cycle, 2624 by the time the
// row was written, then an OOM panic on a 384-byte allocation.
//
// So a sample cycle needs ~2.2KB, and with a client attached that is roughly all
// there is. This marks the phases so the next capture says WHICH part, rather
// than another guess.
#define DL_SAMPLE_HEAP_MARK(tag) \
  Serial.printf("[smp] %-8s heap=%u block=%u\n", (tag), \
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxFreeBlockSize())

void processDataBuffered(void){
  DL_SAMPLE_HEAP_MARK("enter");
  CsvBuffer row;
  csv_init(row, data_row_buf, sizeof(data_row_buf));
  Cal::CalibrationResult cal_result;
  uint32_t now_ms = millis();
  uint32_t gps_chars_this_sample = 0;
  bool gps_date_valid = false;
  bool gps_time_fresh = false;
  bool gps_location_valid = false;
  bool gps_location_fresh = false;

#if USE_GPS
  update_gps_quarantine(now_ms);
  gps_chars_this_sample = gps_chars_since_sample;
  gps_chars_since_sample = 0;
  gps_date_valid = isGPSDateValid();
  gps_time_fresh = isGPSTimeFresh();
  gps_time_sane = isGPSTimeSane(now_ms);
  gps_location_valid = gps.locationValid();
  gps_location_fresh = isGPSLocationFresh();
  if (gps_time_fresh && !gps_time_sane) {
    gps_rejected_time_count++;
    gps_time_rejected_status_pending = true;
  }
  if (gps_time_sane) {
    String date = gps.get_date();
    strncpy(last_valid_gps_date, date.c_str(), sizeof(last_valid_gps_date) - 1);
    last_valid_gps_date[sizeof(last_valid_gps_date) - 1] = '\0';
    gps_boot_locked = true;
  }
  if (!gps_boot_locked) {
    if ((now_ms - boot_ms) >= BOOT_GPS_GRACE_MS && activate_rtc_boot_time(now_ms)) {
      gps_time_sane = false;
    } else {
      Serial.println(F("Waiting for first fresh/sane GPS date/time before logging"));
      return;
    }
  }
  if (!gps_quarantine_active && !gps_time_sane && gps_chars_this_sample == 0) {
    gps_stale_count++;
    if (!gps_stale_active && gps_stale_count >= GPS_STALE_CONSECUTIVE_SAMPLES) {
      gps_stale_active = true;
      gps_stale_status_pending = true;
      gps_i2c_stale_status_pending = true;
    }
    if (gps_stale_count >= GPS_STALE_CONSECUTIVE_SAMPLES &&
        (last_gps_recovery_ms == 0 || now_ms - last_gps_recovery_ms >= GPS_RECOVERY_COOLDOWN_MS)) {
      recoverGPS();
      gps_stale_recovery_count++;
      if (gps_stale_recovery_count >= GPS_QUARANTINE_AFTER_RECOVERIES) {
        start_gps_quarantine(now_ms);
      }
    }
  } else if (gps_time_sane) {
    if (gps_stale_active) gps_recovered_status_pending = true;
    if (gps_stale_active || gps_quarantine_active || gps_stale_count > 0) gps_i2c_recovered_status_pending = true;
    gps_quarantine_active = false;
    gps_quarantine_until_ms = 0;
    gps_stale_active = false;
    gps_stale_count = 0;
    gps_stale_recovery_count = 0;
  }
#else
  gps_boot_locked = true;
#endif

  if (!data_header_ready) build_data_header();
  sample_counter++;

  uint16_t row_status = last_data_write_status;
  now_ms = millis();
  uint32_t uptime_ms = now_ms - boot_ms;
  const char* timestamp_calc_source = "";
  uint32_t timestamp_calc_epoch = calculated_timestamp_epoch(gps_time_fresh, gps_time_sane, now_ms, &timestamp_calc_source);
  bool timestamp_calc_valid = timestamp_calc_epoch > 0;
  if (timestamp_calc_valid) {
    format_utc_timestamp(timestamp_calc_epoch, timestamp_calc_buf, sizeof(timestamp_calc_buf));
    format_utc_date(timestamp_calc_epoch, timestamp_file_date_buf, sizeof(timestamp_file_date_buf));
  } else {
    timestamp_calc_buf[0] = '\0';
    timestamp_file_date_buf[0] = '\0';
  }

  // GPS has just replaced a hand-set clock. Say by how much, once. A small
  // correction means the rows logged on the manual clock are fine; a large one
  // means their timestamps are wrong by roughly that much, and only this message
  // makes that visible after the fact.
  if (manual_time_jump_pending) {
    manual_time_jump_pending = false;
    dl_notify("GPS time acquired, clock corrected by %ld s", manual_time_jump_s);
  }

  csv_uint(row, boot_id);
  csv_uint(row, sample_counter);
  csv_uint(row, uptime_ms);
  csv_uint(row, now_ms);
  csv_field(row, reset_reason_buf);
  csv_field(row, boot_time_source_buf);
  csv_int(row, rtc_time_valid ? 1 : 0);
  csv_field(row, supervisor_restart_reason_buf);

#if USE_GPS
  csv_int(row, gps_date_valid ? 1 : 0);
  csv_int(row, gps_time_fresh ? 1 : 0);
  csv_int(row, gps_time_sane ? 1 : 0);
  csv_int(row, gps_location_valid ? 1 : 0);
  csv_int(row, gps_location_fresh ? 1 : 0);
  csv_uint(row, gps_chars_this_sample);
  csv_uint(row, gps.gpsAgeMs());
  csv_uint(row, gps.locationAgeMs());
  csv_uint(row, gps_stale_count);
  csv_uint(row, gps_recovery_count);
  csv_uint(row, gps_poll_timeout_count);
  csv_uint(row, gps_i2c_recovery_count);
  csv_int(row, gps_quarantine_active ? 1 : 0);
  csv_uint(row, gps_quarantine_count);
  csv_uint(row, gps_rejected_time_count);
  if (gps_time_sane) {
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

  valid_sensor_value_count = 0;
  missing_sensor_value_count = 0;

  // Per-channel I2C health accumulators, indexed by position in i2c_buses[].
  uint32_t bus_elapsed_ms[MAX_I2C_BUSES] = {0};
  bool     bus_error[MAX_I2C_BUSES]      = {false};
  // BME pressure per channel, so a co-located SCD30 can be pressure-compensated
  // (the old code read bme_p in the same loop iteration; type-major splits them).
  float    bme_p_by_channel[MAX_I2C_BUSES];
  for (size_t i = 0; i < MAX_I2C_BUSES; ++i) bme_p_by_channel[i] = NAN;

  // Type-major: each sensor type iterates its own instances. This column order
  // (ADS, BME, SCD, SEN, MLX) MUST match build_data_header() exactly.
#if USE_ADS1115
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    int pos = i2c_bus_pos(ads_buses[i]);
    uint32_t t0 = millis();
    float ads_in = ads_sensors[i]->read_val(1, 16, 25.83);
    float ads_out = ads_sensors[i]->read_val(2, 16, 30.98);
    bool err = isnan(ads_in) || isnan(ads_out);
    count_sensor_value(ads_in, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(ads_out, valid_sensor_value_count, missing_sensor_value_count);
    csv_float(row, ads_in, 2);
    csv_float(row, ads_out, 2);
    if (pos >= 0) { bus_elapsed_ms[pos] += millis() - t0; bus_error[pos] |= err; }
  }
#endif

#if USE_BME280
  for (size_t i = 0; i < BME_COUNT; ++i) {
    int pos = i2c_bus_pos(bme_buses[i]);
    uint32_t t0 = millis();
    float bme_temp = bme_sensors[i]->airT();
    float bme_rh   = bme_sensors[i]->airRH();
    float bme_p    = bme_sensors[i]->airP();
    float bme_h2o  = env.air_water_mole_frac(bme_temp, bme_rh, bme_p);
    bool err = isnan(bme_temp) || isnan(bme_rh) || isnan(bme_p) || isnan(bme_h2o);
    bme_p_by_channel[bme_buses[i]] = bme_p;
    count_sensor_value(bme_temp, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(bme_rh, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(bme_p, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(bme_h2o, valid_sensor_value_count, missing_sensor_value_count);
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
    if (pos >= 0) { bus_elapsed_ms[pos] += millis() - t0; bus_error[pos] |= err; }
  }
#endif

#if USE_SCD30
  for (size_t i = 0; i < SCD_COUNT; ++i) {
    int pos = i2c_bus_pos(scd_buses[i]);
    uint32_t t0 = millis();
  #if USE_BME280
    float p = bme_p_by_channel[scd_buses[i]];
    if (sample_counter % 3 == 0 && !isnan(p)) scd_sensors[i]->set_air_pressure(p);
  #endif
    scd_sensors[i]->getData();
    float scd_T   = scd_sensors[i]->airT();
    float scd_RH  = scd_sensors[i]->airRH();
    float scd_CO2 = scd_sensors[i]->airCO2();
    bool err = isnan(scd_T) || isnan(scd_RH) || isnan(scd_CO2);
    count_sensor_value(scd_T, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(scd_RH, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(scd_CO2, valid_sensor_value_count, missing_sensor_value_count);
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
    if (pos >= 0) { bus_elapsed_ms[pos] += millis() - t0; bus_error[pos] |= err; }
  }
#endif

#if USE_SEN0465
  for (size_t i = 0; i < SEN_COUNT; ++i) {
    int pos = i2c_bus_pos(sen_buses[i]);
    uint32_t t0 = millis();
    float sen_T = sen_sensors[i]->airT();
    float sen_O2 = sen_sensors[i]->airO2();
    float sen_raw_v = sen_sensors[i]->rawV();
    bool err = isnan(sen_T) || isnan(sen_O2) || isnan(sen_raw_v);
    count_sensor_value(sen_T, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(sen_O2, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(sen_raw_v, valid_sensor_value_count, missing_sensor_value_count);
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
    if (pos >= 0) { bus_elapsed_ms[pos] += millis() - t0; bus_error[pos] |= err; }
  }
#endif

#if USE_MLX90614
  for (size_t i = 0; i < MLX_COUNT; ++i) {
    int pos = i2c_bus_pos(mlx_buses[i]);
    uint32_t t0 = millis();
    float mlx_t = mlx_sensors[i]->airT();
    float mlx_obj_t = mlx_sensors[i]->objT();
    bool err = isnan(mlx_t) || isnan(mlx_obj_t);
    count_sensor_value(mlx_t, valid_sensor_value_count, missing_sensor_value_count);
    count_sensor_value(mlx_obj_t, valid_sensor_value_count, missing_sensor_value_count);
    csv_float(row, mlx_t, 2);
    csv_float(row, mlx_obj_t, 2);
    if (pos >= 0) { bus_elapsed_ms[pos] += millis() - t0; bus_error[pos] |= err; }
  }
#endif

  for (size_t p = 0; p < num_i2c_buses; ++p)
    record_i2c_health(p, bus_elapsed_ms[p], bus_error[p]);

  update_sensor_supervisor(now_ms);

  csv_uint(row, heap_free_now());
  csv_uint(row, heap_max_block_now());
  csv_uint(row, heap_min_block_now());
  csv_uint(row, i2c_slow_total());
  csv_uint(row, i2c_error_total());
  csv_uint(row, i2c_recovery_total());
  for (size_t i = 0; i < num_i2c_buses; i++) {
    csv_uint(row, i2c_error_count[i]);
    csv_uint(row, i2c_consecutive_error_count[i]);
    csv_uint(row, i2c_recovery_count[i]);
  }
  csv_uint(row, valid_sensor_value_count);
  csv_uint(row, missing_sensor_value_count);
  csv_uint(row, critical_sensor_loss_count);
  csv_uint(row, gps_i2c_hard_fault_count);
  csv_uint(row, supervisor_reset_count);
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
  DL_SAMPLE_HEAP_MARK("presd");
  uint16_t write_status = sd.write_data(data_fn,
                                        data_header,
                                        data_row_buf,
                                        MEASUREMENT_INTERVAL);
  DL_SAMPLE_HEAP_MARK("wrote");
  last_data_write_status = write_status;
  // Refresh the card figures the status page reports. getFreeMB() walks every
  // file in the root directory over SPI, so it is done here, once per sample,
  // rather than from an HTTP handler where it would block the request.
  if (write_status == MicroSD::WRITE_OK) {
    dl_sd_cap_mb  = sd.getCapacityMB();
    dl_sd_free_mb = sd.getFreeMB();
    DL_SAMPLE_HEAP_MARK("sdstat");
  }
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
  if (write_status == MicroSD::WRITE_OK && gps_i2c_stale_status_pending) {
    if (write_status_row("gps_i2c_stale", write_status) == MicroSD::WRITE_OK) gps_i2c_stale_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && gps_i2c_recovery_status_pending) {
    if (write_status_row("gps_i2c_recovery", write_status) == MicroSD::WRITE_OK) gps_i2c_recovery_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && gps_i2c_quarantine_status_pending) {
    if (write_status_row("gps_i2c_quarantine", write_status) == MicroSD::WRITE_OK) gps_i2c_quarantine_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && gps_time_rejected_status_pending) {
    if (write_status_row("gps_time_rejected", write_status) == MicroSD::WRITE_OK) gps_time_rejected_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && boot_time_from_rtc_status_pending) {
    if (write_status_row("boot_time_from_rtc", write_status) == MicroSD::WRITE_OK) boot_time_from_rtc_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && gps_recovered_status_pending) {
    if (write_status_row("gps_recovered", write_status) == MicroSD::WRITE_OK) gps_recovered_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && gps_i2c_recovered_status_pending) {
    if (write_status_row("gps_i2c_recovered", write_status) == MicroSD::WRITE_OK) gps_i2c_recovered_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && i2c_recovery_status_pending) {
    if (write_status_row("i2c_recovery", write_status) == MicroSD::WRITE_OK) i2c_recovery_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && sensor_data_lost_status_pending) {
    if (write_status_row("sensor_data_lost", write_status) == MicroSD::WRITE_OK) sensor_data_lost_status_pending = false;
  }
  if (write_status == MicroSD::WRITE_OK && supervisor_restart_status_pending) {
    if (write_status_row("supervisor_restart_pending", write_status) == MicroSD::WRITE_OK) supervisor_restart_status_pending = false;
  }
#endif

  Serial.println(data_row_buf);

  if (supervisor_restart_pending) {
    Serial.println(F("Supervisor restart after persistent GPS/I2C sensor loss"));
    write_rtc_restart_record(timestamp_calc_epoch, RTC_REASON_GPS_I2C_HARD_FAULT);
    delay(100);
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    ESP.restart();
#endif
  }
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
