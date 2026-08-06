#pragma once
// ============================================================================
//  Datalogger framework internals: calibration command handlers
//  Included ONLY by datalogger.h (one translation unit). Beginners never need
//  to open this.
//
//  Every calibration action is a command (see the h4p.addCmd calls in
//  dl_setup.h), reachable identically from the serial port and from the web UI
//  over /rest. There is no widget layer: the browser talks to exactly the same
//  command surface a serial cable does.
// ============================================================================

void absolute_calibration(const std::vector<String> gas_list, String abs_cal_type, int sensor, float ref_value);
void differential_calibration(const std::vector<String> gas_list, String dif_cal_type);

// Push a one-line message to any connected browser as an SSE "toast" event.
// Defined in dl_api.h, which is included after this file; both are part of the
// same translation unit, so a forward declaration is all that is needed.
void dl_toast(const std::string& msg);

// ---------------------------------------------------------------------------
//  Telling the user something
//
//  One helper for all user-facing messages. Where the old dashboard needed a
//  Serial.println() plus a guarded h4wifi.uiMessage(), this routes a single
//  message to every channel that can currently receive it:
//
//   - always to the serial port,
//   - to the /rest reply lines when the command came in over the web, so the
//     caller gets it back in the JSON "lines" array,
//   - to any open SSE stream as a toast.
//
//  Gating the reply on h4pSrc matters: these handlers are reachable from the
//  serial port too, and H4P_WiFi::_lines is only ever drained by a /rest or
//  websocket response. Pushing serial output into it would accumulate forever.
//
//  Keep the text to plain ASCII without double quotes or backslashes:
//  H4P_WiFi::_execute() builds the JSON by string concatenation and does not
//  escape reply lines, so either character produces a response the browser
//  cannot parse.
// ---------------------------------------------------------------------------
void dl_notify(const char* fmt, ...){
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  Serial.println(buf);
  if(h4pSrc == wifiTag()){ h4wifi._reply(std::string(buf)); }
  dl_toast(std::string(buf));
}

// ---------------------------------------------------------------------------
//  Calibration sessions
//
//  Calibrating writes to flash, and the flash can only be rewritten so many
//  times, so a calibration is done inside a "session": cal_start, then as many
//  zero/span/differential calibrations as you like, then cal_stop to save them
//  all in one go. cal_discard throws the session away instead.
// ---------------------------------------------------------------------------

// Seconds since midnight, used to check how far apart two calibration steps
// were. Without a GPS there is no wall clock, so count from power-on instead:
// the checks only ever look at the difference between two of these.
unsigned long calibration_time_s(void){
#if USE_GPS
  return(gps.seconds_since_midnight());
#else
  return((millis() / 1000UL) % 86400UL);
#endif
}

// Session state, shown on the Calibration tab. Pushed once a second as part of
// the /api/status payload, so nothing has to refresh it explicitly.
String cal_status_text(void){
  if(!cal.session_open()){ return(String("idle - run cal_start to begin")); }
  size_t pending = cal.pending_count();
  if(!pending){ return(String("recording - no values changed yet")); }
  return(String("recording - ") + String((int)pending) + " value(s) not saved yet");
}

// Explains the numbered results of set_calibration_coeff() / set_differential_coeff()
const char* cal_error_text(int code){
  switch(code){
    case 1: return "Too long since the zero calibration - redo it";
    case 2: return "Less than 10s since the zero calibration - is the span gas stable?";
    case 3: return "Less than 10s since the low calibration - is the concentration stable?";
    case 4: return ">10min since the low calibration - applied a simple offset only";
    case 5: return "No low calibration yet - do the low calibration first";
    default: return "Calibration rejected";
  }
}

// Report a non-zero calibration result to whoever is watching. Doing this matters
// during a session: a rejected step has to be visible before the values are saved.
void cal_report_result(int code){
  if(!code){ return; }
  dl_notify("%s", cal_error_text(code));
}

// Read a number out of a serial command parameter. Typing something that is not
// a number would otherwise abort the firmware: this build has C++ exceptions
// turned off, so std::stof/std::stoi have no way to report a bad value.
bool parse_cmd_float(const std::string& param, float& out){
  if(param.empty()){ return(false); }
  const char* first = param.c_str();
  char* last = nullptr;
  float val = strtof(first, &last);
  if((last == first) || (*last != '\0') || isnan(val)){ return(false); }
  out = val;
  return(true);
}

bool parse_cmd_int(const std::string& param, int& out){
  if(param.empty()){ return(false); }
  const char* first = param.c_str();
  char* last = nullptr;
  long val = strtol(first, &last, 10);
  if((last == first) || (*last != '\0')){ return(false); }
  out = (int)val;
  return(true);
}

// ---------------------------------------------------------------------------
//  settime - start the clock by hand
//
//  The logger records nothing at all until gps_boot_locked is true, so a unit
//  indoors or without a sky view sits idle indefinitely. This is the same trick
//  activate_rtc_boot_time() plays after a supervisor restart, from a different
//  source: anchor the clock, unblock logging, and let GPS overwrite it later.
//
//  Six segments rather than an epoch. parse_cmd_int() yields an int, and an
//  epoch sits uncomfortably close to its range; utc_to_epoch() already exists
//  and validates as it converts; and it matches the "/"-separated shape of every
//  other command here. The browser does the UTC conversion explicitly with
//  getUTC*(), so what arrives is UTC by construction.
//
//    /rest/settime/2026/7/31/13/45/9
// ---------------------------------------------------------------------------
uint32_t settime_cmd(std::vector<std::string> vs) {
  if(vs.size() < 6){
    dl_notify("settime needs year/month/day/hour/minute/second in UTC");
    return H4_CMD_TOO_FEW_PARAMS;
  }
  if(vs.size() > 6){
    dl_notify("settime takes exactly year/month/day/hour/minute/second");
    return H4_CMD_TOO_MANY_PARAMS;
  }

  int part[6];
  for(size_t i = 0; i < 6; i++){
    if(!parse_cmd_int(vs[i], part[i])){
      dl_notify("Every part of the time must be a number");
      return H4_CMD_NOT_NUMERIC;
    }
  }

  // The same window is enforced by is_gps_epoch_sane() and, more importantly, by
  // read_rtc_restart_record() - a year outside it would be accepted here and then
  // silently refused on the way back in after a restart.
  if(part[0] < (int)GPS_YEAR_MIN || part[0] > (int)GPS_YEAR_MAX){
    dl_notify("Year must be between %u and %u", (unsigned)GPS_YEAR_MIN, (unsigned)GPS_YEAR_MAX);
    return H4_CMD_OUT_OF_BOUNDS;
  }
  if(part[1] < 1 || part[1] > 12 || part[2] < 1 || part[2] > 31 ||
     part[3] > 23 || part[4] > 59 || part[5] > 59 ||
     part[3] < 0 || part[4] < 0 || part[5] < 0){
    dl_notify("That is not a valid date and time");
    return H4_CMD_OUT_OF_BOUNDS;
  }

  uint32_t epoch = utc_to_epoch((uint16_t)part[0], (uint8_t)part[1], (uint8_t)part[2],
                                (uint8_t)part[3], (uint8_t)part[4], (uint8_t)part[5]);

  // Anchor the clock. millis() now, NOT boot_ms: the offset is measured from this
  // instant, and using boot_ms would back-date the clock by the whole uptime.
  last_fresh_gps_epoch = epoch;
  last_fresh_gps_millis = millis();
  last_fresh_gps_epoch_valid = true;
  manual_time_active = true;

  // The actual gate on logging.
  gps_boot_locked = true;

  strncpy(boot_time_source_buf, "manual", sizeof(boot_time_source_buf) - 1);
  boot_time_source_buf[sizeof(boot_time_source_buf) - 1] = '\0';

  // write_status_row() refuses to write without this, so the status rows would go
  // missing even though data rows were being logged.
  format_utc_date(epoch, last_valid_gps_date, sizeof(last_valid_gps_date));

  // Survive a watchdog or supervisor restart. Not a power cycle - RTC user memory
  // does not, and pretending otherwise would be worse than saying so.
  write_rtc_restart_record(epoch, RTC_REASON_MANUAL_TIME);

  // Deliberately NOT setting gps_time_sane: it is recomputed every sample, and if
  // it were true this hand-set time would be written into the timestamp_utc
  // column, which is reserved for times that came straight off the GPS.
  char when[20];
  format_utc_timestamp(epoch, when, sizeof(when));
  dl_notify("Clock set to %s UTC. Logging started", when);
  return H4_CMD_OK;
}

void save_calibration_coefficients(void);
void cal_session_commit(const char* reason);

// Restart the "user walked away" countdown. Called after every calibration action.
void cal_session_touch(void){
  if(cal_session_timeout_timer){ h4.cancel({cal_session_timeout_timer}); cal_session_timeout_timer = nullptr; }
  if(!cal.session_open() || !CAL_SESSION_TIMEOUT_MIN){ return; }
  cal_session_timeout_timer = h4.once(CAL_SESSION_TIMEOUT_MIN * 60UL * 1000UL, [](){
    cal_session_timeout_timer = nullptr;
    cal_session_commit("Calibration session was left open - saved automatically");
  });
}

void cal_session_open(void){
  cal.open_session();
  cal_session_touch();
  dl_notify("Calibration started. Nothing is saved until the session is closed");
}

// Save everything collected during the session: reconstruct whatever can be
// derived first (still inside the session, so it costs no extra flash write),
// then write once, then append one row to the calibration log on the SD card.
void cal_session_commit(const char* reason){
  if(!cal.session_open()){ return; }
  if(cal_session_timeout_timer){ h4.cancel({cal_session_timeout_timer}); cal_session_timeout_timer = nullptr; }

  cal.fix_all_calibrations(quantities, numSensors);
  // Counted after the reconstruction, so the log on the SD card gets a row
  // whenever something was actually saved
  bool saved_something = (cal.pending_count() > 0);
  cal.commit_session();
  if(saved_something){ save_calibration_coefficients(); }

  dl_notify("%s", reason);
}

void cal_session_discard(void){
  if(!cal.session_open()){
    dl_notify("No calibration in progress");
    return;
  }
  if(cal_session_timeout_timer){ h4.cancel({cal_session_timeout_timer}); cal_session_timeout_timer = nullptr; }
  cal.discard_session();
  dl_notify("Calibration discarded, nothing was saved");
}

// Calibrating is only allowed inside a session, so that it is always clear when
// values get written to flash.
bool cal_require_session(void){
  if(cal.session_open()){ return(true); }
  dl_notify("No calibration in progress. Run cal_start first");
  return(false);
}

// Start, save or throw away a calibration session. The Calibration tab's
// buttons issue exactly these, so a session is shared between the browser and
// the serial port. The syntax is:
// cal_start
// cal_stop
// cal_discard
uint32_t cal_start_cmd(std::vector<std::string> vs) {
  if(cal.session_open()){
    dl_notify("A calibration is already in progress");
    return H4_CMD_NOT_NOW;
  }
  cal_session_open();
  return H4_CMD_OK;
}

uint32_t cal_stop_cmd(std::vector<std::string> vs) {
  if(!cal.session_open()){
    dl_notify("No calibration in progress. Run cal_start first");
    return H4_CMD_NOT_NOW;
  }
  cal_session_commit("Calibration saved");
  return H4_CMD_OK;
}

uint32_t cal_discard_cmd(std::vector<std::string> vs) {
  if(!cal.session_open()){
    dl_notify("No calibration in progress");
    return H4_CMD_NOT_NOW;
  }
  cal_session_discard();
  return H4_CMD_OK;
}

// Show calibration coefficients. The syntax is:
// show_cal
uint32_t cal_show_cmd(std::vector<std::string> vs) {
  dl_notify("Calibration coefficients:");
  cal.show_all_calibrations(quantities, numSensors);
  return H4_CMD_OK;
}

// Clear every stored calibration coefficient. Was the dashboard's "reset"
// button; now a command so it works from the serial port too. The syntax is:
// cal_reset
uint32_t cal_reset_cmd(std::vector<std::string> vs) {
  // Clearing everything is a calibration change like any other, so it goes
  // through a session too and costs a single flash write.
  bool own_session = !cal.session_open();
  if(own_session){ cal.open_session(); }
  cal.reset_all_calibrations(quantities, numSensors);
  if(own_session){ cal.commit_session(); } else { cal_session_touch(); }
  dl_notify("All calibration coefficients reset");
  return H4_CMD_OK;
}

// Choose which quantity/sensor the live readout on the Calibration tab tracks.
// Purely a display selection - it changes nothing that gets logged, and no
// calibration command depends on it (they all take explicit parameters).
// The syntax is:
// cal_select/co2/1
// cal_select/co2/1/h2o     (third parameter sets the differential quantity)
uint32_t cal_select_cmd(std::vector<std::string> vs) {
  if(vs.size() < 2){
    dl_notify("cal_select needs a quantity and a sensor number");
    return H4_CMD_TOO_FEW_PARAMS;
  }
  if(vs.size() > 3){
    dl_notify("cal_select takes at most quantity, sensor number and differential quantity");
    return H4_CMD_TOO_MANY_PARAMS;
  }
  String gas = vs[0].c_str();
  if((gas != "all") && !utils.in_list(quantities, gas)){
    dl_notify("Unknown quantity: %s", vs[0].c_str());
    return H4_CMD_PAYLOAD_FORMAT;
  }
  int sensor = 0;
  if(!parse_cmd_int(vs[1], sensor)){
    dl_notify("The sensor number must be a number");
    return H4_CMD_NOT_NUMERIC;
  }
  currentGas = gas;
  currentSensor = sensor;
  if(vs.size() == 3){
    String dgas = vs[2].c_str();
    if((dgas != "all") && !utils.in_list(quantities, dgas)){
      dl_notify("Unknown differential quantity: %s", vs[2].c_str());
      return H4_CMD_PAYLOAD_FORMAT;
    }
    currentDiffGas = dgas;
  }
  dl_notify("Watching %s sensor %d", vs[0].c_str(), sensor);
  return H4_CMD_OK;
}

// Manually set calibration coefficients through the Serial Port. Needs a session
// (cal_start). Documented in README.md. The syntax is:
// set/zero/co2/1/12.07/13.08
// set/span/co2/1/376.87/402.87
// Where the first number is the reference, and the second the measured value by the sensor for that reference
uint32_t cal_set_cmd(std::vector<std::string> vs) {
  if (vs.size() < 5) {
    dl_notify("Min. 5 parameters: type (zero/span), gas, sensor number, ref. value and measured value");
    return H4_CMD_TOO_FEW_PARAMS;
  } else if(vs.size() > 5){
    dl_notify("Max. 5 parameters: type (zero/span), gas, sensor number, ref. value and measured value");
    return H4_CMD_TOO_MANY_PARAMS;
  }
  // Collect the values
  String abs_cal_type = vs[0].c_str();
  String cal_gas = vs[1].c_str();
  int sensor_nb = 0;
  float ref_value = 0.0;
  float sen_value = 0.0;
  if(!parse_cmd_int(vs[2], sensor_nb) || !parse_cmd_float(vs[3], ref_value) || !parse_cmd_float(vs[4], sen_value)){
    dl_notify("Sensor number, reference value and measured value must all be numbers");
    return H4_CMD_NOT_NUMERIC;
  }
  // Only calibrate inside a session (start one with cal_start)
  if(!cal.session_open()){
    dl_notify("No calibration in progress. Run cal_start first");
    return H4_CMD_NOT_NOW;
  }
  dl_notify("Setting %s coefficients for %s sensor %d", vs[0].c_str(), vs[1].c_str(), sensor_nb);
  // Forceds acceptance of serial port values no matter the timing
  cal.force_cal_time_reference(0);
  int ts = 0;
  if(abs_cal_type == "span"){ ts = 20; }
  // Set calibration values
  cal_report_result(cal.set_calibration_coeff(abs_cal_type, cal_gas, sensor_nb, ref_value, sen_value, ts));
  return H4_CMD_OK;
}

// Calibration through the Serial Port. Needs a session (cal_start).
// Documented in README.md. The syntax is:
// cal/zero/co2/1
// cal/zero/co2/1/0.54
// cal/span/h2o/0/1.31
// cal/span/co2/1/409.87
// cal/diff/co2/low
// cal/diff/co2/high
uint32_t cal_cmd(std::vector<std::string> vs) {
    if (vs.size() < 2) {
        dl_notify("Calibration needs at least 2 parameters: type and quantity");
        return H4_CMD_TOO_FEW_PARAMS;
    }
    std::string cal_type = vs[0];
    std::string cal_gas = vs[1];
    // Validate calibration type
    if (!utils.in_list(cal.calTypes, String(cal_type.c_str()))) {
        dl_notify("Invalid calibration type: %s (must be %s)", cal_type.c_str(), utils.make_text_list(cal.calTypes));
        return H4_CMD_PAYLOAD_FORMAT;
    }
    // Validate quantity. "all" means every quantity at once, which the old
    // dashboard offered as the first dropdown entry. It is rejected for span:
    // a span calibration needs one reference value per quantity.
    bool all_gases = (cal_gas == "all");
    if (all_gases && (cal_type == "span")) {
        dl_notify("Span must be calibrated one quantity at a time");
        return H4_CMD_PAYLOAD_FORMAT;
    }
    if (!all_gases && !utils.in_list(quantities, String(cal_gas.c_str()))) {
        dl_notify("Invalid quantity: %s (must be %s or all)", cal_gas.c_str(), utils.make_text_list(quantities));
        return H4_CMD_PAYLOAD_FORMAT;
    }
    int cal_sensor = 0;
    float cal_ref_value = 0.0;
    if (cal_type == "zero") {
        // Requires sensor (3 params), cal_ref_value optional (default 0.0)
        if (vs.size() == 2) {
            dl_notify("Zero calibration requires a sensor number");
            return H4_CMD_TOO_FEW_PARAMS;
        } else if (vs.size() == 3 || vs.size() == 4) {
            if(!parse_cmd_int(vs[2], cal_sensor)){
              dl_notify("The sensor number must be a number");
              return H4_CMD_NOT_NUMERIC;
            }
            if(vs.size() == 4 && !parse_cmd_float(vs[3], cal_ref_value)){
              dl_notify("The reference value must be a number");
              return H4_CMD_NOT_NUMERIC;
            }
            dl_notify("Zero calibration: %s, sensor %d, reference %.2f", cal_gas.c_str(), cal_sensor, cal_ref_value);
            // "all" zeroes every quantity in one pass; absolute_calibration then
            // reads 'sensor' as the number of sensors to probe, not an index.
            if(all_gases) absolute_calibration(quantities, "zero", numSensors, cal_ref_value);
            else          absolute_calibration({cal_gas.c_str()}, "zero", cal_sensor, cal_ref_value);
        } else {
            dl_notify("Too many parameters for zero calibration");
            return H4_CMD_TOO_MANY_PARAMS;
        }
    } else if (cal_type == "span") {
        // Requires sensor and cal_ref_value (4 params)
        if (vs.size() < 4) {
            dl_notify("Span calibration requires a sensor number and a reference value");
            return H4_CMD_TOO_FEW_PARAMS;
        } else if (vs.size() == 4) {
            if(!parse_cmd_int(vs[2], cal_sensor) || !parse_cmd_float(vs[3], cal_ref_value)){
              dl_notify("The sensor number and the reference value must be numbers");
              return H4_CMD_NOT_NUMERIC;
            }
            dl_notify("Span calibration: %s, sensor %d, reference %.2f", cal_gas.c_str(), cal_sensor, cal_ref_value);
            absolute_calibration({cal_gas.c_str()}, "span", cal_sensor, cal_ref_value);
        } else {
            dl_notify("Too many parameters for span calibration");
            return H4_CMD_TOO_MANY_PARAMS;
        }
    } else if (cal_type == "diff") {
        // Requires type, gas and low/high (3 params), no sensor/cal_ref_value
        if(vs.size() < 3) {
            dl_notify("Differential calibration needs low or high as the third parameter");
            return H4_CMD_TOO_FEW_PARAMS;
        } else if (vs.size() > 3) {
            dl_notify("Differential calibration takes no sensor number or reference value");
            return H4_CMD_TOO_MANY_PARAMS;
        } else {
            // "sen" is for the high calibration, "ref" for low
            String low_high = "";
            if(vs[2] == "low"){
              low_high = "ref";
            } else if (vs[2] == "high"){
              low_high = "sen";
            } else {
              dl_notify("Differential calibration takes low or high, not %s", vs[2].c_str());
              return H4_CMD_PAYLOAD_FORMAT;
            }
            dl_notify("Differential calibration: %s, %s point", cal_gas.c_str(), vs[2].c_str());
            if(all_gases) differential_calibration(quantities, low_high);
            else          differential_calibration({cal_gas.c_str()}, low_high);
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

  // Bounds-check the sensor index against how many of that type actually exist.
  // A type may have fewer instances than another (e.g. one SCD30 but two BME280),
  // so out-of-range requests (including differential's fixed sensor 1) return NaN.
  if(gas == "co2"){
  #if USE_SCD30
    if(sensor >= 0 && sensor < (int)SCD_COUNT) gas_measured = scd_sensors[sensor]->airCO2();
  #endif
  }
  if (gas == "o2"){
  #if USE_SEN0465
    if(sensor >= 0 && sensor < (int)SEN_COUNT) gas_measured = sen_sensors[sensor]->airO2();
  #endif
  }
  if (gas == "h2o"){
  #if USE_BME280
    // This sensor measures RH, but that's not useful for calibration, so the mole fraction has to be calculated
    if(sensor >= 0 && sensor < (int)BME_COUNT)
      gas_measured = env.air_water_mole_frac(bme_sensors[sensor]->airT(),
                                             bme_sensors[sensor]->airRH(),
                                             bme_sensors[sensor]->airP());
  #endif
  }
  if (gas == "temperature"){
  #if USE_BME280
    if(sensor >= 0 && sensor < (int)BME_COUNT) gas_measured = bme_sensors[sensor]->airT();
  #endif
  }
  if (gas == "pressure"){
  #if USE_BME280
    if(sensor >= 0 && sensor < (int)BME_COUNT) gas_measured = bme_sensors[sensor]->airP();
  #endif
  }
  
  return(gas_measured);
}

void save_calibration_coefficients(void){
  // Save calibration data to file
  String cal_data_str = "";
  String cal_header = "";
  String calibration_fn = "calibration.csv";
  #if USE_GPS
    if (!isGPSDateValid()) {
      dl_notify("Error: no GPS fix, calibration log not written");
      return;
    }
    calibration_fn = "calibration_" + gps.get_date() + ".csv";
    cal_header   = "timestamp_utc,";
    cal_data_str = gps.get_timestamp() + ",";
  #endif
  cal_header   += cal.get_all_cal_header(quantities, numSensors);
  cal_data_str += cal.get_all_cal_data(quantities, numSensors);
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
  // A span calibration needs one reference value per quantity, so it cannot be
  // run across several at once. Tested on the actual argument rather than on the
  // display selection, which has nothing to do with what was asked for.
  if((abs_cal_type == "span") && (gas_list.size() > 1)){
    dl_notify("Span must be calibrated one quantity at a time");
    return;
  }
  // Only calibrate inside a session, so it is clear when values get saved
  if (!cal_require_session()) { return; }
  // Don't run another calibration if it's already running
  if (calibration_running) {
    dl_notify("Calibration already running - ignoring request");
    return;
  }
  calibration_running = true;

  // Debug, show reference value
  Serial.print("Ref.: "); Serial.println(ref_value);

  // Decide how many sensors we are reading:
  // - When multiple quantities are passed, 'sensor' parameter holds # sensors to probe
  // - otherwise treat sensor as a single sensor id (so num_sensors = 1)
  const size_t num_gases = gas_list.size();
  int num_sensors = (num_gases > 1) ? sensor+1 : 1;
  if(num_sensors <= 0) num_sensors = 1; // Safeguard

  // shared accumulators (captured into lambdas by value), multiplying quantities by number of sensors
  auto cumulative_data = std::make_shared<std::vector<float>>(num_gases * num_sensors, 0.0f);
  const int n_measurements = 10;
  
  h4.nTimesRandom(
        // Measure "n_measurements" times (every ~1.0-1.1s) & average
        n_measurements, 1000, 1100,
        // Run at every iteration: measure every gas and accumulate
        [gas_list, cumulative_data, num_sensors, sensor,abs_cal_type]() {
            for (size_t gas = 0; gas < gas_list.size(); gas++) { // Cycle through all quantities
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
        },
        // Run when timer finishes: compute averages & store/set differential for each gas
        [gas_list, cumulative_data, sensor, num_sensors, n_measurements, abs_cal_type, ref_value]() {
            const unsigned long ts = calibration_time_s(); // single timestamp
            for (size_t gas = 0; gas < gas_list.size(); gas++) {
              for (int s = 0; s < num_sensors; s++) {
                float ref_val = 0;
                if((gas_list[gas] == "h2o") && (abs_cal_type == "span")){
                  // A span calibration uses a dew-point generator, so the reference
                  // is a dewpoint in °C: convert it to a mole fraction using this
                  // sensor's pressure. Guard the BME access: a co-located BME may
                  // not exist on channel s.
                  // A zero calibration uses dry gas instead, so its reference is
                  // already a mole fraction (0 for dry air) and needs no conversion.
                  float airp = 101325.0f;
                #if USE_BME280
                  if(s < (int)BME_COUNT) airp = bme_sensors[s]->airP();
                #endif
                  ref_val = env.dewpoint_to_mole_frac(ref_value, airp); // Dewpoint in °C
                } else {
                  ref_val = ref_value;
                }
                size_t idx = gas * num_sensors + s; // index into flattened vector
                float sensor_measured = (*cumulative_data)[idx] / static_cast<float>(n_measurements);
                // Skip sensors that don't exist (measure_gas returns NaN for an
                // out-of-range index) so we don't write phantom calibration entries.
                if(isnan(sensor_measured)) continue;
                // set differential for each gas
                cal_report_result(cal.set_calibration_coeff(abs_cal_type, gas_list[gas], s, ref_val, sensor_measured, ts));


                dl_notify("%s sensor %d: measured %.2f, reference %.2f%s",
                          gas_list[gas].c_str(), s, sensor_measured, ref_val,
                          (gas_list[gas] == "h2o") ? " mmol/mol" : "");
            }
            // reset accumulators (may be optional since shared_ptr is freed after lambda ends)
            std::fill(cumulative_data->begin(), cumulative_data->end(), 0.0f);

            // Values are only written to flash when the session is closed
            calibration_running = false;
            cal_session_touch();
          }
        },
        0, false
    );
}

// Differential calibration
void differential_calibration(const std::vector<String> gas_list, String dif_cal_type){
  // Only calibrate inside a session, so it is clear when values get saved
  if (!cal_require_session()) { return; }
  if (calibration_running) {
    dl_notify("Calibration already running - ignoring request");
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
        },
        // Run when timer finishes: compute averages & store/set differential for each gas
        [gas_list, cumulative_s1, cumulative_s2, n_measurements, dif_cal_type]() {
            const unsigned long ts = calibration_time_s(); // single timestamp
            for (size_t i = 0; i < gas_list.size(); ++i) {
                float avg0 = (*cumulative_s1)[i] / n_measurements;
                float avg1 = (*cumulative_s2)[i] / n_measurements;
                // set differential for each gas
                cal_report_result(cal.set_differential_coeff(dif_cal_type, gas_list[i], avg0, avg1, ts));

                dl_notify("%s: sensor 0 avg %.2f, sensor 1 avg %.2f", gas_list[i].c_str(), avg0, avg1);
            }

            // reset accumulators (optional since shared_ptr is freed after lambda ends)
            for (size_t i = 0; i < gas_list.size(); ++i) { (*cumulative_s1)[i] = 0.0f; (*cumulative_s2)[i] = 0.0f; }

            // Values are only written to flash when the session is closed
            calibration_running = false;
            cal_session_touch();
        },
        0, false
    );
}

/*
void onRTC(){
  Serial.println("Clock valid!");
  ClockValid = true;
  Serial.printf("\nReceived NTP time: %s (UTC)\n\n", CSTR(h4tk.strfDateTime( "%a %Y-%m-%d %H:%M:%S", h4tk.clockEPOCHLocal()) ));
}
*/

