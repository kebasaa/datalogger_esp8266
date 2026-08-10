/*
  Calibration routines for sensors. 
*/

#include "Calibration.h"

// Calibration must survive with no SD card in the slot, so it stores its values
// on the internal filesystem, not through lib/MicroSD.
#include <FS.h>
#include <LittleFS.h>
#define CAL_FS LittleFS
#include "h4_wrapper.h"
#include <cmath>
#include "Utils.h"

Cal::Cal(){
  // Nothing to be done here
}

/*
bool Cal::is_near_zero(float x, float eps){
    return std::fabs(x) <= eps;
}*/

// ---------------------------------------------------------------------------
//  Calibration sessions
//
//  H4 rewrites its whole persistent-globals file every time a stored variable
//  changes, so calibrating one value at a time meant one full flash write per
//  value. A session collects the changes in RAM and writes them once.
// ---------------------------------------------------------------------------

// How a value is turned into stored text and back again (defined further down)
static std::string encode_cal_value(float val);
static float decode_cal_value(const std::string& stored);

// ---------------------------------------------------------------------------
//  Where calibration values live
//
//  In the fixed _values array, and on LittleFS in /cal.dat. NOT in H4's
//  persistent-globals map, which is where they used to be: each entry there cost
//  roughly 120 bytes (an unordered_map node, plus the name stored twice because
//  h4proxy keeps its own copy of the key, plus two heap buffers whenever the name
//  passed the 15-character small-string limit). Sixty of those is about 7 KB, and
//  this board has around 10 KB of heap free with the access point running - so a
//  fully calibrated logger could not serve its own dashboard.
//
//  File format, one entry per line, then a checksum line:
//
//      sen_co2_0_zero=398.000000
//      ref_co2_0_zero=400.000000
//      crc=A31F
//
//  Name-keyed rather than positional on purpose. A positional record would
//  silently shift every value into the wrong slot the day someone adds a
//  quantity or changes the sensor count; here an unrecognised name is ignored
//  and a missing one simply stays NaN.
//
//  Only values that are actually set are written. "Absent" and "nan" mean the
//  same thing to read_var(), so there is nothing to gain by storing placeholders
//  - which is exactly the mistake the globals map version made.
// ---------------------------------------------------------------------------
#define CAL_FILE      "/cal.dat"
#define CAL_FILE_NEW  "/cal.new"

// CRC-16/CCITT, fed incrementally so the file can be checked while it streams
// rather than held in memory. MicroSD has its own copy, but it is private and
// whole-buffer, and calibration has to work with no SD card in the slot.
static uint16_t cal_crc16(uint16_t crc, const char* data, size_t len){
  for(size_t i = 0; i < len; i++){
    crc ^= (uint16_t)((uint8_t)data[i]) << 8;
    for(uint8_t b = 0; b < 8; b++){
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Map a stored name onto an array index. Names are built everywhere as
// "<dataType>_<gas>_<sensor>_<calType>" and no quantity contains an underscore,
// so exactly four parts is both necessary and sufficient.
int Cal::_slot(const std::string& name) const {
  if(!_storage_ready){ return -1; }
  size_t p1 = name.find('_');
  if(p1 == std::string::npos){ return -1; }
  size_t p3 = name.rfind('_');
  size_t p2 = name.rfind('_', p3 ? p3 - 1 : 0);
  if(p3 == std::string::npos || p2 == std::string::npos || p2 <= p1){ return -1; }

  std::string dataType = name.substr(0, p1);
  std::string gas      = name.substr(p1 + 1, p2 - p1 - 1);
  std::string sensor   = name.substr(p2 + 1, p3 - p2 - 1);
  std::string calType  = name.substr(p3 + 1);

  int d = -1;
  for(size_t i = 0; i < _dataTypes.size(); i++) if(_dataTypes[i] == dataType.c_str()) d = (int)i;
  int c = -1;
  for(size_t i = 0; i < calTypes.size(); i++) if(calTypes[i] == calType.c_str()) c = (int)i;
  int g = -1;
  for(size_t i = 0; i < _quantities.size(); i++) if(_quantities[i] == gas.c_str()) g = (int)i;
  if(d < 0 || c < 0 || g < 0){ return -1; }

  if(sensor.empty()){ return -1; }
  for(char ch : sensor) if(ch < '0' || ch > '9'){ return -1; }
  int s = atoi(sensor.c_str());
  if(s < 0 || s >= _numSensors){ return -1; }

  return (((d * (int)_quantities.size() + g) * _numSensors) + s) * (int)calTypes.size() + c;
}

bool Cal::_slot_name(size_t idx, std::string& out) const {
  if(!_storage_ready){ return false; }
  size_t nC = calTypes.size(), nS = (size_t)_numSensors, nQ = _quantities.size();
  if(nC == 0 || nS == 0 || nQ == 0){ return false; }
  size_t c = idx % nC;            idx /= nC;
  size_t s = idx % nS;            idx /= nS;
  size_t g = idx % nQ;            idx /= nQ;
  size_t d = idx;
  if(d >= _dataTypes.size()){ return false; }
  out  = std::string(_dataTypes[d].c_str()) + "_" + std::string(_quantities[g].c_str());
  out += "_" + std::string(String((int)s).c_str()) + "_" + std::string(calTypes[c].c_str());
  return true;
}

bool Cal::_load_file(const char* path){
  fs::File f = CAL_FS.open(path, "r");
  if(!f){ return false; }

  float    staged[CAL_SLOTS];
  for(size_t i = 0; i < CAL_SLOTS; i++){ staged[i] = float(NAN); }

  uint16_t crc = 0xFFFF;
  bool     saw_crc = false;
  bool     crc_ok  = false;

  while(f.available()){
    String raw = f.readStringUntil('\n');
    raw.trim();
    if(raw.length() == 0){ continue; }
    std::string line = std::string(raw.c_str());

    if(line.compare(0, 4, "crc=") == 0){
      // Everything before this line is what the checksum covers.
      saw_crc = true;
      crc_ok  = ((uint16_t)strtoul(line.substr(4).c_str(), nullptr, 16) == crc);
      break;
    }
    // Feed the line to the checksum exactly as it was written, newline included.
    crc = cal_crc16(crc, line.c_str(), line.size());
    crc = cal_crc16(crc, "\n", 1);

    size_t eq = line.find('=');
    if(eq == std::string::npos){ continue; }
    int idx = _slot(line.substr(0, eq));
    if(idx < 0){ continue; }                       // not a name this build knows
    staged[idx] = decode_cal_value(line.substr(eq + 1));
  }
  f.close();

  if(!saw_crc || !crc_ok){
    Serial.print(F("Calibration: ")); Serial.print(path);
    Serial.println(saw_crc ? F(" failed its checksum") : F(" has no checksum"));
    return false;
  }
  // Only now does anything reach the live array, so a rejected file cannot leave
  // half its contents behind.
  for(size_t i = 0; i < CAL_SLOTS; i++){ _values[i] = staged[i]; }
  return true;
}

bool Cal::_write_file(const char* path){
  fs::File f = CAL_FS.open(path, "w");
  if(!f){ return false; }
  uint16_t crc = 0xFFFF;
  for(size_t i = 0; i < CAL_SLOTS; i++){
    if(std::isnan(_values[i])){ continue; }        // absent == not calibrated
    std::string name;
    if(!_slot_name(i, name)){ continue; }
    std::string line = name + "=" + encode_cal_value(_values[i]);
    crc = cal_crc16(crc, line.c_str(), line.size());
    crc = cal_crc16(crc, "\n", 1);
    f.print(line.c_str()); f.print("\n");
  }
  char tail[16];
  snprintf(tail, sizeof(tail), "crc=%04X\n", crc);
  f.print(tail);
  f.close();
  return true;
}

// Write staged, never in place: a power cut during the write must leave either
// the previous values or the new ones, never half of each. The new file is
// written, read back and checksum-verified, and only then replaces the old one.
void Cal::_persist_values(){
  if(!_storage_ready){ return; }
  if(!_write_file(CAL_FILE_NEW)){
    Serial.println(F("Calibration: could not write " CAL_FILE_NEW));
    return;
  }
  // Verify what actually landed on flash, not what we believe we wrote.
  float snapshot[CAL_SLOTS];
  for(size_t i = 0; i < CAL_SLOTS; i++){ snapshot[i] = _values[i]; }
  if(!_load_file(CAL_FILE_NEW)){
    for(size_t i = 0; i < CAL_SLOTS; i++){ _values[i] = snapshot[i]; }
    Serial.println(F("Calibration: " CAL_FILE_NEW " did not verify, keeping the old file"));
    return;
  }
  CAL_FS.remove(CAL_FILE);
  if(!CAL_FS.rename(CAL_FILE_NEW, CAL_FILE)){
    Serial.println(F("Calibration: could not rename " CAL_FILE_NEW));
  }
}

// Pull values written by an older firmware out of H4's globals, then delete them
// from there so the heap they were costing is actually released.
size_t Cal::_migrate_from_globals(){
  size_t moved = 0;
  for(size_t i = 0; i < CAL_SLOTS; i++){
    std::string name;
    if(!_slot_name(i, name)){ continue; }
    if(!h4_gvExists(name)){ continue; }
    float v = decode_cal_value(h4_gvGetString(name));
    if(!std::isnan(v)){ _values[i] = v; moved++; }
    h4_gvSetSave(name, false);      // batch: one flash write at the end, not 60
    h4_gvErase(name);
  }
  if(moved){ _persist_values(); }
  h4_gvPersist();
  return moved;
}

void Cal::storage_begin(const std::vector<String>& quantities, int numSensors){
  _quantities = quantities;
  _numSensors = numSensors;
  if(_quantities.size() > MAX_QUANTITIES || (size_t)numSensors > MAX_SENSORS){
    Serial.println(F("Calibration: too many quantities or sensors for the value store"));
    _storage_ready = false;
    return;
  }
  for(size_t i = 0; i < CAL_SLOTS; i++){ _values[i] = float(NAN); }
  _storage_ready = true;

  // /cal.dat first; /cal.new is what survives a power cut between writing the
  // replacement and renaming it into place.
  if(_load_file(CAL_FILE)){ return; }
  if(_load_file(CAL_FILE_NEW)){
    Serial.println(F("Calibration: recovered from " CAL_FILE_NEW " after an interrupted write"));
    _persist_values();
    return;
  }
  size_t moved = _migrate_from_globals();
  if(moved){
    Serial.print(F("Calibration: migrated ")); Serial.print(moved);
    Serial.println(F(" value(s) out of the globals store"));
  }
}

bool Cal::session_open(void) const {
  return _session;
}

size_t Cal::pending_count(void) const {
  return _pending.size();
}

void Cal::open_session(void){
  if(_session){ return; }
  _pending.clear();
  _session = true;
}

void Cal::commit_session(void){
  if(!_session){ return; }
  // Fold the session into the live array, then write the file exactly once.
  // Values that already match are skipped, so a session that changed nothing in
  // the end costs no write at all.
  //
  // Note what this does NOT do: it never reads the file back and merges. The
  // array was loaded complete at boot and has been the single source of truth
  // since, so a partial calibration rewrites every other value exactly as it
  // already was. There is no read-modify-write for a power cut to catch halfway.
  size_t changed = 0;
  for(auto& entry : _pending){
    int idx = _slot(entry.first);
    if(idx < 0){ continue; }
    bool same = (std::isnan(_values[idx]) && std::isnan(entry.second)) ||
                (_values[idx] == entry.second);
    if(same){ continue; }
    _values[idx] = entry.second;
    changed++;
  }
  _pending.clear();
  _session = false;

  if(changed){
    _persist_values();
    Serial.print("Calibration: saved "); Serial.print(changed); Serial.println(" value(s)");
  }
}

void Cal::force_cal_time_reference(unsigned long secs_since_midnight){
  zero_cal_time_s = secs_since_midnight;
  diff_low_cal_time_s = secs_since_midnight;
}

void Cal::discard_session(void){
  if(!_session){ return; }
  size_t dropped = _pending.size();
  _pending.clear();
  _session = false;
  Serial.print("Calibration session discarded, "); Serial.print(dropped); Serial.println(" change(s) thrown away");
}

// ---------------------------------------------------------------------------
//  Value encoding
//
//  Values are stored as plain decimal numbers, with "nan" for "not calibrated".
//  Older firmware stored them as the value multiplied by 100000 in a 32-bit
//  integer, which could not represent anything above 21474 (air pressure in Pa,
//  for example). decode_cal_value() still understands that older format, so
//  calibrations survive a firmware update; the next write converts them.
// ---------------------------------------------------------------------------

static std::string encode_cal_value(float val){
  if(isnan(val)){ return std::string("nan"); }
  return std::string(String(val, 6).c_str());
}

static float decode_cal_value(const std::string& stored){
  if(stored.empty()){ return float(NAN); }
  // A number with no decimal point, exponent, "nan" or "inf" in it was written
  // by the older firmware, so undo its multiplication by 100000.
  if(stored.find_first_of(".eEnNiI") == std::string::npos){
    long legacy = strtol(stored.c_str(), nullptr, 10);
    if(legacy == -9999){ return float(NAN); }  // the old "not calibrated" marker
    return (float)(legacy / 100000.0);
  }
  const char* first = stored.c_str();
  char* last = nullptr;
  float val = strtof(first, &last);
  if((last == first) || isnan(val)){ return float(NAN); }
  return val;
}

// Create all calibration variables in persistent storage
// ---------------------------------------------------------------------------
//  purge_placeholder_calibrations() used to live here.
//
//  It is gone rather than stubbed. Nothing seeds placeholders any more, and
//  _write_file() only ever writes slots holding a real measurement, so there is
//  nothing left to purge. Clearing out what an older firmware left in H4's
//  globals is now storage_begin()'s job, and it happens once.
// ---------------------------------------------------------------------------

void Cal::show_all_calibrations(std::vector<String> quantities, int numSensors) {
  for (auto& dataType : _dataTypes) {
    for (auto& gas : quantities) {
      for (int sensor = 0; sensor < numSensors; sensor++) {
        for (auto& calType : calTypes) {
          // In the case of differential calibration, dataType == "ref" is low values and "sen" is high values
          String var_name = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          Serial.print("  - "); Serial.print(var_name); Serial.print(": "); //DEBUG
          Serial.println(read_var(var_name));
        }
      }
    }
  }
}

// Reset all calibration variables in persistent storage.
// Deletes them rather than storing "nan": to every reader the two are identical
// (read_var() returns NaN either way), but writing them back would re-create the
// placeholders this firmware deliberately does not keep.
void Cal::reset_all_calibrations(std::vector<String> quantities, int numSensors){
  size_t erased = 0;
  for (auto& dataType : _dataTypes) {
    for (auto& gas : quantities) {
      for (int sensor = 0; sensor < numSensors; sensor++) {
        for (auto& calType : calTypes) {
          // In the case of differential calibration, dataType == "ref" is low values and "sen" is high values
          String var_name = dataType + "_" + gas + "_" + String(sensor) + "_" + calType;  // E.g., ref_h2o_1_zero
          std::string name = std::string(var_name.c_str());
          if(std::isnan(read_var(var_name)) && (_pending.find(name) == _pending.end())){ continue; }
          erase_var(name);
          erased++;
        }
      }
    }
  }
  // erase_var() deliberately does not write, so the whole sweep costs one write.
  if(erased){ _persist_values(); }
  Serial.print("Reset "); Serial.print(erased); Serial.println(" calibration value(s)");
}

int Cal::set_calibration_coeff(String calType, String currentGas, int currentSensor, float ref, float measured, unsigned long secs_since_midnight){
  // Save the variable to persistent storage
  String var_reference = "ref_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Actual real zero value
  String var_measured =  "sen_" + currentGas + "_" + String(currentSensor) + "_" + calType; // Sensor measurement for the zero
  
  if(calType == "zero"){
    zero_cal_time_s = secs_since_midnight;
  }

  // Time since the zero calibration. Both times are seconds since midnight, so
  // the difference is negative when the clock passed midnight in between; the
  // subtraction has to be done signed for that to be visible at all.
  if(zero_cal_time_s == NO_CAL_TIME){
    // No zero calibration has been done since the logger was switched on
    if(calType == "span"){
      Serial.println("No zero calibration since power-on. Redo 0 calibration");
      return(1); // Error 1: no usable zero calibration to compare against
    }
  }
  long secs_since_zero_cal = (long)secs_since_midnight - (long)zero_cal_time_s;
  if(secs_since_zero_cal < 0){
    secs_since_zero_cal += 86400; // Time counting went across midnight, wrap around
  }
  Serial.print("Secs since zero cal: "); Serial.println(secs_since_zero_cal);

  // Tests if too little or too much time has passed since zero calibration. For temperature we allow long time intervals
  if((calType == "span") && (secs_since_zero_cal > 7200) && (currentGas != "temperature")){
    Serial.println(">2h since zero calibration. Redo 0 calibration");
    return(1); // Error 1: Too much time since zero calibration
  } else if((calType == "span") && (secs_since_zero_cal < 10)){
    Serial.println("<10s since zero calibration. Are you sure the span gas is stable?");
    return(2); // Error 2: Too little time since zero calibration
  } else if (calType != "span"){
    Serial.println("Zero calibration:");
  } else {
    Serial.println("Span calibration:");
  }
  // Update the values
  Serial.print("    var: "); Serial.print(var_reference); Serial.print(" - "); Serial.println(ref);
  Serial.print("    mes: "); Serial.print(var_measured); Serial.print(" - "); Serial.println(measured);
  update_var(var_reference, ref);
  update_var(var_measured, measured);

  Serial.print("Setting: "); Serial.print(var_measured); Serial.print(" - "); Serial.println(var_reference);//DEBUG

  return(0);
}

float Cal::read_calibration_var(String dataType, String calType, String currentGas, int currentSensor){
  // Read only sensor measurement values from persistent storage. We assume that reference values are known
  // dataType is either "sen" or "ref"
  if((dataType != "sen") && (dataType != "ref")){
    return(float(NAN));
  }
  // calType is either "zero" or "span"
  if((calType != "zero") && (calType != "span")){
    return(float(NAN));
  }
  if((currentGas == "All") || (currentSensor == -9999)){
    return(float(NAN));
  }
  // No tracing here. This is called dozens of times per /api/cal request, and on
  // an ESP8266 every Serial.print blocks the cooperative H4 loop until the UART
  // has drained - at 115200 baud, during exactly the window a page transfer may
  // be in flight. E.g., sen_h2o_1_zero
  return(read_var(dataType + "_" + currentGas + "_" + String(currentSensor) + "_" + calType));
}

Cal::CalibrationCoeffs Cal::get_calibration_coefficients(String currentGas, int currentSensor){
  CalibrationCoeffs coeffs;
  // default -> raw
  coeffs.gain = 1.0f;
  coeffs.offset = 0.0f;
  coeffs.flag = -1;

  int otherSensor = (currentSensor == 0) ? 1 : 0;

  // ------------------ READ NEEDED VALUES ONCE ------------------
  // Get absolute calibration variables for current sensor
  float var_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero"); // Reference gas, zero
  float var_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span"); // Reference gas, span
  float var_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero"); // Sensor reading at zero
  float var_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span"); // Sensor reading at span

  // Absolute values for otherSensor (needed in many cases)
  float other_ref_zero = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_ref_span = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_span");
  float other_sen_zero = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_sen_span = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_span");

  // Differential values (read once)
  // Mapping used here:
  //   cur_low  = reference low (cur_low)
  //   cur_high  = reference high (cur_high)
  //   other_low  = sensor1 low  (other_low)
  //   S_high  = sensor1 high (other_high)
  float R_low    = read_var("ref_" + currentGas + "_0_diff"); // Sensor 0 (the reference), low value
  float R_high   = read_var("sen_" + currentGas + "_0_diff"); // Sensor 0 (the reference), high value
  float S_low  = read_var("ref_" + currentGas + "_1_diff"); // Sensor 1 (to be adjusted), low value
  float S_high = read_var("sen_" + currentGas + "_1_diff"); // Sensor 1 (to be adjusted), high value

  auto has = [&](float v){ return !std::isnan(v); };
  bool abs_all_present = has(var_ref_zero) && has(var_ref_span) && has(var_sen_zero) && has(var_sen_span);
  bool diff_all_present = has(R_low) && has(R_high) && has(S_low) && has(S_high);

  // ------------------ 1) Full absolute calibration for currentSensor ------------------
  if (abs_all_present) {
    // compute gain/offset normally
    if (Utils::is_near_zero(var_sen_span - var_sen_zero)) {
      coeffs.flag = -1;
      coeffs.gain = 1.0f; coeffs.offset = 0.0f;
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Absolute present but sensor span==zero -> returning raw (-1).");
      return coeffs;
    }
    coeffs.gain = (var_ref_span - var_ref_zero) / (var_sen_span - var_sen_zero);
    coeffs.offset = var_ref_zero - coeffs.gain * var_sen_zero;
    coeffs.flag = 0;
    Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
    Serial.println(": Return full absolute calibration (flag 0).");
    return coeffs;
  }

  // ------------------ 2) Full differential available -> multiple sub-cases ------------------
  if (diff_all_present) {

    // If no slope in S, differential invalid
    if (Utils::is_near_zero(S_high - S_low)) {
      // cannot compute differential slope meaningfully
      coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Differential present but sensor slope zero -> returning raw (-1).");
      return coeffs;
    }

    // differential slope S->R for sensor1
    float gain_diff = (R_high - R_low) / (S_high - S_low);
    float offset_diff = R_low - gain_diff * S_low; // this maps a sensor1 raw -> reference units

    // 2a) If currentSensor has either absolute zero pair OR absolute span pair,
    //     then:
    //       - if currentSensor == 0 -> it's the reference: do not adjust it (raw)
    //       - if currentSensor == 1 -> we can do differential + absolute offset (flag = 1)
    if ((has(var_ref_zero) && has(var_sen_zero)) || (has(var_ref_span) && has(var_sen_span))) {
      if (currentSensor == 0) {
        coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
        Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
        Serial.println(": currentSensor==0 and has absolute partial: do not apply differential to reference -> raw (-1).");
        return coeffs;
      } else {
        // sensor 1: differential gain & offset from diffs, plus adjust offset to align with absolute point(s)
        // If we have zero pair for currentSensor use that to compute absolute adjustment:
        float adj_offset = offset_diff; // start with diff offset
        if (has(var_ref_zero) && has(var_sen_zero)) {
          // measured at zero (sen_zero) should map to ref_zero. Compute correction = ref_zero - (gain_diff * sen_zero + offset_diff)
          float predicted_ref_from_diff = gain_diff * var_sen_zero + offset_diff;
          adj_offset = offset_diff + (var_ref_zero - predicted_ref_from_diff);
        } else if (has(var_ref_span) && has(var_sen_span)) {
          float predicted_ref_from_diff = gain_diff * var_sen_span + offset_diff;
          adj_offset = offset_diff + (var_ref_span - predicted_ref_from_diff);
        }
        coeffs.flag = 1;
        coeffs.gain = gain_diff;
        coeffs.offset = adj_offset;
        Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
        Serial.println(": Differential + absolute offset computed for currentSensor (flag 1).");
        return coeffs;
      }
    }

    // 2b) If otherSensor has either absolute zero pair OR absolute span pair
    if ((has(other_ref_zero) && has(other_sen_zero)) || (has(other_ref_span) && has(other_sen_span))) {
      if (currentSensor == 0) {
        // Use differential mapping and otherSensor absolute offset to compute equivalent offset for currentSensor.
        // Since sensor0 is reference, leave it raw.
        coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
        Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
        Serial.println(": currentSensor==0 and other sensor has absolute -> reference kept raw (flag -1).");
        return coeffs;
      } else {
        // currentSensor == 1, compute differential then adjust offset to match otherSensor's absolute pair
        float adj_offset = offset_diff;
        if (has(other_ref_zero) && has(other_sen_zero)) {
          float predicted_ref_from_diff = gain_diff * other_sen_zero + offset_diff;
          adj_offset = offset_diff + (other_ref_zero - predicted_ref_from_diff);
        } else if (has(other_ref_span) && has(other_sen_span)) {
          float predicted_ref_from_diff = gain_diff * other_sen_span + offset_diff;
          adj_offset = offset_diff + (other_ref_span - predicted_ref_from_diff);
        }
        coeffs.flag = 1;
        coeffs.gain = gain_diff;
        coeffs.offset = adj_offset;
        Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
        Serial.println(": Differential + offset computed for currentSensor using otherSensor absolute (flag 1).");
        return coeffs;
      }
    }

    // 2c) Neither sensor has absolute pairs -> differential-only
    if (!has(other_ref_zero) && !has(other_ref_span) && !has(var_ref_zero) && !has(var_ref_span)) {
      if (currentSensor == 0) {
        coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
        Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
        Serial.println(": No absolute pairs anywhere: currentSensor==0 -> raw (-1).");
        return coeffs;
      } else {
        coeffs.flag = 2; // differential only
        coeffs.gain = gain_diff;
        coeffs.offset = offset_diff;
        Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
        Serial.println(": Differential only for sensor1 (flag 2).");
        return coeffs;
      }
    }
  } // end diff_all_present

  // ------------------ 3) No differential present -> try absolute partial offset for currentSensor ------------------
  // If currentSensor has either zero pair (ref+meas) or span pair, return simple absolute offset
  if ( (has(var_ref_zero) && has(var_sen_zero)) || (has(var_ref_span) && has(var_sen_span)) ) {
    coeffs.flag = 4;
    coeffs.gain = 1.0f;
    if (has(var_ref_zero) && has(var_sen_zero)) {
      coeffs.offset = var_ref_zero - var_sen_zero;
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Absolute zero pair present -> simple absolute offset (flag 4).");
    } else {
      coeffs.offset = var_ref_span - var_sen_span;
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Absolute span pair present -> simple absolute offset (flag 4).");
    }
    return coeffs;
  }

  // ------------------ 4) Partial diffs exist (only lows or only highs) and no absolute for currentSensor ------------------
  // If currentSensor==0 -> raw
  if ( (has(R_low) && has(S_low)) || (has(R_high) && has(S_high)) ) {
    if (currentSensor == 0) {
      coeffs.flag = -1; coeffs.gain = 1.0f; coeffs.offset = 0.0f;
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Only partial diffs and currentSensor==0 -> raw (-1).");
      return coeffs;
    } else {
      // currentSensor == 1 -> differential offset only
      coeffs.flag = 3;
      coeffs.gain = 1.0f;
      if (has(R_low) && has(S_low)) coeffs.offset = R_low - S_low;
      else coeffs.offset = R_high - S_high;
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Partial diffs -> differential offset for sensor1 (flag 3).");
      return coeffs;
    }
  }

  // ------------------ 5) Nothing usable -> raw fallback ------------------
  coeffs.flag = -1;
  coeffs.gain = 1.0f;
  coeffs.offset = 0.0f;
  // Deliberately silent.
  //
  // "no calibration yet" is the normal state of an uncalibrated logger, not an
  // event, and this fired once per gas per sensor. /api/cal walks every
  // combination, so one press of the Calibration tab produced ten of these -
  // measured four such bursts in a single two-minute session. At 115200 baud
  // that is ~50 ms of blocking serial output per burst, on a single-threaded
  // device that cannot service TCP while it writes. The flag (-1) already tells
  // every caller the same thing, and the web UI shows it.
  return coeffs;
}

void Cal::fix_all_calibrations(std::vector<String> quantities, int numSensors){
  for (auto& gas : quantities) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      if(!fix_calibration_coefficients(gas, sensor)){
        Serial.print("Calibration values of "); Serial.print(gas); Serial.print(" for sensor "); Serial.print(sensor); Serial.println(" fixed");
      }
    }
  }
}

int Cal::fix_calibration_coefficients(String currentGas, int currentSensor){
  // returns 0 if nothing needed or fixes applied successfully, >0 to indicate non-fatal conditions
  int otherSensor = (currentSensor == 0) ? 1 : 0;

  // Read all absolute & differential values for both sensors once
  float cur_ref_zero = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero"); // Reference gas, zero
  float cur_ref_span = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_span"); // Reference gas, span
  float cur_sen_zero = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero"); // Sensor reading at zero
  float cur_sen_span = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_span"); // Sensor reading at span

  // Absolute values for otherSensor (needed in many cases)
  float other_ref_zero = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_ref_span = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_span");
  float other_sen_zero = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero");
  float other_sen_span = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_span");

  float cur_low    = read_var("ref_" + currentGas + "_" + String(currentSensor) + "_diff"); // current sensor, low value
  float cur_high   = read_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff"); // current sensor, high value
  float other_low  = read_var("ref_" + currentGas + "_" + String(otherSensor) + "_diff");   // other sensor, low value
  float other_high = read_var("sen_" + currentGas + "_" + String(otherSensor) + "_diff");   // other sensor, high value

  auto has = [&](float v){ return !std::isnan(v); };

  // ---------- 1) If both sensors have full absolute pairs, but diffs missing -> create diffs ----------
  //-----------------------------------------------------------------------------------------------------
  bool currentSensor_abs_full = has(cur_ref_zero) &&   has(cur_ref_span) &&   has(cur_sen_zero) &&   has(cur_sen_span);
  bool otherSensor_abs_full =   has(other_ref_zero) && has(other_ref_span) && has(other_sen_zero) && has(other_sen_span);

  if (currentSensor_abs_full && otherSensor_abs_full) {
    // Create diffs from absolute pairs if missing
    if (std::isnan(cur_low) || std::isnan(other_low)){
	    update_var("ref_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_zero); // current sensor, low value
	    update_var("ref_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_zero); // other sensor, low value
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Created missing differential entries from full absolute pairs.");
      return 0;
	  }
    if (std::isnan(cur_high) || std::isnan(other_high)){
	    update_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_span); // current sensor, high value
	    update_var("sen_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_span); // other sensor, high value
      Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
      Serial.println(": Created missing differential entries from full absolute pairs.");
      return 0;
	  }
  }

  // ---------- 2) If some absolute pair and full diffs present, we can compute abs for missing values ----------
  //-------------------------------------------------------------------------------------------------------------
  bool diffs_present = has(cur_low) && has(cur_high) && has(other_low) && has(other_high);

  if (diffs_present) {
	if (!Utils::is_near_zero(cur_high - cur_low) && !Utils::is_near_zero(other_high - other_low)) {
	  // Gain & offset from diffs (maps current to other sensor)
	  float d_gain_CO   = (cur_high - cur_low) / (other_high - other_low);
	  float d_offset_CO = cur_low - d_gain_CO * other_low;
	  // Gain & offset from diffs (maps other to current sensor)
	  float d_gain_OC   = (other_high - other_low) / (cur_high - cur_low);
	  float d_offset_OC = other_low - d_gain_OC * cur_low;
	  
	  // Flags
	  bool cur_zero_present   = !std::isnan(cur_sen_zero)   && !std::isnan(cur_ref_zero);
	  bool cur_span_present   = !std::isnan(cur_sen_span)   && !std::isnan(cur_ref_span);
	  bool other_zero_present = !std::isnan(other_sen_zero) && !std::isnan(other_ref_zero);
	  bool other_span_present = !std::isnan(other_sen_span) && !std::isnan(other_ref_span);
	  
      // a) Determine current sensor's zero & span from other sensor
      //============================================================
	  if(otherSensor_abs_full && (std::isnan(cur_sen_zero) || std::isnan(cur_sen_span))) {
		// Calculate the theoretical measured calibration values
		float cur_sen_zero_calc = d_gain_CO * other_sen_zero + d_offset_CO;
		float cur_sen_span_calc = d_gain_CO * other_sen_span + d_offset_CO;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero", other_ref_zero);
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_span", other_ref_span);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero", cur_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_span", cur_sen_span_calc);
		Serial.print("ref_" + currentGas + "_" + String(currentSensor) + "_zero: "); Serial.println(other_ref_zero);
		Serial.print("ref_" + currentGas + "_" + String(currentSensor) + "_span: "); Serial.println(other_ref_span);
		Serial.print("sen_" + currentGas + "_" + String(currentSensor) + "_zero: "); Serial.println(cur_sen_zero_calc);
		Serial.print("sen_" + currentGas + "_" + String(currentSensor) + "_span: "); Serial.println(cur_sen_span_calc);
		return 0;
	  }
	  // b) Determine other sensor's zero & span from current sensor
      //=============================================================
	  if(currentSensor_abs_full && (std::isnan(other_sen_zero) || std::isnan(other_sen_span))) {
		// Calculate the theoretical measured calibration values
		float other_sen_zero_calc = d_gain_OC * cur_sen_zero + d_offset_OC;
		float other_sen_span_calc = d_gain_OC * cur_sen_span + d_offset_OC;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero", cur_ref_zero);
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_span", cur_ref_span);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero", other_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_span", other_sen_span_calc);
		Serial.print("ref_" + currentGas + "_" + String(otherSensor) + "_zero: "); Serial.println(cur_ref_zero);
		Serial.print("ref_" + currentGas + "_" + String(otherSensor) + "_span: "); Serial.println(cur_ref_span);
		Serial.print("sen_" + currentGas + "_" + String(otherSensor) + "_zero: "); Serial.println(other_sen_zero_calc);
		Serial.print("sen_" + currentGas + "_" + String(otherSensor) + "_span: "); Serial.println(other_sen_span_calc);
		return 0;
	  }
	  // c) Determine from current sensor's zero & other sensor's span
      //==============================================================
	  if (cur_zero_present && !other_zero_present && !cur_span_present && other_span_present) {
		// Calculate the theoretical measured calibration values
		float other_sen_zero_calc = d_gain_OC * cur_sen_zero + d_offset_OC;
		float cur_sen_span_calc = d_gain_CO * other_sen_span + d_offset_CO;
		// Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
        update_var("ref_" + currentGas + "_" + String(otherSensor) + "_zero", cur_ref_zero);
        update_var("ref_" + currentGas + "_" + String(currentSensor) + "_span", other_ref_span);
        update_var("sen_" + currentGas + "_" + String(otherSensor) + "_zero", other_sen_zero_calc);
        update_var("sen_" + currentGas + "_" + String(currentSensor) + "_span", cur_sen_span_calc);
		return 0;
	  }
	  // d) Determine from other sensor's zero & current sensor's span
      //==============================================================
	  if (!cur_zero_present && other_zero_present && cur_span_present && !other_span_present) {
		  // Calculate the theoretical measured calibration values
		  float cur_sen_zero_calc = d_gain_CO * other_sen_zero + d_offset_CO;
		  float other_sen_span_calc = d_gain_OC * cur_sen_span + d_offset_OC;
		  // Persist abs values for sensor1 if missing, using the other sensor's reference gas measurements as reference
      update_var("ref_" + currentGas + "_" + String(currentSensor) + "_zero", other_ref_zero);
      update_var("ref_" + currentGas + "_" + String(otherSensor) + "_span", cur_ref_span);
      update_var("sen_" + currentGas + "_" + String(currentSensor) + "_zero", cur_sen_zero_calc);
      update_var("sen_" + currentGas + "_" + String(otherSensor) + "_span", other_sen_span_calc);
		  return 0;
	    }
    }
  }
  
  // ---------- 3) If only one differential (low or high) exists but there is a corresponding absolute pair for the other for both,
  //               we can create a full set of diffs by combining the absolute values
  //-------------------------------------------------------------------------------------------------------------

  bool diff_low_pair  = has(cur_low) && has(other_low);
  bool diff_high_pair = has(cur_high) && has(other_high);
  bool abs_zero_pair  = has(cur_sen_zero) && has(other_sen_zero);
  bool abs_span_pair  = has(cur_sen_span) && has(other_sen_span);
  
  if(diff_low_pair && abs_span_pair && !diff_high_pair){
    // Create diff_high_pair using absolute span pair
	  update_var("sen_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_span); // current sensor, high value
	  update_var("sen_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_span); // other sensor, high value 
  }
  if(abs_zero_pair && diff_high_pair && !diff_low_pair){
    // Create diff_low_pair using absolute zero pair
	  update_var("ref_" + currentGas + "_" + String(currentSensor) + "_diff", cur_sen_zero); // current sensor, low value
	  update_var("ref_" + currentGas + "_" + String(otherSensor) + "_diff", other_sen_zero); // other sensor, low value
  }

  // ---------- 4) If we can't reconstruct meaningful missing values, do nothing ----------
  //Serial.print(currentGas);Serial.print(" sensor ");Serial.print(currentSensor);
  //Serial.println(": No further reconstruction possible (either only single isolated points available or nothing).");
  return 1;
}

Cal::CalibrationResult Cal::calibrate_linear(String currentGas, int currentSensor, float currentMeasurement){
  CalibrationCoeffs coeffs = get_calibration_coefficients(currentGas, currentSensor);
  CalibrationResult res; // To store the results

  // Apply calibration
  res.flag = coeffs.flag; // Indicate the type of calibration done
  res.calibratedValue = coeffs.gain * currentMeasurement + coeffs.offset;

  return(res);
}

// Read persistently stored variable
float Cal::read_var(String var_type){
  std::string name = std::string(var_type.c_str());
  // Values changed during an open session are still in RAM
  std::map<std::string, float>::const_iterator pending = _pending.find(name);
  if(pending != _pending.end()){
    return(pending->second);
  }
  // An unset slot and a slot holding nan are the same thing, and so is a name
  // this build has no slot for at all: all three read back as NaN, by the same
  // path, so a caller can never get a stale or wrong number out of one.
  int idx = _slot(name);
  if(idx < 0){ return(float(NAN)); }
  return(_values[idx]);
}

// Store one value. "save" controls whether the file is rewritten now; callers
// working through a set pass false and write once at the end.
void Cal::write_var(const std::string& name, float val, bool save){
  int idx = _slot(name);
  if(idx < 0){ return; }
  _values[idx] = val;
  if(save){ _persist_values(); }
}

void Cal::create_var(String var_type, float val){
  std::string name = std::string(var_type.c_str());
  // Only fill a slot that is genuinely empty - this check is what stops an
  // existing calibration being overwritten.
  if(!std::isnan(read_var(var_type)) || (_pending.find(name) != _pending.end())){
    return;
  }
  if(_session){
    _pending[name] = val;
  } else {
    write_var(name, val, true);
  }
}

void Cal::update_var(String var_type, float val){
  std::string name = std::string(var_type.c_str());
  if(_session){
    // Kept in RAM until the session is committed, so this costs no flash write
    _pending[name] = val;
    return;
  }
  write_var(name, val, true);
}

// Clear one calibration value, in RAM and in any open session. Deliberately does
// NOT write the file - callers erasing a set write once at the end.
void Cal::erase_var(const std::string& name){
  _pending.erase(name);
  int idx = _slot(name);
  if(idx < 0){ return; }
  _values[idx] = float(NAN);
}

// Cycles through all calibration variables and adds their names into a single string
String Cal::get_all_cal_header(std::vector<String> quantities, int numSensors){
  String cal_header = "";
  for (auto& gas : quantities) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      for (auto& dataType : _dataTypes) {
        for (auto& calType : calTypes) {
          cal_header += dataType + "_" + gas + "_" + String(sensor) + "_" + calType + ",";
        }
      }
      // Adds the coefficients (slope/gain & intercept/offset)
      cal_header += gas + "_" + String(sensor) + "_gain,";
      cal_header += gas + "_" + String(sensor) + "_offset,";
      cal_header += gas + "_" + String(sensor) + "_flag,";
    }
  }
  return(cal_header);
}

// Cycles through all calibration variables and adds their values into a single string
String Cal::get_all_cal_data(std::vector<String> quantities, int numSensors){
  String cal_data = "";
  CalibrationCoeffs coeffs;
  for (auto& gas : quantities) {
    for (int sensor = 0; sensor < numSensors; sensor++) {
      for (auto& dataType : _dataTypes) {
        for (auto& calType : calTypes) {
          cal_data += String(read_var(dataType + "_" + gas + "_" + String(sensor) + "_" + calType), 6) + ",";
        }
      }
      // Adds the coefficients (slope/gain & intercept/offset)
      coeffs = get_calibration_coefficients(gas, sensor);
      cal_data += String(coeffs.gain) + ",";
      cal_data += String(coeffs.offset) + ",";
      cal_data += String(coeffs.flag) + ",";
    }
  }
  return(cal_data);
}

// dataType is "ref" or "sen", where "ref" is the low value and "sen" is high value
int Cal::set_differential_coeff(String dataType, String currentGas,
                                float sen0_measurement, float sen1_measurement,
                                unsigned long secs_since_midnight){
  // Save the variable to persistent storage
  String var_sen0 = dataType + "_" + currentGas + "_0_diff"; // Sensor 1 value, e.g. ref_co2_1_diff
  String var_sen1 = dataType + "_" + currentGas + "_1_diff"; // Sensor 2 value

  // First check if the value of "ref" (low) is too similar to that of "sen" (high)
  float var_sen0_low = read_var("ref_" + currentGas + "_0_diff");
  float var_sen1_low = read_var("ref_" + currentGas + "_1_diff");
  if((dataType == "sen") &
     ((std::fabs(sen0_measurement - var_sen0_low) < 100) |
      (std::fabs(sen1_measurement - var_sen1_low) < 100))){
    Serial.println("High calibration value too similar or below that of low calibration. Please repeat calibration");
    // Therefore, set to 0 and to the difference so that the "high" calibration value gets ignored
    // This results in a slope of 1 and an intercept that always yields just the plain difference
    update_var(var_sen0, 0);
    update_var(var_sen1, read_var("ref_" + currentGas + "_1_diff")); // Theoretically it should be ref_2 - ref_1, but ref_1=0
    return(1); // Error 1 = Not enough difference between high and low values
  }

  // Calculate time between calibrations
  if(dataType == "ref"){
    diff_low_cal_time_s = secs_since_midnight;
  }
  if(diff_low_cal_time_s == NO_CAL_TIME){
    // No low calibration has been done since the logger was switched on
    Serial.println("No low calibration since power-on. Do the low calibration first");
    return(5); // Error 5 = No "low" calibration to compare against
  }
  // Calculated time is 0 when "ref" (i.e., the low diff calibration)
  // Otherwise it's a known number of seconds. Both times are seconds since
  // midnight, so the subtraction has to be signed for a value from before
  // midnight to show up as negative rather than as a huge positive number.
  long time_since_last_cal = (long)secs_since_midnight - (long)diff_low_cal_time_s;
  if(time_since_last_cal < 0){
    time_since_last_cal += 86400; // Time counting went across midnight, wrap around
  }
  Serial.print("Secs since diff low cal: "); Serial.println(time_since_last_cal);

  // Check how much time has passed
  if((time_since_last_cal > 0) && (time_since_last_cal < 10)){
    Serial.println("<10s since low calibration. Are you sure the high concentration is stable?");
    return(3); // Error 3 = Too little time since "low" calibration
  } else if((time_since_last_cal > 600)  && (currentGas != "temperature")){ // >10min. To do a simple offset calibration, wait 10min then click "diff span"
    // A simple offset calibration, i.e. there won't be a linear equation
    Serial.println(">10min since low calibration. Apply simple offset, not linear equation");
    diff_low_cal_time_s = NO_CAL_TIME;
    if(dataType == "sen"){
      // Set to 0 and to the difference so that the "high" calibration value gets ignored
      // This results in a slope of 1 and an intercept that always yields just the plain difference
      update_var(var_sen0, float(NAN));
      update_var(var_sen1, float(NAN));
    }
    return(4); // Error 4 = Too much time since "low" calibration, only offset applied
  } else {
    // Either time_since_last_cal is 0 (i.e., the "ref" or low calibration),
    // Or it is between 10-600s (i.e., the "sen" or high calibration)
    Serial.println("All good, doing calibration");
    update_var(var_sen0, sen0_measurement);
    update_var(var_sen1, sen1_measurement);
    // When doing the "high" calibration, update the zero & span values ONLY if there are already values there
    if((dataType == "sen") && (!std::isnan(read_var("sen_" + currentGas + "_1_span")))){
      update_cal_from_diff(currentGas);
      Serial.println("    UPDATE cal from differential");
    }
  }

  return(0); // No error
}

// Update calibration coefficients of sensor 2 based on diff calibration
int Cal::update_cal_from_diff(String currentGas){
  // Get calibration variables
  float var_ref_zero = read_var("ref_" + currentGas + "_0_zero");
  float var_ref_span = read_var("ref_" + currentGas + "_0_span");
  
  // Get differential readings
  String var_sen0_low  = "ref_" + currentGas + "_0_diff";
  String var_sen0_high = "sen_" + currentGas + "_0_diff";
  String var_sen1_low  = "ref_" + currentGas + "_1_diff";
  String var_sen1_high = "sen_" + currentGas + "_1_diff";
  float var_diff_sen0_low  = read_var(var_sen0_low);
  float var_diff_sen0_high = read_var(var_sen0_high);
  float var_diff_sen1_low  = read_var(var_sen1_low);
  float var_diff_sen1_high = read_var(var_sen1_high);

  // Slope & intercept of diff calibration
  float slope_diff     = (var_diff_sen0_high - var_diff_sen0_low) / (var_diff_sen1_high - var_diff_sen1_low);
  float intercept_diff = var_diff_sen0_low - slope_diff * var_diff_sen1_low;

  // Update values
  float var_sen_1_zero =  -intercept_diff/slope_diff;
  float var_sen_1_span = (var_ref_span - intercept_diff)/slope_diff;

  // Store again
  update_var("sen_" + currentGas + "_1_zero", var_sen_1_zero);
  update_var("sen_" + currentGas + "_1_span", var_sen_1_span);

  return(0);
}
