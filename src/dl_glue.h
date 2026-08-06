#pragma once
// ============================================================================
//  Datalogger framework internals: Wrapper definitions, core includes, BIG_SIZE
//  Auto-relocated verbatim from the old monolithic main.cpp. Included ONLY by
//  datalogger.h (one translation unit). Beginners never need to open this.
// ============================================================================

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
std::string h4_gvGetString(std::string name){
  return h4p.gvGetstring(name);
}
// Remove a variable entirely. Note H4's gvErase() writes the whole persistent
// file itself whenever the variable was marked "save", so callers erasing more
// than one should clear that flag first (h4_gvSetSave) and then call
// h4_gvPersist() once, exactly as the calibration session does.
void h4_gvErase(std::string name){
  h4p.gvErase(name);
}
// Turn the automatic "write the file on every change" behaviour on or off for a
// single variable. Guarded by gvExists() because h4p[name] would otherwise
// create an empty variable just by looking it up.
void h4_gvSetSave(std::string name, bool save){
  if(h4p.gvExists(name)) h4p[name]._save = save;
}
// Write all persistent variables to flash, once.
void h4_gvPersist(void){
  H4P_SerialCmd::_persist();
}

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
