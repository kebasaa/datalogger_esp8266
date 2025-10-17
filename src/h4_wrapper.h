#pragma once

#include <stdint.h> // for int types
#include <stdbool.h>

// simple C-style wrappers (easy to call from any C++ file)

// Permanent storage of data/variables
bool    h4_gvExists(std::string name);
void    h4_gvSetInt(std::string name, int value, bool save=false);
void    h4_gvSetString(std::string name, std::string value, bool save=false);
int     h4_gvGetInt(std::string name);
//void    h4_gvUpdInt(const char *name, const char *value);
