#pragma once

#include <stdint.h> // for int types
#include <stdbool.h>

// simple C-style wrappers (easy to call from any C++ file)
bool    h4_gvExists(const char *name);
void    h4_gvSetInt(const char *name, int value, bool save=false);
void    h4_gvSetString(const char *name, const char *value, bool save=false);
int     h4_gvGetInt(const char *name);
//void    h4_gvUpdInt(const char *name, const char *value);