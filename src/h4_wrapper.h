#pragma once

#include <stdint.h> // for int types
#include <stdbool.h>

// simple C-style wrappers (easy to call from any C++ file)

// Permanent storage of data/variables
bool    h4_gvExists(std::string name);
void    h4_gvSetInt(std::string name, int value, bool save=false);
void    h4_gvSetString(std::string name, std::string value, bool save=false);
int     h4_gvGetInt(std::string name);
std::string h4_gvGetString(std::string name);
void    h4_gvErase(std::string name);
//void    h4_gvUpdInt(const char *name, const char *value);

// Batched persistence. H4 rewrites the whole persistent-globals file on every
// change to a variable marked "save", so writing many variables one at a time
// costs one full flash write each. h4_gvSetSave() turns that per-change write
// off (and back on), and h4_gvPersist() then writes the file exactly once.
void    h4_gvSetSave(std::string name, bool save);
void    h4_gvPersist(void);
