/*
  Utility functions
*/
#pragma once

#include "Arduino.h"

class Utils {
  public:
    // main Class
    Utils(void);

    // Functions
    bool in_list(const std::vector<String>& list, const String& value);
    String make_text_list(const std::vector<String>& str_list);
    static bool is_near_zero(float x, float eps = 1e-6f);
    
  private:
    // Nothing here
};
