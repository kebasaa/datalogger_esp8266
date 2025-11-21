/*
  Utility functions
*/

#include "Utils.h"
#include <cmath>
#include <algorithm>

Utils::Utils(){
  // Nothing to be done here
}

// Test if a string is in an array of strings, e.g. "co2" in gases
bool Utils::in_list(const std::vector<String>& list, const String& value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

// Make a list into a single text string, comma separated
String Utils::make_text_list(const std::vector<String>& str_list){
  String text_list;

  for (size_t i = 0; i < str_list.size(); i++) {
    text_list += str_list[i];
    if (i < str_list.size() - 1) text_list += ", ";
  }
  return(text_list);
}

bool Utils::is_near_zero(float x, float eps){
    return std::fabs(x) <= eps;
}