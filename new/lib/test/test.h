/*
 * This is for testing code. Normally shouldn't run
 */
#pragma once

#ifndef Testh_h
#define Testh_h

#include "Arduino.h"
#include <Wire.h>

class Test {
  public:
    // main Class
    Test(void);
  
    // Functions
    //----------
    bool init(void);
    void scanPorts();
    void check_if_exist_I2C();
 
  private:
    //SoftWire sw;
    static const uint8_t NUM_PINS = 8;
    //uint8_t portArray[NUM_PINS]; //portArray[] = {16, 5, 4, 0, 2, 14, 12, 13};
    uint8_t portArray[5] = {16, 5, 4, 0, 2};
    String portMap[5] = {"D0", "D1", "D2", "D3", "D4"}; //for Wemos
    //String portMap[NUM_PINS]; //portMap[] = {"GPIO16", "GPIO5", "GPIO4", "GPIO0", "GPIO2", "GPIO14", "GPIO12", "GPIO13"};

};

#endif
