/*
 * MLX90614 sensor
 */
#pragma once

#ifndef MLX90614h_h
#define MLX90614h_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>

#include "config.h"
#if I2C_MULTI
#include "TCA9548_multiplexer.h"
#endif

class MLX {
    // main Class
  #if I2C_MULTI
    MLX(MULTI* mux = nullptr, uint8_t bus = 0);
    void setMultiplexer(MULTI* mux, uint8_t bus);
  #else
    MLX(void);
  #endif
  
    // Functions
    //----------
    bool  init();
    float airT(void);
    float objT(void);

    // Constants
    //----------
    Adafruit_MLX90614 mlx90614;

  private:
    int error_status = 0;
    int i2c_bus_id = 1;

  #if I2C_MULTI
    MULTI* _mux = nullptr;
    uint8_t _mux_bus = 0;
  #endif
};

#endif
