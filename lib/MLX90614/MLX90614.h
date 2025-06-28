/*
 * MLX90614 sensor
 */

#ifndef MLX90614h_h
#define MLX90614h_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>

class MLX {
  public:
    // main Class
    MLX(void);
  
    // Functions
    //----------
    bool  init();
    float airT(void);
    float objT(void);

    // Constants
    //----------
    Adafruit_MLX90614 mlx90614;
};

#endif
