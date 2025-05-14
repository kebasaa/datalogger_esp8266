/*
 * MLX90614 sensor
 */

#ifndef MLX90614h_h
#define MLX90614h_h

#include "Arduino.h"

class MLX {
  public:
    // main Class
    MLX(void);
  
    // Functions
    //----------
    bool  init(void);
    float airT(void);
    float objT(void);
};

#endif
