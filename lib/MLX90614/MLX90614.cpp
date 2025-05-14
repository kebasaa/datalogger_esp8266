/*
 * MLX90614 sensor
 */
 
#include "Arduino.h"
#include "MLX90614.h"

#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx90614 = Adafruit_MLX90614();

MLX::MLX(void){
  // MLX
}

bool MLX::init(void){
  if(!mlx90614.begin()){
    return(false);
  } else {
    return(true);
  }
}

float MLX::airT(void){
  float T = 0;
  T = mlx90614.readAmbientTempC();
  if(T > 1000){
    return(float(NAN));
  }
  return(T);
}

float MLX::objT(void){
  float T = 0;
  T = mlx90614.readObjectTempC();
  if(T > 1000){
    return(float(NAN));
  }
  return(T);
}
