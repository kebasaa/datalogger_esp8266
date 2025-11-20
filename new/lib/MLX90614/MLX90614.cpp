/*
 * MLX90614 sensor
 */
 
#include "Arduino.h"
#include "MLX90614.h"

//Adafruit_MLX90614 mlx90614 = Adafruit_MLX90614();

/*MLX::MLX(void){
  // MLX
}*/

MLX::MLX(
#if I2C_MULTI
  MULTI* mux, uint8_t bus
#endif
) {
#if I2C_MULTI
  _mux = mux;
  _mux_bus = bus;
#endif
}

#if I2C_MULTI
void MLX::setMultiplexer(MULTI* mux, uint8_t bus) {
  _mux = mux;
  _mux_bus = bus;
}
#endif

// Simple RAII guard to enable/disable a mux channel while in scope
#if I2C_MULTI
struct BusGuard {
  MULTI* mux;
  uint8_t bus;
  BusGuard(MULTI* m, uint8_t b) : mux(m), bus(b) {
    if(mux) mux->enableBus(bus);
  }
  ~BusGuard() {
    if(mux) mux->disableBus(bus);
  }
};
#endif

bool MLX::init(){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  if(!mlx90614.begin()){
    error_status = 1; // Initialisation failed
    return(false);
  } else {
    return(true);
  }
}

float MLX::airT(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  float T = 0;
  T = mlx90614.readAmbientTempC();
  if(T > 1000){
    error_status = 2; // Value completely out of range
    return(float(NAN));
  }
  return(T);
}

float MLX::objT(void){
#if I2C_MULTI
  BusGuard guard(_mux, _mux_bus);
#endif
  float T = 0;
  T = mlx90614.readObjectTempC();
  if(T > 1000){
    error_status = 2; // Value completely out of range
    return(float(NAN));
  }
  return(T);
}
