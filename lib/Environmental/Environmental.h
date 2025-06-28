/*
  Calculate parameters from sensor readings
*/
#pragma once

#ifndef Environmentalh_h
#define Environmentalh_h

#include <math.h>

class Env {
  public:
    // main Class
    Env(void);
  
    // Functions
    float saturation_vapour_pressure(float T_C);
    float current_vapour_pressure(float T_C, float RH);
    float air_water_mole_frac(float T_C, float RH, float P_Pa);
    float vapour_pressure_deficit(float T_C, float RH, float P_Pa);
    float air_density_dry(float T_C, float RH, float P_Pa);
    float air_density_moist(float T_C, float RH, float P_Pa);
    float heat_capacity_dry(float T_C);
    float heat_capacity_moist(float T_C, float h2o_mmol_mol, float P_Pa);

  private:
    const float R_dry_air = 287.058;  // Specific gas const dry air [J kg-1 K-1]
    const float R         = 8.314463; // Ideal gas constant [J K-1 mol-1]
    const float R_v       = 461.5;    // Specific gas constant of water vapor [J kg-1 K-1]
    const float M_C       = 12.01070; // Molar mass of C [g mol-1]
    const float M_d       = 0.02897;  // Molar mass of dry air [kg mol-1]
    const float M_h2o     = 0.01802;  // Molar mass of water vapour [kg mol-1]
};

#endif
