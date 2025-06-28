/*
  Calculate parameters using sensor readings
*/

#include "Environmental.h"
//#include <math.h>

Env::Env(){
  // Nothing to be done here
}

// Saturation vapor pressure of water at the given temperature
// See sources:
// - Eddypro manual: https://www.licor.com/env/support/EddyPro/topics/calculate-micromet-variables.html
// - Campbell & Norman (1998)
float Env::saturation_vapour_pressure(float T_C){
  // T_C: temperature in °C
  // e_s: saturation vapour pressure in Pa
  float T_K = T_C + 273.15; // Convert to Kelvin

  float e_s = pow(T_K, -8.2) * std::exp(77.345 + pow(0.0057, T_K) - 7235 * pow(T_K, -1));
  return(e_s);
}

float Env::current_vapour_pressure(float T_C, float RH){
  // Calculate saturation vapour pressure
  float e_s = saturation_vapour_pressure(T_C);

  float e = RH/100*e_s;
  return(e);
}

// Water vapour mole fraction [mmol mol-1]
float Env::air_water_mole_frac(float T_C, float RH, float P_Pa){
  float e = current_vapour_pressure(T_C, RH);

  double x_v = (e / P_Pa)*1000; // Factor 1000 for mmol mol-1
  return(x_v);
}

// See Eddypro manual: https://www.licor.com/env/support/EddyPro/topics/calculate-micromet-variables.html
float Env::vapour_pressure_deficit(float T_C, float RH, float P_Pa){
  float e_s = air_water_mole_frac(T_C, RH, P_Pa);
  float e =   current_vapour_pressure(T_C, RH);

  float vpd = e_s - e; // VPD (Pa)
  return(vpd);
}

float Env::air_density_dry(float T_C, float RH, float P_Pa){
  float T_K = T_C + 273.15; // Convert to Kelvin
  float e = current_vapour_pressure(T_C, RH);
  float P_d = P_Pa - e; // Dry air partial pressure (P_d, P_a)

  float rho_dry_air = P_d / (R_dry_air * T_K); // Density of dry air (use for approximation)
  return(rho_dry_air);
}

float Env::air_density_moist(float T_C, float RH, float P_Pa){
  float T_K = T_C + 273.15; // Convert to Kelvin
  float e = current_vapour_pressure(T_C, RH);
  float P_d = P_Pa - e; // Dry air partial pressure (P_d, P_a)
  float rho_d = P_d / (R_dry_air * T_K); // Dry‐air density (kg m-3), using R_dry_air [J kg-1 K-1]
  float rho_v = e / (R_v * T_K); // Water-vapor density (kg m-3)

  // Moist air mass density (ρa, kg m-3) 
  float rho_moist_air = rho_d + rho_v;
  return(rho_moist_air);
}

// Dry‐air heat capacity at constant pressure
float Env::heat_capacity_dry(float T_C){
  float cp_dry_air = 1005.0f + pow(T_C + 23.12, 2) / 3364.0f;
  return(cp_dry_air);
}

// Moist‐air heat capacity at constant pressure
float Env::heat_capacity_moist(float T_C, float RH, float P_Pa){
  float T_K = T_C + 273.15; // Convert to Kelvin
  float e = current_vapour_pressure(T_C, RH);
  float e_s = saturation_vapour_pressure(T_C); // Saturation vapor pressure [Pa]
  
  // Water vapor heat capacity at constant pressure (cp_h2o, J kg-1 K-1)
  float cp_v = 1859.0f
               + 0.13f * RH
               + (0.193f + 0.0056f * RH) * T_C
               + (0.001f  + 0.00005f * RH) * pow(T_C, 2);
  float P_d = P_Pa - e;                  // Dry air partial pressure (P_d, P_a)
  float rho_d = P_d / (R_dry_air * T_K); // Dry‐air density (kg m-3), using R_dry_air [J kg-1 K-1]
  float rho_v = e / (R_v * T_K);         // Water-vapor density (kg m-3)
  float rho_air = rho_d + rho_v;         // Moist air mass density (ρa, kg m-3)
  float Q = rho_v / rho_air;             // Specific humidity (Q, kg kg-1)

  //float cp_moist_air = 1005.0f + pow(T_C + 23.12, 2) / 3364.0f;
  float cp_moist_air = heat_capacity_dry(T_C) * (1-Q) + cp_v * Q;
  return(cp_moist_air);
}