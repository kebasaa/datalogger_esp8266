![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# Asynchronous datalogger using ESP8266

WARNING: This is not finalised code. Use with caution!

WARNING: This code may be incomplete and may not function properly, as it is not finalised.

Asynchronous datalogger using ESP8266 for logging scientific sensors to MicroSD

## Prerequisites
- Requires a GPS module for date, time and location

The following sensors/components are implemented currently:
- ADS1115 analogue multiplexer
- TCA9548 i2c multiplexer
- MicroSD card data storage with circular storage buffer
- Environmental physics equations
- XA1110 GPS module
- SCD-30 CO2 sensor
- SEN0465 O2 sensor
- BME280 air temperature, relative humidity & pressure sensor
- MLX90614 thermal radiation sensor
- Analogue battery charge reading (voltage)

## How to Cite

Muller (2025). *datalogger_esp8266: Asynchronous datalogger using ESP8266*

## License

This software is distributed under the GNU GPL version 3. Any modification of the code in this repository may only be released under the same license, and with attribution of authorship of the original code (i.e., citation above).