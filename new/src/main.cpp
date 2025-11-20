#include <Arduino.h>
#include "config.h"

#define H4P_VERBOSE 1 // To see what's going on
#include <H4Plugins.h>
H4_USE_PLUGINS(PROJ_BAUD_RATE, H4_Q_CAPACITY, false) // Serial baud rate, Q size, SerialCmd autostop

// This creates wrapper functions that can't be declared anywhere else than in main.cpp
#include "h4_wrapper.h"
// Permanent storage of data/variables
bool h4_gvExists(std::string name){
  return h4p.gvExists(name);
}
void h4_gvSetInt(std::string name, int value, bool save){
  h4p.gvSetInt(name, value, save);
}
void h4_gvSetString(std::string name, std::string value, bool save){
  h4p.gvSetstring(name, value, save);
}
int h4_gvGetInt(std::string name){
  return h4p.gvGetInt(name);
}

#include <math.h>

// Default I2C bus on D2=SDA, D1=SCL
#include <Wire.h>

//H4P_SerialLogger h4sl;
//H4P_PinMachine h4gm; // For buttons

#ifdef ARDUINO_ARCH_ESP8266
#define BIG_SIZE 500
#else
#define BIG_SIZE 13000
#endif

char* dev_name = "Datalogger";
#if H4P_USE_WIFI_AP
String wifitype = "WIFI: AP mode";
H4P_WiFi h4wifi(dev_name);
#else
String wifitype = "WIFI: Client mode";
H4P_WiFi h4wifi(WIFI_SSID, WIFI_PASS, dev_name);
#endif


#if USE_MQTT
#if H4P_USE_WIFI_AP
H4P_AsyncMQTT h4mqtt;
#else
H4P_AsyncMQTT h4mqtt(MQTT_SERVER);
#endif // H4P_USE_WIFI_AP
uint8_t big[BIG_SIZE];
H4_TIMER mqttSender;
H4_TIMER bigSender;
#endif // USE_MQTT

//H4P_Heartbeat h4hb; // Show uptime

#if USE_HTTPREQ
H4P_AsyncHTTP h4ah;
H4_TIMER httpReqTimer;
//void HTTPClient();
#endif
//void publishDevice(const std::string &topic, const std::string &payload);
//void publishDevice(const std::string &topic, long long payload);

int n_measurements = 0;

// Run a test
#if RUN_TEST
# include <test.h>
Test test;
#endif

#if I2C_MULTI
#include <TCA9548_multiplexer.h>
MULTI mp;
int i2c_buses[] = {0,1};
#endif

// MicroSD card reader
#if USE_MICROSD
# include <MicroSD.h>
MicroSD sd;
#endif

// Battery
#if USE_BATTERY
# include <Battery.h>
Battery bat;
#endif

// GPS device
#if USE_GPS
# include <GPS.h>
#if I2C_MULTI
GPS gps(&mp, 2); // Not connected to the multiplexer, this will not affect anything
#else
GPS gps();
#endif
#endif

// T, RH, P sensor
#if USE_BME280
#include <BME280_sen.h>
#if I2C_MULTI
BME bme1(&mp, 0); // Initialise buses 0 & 1
BME bme2(&mp, 1);
BME* bme_sensors[] = { &bme1, &bme2 };
#else
BME bme(&mp, 1); // Initialise bus 1 only
BME* bme_sensors[] = { &bme };
#endif
#endif

// CO2 sensor
#if USE_SCD30
#include <SCD30.h>
#if I2C_MULTI
SCD scd1(&mp, 0); // Initialise buses 0 & 1
SCD scd2(&mp, 1);
SCD* scd_sensors[] = { &scd1, &scd2 };
#else
SCD scd(&mp, 1); // Initialise bus 2 only
SCD* scd_sensors[] = { &scd };
#endif
#endif

// LWR sensor
#if USE_MLX90614
#include <MLX90614.h>
#if I2C_MULTI
MLX mlx1(&mp, 0); // Initialise buses 0 & 1
MLX mlx2(&mp, 1);
MLX* mlx_sensors[] = { &mlx1, &mlx2 };
#else
MLX mlx(&mp, 1); // Initialise bus 1 only
MLX* mlx_sensors[] = { &mlx };
#endif
#endif

// ADS1115 analog-to-digital converter, not used in this project
#if USE_ADS1115
# include <ADS1115.h>
#if I2C_MULTI
ADS ads1(&mp, 0); // Initialise buses 0 & 1
ADS ads2(&mp, 1);
ADS* ads_sensors[] = { &ads1, &ads2 };
#else
ADS ads(&mp, 1); // Initialise bus 1 only
ADS* ads_sensors[] = { &ads };
#endif
#endif

// SEN0465 O2 sensor
#if USE_SEN0465
#include <SEN0465.h>
#if I2C_MULTI
SEN0465 sen1(&mp, 0); // Initialise buses 0 & 1
SEN0465 sen2(&mp, 1);
SEN0465* sen_sensors[] = { &sen1, &sen2 };
#else
SEN0465 sen(&mp, 1); // Initialise bus 1 only
SEN0465* sen_sensors[] = { &sen };
#endif
#endif

// Calculate environmental parameters
#if USE_ENV
#include <Environmental.h>
Env env;
#endif

#if USE_CAL
#include <Calibration.h>
Cal cal;
std::vector<String> gases;// = {"co2", "o2", "h2o"};
// numSensors depends on whether "Multi" is active
#if I2C_MULTI
  int numSensors = 2;
  #else
  int numSensors = 1;
#endif
int currentSensor = -9999;
String currentGas = "all";
String currentDiffGas = "all";
bool calibration_running = false;
#endif

H4_TIMER  TIM0;

// Timezone setup
//H4P_Timekeeper h4tk(NTP1, NTP2, LocalTZO);  // Time support
//boolean ClockValid = false;  // Flag indicating whether to display the clock or not.

// Wifi
boolean WiFiValid = false;  // Flag indicating a valid WiFi Connection is active.

bool isGPSDateValid() {
    #if USE_GPS
        String date = gps.get_date();
        int year = date.substring(0, 4).toInt();
        return (year >= 2025) && (year <= 2050);
    #else
        return false; // Or true, depending on if you want to proceed without GPS
    #endif
}

void onWiFiConnect() {
	Serial.printf("Wifi connected");
	Serial.printf("IP4 Address: %s\n",WiFi.localIP().toString().c_str());
	WiFiValid = true;
#if USE_HTTPREQ
	h4.queueFunction(HTTPClient);
	httpReqTimer = h4.every(60000, HTTPClient);
#endif
}
void onWiFiDisconnect() {
	Serial.printf("WiFi Disconnected\n");
#if USE_HTTPREQ
	h4.cancel(httpReqTimer);
#endif
}
#if USE_MQTT
void onMQTTConnect() {
	mqttSender = h4.every(2000, []()
					  {
						  publishDevice("heap", _HAL_freeHeap());
						  publishDevice("uptime",h4p.gvGetstring(upTimeTag()));
						  publishDevice("maxbloc",_HAL_maxHeapBlock());
					  });
	
	bigSender = h4.every(3000,[]{
		Serial.printf("SENDING BIG\n");
		h4mqtt.publish("big", &big[0], BIG_SIZE, 1);
	});
}
void onMQTTDisconnect() {
	Serial.printf("onMQTTDisconnect()\n");
	h4.cancel(mqttSender);
	h4.cancel(bigSender);
}
#endif

// Measure for 5s and average, when button is pressed
float measure_gas(String gas, int sensor){
  float gas_measured = float(NAN);
  if((gas == "all") || (gas == "") || (sensor == -9999)){
    return(float(NAN));
  }
  // Note that the sensor list is 0-indexed, i.e. the number needs to be -1
  //sensor -= 1;

  if(gas == "co2"){
  #if USE_SCD30
    gas_measured = scd_sensors[sensor]->airCO2();
  #endif
  }
  if (gas == "o2"){
  #if USE_SEN0465
    gas_measured = sen_sensors[sensor]->airO2();
  #endif
  }
  if (gas == "h2o"){
  #if USE_BME280
    // This sensor measures RH, but that's not useful for calibration, so the mole fraction has to be calculated
    gas_measured = env.air_water_mole_frac(bme_sensors[sensor]->airT(),
                                           bme_sensors[sensor]->airRH(),
                                           bme_sensors[sensor]->airP());
  #endif
  }
  if (gas == "temperature"){
  #if USE_BME280
    gas_measured = bme_sensors[sensor]->airT();
  #endif
  }
  
  return(gas_measured);
}

void onViewersConnect() {
	Serial.printf("onViewersConnect\n");
  h4wifi.uiAddText("Timestamp (UTC)", "");
  h4wifi.uiAddText("Location", "");
  h4wifi.uiAddText("Battery", "");
  h4wifi.uiAddText("---","");

  h4wifi.uiAddText("Zero & Span","Calibration");
  h4wifi.uiAddDropdown("Gas Sensor",{
    {"All","all"},
    {"H₂O","h2o"},
    {"CO₂","co2"},
    {"O₂", "o2"},
    {"Temperature", "temperature"},
  });
  h4wifi.uiAddDropdown("Sensor Number",{
    {"-9999","-9999"},
    {"0","0"},
    {"1","1"}
  });
  h4wifi.uiAddInput("Zero gas");
  h4wifi.uiAddImgButton("setzero");
  h4wifi.uiAddInput("Span gas");
  h4wifi.uiAddImgButton("setspan");
  Serial.print("Current Gas: "); Serial.println(currentGas);
  h4wifi.uiAddText("Gas",currentGas.c_str());
  h4wifi.uiAddText("Sensor",currentSensor);
  h4wifi.uiAddText("Zero value",CSTR(String(cal.read_calibration_var("sen", "zero", currentGas, currentSensor))));
  h4wifi.uiAddText("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
  h4wifi.uiAddText("Span value",CSTR(String(cal.read_calibration_var("sen", "span", currentGas, currentSensor))));
  h4wifi.uiAddText("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
  h4wifi.uiAddText("Raw value", "");
  h4wifi.uiAddText("Calibrated value", "");

  h4wifi.uiAddText("----","");
  h4wifi.uiAddText("Alternative:","Δ Calibration");
  h4wifi.uiAddDropdown("Δ Type",{
    {"All","all"},
    {"H₂O","h2o"},
    {"CO₂","co2"},
    {"O₂", "o2"},
    {"Temperature", "temperature"},
  });
  h4wifi.uiAddImgButton("setdiffLow");
  h4wifi.uiAddImgButton("setdiffHigh");
  h4wifi.uiAddText("Δ Gas",currentDiffGas.c_str());
  h4wifi.uiAddText("Sen. 0 value","");
  h4wifi.uiAddText("Sen. 1 value","");
  h4wifi.uiAddText("Sen. 1 (Cal.)","");
  
  h4wifi.uiAddText("---","");
  h4wifi.uiAddImgButton("reset");
  
  // Every second, update the UI (only while someone is connected to the webUI)
  TIM0=h4.every(1000,[](){
    #if USE_GPS
    if (!isGPSDateValid()) {
      h4wifi.uiSetValue("Timestamp (UTC)","NO GPS SIGNAL");
      h4wifi.uiSetValue("Location","NO GPS SIGNAL");
    } else {
      h4wifi.uiSetValue("Timestamp (UTC)",CSTR(gps.get_timestamp()));
      h4wifi.uiSetValue("Location",CSTR(gps.get_short_location()));
    }
    #else
    h4wifi.uiSetValue("Timestamp (UTC)","No GPS module");
    #endif
    float measured_value = measure_gas(currentGas, currentSensor);
    Cal::CalibrationResult cal_result = cal.calibrate_linear(currentGas, currentSensor, measured_value);
    //float calibrated_value = cal.calibrate_linear(currentGas, currentSensor, measured_value);
    h4wifi.uiSetValue("Battery",CSTR(String(bat.battery_pc())));
    if(!calibration_running){
      h4wifi.uiSetValue("Raw value",CSTR(String(measured_value)));
      h4wifi.uiSetValue("Calibrated value",CSTR(String(cal_result.calibratedValue)));
      h4wifi.uiSetValue("Sen. 0 value",CSTR(String(measure_gas(currentDiffGas, 0))));
      h4wifi.uiSetValue("Sen. 1 value",CSTR(String(measure_gas(currentDiffGas, 1))));
    }
    measured_value = measure_gas(currentDiffGas, 1);
    //calibrated_value = cal.calibrate_linear(currentDiffGas, 1, measured_value);
    cal_result = cal.calibrate_linear(currentDiffGas, 1, measured_value);
    h4wifi.uiSetValue("Sen. 1 (Cal.)",CSTR(String(cal_result.calibratedValue)));
  });       
}

void onViewersDisconnect() {
	Serial.printf("onViewersDisconnect\n");
  h4.cancel({TIM0}); // Stop updating the UI
}

// TODO: Can I use this?
void h4_set_ui_text(std::string  field, std::string  value){
  h4wifi.uiSetValue(field, value);
}

void save_calibration_coefficients(void){
  // Save calibration data to file
  String cal_data_str = "";
  String cal_header = "";
  #if USE_GPS
    if (!isGPSDateValid()) {
      h4wifi.uiMessage("Error: No GPS fix");
      Serial.print(F("Invalid GPS date"));
      return;
    }
    String calibration_fn = "calibration_" + gps.get_date() + ".csv";
    cal_header   = "timestamp_utc,";
    cal_data_str = gps.get_timestamp() + ",";
  #else
    String data_fn = "calibration.csv";
  #endif
  cal_header   += cal.get_all_cal_header(gases, numSensors);
  cal_data_str += cal.get_all_cal_data(gases, numSensors);
  #if USE_MICROSD
    // Write data to disk
    sd.write_data(calibration_fn.c_str(),
                  cal_header.c_str(),
                  cal_data_str.c_str(),
                  86400); // logging max 1x/day, used to calculate space on the SD
                          // (should always be enough as there is only little data from calibrations)
  #endif
  return;
}

void absolute_calibration(const std::vector<String> gas_list, String abs_cal_type, int sensor){
  // Don't do a span calibration for all gases at once
  if((abs_cal_type == "span") && (currentGas == "all")){ return; }
  // Don't run another calibration if it's already running
  if (calibration_running) {
    Serial.println("Calibration already running - ignoring request");
    return;
  }
  calibration_running = true;

  // If all gases are zero'd, then set to them to exactly 0.0
  float ref_value = 0.0;
  if((abs_cal_type == "zero") && (currentGas == "all")){
    ref_value = 0.0;
  } else { // Otherwise, get zero reference gas entered by user
    String input_field = abs_cal_type;
    input_field[0] = toupper(input_field[0]);
    input_field +=  + " gas"; // Creates "Zero gas" or "Span gas" (note the upper-case)
    ref_value = std::stof(std::string(h4p.gvGetstring(input_field.c_str()).c_str()));
    Serial.print("Ref.: "); Serial.println(ref_value);
  }

  // Decide how many sensors we are reading:
  // - When multiple gases are passed, 'sensor' parameter holds # sensors to probe
  // - otherwise treat sensor as a single sensor id (so num_sensors = 1)
  const size_t num_gases = gas_list.size();
  int num_sensors = (num_gases > 1) ? sensor+1 : 1;
  if(num_sensors <= 0) num_sensors = 1; // Safeguard

  // shared accumulators (captured into lambdas by value), multiplying gases by number of sensors
  auto cumulative_data = std::make_shared<std::vector<float>>(num_gases * num_sensors, 0.0f);
  const int n_measurements = 10;
  
  h4.nTimesRandom(
        // Measure "n_measurements" times (every ~1.0-1.1s) & average
        n_measurements, 1000, 1100,
        // Run at every iteration: measure every gas and accumulate
        [gas_list, cumulative_data, num_sensors, sensor,abs_cal_type]() {
            for (size_t gas = 0; gas < gas_list.size(); gas++) { // Cycle through all gases
              if (num_sensors > 1) { // sensor is the number of sensors to be measured
                for (int s = 0; s < num_sensors; s++) { // measure all sensors for this gas
                  size_t idx = gas * num_sensors + s; // index into flattened vector
                  (*cumulative_data)[idx] += measure_gas(gas_list[gas], s);
                }
              } else { // If only 1 gas was given, num_sensors is 1, the for loop is executed once
                // "sensor" is the sensor ID and is the one measured here
                size_t idx = gas * num_sensors + 0; // This always creates index 0
                (*cumulative_data)[idx] += measure_gas(gas_list[gas], sensor);
              }
            }
            if(abs_cal_type == "zero"){
              h4wifi.uiSetValue("Zero value","In progress");
              h4wifi.uiSetValue("Zero reference","In progress");
            } else {
              h4wifi.uiSetValue("Span value","In progress");
              h4wifi.uiSetValue("Span reference","In progress");
            }
        },
        // Run when timer finishes: compute averages & store/set differential for each gas
        [gas_list, cumulative_data, sensor, num_sensors, n_measurements, abs_cal_type, ref_value]() {
            const unsigned long ts = gps.seconds_since_midnight(); // single timestamp
            for (size_t gas = 0; gas < gas_list.size(); gas++) {
              for (int s = 0; s < num_sensors; s++) {
                size_t idx = gas * num_sensors + s; // index into flattened vector
                float sensor_measured = (*cumulative_data)[idx] / static_cast<float>(n_measurements);
                // set differential for each gas
                cal.set_calibration_coeff(abs_cal_type, gas_list[gas], s, ref_value, sensor_measured, ts);
                  
                Serial.print("Gas: "); Serial.println(gas_list[gas]);
                Serial.print("  Sensor: "); Serial.println(s);
                Serial.print("    Value: "); Serial.println(sensor_measured);
                Serial.print("    Reference: "); Serial.println(ref_value);
                if(abs_cal_type == "zero"){
                  h4wifi.uiSetValue("Zero value", CSTR(String(sensor_measured, 2)));
                  h4wifi.uiSetValue("Zero reference", CSTR(String(ref_value, 2)));
                } else {
                  h4wifi.uiSetValue("Span value", CSTR(String(sensor_measured, 2)));
                  h4wifi.uiSetValue("Span reference", CSTR(String(ref_value, 2)));
                }
            }
            // reset accumulators (may be optional since shared_ptr is freed after lambda ends)
            std::fill(cumulative_data->begin(), cumulative_data->end(), 0.0f);

            if(abs_cal_type == "span"){
              save_calibration_coefficients();
            }
            calibration_running = false;
          }
        },
        0, false
    );
}

void onsetzeroButton(){
  h4wifi.uiMessage("Zero Calibration");
  Serial.println("Zero Calibration");

  if(currentSensor == -9999){ return; }
  
  if(currentGas == "all"){
    absolute_calibration(gases, "zero", numSensors);
  } else {
    absolute_calibration({currentGas}, "zero", currentSensor);
  }
}

void onsetspanButton(){
  h4wifi.uiMessage("Span Calibration");
  Serial.println("Span Calibration");

  if(currentSensor == -9999){ return; }

  if(currentGas == "all"){
    h4wifi.uiMessage("ERROR: Please calibrate span individually");
    Serial.println("ERROR: Please calibrate span individually");
    // Note: This will return, doing nothing (because span can't be set for different gases at once)
    absolute_calibration(gases, "span", numSensors);
  } else {
    absolute_calibration({currentGas}, "span", currentSensor);
  }

}

// Differential calibration
void differential_calibration(const std::vector<String> gas_list, String dif_cal_type){
  //h4wifi.uiMessage("Differential calibration running...");
  //Serial.println("Differential calibration running...");
  if (calibration_running) {
    Serial.println("Calibration already running - ignoring request");
    return;
  }
  calibration_running = true;

  // shared accumulators (captured into lambdas by value)
  auto cumulative_s1 = std::make_shared<std::vector<float>>(gas_list.size(), 0.0f);
  auto cumulative_s2 = std::make_shared<std::vector<float>>(gas_list.size(), 0.0f);
  const int n_measurements = 10;

    h4.nTimesRandom(
        // Measure "n_measurements" times (every ~1.0-1.1s) & average
        n_measurements, 1000, 1100,
        // Run at every iteration: measure every gas and accumulate
        [gas_list, cumulative_s1, cumulative_s2]() {
            for (size_t i = 0; i < gas_list.size(); ++i) {
                (*cumulative_s1)[i] += measure_gas(gas_list[i], 0);
                (*cumulative_s2)[i] += measure_gas(gas_list[i], 1);
            }
            h4wifi.uiSetValue("Sen. 0 value", "In progress");
            h4wifi.uiSetValue("Sen. 1 value", "In progress");
        },
        // Run when timer finishes: compute averages & store/set differential for each gas
        [gas_list, cumulative_s1, cumulative_s2, n_measurements, dif_cal_type]() {
            const unsigned long ts = gps.seconds_since_midnight(); // single timestamp
            for (size_t i = 0; i < gas_list.size(); ++i) {
                float avg0 = (*cumulative_s1)[i] / n_measurements;
                float avg1 = (*cumulative_s2)[i] / n_measurements;
                // set differential for each gas
                cal.set_differential_coeff(dif_cal_type, gas_list[i], avg0, avg1, ts);

                Serial.print("Gas: "); Serial.println(gas_list[i]);
                Serial.print("Sen. 0 avg: "); Serial.println(avg0);
                Serial.print("Sen. 1 avg: "); Serial.println(avg1);
            }

            // reset accumulators (optional since shared_ptr is freed after lambda ends)
            for (size_t i = 0; i < gas_list.size(); ++i) { (*cumulative_s1)[i] = 0.0f; (*cumulative_s2)[i] = 0.0f; }

            h4wifi.uiSetValue("Sen. 0 value", "");
            h4wifi.uiSetValue("Sen. 1 value", "");
            if(dif_cal_type == "sen"){
              save_calibration_coefficients();
            }
            calibration_running = false;
        },
        0, false
    );
}

void onsetdiffLowButton(){
  h4wifi.uiMessage("Differential calibration (low value) running...");
  Serial.println("Differential calibration (low)");

  if(currentDiffGas == "all"){
    differential_calibration(gases, "ref"); // "sen" is for the high calibration, "ref" for low
  } else {
    differential_calibration({currentDiffGas}, "ref");  // "sen" is for the high calibration, "ref" for low
  }
}

void onsetdiffHighButton(){
  h4wifi.uiMessage("Differential calibration (high value) running...");
  Serial.println("Differential calibration (high)");

  if(currentDiffGas == "all"){
    differential_calibration(gases, "sen"); // "sen" is for the high calibration, "ref" for low
  } else {
    differential_calibration({currentDiffGas}, "sen");  // "sen" is for the high calibration, "ref" for low
  }
}

void onresetButton(){
  h4wifi.uiMessage("RESET");
  Serial.println("RESET");
  cal.reset_all_calibrations(gases, numSensors);
}

H4P_EventListener allexceptmsg(H4PE_VIEWERS | H4PE_GVCHANGE,[](const std::string& svc,H4PE_TYPE t,const std::string& msg){
  switch(t){
    case H4PE_VIEWERS:
      H4P_SERVICE_ADAPTER(Viewers);
      break;
    case H4PE_GVCHANGE:
      // Connect buttons to relevant functions
      H4P_TACT_BUTTON_CONNECTOR(setspan);
      H4P_TACT_BUTTON_CONNECTOR(setzero);
      H4P_TACT_BUTTON_CONNECTOR(setdiffLow);
      H4P_TACT_BUTTON_CONNECTOR(setdiffHigh);
      H4P_TACT_BUTTON_CONNECTOR(reset);
      // Process button presses
      if(svc=="Gas Sensor"){ // Test Gas Sensor selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Gas",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which gas was chosen
        currentGas = msg.c_str();
        Serial.print("currentGas, currentSensor (GAS): "); Serial.print(currentGas); Serial.print(" "); Serial.println(currentSensor);
        h4wifi.uiSetValue("Zero value",CSTR(String(cal.read_calibration_var("sen", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span value",CSTR(String(cal.read_calibration_var("sen", "span", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
        h4p.gvSetstring("Zero gas",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor)))); // Change input field value when dropdown selection changes
        h4p.gvSetstring("Span gas",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor)))); // Change input field value when dropdown selection changes
        break;
      }
      if(svc=="Sensor Number"){ // Test sensor number selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Sensor",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which sensor was chosen & convert to int
        String msg_str = msg.c_str();
        currentSensor = msg_str.toInt();
        Serial.print("currentGas, currentSensor (#): "); Serial.print(currentGas); Serial.print(" "); Serial.println(currentSensor);
        h4wifi.uiSetValue("Zero value",CSTR(String(cal.read_calibration_var("sen", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span value",CSTR(String(cal.read_calibration_var("sen", "span", currentGas, currentSensor))));
        h4wifi.uiSetValue("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
        // Change input field default value when dropdown selection changes
        h4p.gvSetstring("Zero gas",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
        h4p.gvSetstring("Span gas",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
        break;
      }

      if(svc=="Δ Type"){ // Test differential gas selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Δ Gas",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which gas was chosen
        currentDiffGas = msg.c_str();
        break;
      }
      break;
  }
});

/*
void onRTC(){
  Serial.println("Clock valid!");
  ClockValid = true;
  Serial.printf("\nReceived NTP time: %s (UTC)\n\n", CSTR(h4tk.strfDateTime( "%a %Y-%m-%d %H:%M:%S", h4tk.clockEPOCHLocal()) ));
}
*/

/*
void h4pGlobalEventHandler(const std::string& svc,H4PE_TYPE t,const std::string& msg)
{
	switch (t)
	{
		H4P_DEFAULT_SYSTEM_HANDLER;
	case H4PE_SERVICE:
		H4P_SERVICE_ADAPTER(WiFi);
#if USE_MQTT
		H4P_SERVICE_ADAPTER(MQTT);
#endif
		break;
	case H4PE_VIEWERS:
		H4P_SERVICE_ADAPTER(Viewers);
		break;
	default:
		break;
	}
}
*/

// Collect measurements
void processData(void){
  // Prepare output string
  String data_str = "";

  // Measurement count
  int n_measurements = 0;
  String header = "";
  String cal_header = "";

  Cal::CalibrationResult cal_result;

#if USE_GPS
  // Add timestamps and location
  gps.update_values();
  data_str = data_str + gps.get_timestamp() + ",";
  data_str = data_str + gps.get_location() + ",";
  header += "timestamp_utc,lat,lon,alt,nb_sat,HDOP,";
#endif

#if USE_BATTERY
  header += "bat.mV,bat.perc,";
  data_str += String(bat.battery_mV()) + ",";    // Battery charge     [mV]
  data_str += String(bat.battery_pc()) + ",";    // Battery charge     [%]
  n_measurements += 2;
#endif

  // Go through i2c buses
#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
  for (size_t i = 0; i < n; ++i) {
#endif
  
  #if USE_ADS1115
    header += "Sin.W_m2,Sout.W_m2,";
    data_str += String(ads_sensors[i]->read_val(1, 16, 25.83), 2) + ","; // ADC of Apogee SP-510, A1, Gain 16: ±0.256V, convert by 25.83 W m-2 / mV
    data_str += String(ads_sensors[i]->read_val(2, 16, 30.98), 2) + ","; // ADC of Apogee SP-610, A2, Gain 16: ±0.256V, convert by 30.98 W m-2 / mV
    n_measurements += 2;
  #endif
  #if USE_BME280
    // Uncalibrated values
    header += "bme_RH.perc,bme_P.Pa,"; //alt_agl.m,
    data_str += String(bme_sensors[i]->airRH(), 2) + ",";         // RH                 [%]
    data_str += String(bme_sensors[i]->airP(), 2) + ",";          // Pressure           [Pa]
    //data_str += String(bme_sensors[i]->altitude_agl (), 2) + ","; // Altitude agl       [m]
    n_measurements += 2;
    #if USE_CAL
      header += "bme_T.C,bme_T.flag,bme_H2O.mmol_mol,bme_H2O.mmol_mol.flag,";
      // Calibrated temperature
      cal_result = cal.calibrate_linear("temperature", i, bme_sensors[i]->airT());
      data_str += String(cal_result.calibratedValue, 2) + ",";      // Temperature        [C]
      data_str += String(cal_result.flag) + ",";                    // Data quality flag after calibration
      // Calibrated H2O mole fraction
      float h2o_mole_frac = env.air_water_mole_frac(bme_sensors[i]->airT(),
                                           bme_sensors[i]->airRH(),
                                           bme_sensors[i]->airP()); // H2O mole fraction [mmol/mol]
      cal_result = cal.calibrate_linear("h2o", i, h2o_mole_frac);
      data_str += String(cal_result.calibratedValue, 2) + ",";     // H2O mole fraction [mmol/mol]
      data_str += String(cal_result.flag) + ",";                   // Data quality flag after calibration
      n_measurements += 4;
    #else
      header += "bme_T.C,bme_H2O.mmol_mol,";
      data_str += String(bme_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
      float h2o_mole_frac = env.air_water_mole_frac(bme_sensors[i]->airT(),
                                           bme_sensors[i]->airRH(),
                                           bme_sensors[i]->airP()); // H2O mole fraction [mmol/mol]
      data_str += String(h2o_mole_frac, 2) + ",";     // H2O mole fraction [mmol/mol]
      n_measurements += 2;
    #endif
  #endif
  #if USE_SCD30
    #if USE_BME280
    // Always update air pressure in CO2 sensor before using it
    scd_sensors[i]->set_air_pressure(bme_sensors[i]->airP());
    #endif
    header += "scd_T.C,scd_RH.perc,";
    data_str += String(scd_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
    data_str += String(scd_sensors[i]->airRH(), 2) + ",";         // RH                 [%]
    n_measurements += 2;
    #if USE_CAL
      header += "scd_CO2.ppm,scd_CO2.flag,";
      cal_result = cal.calibrate_linear("co2", i, scd_sensors[i]->airCO2());
      data_str += String(cal_result.calibratedValue, 2) + ",";      // CO2 concentration  [ppm] // Lab calibration measurements: 1.0431*CO2-31.727
      data_str += String(cal_result.flag) + ",";                    // Data quality flag after calibration
      n_measurements += 2;
    #else
      header += "scd_CO2.ppm,";
      data_str += String(scd_sensors[i]->airCO2(), 2) + ",";      // CO2 concentration  [ppm] // Lab calibration measurements: 1.0431*CO2-31.727
      n_measurements += 1;
    #endif
  #endif
  #if USE_SEN0465
    header += "sen_T.C,";
    data_str += String(sen_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
    n_measurements += 1;
    #if USE_CAL
      header += "sen_O2.mmol_mol,sen_O2.flag,";
      cal_result = cal.calibrate_linear("o2", i, sen_sensors[i]->airO2());
      data_str += String(cal_result.calibratedValue, 2) + ",";     // Oxygen             [ppm]
      data_str += String(cal_result.flag) + ",";                   // Data quality flag after calibration
      n_measurements += 2;
    #else
      header += "sen_O2.mmol_mol,";
      data_str += String(sen_sensors[i]->airO2(), 2) + ",";          // Oxygen             [ppm]
      n_measurements += 1;
    #endif
  #endif
  #if USE_MLX90614
    header += "mlx_T.C,mlx_obj_T.C,";
    data_str += String(mlx_sensors[i]->airT(), 2) + ",";          // Air temperature    [C]
    data_str += String(mlx_sensors[i]->objT(), 2) + ",";          // Object temperature [C]
    n_measurements += 2;
  #endif

#if I2C_MULTI
  }
#endif

  // Remove "nan" strings to shorten CSV output
  data_str.replace("nan", "");

  // Save data
  //------------
  // Creates daily file name
  int year = 2020;
#if USE_GPS
  if (!isGPSDateValid()) {
    Serial.println(F("Invalid GPS date"));
    return;
  }
  String data_fn = "data_" + gps.get_date() + ".csv";
#else
  String data_fn = "data.csv";
#endif

#if USE_MICROSD
  // Write data to disk
  sd.write_data(data_fn.c_str(),
                header.c_str(),
                data_str.c_str(),
                MEASUREMENT_INTERVAL); // logging frequency last [s]
#endif
  
  // Debug: Show data on Serial output
  Serial.println(data_str);
}

void h4setup(){
  // Show info on WIFI
  Serial.print(wifitype); Serial.print(F(" connecting to ")); Serial.println(WIFI_SSID);

#if USE_MQTT
	h4p[brokerTag()]=MQTT_SERVER;
	for (int i=0;i<BIG_SIZE;i++) {
		big[i]=i;
	}
#endif

#if H4P_SECURE
#if USE_HTTPREQ && SECURE_HTTPREQ
	auto testRootCA = reinterpret_cast<const uint8_t*>(const_cast<char*>(test_root_ca.c_str()));
	h4ah.secureTLS(testRootCA, test_root_ca.length() + 1);
	Serial.printf("HTTP CERT Validation: %s\n", H4AsyncClient::isCertValid(testRootCA, test_root_ca.length() + 1) ? "SUCCEEDED" : "Failed");
#endif // SECURE_HTTPREQ

#if USE_MQTT && SECURE_MQTT
	auto mqCert = reinterpret_cast<const uint8_t*>(const_cast<char*>(MQTT_CERT.c_str()));
	Serial.printf("MQTT CERT Validation: %s\n", H4AsyncClient::isCertValid(mqCert,MQTT_CERT.length()+1) ? "SUCCEEDED" : "Failed");
	h4mqtt.secureTLS(mqCert, MQTT_CERT.length()+1);
#endif // USE_MQTT

#if SECURE_WEBSERVER
	Serial.printf("WEBSERVER CERT Validation: %s\n", H4AsyncClient::isCertValid((const uint8_t*)WEBSERVER_CERT.c_str(), WEBSERVER_CERT.length() + 1) ? "SUCCEEDED" : "Failed");
	Serial.printf("WEBSERVER KEY Validation: %s\n", H4AsyncClient::isPrivKeyValid((const uint8_t*)WEBSERVER_PRIV_KEY.c_str(), WEBSERVER_PRIV_KEY.length() + 1) ? "SUCCEEDED" : "Failed");
	h4wifi.hookWebserver([](){
		h4wifi.secureTLS((const uint8_t*)WEBSERVER_PRIV_KEY.c_str(), WEBSERVER_PRIV_KEY.length() + 1, 
							NULL, 0,
						(const uint8_t*)WEBSERVER_CERT.c_str(), WEBSERVER_CERT.length() + 1);
		h4wifi.useSecurePort();
	});
#endif // SECURE_WEBSERVER

	h4wifi.authenticate("admin","admin");

#endif // H4P_SECURE

#if USE_MQTT
	h4.every(300, []()
			 {
#if defined(ARDUINO_ARCH_ESP32)
				Serial.printf("H=%u M=%u m=%u S=%u\n", _HAL_freeHeap(MALLOC_CAP_INTERNAL), _HAL_maxHeapBlock(MALLOC_CAP_INTERNAL), _HAL_minHeapBlock(MALLOC_CAP_INTERNAL), uxTaskGetStackHighWaterMark(NULL));
#else
				Serial.printf("H=%u M=%u m=%u\n", _HAL_freeHeap(), _HAL_maxHeapBlock(), _HAL_minHeapBlock());
#endif
				h4p["heap"] = _HAL_freeHeap();
				h4p["pool"] = mbx::pool.size();
				});
#endif

  Serial.println("Starting v"WS_VERSION);

  Serial.println(F(""));
  Serial.println(F("Initialisation:"));

  // Non-i2c devices
#if RUN_TEST
  Serial.print(F("- TEST:                     "));
  Serial.println(test.init() ? F("Success") : F("Failed"));
#endif

  // SPI devices
#if USE_MICROSD
Serial.print(F("- MicroSD:                  "));
  if(! sd.init()){
    Serial.println(F("Failed (SD card missing)"));
  } else {
    Serial.println(F("Success"));
    Serial.print("  Storage capacity:      "); Serial.print(sd.getCapacityMB(), 2); Serial.println(" MB");
    Serial.print("  Free storage capacity: "); Serial.print(sd.getFreeMB(), 2); Serial.println(" MB");
  }
#endif

  // Analogue devices
#if USE_BATTERY
  Serial.print(F("- Battery:                  "));
  Serial.println(bat.init() ? F("Success") : F("Failed (Battery not detected)"));
#endif

  // i2c multiplexer
#if I2C_MULTI
  Serial.print(F("- i2c multiplexer           "));
  if(! mp.init()){
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Success"));
  }
#endif
  Wire.begin();

#if USE_GPS
  Serial.print(F("  - XA1110 GPS:             "));
  Serial.println(gps.init() ? F("Success") : F("Failed"));
#endif

// Creates gas list
#if USE_BME280
  gases.push_back("temperature"); // Add to the gases list
  gases.push_back("h2o"); // Add to the gases list
#endif
#if USE_SCD30
  gases.push_back("co2"); // Add to the gases list
#endif
#if USE_SEN0465
  gases.push_back("o2"); // Add to the gases list
#endif
#if USE_CAL
  cal.init_all_calibrations(gases, numSensors);
#endif

  // Go through i2c buses
#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
  for (size_t i = 0; i < n; ++i) {
    // Enable the i2c bus
    Serial.print(F("- i2c bus "));Serial.print(i2c_buses[i]);Serial.println(F(":"));
#endif

#if USE_BME280
  Serial.print(F("  - BME280 sensor:            "));
  Serial.println(bme_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if USE_SCD30
  Serial.print(F("  - SCD-30 sensor:            "));
  Serial.println(scd_sensors[i]->init() ? F("Success") : F("Failed"));
  scd_sensors[i]->enable_self_calibration(false); // don't calibrate, we want to do that manually
  scd_sensors[i]->set_interval(2);                // minimum every 2s
#endif

#if USE_SEN0465
  Serial.print(F("  - SEN0465 sensor:           "));
  Serial.println(sen_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if USE_ADS1115
  Serial.print(F("  - ADS1115 (Analog-digital): "));
  Serial.println(ads_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if USE_MLX90614
  Serial.print(F("  - MLX90614 sensor:    "));
  Serial.println(mlx_sensors[i]->init() ? F("Success") : F("Failed"));
#endif

#if I2C_MULTI
  }
#endif

  Serial.println(F("Initialisation completed"));
  Serial.println(F(""));
  
  // Print IP address to Serial
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

  // Set up regular measurements
  h4.every(MEASUREMENT_INTERVAL * 1000, processData);
}

/*
void publishDevice(const std::string &topic, const std::string &payload){
	Serial.printf("Publishing %s to %s\n", CSTR(payload), CSTR(topic));
#if USE_MQTT
	h4mqtt.publishDevice(topic, payload, 1);
#endif
}

void publishDevice(const std::string &topic, long long payload){
	publishDevice(topic, stringFromInt(payload, "%lu"));
}
*/
#if USE_HTTPREQ
void HTTPClient() {
#if SECURE_HTTPREQ
	h4ah.GET("https://www.howsmyssl.com/a/check", [](ARMA_HTTP_REPLY reply){
#else
	h4ah.GET("http://jsonplaceholder.typicode.com/todos/1", [](ARMA_HTTP_REPLY reply){
#endif
		auto rCode = reply.httpResponseCode;
		auto response = reply.asJsonstring();
		auto headers = reply.responseHeaders;
		Serial.printf("code %d response %s\n", rCode, response.c_str());

		for (auto &h:headers) {
			Serial.printf("%s : %s\n", h.first.c_str(), h.second.c_str());
		}
		headers.clear();
		publishDevice("response", response);
	});
}
#endif

