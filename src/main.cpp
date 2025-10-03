#include <Arduino.h>
#include "config.h"

#define H4P_VERBOSE 1 // To see what's going on
#include <H4Plugins.h>
H4_USE_PLUGINS(PROJ_BAUD_RATE, H4_Q_CAPACITY, false) // Serial baud rate, Q size, SerialCmd autostop

// This creates wrapper functions that can't be declared anywhere else than in main.cpp
#include "h4_wrapper.h"
bool h4_gvExists(const char *name) {
    return h4p.gvExists(std::string(name));
}
void h4_gvSetInt(const char *name, int value, bool save) {
    h4p.gvSetInt(std::string(name), value, save);
}
void h4_gvSetString(const char *name, const char *value, bool save) {
    h4p.gvSetstring(std::string(name), std::string(value), save);
}
int h4_gvGetInt(const char *name) {
    return h4p.gvGetInt(std::string(name));
}
/*void h4_gvUpdInt(const char *name, const char *value) {
    h4p[name] = value;
}*/

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
#include <BME280.h>
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
// TODO: numSensors needs to depend on whether "Multi" is active!
#if I2C_MULTI
  int numSensors = 2;
  #else
  int numSensors = 1;
#endif
int currentSensor = 0;
String currentGas = "all";
String currentDiffGas = "all";
#endif

H4_TIMER  TIM0;

// Timezone setup
//H4P_Timekeeper h4tk(NTP1, NTP2, LocalTZO);  // Time support
//boolean ClockValid = false;  // Flag indicating whether to display the clock or not.

// Wifi
boolean WiFiValid = false;  // Flag indicating a valid WiFi Connection is active.

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

// TODO IMPORTANT: Measure for 5s and average, when button is pressed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
float measure_gas(String gas, int sensor){
  float gas_measured = float(NAN);
  if((gas == "all") | (gas == "") | (sensor == 0)){
    return(float(NAN));
  }
  // Note that the sensor list is 0-indexed, i.e. the number needs to be -1
  //sensor -= 1;

  if(gas == "co2"){
  #if USE_SCD30
    gas_measured = scd_sensors[sensor-1]->airCO2();
    Serial.print("CO2 measurement: "); Serial.println(gas_measured);
  #endif
  }
  if (gas == "o2"){
  #if USE_SEN0465
    gas_measured = sen_sensors[sensor-1]->airO2();
    //Serial.print("O2 measurement: "); Serial.println(gas_measured);
  #endif
  }
  if (gas == "h2o"){
  #if USE_BME280
    // This sensor measures RH, but that's not useful for calibration, so the mole fraction has to be calculated
    gas_measured = env.air_water_mole_frac(bme_sensors[sensor-1]->airT(),
                                           bme_sensors[sensor-1]->airRH(),
                                           bme_sensors[sensor-1]->airP());
    //Serial.print("H2O measurement: "); Serial.println(gas_measured);
  #endif
  }
  
  return(gas_measured);
}

void onViewersConnect() {
	Serial.printf("onViewersConnect\n");
  h4wifi.uiAddText("Timestamp (UTC)", "");
  h4wifi.uiAddText("Location", "");
  /*h4wifi.uiAddDropdown("Calibration Type",{ // TODO: THIS DOES NOTHING YET
    {"Zero & Span","zero_span"},
    {"Differential","differential"},
  });*/
  h4wifi.uiAddText("---","");

  h4wifi.uiAddText("Zero & Span","Calibration");
  h4wifi.uiAddDropdown("Gas Sensor",{
    {"All","all"},
    {"H₂O","h2o"},
    {"CO₂","co2"},
    {"O₂", "o2"},
  });
  h4wifi.uiAddDropdown("Sensor Number",{
    {"0","0"},
    {"1","1"},
    {"2","2"}
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
  h4wifi.uiAddText("Current value", "");

  // TODO below
  h4wifi.uiAddText("---","");
  h4wifi.uiAddText("Alternative:","Δ Calibration");
  h4wifi.uiAddDropdown("Δ Type",{
    {"All","all"},
    {"H₂O","h2o"},
    {"CO₂","co2"},
    {"O₂", "o2"},
  });
  h4wifi.uiAddImgButton("setdiffLow");
  h4wifi.uiAddImgButton("setdiffHigh");
  h4wifi.uiAddText("Δ Gas",currentDiffGas.c_str());
  h4wifi.uiAddText("Sen. 1 value","");
  h4wifi.uiAddText("Sen. 2 value","");
  
  h4wifi.uiAddText("---","");
  h4wifi.uiAddImgButton("reset");
  
  // Every second, update the UI (only while someone is connected to the webUI)
  TIM0=h4.every(1000,[](){
    #if USE_GPS
    String date = gps.get_date();  
    int year = date.substring(0, 4).toInt();
    if ((year < 2025) | (year > 2050)) {
      h4wifi.uiSetValue("Timestamp (UTC)","NO GPS SIGNAL");
      h4wifi.uiSetValue("Location","NO GPS SIGNAL");
    } else {
      h4wifi.uiSetValue("Timestamp (UTC)",CSTR(gps.get_timestamp()));
      h4wifi.uiSetValue("Location",CSTR(gps.get_short_location()));
    }
    #else
    h4wifi.uiSetValue("Timestamp (UTC)","No GPS module");
    #endif
    h4wifi.uiSetValue("Current value",CSTR(String(measure_gas(currentGas, currentSensor))));
    h4wifi.uiSetValue("Sen. 1 value",CSTR(String(measure_gas(currentDiffGas, 1))));
    h4wifi.uiSetValue("Sen. 2 value",CSTR(String(measure_gas(currentDiffGas, 2))));
  });       
}

void onViewersDisconnect() {
	Serial.printf("onViewersDisconnect\n");
  h4.cancel({TIM0}); // Stop updating the UI
}

void onsetzeroButton(){
  h4wifi.uiMessage("Set zero");
  Serial.println("Set zero");
  if(currentGas == "all"){
    for (const auto &gas : gases){
      for(int sensor = 1; sensor <= numSensors; sensor++){
        float zero_measured = measure_gas(gas, sensor);
        // When all sensors are zero-calibrated together, we have to assume that zero is actually 0.0
        // for all gases (as there is only 1 input field)
        float zero_ref = 0.0;
        cal.set_calibration_coeff("zero", gas, sensor, zero_ref, zero_measured, gps.seconds_since_midnight());
      }
    }
    h4wifi.uiSetValue("Zero value", "All 0"); // TODO: measure. Note here, because this is "all", there is no value to measure
    h4wifi.uiSetValue("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
    h4wifi.uiMessage("IMPORTANT: All zeros set to 0.0");
    Serial.println("IMPORTANT: All zeros set to 0.0");
  } else {
    float zero_measured = measure_gas(currentGas, currentSensor);
    float zero_ref = std::stof(h4p.gvGetstring("Zero gas")); // The string automatically gets updated anytime a user enters a value
    cal.set_calibration_coeff("zero", currentGas, currentSensor, zero_ref, zero_measured, gps.seconds_since_midnight());
    Serial.println(zero_ref);
    Serial.println(zero_measured);
    h4wifi.uiSetValue("Zero value", CSTR(String(zero_measured, 2)));
    h4wifi.uiSetValue("Zero reference",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor))));
  }
}
void onsetspanButton(){
  if(currentGas == "all"){
    h4wifi.uiMessage("ERROR: Please calibrate span individually");
    Serial.println("ERROR: Please calibrate span individually");
  } else {
    h4wifi.uiMessage("Set span");
    Serial.println("Set span");
    float span_measured = measure_gas(currentGas, currentSensor);
    float span_ref = std::stof(h4p.gvGetstring("Span gas")); // The string automatically gets updated anytime a user enters a value
    Serial.println(span_ref);
    //cal.set_span(currentGas, currentSensor, span_ref, span_measured);
    cal.set_calibration_coeff("span", currentGas, currentSensor, span_ref, span_measured, gps.seconds_since_midnight());

    h4wifi.uiSetValue("Span value", CSTR(String(span_measured, 2)));
    h4wifi.uiSetValue("Span reference",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor))));
    //h4wifi.uiSetValue("Current value",span_measured); // Currently only updated when pushing button, should be live though
  }

  String cal_data_str = "";
  String cal_header = "";
#if USE_GPS
  String date = gps.get_date();  
  int year = date.substring(0, 4).toInt();
  if ((year < 2025) | (year > 2050)) {
    h4wifi.uiMessage("Error: No GPS fix");
    Serial.print(F("Skipping logging: year "));
    Serial.print(year);
    Serial.println(F(" > 2050"));
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
}

void onsetdiffLowButton(){
  h4wifi.uiMessage("Set low diff");
  Serial.println("Set low diff");
  if(currentGas == "all"){
    for (const auto &gas : gases){
      // Measure from both sensors
      float sensor1_measured = measure_gas(gas, 1);
      float sensor2_measured = measure_gas(gas, 2);
      // Set differential
      cal.set_differential_coeff("ref", currentDiffGas, sensor1_measured, sensor2_measured, gps.seconds_since_midnight());
      // Show values
      Serial.print("Current diff gas: "); Serial.println(gas);
      Serial.print("Sen. 1 value"); Serial.println(sensor1_measured);
      Serial.print("Sen. 2 value"); Serial.println(sensor2_measured);
      h4wifi.uiSetValue("Sen. 1 value", sensor1_measured);
      h4wifi.uiSetValue("Sen. 2 value", sensor2_measured);
    }
  } else {
    // Measure from both sensors
    float sensor1_measured = measure_gas(currentDiffGas, 1);
    float sensor2_measured = measure_gas(currentDiffGas, 2);
    // DEBUG: TEST (TODO)
    sensor1_measured = 15;
    sensor2_measured = 20;
    // Set differential
    cal.set_differential_coeff("ref", currentDiffGas, sensor1_measured, sensor2_measured, gps.seconds_since_midnight());
    // Show values
    Serial.print("Current diff gas: "); Serial.println(currentDiffGas);
    Serial.print("Sen. 1 value"); Serial.println(sensor1_measured);
    Serial.print("Sen. 2 value"); Serial.println(sensor2_measured);
    h4wifi.uiSetValue("Sen. 1 value", sensor1_measured);
    h4wifi.uiSetValue("Sen. 2 value", sensor2_measured);
  }
}

void onsetdiffHighButton(){
  h4wifi.uiMessage("Set high diff");
  Serial.println("Set high diff");
  if(currentGas == "all"){
    for (const auto &gas : gases){
      // Measure from both sensors
      float sensor1_measured = measure_gas(gas, 1);
      float sensor2_measured = measure_gas(gas, 2);
      // Set differential
      cal.set_differential_coeff("ref", currentDiffGas, sensor1_measured, sensor2_measured, gps.seconds_since_midnight());
      // Show values
      Serial.print("Current diff gas: "); Serial.println(gas);
      Serial.print("Sen. 1 value"); Serial.println(sensor1_measured);
      Serial.print("Sen. 2 value"); Serial.println(sensor2_measured);
      h4wifi.uiSetValue("Sen. 1 value", sensor1_measured);
      h4wifi.uiSetValue("Sen. 2 value", sensor2_measured);
    }
  } else {
    // Measure from both sensors
    float sensor1_measured = measure_gas(currentDiffGas, 1);
    float sensor2_measured = measure_gas(currentDiffGas, 2);
    // DEBUG: TEST (TODO)
    sensor1_measured = 15;
    sensor2_measured = 20;
    // Set differential
    cal.set_differential_coeff("ref", currentDiffGas, sensor1_measured, sensor2_measured, gps.seconds_since_midnight());
    // Show values
    Serial.print("Current diff gas: "); Serial.println(currentDiffGas);
    Serial.print("Sen. 1 value"); Serial.println(sensor1_measured);
    Serial.print("Sen. 2 value"); Serial.println(sensor2_measured);
    h4wifi.uiSetValue("Sen. 1 value", sensor1_measured);
    h4wifi.uiSetValue("Sen. 2 value", sensor2_measured);
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
      //Serial.printf("GLOBAL VARIABLE %s now= %s\n",CSTR(svc),CSTR(msg));
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
        h4p.gvSetstring("Zero gas",CSTR(String(cal.read_calibration_var("ref", "zero", currentGas, currentSensor)))); // Change input field value when dropdown selection changes
        h4p.gvSetstring("Span gas",CSTR(String(cal.read_calibration_var("ref", "span", currentGas, currentSensor)))); // Change input field value when dropdown selection changes
        break;
      }

      if(svc=="Δ Type"){ // Test differential gas selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Δ Gas",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which gas was chosen. IN THEORY THE SELECTION ABOVE IS SUFFICIENT!!!!!!!!!!!!!!!
        currentGas = msg.c_str();
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
    header += "bme_T.C,bme_RH.perc,bme_P.Pa,"; //,alt_agl.m
    data_str += String(bme_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
    data_str += String(bme_sensors[i]->airRH(), 2) + ",";         // RH                 [%]
    data_str += String(bme_sensors[i]->airP(), 2) + ",";          // Pressure           [Pa]
    //data_str += String(bme_sensors[i]->altitude_agl (), 2) + ","; // Altitude agl       [m]
    data_str += String(env.air_water_mole_frac(bme_sensors[i]->airT(),
                                           bme_sensors[i]->airRH(),
                                           bme_sensors[i]->airP()), 2) + ","; // Mole fraction [mmol/mol]
    n_measurements += 4;
    #if USE_CALIBRATION
    cal_header = cal.get_cal_header();
    //cal_header += "h2o.zero.mes,h2o_zero_ref,h2o_span_mes,h2o_span_ref,";
    #endif
  #endif
  #if USE_SCD30
    #if USE_BME280
    // Always update air pressure in CO2 sensor before using it
    scd_sensors[i]->set_air_pressure(bme_sensors[i]->airP());
    #endif
    header += "scd_T.C,scd_RH.perc,scd_CO2.ppm,";
    data_str += String(scd_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
    data_str += String(scd_sensors[i]->airRH(), 2) + ",";         // RH                 [%]
    data_str += String(scd_sensors[i]->airCO2(), 2) + ","; // CO2 concentration  [ppm] // Lab calibration measurements: 1.0431*CO2-31.727
    n_measurements += 3;
    #if USE_CALIBRATION
    cal_header += "co2.zero.mes,co2_zero_ref,co2_span_mes,co2_span_ref,";
    #endif
  #endif
  #if USE_SEN0465
    header += "sen_O2.ppm,sen_T.C,";
    data_str += String(sen_sensors[i]->airO2(), 2) + ",";         // Oxygen             [ppm]
    data_str += String(sen_sensors[i]->airT(), 2) + ",";          // Temperature        [C]
    n_measurements += 2;
    #if USE_CALIBRATION
    cal_header += "o2.zero.mes,o2_zero_ref,o2_span_mes,o2_span_ref,";
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
  String date = gps.get_date();  
  year = date.substring(0, 4).toInt();
  if ((year < 2025) | (year > 2050)) {
    Serial.print(F("Skipping logging: year "));
    Serial.print(year);
    Serial.println(F(" > 2050"));
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

// TODO: This needs to read calibration values for each gas from the relevant sensor (if enabled) and store it on the SD card
void store_calibration(String cal_header){
  // Store calibration data
#if USE_CALIBRATION & USE_MICROSD
  String calibration_fn = "calibration.csv";
#if USE_BME280
  cal_header = cal_header + "";
#endif
  String data_str = "";
  sd.write_data(calibration_fn.c_str(),
                cal_header.c_str(),
                data_str.c_str(),
                MEASUREMENT_INTERVAL); // logging frequency last [s]
#endif
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
  Serial.println(bme_sensors[i]->init(0x76) ? F("Success") : F("Failed"));
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

