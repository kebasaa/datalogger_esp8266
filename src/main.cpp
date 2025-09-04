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
GPS gps;
#endif

// T, RH, P sensor
#if USE_BME280
# include <BME280.h>
BME bme;
#endif

// CO2 sensor
#if USE_SCD30
# include <SCD30.h>
SCD scd;
#endif

// LWR sensor
#if USE_MLX90614
#include <MLX90614.h>
MLX mlx;
#endif

// ADS1115 analog-to-digital converter, unused in this project
#if USE_ADS1115
# include <ADS1115.h>
ADS ads;
#endif

// SEN0465 O2 sensor
#if USE_SEN0465
#include <SEN0465.h>
SEN0465 sen;
#endif

// Calculate environmental parameters
#if USE_ENV
#include <Environmental.h>
Env env;
#endif

#if USE_CAL
#include <Calibration.h>
Cal cal;
std::vector<String> gases = {"co2", "o2", "h2o"};
int numSensors = 2;
int currentSensor = 0;
String currentGas = "all";
//float currentZero = 0.0;
//float currentSpan = 400.0;
#endif

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
// JUST FOR TESTING
const char* USER_TEXT_NAME="User Text";
//const char* myText="Select";
const char* usertext="Upper Or Lower?";

void onViewersConnect() {
	Serial.printf("onViewersConnect\n");
	//h4wifi.uiAddGlobal("heap");
	//h4wifi.uiAddGlobal("pool");
  h4wifi.uiAddText("Zero & Span","Calibration");
  h4wifi.uiAddDropdown("Sensor Type",{
    {"All","everything"},
    {"H₂O","H₂O"},
    {"CO₂","CO₂"},
    {"O₂", "O₂"},
  });
  h4wifi.uiAddDropdown("Sensor Number",{
    {"All","all sensors"},
    {"1","1"},
    {"2","2"}
  });
  h4wifi.uiAddInput("Zero gas");
  h4wifi.uiAddImgButton("setzero");
  h4wifi.uiAddInput("Span gas");
  h4wifi.uiAddImgButton("setspan");

  h4wifi.uiAddText("Selected type:","");
  h4wifi.uiAddText("Selected number:","");

  h4wifi.uiAddText("Alternative:","Δ Calibration");
  h4wifi.uiAddDropdown("Δ Type",{
    {"All","all sensors"},
    {"H₂O","H₂O"},
    {"CO₂","CO₂"},
    {"O₂", "O₂"},
  });
  h4wifi.uiAddImgButton("setdiff");
}

void onViewersDisconnect() {
	Serial.printf("onViewersDisconnect\n");
}

float measure_gas(String currentGas, int currentSensor){
#if I2C_MULTI
  mp.enableBus(i2c_buses[currentSensor]);
#endif

  float gas_measured = 0.0;
  if(currentGas == "co2"){
  #if USE_SCD30
    gas_measured = scd.airCO2();
  #endif
  }
  if (currentGas == "o2"){
  #if USE_SEN0465
    gas_measured = scd.airCO2();
  #endif
  }
  if (currentGas == "h2o"){
  #if USE_BME280
    // This sensor measures RH, but that's not useful for calibration
    gas_measured = env.air_water_mole_frac(bme.airT(), bme.airRH(), bme.airP());
  #endif
  }

#if I2C_MULTI
  mp.disableBus(i2c_buses[currentSensor]);
#endif
  
  return(gas_measured);
}

void onsetzeroButton(){
  h4wifi.uiMessage("Set zero");
  Serial.println("Set zero");
  float zero_measured = measure_gas(currentGas, currentSensor);
  float zero_ref = std::stof(h4p.gvGetstring("Zero gas")); // The string automatically gest updated anytime a user enters a value
  Serial.println(zero_ref);
  cal.set_zero(currentGas, currentSensor, zero_ref, zero_measured);
}
void onsetspanButton(){
  h4wifi.uiMessage("Set span");
  Serial.println("Set span");
  float span_measured = measure_gas(currentGas, currentSensor);
  float span_ref = std::stof(h4p.gvGetstring("Span gas")); // The string automatically gest updated anytime a user enters a value
  Serial.println(span_ref);
  cal.set_span(currentGas, currentSensor, span_ref, span_measured);
}
void onsetdiffButton(){
  h4wifi.uiMessage("Set diff");
  Serial.println("Set diff");
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
      H4P_TACT_BUTTON_CONNECTOR(setdiff);
      // Process button presses
      if(svc=="Sensor Type"){ // Test sensor type selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Selected gas:",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which gas was chosen
        currentGas = msg.c_str();
        break;
      }
      if(svc=="Sensor Number"){ // Test sensor type selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Selected sensor number:",CSTR(msg));
        Serial.print("You chose "); Serial.println(msg.c_str());
        // Store which sensor was chosen & convert to int
        String msg_str = msg.c_str();
        currentSensor = msg_str.toInt();
        break;
      }

      if(svc=="Δ Type"){ // Test sensor type selection
        h4wifi.uiMessage("You chose %s\n",CSTR(msg));
        h4wifi.uiSetValue("Selected gas:",CSTR(msg));
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
  //Serial.println("Logging");
  // Prepare output string
  String data_str = "";

  // Measurement count
  int n_measurements = 0;
  String header = "";
  String cal_header = "";

#if I2C_MULTI
  mp.enableBus(2);
#endif
#if USE_GPS
  // Add timestamps and location
  gps.update_values();
  data_str = data_str + gps.get_timestamp() + ",";
  data_str = data_str + gps.get_location() + ",";
  header += "timestamp,lat,lon,alt,nb_sat,HDOP,";
#endif
#if I2C_MULTI
  mp.disableBus(2);
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
    // Enable the i2c bus
    mp.enableBus(i2c_buses[i]);
#endif
  
  #if USE_ADS1115
    header += "Sin.W_m2,Sout.W_m2,";
    data_str += String(ads.read_val(1, 16, 25.83), 2) + ","; // ADC of Apogee SP-510, A1, Gain 16: ±0.256V, convert by 25.83 W m-2 / mV
    data_str += String(ads.read_val(2, 16, 30.98), 2) + ","; // ADC of Apogee SP-610, A2, Gain 16: ±0.256V, convert by 30.98 W m-2 / mV
    n_measurements += 2;
  #endif
  #if USE_BME280
    header += "bme_T.C,bme_RH.perc,bme_P.Pa,alt_agl.m,";
    data_str += String(bme.airT(), 2) + ",";          // Temperature        [C]
    data_str += String(bme.airRH(), 2) + ",";         // RH                 [%]
    data_str += String(bme.airP(), 2) + ",";          // Pressure           [Pa]
    data_str += String(bme.altitude_agl (), 2) + ","; // Altitude agl       [m]
    n_measurements += 4;
    #if USE_CALIBRATION
    cal_header = cal.get_cal_header();
    //cal_header += "h2o.zero.mes,h2o_zero_ref,h2o_span_mes,h2o_span_ref,";
    #endif
  #endif
  #if USE_SCD30
    #if USE_BME280
    // Always update air pressure in CO2 sensor before using it
    scd.set_air_pressure(bme.airP());
    #endif
    header += "scd_T.C,scd_RH.perc,scd_CO2.ppm,";
    data_str += String(scd.airT(), 2) + ",";          // Temperature        [C]
    data_str += String(scd.airRH(), 2) + ",";         // RH                 [%]
    data_str += String(1.0431*scd.airCO2()-31.727, 2) + ","; // CO2 concentration  [ppm] // Lab calibration measurements: 1.0431*CO2-31.727
    n_measurements += 3;
    #if USE_CALIBRATION
    cal_header += "co2.zero.mes,co2_zero_ref,co2_span_mes,co2_span_ref,";
    #endif
  #endif
  #if USE_SEN0465
    header += "sen_O2.ppm,sen_T.C,";
    data_str += String(sen.airO2(), 2) + ",";         // Oxygen             [ppm]
    data_str += String(sen.airT(), 2) + ",";          // Temperature        [C]
    n_measurements += 2;
    #if USE_CALIBRATION
    cal_header += "o2.zero.mes,o2_zero_ref,o2_span_mes,o2_span_ref,";
    #endif
  #endif
  #if USE_MLX90614
    header += "mlx_T.C,mlx_obj_T.C,";
    data_str += String(mlx.airT(), 2) + ",";          // Air temperature    [C]
    data_str += String(mlx.objT(), 2) + ",";          // Object temperature [C]
    n_measurements += 2;
  #endif

#if I2C_MULTI
    header += ",";
    data_str += ",";
    // Disable the i2c bus
    mp.disableBus(i2c_buses[i]);
  }
#endif

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
  mp.enableBus(2);
  Serial.print(F("  - XA1110 GPS:             "));
  Serial.println(gps.init() ? F("Success") : F("Failed"));
  mp.disableBus(2);
#endif

  // Go through i2c buses
#if I2C_MULTI
  size_t n = sizeof(i2c_buses)/sizeof(i2c_buses[0]);
  for (size_t i = 0; i < n; ++i) {
    // Enable the i2c bus
    Serial.print(F("- i2c bus "));Serial.print(i2c_buses[i]);Serial.println(F(":"));
    mp.enableBus(i2c_buses[i]);
#endif

#if USE_CAL
  //cal.create_var("cal_co2_1_zero", 0.0);
  cal.init_all_calibrations(gases, numSensors);
#endif

#if USE_BME280
  Serial.print(F("  - BME280 sensor:            "));
  Serial.println(bme.init(0x76) ? F("Success") : F("Failed"));
#endif

#if USE_SCD30
  Serial.print(F("  - SCD-30 sensor:            "));
  Serial.println(scd.init() ? F("Success") : F("Failed"));
  scd.enable_self_calibration(false); // don't calibrate, we want to do that manually
  scd.set_interval(2);                // minimum every 2s
#endif

#if USE_SEN0465
  Serial.print(F("  - SEN0465 sensor:           "));
  Serial.println(sen.init() ? F("Success") : F("Failed"));
#endif

#if USE_ADS1115
  Serial.print(F("  - ADS1115 (Analog-digital): "));
  Serial.println(ads.init() ? F("Success") : F("Failed"));
#endif

#if USE_MLX90614
  Serial.print(F("  - MLX90614 sensor:    "));
  Serial.println(mlx.init() ? F("Success") : F("Failed"));
#endif

#if I2C_MULTI
    // Disable the i2c bus
    mp.disableBus(i2c_buses[i]);
  }
#endif

  Serial.println(F("Initialisation completed"));
  Serial.println(F("")); 
  
  // Print IP address to Serial
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

  //TEST calibration buttons
  //h4p.gvSetInt("setzero",10);
  //h4p.gvSetInt("setspan",10);
  h4p.gvSetstring("Zero gas","0.00"); // TEST to store known calibration gas values?
  h4p.gvSetstring("Span gas","400.00"); // TEST to store known calibration gas values?

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

