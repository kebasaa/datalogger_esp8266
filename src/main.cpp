#include <Arduino.h>
#include "config.h"
// #define H4P_VERBOSE 1
#include <H4Plugins.h>
H4_USE_PLUGINS(PROJ_BAUD_RATE, H4_Q_CAPACITY, false) // Serial baud rate, Q size, SerialCmd autostop

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

char* name = "Datalogger";
#if H4P_USE_WIFI_AP
H4P_WiFi h4wifi(name);
#else
H4P_WiFi h4wifi(WIFI_SSID, WIFI_PASS, name);
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

H4P_Heartbeat h4hb; // Show uptime

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
# include <SEN0465.h>
SEN0465 sen;
#endif

// Calculate environmental parameters
#if USE_ENV
# include <Environmental.h>
Env env;
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
void onViewersConnect() {
	Serial.printf("onViewersConnect\n");
	h4wifi.uiAddGlobal("heap");
	h4wifi.uiAddGlobal("pool");
}
void onViewersDisconnect() {
	Serial.printf("onViewersDisconnect\n");
}

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
  Serial.println("Logging");
  // Prepare output string
  String data_str = "";

  // Measurement count
  int n_measurements = 0;
  String header = "";

#if I2C_BUS2
  mp.enableBus(2);
#endif
#if USE_GPS
  // Add timestamps and location
  gps.update_values();
  data_str = data_str + gps.get_timestamp() + ",";
  data_str = data_str + gps.get_location() + ",";
  header += "timestamp,lat,lon,alt,nb_sat,HDOP,";
#endif
#if I2C_BUS2
  mp.disableBus(2);
#endif

#if USE_BATTERY
  header += "bat.mV,bat.perc,";
  data_str += String(bat.battery_mV()) + ",";    // Battery charge     [mV]
  data_str += String(bat.battery_pc()) + ",";    // Battery charge     [%]
  n_measurements += 2;
#endif

#if I2C_BUS2
  mp.enableBus(2);
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
#endif
#if USE_SCD30
#if USE_BME280
  // Always update air pressure in CO2 sensor before using it
  scd.set_air_pressure(bme.airP());
#endif
  header += "co2_T.C,co2_RH.perc,co2_CO2.ppm,";
  data_str += String(scd.airT(), 2) + ",";          // Temperature        [C]
  data_str += String(scd.airRH(), 2) + ",";         // RH                 [%]
  data_str += String(1.0431*scd.airCO2()-31.727, 2) + ","; // CO2 concentration  [ppm] // Lab calibration measurements: 1.0431*CO2-31.727
  n_measurements += 3;
#endif
#if USE_SEN0465
  header += "o2_O2.ppm,o2_T.C,";
  data_str += String(sen.airO2(), 2) + ",";         // Oxygen             [ppm]
  data_str += String(sen.airT(), 2) + ",";          // Temperature        [C]
  n_measurements += 2;
#endif
#if USE_MLX90614
  header += "mlx_T.C,mlx_obj_T.C,";
  data_str += String(mlx.airT(), 2) + ",";          // Air temperature    [C]
  data_str += String(mlx.objT(), 2) + ",";          // Object temperature [C]
  n_measurements += 2;
#endif

#if I2C_BUS2
  mp.disableBus(2);
#endif
// Now measure devices on bus 3
#if I2C_BUS3
  mp.enableBus(3);
  header += "bus3,"; // Add empty column to separate bus 3
  data_str += ","; // Add empty column to separate bus 3

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
#endif
#if USE_SCD30
#if USE_BME280
  // Always update air pressure in CO2 sensor before using it
  scd.set_air_pressure(bme.airP());
#endif
  header += "co2_T.C,co2_RH.perc,co2_CO2.ppm,";
  data_str += String(scd.airT(), 2) + ",";          // Temperature        [C]
  data_str += String(scd.airRH(), 2) + ",";         // RH                 [%]
  data_str += String(1.0431*scd.airCO2()-31.727, 2) + ","; // CO2 concentration  [ppm] // Lab calibration measurements: 1.0431*CO2-31.727
  n_measurements += 3;
#endif
#if USE_SEN0465
  header += "o2_O2.ppm,o2_T.C,";
  data_str += String(sen.airO2(), 2) + ",";         // Oxygen             [ppm]
  data_str += String(sen.airT(), 2) + ",";          // Temperature        [C]
  n_measurements += 2;
#endif
#if USE_MLX90614
  header += "mlx_T.C,mlx_obj_T.C,";
  data_str += String(mlx.airT(), 2) + ",";          // Air temperature    [C]
  data_str += String(mlx.objT(), 2) + ",";          // Object temperature [C]
  n_measurements += 2;
#endif

  mp.disableBus(3);
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
  String filename = "data_" + gps.get_date() + ".csv";
#else
  String filename = "data.csv";
#endif

#if USE_MICROSD
  // Write data to disk
  sd.write_data(filename.c_str(),
                header.c_str(),
                data_str.c_str(),
                MEASUREMENT_INTERVAL); // logging frequency last [s]
#endif
  
  // Debug: Show data on Serial output
  Serial.println(data_str);
}

void h4setup(){
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
  Serial.print(F("- i2c multiplexer           "));
#if I2C_MULTI
  if(! mp.init()){
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Success"));
  }
#endif
  Wire.begin();

#if I2C_BUS2
  int currentBus = 2;
  mp.enableBus(currentBus);
  Serial.print(F("- i2c bus "));Serial.print(currentBus);Serial.println(F(":"));
#endif

#if USE_GPS
  Serial.print(F("  - XA1110 GPS:               "));
  Serial.println(gps.init() ? F("Success") : F("Failed"));
#endif

#if USE_BME280
  Serial.print(F("  - BME280 sensor:            "));
  Serial.println(bme.init(0x76) ? F("Success") : F("Failed"));
#endif

#if USE_SCD30
  Serial.print(F("  - SCD-30 sensor:      "));
  Serial.println(scd.init() ? F("Success") : F("Failed"));
  scd.enable_self_calibration(false); // don't calibrate, we want to do that manually
  scd.set_interval(2);                // minimum every 2s
#endif

#if USE_SEN0465
  Serial.print(F("  - SEN0465 sensor:            "));
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

#if I2C_BUS2
  mp.disableBus(currentBus);
#endif

  // Next i2c bus
#if I2C_BUS3
  currentBus = 3;
  mp.enableBus(currentBus);
  Serial.print(F("- i2c bus "));Serial.print(currentBus);Serial.println(F(":"));


#if USE_BME280
  Serial.print(F("  - BME280 sensor:            "));
  Serial.println(bme.init(0x76) ? F("Success") : F("Failed"));
#endif

#if USE_SCD30
  Serial.print(F("  - SCD-30 sensor:      "));
  Serial.println(scd.init() ? F("Success") : F("Failed"));
  scd.enable_self_calibration(false); // don't calibrate, we want to do that manually
  scd.set_interval(2);                // minimum every 2s
#endif

#if USE_SEN0465
  Serial.print(F("  - SEN0465 sensor:            "));
  Serial.println(sen.init() ? F("Success") : F("Failed"));
#endif


  mp.disableBus(currentBus);
#endif

  Serial.println(F("Initialisation completed"));
  Serial.println(F(""));

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

