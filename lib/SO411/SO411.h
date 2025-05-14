/*
 * SDI-12 sensor
 */

#ifndef SO411h_h
#define SO411h_h

#include "Arduino.h"
#include <SDI12.h>
#include <H4.h>

class SO411 {
  public:
    // Main Class
    //-----------
    SO411(uint8_t addr, uint8_t dataPin, uint8_t powerPin);

    // Variables
    //-----------
    //int addr;
  
    // Functions
    //----------
    bool init(void);
    bool takeMeasurement(char i, String meas_type = "");
    bool takeMeasurement_combo(char i, String meas_type = "");
    void printInfo(char i);
    bool getResults(char i, int resultsExpected);
    //void processSO411Data(char i, int resultsExpected, int& resultsReceived, int cmd_number, float outputArray[]); // Jonathan
    boolean checkActive(char i);
    char decToChar(byte i);
    // Jonathan
    //bool read_data(int addr_int, String meas_type);
    bool sensorResponse(void);
    void checkSensorAvailability();
    void waitForSensor(int waitTime);

    void read_data(void);
    bool async_data_read(String meas_type);
    void send_cmd(String cmd_type, String measurement_type_nb); // Make private
    bool get_sensor_metadata(void); // Make private
    unsigned long startTime = 0, waitingTime = 0; // Make private

    // Jonathan Newest
    void process_results(void); // Make private
    H4_TIMER check_available_timer;
    H4_TIMER get_metadata_timer, sensor_ready_timer, data_ready_timer; // currently unused
    float sensor_data[5] = {-999.0, -999.0, -999.0, -999.0, -999.0};
    int _nb_results_received = 0, _cmd_number = 0;
    void send_measure_command(char i, String meas_type = "");
    bool currently_running = false;
    void read_answer(void);
    String sdiResponse = "";
    void parse_sensor_response(String cmd = "");
    void parse_info(void);
    void read_sensor(String cmd);
    void read_sensor_old(String cmd);

  private:
    SDI12 mySDI12;
    // Define the pin for the SDI-12 bus
    uint8_t _dataPin;
    uint8_t _powerPin;
    uint8_t _addr;
    bool isActive[64];
    // Jonathan
    int _resultsExpected = 0;
    int nb_received = 0;
    uint8_t _wait = 0;
    H4_TIMER check_sdi_timer, waiting_timer, read_timer;

    byte charToDec(char i);
};

#endif
