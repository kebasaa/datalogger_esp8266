/*
 * Apogee SO-411 sensor
 *
 * Note on SO-411
 * 8M! or 8M0!: Calibrated oxygen, sensor mV, sensor body temperature
 * 8M1!:        Calibrated oxygen percent corrected for temperature
 */
 
#include "Arduino.h"
#include "SO411.h"

#include <SDI12.h>
//#include <H4.h>
//H4 scheduler;

//SO411::SO411(uint8_t dataPin, uint8_t powerPin) : mySDI12(dataPin){
SO411::SO411(uint8_t addr, uint8_t dataPin, uint8_t powerPin){
  _dataPin  = dataPin;
  _powerPin = powerPin;
  _addr     = addr;
}

bool SO411::init(void){
  //#define DATA_PIN 4         /*!< The pin of the SDI-12 data bus */
  //#define POWER_PIN -1       /*!< The sensor power pin (or -1 if not switching power) */
  mySDI12 = SDI12(_dataPin);  // Dynamically create the SDI12 object
  mySDI12.begin();

  // If a power pin is provided, it should be used to turn the sensor ON
  if (_powerPin > 0) {
    pinMode(_powerPin, OUTPUT);
    digitalWrite(_powerPin, HIGH);
    //delay(200);
  }

  return(true);
}

/*
 * Convert address characters ('0'-'9', 'a'-'z', 'A'-'Z') to
 * decimal number between 0 and 61 to cover all possible addresses
 */
byte SO411::charToDec(char i) {
  if ((i >= '0') && (i <= '9')) return i - '0';
  if ((i >= 'a') && (i <= 'z')) return i - 'a' + 10;
  if ((i >= 'A') && (i <= 'Z'))
    return i - 'A' + 36;
  else
    return i;
}

/**
 * Map a decimal number between 0 and 61 to
 * address characters '0'-'9', 'a'-'z', 'A'-'Z'
 */
char SO411::decToChar(byte i) {
  if (i < 10) return i + '0';
  if ((i >= 10) && (i < 36)) return i + 'a' - 10;
  if ((i >= 36) && (i <= 62))
    return i + 'A' - 36;
  else
    return i;
}

void SO411::send_cmd(String cmd_type = "M", String measurement_type_nb = ""){
  //String res_data = ""; // To store sensor data
  char addr_char = decToChar(_addr); // Sensor address must be a character

  // Create the full measurement command and send it
  String command = "";
  command += addr_char;
  command += cmd_type;
  command += measurement_type_nb;
  command += "!";  // SDI-12 measurement command format  [address]['M'][!]
  mySDI12.sendCommand(command);
}

void SO411::read_answer(){
  // Read the response from the sensor and trim a bit
  sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();
}

void SO411::parse_sensor_response(String cmd){
  if (cmd == "I") {
    parse_info();
  } else if (cmd == "M") {
    //parse_command();
  } else {
    // If no known condition is met, abort
    return;
  }
}

void SO411::parse_info(){
  // allccccccccmmmmmmvvvxxx...xx<CR><LF>
  Serial.println(sdiResponse);
  Serial.print(sdiResponse.substring(0, 1));  // address
  Serial.print(", ");
  Serial.print(sdiResponse.substring(1, 3).toFloat() / 10);  // SDI-12 version number
  Serial.print(", ");
  Serial.print(sdiResponse.substring(3, 11));  // vendor id
  Serial.print(", ");
  Serial.print(sdiResponse.substring(11, 17));  // sensor model
  Serial.print(", ");
  Serial.print(sdiResponse.substring(17, 20));  // sensor version
  Serial.print(", ");
  Serial.print(sdiResponse.substring(20));  // sensor id
  Serial.print(", ");
  Serial.println("");
}

    /*h4.repeatWhile([this](){ return currently_running == true; },
                   50, [](){},
                   [this](){ Serial.println("Nothing else running"); });*/
    /*h4.repeatWhile([this](){ return currently_running == true; },
                   50, [](){},
                   [this, i](){ 
                    h4.queueFunction([this, i](){
                      // Send command
                      Serial.println("Sending command");
                    },[this, i](){
                      // When done, do the h4.ounce(100,...)
                      // Set currently_running to false
                      h4.once(100, [this]() {
                        Serial.println("Waited 100ms");
                      },[this](){
                        Serial.println("Finishing off");
                      });
                    });
                  });*/

void SO411::read_sensor(String cmd){

  // Create the command
  String command = "";
  if (cmd == "info") {
    command = 'I';
  } else if (cmd == "measure") {
    command = 'M';
  } else {
    // If no known condition is met, abort
    command = '?';
    return;
  }

  h4.repeatWhile([this](){ return currently_running; }, 50,
  [](){}, // Do nothing while waiting for something else to be done
  [this, command](){ // The other thing is done, so now wait 100ms then run the rest
    currently_running = true;
    // Send command for information
    send_cmd(command, "");
    // When done, wait 1s, then set currently_running to false
    h4.once(100, [this, command]() {
      // Read answer from the sensor
      read_answer();
      parse_sensor_response(command);
    },[this](){
      h4.once(100, [this]() {},
      [this](){
        mySDI12.clearBuffer();
        currently_running = false;
      });
    });
  });
}

void SO411::read_sensor_old(String cmd){
  // Create the command
  String command = "";
  if (cmd == "info") {
    command = 'I';
  } else if (cmd == "measure") {
    command = 'M';
  } else {
    // If no known condition is met, abort
    command = '?';
    return;
  }

  // Every 50ms check if something else is running
  // If something else is running, just wait around. Otherwise run the command
  h4.repeatWhile([this](){ return currently_running; }, 50,
    [](){}, // Do nothing while waiting for something else to be done
    [this, command](){ // When the other thing finishes send a command
      h4.queueFunction([this, command](){
      //h4.once(100, [this, command](){
        // Send command for information
        currently_running = true;
        send_cmd(command, "");
      //},[this, command](){ // The other thing is done, so now wait 100ms then run the rest
        // When done, do the h4.ounce(100,...)
        // Set currently_running to false
        h4.once(100, [this, command]() {
          // Read answer from the sensor
          read_answer();
          parse_sensor_response(command);
        },[this](){
          // Wait to finish up
          //h4.once(1000, [](){}, [this](){Serial.println("Done talking to sensor"); currently_running = false;});
          currently_running = false;
        });
      });
    });
}

// Should be removed. Still here for tests
void SO411::printInfo(char i) {
    String cmd = "info";
    // Every 50ms check if something else is running
    // If something else is running, just wait around. Otherwise run the command
    h4.repeatWhile([this](){ return currently_running == true; }, 10,
      [](){}, // Do nothing while waiting for something else to be done
      [this, cmd](){ // When the other thing finishes send a command
        h4.queueFunction([this](){
          // Send command for information
          send_cmd("I", "");
        },[this, cmd](){ // The other thing is done, so now wait 100ms then run the rest
          // When done, do the h4.ounce(100,...)
          // Set currently_running to false
          h4.once(100, [this, cmd]() {
            // Read answer from the sensor
            read_answer();
            parse_sensor_response(cmd);
          },[this](){
            currently_running = false;
          });
        });
      });
}

///////// Newest test

void SO411::send_measure_command(char i, String meas_type) {
    // Queue for sending
    h4.once(100, [this, i, meas_type](){
      // Prepare command
      mySDI12.clearBuffer();
      String command = "";
      command += i;
      command += "M";
      command += meas_type;
      command += "!";  // SDI-12 measurement command format  [address]['M'][!]
      mySDI12.sendCommand(command);
    });

    // Schedule reading response after 100ms (replacing delay)
    h4.once(200, [this]() { //100
        String sdiResponse = mySDI12.readStringUntil('\n');
        sdiResponse.trim();
        Serial.println(sdiResponse); // show the data
    });
}

/////////////



bool SO411::getResults(char i, int resultsExpected) {
  uint8_t resultsReceived = 0;
  uint8_t cmd_number      = 0;

  // Initialise array for data storage
  float outputArray[resultsExpected];
  for(int j = 0; j < resultsReceived; j++){ outputArray[i] = -999; }

  while (resultsReceived < resultsExpected && cmd_number <= 9) {
    String command = "";
    // in this example we will only take the 'DO' measurement
    command = "";
    command += i;
    command += "D";
    command += cmd_number;
    command += "!";  // SDI-12 command to get data [address][D][dataOption][!]
    mySDI12.sendCommand(command);
    //Serial.print("Getting results with: ");Serial.println(command);

    uint32_t start = millis();
    while (mySDI12.available() < 3 && (millis() - start) < 1500) {}
    mySDI12.read();           // ignore the repeated SDI12 address
    char c = mySDI12.peek();  // check if there's a '+' and toss if so
    if (c == '+') { mySDI12.read(); }

    while (mySDI12.available()) {
      char c = mySDI12.peek();
      if (c == '-' || (c >= '0' && c <= '9') || c == '.') {
        float result = mySDI12.parseFloat(SKIP_NONE);
        //Serial.print(String(result, 10));
        if (result != -9999) {
          outputArray[resultsReceived] = result;
          resultsReceived++;
          }
      } else if (c == '+') {
        mySDI12.read();
        //Serial.print(", ");
      } else {
        mySDI12.read();
      }
      delay(10);  // 1 character ~ 7.5ms
    }
    if (resultsReceived < resultsExpected) { Serial.print(", "); }
    cmd_number++;
  }
  mySDI12.clearBuffer();

  // Jonathan's tests
  Serial.println(""); Serial.print("Nb results: "); Serial.println(resultsReceived);
  for(int j = 0; j < resultsReceived; j++){ Serial.print(outputArray[j]); Serial.print(", "); }
  Serial.println("");

  return resultsReceived == resultsExpected;
}

/*
bool SO411::takeMeasurement(char i, String meas_type) {
  mySDI12.clearBuffer();
  String command = "";
  command += i;
  command += "M";
  command += meas_type;
  command += "!";  // SDI-12 measurement command format  [address]['M'][!]
  mySDI12.sendCommand(command);
  delay(100);

  // Sensor replies with data format [address][ttt (3 char, in seconds)][number of
  // measurments available, 0-9]
  String sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();

  String addr = sdiResponse.substring(0, 1);

  // find out how long we have to wait for measurement (in seconds).
  uint8_t wait = sdiResponse.substring(1, 4).toInt();

  // Set up the number of results to expect
  int resultsExpected = sdiResponse.substring(4).toInt();

  // Show time taken to measure in ms
  unsigned long timerStart = millis();
  while ((millis() - timerStart) < (1000 * (wait + 1))) {
    if (mySDI12.available())  // sensor can interrupt us to let us know it is done early
    {
      //Serial.print(millis() - timerStart);
      //Serial.print(", ");
      mySDI12.clearBuffer();
      break;
    }
  }
  // Wait for anything else and clear it out
  delay(30);
  mySDI12.clearBuffer();

  if (resultsExpected > 0) { return getResults(i, resultsExpected); }

  return true;
}
*/



bool SO411::get_sensor_metadata(){
  // Sensor replies with data format [address][ttt (3 char, in seconds)][number of
  // measurments available, 0-9]
  String sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();
  
  // Identify which sensor is replying
  String addr = sdiResponse.substring(0, 1);

  // Save info on how long to wait for measurement [s]
  _wait = sdiResponse.substring(1, 4).toInt();

  // Save the number of results to expect
  _resultsExpected = sdiResponse.substring(4).toInt();

  return(true);
}

void SO411::read_data(){
  // read data
  mySDI12.read();           // ignore the repeated SDI12 address
  char c = mySDI12.peek();  // check if there's a '+' and toss if so
  if (c == '+') { mySDI12.read(); }

  h4.repeatWhile([this](){ return !mySDI12.available(); }, 10, [this](){
    char c = mySDI12.peek();
    if (c == '-' || (c >= '0' && c <= '9') || c == '.') {
      float result = mySDI12.parseFloat(SKIP_NONE);
      if (result != -9999) {
        sensor_data[_nb_results_received] = result;
        _nb_results_received++;
      }
    } else if (c == '+') {
      mySDI12.read();
    } else {
      mySDI12.read();
    }
  });
}

void SO411::process_results(){
  // check every 50ms if the condition is met, otherwise continue
  h4.repeatWhile([this](){
    return (_nb_results_received < _resultsExpected && _cmd_number <= 9);
  }, 50, [this](){
    send_cmd("", "D");
    data_ready_timer = h4.repeatWhile([this](){ return (mySDI12.available() >= 3); }, 50, [this](){}, [this](){
      // When the condition is met or the timer (below) finishes
      read_data();
      _cmd_number++;
    });
    // If the sensor doesn't become available, time out & cancel after ca. "wait" seconds
    h4.once(1500, [this]() {
      h4.finishNow(data_ready_timer);
    });
  }, [this](){
    // Finish by clearing  buffer
    mySDI12.clearBuffer();
    // And reset counter variables
    _cmd_number = 0;
    _nb_results_received = 0;
  });
}

// This is the function to call before getting any data, at regular intervals, to tell the sensor to measure & return data
bool SO411::async_data_read(String meas_type = ""){
  // Send the command to the sensor
  char addr_char = decToChar(_addr); // Sensor address must be a character
  if (!checkActive(addr_char)) { // If sensor is not active, measurement failed
    return(false);
  }
  mySDI12.clearBuffer();
  send_cmd(meas_type, "M");

  // Now time things
  h4.once(100, [this](){
    get_sensor_metadata();
  },[this](){
    // Repeat until X returns 0
    check_available_timer = h4.repeatWhile([this](){ return !mySDI12.available(); }, 50, [this](){
    }, [this](){
      // When the sensor data becomes available, clear the buffer
      mySDI12.clearBuffer();
    });
    // If the sensor doesn't become available, time out & cancel after ca. "wait" seconds
    h4.once((1000 * (_wait + 1)), [this]() {
      h4.finishNow(check_available_timer);
      mySDI12.clearBuffer();
    });
    if (_resultsExpected > 0) { process_results(); }
  });
  
  // Done successfully
  return(true);
}

/////////////////////////////
/*
void SO411::checkSensorAvailability() {
    if (mySDI12.available()) {
        mySDI12.clearBuffer();  // Clear the buffer if sensor responds early
        h4.cancel(check_sdi_timer);  // Cancel the timer, which is checking for timeout
    }
}

void SO411::waitForSensor(int waitTime) {

    // Schedule a repeating check every 100ms
    check_sdi_timer = h4.every(50, [this]() {checkSensorAvailability();});

    // Schedule the timeout event for (waitTime + 1) seconds
    h4.once(1000 * (waitTime + 1), []() {
        // If we get here, the sensor did not respond in time
        Serial.println("Timeout! Sensor did not respond.");
        h4.cancel(check_sdi_timer); // Cancel checking times
    });  // Assign the timer an ID of '1' so we can cancel it if needed
}

bool SO411::sensorResponse(){
  // Sensor replies with data format [address][ttt (3 char, in seconds)][number of
  // measurments available, 0-9]
  String sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();
  
  // Identify which sensor is replying
  String addr = sdiResponse.substring(0, 1);

  // Save info on how long to wait for measurement [s]
  _wait = sdiResponse.substring(1, 4).toInt();

  // Save the number of results to expect
  _resultsExpected = sdiResponse.substring(4).toInt();

  return(true);
}
*/

bool SO411::takeMeasurement(char i, String meas_type) {
  mySDI12.clearBuffer();
  String command = "";
  command += i;
  command += "M";
  command += meas_type;
  command += "!";  // SDI-12 measurement command format  [address]['M'][!]
  mySDI12.sendCommand(command);
  delay(100);

  // Sensor replies with data format [address][ttt (3 char, in seconds)][number of
  // measurments available, 0-9]
  String sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();

  String addr = sdiResponse.substring(0, 1);

  // find out how long we have to wait for measurement (in seconds).
  uint8_t wait = sdiResponse.substring(1, 4).toInt();

  // Set up the number of results to expect
  int resultsExpected = sdiResponse.substring(4).toInt();

  // Show time taken to measure in ms
  unsigned long timerStart = millis();
  while ((millis() - timerStart) < (1000 * (wait + 1))) {
    if (mySDI12.available())  // sensor can interrupt us to let us know it is done early
    {
      //Serial.print(millis() - timerStart);
      //Serial.print(", ");
      mySDI12.clearBuffer();
      break;
    }
  }
  // Wait for anything else and clear it out
  delay(30);
  mySDI12.clearBuffer();

  if (resultsExpected > 0) { return getResults(i, resultsExpected); }

  return true;
}

// this checks for activity at a particular address
// expects a char, '0'-'9', 'a'-'z', or 'A'-'Z'
boolean SO411::checkActive(char i) {
  String myCommand = "";
  myCommand        = "";
  myCommand += (char)i;  // sends basic 'acknowledge' command [address][!]
  myCommand += "!";

  for (int j = 0; j < 3; j++) {  // goes through three rapid contact attempts
    mySDI12.sendCommand(myCommand);
    delay(100);
    if (mySDI12.available()) {  // If we here anything, assume we have an active sensor
      mySDI12.clearBuffer();
      return true;
    }
  }
  mySDI12.clearBuffer();
  return false;
}



//////////////////// combined

bool SO411::takeMeasurement_combo(char i, String meas_type) {
  mySDI12.clearBuffer();
  String command = "";
  command += i;
  command += "M";
  command += meas_type;
  command += "!";  // SDI-12 measurement command format  [address]['M'][!]
  mySDI12.sendCommand(command);
  delay(100);

  // Sensor replies with data format [address][ttt (3 char, in seconds)][number of
  // measurments available, 0-9]
  String sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();

  String addr = sdiResponse.substring(0, 1);

  // find out how long we have to wait for measurement (in seconds).
  uint8_t wait = sdiResponse.substring(1, 4).toInt();

  // Set up the number of results to expect
  int resultsExpected = sdiResponse.substring(4).toInt();

  // Show time taken to measure in ms
  unsigned long timerStart = millis();
  while ((millis() - timerStart) < (1000 * (wait + 1))) {
    if (mySDI12.available())  // sensor can interrupt us to let us know it is done early
    {
      //Serial.print(millis() - timerStart);
      //Serial.print(", ");
      mySDI12.clearBuffer();
      break;
    }
  }
  // Wait for anything else and clear it out
  delay(30);
  mySDI12.clearBuffer();

  if (resultsExpected <= 0) {
    return true;
  }

  uint8_t resultsReceived = 0;
  uint8_t cmd_number      = 0;

  // Initialise array for data storage
  float outputArray[resultsExpected];
  for(int j = 0; j < resultsReceived; j++){ outputArray[i] = -999; }

  while (resultsReceived < resultsExpected && cmd_number <= 9) {
    String command = "";
    // in this example we will only take the 'DO' measurement
    command = "";
    command += i;
    command += "D";
    command += cmd_number;
    command += "!";  // SDI-12 command to get data [address][D][dataOption][!]
    mySDI12.sendCommand(command);
    //Serial.print("Getting results with: ");Serial.println(command);

    uint32_t start = millis();
    while (mySDI12.available() < 3 && (millis() - start) < 1500) {}
    mySDI12.read();           // ignore the repeated SDI12 address
    char c = mySDI12.peek();  // check if there's a '+' and toss if so
    if (c == '+') { mySDI12.read(); }

    while (mySDI12.available()) {
      char c = mySDI12.peek();
      if (c == '-' || (c >= '0' && c <= '9') || c == '.') {
        float result = mySDI12.parseFloat(SKIP_NONE);
        //Serial.print(String(result, 10));
        if (result != -9999) {
          outputArray[resultsReceived] = result;
          resultsReceived++;
          }
      } else if (c == '+') {
        mySDI12.read();
        //Serial.print(", ");
      } else {
        mySDI12.read();
      }
      delay(10);  // 1 character ~ 7.5ms
    }
    if (resultsReceived < resultsExpected) { Serial.print(", "); }
    cmd_number++;
  }
  mySDI12.clearBuffer();

  // Jonathan's tests
  Serial.println(""); Serial.print("Nb results: "); Serial.println(resultsReceived);
  for(int j = 0; j < resultsReceived; j++){ Serial.print(outputArray[j]); Serial.print(", "); }
  Serial.println("");

  return resultsReceived == resultsExpected;
}