/*
 * This is for testing code. Normally shouldn't run
 */

#include "test.h"

Test::Test(void){
    // Battery
}

bool Test::init(void){
    // Something
    //Serial.println("Initializing Test");
    return(1);
}

void Test::run_when_done(void){
    Serial.println("Timer has ended");
}

void Test::run_test(int someParam){
    // Something
    
    some_timer = h4.every(1000, [this]() {
        Serial.println("Running test");
    }, [this]() { run_when_done(); });
    
    h4.once(3500, [this]() {
        Serial.println("Ending the timer");
        h4.cancel(some_timer);
    });

    /*h4.repeatWhile([this](){ return true; }, 1500, [this]() {
        Serial.println("repeat now");
    }, [this](){
        Serial.println("Done repeating");
    });*/

}

/*void Test::run_blocking_test(void){
    h4.repeatWhile([this](){ return currently_running; }, 50,
    [](){ Serial.println("Waiting to finish"); }, // Do nothing while waiting for something else to be done
    [this](){ // The other thing is done, so now wait 100ms then run the rest
      currently_running = true;
      Serial.println("Start: set currently_running == true");
      // When done, wait 1s, then set currently_running to false
      h4.once(1000, [this]() {
        // Done waiting 1s
        Serial.println("Waited 1s");
      },[this](){
        h4.once(1000, [this]() {
          // Read answer from the sensor
          Serial.println("Waited another 1s");
        },[this](){
          currently_running = false;
          Serial.println("Done, setting currently_running = false");
        });
        // Now finish up
        //Serial.println("Done done");
        //currently_running = false;
      });
    });
}*/

void Test::run_blocking_test(int id){
    h4.repeatWhile([this](){ return currently_running; }, 50,
    [](){ Serial.println("Waiting to finish"); }, // Do nothing while waiting for something else to be done
    [this, id](){ // The other thing is done, so now wait 100ms then run the rest
      currently_running = true;
      Serial.print("ID: "); Serial.print(id); Serial.print(" "); Serial.println("Start: set currently_running == true");
      // When done, wait 1s, then set currently_running to false
      int starting_time = millis();
      h4.every(50, [this, starting_time]() {
        
        // Do something every 50ms
      },[this](){
        // Finish up
      });
    });
}

