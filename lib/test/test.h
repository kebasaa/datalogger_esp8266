/*
 * This is for testing code. Normally shouldn't run
 */

#ifndef Testh_h
#define Testh_h

#include <ESP8266WiFi.h>

//#include <H4Plugins.h>
#include <H4.h>

class Test {
  public:
    // main Class
    Test(void);
  
    // Functions
    //----------
    bool init(void);
	  void run_test(int someParam);
    void run_blocking_test(int id = 0);
    void run_when_done(void);
    bool currently_running = false;
 
  private:
    int  somePin = A0;

    H4_TIMER some_timer;

};

#endif
