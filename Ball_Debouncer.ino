#include <Servo.h>
#include <pt.h>   // include protothread library

#define MAX_SYSTEM_LOG_ENTRIES  10000
#define MAX_NUM_ADD_REQUESTS 10
#define MAX_NUM_GET_REQUESTS 10

/*
 * Pins:
 *********************************************************************************
 */
#define PIN_LIGHT_SENSOR   0  // Light sensor for interrupt
#define LEDPIN            13  // LEDPIN is a constant 

// A 'key' which we can lock and unlock 
#define LOGGER_KEY 0

void setup() {
  // put your setup code here, to run once:

 
  /*
   * Interrupts:
   *********************************************************************************
   */
   
  
  void lightSensorInterrupt (void) { ++globalCounter [PIN_LIGHT_SENSOR] ; }
  void myInterrupt1 (void) { ++globalCounter [1] ; }
  void myInterrupt2 (void) { ++globalCounter [2] ; }
  void myInterrupt3 (void) { ++globalCounter [3] ; }
  void myInterrupt4 (void) { ++globalCounter [4] ; }
  void myInterrupt5 (void) { ++globalCounter [5] ; }
  void myInterrupt6 (void) { ++globalCounter [6] ; }
  void myInterrupt7 (void) { ++globalCounter [7] ; }

}

void loop() {
  // put your main code here, to run repeatedly:

}
