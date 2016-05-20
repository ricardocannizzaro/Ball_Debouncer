#include <Servo.h>
#include <pt.h>   // include protothread library
#include "Enums.h"
//#include "LoggerClass.cpp"


/*
 * System Variables:
 *********************************************************************************
 */

#define MAX_SYSTEM_LOG_ENTRIES  10000
#define MAX_NUM_ADD_REQUESTS       10
#define MAX_NUM_GET_REQUESTS       10


enum STATE SYSTEM_STATE;

/*
 * GPIOS:
 *********************************************************************************
 */
#define LIGHT_SENSOR_INTERRUPT_PIN    2  // pin connected to light sensor for interrupt triggering
#define SERVO_CONTROL_PIN             9  // pin to control the servo with PWM
#define LED_PIN                      13  // pin to control the light

// A 'key' which we can lock and unlock 
//#define LOGGER_KEY 0

/*
 * Protothreading:
 *********************************************************************************
 */
static struct pt ptLogger, ptStateMachine; // each protothread needs one of these

/*
 * Interrupts:
 *********************************************************************************
 */
 volatile int LightInterruptTriggered = 0;

void setup() {
  // put your setup code here, to run once:
  SYSTEM_STATE = STARTING_UP;
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT); // LED init
  PT_INIT(&ptLogger);  // initialise the two
  PT_INIT(&ptStateMachine);  // protothread variables
  
  pinMode(LIGHT_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIGHT_SENSOR_INTERRUPT_PIN), LightInterruptServiceRoutine, FALLING);
  
  SYSTEM_STATE = WAITING_FOR_BOUNCE;  // put state machine into waiting for bounce state
}

// function to set LightInterruptTriggered to high
void LightInterruptServiceRoutine(){
  if(LightInterruptTriggered==0){
    Serial.print("Light Interrupt Service Routine has been triggered...\n");
  }
  LightInterruptTriggered = 1;
}


// function to toggle the LED_PIN output
void toggleLED() {
  boolean ledstate = digitalRead(LED_PIN); // get LED state
  ledstate ^= 1;   // toggle LED state using xor
  digitalWrite(LED_PIN, ledstate); // write inversed state back
}

/* This function toggles the LED after 'interval' ms passed */
static int protothreadLogger(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) { // never stop 
    /* each time the function is called the second boolean
    *  argument "millis() - timestamp > interval" is re-evaluated
    *  and if false the function exits after that. */
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    
    // do things here
    Serial.print("protothreadLogger triggered at: ");
    Serial.print(timestamp);
    Serial.print("\n");
    toggleLED();
  }
  PT_END(pt);
}

/* exactly the same as the protothread1 function */
static int protothreadStateMachine(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    
    // do things here
    //toggleLED();
    
    if(SYSTEM_STATE == WAITING_FOR_BOUNCE){
      Serial.print("Waiting for bounce...\n");
      //interrupts();
      // TODO - add timeout logic
      if(LightInterruptTriggered==1){
        // since light interrupt has been triggered, we can move onto the next state
        Serial.print("\tTransitioning to debouncing state...\n");
        SYSTEM_STATE = DEBOUNCING;
      }
      
      //noInterrupts();
    }
    else if(SYSTEM_STATE == DEBOUNCING){
      Serial.print ("Debouncing...\n") ;
      toggleLED();
      /*LCD::Clear();
      LCD::Print("Debouncing\n");*/
  
      //CloseLidStartTime = time(NULL);    // set the close lid start time as the current time
  
      // command a close of the lid - TODO
      /*
      servo::Close();
      //while (difftime(CloseLidStartTime,time(0))<=2 && !servo::IsClosed()){
      while (difftime(time(NULL),CloseLidStartTime)<=2 ){
      // while (not closed & not timeout) (Or maybe just implement a 1-2 second delay)
              //    stay in state
              //    close lid
              // essentially, do nothing;
      }
  
      if (!servo::IsClosed()){
          // if the servo is not closed, we've timed out
          // either call an emergency stop, or ask the user for input
          printf ("servo close timed out\n") ;
          myEvent.timestamp = time(0);
          myEvent.currentState = SYSTEM_STATE;
          myEvent.nextState = EMERGENCY_STOP;
          myEvent.description = "Error: servo did not close during debouncing";
          addEventsQueue.push_back(myEvent);

          SYSTEM_STATE = EMERGENCY_STOP;  // update the system state
      }
      else{
          // move to check result state
          printf ("servo close success\n") ;
          myEvent.timestamp = time(0);
          myEvent.currentState = SYSTEM_STATE;
          myEvent.nextState = CHECKING_RESULT;
          myEvent.description = "Transitioning from debouncing state to checking result";
          addEventsQueue.push_back(myEvent);

          SYSTEM_STATE = CHECKING_RESULT;  // update the system state

      }
      */
      Serial.print("\tTransitioning to checking result state...\n");
      SYSTEM_STATE = CHECKING_RESULT;  // update the system state - TODO: implement servo error logic (if the lid doesnt close)
    }
    else if(SYSTEM_STATE == CHECKING_RESULT){
      Serial.print("Checking result...\n") ;
      //LCD::Clear();
      //LCD::Print("Checking result\n");
      // if timeout, then update system log with timeout
      // if lid closed
      //    if (floor sensor pressed), then update system log with success
      //    else update system log with failure
      // update statistics
      // if(too many failures, enter SHUTTING_DOWN state)
      // else enter RESTARTING state
      Serial.print ("\tTransitioning to waiting for restarting state...\n") ;
      SYSTEM_STATE = RESTARTING;
    }
    else if(SYSTEM_STATE == RESTARTING){
      toggleLED();
      Serial.print ("Restarting / preparing for next bounce...\n") ;
      //LCD::Clear();
      //LCD::Print("Restarting / preparing for next bounce\n");
      // open the lid
      //servo::Open();
      // enter WAITING_FOR_BOUNCE state
      Serial.print ("\tTransitioning to waiting for bounce state...\n\n") ;
      SYSTEM_STATE = WAITING_FOR_BOUNCE;
      LightInterruptTriggered = 0; // reset for next time
    }
    else if(SYSTEM_STATE == ENDING_PROGRAM){
      Serial.print  ("Ending program\n") ;
      //LCD::Clear();
      //LCD::Print("Ending program\n");
      // run end program tasks - TODO
      break;
    }
    else if(SYSTEM_STATE == EMERGENCY_STOP){
      Serial.print ("Emergency Stop - state machine will stop here! \n") ;
      //LCD::Clear();
      //LCD::Print("Emergency Stop\n");
      // run end program tasks - TODO
      break;
    }
  }
  PT_END(pt);
}

void loop() {
  // put your main code here, to run repeatedly:
  //protothreadLogger(&ptLogger, 900); // schedule the two protothreads
  protothreadStateMachine(&ptStateMachine, 1000); // by calling them infinitely
}
