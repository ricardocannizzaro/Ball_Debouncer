#include <pt.h>

#include <Servo.h>
#include <pt.h>   // include protothread library
#include "Enums.h"
#include "LoggerClass.h"

/*
 * External libraries
 * http://playground.arduino.cc/Code/QueueList
 * https://code.google.com/archive/p/arduinode/downloads - pt.h
 * 
 */

/*
 * System Variables:
 *********************************************************************************
 */

#define MAX_SYSTEM_LOG_ENTRIES  10000
#define MAX_NUM_ADD_REQUESTS       10
#define MAX_NUM_GET_REQUESTS       10

#define CRITICAL_DEBOUNCE_ERROR_AMOUNT (3)

//TODO: check these
#define SERVO_OPEN_VALUE (180)
#define SERVO_CLOSE_VALUE (0)
#define LIGHT_SENSOR_BALL_HELD_MIN_VALUE (100) //value has to between min and max for a ball to be held by bin
#define LIGHT_SENSOR_BALL_HELD_MAX_VALUE (400) //value has to between min and max for a ball to be held by bin
 
enum STATE SYSTEM_STATE;
struct Event SYSTEM_LAST_EVENT;

/*
 * GPIOS:
 *********************************************************************************
 */
#define LIGHT_SENSOR_INTERRUPT_PIN    2  // pin connected to light sensor for interrupt triggering
#define SERVO_CONTROL_PIN             9  // pin to control the servo with PWM
#define LED_PIN                      13  // pin to control the light

//TODO: specify pin numbers for these
#define LID_DOWN_SWITCH_PIN           3  // pin connected to the switch that is activated when the lid is fully closed
#define LIGHT_SENSOR_BALL_HELD_PIN    4  // pin connected to the light sensor that checks if a ball is held in the container

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
volatile int LidDownInterruptTriggered = 0;

/*
 * Logger Object:
 *********************************************************************************
 */
 static LoggerClass MyLogger;  // declare the system logger object
 static Servo MyServo;
 
void setup() {
  MyLogger = LoggerClass(); // initialise logger object
  MyServo = Servo();
  MyServo.attach(SERVO_CONTROL_PIN); // set the servo pin
  
  // put your setup code here, to run once:
  SYSTEM_STATE = STARTING_UP;
  
  //Initialise threads
  PT_INIT(&ptLogger);  // initialise the two
  PT_INIT(&ptStateMachine);  // protothread variables

  //set pin modes and interrupts
  pinMode(LED_PIN, OUTPUT); // LED init
  digitalWrite(LED_PIN, 1); // turn on LED
  pinMode(LIGHT_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(LID_DOWN_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIGHT_SENSOR_INTERRUPT_PIN), LightInterruptServiceRoutine, FALLING);
  attachInterrupt(digitalPinToInterrupt(LID_DOWN_SWITCH_PIN), LidDownInterruptServiceRoutine, FALLING);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  Serial.print("setup done\n");
  SYSTEM_STATE = WAITING_FOR_BOUNCE;  // put state machine into waiting for bounce state
}

/*
 * ****************
 * hardware sensor functions
 * TODO : check open and close values
 */
void servo_open()
{
  MyServo.write(SERVO_OPEN_VALUE);
}

void servo_close()
{
  MyServo.write(SERVO_CLOSE_VALUE);
}

bool servo_is_closed()
{
  return true;
  //return (LidDownInterruptTriggered == 1);
}

bool light_sensor_ball_held_in_bin()
{
  int light_v = analogRead(LIGHT_SENSOR_BALL_HELD_PIN);
  bool result =  (light_v > LIGHT_SENSOR_BALL_HELD_MIN_VALUE
  && light_v < LIGHT_SENSOR_BALL_HELD_MAX_VALUE);
  //return result;
  return true;
}

// function to set LightInterruptTriggered to high
void LightInterruptServiceRoutine(){
  if(LightInterruptTriggered==0){
    Serial.print("Light Interrupt Service Routine has been triggered...\n");
  }
  LightInterruptTriggered = 1;
}

// function to set LidDownInterruptTriggered to high
void LidDownInterruptServiceRoutine(){
  if(LidDownInterruptTriggered==0){
    Serial.print("Lid Down Interrupt Service Routine has been triggered...\n");
  }
  LidDownInterruptTriggered = 1;
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

    //toggleLED();

    MyLogger.ProcessLog(Serial);  // process any events on the queue
  }
  PT_END(pt);
}

//time at which a bounce occurs -- TODO: remove this
static unsigned long bounce_interrupt_time = 0;
static unsigned int debounce_errors = 0;


static Event event_create(EVENT_TYPE type, STATE current, STATE next, const char * desc)
{
  Event newEvent = {};
  newEvent.type = type;
  newEvent.currentState = current;
  newEvent.nextState = next;
  newEvent.data = {};
  snprintf(newEvent.description, EVENT_DESCRIPTION_MAX_CHARS, desc);
  newEvent.timestamp = millis();
  return newEvent;
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
      Serial.print("Waiting for bnc\n");
      //interrupts();
      // TODO - add timeout logic
      if(LightInterruptTriggered==1){
        // since light interrupt has been triggered, 
      
        // create interrupt event, then send it to logger input queue        
        Event newEvent = event_create(LIGHT_SENSOR_FALLING_EDGE_EVENT,
                                      WAITING_FOR_BOUNCE,
                                      DEBOUNCING,
                                      "LIGHT_SENSOR_FALLING_EDGE_EVENT. Transitioning to debouncing state.");

        //MyLogger.LogEvent(newEvent);

        bounce_interrupt_time = millis();
        //we can move onto the next state
        Serial.print("\tTransitioning to debouncing state...\n");
        SYSTEM_STATE = DEBOUNCING;
      }
      
      //noInterrupts();
    }
    else if(SYSTEM_STATE == DEBOUNCING){
      Serial.print ("debouncing\n") ;
      //toggleLED();

      //TODO: this is being called every time.. it only needs to be called once on state initial entry...
      servo_close(); 

      //check if 2 seconds has elapsed
      if (millis() - bounce_interrupt_time/* this is set at the end of debouncing*/ > 2000)
      {
          if (servo_is_closed())
          {
            Serial.print ("Lid closed, debnc-> check result\n") ;
            // move to check result state
            Event myEvent = event_create((EVENT_TYPE)0,SYSTEM_STATE,CHECKING_RESULT,
                                        "Lid closed, debnc-> check result");
            //MyLogger.LogEvent(myEvent);
            Serial.print("->checking result state...\n");
            SYSTEM_STATE = CHECKING_RESULT;  // update the system state 
          }
          else
          {
            Serial.print ("Lid ! closed, -> Emergency Stop\n") ;
            // if the servo is not closed, we've timed out
            // either call an emergency stop, or ask the user for input
            printf ("servo close timed out\n") ;
            Event myEvent = event_create((EVENT_TYPE)0,SYSTEM_STATE,EMERGENCY_STOP,
                                         "Error: servo did not close during debouncing");
            //MyLogger.LogEvent(myEvent);
            SYSTEM_STATE = EMERGENCY_STOP;  // update the system state
          }
      }
      /*
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
    }
    else if(SYSTEM_STATE == CHECKING_RESULT){
      Serial.print("CheckingR\n") ;
      //LCD::Clear();
      //LCD::Print("Checking result\n");
      // if timeout, then update system log with timeout
      // if lid closed
      //    if (floor sensor pressed), then update system log with success
      //    else update system log with failure
      // update statistics
      // if(too many failures, enter SHUTTING_DOWN state)
      // else enter RESTARTING state
      
      if (servo_is_closed())
      {
        if (light_sensor_ball_held_in_bin())
        {
          //success
          Serial.print("success.\n");
          //enter restart state
          Serial.print ("\tchecking->restarting\n") ;
          SYSTEM_STATE = RESTARTING;
        }
        else
        {
          //error
          Serial.print("failure\n");
          debounce_errors++;
          if (debounce_errors > CRITICAL_DEBOUNCE_ERROR_AMOUNT + 1)
          {
            //enter shutdown state
            Serial.print ("\tchecking->shuting down\n") ;
            SYSTEM_STATE = ENDING_PROGRAM;
          }
          else
          {
            //enter restart state
            Serial.print ("\tchecking->restarting\n") ;
            SYSTEM_STATE = RESTARTING;
          }
        }
      }
      LidDownInterruptTriggered = 0;
    }
    else if(SYSTEM_STATE == RESTARTING){
      //toggleLED();
      Serial.print ("Restarting / preparing for next bounce...\n") ;
      //LCD::Clear();
      //LCD::Print("Restarting / preparing for next bounce\n");
      // open the lid
      //servo::Open();

      // enter WAITING_FOR_BOUNCE state
      servo_open();
      Serial.print ("\t-> waiting for bounce\n\n") ;
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
  protothreadLogger(&ptLogger, 1000); // schedule the two protothreads
  protothreadStateMachine(&ptStateMachine, 1000); // by calling them infinitely
}
