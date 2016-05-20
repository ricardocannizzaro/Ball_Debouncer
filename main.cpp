/*
 * isr.c:
 *	Wait for Interrupt test program - ISR method
 *
 *	How to test:
 *	  Use the SoC's pull-up and pull down resistors that are avalable
 *	on input pins. So compile & run this program (via sudo), then
 *	in another terminal:
 *		gpio mode 0 up
 *		gpio mode 0 down
 *	at which point it should trigger an interrupt. Toggle the pin
 *	up/down to generate more interrupts to test.
 *
 * Copyright (c) 2013 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <iostream>
#include <string>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h> //remember to use the compiler flag -lwiringPi
#include <time.h>
#include "servo.h"
#include "rgb_lcd.h"
#include <vector>
#include <wiringPiSPI.h>

using namespace std;

// globalCounter:
//	Global variable to count interrupts
//	Should be declared volatile to make sure the compiler doesn't cache it.

static volatile int globalCounter [8] ;

#define MAX_SYSTEM_LOG_ENTRIES  10000
#define MAX_NUM_ADD_REQUESTS 10
#define MAX_NUM_GET_REQUESTS 10



// A 'key' which we can lock and unlock - values are 0 through 3
//	This is interpreted internally as a pthread_mutex by wiringPi
//	which is hiding some of that to make life simple.
#define LOGGER_KEY 0

/*
 * Pins:
 *********************************************************************************
 */
#define PIN_LIGHT_SENSOR (0)
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


/*
 * enums:
 *********************************************************************************
 */

enum STATE {
	STARTING_UP,
	WAITING_FOR_BOUNCE,
	DEBOUNCING,
	CHECKING_RESULT,
	RESTARTING,
	ENDING_PROGRAM,
	EMERGENCY_STOP	};

enum EVENT_TYPE {       //TODO make system transition event types
	SYSTEM_EVENT,
	STATE_CHANGE,
	LIGHT_SENSOR_FALLING_EDGE_EVENT,
	FLOOR_SENSOR_EVENT,
	SERVO_OPEN_EVENT,
	SERVO_CLOSE_EVENT,
	ALARM_EVENT };

enum SERVO_COMMAND { OPEN, CLOSE };	// used to communicate to servo function

enum STATE SYSTEM_STATE;

/*
 * structs:
 *********************************************************************************
 */

 struct event_s {
  enum EVENT_TYPE type;
  enum STATE currentState;  // used to describe the state
  enum STATE nextState;     // used in state change events
  std::vector<int> data;
  std::string description;
  time_t timestamp;
} ;


/*
 * system log variables:
 *********************************************************************************
 */

std::vector <event_s> systemLog;	// the system log
std::vector <event_s> addEventsQueue;	// the queue to add events to the log
//event_s getRequests[MAX_NUM_GET_REQUESTS];	// the queue to get events from the log   // TODO - work out how to return request results



/*
 * Time variable
*/
time_t CloseLidStartTime;

PI_THREAD (environmentMonitorThread)
{
	for (;;){
		printf ("environmentMonitorThread running\n") ;
		delay (1000) ;
	}
}


PI_THREAD (loggerThread)
{
	for (;;){
		printf ("loggerThread running\n") ;
		if(systemLog.size() < MAX_SYSTEM_LOG_ENTRIES){
			if(addEventsQueue.size() > 0 && addEventsQueue.size() < MAX_NUM_ADD_REQUESTS){
				printf ("processing event\n") ;
				// take the first element and add it to the system log
				systemLog.push_back(addEventsQueue[0]);

				// remove the first element
				addEventsQueue.erase(addEventsQueue.begin());
			}
			else if (addEventsQueue.size() <= 0){
				// no requests to add events to the system log. do nothing.
				printf ("no requests to add events. doing nothing\n") ;
			}
			else if (addEventsQueue.size() >= MAX_NUM_ADD_REQUESTS){
				// too many add requests
				printf ("Max number of add requests exceeded\n") ;
			}

            /*
            // TODO - implement this once we figure out how to implement the Get_request vector
			if(numGetRequests > 0 && numGetRequests < MAX_NUM_GET_REQUESTS){
				// TODO: search for a request and return it ?, then remove the first element
			}
			else{
				// no requests to get events to the system log. do nothing.
			}*/
		}
		else{
			printf ("Error - Too many system log entries\n") ;
		}
		delay (1000) ;
	}
}

void setServoState(enum SERVO_COMMAND command){
	// function to handle the PWM Hat servo control

	if( command == OPEN){
		servo::Open();
	}
	if( command == CLOSE){
		servo::Close();
	}
}

/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main (void)
{
  SYSTEM_STATE = STARTING_UP;

  //piThreadCreate (environmentMonitorThread) ;	// create & run environmentMonitorThread
  //piThreadCreate (loggerThread) ;				// create & run loggerThread

  int gotOne, pin ;
  int myCounter [8] ;

  //enum SERVO_COMMAND servoCommand = OPEN;		// initialise to open
  // TODO send command to open the lid

  for (pin = 0 ; pin < 8 ; ++pin)
    globalCounter [pin] = myCounter [pin] = 0 ;

  wiringPiSetup () ;

  //wiringPiISR (PIN_LIGHT_SENSOR, INT_EDGE_FALLING, &lightSensorInterrupt) ;
  //wiringPiISR (1, INT_EDGE_FALLING, &myInterrupt1) ;
  //wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ;
  //wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ;
  //wiringPiISR (4, INT_EDGE_FALLING, &myInterrupt4) ;
  //wiringPiISR (5, INT_EDGE_FALLING, &myInterrupt5) ;
 // wiringPiISR (6, INT_EDGE_FALLING, &myInterrupt6) ;
 // wiringPiISR (7, INT_EDGE_FALLING, &myInterrupt7) ;


  // set up ADC
   /*mcp3008Spi a2d("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);

    int a2dVal = 0;
    int a2dChannel = 0;
    unsigned char data[3];


        data[0] = 1;  //  first byte transmitted -> start bit
        data[1] = 0b10000000 |( ((a2dChannel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
        data[2] = 0; // third byte transmitted....don't care

        a2d.spiWriteRead(data, sizeof(data) );

        a2dVal = 0;
        a2dVal = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
        a2dVal |=  (data[2] & 0xff);


        printf ("The Result is: ", a2dVal,"\n") ;*/

    //#define BASE 100
    //#define SPI_CHAN 0

    unsigned char buffer[100] = {};
    int result;
    int SPI_CHANNEL = 0;
    int speed = 1000000;
    char ADC_channel0 = 0b00001000;
    char ADC_channel1 = 0b00001001;
    char ADC_channel2 = 0b00001010;
    char ADC_channel3 = 0b00001011;
    char ADC_channel4 = 0b00001100;
    char ADC_channel5 = 0b00001101;
    char ADC_channel6 = 0b00001110;
    char ADC_channel7 = 0b00001111;


    wiringPiSPISetup(SPI_CHANNEL, speed);

    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    for(int i = 0; i < 20; i++){
        buffer[0] = 0xff;
        buffer[1] = ADC_channel0;
        digitalWrite(25, HIGH);
        delay(50);
        result = wiringPiSPIDataRW(SPI_CHANNEL, buffer, 2);
        digitalWrite(25, LOW);
        cout << "Channel 0: " << buffer[0] + buffer[1] << endl;
        delay(50);
    }
    digitalWrite(25, LOW);
  //setup servo (static object)
  servo::Initialise();
  servo::Open();                // initialise to open

    // test by doing a sweep
    /*
float pos;
    for(pos = 0.00 ; pos <= 1; pos = pos + .05){
        servo::SetPos(pos);
        delay (100000) ;
    }
*/
  bool TempBool = servo::IsOpen();
  TempBool = servo::IsClosed();

  //setup LCD screen (static object)
  LCD::Initialise();

  // create a test event
  event_s myEvent;
  myEvent.type = SYSTEM_EVENT;
  //myEvent.data[0] = 9;
  myEvent.description = "Test Event";
  myEvent.timestamp = time(NULL); // the current time
  myEvent.currentState = SYSTEM_STATE;

  addEventsQueue.push_back(myEvent);

  delay (1000) ; //some time for everything to finish

  SYSTEM_STATE = WAITING_FOR_BOUNCE;	// setup complete, transition to WAITING_FOR_BOUNCE state
 // SYSTEM_STATE = DEBOUNCING;    // debugging

	// main loop starts here

  LCD::Print("Waiting for bounce");
  for (;;)
  {

    if(SYSTEM_STATE == WAITING_FOR_BOUNCE){

		gotOne = 0 ;
		printf ("Waiting ... ") ; fflush (stdout) ;

		for (;;)
		{
		  for (pin = 0 ; pin < 8 ; ++pin)
		  {
			if (globalCounter [pin] != myCounter [pin])
			{
			  printf (" Int on pin %d: Counter: %5d\n", pin, globalCounter [pin]) ;
			  myCounter [pin] = globalCounter [pin] ;
			  ++gotOne ;
			}
		  }
		  if (gotOne != 0){
			SYSTEM_STATE = DEBOUNCING;
			break ;
		}
		}
	}
	else if(SYSTEM_STATE == DEBOUNCING){
		printf ("Debouncing\n") ;
		LCD::Clear();
		LCD::Print("Debouncing\n");

		CloseLidStartTime = time(NULL);    // set the close lid start time as the current time

		// command a close of the lid
		servo::Close();
		//while (difftime(CloseLidStartTime,time(0))<=2 && !servo::IsClosed()){
		while (difftime(time(NULL),CloseLidStartTime)<=2 ){
		// while (not closed & not timeout) (Or maybe just implement a 1-2 second delay)
            // 		stay in state
            //		close lid
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


	}
	else if(SYSTEM_STATE == CHECKING_RESULT){
		printf ("Checking result\n") ;
		LCD::Clear();
		LCD::Print("Checking result\n");
		// if timeout, then update system log with timeout
		// if lid closed
		//		if (floor sensor pressed), then update system log with success
		//		else update system log with failure
		// update statistics
		// if(too many failures, enter SHUTTING_DOWN state)
		// else enter RESTARTING state
		SYSTEM_STATE = RESTARTING;
	}
	else if(SYSTEM_STATE == RESTARTING){
		printf ("Restarting / preparing for next bounce\n") ;
		LCD::Clear();
		LCD::Print("Restarting / preparing for next bounce\n");
		// open the lid
		servo::Open();
		// enter WAITING_FOR_BOUNCE state
		SYSTEM_STATE = WAITING_FOR_BOUNCE;
	}
	else if(SYSTEM_STATE == ENDING_PROGRAM){
		printf ("Ending program\n") ;
		LCD::Clear();
		LCD::Print("Ending program\n");
		// run end program tasks
		return 0;
	}
	else if(SYSTEM_STATE == EMERGENCY_STOP){
		printf ("Emergency Stop\n") ;
		LCD::Clear();
		LCD::Print("Emergency Stop\n");
		// run end program tasks
		return 0;
	}
  }
  return 0 ;
}
