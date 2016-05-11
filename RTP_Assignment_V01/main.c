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
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <time.h>
#include <string.h>

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
 * Interrupts:
 *********************************************************************************
 */

void lightSensorInterrupt (void) { ++globalCounter [0] ; }
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

enum STATE { STARTING_UP, WAITING_FOR_BOUNCE, DEBOUNCING, CHECKING_RESULT, RESTARTING, ENDING_PROGRAM, EMERGENCY_STOP };
enum EVENT_TYPE { SYSTEM_EVENT, LIGHT_SENSOR_EVENT, FLOOR_SENSOR_EVENT, SERVO_EVENT, ALARM_EVENT };
enum SERVO_COMMAND { OPEN, CLOSE };	// used to communicate to servo function

enum STATE SYSTEM_STATE;

/*
 * structs:
 *********************************************************************************
 */
 
 struct event_s {
  enum EVENT_TYPE type;
  int data[10];
  char description[128];
  time_t timestamp;
} ;


/*
 * system log variables:
 *********************************************************************************
 */

struct event_s systemLog[MAX_SYSTEM_LOG_ENTRIES];	// the system log
int 	numSystemLogEvents = 0;				// the number of entries in the system log
struct event_s addRequests[MAX_NUM_ADD_REQUESTS];	// the queue to add events to the log
int 	numAddRequests = 0;					// the number of events waiting on the add queue
struct event_s getRequests[MAX_NUM_GET_REQUESTS];	// the queue to get events from the log
int 	numGetRequests = 0;					// the number of events waiting on the get queue

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
		if(numSystemLogEvents < MAX_SYSTEM_LOG_ENTRIES){
			if(numAddRequests > 0 && numAddRequests < MAX_NUM_ADD_REQUESTS){
				printf ("processing event\n") ;
				// take the first element and add it to the system log
				systemLog[numSystemLogEvents] = addRequests[0];
				numSystemLogEvents = numSystemLogEvents + 1;
				// TODO remove the first element
				// then decrement the number of add requests
				numAddRequests = numAddRequests - 1;
			}
			else if (numAddRequests <= 0){
				// no requests to add events to the system log. do nothing.
				printf ("no requests to add events. doing nothing\n") ;
			}
			else if (numAddRequests >= MAX_NUM_ADD_REQUESTS){
				// too many add requests
				printf ("Max number of add requests exceeded\n") ;
			}
			
			
			if(numGetRequests > 0 && numGetRequests < MAX_NUM_GET_REQUESTS){
				// TODO: search for a request and return it ?, then remove the first element
			}
			else{
				// no requests to get events to the system log. do nothing.
			}
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
		// TODO - command motor to open
	}
	if( command == CLOSE){
		// TODO - command motor to close
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
  piThreadCreate (loggerThread) ;				// create & run loggerThread
  
  int gotOne, pin ;
  int myCounter [8] ;
  //enum SERVO_COMMAND servoCommand = OPEN;		// initialise to open
  // TODO send command to open the lid

  for (pin = 0 ; pin < 8 ; ++pin) 
    globalCounter [pin] = myCounter [pin] = 0 ;

  wiringPiSetup () ;

  wiringPiISR (0, INT_EDGE_FALLING, &lightSensorInterrupt) ;
  wiringPiISR (1, INT_EDGE_FALLING, &myInterrupt1) ;
  wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ;
  wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ;
  wiringPiISR (4, INT_EDGE_FALLING, &myInterrupt4) ;
  wiringPiISR (5, INT_EDGE_FALLING, &myInterrupt5) ;
  wiringPiISR (6, INT_EDGE_FALLING, &myInterrupt6) ;
  wiringPiISR (7, INT_EDGE_FALLING, &myInterrupt7) ;
  
  // create a test event
  struct event_s myEvent;
  myEvent.type = SYSTEM_EVENT;
  myEvent.data[0] = 9;
  myEvent.description[0] = 'H';
  myEvent.timestamp = time(NULL); // the current time
  
  addRequests[0] = myEvent;
  numAddRequests = numAddRequests + 1;
	
	SYSTEM_STATE = WAITING_FOR_BOUNCE;	// setup complete, transition to WAITING_FOR_BOUNCE state
	// main loop starts here
	
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
		// while (not closed & not timeout) (Or maybe just implement a 1-2 second delay)
		// 		stay in state
		//		close lid
		// move to check result state
		SYSTEM_STATE = CHECKING_RESULT;
	}
	else if(SYSTEM_STATE == CHECKING_RESULT){
		printf ("Checking result\n") ;
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
		// open the lid
		// enter WAITING_FOR_BOUNCE state
		SYSTEM_STATE = WAITING_FOR_BOUNCE;
	}
	else if(SYSTEM_STATE == ENDING_PROGRAM){
		printf ("Ending program\n") ;
		// run end program tasks
		return 0;
	}
	else if(SYSTEM_STATE == EMERGENCY_STOP){
		printf ("Emergency Stop\n") ;
		// run end program tasks
		return 0;
	}
  }

  return 0 ;
}
