
#include "cmsis_os.h"  
#include "stm32f4xx_hal.h"
#include "MachineControlPCB_LED.h"
#include "MachineControlPCB_Motor.h"  
#include "MachineControlPCB_Buttons.h"

enum fsm_states {
    start = 0,
    moving_upward = 1,
    moving_downward = 2,
    moving_upward_wait = 3,
    moving_downward_wait = 4,
		timeoutreached_error = 5
};

static int counter = 0;
static int waitstates_upward = 100;
static int waitstates_downward = 100;
static int timeout_motor = 300;
enum fsm_states state = start;

/*----------------------------------------------------------------------------
 *      Timer: Sample timer functions
 *---------------------------------------------------------------------------*/
 

/*----- One-Shoot Timer Example -----*/
static void Timer1_Callback (void const *arg);                  // prototype for timer callback function

static osTimerId id1;                                           // timer id
static uint32_t  exec1;  // argument for the timer call back function
static osTimerDef (Timer1, Timer1_Callback);                    // define timers

// One-Shoot Timer Function
static void Timer1_Callback (void const *arg) {
	if ((!Button_Downward_Reached()) && (state == moving_downward) || (state == start)) 
	{
		LED_Off(1);
		LED_Off(0);
		LED_Off(2);
		Motor_Off(0);
		counter = 0;
		state = moving_upward_wait;
	}
  else if ((!Button_Upward_Reached()) && (state == moving_upward))
	{
		LED_Off(0);
		LED_Off(1);
		LED_Off(2);
		Motor_Off(0);
		counter = 0;
		state = moving_downward_wait;
	}
	else 
	{
		if (state == moving_downward_wait)
	  {
			if (counter >= waitstates_downward)
			{
				LED_Off(1);
				LED_On(0);
				LED_Off(2);
				Motor_Downward(0);
				counter = 0;
				state = moving_downward;
			}
			else
			{
				counter = counter + 1;
			}
		}
		else if (state == moving_upward_wait)
	  {
			if (counter >= waitstates_upward)
			{
				LED_Off(1);
				LED_On(0);
				LED_Off(2);
				Motor_Upward(0);
				counter = 0;
				state = moving_upward;
			}
			else
			{
				counter = counter + 1;
			}
		}
		else if ((state == moving_downward) || (state == moving_upward))
		{
			if (counter >= timeout_motor)
			{
				LED_Off(1);
				LED_Off(0);
				LED_Off(2);
				Motor_Off(0);
				state = timeoutreached_error;
			}
			else
			{
				counter = counter + 1;
			}
		}
	}
}


/*----- Periodic Timer Example -----*/
static void Timer2_Callback (void const *arg);                  // prototype for timer callback function

static osTimerId id2;                                           // timer id
static uint32_t  exec2;                                         // argument for the timer call back function
static osTimerDef (Timer2, Timer2_Callback);
 
// Periodic Timer Example
static void Timer2_Callback (void const *arg) {
}


// Example: Create and Start timers
void Init_Timers (void) {
  osStatus status;                                              // function return status
 
  // Create one-shoot timer
  exec1 = 1;
  id1 = osTimerCreate (osTimer(Timer1), osTimerPeriodic, &exec1);
  if (id1 != NULL) {    // One-shot timer created
    // start timer with delay 10ms
    status = osTimerStart (id1, 20);            
    if (status != osOK) {
      // Timer could not be started
    }
  }
 
  // Create periodic timer
  exec2 = 2;
  id2 = osTimerCreate (osTimer(Timer2), osTimerPeriodic, &exec2);
  if (id2 != NULL) {    // Periodic timer created
    // start timer with periodic 1000ms interval
    status = osTimerStart (id2, 1000);            
    if (status != osOK) {
      // Timer could not be started
    }
  }
}
