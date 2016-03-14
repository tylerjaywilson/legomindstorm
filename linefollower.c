#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define WHITELINE 470
#define BLACKLINE 300

boolean finished_init = false;

/* You can define the ports used here */
/* #define COLOR_PORT_ID NXT_PORT_S1 */

/* Declare the tasks */
DeclareTask(MotorControlTask);
//DeclareTask(RotationTask);
DeclareCounter(SysTimerCnt);

/* Declare the Events */

/* nxtOSEK hooks */
void ecrobot_device_initialize(void) 
{
  ecrobot_init_nxtcolorsensor(NXT_PORT_S3, NXT_LIGHTSENSOR_RED);
  nxt_motor_set_speed(NXT_PORT_B, 0, 1);
  nxt_motor_set_speed(NXT_PORT_C, 0, 1);  
}
void ecrobot_device_terminate(void) 
{
  ecrobot_term_nxtcolorsensor(NXT_PORT_S3);
  nxt_motor_set_speed(NXT_PORT_B, 0, 1);
  nxt_motor_set_speed(NXT_PORT_C, 0, 1);  
}
void user_1ms_isr_type2()
{
  ecrobot_process_bg_nxtcolorsensor();
  StatusType ercd;

  ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
  if (ercd != E_OK)
  {
   ShutdownOS(ercd);
  }
}

//Start the program by finding the edge of the line
void init_findEdge(void)
{
   /* Your code */
  display_clear(1);
  display_goto_xy(0,0);
  display_string(" Line");
  display_goto_xy(0,1);
  display_string(" Follower!");
  display_goto_xy(0,2);
  display_string(" Light value: ");
  display_goto_xy(1,3);
  
  int lightData = 0;
  //Clear the buffer of out light values and wait for the system to boot
  lightData = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
  //systick_wait_ms(500);  
  
  //Start rotating clockwise to find the edge
  nxt_motor_set_speed(NXT_PORT_B, 30, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, 0, 1);  //Port C controls the right wheel
		
  //Used for determining an edge
  boolean blackFound = false;  
  boolean whiteFound = false;
  
  while(!blackFound || !whiteFound)
  {
	lightData = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
	//We have found a black line
	if(lightData < BLACKLINE)
	{
		blackFound = true;
	}
	else if(lightData > WHITELINE) //We have found a white line
	{
		whiteFound = true;
	}
  }
  nxt_motor_set_speed(NXT_PORT_B, 0, 1);
  nxt_motor_set_speed(NXT_PORT_C, 0, 1);
  finished_init = true;
}

//Go forward and right function
void setForwardRight(void)
{
  nxt_motor_set_speed(NXT_PORT_B, 60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, -20, 1);  //Port C controls the right wheel
}
//Go forward and left function
void setForwardLeft(void)
{
  nxt_motor_set_speed(NXT_PORT_B, -20, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, 60, 1);  //Port C controls the right wheel
}
//Spin clockwise
void clockwise(void)
{
  nxt_motor_set_speed(NXT_PORT_B, 30, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, -40, 1);  //Port C controls the right wheel
}
//Spin counter clockwise
void counterclockwise(void)
{
  nxt_motor_set_speed(NXT_PORT_B, -40, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, 30, 1);  //Port C controls the right wheel
}

TASK(MotorControlTask)
{
  //Call the initialize function to find the starting edge
  init_findEdge();
  
  //Boolean used to determine what happened previously
  boolean turnedRight = true;
  
  int lightData = 0; 
  int display_count = 0;
      
  while(1)
  {    	
	//Determine the next change to the motors
	lightData = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
	
	if(lightData > WHITELINE && turnedRight)
	{
		setForwardLeft();
	}
	else if(lightData > WHITELINE && !turnedRight)
	{
		setForwardRight();
	}
	else if(lightData < BLACKLINE && turnedRight)
	{
		setForwardRight();
	}
	else
	{
		setForwardLeft();
	}
	
	//Update the display periodically
	if(display_count == 50)
	{	 
	  display_goto_xy(1,3);	
	  display_int(lightData, 8);	
	  display_update();
	  display_count = 0;
	}
	display_count++;
  }
  
  TerminateTask();
}
