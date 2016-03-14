#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define WHITELINE 470
#define BLACKLINE 300

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

/********** Functions that control/set the motor speeds ************/
//Spin clockwise
void clockwise(void)
{
  nxt_motor_set_speed(NXT_PORT_B, 60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, -60, 1);  //Port C controls the right wheel
}
//Spin counter clockwise
void counterclockwise(void)
{
  nxt_motor_set_speed(NXT_PORT_B, -60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, 60, 1);  //Port C controls the right wheel
}
//Go straight
void goStraight(void)
{
  nxt_motor_set_speed(NXT_PORT_B, 100, 1); //Port B controls the left wheel
  nxt_motor_set_speed(NXT_PORT_C, 100, 1);  //Port C controls the right wheel
}
/*******************************************************************/

TASK(MotorControlTask)
{
  /* Display the project name and Light value text to the screen */
  display_clear(1);
  display_goto_xy(0,0);
  display_string(" Line");
  display_goto_xy(0,1);
  display_string(" Follower!");
  display_goto_xy(0,2);
  display_string(" Light value: ");
  display_goto_xy(1,3);
 
  //lightData stores the read value from the light sensor
  int lightData = 0; 
      
  while(1)
  {    	
	//Determine the next change to the motors
	lightData = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
	
	if(lightData > WHITELINE)	//We are out of the medium light range - rotate the bot
	{
		counterclockwise();
	}
	else if(lightData < BLACKLINE)	//We are out of the medium light range - rotate the bot
	{
		clockwise();
	}
	else
	{
		goStraight();
	}
	
	
  }
  
  TerminateTask();
}

TASK(DisplayTask)
{
  int lightValue = 0;
  lightValue = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
  display_goto_xy(1,3);	
  display_int(lightValue, 8);	
  display_update();
  
  TerminateTask();
}