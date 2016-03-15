#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
/* #define COLOR_PORT_ID NXT_PORT_S1 */
#define PRIO_IDLE 10
#define PRIO_BUTTON 20
#define PRIO_DIST 15

struct dc_t 
{
  U32 duration;
  S32 speed;
  int priority;
} dc = {0, 0, PRIO_IDLE};

/* Declare the tasks */
DeclareTask(MotorControlTask);
DeclareTask(ButtonPressTask);
DeclareTask(DisplayTask);
DeclareTask(DistanceTask);

/* Declare the resources */
DeclareResource(dc_manage);

/* Declare the counter */
DeclareCounter(SysTimerCnt);

/* nxtOSEK hooks */
void ecrobot_device_initialize(void) 
{
  ecrobot_init_nxtcolorsensor(NXT_PORT_S3, NXT_LIGHTSENSOR_WHITE);
  nxt_motor_set_speed(NXT_PORT_B, 0, 1);
  nxt_motor_set_speed(NXT_PORT_C, 0, 1);  
  display_goto_xy(1,1);
  display_string("Periodic Tasks");
  display_goto_xy(1,2);
  display_string("Speed: ");
  display_goto_xy(1,3);
  display_string("Duration: ");
  display_goto_xy(1,3);
  display_string("Distance: ");
  display_goto_xy(14,3);
  display_string("cm");
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
/* used to change the dc for the dc_t struct */ 
void change_driving_command(int priority, int speed, int duration)
{
	GetResource(dc_manage);
	
	dc.priority = priority;
	dc.speed = speed;
	dc.duration = duration;
	
	ReleaseResource(dc_manage);
	nxt_motor_set_speed(NXT_PORT_B, dc.speed, 1);
	nxt_motor_set_speed(NXT_PORT_C, dc.speed, 1);
}

/* MotorControlTask executed every 50 ms */
TASK(MotorControlTask)
{
	if(dc.duration > 0)
	{
		change_driving_command(dc.priority, dc.speed, dc.duration-50);	
	}
	else if(dc.duration <= 0)
	{
		change_driving_command(PRIO_IDLE,0, dc.duration);
	}
	
	TerminateTask();
}

/* ButtonPressTask executed every 10ms */
 TASK(ButtonPressTask)
 {  
	if(ecrobot_get_touch_sensor(NXT_PORT_S1))
	{
		change_driving_command(PRIO_BUTTON,-50, 1000);	//Drive the bot backwards for 1 second
    }
	TerminateTask();
 }
 
 /* DisplayTask executed every 100ms */
 TASK(DisplayTask)
 {	
	//Get the shared resource
	GetResource(dc_manage);
	
	//Display the updated speed, duration, and distance
	display_goto_xy(8,2);
	display_int(dc.speed, 4);
	display_goto_xy(10,3);
	display_unsigned(dc.duration, 4);
	display_goto_xy(11,3);
	display_int(15, 4);	//Display the distance
	
	//Release the shared resource
	ReleaseResource(dc_manage);
	
	TerminateTask();
 }

 TASK(DistanceTask)
 {
	TerminateTask();
 }
 