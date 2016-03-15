#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
/* #define COLOR_PORT_ID NXT_PORT_S1 */
#define PRIO_IDLE 10
#define PRIO_BUTTON 20
#define PRIO_DIST 15
#define KP 20	//proportional part of PID 
#define OFFSET 20 //offset to keep the controller 
#define TP 50     //base speed

//Shared struct resource amongst the tasks
struct dc_t 
{
  U32 duration;
  S32 speed;
  int priority;
  S32 distance;
} dc = {0, 0, PRIO_IDLE,0};

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
  ecrobot_init_sonar_sensor(NXT_PORT_S2);
  nxt_motor_set_speed(NXT_PORT_B, 0, 1);
  nxt_motor_set_speed(NXT_PORT_C, 0, 1);    
}
void ecrobot_device_terminate(void) 
{
  ecrobot_term_nxtcolorsensor(NXT_PORT_S3);
  ecrobot_term_sonar_sensor(NXT_PORT_S2);
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
	//Lock the resource
	GetResource(dc_manage);
	
	dc.priority = priority;	//change the priority
	dc.speed = speed;		//change the speed of the motors
	dc.duration = duration;	//change the duration of the motors at the given speed
	
	//Free the resource
	ReleaseResource(dc_manage);
	
	//Set the motor speed
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
		change_driving_command(PRIO_IDLE, 0, dc.duration);	//Set the robot to idle
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
	//Display the different parameters
	display_goto_xy(1,1);
	display_string("Periodic Tasks");
	display_goto_xy(1,2);
	display_string("Speed: ");
	display_goto_xy(1,3);
	display_string("Duration: ");
	display_goto_xy(1,4);
	display_string("Distance: ");
	display_goto_xy(14,4);
	display_string("cm");
	
	//Get the shared resource
	GetResource(dc_manage);
	
	//Display the updated speed, duration, and distance
	display_goto_xy(8,2);
	display_int(dc.speed, 4);
	display_goto_xy(10,3);
	display_unsigned(dc.duration, 4);
	display_goto_xy(11,4);
	display_int(dc.distance, 3);	//Display the distance	
	display_update();
	
	//Release the shared resource
	ReleaseResource(dc_manage);
	
	TerminateTask();
 }
/* DistanceTask executed every 100ms */
 TASK(DistanceTask)
 {
	int error;
	int tempSpeed;
	
	//Get the shared resource
	GetResource(dc_manage);
	
	//P algorithm
	dc.distance = ecrobot_get_sonar_sensor(NXT_PORT_S2);
	error = dc.distance - OFFSET;
	tempSpeed = KP * error;
	dc.speed = tempSpeed + TP;
	dc.duration = 100;
	
	//Free the shared resource
	ReleaseResource(dc_manage);
	
	TerminateTask();
 }
 
 