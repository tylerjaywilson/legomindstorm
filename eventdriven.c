#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
/* #define COLOR_PORT_ID NXT_PORT_S1 */

/* Declare the tasks */
DeclareTask(MotorControlTask);
DeclareTask(EventDispatcher);
DeclareCounter(SysTimerCnt);

/* Declare the Events */
DeclareEvent(TouchSensorOnEvent);  /* Event declaration */
DeclareEvent(TouchSensorOffEvent); /* Event declaration */ 

/* nxtOSEK hooks */
void ecrobot_device_initialize(void) 
{
  ecrobot_init_nxtcolorsensor(NXT_PORT_S3, NXT_LIGHTSENSOR_WHITE);
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

TASK(MotorControlTask)
{
  display_clear(1);
  display_goto_xy(1,1);
  display_string("Event Driven");
  
  /* Your code */
  while(1)
  {
	WaitEvent(TouchSensorOnEvent);	// Task is in waiting status until the Event comes
	ClearEvent(TouchSensorOnEvent);
	nxt_motor_set_speed(NXT_PORT_B, 50, 1);
	nxt_motor_set_speed(NXT_PORT_C, 50, 1);
	
	WaitEvent(TouchSensorOffEvent);	// Task is in waiting status until the Event comes
	ClearEvent(TouchSensorOffEvent);
	nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	nxt_motor_set_speed(NXT_PORT_C, 0, 1);	
  }
  TerminateTask();
}

/* EventDispatcher executed every 1ms */
 TASK(EventDispatcher)
 {
   static U8 TouchSensorStatus_old = 0;
   int LightSensorStatus;
   U8 TouchSensorStatus; 

   TouchSensorStatus = ecrobot_get_touch_sensor(NXT_PORT_S1);
   LightSensorStatus = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
   
   if(LightSensorStatus > 300)
   {     
     display_goto_xy(1,3);
	 display_int(LightSensorStatus, 4);
	 nxt_motor_set_speed(NXT_PORT_B, 0, 1);
	 nxt_motor_set_speed(NXT_PORT_C, 0, 1);	
	 display_update();
	 ClearEvent(TouchSensorOnEvent);
	 //ClearEvent(TouchSensorOffEvent);
   }   
   else if (TouchSensorStatus == 1 && TouchSensorStatus_old == 0)
   {
     display_goto_xy(1,3);
	 display_int(LightSensorStatus, 4);
	 display_update();
    /* Send a Touch Sensor ON Event to the Handler */ 
     SetEvent(MotorControlTask, TouchSensorOnEvent);
   }
   else if (TouchSensorStatus == 0 && TouchSensorStatus_old == 1)
   {
     display_goto_xy(1,3);
	 display_int(LightSensorStatus, 4);
	 display_update();
    /* Send a Touch Sensor OFF Event to the Handler */ 
     SetEvent(MotorControlTask, TouchSensorOffEvent);
   }

   TouchSensorStatus_old = TouchSensorStatus;

   TerminateTask();
 }

 