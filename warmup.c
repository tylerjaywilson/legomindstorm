#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
/* #define COLOR_PORT_ID NXT_PORT_S1 */

DeclareTask(TestColorSensor);

/* nxtOSEK hooks */
void ecrobot_device_initialize(void) 
{
  ecrobot_init_nxtcolorsensor(NXT_PORT_S3, NXT_LIGHTSENSOR_WHITE);  
}
void ecrobot_device_terminate(void) 
{
  ecrobot_term_nxtcolorsensor(NXT_PORT_S3);
}
void user_1ms_isr_type2()
{
  ecrobot_process_bg_nxtcolorsensor();
}

TASK(TestColorSensor)
{
  /* Your code */
  display_clear(1);
  display_goto_xy(0,0);
  display_string(" Welcome to");
  display_goto_xy(0,1);
  display_string(" my world!");
  display_goto_xy(0,2);
  display_string(" Light value: ");
  display_goto_xy(1,3);
  int lightData = 0;
  int count = 0;
  
  while(1)
  {    
	display_goto_xy(1,3);
	lightData = ecrobot_get_nxtcolorsensor_light(NXT_PORT_S3);
	display_int(lightData, 8);	
	systick_wait_ms(100);
	if(count == 10)
	{	  
	  display_update();
	  count = 0;
	}
	count++;
  }
  TerminateTask();
}

