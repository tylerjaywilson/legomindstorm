#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

//Define the ports being used
#define LEFT_SENSOR NXT_PORT_S2
#define RIGHT_SENSOR NXT_PORT_S3
#define FRONT_SENSOR NXT_PORT_S1
#define LIGHT_SENSOR NXT_PORT_S4
#define RIGHT_MOTOR NXT_PORT_C
#define LEFT_MOTOR NXT_PORT_B

//Define the directions being used
#define GORIGHT 0
#define GOLEFT 1
#define GOSTRAIGHT 2
#define STOP 3

//Define the maze parameters
#define WALL_DISTANCE_LMIN 8
#define WALL_DISTANCE_LMAX 22
#define NO_LEFT_WALL 33
#define WALL_DISTANCE_RMIN 8
#define WALL_DISTANCE_FMIN 16
#define SAFE_ZONE 400

//Priority #defines
#define PRIO1 1
#define PRIO2 2
#define PRIO3 3
#define PRIO4 4
#define PRIO5 5
#define PRIO_IDLE 20

//Shared struct that holds all of the sensor data
struct sensor_data_t
{
	S32 left_speed;
	S32 right_speed;
	S32 left_distance;
	S32 right_distance;
	S32 front_distance;
	S32 light_value;	
	int priority;
} sensor_data = {0, 0, 0, 0, 0, 0, PRIO_IDLE};

/* Declare the tasks */
DeclareTask(LeftSensorTask);
DeclareTask(RightSensorTask);
DeclareTask(FrontSensorTask);
DeclareTask(LightSensorTask);
DeclareTask(MotorControlTask);
DeclareTask(DisplayTask);

//Declare the shared resource
DeclareResource(sensor_data_manage);

//Declare the counter
DeclareCounter(SysTimerCnt);

/* nxtOSEK hooks */
void ecrobot_device_initialize(void) 
{
  //Initialize light sensor
  ecrobot_init_nxtcolorsensor(LIGHT_SENSOR, NXT_LIGHTSENSOR_RED);
  //Initialize the sonar sensors
  ecrobot_init_sonar_sensor(FRONT_SENSOR);
  ecrobot_init_sonar_sensor(LEFT_SENSOR);
  ecrobot_init_sonar_sensor(RIGHT_SENSOR);
  //Set the motor speed to zero
  nxt_motor_set_speed(LEFT_MOTOR, 0, 1);
  nxt_motor_set_speed(RIGHT_MOTOR, 0, 1);  
}
void ecrobot_device_terminate(void) 
{
  //Terminate all sensor activity
  ecrobot_term_nxtcolorsensor(LIGHT_SENSOR);
  ecrobot_term_sonar_sensor(FRONT_SENSOR);
  ecrobot_term_sonar_sensor(LEFT_SENSOR);
  ecrobot_term_sonar_sensor(RIGHT_SENSOR);
  nxt_motor_set_speed(LEFT_MOTOR, 0, 1);
  nxt_motor_set_speed(RIGHT_MOTOR, 0, 1);  
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
  sensor_data.left_speed = 60;
  sensor_data.right_speed = -60;
  nxt_motor_set_speed(LEFT_MOTOR, 60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(RIGHT_MOTOR, -60, 1);  //Port C controls the right wheel
}
//Spin counter clockwise
void counterclockwise(void)
{
  sensor_data.left_speed = -60;
  sensor_data.right_speed = 60;
  nxt_motor_set_speed(LEFT_MOTOR, -60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(RIGHT_MOTOR, 60, 1);  //Port C controls the right wheel
}
//Go straight
void goStraight(void)
{
  sensor_data.left_speed = 100;
  sensor_data.right_speed = 100;
  nxt_motor_set_speed(LEFT_MOTOR, 100, 1); //Port B controls the left wheel
  nxt_motor_set_speed(RIGHT_MOTOR, 100, 1);  //Port C controls the right wheel
}
//Stop
void stop(void)
{
  sensor_data.left_speed = 0;
  sensor_data.right_speed = 0;
  nxt_motor_set_speed(LEFT_MOTOR, 0, 1);
  nxt_motor_set_speed(RIGHT_MOTOR, 0, 1);
}
/* Used to change the direction/speed of the motors */ 
void change_driving_command(int priority, int direction)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	sensor_data.priority = priority;	//change the priority
	if(direction == GOSTRAIGHT)
		goStraight();
	else if(direction == GORIGHT)
		clockwise();
	else if(direction == GOLEFT)
		counterclockwise();
	else 
		stop();
	
	//Free the resource
	ReleaseResource(sensor_data_manage);	
}
/*******************************************************************/

/* MotorControlTask executed every 50 ms */
TASK(MotorControlTask)
{
	//Lock the Resource
	GetResource(sensor_data_manage);
	
	//Handle all 16 cases
	if((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOSTRAIGHT);
	else if((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance < WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GORIGHT);
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOLEFT); 
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance < WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOLEFT);
	else if ((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GORIGHT);
	else if ((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance < WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GORIGHT);
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOSTRAIGHT);	//THIS ONE WON'T HAPPEN
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance < WALL_DISTANCE_FMIN) && (sensor_data.right_distance > WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOSTRAIGHT);	//THIS ONE WON'T HAPPEN
	else if ((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOLEFT);
	else if ((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOSTRAIGHT);
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOLEFT);
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance > WALL_DISTANCE_LMIN) && (sensor_data.front_distance < WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GORIGHT);
	else if ((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOSTRAIGHT);
	else if ((sensor_data.left_distance < WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance < WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GORIGHT);
	else if ((sensor_data.left_distance > WALL_DISTANCE_LMAX) && (sensor_data.left_distance < WALL_DISTANCE_LMIN) && (sensor_data.front_distance > WALL_DISTANCE_FMIN) && (sensor_data.right_distance < WALL_DISTANCE_RMIN))
		change_driving_command(PRIO1, GOSTRAIGHT);
	else 
		change_driving_command(PRIO1, GOLEFT);	//THIS ONE WON'T HAPPEN
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	TerminateTask();
}

/* FrontSensorTask executed every 50 ms */
TASK(FrontSensorTask)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the distance of the front sensor
	sensor_data.front_distance = ecrobot_get_sonar_sensor(FRONT_SENSOR);
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	TerminateTask();
}

/* LeftSensorTask executed every 50 ms */
TASK(LeftSensorTask)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the distance of the front sensor
	sensor_data.left_distance = ecrobot_get_sonar_sensor(LEFT_SENSOR);
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	TerminateTask();
}

/* RightSensorTask executed every 50 ms */
TASK(RightSensorTask)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the distance of the front sensor
	sensor_data.right_distance = ecrobot_get_sonar_sensor(RIGHT_SENSOR);
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	TerminateTask();
}

/* LightSensorTask executed every 50 ms */
TASK(LightSensorTask)
{	
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the distance of the front sensor
	sensor_data.light_value = ecrobot_get_nxtcolorsensor_light(LIGHT_SENSOR);
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	TerminateTask();
}

/* DisplayTask executed every 100 ms */
TASK(DisplayTask)
{
	//Display the different parameters
	display_goto_xy(0,0);
	display_string("Maze Solver");
	display_goto_xy(0,2);
	display_string("Front Dist: ");
	display_goto_xy(0,3);
	display_string("Left Dist: ");
	display_goto_xy(0,4);
	display_string("Right Dist: ");
	display_goto_xy(0,5);
	display_string("Light Val: ");
	
	//Get the shared resource
	GetResource(sensor_data_manage);
	
	//Display the updated distances and light value
	display_goto_xy(12,2);
	display_int(sensor_data.front_distance, 4);
	display_goto_xy(11,3);
	display_unsigned(sensor_data.left_distance, 4);
	display_goto_xy(12,4);
	display_int(sensor_data.right_distance, 4);	
	display_goto_xy(11,5);
	display_int(sensor_data.light_value, 4);	
	display_update();
	
	//Release the shared resource
	ReleaseResource(sensor_data_manage);
	
	TerminateTask();
}
