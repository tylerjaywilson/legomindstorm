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
#define TURNRIGHT 75
#define TURNFORWARD 360

//Motor speeds
#define NOMINAL_SPEED 65

//PID defines
#define KP 9
#define KI 0
#define KD 0
#define DT 0.055

//Define the maze parameters
#define WALL_DISTANCE_LMIN 10
#define WALL_DISTANCE_LMAX 14
#define WALL_DISTANCE_RMIN 3
#define WALL_DISTANCE_FMIN 17
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
  
  GetResource(sensor_data_manage);
  //Get the initial sensor readings
  //Get the distance of the front sensor
  sensor_data.front_distance = ecrobot_get_sonar_sensor(FRONT_SENSOR);
  //Get the distance of the left sensor
  sensor_data.left_distance = ecrobot_get_sonar_sensor(LEFT_SENSOR);
  //Get the distance of the right sensor
  sensor_data.right_distance = ecrobot_get_sonar_sensor(RIGHT_SENSOR);
  //Get the light value
  sensor_data.light_value = ecrobot_get_nxtcolorsensor_light(LIGHT_SENSOR);
  
  ReleaseResource(sensor_data_manage);
  
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
	//1 ms interrupt to maintain color sensor communication
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
	//Tell the motors to turn clockwise
  sensor_data.left_speed = 60;
  sensor_data.right_speed = -60;
  nxt_motor_set_speed(LEFT_MOTOR, 60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(RIGHT_MOTOR, -60, 1);  //Port C controls the right wheel
}
//Spin counter clockwise
void counterclockwise(void)
{
	//Tell the motors to make a counter-clockwise turn
  sensor_data.left_speed = -60;
  sensor_data.right_speed = 60;
  nxt_motor_set_speed(LEFT_MOTOR, -60, 1); //Port B controls the left wheel
  nxt_motor_set_speed(RIGHT_MOTOR, 60, 1);  //Port C controls the right wheel
}
//Go straight
void goStraight(void)
{
	//Tell the motors to go full-speed and straight
  sensor_data.left_speed = 100;
  sensor_data.right_speed = 100;
  nxt_motor_set_speed(LEFT_MOTOR, 100, 1); //Port B controls the left wheel
  nxt_motor_set_speed(RIGHT_MOTOR, 100, 1);  //Port C controls the right wheel
}
//Stop
void stop(void)
{
	//Tell the motors to stop
  sensor_data.left_speed = 0;
  sensor_data.right_speed = 0;
  nxt_motor_set_speed(LEFT_MOTOR, 0, 1);
  nxt_motor_set_speed(RIGHT_MOTOR, 0, 1);
}
//Change speed - PID controller
void change_motor_speed(int leftSpeed, int rightSpeed)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Update the shared resource speeds
	sensor_data.left_speed = leftSpeed;
	sensor_data.right_speed = rightSpeed;
	
	//Set the motor speed
	nxt_motor_set_speed(LEFT_MOTOR, leftSpeed, 1);
	nxt_motor_set_speed(RIGHT_MOTOR, rightSpeed, 1);
	
	//Release the resource
	ReleaseResource(sensor_data_manage);
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
	//PID static variables
	static int integral = 0; 
	static int derivative = 0;
	static int prev_error = 0;
	
	//Lock the Resource
	GetResource(sensor_data_manage);

	//If the front sensor reading is less than the desired front distance 
	//then turn to the right
	if(sensor_data.front_distance <= WALL_DISTANCE_FMIN)
	{
		//Variables for keeping track of the amount of turning
		int startLeftCount = nxt_motor_get_count(LEFT_MOTOR);
		int startRightCount = nxt_motor_get_count(RIGHT_MOTOR);
		int curLeftCount = nxt_motor_get_count(LEFT_MOTOR) - startLeftCount;
		int curRightCount = nxt_motor_get_count(RIGHT_MOTOR) - startRightCount;
		
		//Start turning to the right
		change_driving_command(PRIO1, GORIGHT);
		
		//Measure the wheel revolutions until we reach the desired turn amount
		while((curLeftCount < TURNRIGHT) && (curRightCount < TURNRIGHT)) 
		{
			curLeftCount = nxt_motor_get_count(LEFT_MOTOR) - startLeftCount;
			curRightCount = nxt_motor_get_count(RIGHT_MOTOR) - startRightCount;
		}
	}
	
	//Determine if the left wall distance is so large that we need to make a more 
	//sharp left hand turn
	if(sensor_data.left_distance >= WALL_DISTANCE_LMAX)
	{
		//Change the motor speed based for a left gradual turn - This occurs when a left turn is needed
		change_motor_speed(30, 70);		
	}
	else if(sensor_data.right_distance <= WALL_DISTANCE_RMIN)
	{
		//Turn back to the left a little
		change_driving_command(PRIO1, GOLEFT);
	}
	else
	{
		//PID controller is here
		//Maintain 11-15 cm from the wall
		//Determine the current error
		int error = sensor_data.left_distance - WALL_DISTANCE_LMIN;
		
		//Calculate the integral portion
		integral = (2/3)*integral + error * DT;
		
		//Calculate the derivative portion
		derivative = (error - prev_error)/DT;
		
		//Determine the turn amount
		int turn = (error * KP) + (integral * KI) + (derivative * KD);
		
		//Update the left and right motor speed	
		int leftSpeed = NOMINAL_SPEED - turn;
		int rightSpeed = NOMINAL_SPEED + turn;
		
		//Store the new error as the previous error
		prev_error = error;
		
		//Change the motor speed based on the PID algorithm
		change_motor_speed(leftSpeed, rightSpeed);
	}
	
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
	
	//Terminate the task
	TerminateTask();
}

/* LeftSensorTask executed every 50 ms */
TASK(LeftSensorTask)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the distance of the left sensor
	sensor_data.left_distance = ecrobot_get_sonar_sensor(LEFT_SENSOR);
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	//Terminate the task
	TerminateTask();
}

/* RightSensorTask executed every 50 ms */
TASK(RightSensorTask)
{
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the distance of the right sensor
	sensor_data.right_distance = ecrobot_get_sonar_sensor(RIGHT_SENSOR);
	
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	//Terminate the Task
	TerminateTask();
}

/* LightSensorTask executed every 50 ms */
TASK(LightSensorTask)
{	
	//Lock the resource
	GetResource(sensor_data_manage);
	
	//Get the value of the light sensor
	sensor_data.light_value = ecrobot_get_nxtcolorsensor_light(LIGHT_SENSOR);
	
	//Variable used to determine if it if the first time going through
	//the upcoming while loop
	int firstTimeThrough = 0;
	
	//Determine if we have found the safe zone area
	while(sensor_data.light_value < SAFE_ZONE)
	{
		if(firstTimeThrough == 0)
		{
			//Made it to the safe zone
			int startLeftCount = nxt_motor_get_count(LEFT_MOTOR);
			int startRightCount = nxt_motor_get_count(RIGHT_MOTOR);
			int curLeftCount = nxt_motor_get_count(LEFT_MOTOR) - startLeftCount;
			int curRightCount = nxt_motor_get_count(RIGHT_MOTOR) - startRightCount;
		
			//Start turning to the right
			change_driving_command(PRIO1, GOSTRAIGHT);
		
			//measure the wheel revolutions until we reach the desired turn amount
			while((curLeftCount < TURNFORWARD) && (curRightCount < TURNFORWARD)) 
			{
				curLeftCount = nxt_motor_get_count(LEFT_MOTOR) - startLeftCount;
				curRightCount = nxt_motor_get_count(RIGHT_MOTOR) - startRightCount;
			}
						
			//Stop the vehicle in the safe zone
			change_driving_command(PRIO1, STOP);
			
			//Indicate that we have already placed the robot in the middle of the safe zone
			firstTimeThrough = 1;
		}
		//Continue to read the light value to see if we are no longer in a safe zone
		sensor_data.light_value = ecrobot_get_nxtcolorsensor_light(LIGHT_SENSOR);
	}
	//Free the Resource
	ReleaseResource(sensor_data_manage);
	
	//Terminate the task
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
	
	//Terminate the task
	TerminateTask();
}
