#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motorD,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motorF,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    a,                    tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C3_2,    b,                    tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#define FORWARD_DIRECTION      1
#define BACKWARDS_DIRECTION   -1
#define STOP                   0

/* Function declaration */
float a1;
float b1;
void runConveyorBelt( byte direction );

void initializeRobot()
{
	return;
}
task main()
{
	a1 = 0;
	b1 = 0;
	initializeRobot();
	waitForStart();

	/* Stage 1. Automation procedure to
	 * drop the block */

  runConveyorBelt( FORWARD_DIRECTION );
  wait1Msec(1800);
  runConveyorBelt( BACKWARDS_DIRECTION );
  wait1Msec(1000);
  runConveyorBelt( STOP );
}

 /* End of Autonomous */

 void runConveyorBelt( byte direction )
{
	if(direction == FORWARD_DIRECTION)
	{
		servo[a] = (int) 200; // pick any value from 128 to 255
    servo[b] = (int) 200;
}
    else if( direction == BACKWARDS_DIRECTION)
{		servo[a] = (int) 100; // pick any value from 0 to 127
    servo[b] = (int) 100;
}
    else if( direction == STOP)
{   servo[a] = (int) 127;
    servo[b] = (int) 127;
}
    else
{ 	servo[a] = 127; //127 stops servo
    servo[b] = 127;
}
    }