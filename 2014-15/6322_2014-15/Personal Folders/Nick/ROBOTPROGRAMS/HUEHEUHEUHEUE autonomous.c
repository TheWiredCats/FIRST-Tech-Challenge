#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Motor,  mtr_S1_C1_1,     BackRight,     tmotorNXT, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     FrontRight,    tmotorNXT, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     armlift,       tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     PullUp,        tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     BackLeft,      tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     FrontLeft,     tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     Flag,          tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     none,          tmotorNXT, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    servoleft,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servoright,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    pulluplock,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)

#include "JoystickDriver.c"

#define FORWARD_DIRECTION    1
#define BACKWARDS_DIRECTION -1
#define STOP                 0
#define LEFT_CLOCKWISE       2

void runAllMotors(int speed);
void stopAllMotors(void);
void turnLeft(int delta);
void turnRight(int delta);

void initializeRobot()
{
	return;
}
task main()
{
	initializeRobot();
	waitForStart();

	runAllMotors(50);
	wait1Msec(1800);
	stopAllMotors();
}


void runAllMotors(int speed)
{
	if (speed > 127)
		speed = 127;
	else if( speed < -127)
		speed = -127;

		motor[FrontRight] = speed;
		motor[FrontLeft]  = speed;
		motor[BackRight]  = speed;
		motor[BackLeft]   = speed;
}


void stopAllMotors()
{
	runAllMotors(0);
}

void turnLeft( int delta )
{
	motor[FrontRight] += delta;
	motor[FrontLeft]  -= delta;
	motor[BackRight]  += delta;
	motor[BackLeft]   -= delta;

}

void turnRight(int delta)
{
	motor[FrontRight] -= delta;
	motor[FrontLeft]  += delta;
	motor[BackRight]  -= delta;
	motor[BackLeft]   += delta;
}
