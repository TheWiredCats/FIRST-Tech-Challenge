#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     backleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     frontleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     backright,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     frontright,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     linear,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     flag,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    lock,             tServoNone)
#pragma config(Servo,  srvo_S1_C4_2,    cube,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "joystickdriver.c";
void runAllMotors (int speed);
void stopAllMotors();
void turnRight (int speed);
void turnLeft (int speed);
void initializeRobot()
{
	return;
}
task main()
{
getJoystickSettings(joystick);
int p = 100;
motor(frontleft) = p;
motor(backleft) = p;
motor(frontright) = p;
motor(backright) = p;
wait1Msec(1500);
runAllMotors( 50 );
wait1Msec(3000);
stopAllMotors();
wait1Msec(1000);

}

//Functions

void runAllMotors ( int speed )
{
if (speed > 127)
	speed = 127;
else if (speed < -127)
	speed = -127;

	motor[frontleft] = speed;
	motor[frontright] = speed;
	motor[backleft] = speed;
	motor[backright] = speed;
}

void stopAllMotors ()
{
	runAllMotors(0);
}

void turnRight( int speed )
{
motor[frontright] = -speed;
motor[backright] = -speed;
motor[frontleft] = speed;
motor[backleft] = speed;
}

void turnLeft( int speed )
{
motor[frontright] = speed;
motor[backright] = speed;
motor[frontleft] = - speed;
motor[backleft] = - speed;
}