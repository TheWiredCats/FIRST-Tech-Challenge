#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     Conveyor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     Left,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    Stopper,              tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)


#include "joystickdriver.c"
void runAllMotors(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void initializeRobot()
{
	return;
}
task main()
{
initializeRobot();
waitForStart();

runAllMotors(50);
wait1Msec(3000);
runAllMotors(0);

/*
motor[Lift] = 100;
wait1Msec(3000);
motor[Lift] = 0;
wait1Msec(600);
servo[Stopper] = 127;
*/


}


void runAllMotors(int speed)
{
		motor[Right] = speed;
	  motor[Left] = speed;
}

void turnRight(int speed)
{
	motor[Right] = -speed;
	motor[Left] = speed;
}

void turnLeft(int speed)
{
	motor[Right] = speed;
	motor[Left] = -speed;
}
